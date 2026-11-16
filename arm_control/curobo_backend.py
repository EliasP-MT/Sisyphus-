"""GPU collision checking and planning for the ElRobot arm with cuRobo (NVIDIA, Apache-2.0).

Mirrors the scene of collision.py (static surface meshes in base_link, arm with self-collision, gripper
frozen at the modelled opening) on the GPU so thousands of configurations are checked in one batch:
  * spheres fitted on the URDF meshes (arm_control/curobo/elrobot.yml, built by curobo/build_model.py)
    against the exact scene triangles, with the scene margin_m as activation distance;
  * self-collision with self_margin_m;
  * collision.py's conservative containment rule: a link centre inside a surface mesh's bounds;
  * the base_link group (base_link and the links fixed to it) is excluded from scene checks, as in
    collision.py (the arm stands on the rover).
cuRobo is an accelerator, not the authority: callers keep the exact PyBullet check (collision.py) on
the start, the goal and a sparse set of trajectory samples."""
import json
from pathlib import Path

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROBOT_YML = HERE / 'curobo' / 'elrobot.yml'
BASE_GROUP = ('base_link', 'ST3215_1_v1_1')  # collision.py: groups[-1] (base and links fixed to it)
# J1 group (yaw stage sitting on the rover top): its spheres protrude a few mm into the margin above the
# chassis in every pose. Its position depends on q1 only; it is left to the exact PyBullet check.
SCENE_EXEMPT = BASE_GROUP + ('Joint_01_1', 'ST3215_2_v1_1')
# Self pairs whose fitted spheres overlap in many exactly-free poses (measured: Joint_04/Joint_06 overlap
# up to 24 mm around the short Joint_05). Left to the exact PyBullet check.
# The list is calibrated against the exact checker by curobo/calibrate_self.py for the current sphere fit.
SELF_EXEMPT_JSON = HERE / 'curobo' / 'self_exempt.json'


def self_exempt():
    if not SELF_EXEMPT_JSON.exists():
        return set()
    data = json.loads(SELF_EXEMPT_JSON.read_text())
    if data.get('robot_yml_sha256') != __import__('hashlib').sha256(ROBOT_YML.read_bytes()).hexdigest():
        raise ValueError('self_exempt.json was calibrated for another sphere model; rerun calibrate_self.py')
    return {frozenset(p) for p in data['pairs']}


def _pose7(xyz, rpy):
    qx, qy, qz, qw = Rotation.from_euler('xyz', rpy).as_quat()
    return [float(v) for v in xyz] + [float(qw), float(qx), float(qy), float(qz)]


class CuroboWorld:
    def __init__(self, scene_path, bounds, device='cuda:0', scene_buffer_m=0.006):
        from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
        self.scene_path = Path(scene_path).resolve()
        s = json.loads(self.scene_path.read_text())
        if s.get('cad') or s.get('payload') or s.get('boxes') or s.get('proxy_boxes') or s.get('arm_collision_proxy'):
            raise ValueError('cuRobo backend: only surface-mesh scenes are mirrored (no CAD/boxes/payload)')
        if s.get('gripper_angle_rad', 0.0) != 0.0:
            raise ValueError('cuRobo model is built with the gripper closed (gripper_angle_rad 0)')
        self.margin, self.self_margin = float(s['margin_m']), float(s['self_margin_m'])
        self.edge_step = float(s['edge_step_rad'])
        self.bounds = np.asarray(bounds, float)
        self.scene_buffer = float(scene_buffer_m)  # the sphere fit covers ~95 % of each link
        self.device = torch.device(device)
        meshes, self.boxes = {}, []
        for entry in s.get('surface_meshes', []):
            mesh = (self.scene_path.parent / entry['mesh']).resolve()
            report = json.loads(mesh.with_name('assembly.json').read_text())
            meshes[entry['name']] = {'pose': _pose7(entry['xyz_m'], entry['rpy_rad']), 'file_path': str(mesh)}
            T = np.eye(4)
            T[:3, :3] = Rotation.from_euler('xyz', entry['rpy_rad']).as_matrix()
            T[:3, 3] = entry['xyz_m']
            self.boxes.append((entry['name'], np.linalg.inv(T), np.array(report['bounds_m'], float)))
        robot = yaml.safe_load(ROBOT_YML.read_text())
        robot_scene = {'kinematics': dict(robot['kinematics'])}
        cfg = RobotCollisionCheckerCfg.load_from_config(
            robot_config={'robot_cfg': robot_scene} if 'robot_cfg' not in robot else robot,
            scene_model={'mesh': meshes},
            collision_activation_distance=self.margin,
            self_collision_activation_distance=self.self_margin,
            n_meshes=max(4, len(meshes)), n_cuboids=4)
        self.checker = RobotCollisionChecker(cfg)
        self._sphere_links()
        self.robot_yml_text = ROBOT_YML.read_text()

    def _sphere_links(self):
        """Per-sphere link index, used for the base exclusion and the link-centre containment rule."""
        robot = yaml.safe_load(ROBOT_YML.read_text())['kinematics']
        names, link_of = [], []
        for link in robot['collision_link_names']:
            spheres = robot['collision_spheres'][link]
            names.append(link)
            link_of += [len(names) - 1] * len(spheres)
        self.link_names = names
        self.link_of = torch.tensor(link_of, device=self.device)
        self.base_mask = torch.tensor([names[i] in SCENE_EXEMPT for i in link_of], device=self.device)
        self.n_links = len(names)
        # Self-collision pairs exactly as collision.py: links of different rigid groups that are not
        # joined by a joint (the frozen gripper is one group with Gripper_Base).
        import xml.etree.ElementTree as ET
        root = ET.parse(HERE / 'curobo' / 'elrobot_arm.generated.urdf').getroot()
        group, parent_of, joints = {'base_link': 'base_link'}, {}, root.findall('joint')
        pending = list(joints)
        while pending:
            rest = []
            for j in pending:
                parent, child = j.find('parent').get('link'), j.find('child').get('link')
                if parent not in group:
                    rest.append(j)
                    continue
                group[child] = group[parent] if j.get('type') == 'fixed' else child
                parent_of[child] = parent
            if len(rest) == len(pending):
                raise ValueError('URDF tree is not connected to base_link')
            pending = rest
        neighbours = {frozenset((group[c], group[p])) for c, p in parent_of.items()}
        exempt = self_exempt()
        allowed = torch.zeros(self.n_links, self.n_links, dtype=torch.bool)
        for a, na in enumerate(names):
            for b, nb in enumerate(names):
                ga, gb = group[na], group[nb]
                allowed[a, b] = (ga != gb and frozenset((ga, gb)) not in neighbours
                                 and frozenset((na, nb)) not in exempt)
        self.self_allowed_links = allowed
        link_of_cpu = torch.tensor(link_of)
        self.self_pair_mask = allowed[link_of_cpu][:, link_of_cpu].to(self.device)  # [n_sph, n_sph]

    def _kin_spheres(self, q):
        state = self.checker.get_kinematics(q)
        return state, state.robot_spheres  # [b, h, n, 4]

    @torch.no_grad()
    def valid_batch(self, Q, details=False):
        """Q: (N, 7) radians -> (N,) bool. Joint bounds, scene (margin), self (self margin), containment.
        details=True also returns per-test failure masks {'self', 'scene', 'inside'}."""
        Q = np.asarray(Q, float).reshape(-1, 7)
        ok = np.all((Q >= self.bounds[:, 0] - 1e-9) & (Q <= self.bounds[:, 1] + 1e-9), axis=1)
        out = np.zeros(len(Q), bool)
        fails = {k: np.zeros(len(Q), bool) for k in ('self', 'scene', 'inside')}
        chunk = 4096
        for start in range(0, len(Q), chunk):
            q = torch.as_tensor(Q[start:start + chunk], device=self.device, dtype=torch.float32)[:, None, :]
            b = q.shape[0]
            self.checker.setup_batch_tensors(b, 1)
            state, spheres = self._kin_spheres(q)
            valid = torch.ones(b, dtype=torch.bool, device=self.device)
            sp = spheres[:, 0]  # [b, n, 4]
            gap = torch.cdist(sp[..., :3], sp[..., :3]) - sp[..., 3, None] - sp[..., None, 3]
            f_self = ((gap < self.self_margin) & self.self_pair_mask).any(-1).any(-1)
            valid &= ~f_self
            # Scene: exclude base-group spheres by moving them far away (radius < 0 = disabled sphere).
            sph = spheres.clone()
            sph[..., 3] += self.scene_buffer
            sph[..., self.base_mask, 3] = -1.0
            state.robot_spheres = sph
            d_world = self.checker.get_collision_constraint(state)
            f_scene = d_world.reshape(b, -1).sum(-1) > 0
            valid &= ~f_scene
            f_inside = torch.zeros(b, dtype=torch.bool, device=self.device)
            # Containment rule on link centres (mean of the link's sphere centres).
            centres = torch.zeros(b, self.n_links, 3, device=self.device)
            centres.index_add_(1, self.link_of, spheres[:, 0, :, :3])
            counts = torch.bincount(self.link_of, minlength=self.n_links).clamp(min=1)[None, :, None]
            centres = centres / counts
            keep = torch.tensor([n not in SCENE_EXEMPT for n in self.link_names], device=self.device)
            for _, inv, bnd in self.boxes:
                inv_t = torch.as_tensor(inv, device=self.device, dtype=torch.float32)
                local = centres @ inv_t[:3, :3].T + inv_t[:3, 3]
                lo = torch.as_tensor(bnd[0], device=self.device, dtype=torch.float32)
                hi = torch.as_tensor(bnd[1], device=self.device, dtype=torch.float32)
                inside = torch.all((local >= lo) & (local <= hi), dim=-1) & keep
                f_inside |= inside.any(-1)
            valid &= ~f_inside
            out[start:start + b] = valid.cpu().numpy()
            for k, f in (('self', f_self), ('scene', f_scene), ('inside', f_inside)):
                fails[k][start:start + b] = f.cpu().numpy()
        return (out & ok, fails) if details else out & ok

    def densify(self, path, step=None):
        """Joint-space linear interpolation of a polyline at <= step rad per joint (endpoints included)."""
        step = step or self.edge_step
        path = [np.asarray(p, float) for p in path]
        rows = [path[0][None]]
        for a, b in zip(path, path[1:]):
            n = max(1, int(np.ceil(np.max(np.abs(b - a)) / step)))
            rows.append(np.linspace(a, b, n + 1)[1:])
        return np.concatenate(rows)

    def path_valid(self, path, step=None):
        return bool(np.all(self.valid_batch(self.densify(path, step))))
