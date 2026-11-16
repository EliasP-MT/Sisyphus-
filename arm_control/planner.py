"""ElRobot IK and local waypoint planning. No Station connection or motor writes."""
import json
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from ikpy.chain import Chain
from ruckig import InputParameter, Result, Ruckig, Synchronization, Trajectory
from scipy.spatial.transform import Rotation

JOINTS = [f'rev_motor_{i:02d}' for i in range(1, 8)]
TICKS_PER_RAD = 4096 / (2 * math.pi)


def joint_scale(joint):
    scale = joint.get('ticks_per_rad', TICKS_PER_RAD)
    if not math.isfinite(scale) or not 100 <= scale <= 5000:
        raise ValueError('Échelle articulaire invalide')
    return scale


def vector(value, size, label):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.all(np.isfinite(a)):
        raise ValueError(f'{label}: {size} valeurs finies requises')
    return a


def load_config(path):
    c = json.loads(Path(path).read_text())
    if c['frame'] != 'base_link' or c['tip_link'] != 'Gripper_Base_v1_1':
        raise ValueError('Ce contrôleur attend le repère base_link et le bout du bras ElRobot')
    vector(c['tcp_xyz_m'], 3, 'tcp_xyz_m')
    if [j['name'] for j in c['joints']] != JOINTS:
        raise ValueError('Mapping ordonné des sept articulations ElRobot requis')
    ids = [j['motor_id'] for j in c['joints']]
    if len(set(ids)) != 7 or any(type(i) is not int or not 1 <= i <= 253 for i in ids):
        raise ValueError('Sept IDs moteurs distincts entre 1 et 253 requis')
    for key in ('dt_s', 'max_velocity_rad_s', 'max_acceleration_rad_s2',
                'max_jerk_rad_s3', 'position_tolerance_m', 'orientation_tolerance_rad',
                'cartesian_step_m', 'max_ik_step_rad', 'tracking_error_rad'):
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'{key} doit être fini et strictement positif')
    if c['dt_s'] > 0.1:
        raise ValueError('dt_s doit être <= 0.1 s')
    if (type(c['servo_speed_ticks_s']) is not int or not 1 <= c['servo_speed_ticks_s'] <= 3400
            or type(c['servo_acceleration']) is not int or not 1 <= c['servo_acceleration'] <= 254):
        raise ValueError('Vitesse/accélération servo invalides')
    scales = joint_limit_scales(c)
    if (c['max_velocity_rad_s'] * float(scales['velocity_scale'].max())
            * max(joint_scale(j) for j in c['joints']) > c['servo_speed_ticks_s']):
        raise ValueError('La limite de vitesse servo est inférieure à celle du plan')
    return c


def joint_limit_scales(c):
    """Per-joint multipliers of the scalar velocity/acceleration/jerk limits (default 1).

    Optional calibration key: "joint_limits": {"velocity_scale": [7], "acceleration_scale": [7],
    "jerk_scale": [7]} (any subset). A joint that must move gently (J2 carries the whole arm)
    gets a scale below 1; the Ruckig profile then synchronises all joints on the slowest."""
    limits = c.get('joint_limits') or {}
    unknown = set(limits) - {'velocity_scale', 'acceleration_scale', 'jerk_scale'}
    if unknown:
        raise ValueError(f'joint_limits: unknown keys {sorted(unknown)}')
    out = {}
    for key in ('velocity_scale', 'acceleration_scale', 'jerk_scale'):
        values = limits.get(key, [1.0] * 7)
        if (not isinstance(values, list) or len(values) != 7
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                       or not 0 < v <= 4 for v in values)):
            raise ValueError(f'joint_limits.{key}: seven values within (0, 4] required')
        out[key] = np.array(values, float)
    return out


def require_calibration(c):
    if c.get('hardware_verified') is not True or not c.get('bus') or c['bus'] == 'auto':
        raise ValueError('Mapping matériel vérifié et numéro de bus explicite requis')
    for j in c['joints']:
        if c['max_velocity_rad_s'] * joint_scale(j) > c['servo_speed_ticks_s']:
            raise ValueError('Échelle incompatible avec la vitesse servo')
        if (j['sign'] not in (-1, 1) or type(j['sign']) is not int
                or any(type(j[k]) is not int for k in ('zero_ticks', 'min_ticks', 'max_ticks'))
                or not 0 <= j['zero_ticks'] <= 4095
                or not 0 <= j['min_ticks'] < j['max_ticks'] <= 4095):
            raise ValueError(f"{j['name']}: zéro URDF, signe et limites mesurés requis")


def ticks_to_joints(c, ticks):
    require_calibration(c)
    ticks = vector(ticks, 7, 'ticks')
    if any(not j['min_ticks'] <= p <= j['max_ticks'] for j, p in zip(c['joints'], ticks)):
        raise ValueError('Position encodeur hors de la plage configurée')
    # No modulo or guessed midpoint: mapping is continuous on a non-wrapping arc.
    return np.array([(p-j['zero_ticks']) * j['sign'] / joint_scale(j)
                     for j, p in zip(c['joints'], ticks)])


def joints_to_ticks(c, q):
    require_calibration(c)
    q = vector(q, 7, 'joints')
    ticks = [round(j['zero_ticks'] + j['sign'] * angle * joint_scale(j))
             for j, angle in zip(c['joints'], q)]
    if any(not j['min_ticks'] <= p <= j['max_ticks'] for j, p in zip(c['joints'], ticks)):
        raise ValueError('Consigne hors calibration : refus, sans écrêtage')
    return ticks


class Arm:
    def __init__(self, urdf, config):
        self.c = config
        self.collision = None
        root = ET.parse(urdf).getroot()
        # Walk BACK from the selected tool mount; never include the gripper branches.
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        path, link = [], config['tip_link']
        while link != config['frame']:
            if link not in parents or len(path) > len(parents):
                raise ValueError('Chaîne base_link -> Gripper_Base absente de cet URDF')
            joint = parents[link]
            path.insert(0, joint)
            link = joint.find('parent').get('link')
        active = [j for j in path if j.get('type') != 'fixed']
        if [j.get('name') for j in active] != JOINTS:
            raise ValueError('URDF incompatible : sept joints rev_motor_01..07 attendus')
        self.bounds = np.array([[float(j.find('limit').get(k)) for k in ('lower', 'upper')]
                                for j in active])
        velocities = [float(j.find('limit').get('velocity')) for j in active]
        if config['max_velocity_rad_s'] * float(joint_limit_scales(config)['velocity_scale'].max()) > min(velocities):
            raise ValueError('Vitesse supérieure aux limites URDF')
        if config.get('hardware_verified'):
            require_calibration(config)
            for i, j in enumerate(config['joints']):
                ends = sorted((p-j['zero_ticks']) * j['sign'] / joint_scale(j)
                              for p in (j['min_ticks'], j['max_ticks']))
                self.bounds[i] = [max(self.bounds[i, 0], ends[0]), min(self.bounds[i, 1], ends[1])]
        if not np.all(np.isfinite(self.bounds)) or np.any(self.bounds[:, 0] >= self.bounds[:, 1]):
            raise ValueError('Limites articulaires incompatibles')
        small = ET.Element('robot', name='elrobot_arm_only')
        ET.SubElement(small, 'link', name=config['frame'])
        for joint in path:
            small.append(joint)
            ET.SubElement(small, 'link', name=joint.find('child').get('link'))
        ET.SubElement(small, 'link', name='postal_tcp')
        tcp = ET.SubElement(small, 'joint', name='postal_tcp_fixed', type='fixed')
        ET.SubElement(tcp, 'parent', link=config['tip_link'])
        ET.SubElement(tcp, 'child', link='postal_tcp')
        ET.SubElement(tcp, 'origin', xyz=' '.join(map(str, config['tcp_xyz_m'])), rpy='0 0 0')
        mask = [False] + [j.get('type') != 'fixed' for j in path] + [False]
        with tempfile.TemporaryDirectory(prefix='postal-urdf-') as tmp:
            p = Path(tmp) / 'arm.urdf'
            ET.ElementTree(small).write(p)
            self.chain = Chain.from_urdf_file(str(p), base_elements=[config['frame']],
                                            active_links_mask=mask, symbolic=False)
        for link, bounds in zip(np.array(self.chain.links, dtype=object)[mask], self.bounds):
            link.bounds = tuple(bounds)

    def check(self, q):
        q = vector(q, 7, 'joints en radians')
        if np.any(q < self.bounds[:, 0]-1e-9) or np.any(q > self.bounds[:, 1]+1e-9):
            raise ValueError('Position hors limites URDF/calibration')
        return q

    def full(self, q):
        return self.chain.active_to_full(self.check(q), np.zeros(len(self.chain.links)))

    def fk(self, q):
        return self.chain.forward_kinematics(self.full(q))

    def ik(self, xyz, seed, rotation=None, max_step=None):
        xyz = vector(xyz, 3, 'cible XYZ en mètres')
        seed_full = self.full(seed)
        links = [link for link, active in zip(self.chain.links, self.chain.active_links_mask) if active]
        saved = [link.bounds for link in links]
        try:
            if max_step is not None:
                for link, bounds, angle in zip(links, saved, seed):
                    link.bounds = (max(bounds[0], angle-max_step), min(bounds[1], angle+max_step))
            result = self.chain.inverse_kinematics(
                target_position=xyz, initial_position=seed_full,
                target_orientation=rotation, orientation_mode='all' if rotation is not None else None)
        finally:
            for link, bounds in zip(links, saved):
                link.bounds = bounds
        q = self.check(self.chain.active_from_full(result))
        reached = self.fk(q)
        error = np.linalg.norm(reached[:3, 3]-xyz)
        if error > self.c['position_tolerance_m']:
            raise ValueError(f'Cible inaccessible ou IK non convergente : erreur {error*1000:.2f} mm')
        if rotation is not None:
            error_r = Rotation.from_matrix(rotation.T @ reached[:3, :3]).magnitude()
            if error_r > self.c['orientation_tolerance_rad']:
                raise ValueError(f'Orientation inaccessible : erreur {error_r:.3f} rad')
        return q

    def segment(self, start, end):
        start, end = self.check(start), self.check(end)
        inp = InputParameter(7)
        inp.current_position, inp.target_position = start.tolist(), end.tolist()
        inp.current_velocity = inp.current_acceleration = [0.0]*7
        inp.target_velocity = inp.target_acceleration = [0.0]*7
        scales = joint_limit_scales(self.c)  # per-joint limits (default: the same for all)
        inp.max_velocity = (self.c['max_velocity_rad_s'] * scales['velocity_scale']).tolist()
        inp.max_acceleration = (self.c['max_acceleration_rad_s2'] * scales['acceleration_scale']).tolist()
        inp.max_jerk = (self.c['max_jerk_rad_s3'] * scales['jerk_scale']).tolist()
        inp.synchronization = Synchronization.Phase
        traj = Trajectory(7)
        status = Ruckig(7).calculate(inp, traj)
        if status < Result.Working:
            raise ValueError(f'Ruckig refuse la trajectoire : {status}')
        n = max(1, math.ceil(traj.duration/self.c['dt_s']))
        if n > 100000:
            raise ValueError('Trajectoire trop longue')
        # Include the exact endpoint and bound the last interval by dt.
        rows = []
        for t in np.linspace(0, traj.duration, n+1):
            q, v, a = traj.at_time(float(t))
            self.check(q)
            rows.append({'t_s': float(t), 'q_rad': q, 'v_rad_s': v, 'a_rad_s2': a})
        return rows

    def line_waypoints(self, q0, delta, step_m=0.002, position_tol_m=0.0003, rotation_tol_rad=0.26,
                       max_joint_step_rad=0.05):
        """Joint configurations along a STRAIGHT Cartesian line of the TCP (see _line_pass).

        First the orientation is held STRICTLY (within 0.003 rad). If that is impossible, typically because a
        joint sits at its limit, the line is solved again letting the orientation drift just enough, never
        beyond rotation_tol_rad. The position is exact either way."""
        try:
            return self._line_pass(q0, delta, step_m, position_tol_m, 0.003, max_joint_step_rad, 100., .01)
        except ValueError:
            return self._line_pass(q0, delta, step_m, position_tol_m, rotation_tol_rad, max_joint_step_rad, .3, .3)

    def _line_pass(self, q0, delta, step_m, position_tol_m, rotation_tol_rad, max_joint_step_rad,
                   rotation_weight, regularization):
        """One solve of the line. From the pose of q0 the TCP moves by `delta` (base_link, metres) in steps
        of at most `step_m`. The POSITION of every point is exact (least-squares IK polished far below
        position_tol_m, which is checked); the orientation error is bounded by rotation_tol_rad (checked).
        rotation_weight / regularization set how much the orientation is held versus how little the joints
        move. Each point is seeded by the previous one. Raises ValueError if a point cannot be reached, leaves
        the joint limits, or the solution jumps between neighbouring points."""
        from scipy.optimize import least_squares
        q0 = self.check(q0)
        delta = vector(delta, 3, 'déplacement de la ligne')
        length = float(np.linalg.norm(delta))
        if not 0 < length <= 0.5:
            raise ValueError('Ligne droite : longueur entre 0 et 50 cm')
        T0 = self.fk(q0)
        p0, R0 = T0[:3, 3], T0[:3, :3]
        n = max(1, math.ceil(length/step_m))
        lo, hi = self.bounds[:, 0], self.bounds[:, 1]
        points, q = [q0], q0
        for k in range(1, n+1):
            target, previous = p0 + delta*k/n, q

            def residual(x, target=target, previous=previous):
                T = self.chain.forward_kinematics(self.chain.active_to_full(x, np.zeros(len(self.chain.links))))
                rotation = Rotation.from_matrix(R0.T @ T[:3, :3]).as_rotvec()
                return np.concatenate([(T[:3, 3]-target)*10000., rotation*rotation_weight, regularization*(x-previous)])  # position first
            seed = np.clip(previous, lo+1e-9, hi-1e-9)
            result = least_squares(residual, seed, bounds=(lo, hi), xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=200)
            q = self.check(result.x)
            reached = self.fk(q)
            error = float(np.linalg.norm(reached[:3, 3]-target))
            error_r = float(Rotation.from_matrix(R0.T @ reached[:3, :3]).magnitude())
            if error > position_tol_m or error_r > rotation_tol_rad:
                raise ValueError(f'Ligne droite inaccessible au point {k}/{n} : erreur {error*1000:.2f} mm, rotation {np.degrees(error_r):.1f}°')
            if np.max(np.abs(q-previous)) > max_joint_step_rad:
                raise ValueError(f'Ligne droite : discontinuité IK au point {k}/{n}')
            points.append(q)
        return points

    def timed_path(self, points):
        """One smooth rest-to-rest profile along a dense joint polyline (no stop at the points).

        The path parameter is the joint-space arc length. Its velocity / acceleration / jerk limits are the
        tightest the per-joint limits allow over all segments (peak |dq_j/ds|), so every joint stays within
        its calibrated (and per-joint scaled) limits. Returns rows like segment()."""
        points = np.array([self.check(p) for p in points], float)
        steps = np.diff(points, axis=0)
        lengths = np.linalg.norm(steps, axis=1)
        if len(points) < 2 or np.any(lengths <= 0):
            raise ValueError('Chemin : au moins deux points distincts requis')
        s = np.concatenate([[0.], np.cumsum(lengths)])
        direction = steps/lengths[:, None]
        peak = np.maximum(np.max(np.abs(direction), axis=0), 1e-9)
        scales = joint_limit_scales(self.c)
        limits = [float(np.min(self.c[key]*scale/peak)) for key, scale in (
            ('max_velocity_rad_s', scales['velocity_scale']), ('max_acceleration_rad_s2', scales['acceleration_scale']),
            ('max_jerk_rad_s3', scales['jerk_scale']))]
        inp = InputParameter(1)
        inp.current_position, inp.target_position = [0.], [float(s[-1])]
        inp.current_velocity = inp.current_acceleration = [0.]
        inp.target_velocity = inp.target_acceleration = [0.]
        inp.max_velocity, inp.max_acceleration, inp.max_jerk = [limits[0]], [limits[1]], [limits[2]]
        traj = Trajectory(1)
        status = Ruckig(1).calculate(inp, traj)
        if status < Result.Working:
            raise ValueError(f'Ruckig refuse la trajectoire : {status}')
        n = max(1, math.ceil(traj.duration/self.c['dt_s']))
        if n > 100000:
            raise ValueError('Trajectoire trop longue')
        rows = []
        for t in np.linspace(0, traj.duration, n+1):
            position, velocity, acceleration = traj.at_time(float(t))
            st = min(max(position[0], 0.), s[-1])
            k = min(int(np.searchsorted(s, st, side='right'))-1, len(direction)-1)
            q = np.array([np.interp(st, s, points[:, j]) for j in range(7)])
            self.check(q)
            rows.append({'t_s': float(t), 'q_rad': q, 'v_rad_s': direction[k]*velocity[0],
                         'a_rad_s2': direction[k]*acceleration[0]})
        return rows

    def plan(self, start, targets, mode='joint', rotation=None):
        if mode not in ('joint', 'linear') or not targets:
            raise ValueError('Mode joint/linear et au moins une cible requis')
        q = self.check(start)
        if mode == 'linear' and rotation is None:
            rotation = self.fk(q)[:3, :3]  # preserve orientation for insertion
        rows = [{'t_s': 0.0, 'q_rad': q.tolist(), 'v_rad_s': [0.0]*7, 'a_rad_s2': [0.0]*7}]
        for target in targets:
            target = vector(target, 3, 'cible')
            origin = self.fk(q)[:3, 3]
            n = max(1, math.ceil(np.linalg.norm(target-origin)/self.c['cartesian_step_m'])) if mode == 'linear' else 1
            if n > 1000:
                raise ValueError('Trop de points cartésiens ; vérifier les unités (mètres)')
            for xyz in np.linspace(origin, target, n+1)[1:]:
                next_q = self.ik(xyz, q, rotation,
                                 self.c['max_ik_step_rad'] if mode == 'linear' else None)
                if mode == 'linear' and np.max(np.abs(next_q-q)) > self.c['max_ik_step_rad']:
                    raise ValueError('Discontinuité IK sur le chemin cartésien')
                if self.collision and mode == 'joint':
                    if not self.collision.valid(next_q):
                        rng = np.random.default_rng(0)
                        for _ in range(8):
                            seed = rng.uniform(self.bounds[:, 0], self.bounds[:, 1])
                            try:
                                alternative = self.ik(xyz, seed, rotation)
                                if self.collision.valid(alternative):
                                    next_q = alternative
                                    break
                            except ValueError:
                                continue
                    path = self.collision.joint_path(q, next_q)
                else:
                    path = [q, next_q]
                    if self.collision and not self.collision.motion_valid(q, next_q):
                        raise ValueError('Approche cartésienne en collision ; aucun détour pendant une insertion droite')
                for edge_start, edge_end in zip(path, path[1:]):
                    segment = self.segment(edge_start, edge_end)
                    offset = rows[-1]['t_s']
                    for row in segment[1:]:
                        if mode == 'linear':
                            pose = self.fk(row['q_rad'])
                            delta = xyz-origin
                            alpha = np.clip(np.dot(pose[:3, 3]-origin, delta)/max(np.dot(delta, delta), 1e-20), 0, 1)
                            if np.linalg.norm(pose[:3, 3]-(origin+alpha*delta)) > self.c['position_tolerance_m']:
                                raise ValueError('Écart excessif au segment cartésien')
                            if Rotation.from_matrix(rotation.T @ pose[:3, :3]).magnitude() > self.c['orientation_tolerance_rad']:
                                raise ValueError('Écart excessif à l’orientation du chemin')
                        row['t_s'] += offset
                        rows.append(row)
                q, origin = next_q, xyz
        plan = {'frame': self.c['frame'], 'tip_link': self.c['tip_link'],
                'tcp_xyz_m': self.c['tcp_xyz_m'], 'mode': mode,
                'collision_checked': bool(self.collision), 'targets_xyz_m': [list(t) for t in targets],
                'final_xyz_m': self.fk(q)[:3, 3].tolist(), 'samples': rows}
        if self.collision:
            plan.update(collision_scene_sha256=self.collision.fingerprint,
                        collision_resolution_rad=self.collision.scene['edge_step_rad'])
            self.collision.validate_plan(plan)
            plan['collision_checks'] = self.collision.checks
        return plan
