"""PyBullet geometry checking and OMPL RRTConnect for the ElRobot joint space."""
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from cad import read_solids, sha256
from planner import JOINTS, vector


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', vector(rpy, 3, 'orientation')).as_matrix()
    result[:3, 3] = vector(xyz, 3, 'position')
    return result


class CollisionWorld:
    def __init__(self, urdf, arm, scene_path):
        self.client = p.connect(p.DIRECT)
        self.arm, self.urdf = arm, Path(urdf).resolve()
        self.scene_path = Path(scene_path).resolve()
        self.files = {self.scene_path: sha256(self.scene_path), self.urdf: sha256(self.urdf)}
        self.checks = 0
        try:
            self.scene = json.loads(self.scene_path.read_text())
            s = self.scene
            if s['version'] != 1 or s['frame'] != 'base_link':
                raise ValueError('Scène version 1 dans base_link requise')
            for key, low, high in [('margin_m', 0.002, 0.05), ('self_margin_m', 0.001, 0.02),
                                   ('edge_step_rad', 0.0001, 0.01), ('planning_time_s', 0.05, 30)]:
                if not math.isfinite(s[key]) or not low <= s[key] <= high:
                    raise ValueError(f'{key}: valeur hors [{low}, {high}]')
            if not 0 <= s['gripper_angle_rad'] <= 2.2028:
                raise ValueError('Ouverture modélisée de pince invalide')
            self.robot = p.loadURDF(str(self.urdf), useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION | p.URDF_MAINTAIN_LINK_ORDER,
                physicsClientId=self.client)
            self.links, self.joints, self.parents, self.groups = {'base_link': -1}, {}, {}, {-1: -1}
            for i in range(p.getNumJoints(self.robot, physicsClientId=self.client)):
                info = p.getJointInfo(self.robot, i, physicsClientId=self.client)
                self.links[info[12].decode()] = i
                self.joints[info[1].decode()] = i
                self.parents[i] = info[16]
                self.groups[i] = self.groups[info[16]] if info[2] == p.JOINT_FIXED else i
            if any(name not in self.joints for name in JOINTS):
                raise ValueError('URDF Bullet incompatible avec le bras IK')
            # The gripper is held at one modelled opening, never planned here.
            # Gear/jaw contacts in the mimic mechanism are intentional.
            tip = self.links['Gripper_Base_v1_1']
            for i in self.parents:
                ancestor = i
                while ancestor != -1 and ancestor != tip:
                    ancestor = self.parents[ancestor]
                if ancestor == tip:
                    self.groups[i] = self.groups[tip]
            for mesh in ET.parse(self.urdf).getroot().iter('mesh'):
                source = (self.urdf.parent/mesh.get('filename')).resolve()
                self.files[source] = sha256(source)
            self.neighbours = {frozenset((self.groups[i], self.groups[parent]))
                               for i, parent in self.parents.items()}
            self.allowed = set()
            for pair in s.get('allowed_self_pairs', []):
                if len(pair) != 2 or any(name not in self.links for name in pair):
                    raise ValueError('Paire de liens autorisée invalide')
                self.allowed.add(frozenset(self.links[name] for name in pair))
            indices = sorted(self.links.values())
            self.self_pairs = np.array([(a, b) for a in indices for b in indices
                if a < b and self.groups[a] != self.groups[b]
                and frozenset((self.groups[a], self.groups[b])) not in self.neighbours
                and frozenset((a, b)) not in self.allowed], dtype=int).reshape(-1, 2)
            self.centres = {}
            for i in self.links.values():
                vertices = p.getMeshData(self.robot, i, flags=p.MESH_DATA_SIMULATION_MESH,
                                         physicsClientId=self.client)[1]
                if vertices:
                    self.centres[i] = np.mean(vertices, axis=0)
            self.cad_bodies, self.obstacles = [], []
            self._set_joints(np.zeros(7))
            if s.get('cad') is not None:
                self._load_cad(s['cad'])
            ids = {b['id'] for b in self.cad_bodies}
            self.allowed_cad = set()
            for pair in s.get('allowed_cad_pairs', []):
                if len(pair) != 2 or any(type(i) is not int or i not in ids for i in pair):
                    raise ValueError('Paire de solides CAD autorisée invalide')
                self.allowed_cad.add(frozenset(pair))
            for box in s.get('boxes', []):
                self.obstacles.append(self._box(box))
            self.payload = self._box(s['payload']) if s.get('payload') else None
            if self.payload:
                self.payload['local'] = transform(s['payload']['xyz_m'], s['payload']['rpy_rad'])
            self.config_digest = hashlib.sha256(json.dumps(arm.c, sort_keys=True).encode()).hexdigest()
            digest = json.dumps({str(path): digest for path, digest in self.files.items()}, sort_keys=True)+self.config_digest
            self.fingerprint = hashlib.sha256(digest.encode()).hexdigest()
        except BaseException:
            self.close()
            raise

    def __enter__(self): return self
    def __exit__(self, *args): self.close()

    def close(self):
        if self.client is not None:
            p.disconnect(physicsClientId=self.client)
            self.client = None

    def _pose(self, index):
        if index == -1:
            return np.eye(4)  # root link frame, not Bullet's inertial centre
        state = p.getLinkState(self.robot, index, computeForwardKinematics=True, physicsClientId=self.client)
        result = np.eye(4)
        result[:3, :3] = np.array(p.getMatrixFromQuaternion(state[5])).reshape(3, 3)
        result[:3, 3] = state[4]
        return result

    def _set_body(self, body, pose):
        q = Rotation.from_matrix(pose[:3, :3]).as_quat()
        p.resetBasePositionAndOrientation(body, pose[:3, 3], q, physicsClientId=self.client)

    def _set_joints(self, q):
        for name, value in zip(JOINTS, q):
            p.resetJointState(self.robot, self.joints[name], float(value), physicsClientId=self.client)
        angle = self.scene['gripper_angle_rad']
        for name, value in [('rev_motor_08', angle), ('rev_motor_08_1', -0.0115*angle),
                            ('rev_motor_08_2', 0.0115*angle)]:
            if name in self.joints:
                p.resetJointState(self.robot, self.joints[name], value, physicsClientId=self.client)

    def _box(self, box):
        size = vector(box['size_m'], 3, 'dimensions de boîte')
        if np.any(size <= 0) or np.max(size) > 10:
            raise ValueError('Dimensions de boîte invalides (mètres)')
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size/2, physicsClientId=self.client)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size/2, rgbaColor=[0.95, 0.55, 0.15, 1], physicsClientId=self.client)
        body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, baseVisualShapeIndex=visual, physicsClientId=self.client)
        self._set_body(body, transform(box['xyz_m'], box['rpy_rad']))
        return dict(body=body, name=box['name'])

    def _load_cad(self, cad):
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        path = (self.scene_path.parent/cad['manifest']).resolve()
        manifest = json.loads(path.read_text())
        self.files[path] = sha256(path)
        if manifest['mesh_units'] != 'metres':
            raise ValueError('Maillages CAD requis en mètres')
        source = (path.parent/manifest['source_path']).resolve()
        if sha256(source) != manifest['source_sha256']:
            raise ValueError('Le STEP a changé ; régénérer le maillage')
        self.files[source] = sha256(source)
        solids = read_solids(source)
        if len(solids) != len(manifest['solids']):
            raise ValueError('Inventaire CAD incompatible')
        reference = self.arm.check(cad['reference_q_rad'])
        self._set_joints(reference)
        cad_pose = transform(cad['xyz_m'], cad['rpy_rad'])
        assigned = []
        for binding in cad['bindings']:
            parent = self.links[binding['parent_link']]
            ignored = {self.links[name] for name in binding.get('ignore_robot_links', [])}
            local = np.linalg.inv(self._pose(parent)) @ cad_pose
            for index in binding['solid_ids']:
                if type(index) is not int or not 0 <= index < len(solids) or index in assigned:
                    raise ValueError('Affectation de solide CAD invalide ou répétée')
                assigned.append(index)
                info = manifest['solids'][index]
                mesh = (path.parent/info['mesh']).resolve()
                if sha256(mesh) != info['mesh_sha256']:
                    raise ValueError('Maillage modifié depuis sa conversion STEP')
                self.files[mesh] = sha256(mesh)
                # Static chassis/backpack retains concavities. Moving CAD uses a conservative convex hull.
                shape = p.createCollisionShape(p.GEOM_MESH, fileName=str(mesh),
                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH if parent == -1 else 0,
                    physicsClientId=self.client)
                if shape < 0:
                    raise ValueError('Échec du chargement de géométrie CAD')
                visual = p.createVisualShape(p.GEOM_MESH, fileName=str(mesh),
                    rgbaColor=[0.2, 0.6, 0.8, 1] if parent == -1 else [0.8, 0.3, 0.5, 1], physicsClientId=self.client)
                body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                                         baseVisualShapeIndex=visual, physicsClientId=self.client)
                self.cad_bodies.append(dict(body=body, id=index, name=f'cad_solid_{index}', parent=parent,
                    ignored=ignored, local=local, bounds=np.array(info['bounds_m']).reshape(2, 3),
                    classifier=BRepClass3d_SolidClassifier(solids[index]), centre=info['centre_m']))
        if sorted(assigned) != list(range(len(solids))):
            raise ValueError('Chaque solide CAD doit être affecté à un lien ; aucun oubli implicite')

    def _robot_centre(self, index):
        if index == -1:
            position, orientation = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        else:
            state = p.getLinkState(self.robot, index, physicsClientId=self.client)
            position, orientation = state[0], state[1]
        return np.array(position)+np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3) @ self.centres.get(index, np.zeros(3))

    def _inside(self, cad, world_point):
        from OCP.gp import gp_Pnt
        from OCP.TopAbs import TopAbs_IN, TopAbs_ON
        point = (cad['inverse'] @ np.append(world_point, 1))[:3]
        if np.any(point < cad['bounds'][0]) or np.any(point > cad['bounds'][1]):
            return False
        cad['classifier'].Perform(gp_Pnt(*(point*1000)), 0.01)
        return cad['classifier'].State() in (TopAbs_IN, TopAbs_ON)

    def collision(self, q):
        q = self.arm.check(q)
        if hasattr(self, '_last_q') and np.array_equal(q, self._last_q):
            return self._last_reason
        reason = self._collision(q)
        self._last_q, self._last_reason = q.copy(), reason
        return reason

    def _collision(self, q):
        q = self.arm.check(q)
        self.checks += 1
        self._set_joints(q)
        for cad in self.cad_bodies:
            if cad['parent'] != -1 or 'pose' not in cad:
                pose = self._pose(cad['parent']) @ cad['local']
                self._set_body(cad['body'], pose)
                cad['pose'], cad['inverse'] = pose, np.linalg.inv(pose)
        margin = self.scene['margin_m']
        self_margin = self.scene['self_margin_m']
        # Broad phase avoids expensive queries for intentional adjacent-link
        # overlaps and for distant convex hulls. AABBs include the Bullet margins.
        aabbs = np.array([p.getAABB(self.robot, i, physicsClientId=self.client)
                         for i in sorted(self.links.values())])
        if len(self.self_pairs):
            aa, bb = aabbs[self.self_pairs[:, 0]+1], aabbs[self.self_pairs[:, 1]+1]
            candidates = np.all(np.maximum(aa[:, 0]-bb[:, 1], bb[:, 0]-aa[:, 1]) < self_margin, axis=1)
            for a, b in self.self_pairs[candidates]:
                hits = p.getClosestPoints(self.robot, self.robot, self_margin,
                    linkIndexA=int(a), linkIndexB=int(b), physicsClientId=self.client)
                if hits:
                    return f'auto-collision {a}/{b} ({hits[0][8]:.4f} m)'
        for obstacle in self.obstacles:
            if p.getClosestPoints(self.robot, obstacle['body'], margin, physicsClientId=self.client):
                return f"bras / {obstacle['name']}"
        for cad in self.cad_bodies:
            excluded = cad['ignored'] | {i for i, group in self.groups.items() if group == self.groups[cad['parent']]}
            box = np.array(p.getAABB(cad['body'], physicsClientId=self.client))
            near = np.all(np.maximum(aabbs[:, 0]-box[1], box[0]-aabbs[:, 1]) < margin, axis=1)
            if not any(near[i+1] for i in self.links.values() if i not in excluded):
                continue
            for hit in p.getClosestPoints(self.robot, cad['body'], margin, physicsClientId=self.client):
                if hit[3] not in excluded:
                    return f"bras / {cad['name']} (distance {hit[8]:.4f} m)"
            # Triangle surfaces alone miss a robot wholly enclosed inside a solid.
            for index in self.links.values():
                if index not in excluded and self._inside(cad, self._robot_centre(index)):
                    return f"bras à l’intérieur de {cad['name']}"
        for i, moving in enumerate(self.cad_bodies):
            if moving['parent'] == -1:
                continue
            for other in self.cad_bodies[:i]+self.cad_bodies[i+1:]:
                if self.groups[moving['parent']] == self.groups[other['parent']]:
                    continue
                if frozenset((moving['id'], other['id'])) in self.allowed_cad:
                    continue
                centre = (moving['pose'] @ np.append(moving['centre'], 1))[:3]
                if (p.getClosestPoints(moving['body'], other['body'], margin, physicsClientId=self.client)
                        or self._inside(other, centre)):
                    return f"{moving['name']} / {other['name']}"
            for obstacle in self.obstacles:
                if p.getClosestPoints(moving['body'], obstacle['body'], margin, physicsClientId=self.client):
                    return f"{moving['name']} / {obstacle['name']}"
        if self.payload:
            tip = self._pose(self.links['Gripper_Base_v1_1'])
            tcp = np.eye(4)
            tcp[:3, 3] = self.arm.c['tcp_xyz_m']
            pose = tip @ tcp @ self.payload['local']
            self._set_body(self.payload['body'], pose)
            ignored = {self.links[name] for name in self.scene['payload']['ignore_robot_links']}
            for hit in p.getClosestPoints(self.robot, self.payload['body'], margin, physicsClientId=self.client):
                if hit[3] not in ignored:
                    return 'bras / lettre tenue'
            for obstacle in self.obstacles+self.cad_bodies:
                if obstacle.get('id') in self.scene['payload'].get('ignore_cad_solids', []):
                    continue
                if p.getClosestPoints(self.payload['body'], obstacle['body'], margin, physicsClientId=self.client):
                    return f"lettre tenue / {obstacle['name']}"
                if 'classifier' in obstacle and self._inside(obstacle, pose[:3, 3]):
                    return f"lettre à l’intérieur de {obstacle['name']}"
        return None

    def valid(self, q):
        return self.collision(q) is None

    def motion_valid(self, start, end):
        n = max(1, math.ceil(np.max(np.abs(np.asarray(end)-start))/self.scene['edge_step_rad']))
        return all(self.valid(q) for q in np.linspace(start, end, n+1))

    def require_pose(self, q, label):
        reason = self.collision(q)
        if reason:
            raise ValueError(f'{label} en collision : {reason}')

    def joint_path(self, start, goal):
        self.require_pose(start, 'Départ')
        self.require_pose(goal, 'Arrivée')
        if self.motion_valid(start, goal):
            return [np.array(start), np.array(goal)]
        from ompl import base as ob, geometric as og
        space = ob.RealVectorStateSpace(7)
        bounds = ob.RealVectorBounds(7)
        for i, (low, high) in enumerate(self.arm.bounds):
            bounds.setLow(i, float(low))
            bounds.setHigh(i, float(high))
        space.setBounds(bounds)
        # OMPL's native discrete validator also supports lastValid queries and
        # avoids ownership cycles from Python MotionValidator subclasses.
        space.setLongestValidSegmentFraction(self.scene['edge_step_rad']/space.getMaximumExtent())
        setup = og.SimpleSetup(space)
        si = setup.getSpaceInformation()
        setup.setStateValidityChecker(lambda state: self.valid([state[i] for i in range(7)]))
        a, b = space.allocState(), space.allocState()
        for i in range(7): a[i], b[i] = float(start[i]), float(goal[i])
        setup.setStartAndGoalStates(a, b)
        planner = og.RRTConnect(si)
        planner.setRange(0.25)
        setup.setPlanner(planner)
        setup.solve(self.scene['planning_time_s'])
        if not setup.haveExactSolutionPath():
            raise ValueError('Aucun chemin sans collision trouvé dans le temps imparti')
        path = [np.array([state[i] for i in range(7)]) for state in setup.getSolutionPath().getStates()]
        if any(not self.motion_valid(a, b) for a, b in zip(path, path[1:])):
            raise ValueError('Échec de validation du chemin OMPL')
        return path

    def validate_plan(self, plan):
        if plan.get('collision_scene_sha256') != self.fingerprint:
            raise ValueError('Le plan ne correspond pas à la scène de collision courante')
        rows = plan['samples']
        self.require_pose(rows[0]['q_rad'], 'Départ temporisé')
        for a, b in zip(rows, rows[1:]):
            if not self.motion_valid(a['q_rad'], b['q_rad']):
                raise ValueError('Collision sur la trajectoire temporisée')

    def require_hardware_ready(self):
        if self.scene.get('hardware_verified') is not True:
            raise ValueError('Placement CAD, affectations et géométrie de scène non validés sur le robot')
        if any(sha256(path) != digest for path, digest in self.files.items()):
            raise ValueError('La scène ou sa géométrie a changé ; recalculer le plan')
        if hashlib.sha256(json.dumps(self.arm.c, sort_keys=True).encode()).hexdigest() != self.config_digest:
            raise ValueError('La configuration du bras a changé ; recalculer le plan')

    def save_preview(self, path, q):
        reason = self.collision(q)
        bounds = [p.getAABB(self.robot, i, physicsClientId=self.client) for i in self.links.values()]
        bounds += [p.getAABB(body['body'], physicsClientId=self.client)
                   for body in self.cad_bodies+self.obstacles+([self.payload] if self.payload else [])]
        low, high = np.min(np.array(bounds)[:, 0], axis=0), np.max(np.array(bounds)[:, 1], axis=0)
        centre = (low+high)/2
        distance = max(0.4, np.linalg.norm(high-low)*1.5)
        width, height = 1000, 750
        view = p.computeViewMatrixFromYawPitchRoll(centre, distance, 45, -25, 0, 2)
        projection = p.computeProjectionMatrixFOV(45, width/height, 0.01, distance*5)
        rgba = np.array(p.getCameraImage(width, height, view, projection, renderer=p.ER_TINY_RENDERER,
                                        physicsClientId=self.client)[2], dtype=np.uint8).reshape(height, width, 4)
        def chunk(kind, data):
            return struct.pack('!I', len(data))+kind+data+struct.pack('!I', zlib.crc32(kind+data)&0xffffffff)
        png = b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 6, 0, 0, 0))
        png += chunk(b'IDAT', zlib.compress(b''.join(b'\0'+row.tobytes() for row in rgba)))+chunk(b'IEND', b'')
        Path(path).write_bytes(png)
        return reason
