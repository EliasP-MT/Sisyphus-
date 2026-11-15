"""Reference postal FSM and arm adapter; observations, semantic fixtures and feedback.

No camera, MuJoCo ground-truth pose, or hardware transport is used here.
"""
import json
import sys
import tempfile
from enum import Enum

import numpy as np

from .common import HERE, ROOT, URDF, rotation, wrap
from .navigation import HolonomicNavigator
from .model import mailbox_geometry


class State(str, Enum):
    INITIALISE = 'INITIALISE'
    PICK = 'PICK'
    CLOSE = 'CLOSE'
    STOW = 'STOW'
    NAVIGATE = 'NAVIGATE'
    DOCK = 'DOCK'
    APPROACH = 'APPROACH'
    ALIGN = 'ALIGN'
    INSERT = 'INSERT'
    RELEASE = 'RELEASE'
    RETRACT = 'RETRACT'
    DONE = 'DONE'
    FAULT = 'FAULT'


class ArmController:
    def __init__(self, arm, c):
        # Reuse the existing ElRobot IK, Bullet/CAD checks, OMPL and Ruckig.
        sys.path.insert(0, str(ROOT/'arm_control'))
        from collision import CollisionWorld
        scene = json.loads((ROOT/'arm_control/robot_scene.example.json').read_text())
        scene['cad']['manifest'] = str(ROOT/'arm_control/assets/full_robot_collision/manifest.json')
        scene['cad']['xyz_m'] = (np.asarray(c['cad_xyz_m'])-c['arm_mount_xyz_m']).tolist()
        scene['cad']['rpy_rad'] = c['cad_rpy_rad']
        scene['planning_time_s'] = 12
        self.scene = scene
        self._directory = tempfile.TemporaryDirectory(prefix='arm-scene-', dir=HERE/'.runtime')
        from pathlib import Path
        self.path = Path(self._directory.name)/'scene.json'
        self.path.write_text(json.dumps(scene))
        self.world = CollisionWorld(URDF, arm, self.path)
        self.arm, self.c = arm, c
        self.times, self.positions = None, None
        self.end_time = 0
        self.goal = np.zeros(7)

    def update_scene(self, base_pose, held):
        from collision import CollisionWorld
        self.scene['gripper_angle_rad'] = 1.0 if held else 0.0
        self.scene['boxes'] = []
        for box in self.c['mailboxes']:
            for name, position, size in mailbox_geometry(box):
                xy = rotation(base_pose[2]).T @ (np.asarray(position[:2])-base_pose[:2])
                local = np.r_[xy, position[2]-self.c['base_height_m']]-self.c['arm_mount_xyz_m']
                self.scene['boxes'].append(dict(name=name, xyz_m=local.tolist(), size_m=size, rpy_rad=[0, 0, -base_pose[2]]))
        self.scene['payload'] = (dict(name='letter', size_m=[.1, .1, .002],
            xyz_m=self.c['letter_center_from_tcp_m'], rpy_rad=[0, 0, 0],
            ignore_robot_links=[name for name in self.world.links if name.startswith('Gripper_') or name == 'ST3215_8_v1_1']) if held else None)
        self.path.write_text(json.dumps(self.scene))
        self.world.close()
        self.world = CollisionWorld(URDF, self.arm, self.path)
        self.arm.collision = self.world

    def ik(self, xyz, seed, orientation=None):
        rng = np.random.default_rng(21)
        seeds = [np.asarray(seed), np.zeros(7)]+[rng.uniform(self.arm.command_bounds[:, 0], self.arm.command_bounds[:, 1]) for _ in range(8)]
        reason = 'IK non convergente'
        for initial in seeds:
            try:
                initial = np.clip(initial, self.arm.command_bounds[:, 0], self.arm.command_bounds[:, 1])
                candidate = self.arm.ik(xyz, initial, orientation)
                collision = self.world.collision(candidate)
                if collision is None: return candidate
                reason = collision
            except ValueError as exc:
                reason = str(exc)
        raise ValueError(f'Aucune IK libre pour la cible : {reason}')

    def move(self, current, goal, now, linear=False):
        self.arm.check(goal)
        if linear:
            self.arm.collision = self.world
            pose = self.arm.fk(goal)
            rows = self.arm.plan(current, [pose[:3, 3]], mode='linear', rotation=pose[:3, :3])['samples']
            self.times = now+np.array([row['t_s'] for row in rows])
            self.positions = np.array([row['q_rad'] for row in rows])
            self.end_time, self.goal = self.times[-1], self.positions[-1].copy()
            return
        if linear and not self.world.motion_valid(current, goal):
            raise ValueError('Insertion nominale en collision')
        path = [current, goal] if linear else self.world.joint_path(current, goal)
        rows, elapsed = [], 0
        for a, b in zip(path, path[1:]):
            segment = self.arm.segment(a, b)
            for row in segment[1:]: rows.append((elapsed+row['t_s'], row['q_rad']))
            elapsed += segment[-1]['t_s']
        if not rows: rows = [(0, np.asarray(goal).tolist())]
        self.times = np.r_[now, now+np.array([row[0] for row in rows])]
        self.positions = np.vstack([current, np.array([row[1] for row in rows])])
        self.end_time, self.goal = now+elapsed, np.asarray(goal).copy()

    def command(self, now):
        if self.times is None: return self.goal.copy()
        return np.array([np.interp(now, self.times, self.positions[:, i]) for i in range(7)])

    def done(self, observation):
        now = observation['time'][0]
        return now >= self.end_time and np.max(np.abs(observation['joints']-self.goal)) < .018

    def close(self):
        self.world.close()
        self._directory.cleanup()


class PostalFSM:
    def __init__(self, c, arm, pickup_q):
        self.c, self.arm = c, arm
        self.motion = ArmController(arm, c)
        self.navigator = HolonomicNavigator(c)
        self.pickup_q = np.asarray(pickup_q)
        self.pickup_pose = arm.fk(pickup_q)
        self.carry_q = self.motion.ik([0, .13, .20], np.zeros(7))
        self.state, self.entered = State.INITIALISE, 0.0
        self.index, self.history, self.error = 0, [], None
        self.initialised = False
        self.pending_grasp = False
        self.pending_release = False
        self.box = None
        self.dock_pose = None
        self.finished = False

    @property
    def letter(self): return self.c['letters'][self.index]

    def transition(self, state, now):
        self.history.append(dict(time_s=float(now), before=self.state.value, after=state.value, letter=self.index))
        self.state, self.entered = state, now

    def fail(self, reason, now):
        self.error = str(reason)
        self.transition(State.FAULT, now)
        self.finished = True

    def _pick_target(self):
        xyz = self.pickup_pose[:3, 3]+self.pickup_pose[:3, :3] @ np.array([0, 0, .006*self.index])
        return self.motion.ik(xyz, self.pickup_q, self.pickup_pose[:3, :3])

    def _mailbox_target(self, slam, depth, neutral=False):
        world = np.asarray(self.box['front_xyz_m'])+np.array([0, depth, 0])
        xy = rotation(slam.pose[2]).T @ (world[:2]-slam.pose[:2])
        local = np.r_[xy, world[2]-self.c['base_height_m']]-self.c['arm_mount_xyz_m']
        yaw = -slam.pose[2]
        target_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return self.motion.ik(local, np.zeros(7) if neutral else self.motion.goal, target_rotation)

    def update(self, observation, slam, feedback=None):
        feedback = feedback or {}
        now = float(observation['time'][0])
        action = dict(twist=np.zeros(3), arm=self.motion.command(now), gripper=np.clip(observation['gripper'].copy(), 0, 2.2028))
        events = []
        try:
            if self.state in (State.DONE, State.FAULT): return action, events
            if now-self.entered > self.c['task']['state_timeout_s']:
                raise RuntimeError(f'Timeout {self.state.value}')
            if self.state != State.INITIALISE and now-slam.last_scan_time > .6:
                raise RuntimeError('LiDAR virtuel périmé ; base arrêtée')
            if self.state == State.INITIALISE:
                action['gripper'][:] = 0
                if slam.healthy(now):
                    self.motion.move(observation['joints'], self._pick_target(), now)
                    self.transition(State.PICK, now)
            elif self.state == State.PICK:
                action['gripper'][:] = 0
                if self.motion.done(observation): self.transition(State.CLOSE, now)
            elif self.state == State.CLOSE:
                action['gripper'][:] = 1.0
                if feedback.get('grasped'):
                    self.pending_grasp = False
                    self.motion.update_scene(slam.pose, held=True)
                    self.carry_q = self.motion.ik([0, .13, .20], observation['joints'])
                    self.motion.move(observation['joints'], self.carry_q, now)
                    self.transition(State.STOW, now)
                elif now-self.entered > .8 and not self.pending_grasp:
                    events.append(dict(type='grasp', letter=self.letter['id']))
                    self.pending_grasp = True
                elif feedback.get('grasp_failed'):
                    raise RuntimeError('Échec de prise : pince ou distance TCP')
            elif self.state == State.STOW:
                action['gripper'][:] = 1.0
                if self.motion.done(observation):
                    # Recipient labels/box poses are semantic fixtures while vision is out of scope.
                    self.box = next((b for b in self.c['mailboxes'] if b['name'] == self.letter['recipient']), None)
                    if self.box is None: raise RuntimeError('Destinataire inconnu')
                    self.transition(State.NAVIGATE, now)
            elif self.state == State.NAVIGATE:
                action['gripper'][:] = 1.0
                action['twist'], arrived = self.navigator.command(self.box['approach_pose'], slam, observation)
                if self.navigator.blocked_since is not None and now-self.navigator.blocked_since > 8:
                    raise RuntimeError('Navigation bloquée')
                if arrived:
                    self.dock_pose = np.array(self.box['approach_pose'])
                    self.transition(State.DOCK, now)
            elif self.state == State.DOCK:
                action['gripper'][:] = 1
                delta = self.dock_pose[:2]-slam.pose[:2]
                yaw = wrap(self.dock_pose[2]-slam.pose[2])
                action['twist'] = np.r_[rotation(slam.pose[2]).T @ np.clip(1.5*delta, -.025, .025), np.clip(1.5*yaw, -.15, .15)]
                if np.linalg.norm(delta) < .008 and abs(yaw) < .012:
                    self.motion.update_scene(slam.pose, held=True)
                    self.motion.move(observation['joints'], self._mailbox_target(slam, -.12, neutral=True), now)
                    self.transition(State.APPROACH, now)
            elif self.state == State.APPROACH:
                action['gripper'][:] = 1.0
                if self.motion.done(observation):
                    self.motion.update_scene(slam.pose, held=True)
                    self.motion.move(observation['joints'], self._mailbox_target(slam, -.06), now, linear=True)
                    self.transition(State.ALIGN, now)
            elif self.state == State.ALIGN:
                action['gripper'][:] = 1.0
                if self.motion.done(observation):
                    self.motion.update_scene(slam.pose, held=True)
                    self.motion.move(observation['joints'], self._mailbox_target(slam, -.025), now, linear=True)
                    self.transition(State.INSERT, now)
            elif self.state == State.INSERT:
                action['gripper'][:] = 1.0
                if self.motion.done(observation): self.transition(State.RELEASE, now)
            elif self.state == State.RELEASE:
                action['gripper'][:] = 0
                if 'released' in feedback:
                    if not feedback['released'].get('accepted'): raise RuntimeError(f"Dépôt invalide : {feedback['released']}")
                    self.pending_release = False
                    self.motion.update_scene(slam.pose, held=False)
                    self.carry_q = self.motion.ik([0, .13, .20], self.carry_q)
                    self.motion.move(observation['joints'], self.carry_q, now)
                    self.transition(State.RETRACT, now)
                elif now-self.entered > .5 and not self.pending_release:
                    events.append(dict(type='release'))
                    self.pending_release = True
            elif self.state == State.RETRACT:
                action['gripper'][:] = 0
                if self.motion.done(observation):
                    if self.letter['id'] not in feedback.get('delivered', {}):
                        if now-self.motion.end_time > 6: raise RuntimeError('Lettre non confirmée dans le volume intérieur')
                        return action, events
                    self.index += 1
                    if self.index == len(self.c['letters']):
                        self.transition(State.DONE, now); self.finished = True
                    else:
                        self.motion.move(observation['joints'], self._pick_target(), now)
                        self.transition(State.PICK, now)
            action['arm'] = self.motion.command(now)
            if self.state in (State.APPROACH, State.ALIGN, State.INSERT, State.RELEASE, State.RETRACT) and self.dock_pose is not None:
                delta = self.dock_pose[:2]-slam.pose[:2]
                body = rotation(slam.pose[2]).T @ np.clip(1.5*delta, -.025, .025)
                action['twist'] = np.r_[body, np.clip(1.5*wrap(self.dock_pose[2]-slam.pose[2]), -.15, .15)]
        except (ValueError, RuntimeError) as exc:
            self.fail(exc, now)
            action['twist'][:] = 0
            action['arm'] = observation['joints'].copy()
        return action, events

    def close(self): self.motion.close()
