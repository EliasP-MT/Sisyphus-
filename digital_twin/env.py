"""Gymnasium MuJoCo plant with wheel odometry and a virtual 2-D range sensor."""
import copy
import math

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from .common import KiwiKinematics, compose, config, rotation, wrap
from .model import build


class PostalEnv(gym.Env):
    metadata = {'render_modes': ['rgb_array'], 'render_fps': 20}

    def __init__(self, settings=None, arm_config=None, render_mode=None):
        self.c = copy.deepcopy(settings or config())
        self.model, self.arm, self.xml_path, self.pickup_q = build(self.c, arm_config)
        self.data = mujoco.MjData(self.model)
        self.kinematics = KiwiKinematics(self.c)
        self.render_mode, self.renderer = render_mode, None
        self.joint_ids = [self.model.joint(f'rev_motor_{i:02d}').id for i in range(1, 8)]
        self.arm_qadr = self.model.jnt_qposadr[self.joint_ids]
        self.arm_dadr = self.model.jnt_dofadr[self.joint_ids]
        self.arm_aids = [self.model.actuator(f'arm_motor_{i}').id for i in range(7)]
        self.wheel_ids = [self.model.joint(f'wheel_joint_{i}').id for i in range(3)]
        self.wheel_qadr = self.model.jnt_qposadr[self.wheel_ids]
        self.wheel_dadr = self.model.jnt_dofadr[self.wheel_ids]
        self.wheel_aids = [self.model.actuator(f'wheel_motor_{i}').id for i in range(3)]
        self.base_id, self.tcp_id = self.model.body('platform').id, self.model.site('tcp').id
        self.arm_root_id = self.model.body('base_link').id
        self.lidar_id = self.model.site('lidar').id
        self.gripper_id = self.model.joint('rev_motor_08').id
        self.gripper_aid = self.model.actuator('gripper_motor').id
        self.substeps = round(self.c['control_period_s']/self.c['timestep_s'])
        if not np.isclose(self.substeps*self.c['timestep_s'], self.c['control_period_s']):
            raise ValueError('La période de commande doit être un multiple du pas MuJoCo')
        rays = self.c['lidar']['rays']
        self.angles = np.linspace(-np.pi, np.pi, rays, endpoint=False)
        self.action_space = spaces.Dict(dict(twist=spaces.Box(np.array([-.2, -.2, -1.]), np.array([.2, .2, 1.]), dtype=np.float64),
            arm=spaces.Box(self.arm.bounds[:, 0], self.arm.bounds[:, 1], dtype=np.float64),
            gripper=spaces.Box(0, 2.2028, shape=(1,), dtype=np.float64)))
        self.observation_space = spaces.Dict(dict(joints=spaces.Box(-10, 10, shape=(7,), dtype=np.float64),
            joint_velocity=spaces.Box(-100, 100, shape=(7,), dtype=np.float64),
            wheel_delta=spaces.Box(-100, 100, shape=(3,), dtype=np.float64),
            wheel_velocity=spaces.Box(-100, 100, shape=(3,), dtype=np.float64),
            scan=spaces.Box(0, self.c['lidar']['max_range_m'], shape=(rays,), dtype=np.float64),
            scan_fresh=spaces.Discrete(2), gyro=spaces.Box(-100, 100, shape=(3,), dtype=np.float64),
            gripper=spaces.Box(-.1, 2.4, shape=(1,), dtype=np.float64), time=spaces.Box(0, np.inf, shape=(1,), dtype=np.float64)))
        self.held = None
        self.delivered = {}
        self.pending_deposits = {}
        self.sensor_dropout = False
        self.reset()

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.held, self.delivered, self.pending_deposits = None, {}, {}
        self.sensor_dropout = False
        self.arm_goal = self.arm.check((options or {}).get('arm_q', np.zeros(7))).copy()
        self.data.qpos[self.arm_qadr] = self.arm_goal
        self.data.ctrl[self.arm_aids] = self.arm_goal
        self.data.ctrl[self.gripper_aid] = 0
        self._scan = np.full(len(self.angles), self.c['lidar']['max_range_m'])
        self._scan_time = -np.inf
        self._last_wheels = self.data.qpos[self.wheel_qadr].copy()
        mujoco.mj_forward(self.model, self.data)
        # The initial dock pose is known. Subsequent settling is observed like any motion.
        self._last_wheels = self.data.qpos[self.wheel_qadr].copy()
        return self._observation(), self.diagnostics()

    def ground_truth_pose(self):
        matrix = self.data.xmat[self.base_id].reshape(3, 3)
        return np.r_[self.data.xpos[self.base_id, :2], math.atan2(matrix[1, 0], matrix[0, 0])]

    def arm_position(self): return self.data.qpos[self.arm_qadr].copy()

    def tcp_position(self): return self.data.site_xpos[self.tcp_id].copy()

    def local_tcp(self):
        rotation3 = self.data.xmat[self.arm_root_id].reshape(3, 3)
        return rotation3.T @ (self.tcp_position()-self.data.xpos[self.arm_root_id])

    def _scan_ranges(self):
        position = self.data.site_xpos[self.lidar_id]
        matrix = self.data.site_xmat[self.lidar_id].reshape(3, 3)
        ranges = []
        group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        for angle in self.angles:
            direction = matrix @ np.array([np.cos(angle), np.sin(angle), 0])
            geom = np.zeros(1, dtype=np.int32)
            distance = mujoco.mj_ray(self.model, self.data, position, direction, group, 1, -1, geom)
            ranges.append(self.c['lidar']['max_range_m'] if distance < 0 else min(distance, self.c['lidar']['max_range_m']))
        noisy = np.asarray(ranges)+self.np_random.normal(0, self.c['lidar']['noise_std_m'], len(ranges))
        return np.clip(noisy, 0.03, self.c['lidar']['max_range_m'])

    def _observation(self):
        fresh = self.data.time-self._scan_time >= self.c['lidar']['period_s']-1e-9 and not self.sensor_dropout
        if fresh:
            self._scan, self._scan_time = self._scan_ranges(), self.data.time
        wheels = self.data.qpos[self.wheel_qadr].copy()
        delta = (wheels-self._last_wheels)*self.c['odometry']['wheel_scale']
        delta += self.np_random.normal(0, self.c['odometry']['noise_std_rad'], 3)
        self._last_wheels = wheels
        return dict(joints=self.arm_position(), joint_velocity=self.data.qvel[self.arm_dadr].copy(),
            wheel_delta=delta, wheel_velocity=self.data.qvel[self.wheel_dadr].copy(), scan=self._scan.copy(),
            scan_fresh=int(fresh), gyro=self.data.sensor('gyro').data.copy(),
            gripper=np.array([self.data.qpos[self.model.jnt_qposadr[self.gripper_id]]]), time=np.array([self.data.time]))

    def step(self, action):
        target = self.arm.check(action['arm'])
        grip = float(np.asarray(action.get('gripper', [0])).ravel()[0])
        if not np.isfinite(grip) or not 0 <= grip <= 2.2028: raise ValueError('Consigne pince hors butées')
        speed = min(self.c['arm_velocity_rad_s'], self.arm.c['max_velocity_rad_s'])
        self.arm_goal += np.clip(target-self.arm_goal, -speed*self.c['control_period_s'], speed*self.c['control_period_s'])
        self.arm.check(self.arm_goal)
        self.data.ctrl[self.arm_aids] = self.arm_goal
        self.data.ctrl[self.gripper_aid] = grip
        self.data.ctrl[self.wheel_aids] = self.kinematics.wheels(action['twist'])
        for _ in range(self.substeps): mujoco.mj_step(self.model, self.data)
        for letter_id, name in list(self.pending_deposits.items()):
            body = self.model.body(letter_id).id
            box = next(b for b in self.c['mailboxes'] if b['name'] == name)
            x, y, z = box['front_xyz_m']
            point = self.data.xpos[body]
            if (abs(point[0]-x) < box['slot_width_m']/2 and y+.004 < point[1] < y+box['depth_m']
                    and z-.16 < point[2] < z-box['slot_height_m']/2):
                self.delivered[letter_id] = name
                del self.pending_deposits[letter_id]
        if not np.all(np.isfinite(self.data.qpos)): raise RuntimeError('Divergence MuJoCo')
        observation = self._observation()
        info = self.diagnostics()
        unstable = info['tilt_rad'] > 0.45 or info['height_m'] < .025
        truncated = self.data.time >= self.c['task']['max_time_s']
        return observation, float(len(self.delivered)), unstable, truncated, info

    def grasp(self, letter_id):
        if self.held is not None: return False
        body = self.model.body(letter_id).id
        grasp_point = self.data.xpos[body]-self.data.xmat[body].reshape(3, 3) @ self.c['letter_center_from_tcp_m']
        if np.linalg.norm(grasp_point-self.tcp_position()) > .018: return False
        if self.data.qpos[self.model.jnt_qposadr[self.gripper_id]] < .3: return False
        frame = self.model.body('grasp_frame').id
        r = self.data.xmat[frame].reshape(3, 3)
        relative_pos = r.T @ (self.data.xpos[body]-self.data.xpos[frame])
        relative_rot = r.T @ self.data.xmat[body].reshape(3, 3)
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, relative_rot.ravel())
        eq = self.model.equality(f'{letter_id}_grasp').id
        self.model.eq_data[eq, 3:6], self.model.eq_data[eq, 6:10] = relative_pos, quaternion
        self.data.eq_active[self.model.equality(f'{letter_id}_storage').id] = False
        self.data.eq_active[eq] = True
        self.held = letter_id
        return True

    def release(self):
        if self.held is None: return None
        letter_id = self.held
        recipient = next(l['recipient'] for l in self.c['letters'] if l['id'] == letter_id)
        position = self.data.xpos[self.model.body(letter_id).id].copy()
        self.data.eq_active[self.model.equality(f'{letter_id}_grasp').id] = False
        self.held = None
        for box in self.c['mailboxes']:
            x, y, z = box['front_xyz_m']
            if (abs(position[0]-x) < box['slot_width_m']/2-.01 and y+.015 < position[1] < y+box['depth_m']-.01
                    and abs(position[2]-z) < box['slot_height_m']/2):
                if recipient != box['name']: return dict(id=letter_id, accepted=False, reason='wrong_recipient')
                self.pending_deposits[letter_id] = box['name']
                return dict(id=letter_id, accepted=True, mailbox=box['name'])
        return dict(id=letter_id, accepted=False, reason='outside_mailbox')

    def diagnostics(self):
        matrix = self.data.xmat[self.base_id].reshape(3, 3)
        tilt = math.acos(np.clip(matrix[2, 2], -1, 1))
        contacts = []
        for i in range(self.data.ncon):
            hit = self.data.contact[i]
            names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or '' for g in hit.geom]
            if 'floor' in names and any(name.startswith('roller_geom_') for name in names): continue
            if any(name.startswith('letter_') for name in names): continue  # insertion/drop contacts are expected
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            if hit.dist < -0.001 or (hit.dist < .001 and force[0] > 1.0): contacts.append(names)
        return dict(ground_truth_pose=self.ground_truth_pose().tolist(), tilt_rad=tilt,
                    height_m=float(self.data.xpos[self.base_id, 2]), collision_pairs=contacts,
                    delivered=dict(self.delivered), held=self.held)

    def render(self):
        if self.render_mode == 'human':
            raise ValueError('Utiliser le viewer passif dans digital_twin.run --viewer')
        if self.renderer is None: self.renderer = mujoco.Renderer(self.model, height=720, width=960)
        camera = mujoco.MjvCamera()
        camera.lookat = [1.8, 1.45, .2]; camera.distance = 5.5; camera.azimuth = 90; camera.elevation = -50
        self.renderer.update_scene(self.data, camera=camera)
        return self.renderer.render()

    def close(self):
        if self.renderer is not None: self.renderer.close(); self.renderer = None
