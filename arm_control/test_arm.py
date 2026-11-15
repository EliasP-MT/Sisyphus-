"""Offline planner acceptance tests (execution is tested in ros2_robot); ARM_URDF selects the unmodified official model."""
import copy
import json
import os
import sys
import time
import tempfile
import unittest
from contextlib import contextmanager
from pathlib import Path

import numpy as np

from planner import Arm, JOINTS, joints_to_ticks, load_config, require_calibration, ticks_to_joints

HERE = Path(__file__).resolve().parent
URDF = Path(os.environ.get('ARM_URDF', HERE/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'))


def calibration():
    c = load_config(HERE/'elrobot.example.json')
    c.update(hardware_verified=True, bus='test-arm-bus')
    for i, j in enumerate(c['joints']):
        j.update(zero_ticks=2048, sign=(-1 if i % 2 else 1), min_ticks=100, max_ticks=3995)
    return c


@contextmanager
def collision_scene(arm, changes=None):
    from collision import CollisionWorld
    scene = dict(version=1, frame='base_link', hardware_verified=True, margin_m=0.005,
                 self_margin_m=0.001, edge_step_rad=0.005, planning_time_s=5.0,
                 gripper_angle_rad=0, cad=None, boxes=[], payload=None, allowed_self_pairs=[])
    scene.update(changes or {})
    with tempfile.TemporaryDirectory(prefix='elrobot-scene-test-') as directory:
        path = Path(directory)/'scene.json'
        path.write_text(json.dumps(scene))
        with CollisionWorld(URDF, arm, path) as world:
            arm.collision = world
            try:
                yield world
            finally:
                arm.collision = None


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not URDF.is_file():
            raise RuntimeError(f'URDF officiel absent : {URDF}. Lancer setup.sh ou définir ARM_URDF.')
        cls.c = load_config(HERE/'elrobot.example.json')
        cls.arm = Arm(URDF, cls.c)

    def test_only_seven_arm_joints_and_expected_zero_fk(self):
        names = [link.name for link, active in zip(self.arm.chain.links, self.arm.chain.active_links_mask) if active]
        self.assertEqual(names, JOINTS)
        self.assertNotIn('rev_motor_08', [link.name for link in self.arm.chain.links])
        np.testing.assert_allclose(self.arm.fk([0]*7)[:3, 3], [0.002925, 0.259566, 0.218892], atol=1e-8)

    def test_tcp_offset_in_tool_frame(self):
        c = copy.deepcopy(self.c)
        c['tcp_xyz_m'] = [0.01, 0.02, 0.03]
        arm = Arm(URDF, c)
        q = [0.2, -0.2, 0.3, -0.1, 0.2, 0.1, -0.2]
        base = self.arm.fk(q)
        np.testing.assert_allclose(arm.fk(q)[:3, 3], base[:3, 3]+base[:3, :3] @ c['tcp_xyz_m'])

    def test_reachable_and_unreachable_ik(self):
        xyz = [0.02, 0.23, 0.25]
        q = self.arm.ik(xyz, [0]*7)
        self.assertLess(np.linalg.norm(self.arm.fk(q)[:3, 3]-xyz), self.c['position_tolerance_m'])
        with self.assertRaises(ValueError): self.arm.ik([2, 0, 0], [0]*7)
        with self.assertRaises(ValueError): self.arm.ik([float('nan'), 0, 0], [0]*7)

    def test_ruckig_limits_and_exact_endpoint(self):
        end = np.array([0.2, -0.1, 0.2, -0.2, 0.1, 0.3, -0.1])
        samples = self.arm.segment(np.zeros(7), end)
        t = np.array([s['t_s'] for s in samples])
        v = np.array([s['v_rad_s'] for s in samples])
        a = np.array([s['a_rad_s2'] for s in samples])
        self.assertTrue(np.all(np.diff(t) > 0))
        self.assertLessEqual(np.max(np.diff(t)), self.c['dt_s']+1e-9)
        self.assertLessEqual(np.max(np.abs(v)), self.c['max_velocity_rad_s']+1e-8)
        self.assertLessEqual(np.max(np.abs(a)), self.c['max_acceleration_rad_s2']+1e-8)
        self.assertLessEqual(np.max(np.abs(np.diff(a, axis=0)/np.diff(t)[:, None])), self.c['max_jerk_rad_s3']+1e-8)
        np.testing.assert_allclose(samples[-1]['q_rad'], end, atol=1e-8)
        np.testing.assert_allclose(v[[0, -1]], 0, atol=1e-8)

    def test_linear_path_preserves_orientation(self):
        origin = self.arm.fk([0]*7)[:3, 3]
        target = origin+np.array([0.006, -0.008, 0.005])
        plan = self.arm.plan([0]*7, [target], mode='linear')
        for sample in plan['samples']:
            pose = self.arm.fk(sample['q_rad'])
            np.testing.assert_allclose(pose[:3, :3], np.eye(3), atol=0.03)
        self.assertLess(np.linalg.norm(np.array(plan['final_xyz_m'])-target), 0.002)
        self.assertFalse(plan['collision_checked'])

    def test_waypoints_and_calibrated_bounds(self):
        plan = self.arm.plan([0]*7, [[0.02, 0.23, 0.25], [0.01, 0.24, 0.24]])
        self.assertLess(np.linalg.norm(np.array(plan['final_xyz_m'])-[0.01, 0.24, 0.24]), 0.002)
        arm = Arm(URDF, calibration())
        with self.assertRaises(ValueError): arm.check([0, 0, 0, 0, 3.2, 0, 0])


if __name__ == '__main__':
    unittest.main()
