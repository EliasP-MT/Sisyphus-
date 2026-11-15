"""MuJoCo hardware-shape, sensor, control and software contract tests, no hardware."""
import copy
import json
import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np
from gymnasium.utils.env_checker import check_env
from scipy.spatial.transform import Rotation

from .common import KiwiKinematics, ROOT, config, relative
from .env import PostalEnv
from .model import arm_model
from .slam import RangeSlam


class PlantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.env = PostalEnv()
    @classmethod
    def tearDownClass(cls): cls.env.close()
    def setUp(self): self.env.reset(seed=11)

    def test_gymnasium_observation_and_determinism(self):
        check_env(self.env, skip_render_check=True)

    def test_joint_stops_and_motor_effort_come_from_elrobot(self):
        env = self.env
        for i, joint in enumerate(env.joint_ids):
            np.testing.assert_allclose(env.model.jnt_range[joint], env.arm.bounds[i], atol=1e-9)
            np.testing.assert_allclose(env.model.actuator_ctrlrange[env.arm_aids[i]], env.arm.command_bounds[i])
            np.testing.assert_allclose(env.model.actuator_forcerange[env.arm_aids[i]], [-2.94, 2.94])
        with self.assertRaises(ValueError):
            env.step(dict(twist=[0, 0, 0], arm=env.arm.bounds[:, 1]+.1, gripper=[0]))
        with self.assertRaises(ValueError):
            env.step(dict(twist=[0, 0, 0], arm=np.zeros(7), gripper=[3]))

    def test_fk_matches_the_existing_arm_planner(self):
        env = self.env
        q = np.array([.1, -.3, .5, -.3, .2, .3, -.1])
        env.reset(options={'arm_q': q})
        np.testing.assert_allclose(env.local_tcp(), env.arm.fk(q)[:3, 3], atol=2e-6)

    def test_three_holonomic_axes_are_physically_independent(self):
        env = self.env
        for twist in ([.1, 0, 0], [0, .1, 0], [0, 0, .4]):
            with self.subTest(twist=twist):
                env.reset(seed=1)
                start = env.ground_truth_pose()
                for _ in range(40):
                    _, _, terminated, _, info = env.step(dict(twist=twist, arm=np.zeros(7), gripper=[0]))
                    self.assertFalse(terminated)
                    self.assertEqual(info['collision_pairs'], [])
                change = relative(start, env.ground_truth_pose())
                np.testing.assert_allclose(change, np.array(twist)*2, atol=.03)

    def test_gripper_mimic_joints_follow_the_elrobot_gear(self):
        env = self.env
        for _ in range(20): env.step(dict(twist=[0, 0, 0], arm=np.zeros(7), gripper=[1.0]))
        gear = env.data.qpos[env.model.jnt_qposadr[env.gripper_id]]
        for name, factor in [('rev_motor_08_1', -.0115), ('rev_motor_08_2', .0115)]:
            value = env.data.qpos[env.model.jnt_qposadr[env.model.joint(name).id]]
            self.assertAlmostEqual(value, gear*factor, delta=.0003)

    def test_lidar_is_a_range_sensor_and_observation_hides_truth(self):
        env = self.env
        observation, _ = env.reset(seed=1)
        self.assertNotIn('pose', observation)
        self.assertNotIn('ground_truth_pose', observation)
        self.assertEqual(len(observation['scan']), 181)
        self.assertAlmostEqual(observation['scan'][0], .65, delta=.015)
        index = np.argmin(np.abs(env.angles))
        self.assertAlmostEqual(observation['scan'][index], (3.6-.65)/np.cos(env.angles[index]), delta=.015)
        env.sensor_dropout = True
        next_observation, *_ = env.step(dict(twist=[0, 0, 0], arm=np.zeros(7), gripper=[0]))
        self.assertFalse(next_observation['scan_fresh'])

    def test_cad_decomposition_keeps_backpack_open(self):
        env = self.env
        # Known free point in solid 8's cavity, from the exact STEP model.
        local = Rotation.from_euler('xyz', env.c['cad_rpy_rad']).apply([-.13, .02, .08])+env.c['cad_xyz_m']
        base_rotation = env.data.xmat[env.base_id].reshape(3, 3)
        world = env.data.xpos[env.base_id]+base_rotation @ local
        ids = np.flatnonzero(env.model.geom_group == 3)
        def ray(direction):
            hits = [mujoco.mj_rayMesh(env.model, env.data, int(g), world, base_rotation @ direction) for g in ids]
            return min([x for x in hits if x >= 0], default=-1)
        self.assertLess(ray([0, 0, 1]), 0, 'Une enveloppe pleine a fermé la cavité du sac')
        distance = ray([0, 0, -1])
        self.assertGreater(distance, 0)
        self.assertLess(distance, .05)

    def test_grasp_requires_nearby_letter_and_closed_gripper(self):
        env = self.env
        self.assertFalse(env.grasp('letter_0'))
        env.reset(seed=1, options={'arm_q': env.pickup_q})
        self.assertFalse(env.grasp('letter_0'))
        for _ in range(20): env.step(dict(twist=[0, 0, 0], arm=env.pickup_q, gripper=[1.0]))
        before = env.data.xpos[env.model.body('letter_0').id].copy()
        self.assertTrue(env.grasp('letter_0'))
        np.testing.assert_allclose(env.data.xpos[env.model.body('letter_0').id], before)
        self.assertFalse(env.release()['accepted'])
        self.assertEqual(env.delivered, {})


class AlgorithmTests(unittest.TestCase):
    def test_kiwi_matrix_and_saturation_preserve_full_body_twist(self):
        k = KiwiKinematics(config())
        twist = np.array([.08, -.06, .2])
        np.testing.assert_allclose(k.body(k.wheels(twist)), twist)
        large = np.array([2., 3., 4.])
        measured = k.body(k.wheels(large))
        np.testing.assert_allclose(measured/large, np.full(3, measured[0]/large[0]))
        self.assertLessEqual(np.max(np.abs(k.wheels(large))), k.limit)

    def test_verified_encoder_ranges_tighten_simulation_limits(self):
        settings = json.loads((ROOT/'arm_control/elrobot.example.json').read_text())
        settings.update(hardware_verified=True, bus='synthetic-test-bus')
        for joint in settings['joints']:
            joint.update(zero_ticks=2048, sign=1, min_ticks=1600, max_ticks=2500)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'calibration.json'; path.write_text(json.dumps(settings))
            arm = arm_model(config(), path)
        expected = (np.array([1600, 2500])-2048)*(2*np.pi/4096)
        np.testing.assert_allclose(arm.bounds, np.tile(expected, (7, 1)))

    def test_slam_does_not_require_world_geometry_or_ground_truth(self):
        c = config()
        for key in ('room_size_m', 'obstacles', 'mailboxes', 'letters', 'cad_xyz_m', 'cad_rpy_rad'):
            del c[key]
        slam = RangeSlam(c)
        ranges = np.full(c['lidar']['rays'], 2.0)
        observation = dict(wheel_delta=np.zeros(3), scan=ranges, scan_fresh=1, time=np.array([0.]))
        slam.update(observation)
        observation.update(wheel_delta=slam.kinematics.wheels([0, .1, 0])*.1, scan_fresh=0)
        pose = slam.update(observation)
        self.assertGreater(pose[1], c['start_pose'][1]+.009)
        self.assertGreater(np.count_nonzero(slam.map.logodds), 0)


if __name__ == '__main__': unittest.main()
