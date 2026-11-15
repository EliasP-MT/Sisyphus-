"""Real STEP, URDF, Bullet and OMPL acceptance tests, without a Station connection."""
import copy
import json
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from cad import read_solids, sha256
from planner import Arm, load_config
from register_scene import register
from test_arm import HERE, URDF, calibration, collision_scene


class CollisionTests(unittest.TestCase):
    def setUp(self):
        self.arm = Arm(URDF, load_config(HERE/'elrobot.example.json'))

    def cad_scene(self):
        cad = json.loads((HERE/'robot_scene.example.json').read_text())['cad']
        cad['manifest'] = str(HERE/cad['manifest'])
        return {'cad': cad}

    def obstacle(self):
        return dict(name='obstacle-test', size_m=[0.04, 0.04, 0.06],
                    xyz_m=[0.003, 0.24, 0.22], rpy_rad=[0, 0, 0])

    def test_supplied_step_and_converted_units(self):
        source = HERE/'assets/full_robot.step'
        manifest = json.loads((HERE/'assets/full_robot_collision/manifest.json').read_text())
        self.assertEqual(sha256(source), 'cf0028e27277768bb504c611d7a2f959249d54ea7a5f956b3a922118bdbae0d8')
        self.assertEqual(manifest['source_sha256'], sha256(source))
        self.assertEqual(len(read_solids(source)), 9)
        bounds = np.array([s['bounds_m'] for s in manifest['solids']])
        size = np.max(bounds[:, 3:], axis=0)-np.min(bounds[:, :3], axis=0)
        np.testing.assert_allclose(size, [0.144, 0.211, 0.103], atol=0.0005)

    def test_fk_consistency_and_real_cad_collision(self):
        with collision_scene(self.arm, self.cad_scene()) as world:
            q = np.array([0.1, -0.3, 0.5, -0.3, 0.2, 0.3, -0.1])
            self.assertIsNone(world.collision(q))
            np.testing.assert_allclose(world._pose(world.links['Gripper_Base_v1_1']), self.arm.fk(q), atol=2e-6)
            preview = world.scene_path.parent/'preview.png'
            world.save_preview(preview, q)
            self.assertEqual(preview.read_bytes()[:8], b'\x89PNG\r\n\x1a\n')
            collision = world.collision([-1.052375, -1.611194, -0.99814, -0.462768, -2.608102, -0.770911, 2.679367])
            self.assertIn('cad_solid', collision)

    def test_closed_solid_contains_material_but_preserves_hollow_space(self):
        with collision_scene(self.arm, self.cad_scene()) as world:
            world.collision(np.zeros(7))
            body = world.cad_bodies[8]
            self.assertFalse(world._inside(body, [-0.13, 0.02, 0.08]))
            self.assertTrue(world._inside(body, [-0.1102773723, -0.025693967, 0.0593516411]))

    def test_self_collision_is_checked(self):
        with collision_scene(self.arm) as world:
            self.assertIsNone(world.collision(np.zeros(7)))
            q = [1.06, -0.24, 1.609, 1.141, -0.634, 0.431, 1.269]
            self.assertIn('auto-collision', world.collision(q))

    def test_middle_of_edge_and_colliding_endpoint_are_rejected(self):
        with collision_scene(self.arm, {'boxes': [self.obstacle()]}) as world:
            start = np.array([-0.6, 0, 0, 0, 0, 0, 0])
            goal = -start
            self.assertTrue(world.valid(start))
            self.assertTrue(world.valid(goal))
            self.assertFalse(world.motion_valid(start, goal))
            with self.assertRaisesRegex(ValueError, 'Arrivée en collision'):
                world.joint_path(start, np.zeros(7))

    def test_rrt_finds_detour_and_timed_motion_is_valid(self):
        from ompl import util as ou
        ou.RNG.setSeed(7)
        with collision_scene(self.arm, {'boxes': [self.obstacle()]}) as world:
            start = np.array([-0.6, 0, 0, 0, 0, 0, 0])
            goal = -start
            path = world.joint_path(start, goal)
            self.assertGreater(len(path), 2)
            np.testing.assert_allclose(path[0], start)
            np.testing.assert_allclose(path[-1], goal)
            samples = []
            for a, b in zip(path, path[1:]):
                samples.extend(self.arm.segment(a, b))
            world.validate_plan(dict(collision_scene_sha256=world.fingerprint, samples=samples))

    def test_payload_is_part_of_collision_model(self):
        payload = dict(name='letter', size_m=[0.08, 0.10, 0.001], xyz_m=[0, 0.2, 0],
                       rpy_rad=[0, 0, 0], ignore_robot_links=[])
        obstacle = dict(name='letter-obstacle', size_m=[0.01]*3,
                        xyz_m=[0.002925, 0.459566, 0.218892], rpy_rad=[0, 0, 0])
        with collision_scene(self.arm, {'boxes': [obstacle]}) as world:
            self.assertIsNone(world.collision(np.zeros(7)))
        with collision_scene(self.arm, {'boxes': [obstacle], 'payload': payload}) as world:
            self.assertIn('lettre tenue', world.collision(np.zeros(7)))

    def test_attached_cad_follows_mobile_link(self):
        scene = self.cad_scene()
        scene['cad']['bindings'] = [dict(solid_ids=[0, 1, 2, 4, 5, 6, 7, 8], parent_link='base_link'),
                                  dict(solid_ids=[3], parent_link='Gripper_Base_v1_1')]
        with collision_scene(self.arm, scene) as world:
            world.collision(np.zeros(7))
            body = next(b for b in world.cad_bodies if b['name'] == 'cad_solid_3')
            original = body['pose'].copy()
            q = np.array([0.3, 0, 0, 0, 0, 0, 0])
            world.collision(q)
            expected = self.arm.fk(q) @ np.linalg.inv(self.arm.fk(np.zeros(7))) @ original
            np.testing.assert_allclose(body['pose'], expected, atol=2e-6)

    def test_scene_requires_registration_and_rejects_changed_files(self):
        with collision_scene(self.arm, {'hardware_verified': False}) as world:
            with self.assertRaises(ValueError): world.require_hardware_ready()
        with collision_scene(self.arm) as world:
            world.require_hardware_ready()
            world.scene_path.write_text('{}')
            with self.assertRaises(ValueError): world.require_hardware_ready()
        with collision_scene(self.arm) as world:
            self.arm.c['tcp_xyz_m'] = [0, 0.01, 0]
            with self.assertRaises(ValueError): world.require_hardware_ready()


class RegistrationTests(unittest.TestCase):
    def test_registration_recovers_transform_without_approving_hardware(self):
        scene = json.loads((HERE/'robot_scene.example.json').read_text())
        points = np.array([[0, 0, 0], [100, 0, 0], [0, 150, 0], [0, 0, 80]])
        rotation = Rotation.from_euler('xyz', [0.1, -0.2, 0.3])
        translation = np.array([0.2, 0.1, 0.05])
        base = rotation.apply(points/1000)+translation
        measurement = dict(cad_units='mm', cad_points=points.tolist(), base_points_m=base.tolist())
        result = register(scene, measurement)
        np.testing.assert_allclose(result['cad']['xyz_m'], translation, atol=1e-8)
        np.testing.assert_allclose(Rotation.from_euler('xyz', result['cad']['rpy_rad']).as_matrix(), rotation.as_matrix())
        self.assertFalse(result['hardware_verified'])
        bad = copy.deepcopy(measurement)
        bad['base_points_m'][0][0] += 0.1
        with self.assertRaises(ValueError): register(scene, bad)
        with self.assertRaises(ValueError): register(scene, dict(cad_units='m', cad_points=[[0, 0, 0]]*3, base_points_m=[[0, 0, 0]]*3))


if __name__ == '__main__':
    unittest.main()
