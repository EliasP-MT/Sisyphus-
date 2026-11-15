"""Commissioning acceptance tests using simulated telemetry, never a live Station."""
import asyncio
import copy
import unittest
from unittest.mock import patch

import numpy as np

from commission import check_pose, fit_calibration, fit_tcp, jog, jog_samples, nudge_plan, reference_ticks, snapshot
from planner import Arm, joints_to_ticks, load_config
from test_arm import HERE, URDF, calibration, fake_station as base_station


def fake_station(c):
    station = base_station(c)
    station.bus = c['bus']
    return station


def reference(c, q, xyz=None):
    ticks = joints_to_ticks(c, q)
    result = dict(schema='elrobot-reference-v1', bus=c['bus'], known_q_rad=list(q),
                  motors=[dict(motor_id=j['motor_id'], ticks=t,
                               min_ticks=j['min_ticks'], max_ticks=j['max_ticks'])
                          for j, t in zip(c['joints'], ticks)])
    if xyz is not None:
        result['measured_tcp_xyz_m'] = list(xyz)
    return result


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.c = calibration()
        self.references = [reference(self.c, [a]*7) for a in (-0.15, 0.05, 0.2)]

    def test_fit_from_known_poses_recovers_zero_and_sign_without_approving(self):
        config = copy.deepcopy(self.c)
        config['hardware_verified'] = False
        for joint in config['joints']:
            joint.update(zero_ticks=None, sign=None, min_ticks=None, max_ticks=None)
        result = fit_calibration(config, self.references, URDF)
        self.assertFalse(result['hardware_verified'])
        self.assertEqual([j['zero_ticks'] for j in result['joints']], [2048]*7)
        self.assertEqual([j['sign'] for j in result['joints']], [j['sign'] for j in self.c['joints']])
        self.assertEqual(result['joints'][0]['min_ticks'], 120)

    def test_insufficient_or_inconsistent_references_rejected(self):
        for kind in ('duplicate', 'angles', 'bus', 'ids', 'wrap', 'nan'):
            refs = copy.deepcopy(self.references)
            if kind == 'duplicate': refs = [refs[0], refs[0]]
            elif kind == 'angles': refs[0]['known_q_rad'][0] += 0.2
            elif kind == 'bus': refs[1]['bus'] = 'wheel-bus'
            elif kind == 'ids': refs[1]['motors'][0]['motor_id'] = 8
            elif kind == 'wrap': refs[1]['motors'][0]['ticks'] = 3990
            else: refs[0]['known_q_rad'][0] = float('nan')
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                fit_calibration(self.c, refs, URDF)

    def test_reference_outside_calibration_rejected(self):
        self.references[0]['motors'][0]['ticks'] = -1
        with self.assertRaises(ValueError): reference_ticks(self.c, self.references[0])

    def test_independent_pose_detects_bad_calibration_without_approving(self):
        candidate = fit_calibration(self.c, self.references, URDF)
        validation = reference(self.c, [-0.05]*7)
        report = check_pose(candidate, validation, URDF)
        self.assertTrue(report['within_reference_tolerances'])
        self.assertFalse(report['hardware_verified'])
        candidate['joints'][0]['zero_ticks'] += 40
        self.assertFalse(check_pose(candidate, validation, URDF)['within_reference_tolerances'])

    def test_tcp_fit_with_distinct_orientations(self):
        mount = Arm(URDF, self.c)
        offset = np.array([0.01, 0.04, -0.01])
        refs = []
        for a in (-0.2, 0, 0.2):
            q = np.array([a]*7)
            # Ground truth uses the quantized angles actually recorded by the encoders.
            from planner import ticks_to_joints
            actual = ticks_to_joints(self.c, joints_to_ticks(self.c, q))
            pose = mount.fk(actual)
            refs.append(reference(self.c, q, pose[:3, 3]+pose[:3, :3] @ offset))
        result = fit_tcp(self.c, refs, URDF)
        np.testing.assert_allclose(result['tcp_xyz_m'], offset, atol=1e-9)
        self.assertFalse(result['hardware_verified'])
        refs[0]['measured_tcp_xyz_m'][0] += 0.04
        with self.assertRaises(ValueError): fit_tcp(self.c, refs, URDF)
        with self.assertRaises(ValueError): fit_tcp(self.c, [refs[1]]*3, URDF)


class JogTests(unittest.TestCase):
    def test_jog_profile_is_bounded_and_finishes_at_target(self):
        for delta in (-16, 8, 16):
            samples = jog_samples(2048, delta)
            self.assertEqual(samples[0], (0, 2048))
            self.assertEqual(samples[-1][1], 2048+delta)
            self.assertTrue(all(2048+min(0, delta) <= p <= 2048+max(0, delta) for _, p in samples))
        for delta in (0, 17, -17, 1.5):
            with self.assertRaises(ValueError): jog_samples(2048, delta)

    def test_preview_has_no_writes(self):
        c = calibration()
        station = fake_station(c)
        result = asyncio.run(jog(station, c['joints'][0], 8, False))
        self.assertFalse(result['executed'])
        self.assertEqual(station.writes, [])

    def test_live_jog_only_writes_selected_id_and_never_torque(self):
        c = calibration()
        station = fake_station(c)
        writes = []

        async def write(writer, address, values, size):
            writes.append((writer.ids[:], address, values))
            station.received = __import__('time').monotonic()
            if address == 0x2a:
                for mid, val in zip(writer.ids, values):
                    station.motors[mid].raw[0x38:0x3a] = val.to_bytes(2, 'little')

        # A real method is needed so copy.copy binds the selected writer's IDs.
        from station import Station
        del station.write
        with patch.object(Station, 'write', write):
            result = asyncio.run(jog(station, c['joints'][2], 8, True))
        self.assertTrue(result['executed'])
        self.assertTrue(all(ids == [3] and address in (0x2e, 0x29, 0x2a)
                            for ids, address, _ in writes))
        self.assertEqual(snapshot(station)['motors'][0]['ticks'], 2048)

    def test_no_write_on_torque_or_range_failure(self):
        c = calibration()
        for issue in ('torque', 'limit'):
            station = fake_station(c)
            if issue == 'torque': station.motors[1].raw[0x28] = 0
            else: station.motors[1].raw[0x38:0x3a] = (3990).to_bytes(2, 'little')
            with self.subTest(issue=issue), self.assertRaises((ValueError, RuntimeError)):
                asyncio.run(jog(station, c['joints'][0], 8, True))
            self.assertEqual(station.writes, [])


class NudgeTests(unittest.TestCase):
    def test_nudge_plan_and_size_limits(self):
        arm = Arm(URDF, load_config(HERE/'elrobot.example.json'))
        for delta in ([0, 0, 0], [6, 0, 0], [float('nan'), 0, 0]):
            with self.assertRaises(ValueError): nudge_plan(arm, np.zeros(7), delta)
        # A regular pose avoids the large joint movements of a near singular pose.
        start = np.array([0.1, -0.3, 0.5, -0.3, 0.2, 0.3, -0.1])
        plan = nudge_plan(arm, start, [2, 0, 0])
        expected = arm.fk(start)[:3, 3]+[0.002, 0, 0]
        self.assertLess(np.linalg.norm(np.array(plan['final_xyz_m'])-expected), 0.002)


if __name__ == '__main__':
    unittest.main()
