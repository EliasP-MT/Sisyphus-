"""Offline acceptance tests; ARM_URDF selects the unmodified official model."""
import asyncio
import copy
import json
import os
import sys
import time
import tempfile
import unittest
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from planner import Arm, JOINTS, joints_to_ticks, load_config, require_calibration, ticks_to_joints
from station import Station, execute, measured

HERE = Path(__file__).resolve().parent
URDF = Path(os.environ.get('ARM_URDF', HERE/'.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'))


def calibration():
    c = load_config(HERE/'elrobot.example.json')
    c.update(hardware_verified=True, bus='test-arm-bus')
    for i, j in enumerate(c['joints']):
        j.update(zero_ticks=2048, sign=(-1 if i % 2 else 1), min_ticks=100, max_ticks=3995)
    return c


class FakeMotor:
    def __init__(self, ticks=2048, torque=1, speed=0, error=False, stale=False):
        self.raw = bytearray(0x46)
        self.raw[0x28] = torque
        self.raw[0x38:0x3a] = ticks.to_bytes(2, 'little')
        self.raw[0x3a:0x3c] = speed.to_bytes(2, 'little')
        self.raw[0x41] = int(error)
        self.stale = stale

    def get_state(self): return self.raw
    def get_system_stamp_ns(self): return time.time_ns()-(1_000_000_000 if self.stale else 1_000_000)
    def get_range_min(self): return 100
    def get_range_max(self): return 3995
    def get_range_freezed(self): return True
    def get_error(self):
        return SimpleNamespace(get_description=lambda: '', get_servo=lambda: 0,
                               get_command_packet=lambda: 0)


def fake_station(c):
    s = Station.__new__(Station)
    s.ids = [j['motor_id'] for j in c['joints']]
    s.motors = {mid: FakeMotor() for mid in s.ids}
    s.received = time.monotonic()
    s.writes = []

    async def write(address, values, size):
        s.received = time.monotonic()
        s.writes.append((address, values, size))
        if address == 0x2a:
            for mid, value in zip(s.ids, values):
                s.motors[mid].raw[0x38:0x3a] = value.to_bytes(2, 'little')
    s.write = write
    return s


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


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.c = calibration()
        self.s = fake_station(self.c)

    def test_mapping_roundtrip_and_signs(self):
        q = np.array([0.2, -0.3, 0.1, -0.1, 0.4, -0.2, 0.1])
        np.testing.assert_allclose(ticks_to_joints(self.c, joints_to_ticks(self.c, q)), q, atol=0.00077)
        self.assertGreater(joints_to_ticks(self.c, q)[1], 2048)

    def test_placeholder_calibration_rejected(self):
        with self.assertRaises(ValueError): require_calibration(load_config(HERE/'elrobot.example.json'))
        self.c['joints'][3]['sign'] = None
        with self.assertRaises(ValueError): joints_to_ticks(self.c, [0]*7)

    def test_no_wrap_or_clamp(self):
        with self.assertRaises(ValueError): joints_to_ticks(self.c, [10]*7)
        with self.assertRaises(ValueError): ticks_to_joints(self.c, [50]*7)
        with self.assertRaises(ValueError): joints_to_ticks(self.c, [float('nan')]*7)

    def test_stale_missing_and_servo_error_rejected(self):
        for issue in ('stale', 'missing', 'error', 'old_frame'):
            s = fake_station(self.c)
            if issue == 'missing': del s.motors[1]
            elif issue == 'old_frame': s.received -= 1
            else: s.motors[1] = FakeMotor(stale=issue == 'stale', error=issue == 'error')
            with self.subTest(issue=issue), self.assertRaises(RuntimeError): s.state()

    def test_torque_motion_and_changed_ranges_rejected(self):
        self.s.motors[1] = FakeMotor(torque=0)
        with self.assertRaises(RuntimeError): measured(self.s, self.c, powered=True)
        self.s.motors[1] = FakeMotor(speed=30)
        with self.assertRaises(RuntimeError): measured(self.s, self.c, stationary=True)
        self.c['joints'][0]['min_ticks'] = 0
        with self.assertRaises(RuntimeError): measured(self.s, self.c)

    def test_signed_encoder_decoding_uses_existing_transport(self):
        self.s.motors[1].raw[0x38:0x3a] = (0x8000 | 1000).to_bytes(2, 'little')
        self.assertEqual(self.s.state()[0][0], 3096)

    def test_sync_write_decodes_with_official_sdk(self):
        sdk = Path(os.environ.get('STATION_SDK', HERE/'.sdk'))
        if not (sdk/'target/gen_python/protobuf/drivers/st3215/st3215.py').is_file():
            self.fail('SDK absent : lancer setup.sh ou définir STATION_SDK')
        sys.path.insert(0, str(sdk))
        from target.gen_python.protobuf.drivers.st3215 import st3215
        from target.gen_python.protobuf.station import commands, drivers
        captured = []

        async def send(client, messages):
            captured.extend(messages)

        s = self.s
        s.client, s.pb, s.commands, s.drivers = None, st3215, commands, drivers
        s.send, s.bus = send, self.c['bus']
        values = [2000+i for i in range(7)]
        asyncio.run(Station.write(s, 0x2a, values, 2))
        self.assertEqual(len(captured), 1)
        self.assertEqual(captured[0].type, drivers.StationCommandType.STC_ST3215_COMMAND)
        decoded = st3215.CommandReader(memoryview(captured[0].body))
        self.assertEqual(decoded.get_target_bus_serial(), self.c['bus'])
        write = decoded.get_sync_write()
        self.assertEqual(write.get_address(), 0x2a)
        self.assertEqual([m.get_motor_id() for m in write.get_motors()], list(range(1, 8)))
        self.assertEqual([int.from_bytes(m.get_value(), 'little') for m in write.get_motors()], values)


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

    def test_executor_sends_only_arm_and_checks_arrival(self):
        c = calibration()
        arm = Arm(URDF, c)
        target = arm.fk(np.array([0.0008]*7))[:3, 3]
        with collision_scene(arm):
            plan = arm.plan([0]*7, [target])
            s = fake_station(c)
            asyncio.run(execute(s, arm, plan))
        self.assertEqual([w[0] for w in s.writes[:2]], [0x2e, 0x29])
        self.assertTrue(all(w[0] in (0x2e, 0x29, 0x2a) and len(w[1]) == 7 for w in s.writes))

    def test_executor_refuses_drift_and_invalid_plan_before_writes(self):
        c = calibration()
        arm = Arm(URDF, c)
        target = arm.fk([0]*7)[:3, 3]
        with collision_scene(arm):
            plan = arm.plan([0]*7, [target])
            s = fake_station(c)
            s.motors[1] = FakeMotor(ticks=2100)
            with self.assertRaises(RuntimeError): asyncio.run(execute(s, arm, plan))
            self.assertEqual(s.writes, [])
            plan['samples'][-1]['q_rad'] = [20]*7
            with self.assertRaises(ValueError): asyncio.run(execute(fake_station(c), arm, plan))

    def test_paced_executor_handles_slow_checks_without_catchup(self):
        c = calibration()
        arm = Arm(URDF, c)
        with collision_scene(arm) as world:
            target = arm.fk(np.array([0.0008]*7))[:3, 3]
            plan = arm.plan([0]*7, [target])
            original = world.require_pose
            def slow_check(*args):
                time.sleep(.15)  # Exceeds the streaming lateness guard.
                return original(*args)
            world.require_pose = slow_check
            s = fake_station(c)
            progress = []
            async def run():
                async def feedback():
                    while True:
                        s.received = time.monotonic()
                        await asyncio.sleep(.01)
                reader = asyncio.create_task(feedback())
                try:
                    await execute(s, arm, plan, paced=True, progress=progress.append)
                finally:
                    reader.cancel()
                    await asyncio.gather(reader, return_exceptions=True)
            asyncio.run(run())
            self.assertTrue(any('paced mode' in p for p in progress))
            self.assertTrue(s.writes)

    def test_stream_period_keeps_exact_final_target(self):
        c=calibration();arm=Arm(URDF,c)
        with collision_scene(arm):
            target=arm.fk(np.array([.0008]*7))[:3,3]
            plan=arm.plan([0]*7,[target]);s=fake_station(c)
            asyncio.run(execute(s,arm,plan,command_period=.06))
            from planner import joints_to_ticks
            self.assertEqual(s.writes[-1][0],0x2a)
            self.assertEqual(s.writes[-1][1],joints_to_ticks(c,plan['samples'][-1]['q_rad']))


if __name__ == '__main__':
    unittest.main()
