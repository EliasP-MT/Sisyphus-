"""ROS integration test of the robot core on the simulated serial bus (sim://).

Run inside the Humble container (separate domain, no hardware):
  ROS_DOMAIN_ID=77 bash ros2_robot/deploy/station_docker.sh \
      python3 /postal/src/postal_robot/tests_ros/test_core_sim.py
"""
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from postal_robot_interfaces.action import ExecuteTrajectory
from postal_robot_interfaces.msg import Heartbeat, MotorStateArray, SupervisorState
from postal_robot_interfaces.srv import (Acknowledge, CapturePose, ListPoses, RunDispenser,
                                         SetGripper, SetTorque)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tests'))
from fixtures import calibration, calibration_data  # noqa: E402

JOINTS = [f'rev_motor_{i:02d}' for i in range(1, 8)]
BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class Harness(Node):
    def __init__(self):
        super().__init__('core_test')
        self.state = self.motors = None
        self.beating, self.session, self.seq = False, 'session-A', 0
        self.create_subscription(SupervisorState, '/robot/supervisor/state', self._state, 10)
        self.create_subscription(MotorStateArray, '/robot/motors', self._motors, 10)
        self.hb = self.create_publisher(Heartbeat, '/station/heartbeat', BEST_EFFORT)
        self.cmd = self.create_publisher(Twist, '/robot/base/cmd_vel', BEST_EFFORT)
        self.twist = None
        self.create_timer(.1, self._tick)
        self.srv = {name: self.create_client(t, f'/robot/{name}') for name, t in [
            ('supervisor/acknowledge', Acknowledge), ('set_torque', SetTorque),
            ('dispenser/run', RunDispenser), ('dispenser/stop', RunDispenser),
            ('gripper/set', SetGripper), ('poses/capture', CapturePose), ('poses/list', ListPoses)]}
        self.action = ActionClient(self, ExecuteTrajectory, '/robot/arm/execute_trajectory')

    def _state(self, m):
        self.state = m

    def _motors(self, m):
        self.motors = {s.motor_id: s for s in m.motors}

    def _tick(self):
        if self.beating:
            self.seq += 1
            self.hb.publish(Heartbeat(session_id=self.session, sequence=self.seq))
        if self.twist is not None:
            self.cmd.publish(self.twist)

    def call(self, service, **fields):
        client = self.srv[service]
        assert client.wait_for_service(10), service
        request = client.srv_type.Request(**fields)
        future = client.call_async(request)
        deadline = time.time() + 10
        while not future.done() and time.time() < deadline:
            time.sleep(.02)
        return future.result()


def wait(predicate, timeout=5.):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if predicate():
                return True
        except (AttributeError, KeyError, TypeError):
            pass
        time.sleep(.05)
    return False


def trajectory(start, joint=0, delta=.05, duration=1., dt=.02):
    msg = JointTrajectory(joint_names=JOINTS)
    n = int(duration / dt)
    for k in range(n + 1):
        q = list(start)
        q[joint] += delta * k / n
        t = k * dt
        msg.points.append(JointTrajectoryPoint(positions=q, time_from_start=Duration(
            sec=int(t), nanosec=int(round((t % 1) * 1e9)))))
    return msg


class CoreSimTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.TemporaryDirectory()
        d = Path(cls.dir.name)
        (d / 'cal.json').write_text(json.dumps(calibration_data()))
        (d / 'poses.json').write_text(json.dumps({
            'schema': 'encoder-poses-v1', 'bus': '5B8E114718', 'model_calibrated': False,
            'poses': {'legacy': {'recorded_at': 'x', 'ticks': {str(i): 2000 for i in range(1, 9)}}}}))
        cls.core = subprocess.Popen(
            ['ros2', 'run', 'postal_robot', 'core', '--ros-args', '-r', '__ns:=/robot',
             '-p', 'device:=sim://', '-p', f'calibration:={d}/cal.json', '-p', f'poses:={d}/poses.json',
             '-p', 'lease_timeout_s:=0.8'], stdout=open(d / 'core.log', 'w'), stderr=subprocess.STDOUT)
        rclpy.init()
        cls.h = Harness()
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.h)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        assert wait(lambda: cls.h.state is not None and cls.h.motors and cls.h.motors[12].fresh, 20), \
            (d / 'core.log').read_text()
        cls.cal = calibration()

    @classmethod
    def tearDownClass(cls):
        cls.core.terminate()
        cls.core.wait(10)
        print((Path(cls.dir.name) / 'core.log').read_text()[-3000:])
        cls.executor.shutdown()
        rclpy.shutdown()

    def measured(self):
        return [self.cal.to_radians(n, self.h.motors[i].position_ticks) for n, i in zip(JOINTS, range(1, 8))]

    def activate(self):
        self.h.beating = True
        self.assertTrue(wait(lambda: self.h.state.state_name in ('AWAITING_ACK', 'ACTIVE')))
        if self.h.state.state_name != 'ACTIVE':
            r = self.h.call('supervisor/acknowledge', session_id=self.h.session)
            self.assertTrue(r.success, r.message)
        self.assertTrue(wait(lambda: self.h.state.state_name == 'ACTIVE'))

    def execute(self, traj, checked=True, wait_result=True):
        self.assertTrue(self.h.action.wait_for_server(10))
        goal = ExecuteTrajectory.Goal(trajectory=traj, plan_id='t', collision_checked=checked,
                                      collision_scene_sha256='abc' if checked else '')
        f = self.h.action.send_goal_async(goal)
        self.assertTrue(wait(f.done, 10))
        handle = f.result()
        if not wait_result:
            return handle
        if not handle.accepted:
            return None
        rf = handle.get_result_async()
        self.assertTrue(wait(rf.done, 30))
        return rf.result().result

    def test_1_waiting_refuses_commands(self):
        s = self.h.state
        self.assertEqual(s.state_name, 'WAITING_FOR_STATION')
        self.assertIn('SIMULATED', s.warnings[0])
        r = self.h.call('set_torque', group='arm', enable=True)
        self.assertFalse(r.success)
        self.assertIn('acknowledge', r.message)
        self.h.beating = True
        self.assertTrue(wait(lambda: self.h.state.state_name == 'AWAITING_ACK'))
        self.assertFalse(self.h.call('supervisor/acknowledge', session_id='other').success)
        self.assertFalse(self.h.call('set_torque', group='arm', enable=True).success)

    def test_2_poses(self):
        r = self.h.call('poses/list')
        self.assertTrue(r.success, r.message)
        self.assertEqual([p.name for p in r.poses], ['legacy'])
        r = self.h.call('poses/capture', name='sim_rest', replace=False)
        self.assertTrue(r.success, r.message)
        self.assertEqual(list(r.pose.ticks)[:7], [2075, 2065, 2024, 2045, 1888, 1910, 2240])
        self.assertFalse(self.h.call('poses/capture', name='sim_rest', replace=False).success)

    def test_3_arm_trajectory(self):
        self.activate()
        result = self.execute(trajectory(self.measured()))
        self.assertTrue(result is None or result.error_code == ExecuteTrajectory.Result.REJECTED)
        r = self.h.call('set_torque', group='arm', enable=True)
        self.assertTrue(r.success, r.message)
        self.assertTrue(wait(lambda: all(self.h.motors[i].torque_enabled for i in range(1, 9))))
        result = self.execute(trajectory(self.measured()), checked=False)
        self.assertEqual(result.error_code, ExecuteTrajectory.Result.REJECTED)
        self.assertIn('collision', result.message)
        bad = trajectory(self.measured(), joint=1, delta=-1.7, duration=22.)
        result = self.execute(bad)
        self.assertEqual(result.error_code, ExecuteTrajectory.Result.REJECTED)
        self.assertIn('outside', result.message)
        start = self.measured()
        result = self.execute(trajectory(start, delta=.05, duration=1.))
        self.assertEqual(result.error_code, ExecuteTrajectory.Result.SUCCEEDED, result.message)
        self.assertAlmostEqual(self.measured()[0], start[0] + .05, delta=.011)

    def test_4_concurrent_and_cancel(self):
        self.activate()
        first = self.execute(trajectory(self.measured(), delta=-.3, duration=4.), wait_result=False)
        self.assertTrue(first.accepted)
        self.assertTrue(wait(lambda: self.h.state.arm_busy))
        second = self.execute(trajectory(self.measured(), delta=.01, duration=1.), wait_result=False)
        self.assertFalse(second.accepted)
        time.sleep(1.)
        cf = first.cancel_goal_async()
        self.assertTrue(wait(cf.done))
        rf = first.get_result_async()
        self.assertTrue(wait(rf.done, 10))
        self.assertEqual(rf.result().result.error_code, ExecuteTrajectory.Result.CANCELED)
        time.sleep(.5)
        before = self.measured()
        time.sleep(1.)
        self.assertLess(max(abs(a - b) for a, b in zip(before, self.measured())), .005, 'arm must be held')

    def test_5_dispenser(self):
        self.activate()
        r = self.h.call('dispenser/run', duration_s=1.)
        self.assertFalse(r.success)  # torque off in the simulated servo
        self.assertTrue(self.h.call('set_torque', group='dispenser', enable=True).success)
        for d in (0., -1., 11., float('nan')):
            self.assertFalse(self.h.call('dispenser/run', duration_s=d).success, d)
        r = self.h.call('dispenser/run', duration_s=1.)
        self.assertTrue(r.success, r.message)
        self.assertTrue(wait(lambda: self.h.motors[10].speed_ticks_s == -1000, 2))
        self.assertTrue(wait(lambda: self.h.motors[10].speed_ticks_s == 0, 2.5))
        self.assertFalse(self.h.state.dispenser_running)

    def test_6_base_deadman(self):
        self.activate()
        self.assertTrue(self.h.call('set_torque', group='base', enable=True).success)
        self.h.twist = Twist()
        self.h.twist.linear.x = .03
        self.assertTrue(wait(lambda: any(self.h.motors[i].speed_ticks_s for i in (9, 11, 12)), 2))
        self.h.twist = None
        self.assertTrue(wait(lambda: all(self.h.motors[i].speed_ticks_s == 0 for i in (9, 11, 12)), 1.5))

    def test_7_link_loss_and_explicit_resume(self):
        self.activate()
        self.assertTrue(self.h.call('dispenser/run', duration_s=8.).success)
        self.h.twist = Twist()
        self.h.twist.angular.z = .2
        handle = self.execute(trajectory(self.measured(), delta=.3, duration=4.), wait_result=False)
        self.assertTrue(handle.accepted)
        self.assertTrue(wait(lambda: any(self.h.motors[i].speed_ticks_s for i in (9, 11, 12)), 2))
        time.sleep(.5)
        lost_at = time.time()
        self.h.beating = False
        self.assertTrue(wait(lambda: self.h.state.state_name == 'LINK_LOST', 3))
        self.assertLess(time.time() - lost_at, 2.)
        self.assertTrue(wait(lambda: all(self.h.motors[i].speed_ticks_s == 0 for i in (9, 10, 11, 12)), 1.5))
        rf = handle.get_result_async()
        self.assertTrue(wait(rf.done, 5))
        self.assertEqual(rf.result().result.error_code, ExecuteTrajectory.Result.ABORTED)
        held = self.measured()
        time.sleep(1.)
        self.assertLess(max(abs(a - b) for a, b in zip(held, self.measured())), .005)
        self.assertTrue(all(self.h.motors[i].torque_enabled for i in range(1, 9)), 'arm torque kept')
        self.h.beating = True  # cmd_vel still streaming: must NOT move before acknowledgement
        self.assertTrue(wait(lambda: self.h.state.state_name == 'AWAITING_ACK', 3))
        time.sleep(1.)
        self.assertTrue(all(self.h.motors[i].speed_ticks_s == 0 for i in (9, 11, 12)))
        self.assertFalse(self.h.state.dispenser_running)
        self.assertTrue(self.h.call('supervisor/acknowledge', session_id=self.h.session).success)
        self.assertTrue(wait(lambda: any(self.h.motors[i].speed_ticks_s for i in (9, 11, 12)), 2))
        self.h.twist = None
        self.assertTrue(wait(lambda: all(self.h.motors[i].speed_ticks_s == 0 for i in (9, 11, 12)), 1.5))

    def test_8_new_session_requires_ack(self):
        self.activate()
        self.h.session = 'session-B'
        self.assertTrue(wait(lambda: self.h.state.state_name == 'AWAITING_ACK', 2))
        self.assertTrue(self.h.call('supervisor/acknowledge', session_id='session-B').success)

    def test_9_gripper_range(self):
        self.activate()
        self.assertFalse(self.h.call('gripper/set', ticks=100).success)
        r = self.h.call('gripper/set', ticks=1600)
        self.assertTrue(r.success, r.message)
        self.assertTrue(wait(lambda: abs(self.h.motors[8].position_ticks - 1600) <= 2, 5))


if __name__ == '__main__':
    unittest.main(verbosity=2)
