"""Robot core: the ONLY owner of the servo bus (arm, gripper, wheels, dispenser).

All serial I/O happens in one control thread. ROS callbacks submit requests
to it and wait for the result, so commands and feedback are never interleaved
by two owners. Supervision (lease, faults, safe stop) runs in the same thread
and does not depend on DDS between robot processes.

Safe stop (lease loss, fault, new station session): wheels -> speed 0 (torque
kept, verified by measured speed), dispenser -> speed 0, active arm trajectory
aborted and the arm held at its last fresh measured position (torque kept).
Nothing restarts automatically: an explicit acknowledgement is required and
only commands issued after it are executed.
"""
import collections
import concurrent.futures
import math
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from postal_robot_interfaces.action import ExecuteTrajectory
from postal_robot_interfaces.msg import (Heartbeat, MotorState, MotorStateArray, PoseInfo,
                                         SupervisorState)
from postal_robot_interfaces.srv import (Acknowledge, CapturePose, DeletePose, ListPoses,
                                         RunDispenser, SetGripper, SetTorque)

from . import st3215
from .bus import Bus, BusError
from .calibration import JOINTS, Calibration
from .kiwi import Kiwi
from .poses import ARM_IDS, PoseError, PoseStore, capture_from_snapshots
from .supervisor import ACTIVE, FAULT, Supervisor
from .trajectory import ABORTED, RUNNING, SUCCEEDED, TrajectoryError, TrajectoryRun

RESPONSE_LEVEL = 0x08


@dataclass
class Motor:
    motor_id: int
    role: str
    expected_mode: int
    mode: int = None
    data: dict = None
    packet_error: int = 0
    received: float = None
    sequence: int = 0
    responding: bool = False
    error: str = 'not read yet'


class Core(Node):
    def __init__(self):
        super().__init__('core', namespace='robot')
        p = self.declare_parameter
        p('device', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E114718-if00')
        p('calibration', '')
        p('poses', '')
        p('period_s', .05)
        p('feedback_max_age_s', .3)
        p('lease_timeout_s', .8)
        p('wheel_ids', [11, 12, 9])                 # left, back, right
        p('wheel_signs', [1, 1, 1])
        p('wheel_radius_m', .05)
        p('base_radius_m', .125)
        p('wheel_mount_deg', [240., 0., 120.])
        p('max_wheel_raw', 1500)
        p('max_linear_m_s', .15)
        p('max_angular_rad_s', .6)
        p('base_direction_verified', False)
        p('cmd_timeout_s', .3)
        p('wheel_stop_speed_raw', 15)
        p('wheel_stop_timeout_s', 1.5)
        p('dispenser_id', 10)
        p('dispenser_default_speed', -1000)
        p('dispenser_max_duration_s', 10.)
        p('gripper_id', 8)
        p('gripper_min_ticks', 1268)
        p('gripper_max_ticks', 2828)
        p('gripper_speed_ticks_s', 200)
        p('start_tolerance_rad', .02)
        p('arrival_tolerance_rad', .01)
        p('arrival_timeout_s', 20.)
        p('max_compensation_rad', .06)
        p('max_speed_factor', 2.)
        p('overload_load_raw', 200)  # raw servo load (0-1000) at which the servo's own protection trips; 0 = guard off
        p('stationary_speed_ticks_s', 10)
        p('publish_odom_tf', False)
        g = lambda name: self.get_parameter(name).value

        self.calibration = Calibration.load(g('calibration'))
        self.device = g('device')
        self.simulated = self.device == 'sim://'
        if not self.simulated and self.calibration.data['bus'] not in self.device:
            raise ValueError(f"Device {self.device} is not calibration bus {self.calibration.data['bus']}")
        self.poses = PoseStore(g('poses'), self.calibration.data['bus'])
        self.period, self.max_age = g('period_s'), g('feedback_max_age_s')
        self.wheel_ids = list(g('wheel_ids'))
        self.dispenser_id, self.gripper_id = g('dispenser_id'), g('gripper_id')
        ids = list(range(1, 8)) + [self.gripper_id] + self.wheel_ids + [self.dispenser_id]
        if len(set(ids)) != 12 or self.gripper_id != 8:
            raise ValueError('Motor IDs must be distinct: arm 1-7, gripper 8, wheels, dispenser')
        roles = {i: 'arm' for i in range(1, 8)}
        roles[self.gripper_id] = 'gripper'
        roles.update(zip(self.wheel_ids, ['wheel_left', 'wheel_back', 'wheel_right']))
        roles[self.dispenser_id] = 'dispenser'
        self.motors = {i: Motor(i, roles[i], st3215.MODE_POSITION if i <= 8 else st3215.MODE_WHEEL)
                       for i in sorted(roles)}
        self.groups = {'arm': list(ARM_IDS), 'base': self.wheel_ids, 'dispenser': [self.dispenser_id]}
        self.verified = g('base_direction_verified')
        self.max_linear = g('max_linear_m_s') if self.verified else min(g('max_linear_m_s'), .05)
        self.max_angular = g('max_angular_rad_s') if self.verified else min(g('max_angular_rad_s'), .3)
        self.kiwi = Kiwi(g('wheel_radius_m'), g('base_radius_m'), g('wheel_mount_deg'),
                         g('wheel_signs'), g('max_wheel_raw'))
        self.cfg = {k: g(k) for k in ('cmd_timeout_s', 'wheel_stop_speed_raw', 'wheel_stop_timeout_s',
                                      'dispenser_default_speed', 'dispenser_max_duration_s',
                                      'gripper_min_ticks', 'gripper_max_ticks', 'gripper_speed_ticks_s',
                                      'start_tolerance_rad', 'arrival_tolerance_rad', 'arrival_timeout_s',
                                      'stationary_speed_ticks_s', 'publish_odom_tf', 'max_compensation_rad', 'max_speed_factor', 'overload_load_raw')}
        if not 0 < abs(self.cfg['dispenser_default_speed']) <= 1000:
            raise ValueError('Dispenser default speed must be within -1000..1000 and non-zero')
        if not 0 < self.cfg['dispenser_max_duration_s'] <= 60:
            raise ValueError('Dispenser maximum duration must be within 0..60 s')
        if not 0 <= self.cfg['gripper_min_ticks'] < self.cfg['gripper_max_ticks'] <= 4095:
            raise ValueError('Invalid gripper range')

        self.supervisor = Supervisor(g('lease_timeout_s'))
        self.lock = threading.RLock()
        self.requests = queue.Queue()
        self.stop_request = None
        self.bus, self.bus_retry = None, 0.
        self.cycle, self.cycle_period = 0, 0.
        self.snapshots = collections.deque(maxlen=200)
        self.cmd, self.cmd_time = (0., 0., 0.), None
        self.wheel_target, self.wheel_written, self.wheel_zero_since = [0, 0, 0], 0., None
        self.dispenser_deadline = None
        self.run = self.run_plan = None
        self.odom = [0., 0., 0.]
        self.warnings, self.last_diag = [], 0.

        cb = ReentrantCallbackGroup()
        self.pub_joints = self.create_publisher(JointState, 'joint_states', qos_profile_sensor_data)
        self.pub_motors = self.create_publisher(MotorStateArray, 'motors', 10)
        self.pub_state = self.create_publisher(SupervisorState, 'supervisor/state', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.pub_odom = self.create_publisher(Odometry, 'base/odom', 10)
        if self.cfg['publish_odom_tf']:
            from tf2_ros import TransformBroadcaster
            self.tf = TransformBroadcaster(self)
        lease_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Heartbeat, '/station/heartbeat', self.on_heartbeat, lease_qos,
                                 callback_group=cb)
        self.create_subscription(Twist, 'base/cmd_vel', self.on_cmd_vel, lease_qos, callback_group=cb)
        self.create_service(Acknowledge, 'supervisor/acknowledge', self.on_ack, callback_group=cb)
        self.create_service(SetTorque, 'set_torque', self.on_torque, callback_group=cb)
        self.create_service(RunDispenser, 'dispenser/run', self.on_dispense, callback_group=cb)
        self.create_service(RunDispenser, 'dispenser/stop', self.on_dispense_stop, callback_group=cb)
        self.create_service(SetGripper, 'gripper/set', self.on_gripper, callback_group=cb)
        self.create_service(CapturePose, 'poses/capture', self.on_capture, callback_group=cb)
        self.create_service(ListPoses, 'poses/list', self.on_list, callback_group=cb)
        self.create_service(DeletePose, 'poses/delete', self.on_delete, callback_group=cb)
        self.action = ActionServer(self, ExecuteTrajectory, 'arm/execute_trajectory',
                                   execute_callback=self.on_execute, goal_callback=self.on_goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
        self.running = True
        self.thread = threading.Thread(target=self.loop, name='postal-control', daemon=True)
        self.thread.start()
        self.get_logger().info(f'Core started; calibration {self.calibration.sha256[:12]}; '
                               f'base direction verified={self.verified}')

    # ------------------------------------------------------------------ control thread
    def submit(self, fn, timeout=3.):
        future = concurrent.futures.Future()
        self.requests.put((fn, future))
        return future.result(timeout)

    def loop(self):
        last = time.monotonic()
        while self.running:
            start = time.monotonic()
            self.cycle_period, last = start - last, start
            try:
                self._ensure_bus()
                while not self.requests.empty():
                    fn, future = self.requests.get_nowait()
                    try:
                        future.set_result(fn())
                    except Exception as exc:  # reported to the caller
                        future.set_exception(exc)
                self._poll()
                with self.lock:
                    self._supervise()
                    self._base_step()
                    self._dispenser_step()
                    self._arm_step()
                self._publish()
            except Exception as exc:  # never let the safety loop die silently
                self.get_logger().error(f'Control cycle error: {exc!r}', throttle_duration_sec=1)
                with self.lock:
                    if self.supervisor.fault(f'control cycle error: {exc}'):
                        self._safe_stop('control cycle error')
            time.sleep(max(0., self.period - (time.monotonic() - start)))

    def _ensure_bus(self):
        if self.bus is not None or time.monotonic() < self.bus_retry:
            return
        try:
            transport = None
            if self.simulated:
                from .fake_bus import FakeTransport
                transport = self.fake = FakeTransport()
            self.bus = Bus(self.device, writable_ids=self.motors, transport=transport)
            self.get_logger().info(f'Opened {self.device}')
        except Exception as exc:
            self.bus_retry = time.monotonic() + 2
            self.get_logger().error(f'Serial bus unavailable: {exc}', throttle_duration_sec=10)

    def _bus_lost(self, exc):
        self.get_logger().error(f'Serial bus lost: {exc}')
        try:
            self.bus.close()
        except Exception:
            pass
        self.bus, self.bus_retry = None, time.monotonic() + 2
        for m in self.motors.values():
            m.responding, m.mode, m.error = False, None, f'bus lost: {exc}'

    def _poll(self):
        if self.bus is None:
            return
        self.cycle += 1
        for m in self.motors.values():
            try:
                if m.mode is None:  # EEPROM read-only probe: mode and write-reply level
                    level = self.bus.read(m.motor_id, RESPONSE_LEVEL, 1)[0][0]
                    mode = self.bus.read(m.motor_id, st3215.MODE, 1)[0][0]
                    self.bus.reply_to_writes[m.motor_id] = level != 0
                    m.mode = mode
                payload, status, received = self.bus.read(m.motor_id, st3215.BLOCK_START, st3215.BLOCK_SIZE)
                m.data, m.packet_error, m.received = st3215.decode_block(payload), status, received
                m.sequence += 1
                m.responding, m.error = True, ''
            except (BusError, st3215.ProtocolError) as exc:
                m.responding, m.error = False, str(exc)
            except OSError as exc:
                self._bus_lost(exc)
                return
        snapshot = {i: (m.data['position'], m.received, m.sequence)
                    for i, m in self.motors.items() if i in ARM_IDS and self._healthy(m)}
        self.snapshots.append((self.cycle, snapshot))

    # ------------------------------------------------------------------ state helpers
    def _fresh(self, m, now=None):
        return m.received is not None and (now or time.monotonic()) - m.received <= self.max_age

    def _healthy(self, m):
        return self._fresh(m) and m.packet_error == 0 and m.data['servo_status'] == 0

    def _problem(self, m, detail=False):
        extra = f' ({m.error})' if detail and m.error else ''
        if m.received is None:
            return f'ID {m.motor_id} ({m.role}): never answered{extra}'
        if not self._fresh(m):
            return f'ID {m.motor_id} ({m.role}): feedback stale{extra}'
        if m.packet_error:
            return f'ID {m.motor_id} ({m.role}): packet error 0x{m.packet_error:02x}'
        if m.data['servo_status']:
            return f'ID {m.motor_id} ({m.role}): servo error 0x{m.data["servo_status"]:02x}'
        if m.mode is not None and m.mode != m.expected_mode:
            return f'ID {m.motor_id} ({m.role}): mode {m.mode}, expected {m.expected_mode}'
        if m.mode is None:
            return f'ID {m.motor_id} ({m.role}): mode not read'
        return None

    def _group_problems(self, group):
        return [p for p in (self._problem(self.motors[i]) for i in self.groups[group]) if p]

    def _torque(self, group):
        """True if any motor of the group reports torque on (last valid reply)."""
        return any(self.motors[i].data and self.motors[i].data['torque_enabled'] for i in self.groups[group])

    def _arm_radians(self):
        values = []
        for name, i in zip(JOINTS, range(1, 8)):
            m = self.motors[i]
            if not self._healthy(m) or not m.data['torque_enabled']:
                return None
            values.append(self.calibration.to_radians(name, m.data['position']))
        return values

    def _write(self, motor_id, address, value, tolerate_status=False):
        """Write one register. A non-zero reply status carries the servo error
        flags (e.g. 0x20 overload); the write itself was executed. Motion writes
        treat it as a failure; stop/disable writes tolerate it so that every
        servo of the group still receives the stop (the fault is reported by
        feedback monitoring)."""
        if self.bus is None:
            raise BusError('Serial bus unavailable')
        try:
            status = self.bus.write(motor_id, address, value)
        except OSError as exc:
            self._bus_lost(exc)
            raise BusError(str(exc)) from exc
        if status and not tolerate_status:
            raise BusError(f'ID {motor_id}: write returned status 0x{status:02x}')
        return status

    # ------------------------------------------------------------------ supervision
    def _supervise(self):
        events = self.supervisor.tick()
        if self.stop_request:
            events.append(self.stop_request)
            self.stop_request = None
        for group in ('arm', 'base', 'dispenser'):
            if self._torque(group) or (group == 'arm' and self.run):
                for problem in self._group_problems(group):
                    if self.supervisor.fault(problem):
                        events.append('fault')
        if events:
            self._safe_stop(', '.join(events))
        warnings = []
        for m in self.motors.values():
            if m.data and self._fresh(m):
                if m.data['temperature'] >= 60:
                    warnings.append(f'ID {m.motor_id} temperature {m.data["temperature"]} °C')
                if m.data['voltage'] < 11.:
                    warnings.append(f'ID {m.motor_id} supply {m.data["voltage"]:.1f} V')
        for group in ('arm', 'base', 'dispenser'):
            if not self._torque(group):
                warnings += [f'{group} (torque off): {p}' for p in self._group_problems(group)]
        if self.simulated:
            warnings.insert(0, 'SIMULATED SERIAL BUS (sim://): not hardware feedback')
        if not self.verified:
            warnings.append('Base wheel directions not yet verified on hardware: speed capped')
        self.warnings = warnings

    def _safe_stop(self, reason):
        self.get_logger().warn(f'Safe stop: {reason}')
        self.cmd_time = None
        errors = self._stop_wheel_mode(self.wheel_ids + [self.dispenser_id])
        self.wheel_target, self.dispenser_deadline = [0, 0, 0], None
        if self.run is not None and self.run.state == RUNNING:
            # Only an interrupted trajectory needs a new goal; an idle arm already
            # holds its last goal (rewriting it would accumulate gravity sag).
            self.run._abort(f'Stopped: {reason}')
            errors += self._hold_arm()
            self.run.held = True
        for e in errors:
            self.supervisor.fault(e)

    def _stop_wheel_mode(self, ids):
        """Zero speed for wheel-mode servos that are energized or of unknown state.

        Writing a goal register turns ST3215 torque on, so servos whose fresh
        feedback says torque is off are left free (they cannot be driving)."""
        errors = []
        for i in ids:
            m = self.motors[i]
            if self._fresh(m) and not m.data['torque_enabled']:
                continue
            try:
                self._write(i, st3215.GOAL_SPEED, 0, tolerate_status=True)
            except BusError as exc:
                errors.append(f'ID {i}: stop failed: {exc}')
        return errors

    def _hold_arm(self):
        """Stop sending arm goals; the last goal is a validated path sample at most
        one control period (and the tracking bound) ahead, as in the legacy
        executor. Rewriting goal = measured position would add gravity sag at
        every stop, so only a moving gripper is stopped where it is."""
        errors = []
        for i in [self.gripper_id]:
            m = self.motors[i]
            if not (m.data and m.data['torque_enabled']) or (
                    self._fresh(m) and abs(m.data['speed']) <= self.cfg['stationary_speed_ticks_s']):
                continue
            if not self._fresh(m):
                errors.append(f'ID {i}: cannot hold at measured position (stale); last goal kept')
                continue
            try:
                self._write(i, st3215.GOAL_POSITION, m.data['position'], tolerate_status=True)
            except BusError as exc:
                errors.append(f'ID {i}: hold failed: {exc}')
        return errors

    # ------------------------------------------------------------------ base
    def _set_wheels(self, raw, force=False):
        now = time.monotonic()
        if not force and raw == self.wheel_target and now - self.wheel_written < .5:
            return
        if raw == [0, 0, 0] and self.wheel_target == [0, 0, 0] and not self._torque('base'):
            return  # never energize free wheels just to write a zero speed
        if raw == [0, 0, 0] and self.wheel_target != [0, 0, 0]:
            self.wheel_zero_since = now
        failed = []
        for i, value in zip(self.wheel_ids, raw):
            try:
                self._write(i, st3215.GOAL_SPEED, st3215.encode_speed(value))
            except BusError as exc:
                failed.append(str(exc))
        self.wheel_target, self.wheel_written = list(raw), now
        if failed:
            raise BusError('; '.join(failed))

    def _base_step(self):
        now = time.monotonic()
        target = [0, 0, 0]
        acked = self.supervisor.acknowledged_at
        if (self.cmd_time is not None and acked is not None and self.cmd_time > acked
                and now - self.cmd_time <= self.cfg['cmd_timeout_s']
                and self.supervisor.commands_allowed() and self._torque('base')
                and not self._group_problems('base')):
            vx, vy, wz = self.cmd
            linear = math.hypot(vx, vy)
            if linear > self.max_linear:
                vx, vy = vx * self.max_linear / linear, vy * self.max_linear / linear
            wz = max(-self.max_angular, min(self.max_angular, wz))
            target = self.kiwi.raw_speeds(vx, vy, wz)
        try:
            self._set_wheels(target)
        except BusError as exc:
            if self.supervisor.fault(f'wheel command failed: {exc}'):
                self._safe_stop('wheel command failure')
        if target == [0, 0, 0] and self.wheel_zero_since is not None:
            speeds = [self.motors[i].data['speed'] for i in self.wheel_ids
                      if self.motors[i].data and self._fresh(self.motors[i])]
            if len(speeds) == 3 and all(abs(s) <= self.cfg['wheel_stop_speed_raw'] for s in speeds):
                self.wheel_zero_since = None
            elif now - self.wheel_zero_since > self.cfg['wheel_stop_timeout_s']:
                self.wheel_zero_since = None
                if self.supervisor.fault(f'wheels not confirmed stopped: speeds {speeds}'):
                    self._safe_stop('wheel stop not confirmed')

    # ------------------------------------------------------------------ dispenser
    def _dispenser_step(self):
        if self.dispenser_deadline is None:
            return
        if time.monotonic() >= self.dispenser_deadline or not self.supervisor.commands_allowed():
            self.dispenser_deadline = None
            try:
                self._write(self.dispenser_id, st3215.GOAL_SPEED, 0, tolerate_status=True)
            except BusError as exc:
                if self.supervisor.fault(f'dispenser stop failed: {exc}'):
                    self._safe_stop('dispenser stop failure')

    # ------------------------------------------------------------------ arm
    def _arm_loads(self):
        """Raw loads of the seven arm servos from their latest valid reply (None if unavailable)."""
        loads = []
        for i in range(1, 8):
            data = self.motors[i].data
            if not data:
                return None
            loads.append(data['load'])
        return loads

    def _arm_step(self):
        run = self.run
        if run is None:
            return
        if run.state != RUNNING:
            pass
        elif not self.supervisor.commands_allowed():
            run._abort('Supervisor no longer active')
        else:
            goals = run.step(time.monotonic(), self._arm_radians(), self._arm_loads())
            if goals:
                try:
                    for i, value in zip(range(1, 8), goals):
                        self._write(i, st3215.GOAL_POSITION, value)
                except BusError as exc:
                    run._abort(f'Goal write failed: {exc}')
        if run.state == ABORTED and not getattr(run, 'held', False):
            run.held = True
            for e in self._hold_arm():
                self.supervisor.fault(e)

    # ------------------------------------------------------------------ publishing
    def _publish(self):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        motors = MotorStateArray()
        motors.header.stamp, motors.cycle_period_s = stamp, float(self.cycle_period)
        joints = JointState()
        joints.header.stamp = stamp
        with self.lock:
            for m in self.motors.values():
                s = MotorState(motor_id=m.motor_id, role=m.role, ever_received=m.received is not None,
                               fresh=self._fresh(m, now), responding=m.responding,
                               age_s=float(now - m.received) if m.received else -1.,
                               packet_error=m.packet_error, mode=255 if m.mode is None else m.mode,
                               fault=self._problem(m, detail=True) or '')
                if m.data:
                    d = m.data
                    s.position_ticks, s.speed_ticks_s, s.load_raw = d['position'], d['speed'], d['load']
                    s.current_raw, s.voltage_v, s.temperature_c = d['current'], float(d['voltage']), d['temperature']
                    s.servo_status, s.torque_enabled, s.moving = d['servo_status'], d['torque_enabled'], d['moving']
                motors.motors.append(s)
                if m.role == 'arm' and self._healthy(m) and m.sequence and now - m.received < self.period * 1.5:
                    name = JOINTS[m.motor_id - 1]
                    joints.name.append(name)
                    joints.position.append(self.calibration.to_radians(name, m.data['position']))
                    joints.velocity.append(m.data['speed'] * self.calibration.data['joints'][m.motor_id - 1]['sign']
                                           / Calibration.scale(self.calibration.joint(name)))
            state = self._state_msg(stamp)
            wheels = [self.motors[i] for i in self.wheel_ids]
            wheel_speeds = [w.data['speed'] for w in wheels if w.data and self._fresh(w, now)]
        self.pub_motors.publish(motors)
        if joints.name:
            self.pub_joints.publish(joints)
        self.pub_state.publish(state)
        if len(wheel_speeds) == 3:
            self._publish_odom(stamp, wheel_speeds)
        if now - self.last_diag >= 1.:
            self.last_diag = now
            self._publish_diagnostics(stamp, motors)

    def _state_msg(self, stamp):
        s = self.supervisor
        msg = SupervisorState(state=s.state, state_name=s.name, session_id=s.session,
                              lease_age_s=min(s.lease_age(), 1e9), faults=list(s.faults),
                              warnings=list(self.warnings), last_event=s.last_event)
        msg.header.stamp = stamp
        msg.arm_torque = all(self.motors[i].data and self.motors[i].data['torque_enabled'] for i in ARM_IDS)
        msg.base_enabled = all(self.motors[i].data and self.motors[i].data['torque_enabled'] for i in self.wheel_ids)
        d = self.motors[self.dispenser_id].data
        msg.dispenser_enabled = bool(d and d['torque_enabled'])
        msg.arm_busy = self.run is not None and self.run.state == RUNNING
        msg.base_moving = self.wheel_target != [0, 0, 0]
        msg.dispenser_running = self.dispenser_deadline is not None
        msg.dispenser_remaining_s = max(0., self.dispenser_deadline - time.monotonic()) if self.dispenser_deadline else 0.
        return msg

    def _publish_odom(self, stamp, speeds):
        vx, vy, wz = self.kiwi.twist_from_raw(speeds)
        dt = min(self.cycle_period, .2)
        x, y, yaw = self.odom
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))
        self.odom = [x, y, yaw]
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'odom', 'base_footprint'
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = vx, vy, wz
        unverified = 0. if self.verified else 100.
        msg.pose.covariance[0] = msg.pose.covariance[7] = .0025 + unverified
        msg.pose.covariance[35] = .01 + unverified
        self.pub_odom.publish(msg)
        if self.cfg['publish_odom_tf']:
            from geometry_msgs.msg import TransformStamped
            t = TransformStamped()
            t.header, t.child_frame_id = msg.header, msg.child_frame_id
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(t)

    def _publish_diagnostics(self, stamp, motors):
        array = DiagnosticArray()
        array.header.stamp = stamp
        for s in motors.motors:
            level = DiagnosticStatus.OK if not s.fault else (
                DiagnosticStatus.ERROR if s.role in ('arm', 'gripper') or s.torque_enabled else DiagnosticStatus.WARN)
            array.status.append(DiagnosticStatus(
                level=level, name=f'postal/motor/{s.motor_id}', hardware_id=f'st3215:{s.motor_id}',
                message=s.fault or 'fresh feedback', values=[
                    KeyValue(key='role', value=s.role), KeyValue(key='position_ticks', value=str(s.position_ticks)),
                    KeyValue(key='age_s', value=f'{s.age_s:.3f}'), KeyValue(key='voltage_V', value=f'{s.voltage_v:.1f}'),
                    KeyValue(key='temperature_C', value=str(s.temperature_c)),
                    KeyValue(key='servo_status', value=f'0x{s.servo_status:02x}'),
                    KeyValue(key='torque', value=str(s.torque_enabled))]))
        sv = self.supervisor
        array.status.append(DiagnosticStatus(
            level=DiagnosticStatus.OK if sv.state == ACTIVE else DiagnosticStatus.WARN if sv.state != FAULT
            else DiagnosticStatus.ERROR, name='postal/supervisor', hardware_id='rdk', message=sv.name,
            values=[KeyValue(key='faults', value='; '.join(sv.faults)), KeyValue(key='last_event', value=sv.last_event)]))
        self.pub_diag.publish(array)

    # ------------------------------------------------------------------ ROS inputs
    def on_heartbeat(self, msg):
        with self.lock:
            if self.supervisor.heartbeat(msg.session_id, msg.sequence):
                self.stop_request = 'new station session'

    def on_cmd_vel(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        with self.lock:
            if all(math.isfinite(v) for v in values):
                self.cmd, self.cmd_time = values, time.monotonic()
            else:
                self.cmd_time = None

    def _require_active(self):
        if not self.supervisor.commands_allowed():
            raise RuntimeError(f'Supervisor is {self.supervisor.name}; acknowledge from the station first')

    def _respond(self, response, fn):
        try:
            response.message = self.submit(fn) or 'OK'
            response.success = True
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def on_ack(self, request, response):
        def act():
            blocking = [p for g in ('arm', 'base', 'dispenser') if self._torque(g) for p in self._group_problems(g)]
            ok, message = self.supervisor.acknowledge(request.session_id, blocking)
            if not ok:
                raise RuntimeError(message)
            self.cmd_time = None
            return message
        return self._respond(response, act)

    def on_torque(self, request, response):
        def act():
            group = request.group
            if group not in self.groups:
                raise ValueError('group must be arm, base or dispenser')
            if request.enable:
                self._require_active()
                problems = self._group_problems(group)
                if problems:
                    raise RuntimeError('; '.join(problems))
            elif not self.supervisor.lease_fresh():
                raise RuntimeError('No fresh station lease')
            if group == 'arm':
                if self.run is not None and self.run.state == RUNNING:
                    raise RuntimeError('Trajectory running; cancel it first')
                if not request.enable and not request.confirm_arm_unsupported:
                    raise RuntimeError('Support the arm and confirm: it can fall when torque is removed')
                if request.enable:
                    c = self.calibration.data
                    for i in ARM_IDS:
                        # Goal = measured position FIRST: any goal write energizes an
                        # ST3215, so it must never meet a stale goal register.
                        self._write(i, st3215.GOAL_POSITION, self.motors[i].data['position'])
                        speed = self.cfg['gripper_speed_ticks_s'] if i == self.gripper_id else c['servo_speed_ticks_s']
                        self._write(i, st3215.GOAL_SPEED, speed)
                        self._write(i, st3215.ACCELERATION, c['servo_acceleration'])
                if request.enable:
                    for i in ARM_IDS:
                        self._write(i, st3215.TORQUE_ENABLE, 1)
                    return 'arm torque enabled (confirm in motor feedback)'
                return self._disable(ARM_IDS, group)
            if request.enable:
                for i in self.groups[group]:
                    self._write(i, st3215.GOAL_SPEED, 0)
                for i in self.groups[group]:
                    self._write(i, st3215.TORQUE_ENABLE, 1)
                return f'{group} torque enabled (confirm in motor feedback)'
            if group == 'base':
                self.wheel_target = [0, 0, 0]
            else:
                self.dispenser_deadline = None
            return self._disable(self.groups[group], group, zero_speed=True)
        return self._respond(response, act)

    def _disable(self, ids, group, zero_speed=False):
        """Remove torque from EVERY servo of the group, even if some reply with
        error flags or fail; report them afterwards."""
        flagged, failed = [], []
        for i in ids:
            try:
                status = 0
                if zero_speed:
                    status |= self._write(i, st3215.GOAL_SPEED, 0, tolerate_status=True) or 0
                status |= self._write(i, st3215.TORQUE_ENABLE, 0, tolerate_status=True) or 0
                if status:
                    flagged.append(f'ID {i} flags 0x{status:02x}')
            except BusError as exc:
                failed.append(str(exc))
        if failed:
            raise BusError(f'{group} torque NOT removed everywhere: ' + '; '.join(failed + flagged))
        return f'{group} torque disabled' + (f' (servo flags: {", ".join(flagged)})' if flagged else '') \
            + ' (confirm in motor feedback)'

    def on_dispense(self, request, response):
        def act():
            self._require_active()
            d = request.duration_s
            if not (math.isfinite(d) and 0 < d <= self.cfg['dispenser_max_duration_s']):
                raise ValueError(f"duration_s must be within (0, {self.cfg['dispenser_max_duration_s']}] s")
            speed = request.speed or self.cfg['dispenser_default_speed']
            if not 0 < abs(speed) <= 1000:
                raise ValueError('speed must be within -1000..1000 and non-zero')
            m = self.motors[self.dispenser_id]
            problem = self._problem(m)
            if problem:
                raise RuntimeError(problem)
            if not m.data['torque_enabled']:
                raise RuntimeError('Dispenser torque is off; enable it explicitly first')
            self._write(self.dispenser_id, st3215.GOAL_SPEED, st3215.encode_speed(speed))
            self.dispenser_deadline = time.monotonic() + d
            return f'Dispenser running at {speed} for {d:.2f} s'
        return self._respond(response, act)

    def on_dispense_stop(self, request, response):
        def act():
            self.dispenser_deadline = None
            errors = self._stop_wheel_mode([self.dispenser_id])
            if errors:
                raise BusError('; '.join(errors))
            return 'Dispenser stopped'
        return self._respond(response, act)

    def on_gripper(self, request, response):
        def act():
            self._require_active()
            lo, hi = self.cfg['gripper_min_ticks'], self.cfg['gripper_max_ticks']
            if not lo <= request.ticks <= hi:
                raise ValueError(f'Gripper target {request.ticks} outside {lo}..{hi} ticks (refused, not clipped)')
            m = self.motors[self.gripper_id]
            problem = self._problem(m)
            if problem:
                raise RuntimeError(problem)
            if not m.data['torque_enabled']:
                raise RuntimeError('Gripper torque is off; enable arm torque first')
            if self.run is not None and self.run.state == RUNNING:
                raise RuntimeError('Arm trajectory running')
            self._write(self.gripper_id, st3215.GOAL_SPEED, self.cfg['gripper_speed_ticks_s'])
            self._write(self.gripper_id, st3215.GOAL_POSITION, request.ticks)
            return f'Gripper moving to {request.ticks} ticks'
        return self._respond(response, act)

    # ------------------------------------------------------------------ poses
    def _pose_info(self, name, pose):
        ticks = pose['ticks']
        info = PoseInfo(name=name, motor_ids=list(ARM_IDS), ticks=[ticks[str(i)] for i in ARM_IDS],
                        joint_names=list(JOINTS), recorded_at=pose.get('recorded_at', ''),
                        calibration_sha256=pose.get('calibration_sha256', ''))
        info.joint_positions = [self.calibration.to_radians(n, ticks[str(i)]) for n, i in zip(JOINTS, range(1, 8))]
        warnings = [w for w in (self.calibration.margin_violation(n, ticks[str(i)])
                                for n, i in zip(JOINTS, range(1, 8))) if w]
        g = ticks[str(self.gripper_id)]
        if not self.cfg['gripper_min_ticks'] <= g <= self.cfg['gripper_max_ticks']:
            warnings.append(f'gripper at {g} ticks outside configured range')
        if info.calibration_sha256 and info.calibration_sha256 != self.calibration.sha256:
            warnings.append('captured with a different calibration file')
        info.warnings = warnings
        return info

    def on_capture(self, request, response):
        start_cycle, deadline = self.cycle, time.monotonic() + 5.
        ticks = None
        while time.monotonic() < deadline:
            with self.lock:
                snaps = [s for c, s in self.snapshots if c > start_cycle]
            ticks = capture_from_snapshots(snaps)
            if ticks:
                break
            time.sleep(self.period)
        if not ticks:
            with self.lock:
                problems = self._group_problems('arm')
            response.success = False
            response.message = ('No stable fresh capture within 5 s: ' +
                                ('; '.join(problems) if problems else 'arm moving (>3 ticks between readings)'))
            return response
        try:
            pose = self.poses.save(request.name, ticks, request.replace, self.calibration.sha256, 'ros2 capture')
            response.pose = self._pose_info(request.name, pose)
            response.success = True
            response.message = f'Saved "{request.name}"' + (
                ' with warnings: ' + '; '.join(response.pose.warnings) if response.pose.warnings else '')
        except (PoseError, OSError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def on_list(self, request, response):
        try:
            response.poses = [self._pose_info(n, p) for n, p in sorted(self.poses.poses().items())]
            response.calibration_sha256 = self.calibration.sha256
            response.success, response.message = True, f'{len(response.poses)} poses'
        except (PoseError, OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def on_delete(self, request, response):
        try:
            self.poses.delete(request.name)
            response.success, response.message = True, f'Deleted "{request.name}" (backup kept)'
        except (PoseError, OSError) as exc:
            response.success, response.message = False, str(exc)
        return response

    # ------------------------------------------------------------------ trajectory action
    def on_goal(self, goal):
        with self.lock:
            if self.run is not None and self.run.state == RUNNING:
                self.get_logger().warn('Rejected concurrent trajectory goal')
                return GoalResponse.REJECT
            if not self.supervisor.commands_allowed():
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_execute(self, handle):
        goal, result = handle.request, ExecuteTrajectory.Result()
        result.error_code = ExecuteTrajectory.Result.REJECTED
        try:
            if not goal.collision_checked or not goal.collision_scene_sha256:
                raise TrajectoryError('Refused: trajectory not validated against a collision scene')
            points = goal.trajectory.points
            times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
            run = TrajectoryRun(self.calibration, goal.trajectory.joint_names, times,
                                [list(p.positions) for p in points],
                                start_tolerance=max(self.cfg['start_tolerance_rad'],
                                                    min(goal.start_tolerance_rad, .08)),
                                arrival_tolerance=max(self.cfg['arrival_tolerance_rad'],
                                                      min(goal.arrival_tolerance_rad, .08)),
                                arrival_timeout=self.cfg['arrival_timeout_s'],
                                max_step_gap=max(.25, 4 * self.period),
                                max_speed_factor=self.cfg['max_speed_factor'],
                                load_limit_raw=self.cfg['overload_load_raw'] or None,
                                compensation=[min(c, self.cfg['max_compensation_rad'])
                                              for c in goal.compensation_limit_rad] or None)

            def start():
                self._require_active()
                if self.run is not None and self.run.state == RUNNING:
                    raise RuntimeError('Another trajectory is running')
                problems = self._group_problems('arm')
                if problems:
                    raise RuntimeError('; '.join(problems))
                if not all(self.motors[i].data['torque_enabled'] for i in range(1, 8)):
                    raise RuntimeError('Arm torque is off; enable it explicitly first')
                if any(abs(self.motors[i].data['speed']) > self.cfg['stationary_speed_ticks_s'] for i in range(1, 8)):
                    raise RuntimeError('Arm must be stationary before a trajectory starts')
                measured = self._arm_radians()
                if measured is None:
                    raise RuntimeError('Fresh arm feedback unavailable')
                c = self.calibration.data
                vscale = max(run.velocity_scale)
                # The ceiling must leave the 25 % headroom above the fastest plan the trajectory check
                # accepts (max_speed_factor x velocity margin): with cap == plan speed the servo
                # cannot catch up on any lag and the tracking error only grows.
                base = c['servo_speed_ticks_s']
                cap = base * self.cfg['max_speed_factor'] * vscale * run.velocity_margin
                # Servo acceleration follows the PLAN's acceleration ratio (the station sends it);
                # 0 = an older station: legacy behaviour, speed ratio squared.
                accel_ratio = goal.acceleration_ratio if goal.acceleration_ratio > 0 else run.speed_ratio ** 2
                accel = int(min(254, max(1, math.ceil(c['servo_acceleration'] * min(accel_ratio, 16.)))))
                for i, peak in zip(range(1, 8), run.peak_ticks_per_s()):
                    # Servo speed follows the plan (25 % headroom), never below the
                    # calibrated speed nor above the configured ceiling.
                    self._write(i, st3215.GOAL_SPEED, int(min(cap, max(base, math.ceil(peak * 1.25)))))
                    self._write(i, st3215.ACCELERATION, accel)  # lag shrinks with acceleration
                run.begin(measured, time.monotonic())
                self.run, self.run_plan = run, goal.plan_id
            self.submit(start)
        except Exception as exc:
            result.message = str(exc)
            handle.abort()
            return result
        feedback = ExecuteTrajectory.Feedback()
        while run.state == RUNNING:
            if handle.is_cancel_requested:
                with self.lock:
                    run._abort('Canceled by request')
                break
            feedback.stage, feedback.progress = 'executing', run.progress(time.monotonic())
            feedback.max_tracking_error_rad = run.max_tracking
            handle.publish_feedback(feedback)
            time.sleep(.1)
        deadline = time.monotonic() + 1
        while run.state == ABORTED and not getattr(run, 'held', False) and time.monotonic() < deadline:
            time.sleep(.02)  # the hold write happens in the control thread
        with self.lock:
            measured = self._arm_radians()
        result.final_positions = measured or []
        result.message = run.message
        if run.state == SUCCEEDED:
            result.error_code = ExecuteTrajectory.Result.SUCCEEDED
            handle.succeed()
        elif handle.is_cancel_requested:
            result.error_code = ExecuteTrajectory.Result.CANCELED
            handle.canceled()
        else:
            result.error_code = ExecuteTrajectory.Result.ABORTED
            handle.abort()
        return result

    def shutdown(self):
        self.running = False
        self.thread.join(timeout=2)
        if self.bus is not None:
            with self.lock:
                self._stop_wheel_mode(self.wheel_ids + [self.dispenser_id])
            self.bus.close()


def main():
    rclpy.init()
    node = None
    try:
        node = Core()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
