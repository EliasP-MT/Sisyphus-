"""Station web interface and ROS bridge (runs on the PC).

* HTTP bound to 127.0.0.1 only; every API call needs the per-process token
  embedded in the page (no cross-site commands) and a loopback Host header
  (no DNS rebinding). Nothing is exposed on the Wi-Fi interface.
* The station lease (/station/heartbeat) is published ONLY while a browser
  tab is open and pinging: closing the tab or losing the PC link stops the
  base on the robot. A new process uses a new session id, so the robot asks
  for a fresh acknowledgement.
* Base commands are relayed only while the browser keeps sending them
  (deadman); one zero twist is sent on release.
"""
import json
import math
import secrets
import threading
import time
import urllib.error
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from postal_robot_interfaces.action import GoToPose
from postal_robot_interfaces.msg import Heartbeat, MotorStateArray, SupervisorState
from postal_robot_interfaces.srv import (Acknowledge, CapturePose, DeletePose, ListPoses, PlanToPose,
                                         RunDispenser, SetGripper, SetTorque)

BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def msg_dict(msg):
    from rosidl_runtime_py.convert import message_to_ordereddict
    return json.loads(json.dumps(message_to_ordereddict(msg), default=float))


DETECT_FRAMES = 4  # frames read per mailbox detection


class Camera:
    def __init__(self):
        self.jpeg, self.stamp, self.received, self.count = None, None, 0., 0
        self.cond = threading.Condition()

    def update(self, msg):
        with self.cond:
            self.jpeg, self.received, self.count = bytes(msg.data), time.time(), self.count + 1
            self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.cond.notify_all()


class Station(Node):
    def __init__(self):
        super().__init__('web', namespace='station')
        self.declare_parameter('port', 8895)
        self.declare_parameter('browser_timeout_s', 1.0)
        self.declare_parameter('planner_url', 'http://127.0.0.1:8893')
        self.port = self.get_parameter('port').value
        self.browser_timeout = self.get_parameter('browser_timeout_s').value
        self.planner_url = self.get_parameter('planner_url').value
        self.token, self.session = secrets.token_urlsafe(24), uuid.uuid4().hex
        self.lock = threading.Lock()
        self.last_ping, self.sequence = 0., 0
        self.supervisor = self.motors = self.joints = None
        self.received = {}
        self.arm_status = {'stage': 'idle', 'message': ''}
        self.base_cmd, self.base_time, self.base_sent_zero = None, 0., True
        self.odom_xy, self.odom_time = None, 0.  # wheel odometry of the robot (task drive steps)
        self.cameras = {1: Camera(), 2: Camera()}
        self.go_handle, self.go_feedback, self.go_result = None, None, None
        self.planner_status, self.planner_time = {'hardware_ready': False, 'reason': 'not queried yet'}, 0.
        cb = ReentrantCallbackGroup()
        self.create_subscription(SupervisorState, '/robot/supervisor/state', self.keep('supervisor'), 10)
        self.create_subscription(MotorStateArray, '/robot/motors', self.keep('motors'), 10)
        self.create_subscription(JointState, '/robot/joint_states', self.keep('joints'), qos_profile_sensor_data)
        self.create_subscription(String, '/station/arm/status', self.on_arm_status, 10)
        self.create_subscription(Odometry, '/robot/base/odom', self.on_odom, qos_profile_sensor_data)
        self.detect_result, self.detect_frame = None, None
        for i, cam in self.cameras.items():
            self.create_subscription(CompressedImage, f'/robot/camera_{i}/image/compressed', cam.update,
                                     qos_profile_sensor_data)
        self.heartbeat = self.create_publisher(Heartbeat, '/station/heartbeat', BEST_EFFORT)
        self.cmd_vel = self.create_publisher(Twist, '/robot/base/cmd_vel', BEST_EFFORT)
        self.srv = {name: self.create_client(t, path, callback_group=cb) for name, t, path in [
            ('ack', Acknowledge, '/robot/supervisor/acknowledge'), ('torque', SetTorque, '/robot/set_torque'),
            ('dispense', RunDispenser, '/robot/dispenser/run'), ('dispense_stop', RunDispenser, '/robot/dispenser/stop'),
            ('gripper', SetGripper, '/robot/gripper/set'), ('capture', CapturePose, '/robot/poses/capture'),
            ('list', ListPoses, '/robot/poses/list'), ('delete', DeletePose, '/robot/poses/delete'),
            ('plan', PlanToPose, '/station/arm/plan_to_pose')]}
        from .tasks import TaskRunner
        self.tasks = TaskRunner(self)
        self.go = ActionClient(self, GoToPose, '/station/arm/go_to_pose', callback_group=cb)
        self.create_timer(.1, self.tick, callback_group=cb)
        self.page = (Path(get_package_share_directory('postal_station')) / 'web' / 'index.html').read_text()

    def keep(self, name):
        def store(msg):
            with self.lock:
                setattr(self, name, msg)
                self.received[name] = time.time()
        return store

    def detect_mailboxes(self, camera, frames=DETECT_FRAMES):
        """Run the mailbox detector (+ OCR) on several successive frames of the camera and keep every
        match: a label whose name was read in any frame keeps it (one frame alone often misses a name)."""
        import base64
        cam = self.cameras.get(camera)
        if cam is None:
            return {'success': False, 'message': f'unknown camera {camera}'}
        began = time.time()
        replies, last_received, error = [], 0., None
        for _ in range(max(1, int(frames))):
            deadline = time.time() + 1.5
            while True:  # a frame newer than the previous one (the arm is still, the views are alike)
                with cam.cond:
                    jpeg, received = cam.jpeg, cam.received
                if jpeg is not None and received > last_received:
                    break
                if time.time() > deadline:
                    jpeg = None
                    break
                time.sleep(0.05)
            if jpeg is None or time.time() - received > 3.:
                error = error or f'camera {camera} has no fresh image (stream stalled?)'
                continue
            last_received = received
            try:
                http = urllib.request.Request('http://127.0.0.1:8894/detect',
                                              json.dumps({'image_b64': base64.b64encode(jpeg).decode()}).encode(),
                                              {'Content-Type': 'application/json'})
                reply = json.load(urllib.request.urlopen(http, timeout=30))
            except Exception as exc:
                return {'success': False, 'message': f'mailbox detector unavailable (OCR worker on :8894?): {exc}'}
            if not reply.get('success'):
                return {'success': False, 'message': reply.get('message', 'detection failed')}
            replies.append(reply)
        if not replies:
            return {'success': False, 'message': error or 'no frame processed'}
        reply = self.merge_detections(replies)
        self.detect_frame = base64.b64decode(reply.pop('image_b64'))
        reply['camera'], reply['time'], reply['frame_id'] = camera, time.time(), int(time.time() * 1000)
        reply['frames'] = len(replies)
        try:
            ticks = self.arm_ticks()
            self.add_base_coordinates(reply['labels'], ticks, reply['geometry'])
            reply['ticks'] = ticks
        except Exception as exc:  # the detection itself is still useful without the arm coordinates
            reply['base_error'] = str(exc)
        self.detect_result = reply
        n = len(reply['labels'])
        named = sum(1 for l in reply['labels'] if l.get('name'))
        return {'success': True, 'message': f'{n} mailbox label{"" if n == 1 else "s"} ({named} named) kept from '
                                           f'{len(replies)} frame{"" if len(replies) == 1 else "s"} on camera {camera} '
                                           f'in {time.time() - began:.1f} s'}

    @staticmethod
    def merge_detections(replies):
        """Union of the labels of several frames. The frame with the most named labels is the base (its
        image and ids); a label from another frame is the same mailbox when its centre is within 40 px,
        and then the entry with a read name wins; labels seen only in other frames are added."""
        import numpy as np

        def centre(label):
            return np.mean(np.asarray(label['corners_px'], float), axis=0)

        def rank(label):  # a name beats none, then the better match, then the larger (closer) label
            return (bool(label.get('name')), label.get('match_score', 0.), label.get('area_px2', 0))

        base = max(replies, key=lambda r: (sum(1 for l in r['labels'] if l.get('name')), len(r['labels'])))
        merged = [dict(l, frames_seen=1) for l in base['labels']]
        for reply in replies:
            if reply is base:
                continue
            for label in reply['labels']:
                match = next((m for m in merged if np.linalg.norm(centre(m) - centre(label)) < 40.), None)
                if match is None:
                    merged.append(dict(label, id=max([m['id'] for m in merged] + [0]) + 1, frames_seen=1))
                    continue
                match['frames_seen'] += 1
                if rank(label) > rank(match):
                    keep_id, seen = match['id'], match['frames_seen']
                    match.clear()
                    match.update(label, id=keep_id, frames_seen=seen)
        return dict(base, labels=merged)

    # ------------------------------------------------------------------ slot targets (arm)
    def arm_ticks(self):
        with self.lock:
            motors = {m.motor_id: m for m in self.motors.motors} if self.motors else {}
        if any(i not in motors or not motors[i].fresh for i in range(1, 8)):
            raise RuntimeError('No fresh arm feedback')
        return [motors[i].position_ticks for i in range(1, 8)]

    def arm_speeds(self):
        """Measured speed (ticks/s) of arm motors 1..7, for diagnostics."""
        with self.lock:
            motors = {m.motor_id: m for m in self.motors.motors} if self.motors else {}
        return [int(motors[i].speed_ticks_s) if i in motors else None for i in range(1, 8)]

    def wait_arm_still(self, timeout=4., speed_ticks_s=8, hold_s=.3):
        """Wait until every arm motor has been still for hold_s (the robot refuses to start a trajectory
        while the arm still creeps, e.g. just after the end-of-leg compensation)."""
        deadline, still_since = time.monotonic() + timeout, None
        while time.monotonic() < deadline:
            with self.lock:
                motors = {m.motor_id: m for m in self.motors.motors} if self.motors else {}
            moving = any(i in motors and abs(motors[i].speed_ticks_s) > speed_ticks_s for i in range(1, 8))
            if moving:
                still_since = None
            else:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since >= hold_s:
                    return True
            time.sleep(.05)
        return False

    def planner_call(self, path, body, timeout=60.):
        http = urllib.request.Request(self.planner_url + path, json.dumps(body).encode(),
                                      {'Content-Type': 'application/json'})
        try:
            return json.load(urllib.request.urlopen(http, timeout=timeout))
        except urllib.error.HTTPError as exc:
            raise RuntimeError(json.load(exc).get('error', str(exc))) from exc

    def add_base_coordinates(self, labels, ticks, geometry):
        """Position of each label and of its slot in the arm base frame (base_link).

        The slot is at the measured offset along the label's x axis; that axis is projected on the
        horizontal plane first: the wall is vertical, so a tilt in the estimate is noise."""
        import numpy as np
        T = np.array(self.planner_call('/camera_pose', {'ticks': ticks})['T_base_camera'])
        camera = T[:3, 3]
        # The robot faces the wall: the wall is vertical and perpendicular to the camera's heading
        # (horizontal part of its optical axis), as for the SLAM. Across the wall = perpendicular to it.
        forward = T[:3, 2]
        forward_h = np.array([forward[0], forward[1], 0.])
        if np.linalg.norm(forward_h) < 0.5:
            for label in labels:
                label['slot_base_m'] = None  # camera looking mostly up or down: no wall heading
            return
        forward_h /= np.linalg.norm(forward_h)
        across = np.array([-forward_h[1], forward_h[0], 0.])
        for label in labels:
            centre = (T @ np.append(label['center_m'], 1.))[:3]
            if not label.get('orientation_known', True):
                label['slot_base_m'] = None  # name unread: which side the slot is on is not known
                continue
            # The estimated 3D x axis of the label is noisy (planar ambiguity): it only gives the sign.
            x_axis = T[:3, :3] @ np.array(label['label_rotation'])[:, 0]
            sign = 1. if x_axis @ across >= 0 else -1.
            slot = centre + sign * geometry['slot_from_label_center_m'][0] * across
            label.update(label_base_m=[round(float(v), 4) for v in centre],
                         slot_base_m=[round(float(v), 4) for v in slot],
                         approach_dir=[round(float(v), 5) for v in forward_h],
                         camera_base_m=[round(float(v), 4) for v in camera])

    def slot_plan(self, body):
        """Plan the fingertips to a point in front of a detected slot, J7 pivoted."""
        d = self.detect_result
        if not d or time.time() - d['time'] > 300:
            return {'success': False, 'message': 'Detect the mailboxes first (and plan within 5 minutes)'}
        label = next((l for l in d['labels'] if l['id'] == int(body['label_id'])), None)
        if label is None or not label.get('slot_base_m'):
            return {'success': False, 'message': 'This label has no reliable slot position (seen edge-on?)'}
        ticks = self.arm_ticks()
        if d.get('ticks') and max(abs(a - b) for a, b in zip(ticks, d['ticks'])) > 12:
            return {'success': False, 'message': 'The arm moved since the detection: detect again'}
        standoff = float(body.get('standoff_cm', 5)) / 100.
        offset = float(body.get('tcp_offset_cm', 10.6)) / 100.
        pivot = math.radians(float(body.get('pivot_deg', 0.)))
        if not -0.15 <= standoff <= 0.5 or not 0 <= offset <= 0.2 or abs(pivot) > math.radians(181):
            return {'success': False, 'message': 'standoff -15..50 cm (negative = inside the box), fingertip offset 0-20 cm, pivot within +/-180 deg'}
        poses = self.call('list')
        scan = next((p for p in poses.poses if p.name == 'scan'), None) if poses.success else None
        reference = list(scan.ticks)[:7] if scan is not None else None  # the pivot is measured from the scan orientation
        ray = label['approach_dir']
        target = [s - standoff * r for s, r in zip(label['slot_base_m'], ray)]
        try:
            ik = self.planner_call('/ik_xyz', {'start_ticks': ticks, 'target_xyz_m': target,
                                                'tcp_offset_m': [0., offset, 0.], 'approach_dir': ray,
                                                'pivot_rad': pivot, 'pivot_reference_ticks': reference})
        except RuntimeError as exc:
            return {'success': False, 'message': f'Arm cannot reach it: {exc}', 'target_xyz_m': target}
        goal = ik['goal_ticks']
        # Wrist first: turn J7 in place to its final angle, then move the arm (never both at once).
        via = list(ticks[:6]) + [goal[6]] if abs(goal[6] - ticks[6]) > 5 else []
        r = self.call('plan', timeout=200., pose_name=f'slot #{label["id"]}', goal_ticks=goal, via_ticks=via,
                      speed_factor=float(body.get('speed_factor', 1.)))
        return {'success': r.success, 'executable': r.executable, 'plan_id': r.plan_id, 'message': r.message,
                'reasons': list(r.reasons), 'duration_s': r.duration_s, 'points': len(r.trajectory.points),
                'target_xyz_m': [round(v, 4) for v in target], 'slot_base_m': label['slot_base_m'],
                'ik': {k: ik[k] for k in ('position_error_m', 'axis_error_deg', 'tip_xyz_m')}}

    def on_arm_status(self, msg):
        try:
            self.arm_status = json.loads(msg.data)
        except ValueError:
            pass

    def browser_alive(self):
        return time.monotonic() - self.last_ping <= self.browser_timeout

    def on_odom(self, msg):
        self.odom_xy, self.odom_time = (msg.pose.pose.position.x, msg.pose.pose.position.y), time.monotonic()

    def drive_check(self):
        """Raise RuntimeError unless the base can be driven right now (no motion is started)."""
        sup = self.supervisor
        if sup is None or sup.state != 1:
            raise RuntimeError('Base drive: the robot is not ACTIVE (acknowledge first)')
        if not sup.base_enabled:
            raise RuntimeError('Base drive: base torque is off (enable the base)')
        if not self.browser_alive():
            raise RuntimeError('Base drive: the station page is not open (no heartbeat, the robot would stop the base)')
        if self.odom_xy is None or time.monotonic() - self.odom_time > 1.:
            raise RuntimeError('Base drive: no wheel odometry from the robot')

    def drive(self, distance_m, speed_m_s=.08, should_stop=lambda: False):
        """Drive straight by `distance_m` (+ forward, - backward), closed loop on the wheel odometry.

        Speed is capped at 0.1 m/s and slows down over the last 4 cm. A drive that does not make progress
        (base off, wheels blocked) or takes more than twice the expected time stops and raises. Returns the
        distance travelled according to the odometry (wheel-speed integration: it does not see slip)."""
        distance = abs(float(distance_m))
        if not 0 < distance <= .5:
            raise ValueError('Base drive: distance must be within 0..50 cm')
        speed = min(abs(float(speed_m_s)), .1)
        sign = 1. if distance_m > 0 else -1.
        self.drive_check()
        start, began, travelled = self.odom_xy, time.monotonic(), 0.
        limit = 2 * distance / speed + 2.
        try:
            while travelled < distance:
                if should_stop():
                    break
                now = time.monotonic()
                if now - began > limit:
                    raise RuntimeError(f'Base drive: only {travelled * 100:.1f} cm of {distance * 100:.0f} cm in {limit:.0f} s')
                if not self.browser_alive() or now - self.odom_time > 1.:
                    raise RuntimeError('Base drive: heartbeat or odometry lost; base stopped')
                travelled = math.hypot(self.odom_xy[0] - start[0], self.odom_xy[1] - start[1])
                v = min(speed, max(.03, (distance - travelled) * 2.5))
                self.base_cmd, self.base_time = [sign * v, 0., 0.], now
                time.sleep(.05)
        finally:
            self.base_cmd = None  # tick() sends one zero Twist; the robot also times out after 0.3 s
        return travelled

    def tick(self):
        if self.browser_alive():
            self.sequence += 1
            hb = Heartbeat(session_id=self.session, sequence=self.sequence)
            hb.stamp = self.get_clock().now().to_msg()
            self.heartbeat.publish(hb)
        fresh = self.base_cmd is not None and time.monotonic() - self.base_time <= .25 and self.browser_alive()
        if fresh:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = self.base_cmd
            self.cmd_vel.publish(t)
            self.base_sent_zero = False
        elif not self.base_sent_zero:
            self.cmd_vel.publish(Twist())
            self.base_sent_zero = True
        if time.monotonic() - self.planner_time > 3:
            self.planner_time = time.monotonic()
            threading.Thread(target=self.query_planner, daemon=True).start()

    def query_planner(self):
        try:
            self.planner_status = json.load(urllib.request.urlopen(self.planner_url + '/status', timeout=2))
        except Exception as exc:
            self.planner_status = {'hardware_ready': False, 'reason': f'planner worker unreachable: {exc}'}

    # ------------------------------------------------------------------ API
    def call(self, service, timeout=20., **fields):
        client = self.srv[service]
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError(f'{client.srv_name} unavailable (robot or station node not reachable)')
        future = client.call_async(client.srv_type.Request(**fields))
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f'{client.srv_name} timed out')
            time.sleep(.02)
        return future.result()

    def state(self):
        now = time.time()
        with self.lock:
            sup, motors, joints = self.supervisor, self.motors, self.joints
            ages = {k: now - v for k, v in self.received.items()}
        cams = {i: {'age_s': (now - c.received) if c.received else None, 'frames': c.count}
                for i, c in self.cameras.items()}
        go = None
        if self.go_handle is not None or self.go_result is not None:
            go = {'feedback': self.go_feedback, 'result': self.go_result, 'active': self.go_result is None}
        return {'session': self.session, 'browser_lease': self.browser_alive(), 'ages_s': ages,
                'supervisor': msg_dict(sup) if sup else None, 'motors': msg_dict(motors)['motors'] if motors else [],
                'joints': dict(zip(joints.name, joints.position)) if joints else {},
                'cameras': cams, 'arm_status': self.arm_status, 'task': self.tasks.snapshot(),
                'detect': self.detect_result, 'go': go, 'planner': self.planner_status}

    def api(self, path, body):
        if path == '/api/ping':
            self.last_ping = time.monotonic()
            return {'ok': True}
        if path == '/api/state':
            return self.state()
        if path == '/api/base':
            v = [float(body.get(k, 0)) for k in ('vx', 'vy', 'wz')]
            self.base_cmd, self.base_time = v, time.monotonic()
            return {'ok': True}
        if path == '/api/base/stop':
            self.base_cmd = None
            return {'ok': True}
        simple = {
            '/api/ack': ('ack', lambda: dict(session_id=self.session)),
            '/api/torque': ('torque', lambda: dict(group=str(body['group']), enable=bool(body['enable']),
                                                   confirm_arm_unsupported=bool(body.get('confirm', False)))),
            '/api/dispenser/run': ('dispense', lambda: dict(duration_s=float(body['duration_s']))),
            '/api/dispenser/stop': ('dispense_stop', lambda: dict()),
            '/api/gripper': ('gripper', lambda: dict(ticks=int(body['ticks']))),
            '/api/pose/capture': ('capture', lambda: dict(name=str(body['name']), replace=bool(body.get('replace')))),
            '/api/pose/delete': ('delete', lambda: dict(name=str(body['name']))),
        }
        if path in simple:
            service, fields = simple[path]
            r = self.call(service, **fields())
            return {'success': r.success, 'message': r.message}
        if path == '/api/poses':
            r = self.call('list')
            return {'success': r.success, 'message': r.message, 'calibration_sha256': r.calibration_sha256,
                    'poses': [msg_dict(p) for p in r.poses]}
        if path == '/api/plan':
            r = self.call('plan', timeout=100., pose_name=str(body['pose']),
                          speed_factor=float(body.get('speed_factor', 1.)),
                          accel_factor=float(body.get('accel_factor', 1.)),
                          jerk_factor=float(body.get('jerk_factor', 1.)),
                          arrival_tolerance_rad=float(body.get('arrival_tolerance_rad', 0.)))
            return {'success': r.success, 'executable': r.executable, 'plan_id': r.plan_id,
                    'message': r.message, 'reasons': list(r.reasons), 'duration_s': r.duration_s,
                    'points': len(r.trajectory.points)}
        if path == '/api/execute':
            return self.execute(str(body['plan_id']), bool(body.get('include_gripper')))
        if path == '/api/slot/plan':
            return self.slot_plan(body)
        if path == '/api/detect':
            return self.detect_mailboxes(int(body.get('camera', 2)))
        if path == '/api/task/plan':
            return self.tasks.prepare(str(body['task']), float(body.get('speed_factor', 1.)), int(body.get('ocr_camera', 1)),
                                      body.get('params') or {})
        if path == '/api/task/ocr':
            return self.tasks.read_now(int(body.get('camera', 1)))
        if path == '/api/task/start':
            return self.tasks.start()
        if path == '/api/task/stop':
            return self.tasks.stop()
        if path == '/api/cancel':
            if self.go_handle is None:
                return {'success': False, 'message': 'No motion in progress'}
            self.go_handle.cancel_goal_async()
            return {'success': True, 'message': 'Cancel requested; the robot holds the arm'}
        raise KeyError(path)

    def execute(self, plan_id, gripper):
        if self.go_handle is not None and self.go_result is None:
            return {'success': False, 'message': 'A motion is already running'}
        if not self.go.wait_for_server(timeout_sec=2.):
            return {'success': False, 'message': 'Station arm_motion node unavailable'}
        self.go_feedback, self.go_result = None, None
        future = self.go.send_goal_async(GoToPose.Goal(plan_id=plan_id, include_gripper=gripper),
                                         feedback_callback=lambda f: setattr(
                                             self, 'go_feedback', {'stage': f.feedback.stage,
                                                                   'progress': f.feedback.progress}))
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.02)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            self.go_result = {'success': False, 'message': 'Motion request rejected'}
            return self.go_result
        self.go_handle = handle

        def done(f):
            r = f.result().result
            self.go_result = {'success': r.success, 'message': r.message}
            self.go_handle = None
        handle.get_result_async().add_done_callback(done)
        return {'success': True, 'message': 'Motion started'}


def make_handler(station):
    allowed_hosts = {f'127.0.0.1:{station.port}', f'localhost:{station.port}'}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def send(self, code, body, kind='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Options', 'DENY')
            self.end_headers()
            self.wfile.write(data)

        def authorized(self, query_token=None):
            if self.headers.get('Host') not in allowed_hosts:
                return False
            token = self.headers.get('X-Postal-Token') or query_token
            return token is not None and secrets.compare_digest(token, station.token)

        def do_GET(self):
            path, _, query = self.path.partition('?')
            if self.headers.get('Host') not in allowed_hosts:
                return self.send(403, {'error': 'loopback host required'})
            if path == '/':
                return self.send(200, station.page.replace('__TOKEN__', station.token).encode(),
                                 'text/html; charset=utf-8')
            params = dict(p.partition('=')[::2] for p in query.split('&') if p)
            if path == '/detect_frame.jpg':
                if not self.authorized(params.get('token')) or station.detect_frame is None:
                    return self.send(404, {'error': 'no frame'})
                return self.send(200, station.detect_frame, 'image/jpeg')
            if path == '/ocr_frame.jpg':
                frame = getattr(station.tasks, 'last_frame', None)
                if not self.authorized(params.get('token')) or frame is None:
                    return self.send(404, {'error': 'no frame'})
                return self.send(200, frame, 'image/jpeg')
            if path.startswith('/camera/') and path.endswith('.mjpg'):
                if not self.authorized(params.get('token')):
                    return self.send(403, {'error': 'token required'})
                return self.stream(int(path[8:-5]))
            self.send(404, {'error': 'not found'})

        def stream(self, index):
            cam = station.cameras.get(index)
            if cam is None:
                return self.send(404, {'error': 'unknown camera'})
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            last = -1
            try:
                while rclpy.ok():
                    with cam.cond:
                        cam.cond.wait_for(lambda: cam.count != last, timeout=5)
                        jpeg, last = cam.jpeg, cam.count
                    if jpeg is None:
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if not self.authorized():
                return self.send(403, {'error': 'invalid token or host'})
            try:
                length = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(length) or b'{}') if length else {}
                self.send(200, station.api(self.path, body))
            except KeyError as exc:
                self.send(404 if str(exc).strip("'").startswith('/') else 400, {'error': f'missing {exc}'})
            except Exception as exc:
                self.send(200, {'success': False, 'message': str(exc)})
    return Handler


def main():
    rclpy.init()
    node = None
    try:
        node = Station()
        server = ThreadingHTTPServer(('127.0.0.1', node.port), make_handler(node))
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        node.get_logger().info(f'Station UI: http://127.0.0.1:{node.port}/ (session {node.session[:8]})')
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
