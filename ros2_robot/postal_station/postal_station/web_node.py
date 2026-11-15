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
import secrets
import threading
import time
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
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
        self.cameras = {1: Camera(), 2: Camera()}
        self.go_handle, self.go_feedback, self.go_result = None, None, None
        self.planner_status, self.planner_time = {'hardware_ready': False, 'reason': 'not queried yet'}, 0.
        cb = ReentrantCallbackGroup()
        self.create_subscription(SupervisorState, '/robot/supervisor/state', self.keep('supervisor'), 10)
        self.create_subscription(MotorStateArray, '/robot/motors', self.keep('motors'), 10)
        self.create_subscription(JointState, '/robot/joint_states', self.keep('joints'), qos_profile_sensor_data)
        self.create_subscription(String, '/station/arm/status', self.on_arm_status, 10)
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

    def on_arm_status(self, msg):
        try:
            self.arm_status = json.loads(msg.data)
        except ValueError:
            pass

    def browser_alive(self):
        return time.monotonic() - self.last_ping <= self.browser_timeout

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
                'cameras': cams, 'arm_status': self.arm_status, 'task': self.tasks.snapshot(), 'go': go, 'planner': self.planner_status}

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
                          speed_factor=float(body.get('speed_factor', 1.)))
            return {'success': r.success, 'executable': r.executable, 'plan_id': r.plan_id,
                    'message': r.message, 'reasons': list(r.reasons), 'duration_s': r.duration_s,
                    'points': len(r.trajectory.points)}
        if path == '/api/execute':
            return self.execute(str(body['plan_id']), bool(body.get('include_gripper')))
        if path == '/api/task/plan':
            return self.tasks.prepare(str(body['task']), float(body.get('speed_factor', 1.)))
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
