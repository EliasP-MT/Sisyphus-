"""Station-side arm motion: plan to a stored pose, preview, execute on the robot.

PlanToPose (/station/arm/plan_to_pose):
  pose from the robot library + fresh measured arm ticks -> planner worker
  (legacy collision checker) -> single-use plan valid for plan_ttl_s, with the
  reasons it is not executable. The plan is animated on
  /station/preview/joint_states for RViz (frame prefix "preview/").
GoToPose (/station/arm/go_to_pose):
  only an executable, unexpired, unused plan whose start still matches the
  measured arm; sent to the robot ExecuteTrajectory action. While it runs the
  measured pose is re-checked for collision and the robot goal is canceled on
  a violation. Optionally moves the gripper to the pose ticks afterwards.
"""
import json
import os
import threading
import time
import urllib.error
import urllib.request
import uuid

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from postal_robot_interfaces.action import ExecuteTrajectory, GoToPose
from postal_robot_interfaces.msg import MotorStateArray
from postal_robot_interfaces.srv import ListPoses, PlanToPose, SetGripper

JOINTS = [f'rev_motor_{i:02d}' for i in range(1, 8)]


class ArmMotion(Node):
    def __init__(self):
        super().__init__('arm_motion', namespace='station')
        self.declare_parameter('planner_url', 'http://127.0.0.1:8893')
        self.declare_parameter('plan_ttl_s', 120.)
        self.declare_parameter('start_tolerance_ticks', 6)
        self.declare_parameter('feedback_max_age_s', .5)
        self.url = self.get_parameter('planner_url').value
        self.ttl = self.get_parameter('plan_ttl_s').value
        self.leg_cache, self.ids, self.ids_time = {}, '', 0.
        self.cache_path = os.path.join(os.path.expanduser('~'), 'leg_cache.json')
        try:
            with open(self.cache_path) as f:
                self.leg_cache = json.load(f)
        except (OSError, ValueError):
            pass
        self.start_tol = self.get_parameter('start_tolerance_ticks').value
        self.max_age = self.get_parameter('feedback_max_age_s').value
        cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.motors, self.motors_time = {}, 0.
        self.plans, self.status = {}, {'stage': 'idle', 'message': ''}
        self.create_subscription(MotorStateArray, '/robot/motors', self.on_motors, 10, callback_group=cb)
        self.list_poses = self.create_client(ListPoses, '/robot/poses/list', callback_group=cb)
        self.gripper = self.create_client(SetGripper, '/robot/gripper/set', callback_group=cb)
        self.robot = ActionClient(self, ExecuteTrajectory, '/robot/arm/execute_trajectory', callback_group=cb)
        self.preview = self.create_publisher(JointState, 'preview/joint_states', 10)
        self.status_pub = self.create_publisher(String, 'arm/status', 10)
        self.create_service(PlanToPose, 'arm/plan_to_pose', self.on_plan, callback_group=cb)
        self.server = ActionServer(self, GoToPose, 'arm/go_to_pose', execute_callback=self.on_go,
                                   goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=cb)
        self.preview_plan, self.preview_start = None, 0.
        self.active = False
        self.create_timer(.1, self.tick, callback_group=cb)

    # ------------------------------------------------------------------ helpers
    def on_motors(self, msg):
        with self.lock:
            self.motors = {m.motor_id: m for m in msg.motors}
            self.motors_time = time.monotonic()

    def measured_ticks(self, wait=2.):
        # Before a motion a short feedback gap (Wi-Fi hiccup) is waited out, never used afterwards.
        deadline = time.monotonic() + wait
        while True:
            with self.lock:
                motors, age = dict(self.motors), time.monotonic() - self.motors_time
            if age <= self.max_age or time.monotonic() >= deadline:
                break
            time.sleep(.05)
        if age > self.max_age:
            raise RuntimeError(f'No fresh robot motor feedback (last {age:.1f} s ago)')
        ticks = []
        for i in range(1, 9):
            m = motors.get(i)
            if m is None or not m.fresh or m.fault:
                raise RuntimeError(f'Motor {i}: ' + (m.fault if m and m.fault else 'no fresh feedback'))
            ticks.append(m.position_ticks)
        return ticks

    def planner_ids(self):
        """Scene + calibration identity of the planner worker (cache validity)."""
        if time.monotonic() - self.ids_time > 30:
            try:
                st = json.load(urllib.request.urlopen(self.url + '/status', timeout=3))
            except OSError as exc:
                raise RuntimeError('Planner worker is not running (check ros2_robot/.station_ws/planner.log; '
                                   f'a missing CAD file, e.g. an unplugged USB drive, stops it): {exc}') from exc
            self.ids = f"{st.get('scene_sha256')}|{st.get('calibration_sha256')}"
            self.ids_time = time.monotonic()
        return self.ids

    def save_cache(self):
        try:
            tmp = self.cache_path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(self.leg_cache, f)
            os.replace(tmp, self.cache_path)
        except OSError as exc:
            self.get_logger().warn(f'Leg cache not saved: {exc}')

    def request(self, path, body=None, timeout=120.):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + path, data, {'Content-Type': 'application/json'})
        try:
            return json.load(urllib.request.urlopen(req, timeout=timeout))
        except urllib.error.HTTPError as exc:
            raise RuntimeError(json.load(exc).get('error', str(exc))) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f'Planner worker unreachable at {self.url}: {exc.reason}') from exc
        except TimeoutError as exc:
            raise RuntimeError(f'Planner worker did not answer within {timeout:.0f} s (busy or stuck; '
                               'restart the station)') from exc

    def call(self, client, request, timeout=10.):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError(f'Robot service {client.srv_name} unavailable')
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f'{client.srv_name} timed out')
            time.sleep(.02)
        return future.result()

    def set_status(self, stage, message):
        self.status = {'stage': stage, 'message': message, 'time': time.time()}
        self.status_pub.publish(String(data=json.dumps(self.status)))

    def tick(self):
        plan = self.preview_plan
        if plan is None:
            return
        t = time.monotonic() - self.preview_start
        times = plan['times']
        index = next((i for i, v in enumerate(times) if v >= t), len(times) - 1)
        msg = JointState(name=JOINTS, position=plan['positions'][index])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.preview.publish(msg)
        if t > times[-1] + 3:
            self.preview_start = time.monotonic()  # loop the preview

    # ------------------------------------------------------------------ planning
    def on_plan(self, request, response):
        response.success = False
        try:
            self.set_status('planning', f'Planning to "{request.pose_name}"')
            if len(request.goal_ticks) == 7:  # a computed target (e.g. the slot of a mailbox), not a stored pose
                from types import SimpleNamespace
                pose = SimpleNamespace(name=request.pose_name or 'target', warnings=[],
                                       ticks=list(request.goal_ticks) + [self.measured_ticks()[7]])
                poses = SimpleNamespace(poses=[])
            else:
                poses = self.call(self.list_poses, ListPoses.Request())
                if not poses.success:
                    raise RuntimeError(poses.message)
                pose = next((p for p in poses.poses if p.name == request.pose_name), None)
                if pose is None:
                    raise RuntimeError(f'Unknown pose "{request.pose_name}"')
            if request.start_pose_name:
                spose = next((p for p in poses.poses if p.name == request.start_pose_name), None)
                if spose is None:
                    raise RuntimeError(f'Unknown start pose "{request.start_pose_name}"')
                start = list(spose.ticks)
            elif len(request.start_ticks) == 7:
                start = list(request.start_ticks) + [self.measured_ticks()[7]]
            else:
                start = self.measured_ticks()
            speed = request.speed_factor or 1.
            accel, jerk = request.accel_factor or 1., request.jerk_factor or 1.
            began = time.monotonic()
            fixed = request.start_pose_name != ''
            key = None
            if fixed:  # pose-to-pose legs never change: computed once, kept on disk for later runs
                ids = self.planner_ids()
                key = f'{ids}|{list(start[:7])}|{list(pose.ticks[:7])}|{speed}|{accel}|{jerk}'
            if key is not None and key in self.leg_cache:
                result = self.leg_cache[key]
                self.get_logger().info(f'Plan to {request.pose_name}: cached leg')
            else:
                via = [list(request.via_ticks[k:k + 7]) for k in range(0, len(request.via_ticks) - 6, 7)]
                body = {'start_ticks': start[:7], 'goal_ticks': list(pose.ticks)[:7],
                        'speed_factor': speed, 'accel_factor': accel, 'jerk_factor': jerk}
                if via:
                    result = self.request('/plan_via', {**body, 'via_ticks': via}, timeout=75. * (len(via) + 1))
                else:
                    result = self.request('/plan', body, timeout=75.)
                if key is not None and not result['reasons']:
                    self.leg_cache[key] = result
                    self.save_cache()
            self.get_logger().info(f'Plan to {request.pose_name} at {speed:g}x in {time.monotonic() - began:.1f} s')
            reasons = list(result['reasons'])
            if pose.warnings:
                reasons += [f'pose warning: {w}' for w in pose.warnings]
            plan_id = uuid.uuid4().hex[:12]
            traj = JointTrajectory(joint_names=JOINTS)
            for t, q in zip(result['times'], result['positions']):
                traj.points.append(JointTrajectoryPoint(positions=q, time_from_start=Duration(
                    sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1000000000)))
            record = dict(id=plan_id, created=time.monotonic(), start=start, pose=request.pose_name,
                          gripper=list(pose.ticks)[7], traj=traj, executable=not reasons, used=False,
                          sha=result['collision_scene_sha256'], times=result['times'],
                          compensation=([0.] * 7 if request.skip_compensation
                                        else [float(c) for c in result['compensation_limit_rad']]),
                          positions=result['positions'],
                          ttl=request.valid_for_s or self.ttl,
                          start_pose=request.start_pose_name,
                          accel_ratio=float(accel),
                          arrival_tol=float(request.arrival_tolerance_rad),
                          start_rad=float(request.start_tolerance_rad),
                          start_tol=max(13 if request.start_pose_name else self.start_tol,
                                        int(request.start_tolerance_rad * 652) + 1))  # ticks; 13 = 0.02 rad, as the robot
            with self.lock:
                self.plans = {k: v for k, v in self.plans.items()
                              if time.monotonic() - v['created'] < v.get('ttl', self.ttl)}
                self.plans[plan_id] = record
            self.preview_plan, self.preview_start = record, time.monotonic()
            response.success, response.executable, response.plan_id = True, not reasons, plan_id
            response.reasons, response.duration_s = reasons, result['duration_s']
            response.collision_scene_sha256, response.trajectory = result['collision_scene_sha256'], traj
            response.message = (f'Collision-free plan at {speed:g}x speed (accel {accel:g}x, jerk {jerk:g}x), '
                                f'{result["duration_s"]:.1f} s, '
                                f'{len(result["times"])} checked samples, planned in {result["planning_s"]:.1f} s; '
                                f'{result["compensation_note"]}')
            self.set_status('planned', response.message + ('' if not reasons else '; NOT executable: ' + '; '.join(reasons)))
        except Exception as exc:
            response.message = str(exc)
            self.set_status('error', f'Planning refused: {exc}')
        return response

    # ------------------------------------------------------------------ execution
    def on_goal(self, goal):
        return GoalResponse.REJECT if self.active else GoalResponse.ACCEPT

    def on_go(self, handle):
        self.active = True
        result = GoToPose.Result(success=False)
        try:
            result.message = self._go(handle)
            result.success = True
            self.set_status('done', result.message)
            handle.succeed()
        except Exception as exc:
            result.message = str(exc)
            self.set_status('canceled' if handle.is_cancel_requested else 'error', result.message)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            self.active = False
        return result

    def _go(self, handle):
        with self.lock:
            plan = self.plans.get(handle.request.plan_id)
            if plan is None or time.monotonic() - plan['created'] > plan.get('ttl', self.ttl):
                raise RuntimeError('Unknown or expired plan; plan again')
            if plan['used']:
                raise RuntimeError('Plan already used; plan again')
            if not plan['executable']:
                raise RuntimeError('Plan is not executable (see its reasons)')
            plan['used'] = True
        current = self.measured_ticks()
        drift = max(abs(a - b) for a, b in zip(current[:7], plan['start'][:7]))
        if drift > plan.get('start_tol', self.start_tol):
            raise RuntimeError(f'Arm is {drift} ticks away from where the plan starts'
                               + (f' (pose "{plan["start_pose"]}"): move it there' if plan.get('start_pose') else '; plan again'))
        if not self.robot.wait_for_server(timeout_sec=3.):
            raise RuntimeError('Robot trajectory action unavailable')
        self.set_status('executing', f'Executing plan to "{plan["pose"]}"')
        goal = ExecuteTrajectory.Goal(trajectory=plan['traj'], plan_id=plan['id'], collision_checked=True,
                                      collision_scene_sha256=plan['sha'],
                                      compensation_limit_rad=plan['compensation'],
                                      acceleration_ratio=plan.get('accel_ratio', 0.),
                                      arrival_tolerance_rad=plan.get('arrival_tol', 0.),
                                      start_tolerance_rad=plan.get('start_rad', 0.))
        feedback = GoToPose.Feedback()

        def relay(fb):
            feedback.stage, feedback.progress = fb.feedback.stage, fb.feedback.progress
            handle.publish_feedback(feedback)
        sent = self.robot.send_goal_async(goal, feedback_callback=relay)
        while not sent.done():
            time.sleep(.02)
        robot_goal = sent.result()
        if not robot_goal.accepted:
            raise RuntimeError('Robot rejected the trajectory (supervisor not active or arm busy)')
        try:
            return self._follow(handle, plan, robot_goal)
        except BaseException:
            robot_goal.cancel_goal_async()  # never leave the robot running a goal we no longer track
            raise

    def _follow(self, handle, plan, robot_goal):
        outcome = robot_goal.get_result_async()
        last_check = 0.
        while not outcome.done():
            if handle.is_cancel_requested:
                robot_goal.cancel_goal_async()
                break
            if time.monotonic() - last_check > .25:
                last_check = time.monotonic()
                try:
                    ticks = self.measured_ticks(wait=0)
                    check = self.request('/check', {'ticks': ticks[:7]}, timeout=2.)
                except Exception as exc:  # monitor unavailable: the robot keeps its own guards
                    self.get_logger().warn(f'Execution monitor: {exc}', throttle_duration_sec=2)
                    check = {'valid': True}
                if not check['valid']:
                    robot_goal.cancel_goal_async()
                    raise RuntimeError(f'Measured arm pose rejected: {check["reason"]}; robot goal canceled')
            time.sleep(.05)
        while not outcome.done():
            time.sleep(.02)
        res = outcome.result().result
        if res.error_code != ExecuteTrajectory.Result.SUCCEEDED:
            raise RuntimeError(f'Robot: {res.message}')
        message = f'Arrived at "{plan["pose"]}": {res.message}'
        if handle.request.include_gripper:
            self.set_status('gripper', f'Moving gripper to {plan["gripper"]} ticks')
            r = self.call(self.gripper, SetGripper.Request(ticks=plan['gripper']))
            if not r.success:
                raise RuntimeError(f'Arm arrived; gripper refused: {r.message}')
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if abs(self.measured_ticks()[7] - plan['gripper']) <= 10:
                    return message + '; gripper reached target'
                time.sleep(.1)
            raise RuntimeError('Arm arrived; gripper did not reach its target within 15 s (blocked?)')
        return message


def main():
    rclpy.init()
    node = None
    try:
        node = ArmMotion()
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
