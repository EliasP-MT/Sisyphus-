"""Collision-checked joint planning service for the station (PC, no ROS).

Runs with the validated arm_control virtual environment (pybullet, OMPL,
ikpy, ruckig) and reuses arm_control/planner.py and collision.py unchanged:
the same limits, scene validation, OMPL RRTConnect search, Ruckig timing and
quantized re-validation as the legacy preview/executor. Listens on loopback
only; the ROS station node (container, --net=host) is its single client.

  arm_control/.venv/bin/python ros2_robot/postal_station/planner_worker/worker.py \
      --calibration arm_control/calibration-candidate-...local.json \
      --scene arm_control/assets/rover-fixed.local/rover-scene.local.json
"""
import argparse
import copy
import hashlib
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'arm_control'))

from planner import Arm, joints_to_ticks, load_config, require_calibration, ticks_to_joints  # noqa: E402

URDF = REPO / 'arm_control/.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'
MONITOR_SLACK_M = 0.002  # the live collision monitor uses (plan margin - this)


class Planner:
    def __init__(self, calibration, scene):
        from collision import CollisionWorld
        self.calibration_path, self.scene_path = Path(calibration).resolve(), Path(scene).resolve()
        self.config = load_config(self.calibration_path)
        require_calibration(self.config)
        self.calibration_sha = hashlib.sha256(self.calibration_path.read_bytes()).hexdigest()
        self.arm = Arm(URDF, self.config)
        self.world = CollisionWorld(URDF, self.arm, self.scene_path)
        self.arm.collision = self.world
        self.gpu, self.gpu_error = None, ''
        if os.environ.get('POSTAL_PLANNER_BACKEND', 'curobo') == 'curobo':
            try:  # cuRobo accelerator; any failure keeps the exact CPU pipeline (OMPL + PyBullet)
                from curobo_backend import CuroboWorld
                from gpu_planning import GpuPlanning
                self.gpu = GpuPlanning(CuroboWorld(self.scene_path, self.arm.bounds), self.world)
                self.gpu.gpu.valid_batch(self.arm.bounds.mean(1)[None])  # warm-up (CUDA kernels)
            except Exception as exc:  # noqa: BLE001 - reported in /status
                self.gpu, self.gpu_error = None, f'{type(exc).__name__}: {exc}'
                print(f'cuRobo backend unavailable, CPU pipeline only: {self.gpu_error}', flush=True)
        self.lock = threading.Lock()
        self.ready_cache, self.ready_time = None, 0.
        import xml.etree.ElementTree as ET
        limits = [float(j.find('limit').get('velocity')) for j in ET.parse(URDF).getroot().findall('joint')
                  if j.get('name') in [f'rev_motor_{i:02d}' for i in range(1, 8)]]
        self.urdf_velocity = min(limits)

    def camera_pose(self, ticks):
        """T_base_camera (optical frame) for the arm at these ticks, from the measured camera mount."""
        import slot_ik
        if getattr(self, 'camera_mount', None) is None:
            raise ValueError('No camera mount configured (arm_control/camera-mount.local.json)')
        self._check_ticks(ticks[:7], 'Current arm')
        with self.lock:
            q = ticks_to_joints(self.config, ticks[:7])
            return {'T_base_camera': slot_ik.camera_pose(self.arm, q, self.camera_mount).tolist()}

    def plan_via(self, start_ticks, via, goal_ticks, speed_factor=1., accel_factor=1., jerk_factor=1.):
        """One trajectory start -> via[0] -> ... -> goal, each leg planned (collision-free) and joined; the
        arm is at rest at every junction (e.g. the wrist pivot is finished before the arm moves on)."""
        waypoints = [list(start_ticks)] + [list(v) for v in via] + [list(goal_ticks)]
        legs = [self.plan(a, b, speed_factor, accel_factor, jerk_factor) for a, b in zip(waypoints, waypoints[1:])]
        times, positions, offset = [], [], 0.
        for k, leg in enumerate(legs):
            skip = 1 if k else 0  # the first sample of a leg is the last of the previous one
            times += [offset + t for t in leg['times'][skip:]]
            positions += leg['positions'][skip:]
            offset += leg['times'][-1]
        last = legs[-1]
        return dict(last, times=times, positions=positions, duration_s=float(offset),
                    reasons=[r for leg in legs for r in leg['reasons']],
                    executable=all(leg['executable'] for leg in legs), waypoints=sum(leg['waypoints'] for leg in legs),
                    planning_s=sum(leg['planning_s'] for leg in legs),
                    collision_checks=sum(leg['collision_checks'] for leg in legs), legs=len(legs))

    def pivot_ticks(self, ticks, delta_rad):
        """The same arm configuration with J7 turned by delta_rad (the wrist pivot, in place)."""
        self._check_ticks(ticks[:7], 'Pivot start')
        with self.lock:
            q = ticks_to_joints(self.config, ticks[:7])
            q[6] += float(delta_rad)
            goal = [int(t) for t in joints_to_ticks(self.config, self.arm.check(q))]
            self._check_ticks(goal, 'Pivot goal')
            return {'goal_ticks': goal}

    def ik_xyz(self, start_ticks, target_xyz, tcp_offset_m, approach_dir=None, pivot_rad=0., pivot_reference_ticks=None):
        """Goal joint ticks putting the gripper fingertips on a base_link point, J7 imposed at its
        reference angle plus the wrist pivot (the reference is the J7 of pivot_reference_ticks, e.g. the
        `scan` pose, or the current angle). The path itself is planned by plan() (collision-free)."""
        import slot_ik
        self._check_ticks(start_ticks[:7], 'Current arm')
        offset = [float(v) for v in tcp_offset_m]
        if len(offset) != 3 or not all(abs(v) <= 0.3 for v in offset):
            raise ValueError('tcp_offset_m: three values within +/-0.3 m')
        with self.lock:
            q0 = ticks_to_joints(self.config, start_ticks[:7])
            if pivot_reference_ticks:
                self._check_ticks(pivot_reference_ticks[:7], 'Pivot reference')
                reference = float(ticks_to_joints(self.config, pivot_reference_ticks[:7])[6])
            else:
                reference = float(q0[6])
            q, info = slot_ik.solve_tcp_ik(self.arm, q0, target_xyz, offset, approach_dir,
                                           pinned={6: reference + float(pivot_rad)})
            goal_ticks = [int(t) for t in joints_to_ticks(self.config, q)]
            self._check_ticks(goal_ticks, 'Goal')
            T = self.arm.fk(q)
            tip = T[:3, 3] + T[:3, :3] @ offset
            return dict(goal_ticks=goal_ticks, goal_q_rad=[float(v) for v in q], tip_xyz_m=[float(v) for v in tip],
                        gripper_axis=[float(v) for v in T[:3, :3] @ slot_ik.FORWARD], **info)

    def readiness(self):
        """Scene readiness (hashes every geometry file): cached 20 s for /status.
        Plans always re-run require_hardware_ready themselves."""
        if self.ready_cache is None or time.monotonic() - self.ready_time > 20:
            with self.lock:
                try:
                    self.world.require_hardware_ready()
                    self.ready_cache = (True, 'Scene, registration and geometry fingerprints verified')
                except ValueError as exc:
                    self.ready_cache = (False, str(exc))
                self.ready_time = time.monotonic()
        return self.ready_cache

    def status(self):
        ready, reason = self.readiness()
        s = self.world.scene
        return dict(scene=str(self.scene_path), scene_sha256=self.world.fingerprint,
                    calibration=str(self.calibration_path), calibration_sha256=self.calibration_sha,
                    hardware_ready=ready, reason=reason, margin_m=s['margin_m'],
                    self_margin_m=s['self_margin_m'], edge_step_rad=s['edge_step_rad'],
                    gripper_angle_rad=s['gripper_angle_rad'], planning_time_s=s['planning_time_s'],
                    registration=s.get('registration', {}).get('method', ''),
                    max_velocity_rad_s=self.config['max_velocity_rad_s'],
                    backend='curobo' if self.gpu else 'cpu', backend_error=self.gpu_error)

    def _check_ticks(self, ticks, label):
        if len(ticks) != 7 or any(type(t) is not int for t in ticks):
            raise ValueError(f'{label}: seven integer encoder values required')
        for j, t in zip(self.config['joints'], ticks):
            if not j['min_ticks'] <= t <= j['max_ticks']:
                raise ValueError(f"{label}: {j['name']} at {t} ticks is outside the operating margin "
                                 f"{j['min_ticks']}..{j['max_ticks']}; not clipped. Reposition first.")

    def plan(self, start_ticks, goal_ticks, speed_factor=1., accel_factor=1., jerk_factor=1.):
        """Speed scales the velocity limit only. Acceleration and jerk have their own factors
        (default 1 = the calibrated values): the servo torque peaks come from acceleration, so a
        faster plan does not have to accelerate harder."""
        f, fa, fj = float(speed_factor), float(accel_factor or 1.), float(jerk_factor or 1.)
        if not (0.1 <= f <= self.max_speed_factor):
            raise ValueError(f'speed_factor must be within 0.1..{self.max_speed_factor}')
        if not (0.1 <= fa <= 4.) or not (0.1 <= fj <= 4.):
            raise ValueError('accel_factor and jerk_factor must be within 0.1..4')
        from planner import joint_limit_scales
        vscale = float(joint_limit_scales(self.config)['velocity_scale'].max())
        if self.config['max_velocity_rad_s'] * f * vscale > self.urdf_velocity:
            raise ValueError('Requested speed exceeds the URDF joint velocity limit')
        self._check_ticks(start_ticks, 'Current arm')
        self._check_ticks(goal_ticks, 'Target pose')
        with self.lock:
            begin = time.monotonic()
            c, arm, world = self.config, self.arm, self.world
            start, end = ticks_to_joints(c, start_ticks), ticks_to_joints(c, goal_ticks)
            gpu, backend = self.gpu, 'cpu'
            path = None
            if gpu is not None:
                gpu.exact_checks = 0
                path = gpu.joint_path(start, end)
                backend = 'curobo'
            if path is None:  # no GPU, or no direct / one-via path: exact OMPL search
                path = world.joint_path(start, end)  # geometry does not depend on speed
                if gpu is not None:
                    backend = 'curobo+ompl'
            keys = ('max_velocity_rad_s', 'max_acceleration_rad_s2', 'max_jerk_rad_s3')
            saved = {k: c[k] for k in keys}
            rows = []
            try:
                for k, factor in zip(keys, (f, fa, fj)):  # Ruckig limits only; restored before readiness checks
                    c[k] = saved[k] * factor
                for a, b in zip(path, path[1:]):
                    segment = arm.segment(a, b)
                    offset = rows[-1]['t_s'] if rows else 0.
                    for row in (segment[1:] if rows else segment):
                        row['t_s'] += offset
                        rows.append(row)
            finally:
                c.update(saved)
            return self._finish(rows, end, len(path), begin, backend, f, fa, fj)

    def _finish(self, rows, end, waypoints, begin, backend, f, fa, fj):
        """Common end of a plan: validate the timed rows AND their tick-quantized version (what the servos
        will really be asked to do), compute the compensation bounds, pack the reply. Called with the lock."""
        c, arm, world, gpu = self.config, self.arm, self.world, self.gpu
        plan = dict(samples=rows, collision_checked=True, collision_scene_sha256=world.fingerprint,
                    targets_xyz_m=[arm.fk(end)[:3, 3].tolist()])
        quantized = copy.deepcopy(plan)
        for row in quantized['samples']:
            row['q_rad'] = ticks_to_joints(c, joints_to_ticks(c, row['q_rad'])).tolist()
        if gpu is not None:
            try:
                gpu.validate_rows(plan['samples'])
                gpu.validate_rows(quantized['samples'])  # what the servos will actually be asked to do
            except ValueError:
                # The GPU spheres are conservative (~7 % false alarms): the exact checker decides.
                world.validate_plan(plan)
                world.validate_plan(quantized)
                backend += '+exact-validation'
            compensation, note = self._compensation_box_gpu(end)
        else:
            world.validate_plan(plan)
            world.validate_plan(quantized)  # what the servos will actually be asked to do
            compensation, note = self._compensation_box(end)
        reasons = []
        try:
            world.require_hardware_ready()
        except ValueError as exc:
            reasons.append(str(exc))
        return dict(times=[float(r['t_s']) for r in quantized['samples']],
                    positions=[[float(v) for v in r['q_rad']] for r in quantized['samples']],
                    end_ticks=[int(t) for t in joints_to_ticks(c, quantized['samples'][-1]['q_rad'])],
                    duration_s=float(rows[-1]['t_s']), collision_scene_sha256=world.fingerprint,
                    calibration_sha256=self.calibration_sha, waypoints=waypoints,
                    reasons=reasons, executable=not reasons,
                    compensation_limit_rad=compensation, compensation_note=note,
                    planning_s=time.monotonic() - begin, collision_checks=world.checks,
                    speed_factor=f, accel_factor=fa, jerk_factor=fj, backend=backend,
                    exact_checks=gpu.exact_checks if gpu is not None else world.checks)

    def tcp_error(self, start_ticks, ideal_delta_m, measured_ticks):
        """Where the fingertips are (measured ticks) minus where a straight line of ideal_delta_m (base_link, m)
        from start_ticks should end, in mm (x, y, z). Positive z = higher than the ideal end."""
        import numpy as np
        self._check_ticks(start_ticks, 'Line start')
        self._check_ticks(measured_ticks, 'Measured arm')
        delta = np.array([float(v) for v in ideal_delta_m])
        if delta.shape != (3,):
            raise ValueError('ideal_delta_m: three values')
        with self.lock:
            ideal = self.arm.fk(ticks_to_joints(self.config, start_ticks))[:3, 3] + delta
            got = self.arm.fk(ticks_to_joints(self.config, measured_ticks))[:3, 3]
        return dict(error_mm=[round(float(v), 2) for v in (got - ideal) * 1000.])

    def plan_line(self, start_ticks, delta_m, speed_factor=1., accel_factor=1., jerk_factor=1.,
                  max_error_mm=1.5, max_rotation_deg=15.):
        """A STRAIGHT Cartesian line of the TCP from the arm configuration `start_ticks`: it moves by
        `delta_m` (base_link, metres), as ONE smooth profile (no stop on the way). The position is exact; the
        gripper orientation is held as well as the joint limits allow. The planned joint trajectory is checked
        after the tick quantization the servos will execute: the plan is refused if that path leaves the ideal
        line by more than max_error_mm or turns the gripper by more than max_rotation_deg."""
        import numpy as np
        from scipy.spatial.transform import Rotation
        f, fa, fj = float(speed_factor), float(accel_factor or 1.), float(jerk_factor or 1.)
        if not (0.1 <= f <= self.max_speed_factor):
            raise ValueError(f'speed_factor must be within 0.1..{self.max_speed_factor}')
        if not (0.1 <= fa <= 4.) or not (0.1 <= fj <= 4.):
            raise ValueError('accel_factor and jerk_factor must be within 0.1..4')
        delta = [float(v) for v in delta_m]
        if len(delta) != 3 or not all(abs(v) <= .5 for v in delta):
            raise ValueError('delta_m: three values within +/-0.5 m')
        from planner import joint_limit_scales
        vscale = float(joint_limit_scales(self.config)['velocity_scale'].max())
        if self.config['max_velocity_rad_s'] * f * vscale > self.urdf_velocity:
            raise ValueError('Requested speed exceeds the URDF joint velocity limit')
        self._check_ticks(start_ticks, 'Current arm')
        with self.lock:
            begin = time.monotonic()
            c, arm, world, gpu = self.config, self.arm, self.world, self.gpu
            if gpu is not None:
                gpu.exact_checks = 0
            start = ticks_to_joints(c, start_ticks)
            points = arm.line_waypoints(start, delta, step_m=0.003, rotation_tol_rad=float(np.radians(max_rotation_deg)))
            end = points[-1]
            keys = ('max_velocity_rad_s', 'max_acceleration_rad_s2', 'max_jerk_rad_s3')
            saved = {k: c[k] for k in keys}
            try:
                for k, factor in zip(keys, (f, fa, fj)):
                    c[k] = saved[k] * factor
                rows = arm.timed_path(points)
            finally:
                c.update(saved)
            # accuracy of what the servos will execute (ticks), against the ideal line and the start orientation
            T0 = arm.fk(start)
            p0, R0 = T0[:3, 3], T0[:3, :3]
            length = float(np.linalg.norm(delta))
            direction = np.array(delta) / length
            worst_mm = worst_deg = 0.
            for row in rows:
                T = arm.fk(ticks_to_joints(c, joints_to_ticks(c, row['q_rad'])))
                offset = T[:3, 3] - p0
                along = float(offset @ direction)
                if not -0.0005 <= along <= length + 0.0005:
                    worst_mm = max(worst_mm, 1000 * max(-along, along - length))
                worst_mm = max(worst_mm, 1000 * float(np.linalg.norm(offset - along * direction)))
                worst_deg = max(worst_deg, float(np.degrees(Rotation.from_matrix(R0.T @ T[:3, :3]).magnitude())))
            if worst_mm > max_error_mm or worst_deg > max_rotation_deg:
                raise ValueError(f'The line is not precise enough after the servo tick quantization: '
                                 f'{worst_mm:.2f} mm off the line (limit {max_error_mm}), gripper turned {worst_deg:.2f}° '
                                 f'(limit {max_rotation_deg})')
            result = self._finish(rows, end, len(points), begin, 'curobo-line' if gpu is not None else 'cpu-line', f, fa, fj)
            result.update(line_length_m=length, line_error_mm=round(worst_mm, 3), line_rotation_deg=round(worst_deg, 3))
            return result

    def _compensation_box_gpu(self, final):
        def in_range(q):
            try:
                joints_to_ticks(self.config, self.arm.check(q))
                return True
            except ValueError:
                return False
        bounds = self.gpu.compensation_box(final, in_range)
        if not any(bounds):
            return [0.] * 7, 'compensation unavailable (no validated collision-free bound around the target)'
        text = ', '.join(f'J{j + 1} {b:g}' for j, b in enumerate(bounds) if b)
        return bounds, f'static-error compensation (rad) {text} validated'

    def _compensation_box(self, final, limit=.06):
        """Validate the per-joint static-error compensation bounds around the final pose.

        Each joint gets the largest bound (<= limit) whose motion from the final pose
        to final +/- bound is within calibration and collision-free; then every
        vertex of the resulting box (and the straight motion to it) must be valid,
        halving all bounds until it is. Poses next to an obstacle therefore keep a
        smaller, still validated, bound instead of losing compensation altogether."""
        import itertools
        import numpy as np
        final = np.array(final, dtype=float)

        def valid(q):
            try:
                joints_to_ticks(self.config, self.arm.check(q))
            except ValueError:
                return False
            return self.world.motion_valid(final, q)

        bounds = [0.] * 7
        for j in range(7):
            for b in (limit, .04, .025, .012):
                ok = True
                for d in (-b, b):
                    q = final.copy()
                    q[j] += d
                    ok = ok and valid(q)
                if ok:
                    bounds[j] = b
                    break
        for _ in range(4):
            active = [j for j in range(7) if bounds[j] > 0]
            if not active:
                break
            if all(valid(final + np.array([d * bounds[j] if j in active else 0. for j, d in
                                           zip(range(7), signs)]))
                   for signs in itertools.product((-1., 1.), repeat=7) if all(
                       signs[j] == 1. for j in range(7) if j not in active)):
                break
            bounds = [b / 2 if b / 2 >= .006 else 0. for b in bounds]
        else:
            bounds = [0.] * 7
        if not any(bounds):
            return [0.] * 7, 'compensation unavailable (no validated collision-free bound around the target)'
        text = ', '.join(f'J{j + 1} {b:g}' for j, b in enumerate(bounds) if b)
        return bounds, f'static-error compensation (rad) {text} validated'

    def check(self, ticks):
        """Collision check of a MEASURED pose (execution monitor)."""
        with self.lock:
            try:
                q = self.arm.check(ticks_to_joints(self.config, ticks))
            except ValueError as exc:
                return dict(valid=False, reason=f'outside calibrated range: {exc}')
            # The execution monitor tolerates MONITOR_SLACK_M less margin than the plans were made with: servo
            # tracking and gravity sag move the real arm a millimetre or two (4.9 mm from a box with a 5 mm margin
            # cancelled a safe move). The plans themselves keep the full margin.
            world, saved = self.world, self.world.scene['margin_m']
            world.scene['margin_m'] = max(0.002, saved - MONITOR_SLACK_M)
            world.__dict__.pop('_last_q', None)  # the one-pose cache was computed with another margin
            try:
                reason = world.collision(q)
            finally:
                world.scene['margin_m'] = saved
                world.__dict__.pop('_last_q', None)
            return dict(valid=reason is None, reason=reason or '')


def serve(planner, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, body):
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/status':
                return self.reply(200, planner.status())
            self.reply(404, {'error': 'not found'})

        def do_POST(self):
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                if self.path == '/plan':
                    return self.reply(200, planner.plan(body['start_ticks'], body['goal_ticks'],
                                                        body.get('speed_factor', 1.),
                                                        body.get('accel_factor', 1.),
                                                        body.get('jerk_factor', 1.)))
                if self.path == '/tcp_error':
                    return self.reply(200, planner.tcp_error(body['start_ticks'], body['ideal_delta_m'], body['measured_ticks']))
                if self.path == '/plan_line':
                    return self.reply(200, planner.plan_line(body['start_ticks'], body['delta_m'],
                                                             body.get('speed_factor', 1.), body.get('accel_factor', 1.),
                                                             body.get('jerk_factor', 1.)))
                if self.path == '/plan_via':
                    return self.reply(200, planner.plan_via(body['start_ticks'], body['via_ticks'], body['goal_ticks'],
                                                            body.get('speed_factor', 1.), body.get('accel_factor', 1.),
                                                            body.get('jerk_factor', 1.)))
                if self.path == '/pivot_ticks':
                    return self.reply(200, planner.pivot_ticks(body['ticks'], body['delta_rad']))
                if self.path == '/camera_pose':
                    return self.reply(200, planner.camera_pose(body['ticks']))
                if self.path == '/ik_xyz':
                    return self.reply(200, planner.ik_xyz(body['start_ticks'], body['target_xyz_m'],
                                                          body['tcp_offset_m'], body.get('approach_dir'),
                                                          body.get('pivot_rad', 0.),
                                                          body.get('pivot_reference_ticks')))
                if self.path == '/check':
                    return self.reply(200, planner.check(body['ticks']))
                self.reply(404, {'error': 'not found'})
            except (ValueError, KeyError, TypeError) as exc:
                self.reply(400, {'error': str(exc)})

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    print(f'Planner worker on http://127.0.0.1:{port} ({planner.scene_path.name})', flush=True)
    server.serve_forever()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--port', type=int, default=8893)
    parser.add_argument('--camera-mount', help='measured {parent_link, T_parent_camera} JSON (camera 2)')
    parser.add_argument('--max-speed-factor', type=float, default=2.,
                        help='must not exceed the robot max_speed_factor parameter')
    args = parser.parse_args()
    planner = Planner(args.calibration, args.scene)
    planner.max_speed_factor = args.max_speed_factor
    planner.camera_mount = json.loads(Path(args.camera_mount).read_text()) if args.camera_mount else None
    serve(planner, args.port)


if __name__ == '__main__':
    main()
