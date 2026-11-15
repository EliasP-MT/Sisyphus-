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
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'arm_control'))

from planner import Arm, joints_to_ticks, load_config, require_calibration, ticks_to_joints  # noqa: E402

URDF = REPO / 'arm_control/.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'


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
        self.lock = threading.Lock()
        self.ready_cache, self.ready_time = None, 0.
        import xml.etree.ElementTree as ET
        limits = [float(j.find('limit').get('velocity')) for j in ET.parse(URDF).getroot().findall('joint')
                  if j.get('name') in [f'rev_motor_{i:02d}' for i in range(1, 8)]]
        self.urdf_velocity = min(limits)

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
                    max_velocity_rad_s=self.config['max_velocity_rad_s'])

    def _check_ticks(self, ticks, label):
        if len(ticks) != 7 or any(type(t) is not int for t in ticks):
            raise ValueError(f'{label}: seven integer encoder values required')
        for j, t in zip(self.config['joints'], ticks):
            if not j['min_ticks'] <= t <= j['max_ticks']:
                raise ValueError(f"{label}: {j['name']} at {t} ticks is outside the operating margin "
                                 f"{j['min_ticks']}..{j['max_ticks']}; not clipped. Reposition first.")

    def plan(self, start_ticks, goal_ticks, speed_factor=1.):
        f = float(speed_factor)
        if not (0.1 <= f <= self.max_speed_factor):
            raise ValueError(f'speed_factor must be within 0.1..{self.max_speed_factor}')
        if self.config['max_velocity_rad_s'] * f > self.urdf_velocity:
            raise ValueError('Requested speed exceeds the URDF joint velocity limit')
        self._check_ticks(start_ticks, 'Current arm')
        self._check_ticks(goal_ticks, 'Target pose')
        with self.lock:
            begin = time.monotonic()
            c, arm, world = self.config, self.arm, self.world
            start, end = ticks_to_joints(c, start_ticks), ticks_to_joints(c, goal_ticks)
            path = world.joint_path(start, end)  # geometry does not depend on speed
            keys = ('max_velocity_rad_s', 'max_acceleration_rad_s2', 'max_jerk_rad_s3')
            saved = {k: c[k] for k in keys}
            rows = []
            try:
                for k in keys:  # Ruckig limits only; restored before readiness checks
                    c[k] = saved[k] * f
                for a, b in zip(path, path[1:]):
                    segment = arm.segment(a, b)
                    offset = rows[-1]['t_s'] if rows else 0.
                    for row in (segment[1:] if rows else segment):
                        row['t_s'] += offset
                        rows.append(row)
            finally:
                c.update(saved)
            plan = dict(samples=rows, collision_checked=True, collision_scene_sha256=world.fingerprint,
                        targets_xyz_m=[arm.fk(end)[:3, 3].tolist()])
            world.validate_plan(plan)
            quantized = copy.deepcopy(plan)
            for row in quantized['samples']:
                row['q_rad'] = ticks_to_joints(c, joints_to_ticks(c, row['q_rad'])).tolist()
            world.validate_plan(quantized)  # what the servos will actually be asked to do
            compensation, note = self._compensation_box(end)
            reasons = []
            try:
                world.require_hardware_ready()
            except ValueError as exc:
                reasons.append(str(exc))
            return dict(times=[float(r['t_s']) for r in quantized['samples']],
                        positions=[[float(v) for v in r['q_rad']] for r in quantized['samples']],
                        duration_s=float(rows[-1]['t_s']), collision_scene_sha256=world.fingerprint,
                        calibration_sha256=self.calibration_sha, waypoints=len(path),
                        reasons=reasons, executable=not reasons,
                        compensation_limit_rad=compensation, compensation_note=note,
                        planning_s=time.monotonic() - begin, collision_checks=world.checks,
                        speed_factor=f)

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
            reason = self.world.collision(q)
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
                                                        body.get('speed_factor', 1.)))
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
    parser.add_argument('--max-speed-factor', type=float, default=2.,
                        help='must not exceed the robot max_speed_factor parameter')
    args = parser.parse_args()
    planner = Planner(args.calibration, args.scene)
    planner.max_speed_factor = args.max_speed_factor
    serve(planner, args.port)


if __name__ == '__main__':
    main()
