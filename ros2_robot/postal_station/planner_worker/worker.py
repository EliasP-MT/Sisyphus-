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

    def status(self):
        with self.lock:
            try:
                self.world.require_hardware_ready()
                ready, reason = True, 'Scene, registration and geometry fingerprints verified'
            except ValueError as exc:
                ready, reason = False, str(exc)
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

    def plan(self, start_ticks, goal_ticks):
        self._check_ticks(start_ticks, 'Current arm')
        self._check_ticks(goal_ticks, 'Target pose')
        with self.lock:
            begin = time.monotonic()
            c, arm, world = self.config, self.arm, self.world
            start, end = ticks_to_joints(c, start_ticks), ticks_to_joints(c, goal_ticks)
            path = world.joint_path(start, end)
            rows = []
            for a, b in zip(path, path[1:]):
                segment = arm.segment(a, b)
                offset = rows[-1]['t_s'] if rows else 0.
                for row in (segment[1:] if rows else segment):
                    row['t_s'] += offset
                    rows.append(row)
            plan = dict(samples=rows, collision_checked=True, collision_scene_sha256=world.fingerprint,
                        targets_xyz_m=[arm.fk(end)[:3, 3].tolist()])
            world.validate_plan(plan)
            quantized = copy.deepcopy(plan)
            for row in quantized['samples']:
                row['q_rad'] = ticks_to_joints(c, joints_to_ticks(c, row['q_rad'])).tolist()
            world.validate_plan(quantized)  # what the servos will actually be asked to do
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
                        planning_s=time.monotonic() - begin, collision_checks=world.checks)

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
                    return self.reply(200, planner.plan(body['start_ticks'], body['goal_ticks']))
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
    args = parser.parse_args()
    serve(Planner(args.calibration, args.scene), args.port)


if __name__ == '__main__':
    main()
