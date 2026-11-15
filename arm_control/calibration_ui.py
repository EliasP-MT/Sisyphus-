#!/usr/bin/env python3
"""Local calibration UI; reads Station telemetry and never sends commands."""
import argparse
import asyncio
from dataclasses import asdict
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'teleop'))
from teleop import Station

ROOT = Path(__file__).resolve().parent


def candidate(base, incoming):
    result = json.loads(json.dumps(base))
    joints = incoming['joints']
    if len(joints) != 7:
        raise ValueError('Expected seven joints')
    for index, (target, source) in enumerate(zip(result['joints'], joints), 1):
        if source['motor_id'] != target['motor_id']:
            raise ValueError('Changing motor IDs is not allowed here')
        zero, scale, sign = source['zero_ticks'], source['ticks_per_rad'], source['sign']
        if (type(zero) is not int or not 0 <= zero <= 4095 or sign not in (-1, 1)
                or type(sign) is not int or not math.isfinite(scale) or not 100 <= scale <= 5000):
            raise ValueError(f'Invalid calibration for J{index}')
        target.update(zero_ticks=zero, sign=sign, ticks_per_rad=scale)
    result['max_velocity_rad_s'] = min(result['max_velocity_rad_s'],
        .95 * result['servo_speed_ticks_s'] / max(j['ticks_per_rad'] for j in result['joints']))
    result.update(hardware_verified=False, calibration_source='interactive candidate; requires independent validation')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1:18888')
    parser.add_argument('--port', type=int, default=8891)
    parser.add_argument('--config', type=Path, default=ROOT/'elrobot.local.json')
    parser.add_argument('--scene', type=Path, help='Registered collision scene for motion preview')
    parser.add_argument('--verified-config', type=Path, help='Independently verified candidate; required for real execution')
    parser.add_argument('--poses',type=Path,default=ROOT/'poses.local.json')
    parser.add_argument('--cad-root',type=Path,default=ROOT/'assets/full_robot_box_complete_collision')
    args = parser.parse_args()
    base = json.loads(args.config.read_text())
    state = dict(timestamp=0, motors={}, error='Connecting to Station')
    lock = threading.Lock()
    pose_lock = threading.Lock()
    from ui_motion import Motions
    motions = Motions(ROOT, args.host, args.scene, args.verified_config)

    async def poll():
        while True:
            station = None
            try:
                station = await Station.connect(args.host)
                while True:
                    await station.wait_frame(timeout=3)
                    motors = station.buses.get(base['bus'], {})
                    with lock:
                        state.update(timestamp=time.time(), motors={i: asdict(m) for i, m in motors.items()}, error=None)
            except (Exception, SystemExit) as exc:
                with lock:
                    state['error'] = str(exc)
                await asyncio.sleep(2)
            finally:
                if station:
                    station._task.cancel()
                    await asyncio.gather(station._task, return_exceptions=True)
                    station.client.close_connection()

    class Handler(BaseHTTPRequestHandler):
        def reply(self, value, status=200):
            payload = json.dumps(value).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if self.path == '/cad/scene':
                if not args.scene:
                    self.reply({'surface_meshes': []})
                    return
                scene = json.loads(args.scene.read_text())
                if 'display_surface_meshes' in scene:
                    scene['surface_meshes'] = scene['display_surface_meshes']
                for i, entry in enumerate(scene.get('surface_meshes', [])):
                    entry['mesh'] = f'/cad/fixed-{i}.obj'
                self.reply(scene)
            elif self.path.startswith('/cad/fixed-') and self.path.endswith('.obj'):
                try:
                    index = int(self.path[len('/cad/fixed-'):-4])
                    scene = json.loads(args.scene.read_text())
                    entries = scene.get('display_surface_meshes', scene.get('surface_meshes', []))
                    if not 0 <= index < len(entries):
                        raise ValueError('Invalid mesh')
                    data = (args.scene.parent/entries[index]['mesh']).read_bytes()
                except (ValueError, OSError, TypeError, KeyError, AttributeError):
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif self.path in ('/cad/assembly.obj', '/cad/assembly.json'):
                path = args.cad_root/self.path.rsplit('/', 1)[-1]
                if not path.exists():
                    self.send_error(404, 'Full assembly import not complete')
                    return
                data = path.read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json' if path.suffix == '.json' else 'text/plain')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif self.path == '/motion/status':
                self.reply(motions.status)
            elif self.path == '/state':
                with lock:
                    self.reply(state)
            elif self.path == '/config':
                self.reply(base)
            elif self.path == '/poses':
                path = args.poses
                self.reply(json.loads(path.read_text()) if path.exists() else {'poses': {}})
            elif self.path == '/':
                body = (ROOT/'calibration_ui.html').read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

        def do_POST(self):
            # Same-origin JSON only; no CORS and no hardware write endpoints.
            if self.path not in ('/save', '/pose', '/motion/preview', '/motion/execute') or self.headers.get('Content-Type') != 'application/json':
                self.send_error(400)
                return
            origin = self.headers.get('Origin')
            if origin and origin != f'http://{self.headers.get("Host")}':
                self.send_error(403)
                return
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length < 20000:
                    raise ValueError('Invalid request size')
                incoming = json.loads(self.rfile.read(length))
                if self.path == '/motion/execute':
                    self.reply(motions.execute(incoming['token']))
                    return
                if self.path == '/motion/preview':
                    config = candidate(base, incoming)
                    with lock:
                        snapshot = json.loads(json.dumps(state))
                    if snapshot['error'] or time.time()-snapshot['timestamp'] > 1:
                        raise ValueError('Live telemetry is stale')
                    current = snapshot['motors']
                    for i in range(1, 9):
                        m = current.get(str(i))
                        if not m or m['status'] or not m['calibrated']:
                            raise ValueError(f'ID {i}: missing, uncalibrated or error')
                    poses = json.loads(args.poses.read_text())
                    if poses.get('bus') != base['bus']:
                        raise ValueError('Saved poses belong to a different bus')
                    self.reply(motions.preview(config, current, poses['poses'][incoming['target']]))
                    return
                if self.path == '/pose':
                    name = incoming.get('name', '').strip()
                    if not name or len(name) > 60 or any(not (c.isalnum() or c in '_- ') for c in name):
                        raise ValueError('Use a pose name of 1–60 letters, digits, spaces, _ or -')
                    # Require several new frames before accepting the capture.
                    samples = []
                    stamp = 0
                    deadline = time.monotonic() + 3
                    while len(samples) < 3 and time.monotonic() < deadline:
                        with lock:
                            snapshot = json.loads(json.dumps(state))
                        if snapshot['error'] or time.time() - snapshot['timestamp'] > 1:
                            raise ValueError('Live telemetry unavailable or stale')
                        if snapshot['timestamp'] > stamp:
                            stamp = snapshot['timestamp']
                            motors = snapshot['motors']
                            for i in range(1, 9):
                                m = motors.get(str(i))
                                if not m or m['status'] or not m['calibrated']:
                                    raise ValueError(f'ID {i}: missing, error or unfrozen calibration')
                                if not m['range_min'] < m['range_max'] or not m['range_min'] <= m['position'] <= m['range_max']:
                                    raise ValueError(f'ID {i}: outside usable calibration')
                            samples.append(motors)
                        time.sleep(.15)
                    if len(samples) < 3 or any(max(s[str(i)]['position'] for s in samples) - min(s[str(i)]['position'] for s in samples) > 3 for i in range(1, 9)):
                        raise ValueError('Arm moved during recording; support and hold still')
                    motors = {str(i): samples[-1][str(i)] for i in range(1, 9)}
                    path = args.poses
                    with pose_lock:
                        data = json.loads(path.read_text()) if path.exists() else dict(schema='encoder-poses-v1', bus=base['bus'], model_calibrated=False, poses={})
                        if data.get('schema') != 'encoder-poses-v1' or data.get('bus') != base['bus']:
                            raise ValueError('Existing pose file has a different schema or bus')
                        exists = name in data['poses']
                        if exists != (incoming.get('replace') is True):
                            raise ValueError('Pose already exists; choose Update' if exists else 'Pose does not exist; choose Add')
                        data['poses'][name] = dict(recorded_at=datetime.now(timezone.utc).isoformat(), motors=motors, ticks={i: m['position'] for i, m in motors.items()})
                        if path.exists():
                            backup = ROOT/('poses-backup-' + datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S-%f') + '.local.json')
                            with backup.open('x') as stream:
                                stream.write(path.read_text())
                        temporary = path.with_suffix('.json.tmp')
                        with temporary.open('x') as stream:
                            json.dump(data, stream, indent=2)
                        temporary.replace(path)
                    self.reply({'saved_pose': name, 'poses': data['poses']})
                    return
                config = candidate(base, incoming)
                # Explicit operator verification is separate from numerical fitting.
                if incoming.get('verified') is True:
                    config['hardware_verified'] = True
                    config['verification_method'] = 'operator confirmed physical model calibration'
                name = 'calibration-candidate-' + datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S-%f') + '.local.json'
                with (ROOT/name).open('x') as stream:
                    json.dump(config, stream, indent=2)
                if config['hardware_verified']:
                    motions.verified_config = ROOT/name
                    base.clear()
                    base.update(config)
                self.reply({'saved': str(ROOT/name), 'hardware_verified': config['hardware_verified']})
            except (ValueError, KeyError, TypeError, OSError, RuntimeError, ImportError) as exc:
                if self.path == '/motion/execute':
                    motions.status = {'stage': 'error', 'message': str(exc)}
                self.reply({'error': str(exc)}, 400)

        def log_message(self, *_):
            pass

    threading.Thread(target=lambda: asyncio.run(poll()), daemon=True).start()
    print(f'Calibration UI: http://127.0.0.1:{args.port}', flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
