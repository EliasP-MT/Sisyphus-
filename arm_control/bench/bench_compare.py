"""Planner worker: cuRobo backend vs CPU pipeline on recorded poses and random collision-free pairs."""
import json, os, sys, time
from pathlib import Path
import numpy as np
REPO = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO/'ros2_robot/postal_station/planner_worker'), str(REPO/'arm_control')]
import worker
CAL = REPO/'arm_control/calibration-candidate-20261115-105050-197423.local.json'
SCENE = REPO/'arm_control/assets/rover-fixed.local/rover-scene.local.json'
n_random = int(sys.argv[1]) if len(sys.argv) > 1 else 4
backends = sys.argv[2].split(',') if len(sys.argv) > 2 else ['curobo', 'cpu']
planners = {}
for b in backends:
    os.environ['POSTAL_PLANNER_BACKEND'] = b
    t = time.monotonic(); planners[b] = worker.Planner(CAL, SCENE); planners[b].max_speed_factor = 2.
    print(f'{b} init {time.monotonic()-t:.1f}s gpu={planners[b].gpu is not None} {planners[b].gpu_error}')
P = planners[backends[0]]
poses = {k: [v['ticks'][str(i)] for i in range(1, 8)] for k, v in json.loads((REPO/'arm_control/poses.local.json').read_text())['poses'].items()}
pairs = [('vertical', poses['vertical'], 'scan', poses['scan']), ('scan', poses['scan'], 'vertical', poses['vertical'])]
rng = np.random.default_rng(3)
while len(pairs) < 2 + n_random:
    qs = [rng.uniform(P.arm.bounds[:, 0], P.arm.bounds[:, 1]) for _ in range(2)]
    if all(P.world.valid(q) for q in qs):
        try:
            t = [worker.joints_to_ticks(P.config, q) for q in qs]
        except ValueError:
            continue
        pairs.append((f'rand{len(pairs)}a', t[0], f'rand{len(pairs)}b', t[1]))
for na, a, nb, b in pairs:
    line = f'{na:>9}->{nb:<9}'
    for name, pl in planners.items():
        try:
            r = pl.plan(list(map(int, a)), list(map(int, b)))
            line += f" | {name}: plan {r['planning_s']:6.2f}s motion {r['duration_s']:5.1f}s wp {r['waypoints']} {r['backend']} exact {r['exact_checks']} comp {sum(r['compensation_limit_rad']):.3f}"
        except ValueError as e:
            line += f' | {name}: ERR {str(e)[:60]}'
    print(line, flush=True)
