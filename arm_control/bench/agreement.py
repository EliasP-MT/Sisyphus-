"""Agreement of the cuRobo GPU checker with the exact PyBullet checker (collision.py)."""
import json, sys, time
from pathlib import Path
import numpy as np
REPO = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO/'ros2_robot/postal_station/planner_worker'), str(REPO/'arm_control')]
import worker
from curobo_backend import CuroboWorld
CAL = REPO/'arm_control/calibration-candidate-20261115-105050-197423.local.json'
SCENE = REPO/'arm_control/assets/rover-fixed.local/rover-scene.local.json'
P = worker.Planner(CAL, SCENE)
t = time.monotonic(); G = CuroboWorld(SCENE, P.arm.bounds); print(f'curobo init {time.monotonic()-t:.1f}s')
N = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
rng = np.random.default_rng(0)
poses = {k: [v['ticks'][str(i)] for i in range(1, 8)] for k, v in json.loads((REPO/'arm_control/poses.local.json').read_text())['poses'].items()}
anchors = [worker.ticks_to_joints(P.config, t) for t in poses.values()]
Q = [rng.uniform(P.arm.bounds[:, 0], P.arm.bounds[:, 1]) for _ in range(N//2)]
Q += [np.clip(anchors[i % 3] + rng.normal(0, 0.4, 7), P.arm.bounds[:, 0], P.arm.bounds[:, 1]) for i in range(N - N//2)]
Q = np.array(Q)
G.valid_batch(Q[:10])
t = time.monotonic(); g = G.valid_batch(Q); tg = time.monotonic()-t
t = time.monotonic(); e = np.array([P.world.valid(q) for q in Q]); te = time.monotonic()-t
reasons = {}
for q, gi, ei in zip(Q, g, e):
    if gi and not ei:
        r = P.world.collision(q).split('(')[0].strip(); reasons[r] = reasons.get(r, 0)+1
print(f'N={N} gpu {tg*1000:.0f} ms ({tg/N*1e6:.0f} us/q)  exact {te:.1f} s ({te/N*1e3:.1f} ms/q)')
print(f'exact valid {e.mean():.1%}  gpu valid {g.mean():.1%}')
print(f'UNSAFE gpu-valid/exact-collision: {(g & ~e).sum()}   conservative gpu-collision/exact-valid: {(~g & e).sum()}')
print('unsafe reasons:', reasons)
_, f = G.valid_batch(Q, details=True)
print('conservative by test:', {k: int((v & e).sum()) for k, v in f.items()})
