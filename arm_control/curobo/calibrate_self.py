"""Calibrate the GPU self-collision pair exemptions against the exact PyBullet checker (collision.py).

A link pair is exempted from the GPU self test when its spheres overlap in more than --threshold of the
poses the exact checker finds free (sphere over-approximation around short links). Exempted pairs remain
covered by the exact check that the planner runs on start, goal and sparse trajectory samples."""
import argparse, collections, hashlib, json, sys
from pathlib import Path
import numpy as np, torch
REPO = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO/'ros2_robot/postal_station/planner_worker'), str(REPO/'arm_control')]
import worker
import curobo_backend as cb

ap = argparse.ArgumentParser()
ap.add_argument('--calibration', default=str(REPO/'arm_control/calibration-candidate-20261115-105050-197423.local.json'))
ap.add_argument('--scene', default=str(REPO/'arm_control/assets/rover-fixed.local/rover-scene.local.json'))
ap.add_argument('-n', type=int, default=4000)
ap.add_argument('--threshold', type=float, default=0.01)
a = ap.parse_args()
cb.SELF_EXEMPT_JSON.unlink(missing_ok=True)
P = worker.Planner(a.calibration, a.scene)
G = cb.CuroboWorld(a.scene, P.arm.bounds)
rng = np.random.default_rng(1)
poses = json.loads((REPO/'arm_control/poses.local.json').read_text())['poses']
anchors = [worker.ticks_to_joints(P.config, [v['ticks'][str(i)] for i in range(1, 8)]) for v in poses.values()]
lo, hi = P.arm.bounds[:, 0], P.arm.bounds[:, 1]
Q = np.array([rng.uniform(lo, hi) if k % 2 else np.clip(anchors[k % len(anchors)] + rng.normal(0, .5, 7), lo, hi)
              for k in range(a.n)])
Q = Q[[P.world.valid(q) for q in Q]]
q = torch.as_tensor(Q, device=G.device, dtype=torch.float32)[:, None]
G.checker.setup_batch_tensors(len(Q), 1)
sp = G.checker.get_kinematics(q).robot_spheres[:, 0]
gap = torch.cdist(sp[..., :3], sp[..., :3]) - sp[..., 3, None] - sp[..., None, 3]
hit = (gap < G.self_margin) & G.self_pair_mask
L = G.link_of.cpu().numpy()
per_pair = collections.defaultdict(set)
for bi, i, j in hit.nonzero().cpu().numpy():
    per_pair[tuple(sorted((G.link_names[L[i]], G.link_names[L[j]])))].add(int(bi))
pairs = sorted([list(k) for k, v in per_pair.items() if len(v) / len(Q) > a.threshold])
for k, v in sorted(per_pair.items(), key=lambda kv: -len(kv[1])):
    print(f'{len(v)/len(Q):6.1%} {k} {"EXEMPT" if list(k) in pairs else ""}')
cb.SELF_EXEMPT_JSON.write_text(json.dumps({
    'robot_yml_sha256': hashlib.sha256(cb.ROBOT_YML.read_bytes()).hexdigest(),
    'exact_free_samples': int(len(Q)), 'threshold': a.threshold, 'pairs': pairs}, indent=1))
print(len(Q), 'exact-free samples;', len(pairs), 'pairs exempted ->', cb.SELF_EXEMPT_JSON)
