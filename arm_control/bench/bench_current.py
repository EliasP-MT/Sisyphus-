"""Time breakdown of the current planner worker on recorded poses (no robot connection)."""
import itertools, json, sys, time
from pathlib import Path
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'ros2_robot/postal_station/planner_worker'))
sys.path.insert(0, str(REPO / 'arm_control'))
import worker
CAL = REPO / 'arm_control/calibration-candidate-20261115-105050-197423.local.json'
SCENE = REPO / 'arm_control/assets/rover-fixed.local/rover-scene.local.json'
t = time.monotonic(); P = worker.Planner(CAL, SCENE); P.max_speed_factor = 2.; print(f'init {time.monotonic()-t:.2f}s')
poses = {k: [v['ticks'][str(i)] for i in range(1, 8)] for k, v in json.loads((REPO/'arm_control/poses.local.json').read_text())['poses'].items()}
W = P.world
orig = {n: getattr(W, n) for n in ('joint_path', 'validate_plan')}
orig_comp = P._compensation_box
acc = {}
def timed(name, f):
    def g(*a, **k):
        t = time.monotonic(); r = f(*a, **k); acc[name] = acc.get(name, 0) + time.monotonic() - t; return r
    return g
W.joint_path = timed('ompl', orig['joint_path']); W.validate_plan = timed('validate', orig['validate_plan'])
P._compensation_box = timed('compensation', orig_comp)
for a, b in itertools.permutations(poses, 2):
    acc.clear(); W.checks = 0
    try:
        r = P.plan(poses[a], poses[b])
        print(f"{a:>10}->{b:<10} plan {r['planning_s']:.2f}s motion {r['duration_s']:.2f}s wp {r['waypoints']} checks {r['collision_checks']} "
              + ' '.join(f'{k} {v:.2f}' for k, v in acc.items()))
    except Exception as e:
        print(a, b, 'ERR', e)
