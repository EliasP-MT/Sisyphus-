"""cuRobo backend vs the exact checker (skipped without CUDA/cuRobo or the local scene files)."""
import json, sys, unittest
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
CAL = HERE / 'calibration-candidate-20261115-105050-197423.local.json'
SCENE = HERE / 'assets/rover-fixed.local/rover-scene.local.json'
try:
    import torch
    READY = torch.cuda.is_available() and CAL.exists() and SCENE.exists()
except ImportError:
    READY = False


@unittest.skipUnless(READY, 'cuRobo/CUDA or local scene unavailable')
class CuroboBackendTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path[:0] = [str(HERE.parent / 'ros2_robot/postal_station/planner_worker'), str(HERE)]
        import worker
        cls.worker = worker
        cls.P = worker.Planner(CAL, SCENE)
        cls.P.max_speed_factor = 2.
        assert cls.P.gpu is not None, cls.P.gpu_error

    def test_agreement_with_exact_checker(self):
        rng = np.random.default_rng(7)
        Q = rng.uniform(self.P.arm.bounds[:, 0], self.P.arm.bounds[:, 1], (600, 7))
        g = self.P.gpu.gpu.valid_batch(Q)
        e = np.array([self.P.world.valid(q) for q in Q])
        self.assertLessEqual((g & ~e).sum(), 3)      # GPU-free but exact-collision (exact check covers it)
        self.assertLess((~g & e).sum(), 0.15 * e.sum())  # conservative false alarms

    def test_recorded_poses_plan_fast_and_match_cpu_timing(self):
        poses = json.loads((HERE / 'poses.local.json').read_text())['poses']
        a, b = ([poses[k]['ticks'][str(i)] for i in range(1, 8)] for k in ('vertical', 'scan'))
        r = self.P.plan(a, b)
        self.assertTrue(r['backend'].startswith('curobo'))
        self.assertLess(r['planning_s'], 6.)
        self.assertGreater(r['exact_checks'], 10)
        self.assertEqual(r['times'][0], 0.)


if __name__ == '__main__':
    unittest.main()
