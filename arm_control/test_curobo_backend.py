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

    def test_tcp_error_measures_the_tip_against_the_ideal_line_end(self):
        poses = json.loads((HERE / 'poses.local.json').read_text())['poses']
        start = [poses['scan']['ticks'][str(i)] for i in range(1, 8)]
        r = self.P.plan_line(start, [0., -0.03, 0.])
        end = r['end_ticks']
        e = self.P.tcp_error(start, [0., -0.03, 0.], end)['error_mm']
        self.assertLess(max(abs(v) for v in e), 1.0)  # the planned end IS the ideal end, up to the tick rounding
        up = self.P.tcp_error(start, [0., -0.03, -0.005], end)['error_mm']  # ideal end 5 mm lower -> the tip is 5 mm too high
        self.assertAlmostEqual(up[2], 5., delta=1.0)

    def test_straight_line_plan_is_precise_and_ends_where_asked(self):
        poses = json.loads((HERE / 'poses.local.json').read_text())['poses']
        start = [poses['scan']['ticks'][str(i)] for i in range(1, 8)]
        r = self.P.plan_line(start, [0., -0.03, 0.])
        self.assertTrue(r['executable'], r['reasons'])
        self.assertLess(r['line_error_mm'], 1.5)  # worst distance to the ideal line, measured on the tick-quantized path
        self.assertLess(r['line_rotation_deg'], 15.)
        q = np.array(r['positions'])
        T0, T1 = (self.P.arm.fk(self.worker.ticks_to_joints(self.P.config, start)), self.P.arm.fk(q[-1]))
        np.testing.assert_allclose((T1[:3, 3] - T0[:3, 3]) * 1000, [0, -30, 0], atol=1.0)  # 30 mm, within 1 mm
        self.assertEqual(r['times'][0], 0.)
        self.assertEqual(r['end_ticks'], [int(t) for t in self.worker.joints_to_ticks(self.P.config, q[-1])])
        with self.assertRaises(ValueError):
            self.P.plan_line(start, [0., 0.5, 0.])  # out of reach: refused, never approximated


if __name__ == '__main__':
    unittest.main()
