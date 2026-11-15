import unittest

from fixtures import calibration
from postal_robot.calibration import JOINTS
from postal_robot.trajectory import ABORTED, RUNNING, SUCCEEDED, TrajectoryError, TrajectoryRun


def plan(c, steps=5, dt=.1, delta=.01):
    start = [c.to_radians(n, t) for n, t in zip(JOINTS, [2075, 2065, 2024, 2045, 1888, 1910, 2240])]
    points = [[q + delta * k if j == 0 else q for j, q in enumerate(start)] for k in range(steps)]
    return [dt * k for k in range(steps)], points


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.c = calibration()
        self.times, self.points = plan(self.c)

    def run_(self, **kw):
        return TrajectoryRun(self.c, JOINTS, self.times, self.points, **kw)

    def test_refuses_out_of_range_point_before_any_write(self):
        self.points[2][1] = self.c.to_radians('rev_motor_02', 900)
        with self.assertRaises(TrajectoryError):
            self.run_()

    def test_refuses_bad_times_names_and_speed(self):
        with self.assertRaises(TrajectoryError):
            TrajectoryRun(self.c, JOINTS[::-1], self.times, self.points)
        with self.assertRaises(TrajectoryError):
            TrajectoryRun(self.c, JOINTS, [0, .1, .1, .2, .3], self.points)
        fast = [list(p) for p in self.points]
        fast[1][0] += .5
        with self.assertRaises(TrajectoryError):
            TrajectoryRun(self.c, JOINTS, self.times, fast)

    def test_start_mismatch(self):
        r = self.run_()
        moved = list(self.points[0])
        moved[3] += .05
        with self.assertRaises(TrajectoryError):
            r.begin(moved, 0.)

    def test_nominal_execution_and_arrival(self):
        r = self.run_()
        r.begin(self.points[0], 0.)
        sent = []
        t = 0.
        while r.state == RUNNING and t < 2:
            idx = max(r.commanded, 0)
            goals = r.step(t, self.points[idx])
            if goals:
                sent.append(goals)
            t += .05
        self.assertEqual(r.state, SUCCEEDED)
        self.assertEqual(sent[-1], r.goals[-1])
        self.assertTrue(all(g in r.goals for g in sent))   # only plan samples, no shortcuts

    def test_missing_feedback_tracking_and_lateness_abort(self):
        r = self.run_()
        r.begin(self.points[0], 0.)
        r.step(0., self.points[0])
        self.assertIsNone(r.step(.05, None))
        self.assertEqual(r.state, ABORTED)
        r = self.run_()
        r.begin(self.points[0], 0.)
        off = list(self.points[0])
        off[0] += .2
        r.step(.05, off)
        self.assertIn('Tracking', r.message)
        r = self.run_()
        r.begin(self.points[0], 0.)
        r.step(.05, self.points[0])
        r.step(1., self.points[0])
        self.assertIn('late', r.message)

    def test_arrival_timeout(self):
        r = self.run_(arrival_timeout=.2)
        r.begin(self.points[0], 0.)
        t = 0.
        while r.state == RUNNING and t < 3:
            r.step(t, self.points[-2] if r.commanded == len(self.points) - 1 else self.points[max(r.commanded, 0)])
            t += .05
        self.assertEqual(r.state, ABORTED)
        self.assertIn('not reached', r.message)


if __name__ == '__main__':
    unittest.main()


class CompensationTests(unittest.TestCase):
    """Proportional servo with a constant gravity sag on J4 (index 3)."""

    def simulate(self, limits, sag=.03):
        c = calibration()
        times, points = plan(c)
        r = TrajectoryRun(c, JOINTS, times, points, compensation=limits)
        r.begin(points[0], 0.)
        goal = [c.to_radians(n, t) for n, t in zip(JOINTS, r.goals[0])]
        t = 0.
        while r.state == RUNNING and t < 20:
            measured = list(goal)
            measured[3] -= sag
            out = r.step(t, measured)
            if out:
                goal = [c.to_radians(n, v) for n, v in zip(JOINTS, out)]
            t += .05
        return r, goal, points[-1]

    def test_compensation_reaches_planned_pose(self):
        r, goal, final = self.simulate([0, .06, .06, .06, 0, 0, 0])
        self.assertEqual(r.state, SUCCEEDED, r.message)
        self.assertIn('compensation', r.message)
        self.assertAlmostEqual(goal[3] - final[3], .03, delta=.003)  # commanded goal offset, bounded

    def test_without_compensation_reports_failure(self):
        r, _, _ = self.simulate(None)
        self.assertEqual(r.state, ABORTED)
        self.assertIn('not reached', r.message)

    def test_sag_beyond_bound_is_refused(self):
        r, goal, final = self.simulate([0, .02, .02, .02, 0, 0, 0])
        self.assertEqual(r.state, ABORTED)
        self.assertLessEqual(abs(goal[3] - final[3]), .02 + 1e-3)

    def test_invalid_limits(self):
        c = calibration()
        times, points = plan(c)
        with self.assertRaises(TrajectoryError):
            TrajectoryRun(c, JOINTS, times, points, compensation=[.5] * 7)
