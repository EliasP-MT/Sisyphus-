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
        off[0] += .3
        r.step(.05, off)
        self.assertIn('Tracking', r.message)
        self.assertIn('J1', r.message)
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
        r = TrajectoryRun(c, JOINTS, times, points, compensation=limits, arrival_timeout=8.)
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

    def run_relaxed(self, tolerance, sag=.04):
        c = calibration()
        times, points = plan(c)
        r = TrajectoryRun(c, JOINTS, times, points, arrival_tolerance=tolerance, arrival_timeout=8.)
        r.begin(points[0], 0.)
        goal = [c.to_radians(n, t) for n, t in zip(JOINTS, r.goals[0])]
        t, success_at = 0., None
        while r.state == RUNNING and t < 20:
            measured = list(goal)
            measured[3] -= sag
            out = r.step(t, measured)
            if out:
                goal = [c.to_radians(n, v) for n, v in zip(JOINTS, out)]
            t += .05
        return r, t

    def test_relaxed_tolerance_accepts_a_settled_arm_only(self):
        r, t = self.run_relaxed(.07)
        self.assertEqual(r.state, SUCCEEDED, r.message)
        self.assertGreaterEqual(t, r.duration + r.settle_time)  # not accepted while still travelling
        r, _ = self.run_relaxed(.03)                              # sag 0.04 above the accepted tolerance
        self.assertEqual(r.state, ABORTED)

    def test_a_failed_arrival_names_the_joint_and_its_offset(self):
        r, _, _ = self.simulate([0, 0, 0, .01, 0, 0, 0])  # bound far below the 0.03 sag of J4
        self.assertEqual(r.state, ABORTED)
        self.assertIn('worst joint J4', r.message)
        self.assertIn('offset', r.message)

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


class SpeedFactorTests(unittest.TestCase):
    def test_faster_plan_needs_speed_ceiling(self):
        c = calibration()
        times, points = plan(c, steps=5, dt=.1, delta=.02)  # 0.2 rad/s > 0.1457 * 1.25
        with self.assertRaises(TrajectoryError):
            TrajectoryRun(c, JOINTS, times, points)
        r = TrajectoryRun(c, JOINTS, times, points, max_speed_factor=2.)
        self.assertAlmostEqual(r.peak_ticks_per_s()[0], .02 * 651.9 / .1, delta=10)
        times, points = plan(c, steps=5, dt=.1, delta=.05)  # 0.5 rad/s > 2x ceiling
        with self.assertRaises(TrajectoryError):
            TrajectoryRun(c, JOINTS, times, points, max_speed_factor=2.)


class LoadGuardTests(unittest.TestCase):
    def play(self, loads_at, limit=200, seconds=30., **kw):
        """Follow the plan perfectly while the servo loads are given by loads_at(t)."""
        c = calibration()
        times, points = plan(c, steps=21, dt=.5, delta=.02)  # 10 s long
        r = TrajectoryRun(c, JOINTS, times, points, load_limit_raw=limit, arrival_timeout=20., **kw)
        r.begin(points[0], 0.)
        t, history = 0., []
        while r.state == RUNNING and t < seconds:
            reference = points[max(r.commanded, 0)]
            r.step(t, list(reference), loads_at(t))
            history.append((t, r.rate, r.virtual))
            t += .05
        return r, history, t

    def test_light_load_plays_at_full_speed(self):
        r, history, t = self.play(lambda t: [30, 100, 20, 0, 0, 0, 0])
        self.assertEqual(r.state, SUCCEEDED, r.message)
        self.assertEqual(min(h[1] for h in history), 1.)
        self.assertLess(t, 11.5)  # the 10 s plan took about 10 s

    def test_a_high_load_slows_the_playback_and_the_run_still_finishes(self):
        # J2 at 80 % of its threshold for the middle of the motion: slower, but it arrives.
        r, history, t = self.play(lambda t: [0, 160 if 3 < t < 8 else 60, 0, 0, 0, 0, 0])
        self.assertEqual(r.state, SUCCEEDED, r.message)
        self.assertLess(min(h[1] for h in history), .8)
        self.assertGreater(t, 12.)  # it took visibly longer than the plan
        self.assertIn('slowed', r.message)

    def test_a_load_that_stays_at_the_threshold_holds_the_arm_before_the_servo_trips(self):
        r, history, t = self.play(lambda t: [0, 195 if t > 4 else 50, 0, 0, 0, 0, 0])
        self.assertEqual(r.state, ABORTED)
        self.assertIn('Load guard: J2', r.message)
        self.assertIn('held before the servo protection trips', r.message)

    def test_guard_is_off_without_a_limit_or_loads(self):
        c = calibration()
        times, points = plan(c)
        r = TrajectoryRun(c, JOINTS, times, points)
        r.begin(points[0], 0.)
        r.step(.05, list(points[0]), [0, 999, 0, 0, 0, 0, 0])  # no limit configured
        self.assertEqual(r.state, RUNNING)
        self.assertEqual(r.rate, 1.)


class PerJointLimitTests(unittest.TestCase):
    def test_a_slower_joint_limit_refuses_a_plan_that_is_fine_for_the_others(self):
        c = calibration()
        times, points = plan(c, steps=5, dt=.1, delta=.02)  # joint 1 at 0.2 rad/s
        TrajectoryRun(c, JOINTS, times, points, max_speed_factor=2.)  # accepted with default limits
        c.data['joint_limits'] = {'velocity_scale': [0.5, 1, 1, 1, 1, 1, 1]}
        with self.assertRaises(TrajectoryError):
            TrajectoryRun(c, JOINTS, times, points, max_speed_factor=1.)

    def test_speed_ratio_is_measured_against_each_joints_own_limit(self):
        c = calibration()
        times, points = plan(c, steps=5, dt=.1, delta=.01)  # 0.1 rad/s on joint 1
        base = TrajectoryRun(c, JOINTS, times, points, max_speed_factor=2.)
        c.data['joint_limits'] = {'velocity_scale': [0.5, 1, 1, 1, 1, 1, 1]}
        slow_joint = TrajectoryRun(c, JOINTS, times, points, max_speed_factor=2.)
        self.assertEqual(base.speed_ratio, 1.)
        self.assertGreater(slow_joint.speed_ratio, base.speed_ratio)  # 0.1 rad/s is 1.4x of 0.073

    def test_invalid_scales_are_rejected(self):
        c = calibration()
        for bad in ([1] * 6, [1, 1, 1, 1, 1, 1, 0], [1, 1, 1, 1, 1, 1, 5], 'x'):
            c.data['joint_limits'] = {'velocity_scale': bad}
            with self.assertRaises(TrajectoryError):
                TrajectoryRun(c, JOINTS, *plan(c))


class TrackingScalingTests(unittest.TestCase):
    def test_tracking_bound_is_proportional_to_speed(self):
        c = calibration()
        slow = TrajectoryRun(c, JOINTS, *plan(c, steps=5, dt=.1, delta=.01))      # 0.1 rad/s < calibrated
        fast = TrajectoryRun(c, JOINTS, *plan(c, steps=5, dt=.1, delta=.02), max_speed_factor=2.)
        self.assertEqual(slow.speed_ratio, 1.)
        self.assertAlmostEqual(slow.tracking_error, .2)
        self.assertAlmostEqual(fast.tracking_error, .1 * fast.speed_ratio + .1)

    def test_start_sag_is_removed_from_tracking_error(self):
        c = calibration()
        times, points = plan(c)
        sag = [0, 0, -.018, -.018, 0, 0, 0]                    # held start pose sags, within start tolerance
        start = [p + s for p, s in zip(points[0], sag)]
        r = TrajectoryRun(c, JOINTS, times, points)
        r.begin(start, 0.)
        lagging = [p + s for p, s in zip(points[0], sag)]
        lagging[0] -= .19                                       # pure lag below the bound
        r.step(.05, lagging)
        self.assertEqual(r.state, RUNNING)
        lagging[0] -= .03                                       # lag + sag above bound
        r.step(.1, lagging)
        self.assertEqual(r.state, ABORTED)
        self.assertIn('Tracking', r.message)

    def test_sag_cannot_hide_a_real_error(self):
        c = calibration()
        times, points = plan(c)
        r = TrajectoryRun(c, JOINTS, times, points)
        r.begin([p + .019 for p in points[0]], 0.)             # at most the start tolerance
        big = [p + .019 + .25 for p in points[0]]
        r.step(.05, big)
        self.assertEqual(r.state, ABORTED)
