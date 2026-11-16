"""Straight Cartesian line of the TCP (Arm.line_waypoints) and one-profile timing (Arm.timed_path)."""
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from planner import Arm
from test_arm import URDF, calibration

Q0 = np.array([0.2, 0.5, -0.3, 0.6, -0.4, 0.5, 0.3])


class LineTests(unittest.TestCase):
    def setUp(self):
        self.arm = Arm(URDF, calibration())
        self.T0 = self.arm.fk(Q0)

    def test_every_point_is_on_the_line_and_the_orientation_is_held(self):
        delta = np.array([0.0, 0.05, 0.0])
        points = self.arm.line_waypoints(Q0, delta, step_m=0.002)
        self.assertGreaterEqual(len(points), 26)
        for k, q in enumerate(points):
            T = self.arm.fk(q)
            expected = self.T0[:3, 3] + delta*k/(len(points)-1)
            self.assertLess(np.linalg.norm(T[:3, 3]-expected), 1e-4)  # 0.1 mm
            self.assertLess(Rotation.from_matrix(self.T0[:3, :3].T @ T[:3, :3]).magnitude(), np.radians(1.))
        self.assertLess(np.max(np.abs(np.diff(np.array(points), axis=0))), 0.05)  # no jump between neighbours

    def test_unreachable_line_is_refused_not_approximated(self):
        with self.assertRaises(ValueError):
            self.arm.line_waypoints(Q0, [0., 0.5, 0.])  # 50 cm further out: beyond the reach

    def test_bad_lengths_are_refused(self):
        for delta in ([0, 0, 0], [0, 0.6, 0], [0, np.nan, 0]):
            with self.assertRaises(ValueError):
                self.arm.line_waypoints(Q0, delta)

    def test_one_smooth_profile_through_the_points(self):
        points = self.arm.line_waypoints(Q0, [0., 0.05, 0.], step_m=0.003)
        rows = self.arm.timed_path(points)
        q = np.array([r['q_rad'] for r in rows])
        t = np.array([r['t_s'] for r in rows])
        np.testing.assert_allclose(q[0], points[0], atol=1e-9)
        np.testing.assert_allclose(q[-1], points[-1], atol=1e-9)
        np.testing.assert_allclose(rows[0]['v_rad_s'], 0., atol=1e-9)  # starts and ends at rest...
        np.testing.assert_allclose(rows[-1]['v_rad_s'], 0., atol=1e-9)
        speed = np.abs(np.diff(q, axis=0))/np.diff(t)[:, None]
        self.assertLessEqual(speed.max(), self.arm.c['max_velocity_rad_s']*1.02)
        # ...but never stops in between: the speed is positive all along (no stop-and-go at each point)
        self.assertGreater(speed[5:-5].min(), 0.)
        # the samples stay on the polyline: every one is within 0.05 mm of the straight Cartesian line
        line = self.T0[:3, 3]
        for qq in q[::7]:
            offset = self.arm.fk(qq)[:3, 3]-line
            along = offset @ np.array([0, 1., 0])
            self.assertLess(np.linalg.norm(offset-along*np.array([0, 1., 0])), 5e-5)

    def test_speed_limit_follows_the_joint_limits(self):
        points = self.arm.line_waypoints(Q0, [0., 0.04, 0.])
        slow = self.arm.timed_path(points)[-1]['t_s']
        self.arm.c['max_velocity_rad_s'] *= 2
        self.arm.c['max_acceleration_rad_s2'] *= 2
        fast = self.arm.timed_path(points)[-1]['t_s']
        self.assertLess(fast, slow)


if __name__ == '__main__':
    unittest.main()
