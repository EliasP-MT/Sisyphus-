"""Per-joint velocity/acceleration/jerk limits and separate speed/accel/jerk factors."""
import unittest

import numpy as np

from planner import Arm, joint_limit_scales
from test_arm import URDF, calibration


class JointLimitTests(unittest.TestCase):
    def setUp(self):
        self.c = calibration()
        self.arm = Arm(URDF, self.c)

    def duration_and_peaks(self, c):
        self.arm.c = c
        start = np.zeros(7)
        end = np.array([0.4, 0.3, -0.2, 0.2, 0.3, 0.2, 0.1])
        rows = self.arm.segment(start, end)
        v = np.abs(np.array([r['v_rad_s'] for r in rows]))
        return rows[-1]['t_s'], v.max(axis=0)

    def test_defaults_are_the_scalar_limits_for_every_joint(self):
        scales = joint_limit_scales(self.c)
        for key in ('velocity_scale', 'acceleration_scale', 'jerk_scale'):
            np.testing.assert_array_equal(scales[key], np.ones(7))

    def test_a_gentle_joint_slows_the_whole_synchronised_motion(self):
        base_t, base_peaks = self.duration_and_peaks(dict(self.c))
        gentle = dict(self.c, joint_limits={'velocity_scale': [1, 0.4, 1, 1, 1, 1, 1]})
        t, peaks = self.duration_and_peaks(gentle)
        self.assertGreater(t, base_t * 1.2)                      # the limited joint paces everyone
        self.assertLessEqual(peaks[1], self.c['max_velocity_rad_s'] * 0.4 + 1e-6)

    def test_acceleration_and_jerk_limits_are_honoured_per_joint(self):
        c = dict(self.c, joint_limits={'acceleration_scale': [0.3] + [1] * 6, 'jerk_scale': [0.3] + [1] * 6})
        self.arm.c = c
        rows = self.arm.segment(np.zeros(7), np.array([0.4, 0.3, -0.2, 0.2, 0.3, 0.2, 0.1]))
        a = np.abs(np.array([r['a_rad_s2'] for r in rows]))
        self.assertLessEqual(a[:, 0].max(), self.c['max_acceleration_rad_s2'] * 0.3 + 1e-6)

    def test_invalid_limits_are_refused(self):
        for bad in ({'velocity_scale': [1] * 6}, {'jerk_scale': [1, 1, 1, 1, 1, 1, 0]},
                    {'acceleration_scale': [1, 1, 1, 1, 1, 1, 9]}, {'speed': [1] * 7}):
            with self.assertRaises(ValueError):
                joint_limit_scales(dict(self.c, joint_limits=bad))

    def test_lower_acceleration_alone_keeps_the_speed_but_lengthens_the_ramps(self):
        fast_ramp, _ = self.duration_and_peaks(dict(self.c, max_velocity_rad_s=0.29, max_acceleration_rad_s2=0.6))
        soft_ramp, peaks = self.duration_and_peaks(dict(self.c, max_velocity_rad_s=0.29, max_acceleration_rad_s2=0.3))
        self.assertGreater(soft_ramp, fast_ramp)  # acceleration 0.3 instead of 0.6 is slower to ramp


if __name__ == '__main__':
    unittest.main()
