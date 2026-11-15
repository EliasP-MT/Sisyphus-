import math
import unittest

from postal_robot.kiwi import Kiwi


class KiwiTests(unittest.TestCase):
    def setUp(self):
        self.k = Kiwi(.05, .125, (240, 0, 120), (1, 1, 1), 1500)

    def test_pure_rotation_drives_wheels_equally(self):
        rates = self.k.wheel_rates(0, 0, 1.)
        for r in rates:
            self.assertAlmostEqual(r, .125 / .05)

    def test_matches_digital_twin_convention(self):
        # digital_twin/common.py: angles = mount - 90 deg; rows [cos, sin, R] / r
        vx, vy, wz = .1, -.05, .3
        for (m, rate) in zip((240, 0, 120), self.k.wheel_rates(vx, vy, wz)):
            a = math.radians(m - 90)
            self.assertAlmostEqual(rate, (math.cos(a) * vx + math.sin(a) * vy + .125 * wz) / .05)

    def test_round_trip(self):
        raw = self.k.raw_speeds(.05, .02, .1)
        back = self.k.twist_from_raw(raw)
        for a, b in zip(back, (.05, .02, .1)):
            self.assertAlmostEqual(a, b, places=3)

    def test_saturation_keeps_direction(self):
        raw = self.k.raw_speeds(5., 0, 0)
        self.assertEqual(max(abs(v) for v in raw), 1500)
        unsat = self.k.wheel_rates(5., 0, 0)
        ratio = [r / u for r, u in zip(raw, unsat) if abs(u) > 1e-9]
        self.assertTrue(all(abs(x - ratio[0]) < .5 for x in ratio))

    def test_rejects_nan_and_bad_signs(self):
        with self.assertRaises(ValueError):
            self.k.raw_speeds(float('nan'), 0, 0)
        with self.assertRaises(ValueError):
            Kiwi(signs=(1, 0, 1))


if __name__ == '__main__':
    unittest.main()
