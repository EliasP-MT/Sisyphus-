import unittest

from fixtures import calibration, calibration_data
from postal_robot.calibration import Calibration, CalibrationError


class CalibrationTests(unittest.TestCase):
    def test_round_trip_and_refusal_without_clipping(self):
        c = calibration()
        self.assertEqual(c.to_ticks('rev_motor_01', c.to_radians('rev_motor_01', 2500)), 2500)
        with self.assertRaises(CalibrationError):
            c.to_ticks('rev_motor_02', c.to_radians('rev_motor_02', 1010))
        self.assertIn('below', c.margin_violation('rev_motor_02', 1010))

    def test_rejects_unverified_or_incomplete(self):
        for change in ({'hardware_verified': False}, {'bus': 'auto'}):
            with self.assertRaises(CalibrationError):
                Calibration(dict(calibration_data(), **change))
        data = calibration_data()
        data['joints'][3]['sign'] = None
        with self.assertRaises(CalibrationError):
            Calibration(data)


if __name__ == '__main__':
    unittest.main()
