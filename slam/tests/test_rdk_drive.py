"""Hardware-free checks for RDK wheel commands and shutdown."""

import math
import unittest
from unittest.mock import patch

import rdk_drive


class RdkTests(unittest.TestCase):
    def test_kinematics_and_cap(self):
        self.assertEqual(rdk_drive.wheel_commands(0, 0, 0), [0, 0, 0])
        forward = rdk_drive.wheel_commands(0.5, 0, 0)
        self.assertEqual(max(map(abs, forward)), rdk_drive.MAX_RAW)
        self.assertEqual(forward, [-v for v in rdk_drive.wheel_commands(-0.5, 0, 0)])
        turn = rdk_drive.wheel_commands(0, 0, math.radians(10))
        self.assertEqual(turn[0], turn[1])
        self.assertEqual(turn[1], turn[2])
        with self.assertRaises(ValueError):
            rdk_drive.wheel_commands(float("nan"), 0, 0)

    def test_negative_velocity_encoding(self):
        with patch.object(rdk_drive, "Bus"):
            controller = rdk_drive.Controller("unused")
        controller.move(-0.01, 0, 0)
        values = rdk_drive.wheel_commands(-0.01, 0, 0)
        for motor, value in zip(rdk_drive.MOTORS, values):
            controller.bus.write.assert_any_call(
                motor, 46, abs(value) | (0x8000 if value < 0 else 0), 2
            )

    def test_cleanup_attempts_all_motors_after_error(self):
        with patch.object(rdk_drive, "Bus"):
            controller = rdk_drive.Controller("unused")
        controller.started = True
        controller.modes = {7: 0, 8: 1, 9: 0}
        controller.bus.write.side_effect = [RuntimeError("disconnected")] + [None] * 8
        with self.assertRaisesRegex(RuntimeError, "Stop/restore failed"):
            controller.close()
        for motor in rdk_drive.MOTORS:
            controller.bus.write.assert_any_call(motor, 40, 0, 1)
            controller.bus.write.assert_any_call(motor, 33, controller.modes[motor])
        controller.bus.serial.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
