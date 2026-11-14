import time
import unittest
from types import SimpleNamespace
from control import Station, bounded_goal, targets


class ControlTests(unittest.TestCase):
    def test_no_jump_when_armed(self):
        self.assertEqual(bounded_goal(1500, .9, .9, 1500, 1000, 2000, .05), 1500)

    def test_rate_limited_even_after_delay(self):
        self.assertEqual(bounded_goal(1500, 1, 0, 1500, 1000, 2000, 5), 1510)

    def test_limits(self):
        for signal in (-10, 10):
            goal = bounded_goal(1500, signal, 0, 1500, 1499, 1501, .1)
            self.assertTrue(1499 <= goal <= 1501)

    def test_missing_or_stale_motors_rejected(self):
        station = object.__new__(Station)
        station.received = 0
        station.ids, station.motors = [1], {}
        with self.assertRaisesRegex(RuntimeError, 'périmé'):
            station.state()
        station.received = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, 'absent'):
            station.state()

    def test_pinch_normalized(self):
        hand = [SimpleNamespace(x=.5, y=.5) for _ in range(21)]
        hand[9] = SimpleNamespace(x=.5, y=.25)
        self.assertEqual(targets(hand), [.5, .75, 0])


if __name__ == '__main__':
    unittest.main()
