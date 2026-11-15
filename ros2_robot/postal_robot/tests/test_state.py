import unittest
from postal_robot.state import JointCache, StableCapture


class StateTests(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.cache = JointCache(['a', 'b'], clock=lambda: self.now)

    def test_partial_update_does_not_refresh_other_joint(self):
        self.cache.update(['a','b'], [1,2])
        self.now = 1
        self.cache.update(['a'], [1])
        with self.assertRaisesRegex(ValueError, 'b'):
            self.cache.snapshot()

    def test_unchanged_but_fresh_is_valid(self):
        capture = StableCapture()
        for i in range(3):
            self.now = i*.1
            self.cache.update(['a','b'], [1,2])
            result = capture.feed(self.cache.snapshot())
        self.assertEqual(result, {'a':1,'b':2})

    def test_repeated_snapshot_cannot_fake_stability(self):
        self.cache.update(['a','b'], [1,2])
        capture = StableCapture()
        for _ in range(10):
            self.assertIsNone(capture.feed(self.cache.snapshot()))

    def test_fault_invalidates_cached_value(self):
        self.cache.update(['a','b'], [1,2])
        self.cache.invalidate('b')
        with self.assertRaises(ValueError):
            self.cache.snapshot()
