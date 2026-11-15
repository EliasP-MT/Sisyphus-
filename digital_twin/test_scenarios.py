"""End-to-end software acceptance tests on the MuJoCo plant."""
import tempfile
import unittest
from pathlib import Path

from .run import run


class ScenarioTests(unittest.TestCase):
    def test_two_letters_reach_their_correct_mailboxes(self):
        with tempfile.TemporaryDirectory(prefix='postal-mission-test-') as directory:
            result = run(seed=3, output=Path(directory)/'run')
        self.assertEqual(result['status'], 'done', result['error'])
        self.assertEqual(result['delivered'], {'letter_0': 'Alice', 'letter_1': 'Bob'})
        self.assertEqual(result['collision_steps'], 0)
        self.assertEqual(result['joint_limit_violation_steps'], 0)
        self.assertEqual(result['instability_steps'], 0)
        self.assertLess(result['slam_rmse_m'], .03)

    def test_navigation_mapping_and_loop_closure(self):
        with tempfile.TemporaryDirectory(prefix='postal-loop-test-') as directory:
            result = run(scenario='loop', seed=4, output=Path(directory)/'run')
        self.assertEqual(result['status'], 'done', result['error'])
        self.assertEqual(result['goals_reached'], 3)
        self.assertGreater(result['loop_closures'], 0)
        self.assertLess(result['slam_rmse_m'], .02)
        self.assertLess(result['slam_rmse_m'], result['odometry_rmse_m']*.7)
        self.assertEqual(result['collision_steps'], 0)

    def test_sensor_dropout_stops_the_fsm(self):
        with tempfile.TemporaryDirectory(prefix='postal-dropout-test-') as directory:
            result = run(scenario='dropout', seed=2, output=Path(directory)/'run')
        self.assertEqual(result['status'], 'fault')
        self.assertIn('périmé', result['error'])
        self.assertLess(result['simulated_time_s'], 3)
        self.assertEqual(result['delivered'], {})


if __name__ == '__main__': unittest.main()
