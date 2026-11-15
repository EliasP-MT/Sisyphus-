import copy
from pathlib import Path
import time
import unittest

from calibration_ui import candidate
from planner import load_config
from ui_motion import Motions


class MotionWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parent
        base = load_config(self.root/'elrobot.example.json')
        base['bus'] = 'test'
        for j in base['joints']:
            j.update(min_ticks=100, max_ticks=3990)
        self.config = candidate(base, {'joints': [dict(motor_id=i, zero_ticks=2048,
                    sign=1, ticks_per_rad=4096/6.283185307179586) for i in range(1, 8)]})
        self.current = {str(i): dict(position=2048, torque=False) for i in range(1, 9)}
        self.target = {'motors': copy.deepcopy(self.current)}
        self.target['motors']['1']['position'] = 2070
        self.motions = Motions(self.root, 'not-a-real-server')

    def test_unverified_preview_cannot_execute_even_after_animation(self):
        result = self.motions.preview(self.config, self.current, self.target)
        self.assertFalse(result['executable'])
        self.assertFalse(self.config['hardware_verified'])
        self.assertGreater(result['duration'], 0)
        self.motions.plans[result['token']]['created'] = time.monotonic()-100
        with self.assertRaisesRegex(ValueError, 'collision scene'):
            self.motions.execute(result['token'])

    def test_early_or_unknown_confirmation_rejected(self):
        result = self.motions.preview(self.config, self.current, self.target)
        with self.assertRaisesRegex(ValueError, 'Finish'):
            self.motions.execute(result['token'])
        with self.assertRaisesRegex(ValueError, 'Unknown'):
            self.motions.execute('unknown')

    def test_out_of_range_pose_refused(self):
        self.target['motors']['2']['position'] = 4095
        with self.assertRaises(ValueError):
            self.motions.preview(self.config, self.current, self.target)

    def test_current_inside_station_range_can_preview_outside_margin(self):
        self.config['joints'][0]['min_ticks'] = 2030
        self.current['1'].update(position=2020, range_min=2000, range_max=3990)
        result = self.motions.preview(self.config, self.current, self.target)
        self.assertFalse(result['executable'])
        self.assertTrue(any('operating margin' in reason for reason in result['reasons']))
        self.assertEqual(self.config['joints'][0]['min_ticks'], 2030)
