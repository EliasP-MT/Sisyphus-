import copy
import unittest

from calibration_ui import candidate
from planner import joints_to_ticks, ticks_to_joints, load_config
from pathlib import Path


class CalibrationUITests(unittest.TestCase):
    def setUp(self):
        self.base = load_config(Path(__file__).with_name('elrobot.example.json'))
        self.base['bus'] = 'test'
        self.incoming = {'joints': [dict(motor_id=i, zero_ticks=2048, sign=1,
                                        ticks_per_rad=700.0) for i in range(1, 8)]}
        for j in self.base['joints']:
            j.update(min_ticks=100, max_ticks=3990)

    def test_candidate_never_approves_hardware(self):
        self.base['hardware_verified'] = True
        result = candidate(self.base, self.incoming)
        self.assertFalse(result['hardware_verified'])
        self.assertEqual(result['joints'][0]['ticks_per_rad'], 700)

    def test_reject_changed_id_and_bad_scale(self):
        for key, value in [('motor_id', 11), ('ticks_per_rad', float('nan')),
                           ('zero_ticks', -1), ('sign', 0)]:
            data = copy.deepcopy(self.incoming)
            data['joints'][0][key] = value
            with self.assertRaises(ValueError):
                candidate(self.base, data)

    def test_per_joint_scale_roundtrip(self):
        c = candidate(self.base, self.incoming)
        c['hardware_verified'] = True  # Synthetic fixture only.
        q = ticks_to_joints(c, [2118]*7)
        self.assertAlmostEqual(q[0], .1)
        self.assertEqual(joints_to_ticks(c, q), [2118]*7)
