import json
import tempfile
import unittest
from pathlib import Path

from postal_robot.poses import PoseError, PoseStore, capture_from_snapshots

LEGACY = {'schema': 'encoder-poses-v1', 'bus': '5B8E114718', 'model_calibrated': False,
          'poses': {'vertical': {'recorded_at': 'x', 'motors': {'1': {'position': 2020}},
                                 'ticks': {str(i): 2000 + i for i in range(1, 9)}}}}


def snap(seq, base=2000, t=0., jitter=0):
    return {i: (base + i + (jitter if i == 3 else 0), t + i * .001, seq) for i in range(1, 9)}


class PoseStoreTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / 'poses.local.json'
        self.path.write_text(json.dumps(LEGACY))
        self.store = PoseStore(self.path, '5B8E114718')

    def tearDown(self):
        self.dir.cleanup()

    def test_legacy_poses_preserved_on_save(self):
        self.store.save('new', {i: 1500 for i in range(1, 9)}, False, 'abc', 'test')
        data = json.loads(self.path.read_text())
        self.assertEqual(data['poses']['vertical'], LEGACY['poses']['vertical'])
        self.assertEqual(data['poses']['new']['ticks']['8'], 1500)
        self.assertEqual(list(Path(self.dir.name).glob('poses-backup-*')), [])

    def test_replace_requires_flag_and_keeps_backup(self):
        with self.assertRaises(PoseError):
            self.store.save('vertical', {i: 1 for i in range(1, 9)}, False, '', 't')
        self.store.save('vertical', {i: 1 for i in range(1, 9)}, True, '', 't')
        backups = list(Path(self.dir.name).glob('poses-backup-*.local.json'))
        self.assertEqual(len(backups), 1)
        self.assertEqual(json.loads(backups[0].read_text()), LEGACY)

    def test_delete_keeps_backup(self):
        self.store.delete('vertical')
        self.assertEqual(self.store.poses(), {})
        self.assertEqual(len(list(Path(self.dir.name).glob('poses-backup-*'))), 1)

    def test_rejects_other_bus_and_bad_names(self):
        with self.assertRaises(PoseError):
            PoseStore(self.path, 'OTHER')
        with self.assertRaises(PoseError):
            self.store.save('../evil', {i: 1 for i in range(1, 9)}, False, '', 't')


class CaptureTests(unittest.TestCase):
    def test_three_new_stable_samples(self):
        self.assertEqual(capture_from_snapshots([snap(1), snap(2), snap(3)])[8], 2008)

    def test_same_sample_not_counted_twice(self):
        self.assertIsNone(capture_from_snapshots([snap(1), snap(1), snap(1)]))

    def test_motion_restarts_capture(self):
        self.assertIsNone(capture_from_snapshots([snap(1), snap(2, jitter=10), snap(3, jitter=10)]))
        self.assertIsNotNone(capture_from_snapshots([snap(1), snap(2, jitter=10), snap(3, jitter=10),
                                                     snap(4, jitter=10)]))

    def test_missing_motor_or_skew_refused(self):
        partial = snap(2)
        del partial[5]
        self.assertIsNone(capture_from_snapshots([snap(1), partial, snap(3)]))
        skewed = {i: (2000, i * .05, 2) for i in range(1, 9)}
        self.assertIsNone(capture_from_snapshots([snap(1), skewed, snap(3)]))


if __name__ == '__main__':
    unittest.main()
