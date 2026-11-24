"""The click-to-detect task: every label of one camera image, contours drawn, list returned.
Needs OpenCV/numpy (run with base_ocr/.venv/bin/python)."""
import base64
import importlib.util
import unittest
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:  # the ROS test run has no OpenCV
    cv2 = None

ROOT = Path(__file__).resolve().parents[3]


@unittest.skipIf(cv2 is None, 'OpenCV not available')
class DetectTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('ocr_worker_under_test', ROOT / 'ros2_robot/postal_station/ocr_worker/worker.py')
        cls.worker = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.worker)

    def synthetic(self, centres):
        """A grey 640x480 image with black-bordered white labels (6x5 cm at ~45 cm)."""
        image = np.full((480, 640, 3), 140, np.uint8)
        for x, y in centres:
            cv2.rectangle(image, (x - 30, y - 25), (x + 30, y + 25), (15, 15, 15), -1)
            cv2.rectangle(image, (x - 24, y - 19), (x + 24, y + 19), (250, 250, 250), -1)
            cv2.putText(image, 'JAN', (x - 18, y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        return image

    def test_finds_several_labels_in_one_frame_and_draws_them(self):
        frame = self.synthetic([(150, 200), (330, 200), (500, 330)])
        labels, jpeg = self.worker.detect_mailboxes(frame)
        self.assertEqual(len(labels), 3)
        self.assertEqual([l['id'] for l in labels], [1, 2, 3])
        self.assertTrue(all(l['distance_m'] > 0 for l in labels))
        drawn = cv2.imdecode(np.frombuffer(base64.b64decode(jpeg), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(drawn.shape, frame.shape)
        self.assertGreater(int((np.abs(drawn.astype(int) - frame.astype(int)).sum(axis=2) > 60).sum()), 200)  # contours

    def test_names_are_read_and_matched_to_the_recipient_list(self):
        image = np.full((480, 640, 3), 140, np.uint8)
        cv2.rectangle(image, (200, 150), (290, 215), (15, 15, 15), -1)
        cv2.rectangle(image, (208, 158), (282, 207), (250, 250, 250), -1)
        cv2.putText(image, 'BOB', (216, 178), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.putText(image, 'ROSS', (214, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        labels, jpeg = self.worker.detect_mailboxes(image)
        self.assertEqual(len(labels), 1)
        self.assertEqual(labels[0]['name'], 'Bob Ross', labels[0])
        self.assertIn('BOB', labels[0]['ocr_text'].upper())
        labels, _ = self.worker.detect_mailboxes(image, read_names=False)
        self.assertIsNone(labels[0]['name'])

    def test_a_label_turned_by_90_degrees_is_read_and_its_slot_follows_the_text(self):
        def scene(width, height):
            face = np.full((height, width, 3), 140, np.uint8)
            y = height // 2 - 32
            cv2.rectangle(face, (40, y), (130, y + 65), (15, 15, 15), -1)      # label border
            cv2.rectangle(face, (48, y + 8), (122, y + 57), (250, 250, 250), -1)
            cv2.putText(face, 'BOB', (56, y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            cv2.putText(face, 'ROSS', (54, y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            cv2.rectangle(face, (155, y), (width - 30, y + 65), (25, 25, 25), -1)  # slot, right of the label
            return face
        for turn, flag in ((0, None), (1, cv2.ROTATE_90_CLOCKWISE), (3, cv2.ROTATE_90_COUNTERCLOCKWISE)):
            # a quarter-turned camera image is still 640x480: draw the scene portrait, then rotate it
            image = scene(640, 480) if flag is None else cv2.rotate(scene(480, 640), flag)
            labels, _ = self.worker.detect_mailboxes(image)
            self.assertEqual(len(labels), 1, turn)
            label = labels[0]
            self.assertEqual(label['name'], 'Bob Ross', f'turn {turn}: {label}')
            self.assertEqual(label['quarter_turns'], turn)
            corners = np.array(label['corners_px'])
            outline = self.worker.mailbox_detector()['ll'].slot_outline_px(
                dict(label_corners_px=label['corners_px']), self.worker.mailbox_detector()['calibration'],
                self.worker.mailbox_detector()['geometry'])
            direction = outline[4] - corners.mean(axis=0)  # label centre -> slot centre, in the image
            expected = {0: (1, 0), 1: (0, 1), 3: (0, -1)}[turn]  # slot is to the right of the text
            self.assertGreater(float(direction @ np.array(expected)) / np.linalg.norm(direction), 0.95, f'turn {turn}')

    def test_empty_scene_returns_no_label(self):
        labels, jpeg = self.worker.detect_mailboxes(np.full((480, 640, 3), 128, np.uint8))
        self.assertEqual(labels, [])
        self.assertTrue(jpeg)


if __name__ == '__main__':
    unittest.main()
