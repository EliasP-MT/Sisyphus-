import threading
import unittest

import numpy as np

from mailbox_slam.remote_ocr import RemoteOCR


def poly(x, y, w=60, h=16):
    return [[x, y], [x + w, y], [x + w, y + h], [x, y + h]]


class RemoteOcrTests(unittest.TestCase):
    def test_request_reply_is_merged_into_text_groups(self):
        box = {}

        def send(payload):
            box["request"] = payload
            reply = dict(
                id=payload["id"],
                success=True,
                rec_texts=["Jan", "Smith", "noise"],
                rec_scores=[0.95, 0.9, 0.1],
                rec_polys=[poly(10, 20, 30), poly(46, 20, 50), poly(10, 90)],
            )
            threading.Timer(0.05, box["ocr"].deliver, [reply]).start()

        ocr = box["ocr"] = RemoteOCR(send, timeout_s=3)
        groups = ocr.read_text(np.full((60, 120, 3), 255, np.uint8))
        self.assertEqual([g["text"] for g in groups], ["Jan Smith"])  # low-confidence "noise" dropped
        self.assertEqual(ocr.last_detections, groups)
        self.assertTrue(box["request"]["image_b64"])

    def test_silent_station_times_out(self):
        ocr = RemoteOCR(lambda payload: None, timeout_s=0.1)
        with self.assertRaises(TimeoutError):
            ocr.read_text(np.zeros((10, 10, 3), np.uint8))
        self.assertEqual(ocr.last_detections, [])

    def test_failure_reply_raises_and_unknown_ids_are_ignored(self):
        box = {}

        def send(payload):
            box["ocr"].deliver(dict(id="other", success=True))  # not ours
            box["ocr"].deliver(dict(id=payload["id"], success=False, message="model down"))

        ocr = box["ocr"] = RemoteOCR(send, timeout_s=2)
        with self.assertRaisesRegex(RuntimeError, "model down"):
            ocr.read_text(np.zeros((10, 10, 3), np.uint8))


if __name__ == "__main__":
    unittest.main()
