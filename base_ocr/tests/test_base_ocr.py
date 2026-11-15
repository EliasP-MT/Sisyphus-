import sys
import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from ocr_base import BaseOCR, _recipient_name, main
from ocr_preview import _process_frame


def result(*items):
    return dict(
        rec_texts=[i[0] for i in items],
        rec_scores=[0.95] * len(items),
        rec_polys=[[[x, y], [x + w, y], [x + w, y + 15], [x, y + 15]] for _, x, y, w in items],
    )


def reader(**kwargs):
    engine = Mock()
    with patch.dict(sys.modules, {"paddleocr": Mock(PaddleOCR=Mock(return_value=engine))}):
        instance = BaseOCR(**kwargs)
    return instance, engine


class BaseTests(unittest.TestCase):
    def test_select_recipient_over_sender_and_unrelated_text(self):
        raw = result(
            ("Sender", 10, 20, 80),
            ("1 rue Victor Hugo", 10, 40, 150),
            ("69001 Lyon", 10, 60, 110),
            ("Jane Doe", 300, 200, 110),
            ("12 Main Street", 300, 220, 150),
            ("Boston MA 02110", 300, 240, 150),
        )
        self.assertEqual(_recipient_name(raw, 640, 480), "Jane Doe")
        self.assertEqual(_recipient_name(result(), 640, 480), "")

    def test_unchanged_full_frame_and_string_return(self):
        instance, engine = reader(max_seconds=10)
        frame = np.zeros((480, 640, 3), np.uint8)
        engine.predict.return_value = [
            result(
                ("Jane Doe", 100, 100, 100),
                ("12 Main Street", 100, 120, 140),
                ("Boston MA 02110", 100, 140, 150),
            )
        ]
        self.assertEqual(instance.read_name(frame), "Jane Doe")
        self.assertIs(engine.predict.call_args.args[0], frame)
        instance.close()

    def test_mac_camera_stays_open_and_is_released(self):
        instance, engine = reader(target="mac", device="0", max_seconds=10)
        engine.predict.return_value = []
        camera = Mock()

        def capture():
            time.sleep(0.002)
            return True, np.zeros((480, 640, 3), np.uint8)

        camera.read.side_effect = capture
        with patch.object(cv2, "VideoCapture", return_value=camera) as factory:
            try:
                instance.read_name()
                instance.read_name()
                factory.assert_called_once_with(0, cv2.CAP_AVFOUNDATION)
                camera.release.assert_not_called()
            finally:
                instance.close()
        camera.release.assert_called_once()

    def test_preview_shows_slow_result_without_mutating_input(self):
        instance, engine = reader()
        instance.last_frame = np.zeros((100, 160, 3), np.uint8)
        instance.last_result = result(("Jane Doe", 10, 30, 100))
        instance.last_name = "Jane Doe"
        instance.last_timings = {"ocr_s": 2.0, "total_s": 2.01}
        instance.last_elapsed_seconds = 2.01
        with patch.object(instance, "read_name", side_effect=TimeoutError()):
            jpeg, status = _process_frame(instance)
        self.assertTrue(jpeg.startswith(b"\xff\xd8"))
        self.assertTrue(status["over_budget"])
        self.assertEqual(status["recipient_name"], "Jane Doe")
        self.assertFalse(instance.last_frame.any())

    def test_robot_is_default_and_preview_opt_in(self):
        for args, visual in [([], False), (["--target", "mac", "--device", "0", "--test"], True)]:
            with (
                patch("ocr_base.BaseOCR") as factory,
                patch("ocr_preview.run_preview") as preview,
                patch("builtins.print"),
            ):
                main(args)
                if visual:
                    factory.assert_called_once_with(device="0", target="mac")
                    preview.assert_called_once()
                else:
                    factory.assert_called_once_with(device=None, target="robot")
                    preview.assert_not_called()
