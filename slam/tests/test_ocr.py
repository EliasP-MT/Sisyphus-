"""OCR/camera/preview contract tests without model downloads or camera devices."""

import contextlib
import io
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np

from ocr_arm import ArmOCR, merge_text_regions
from ocr_base import BaseOCR, main
from ocr_preview import _process_frame


def raw_text(*items):
    return dict(
        rec_texts=[i[0] for i in items],
        rec_scores=[0.95] * len(items),
        rec_polys=[[[x, y], [x + w, y], [x + w, y + h], [x, y + h]] for _, x, y, w, h in items],
    )


def make_reader(cls=BaseOCR, **kwargs):
    engine = Mock()
    factory = Mock(return_value=engine)
    with patch.dict(sys.modules, {"paddleocr": Mock(PaddleOCR=factory)}):
        reader = cls(**kwargs)
    return reader, engine, factory


class OCRTests(unittest.TestCase):
    def test_same_mobile_models_and_unchanged_full_frame(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        raw = raw_text(
            ("Jane Doe", 100, 100, 100, 15),
            ("12 Main Street", 100, 120, 140, 15),
            ("Boston MA 02110", 100, 140, 150, 15),
        )
        for cls in (BaseOCR, ArmOCR):
            reader, engine, factory = make_reader(cls, max_seconds=10)
            engine.predict.return_value = [raw]
            if cls is BaseOCR:
                self.assertEqual(reader.read_name(frame), "Jane Doe")
            else:
                self.assertEqual(len(reader.read_text(frame)), 3)
            self.assertIs(engine.predict.call_args.args[0], frame)
            opts = factory.call_args.kwargs
            self.assertEqual(opts["text_detection_model_name"], "PP-OCRv5_mobile_det")
            self.assertEqual(opts["text_recognition_model_name"], "PP-OCRv5_mobile_rec")
            self.assertFalse(opts["use_doc_orientation_classify"])
            self.assertFalse(opts["use_doc_unwarping"])
            self.assertFalse(opts["use_textline_orientation"])
            reader.close()

    def test_arm_word_merging_preserves_separate_names(self):
        raw = raw_text(
            ("Jane", 10, 30, 40, 20),
            ("Doe", 60, 30, 30, 20),
            ("Anna Berg", 180, 30, 100, 20),
            ("Another", 10, 70, 80, 20),
        )
        groups = merge_text_regions(raw)
        self.assertEqual([g["text"] for g in groups], ["Jane Doe", "Anna Berg", "Another"])
        self.assertEqual(groups[0]["bbox"], [10.0, 30.0, 90.0, 50.0])
        self.assertEqual(groups[0]["center"], [50.0, 40.0])

    def test_arm_never_falls_back_to_base_camera(self):
        reader, _, _ = make_reader(ArmOCR)
        with self.assertRaisesRegex(ValueError, "arm camera"):
            reader.read_text()
        reader.close()

    def test_persistent_camera_and_mac_backend(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        reader, engine, _ = make_reader(target="mac", device="0", max_seconds=10)
        engine.predict.return_value = []
        camera = Mock()

        def read():
            time.sleep(0.002)
            return True, frame

        camera.read.side_effect = read
        with patch.object(cv2, "VideoCapture", return_value=camera) as factory:
            try:
                self.assertEqual(reader.read_name(), "")
                self.assertEqual(reader.read_name(), "")
                factory.assert_called_once_with(0, cv2.CAP_AVFOUNDATION)
                camera.release.assert_not_called()
            finally:
                reader.close()
        camera.release.assert_called_once()

    def test_headless_is_default_and_visualization_is_explicit(self):
        for argv, visual in [([], False), (["--test"], True)]:
            with (
                patch("ocr_base.BaseOCR") as factory,
                patch("ocr_preview.run_preview") as preview,
                contextlib.redirect_stdout(io.StringIO()),
            ):
                main(argv)
                factory.assert_called_once_with(device=None, target="robot")
                if visual:
                    preview.assert_called_once()
                else:
                    preview.assert_not_called()
                    factory.return_value.__enter__.return_value.read_name.assert_called_once_with(
                        None
                    )

    def test_preview_accepts_base_and_arm_and_retains_slow_diagnostics(self):
        polygon = [[10, 30], [100, 30], [100, 60], [10, 60]]
        for arm in (False, True):
            reader = SimpleNamespace(
                last_frame=np.zeros((100, 160, 3), np.uint8),
                last_name="Jane Doe",
                last_result=dict(rec_texts=["Jane Doe"], rec_scores=[0.95], rec_polys=[polygon]),
                last_timings={"ocr_s": 2.0, "total_s": 2.01},
                last_elapsed_seconds=2.01,
                min_confidence=0.6,
                max_seconds=1.0,
                read_name=Mock(side_effect=TimeoutError()),
            )
            if arm:
                reader.read_text = Mock(side_effect=TimeoutError())
                reader.last_detections = [dict(text="Jane Doe", confidence=0.95, polygon=polygon)]
            jpeg, status = _process_frame(reader)
            self.assertTrue(jpeg.startswith(b"\xff\xd8"))
            self.assertTrue(status["over_budget"])
            self.assertEqual(status["mode"], "arm" if arm else "base")
            self.assertFalse(reader.last_frame.any())
