import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from ocr_base import _first_line, _known_name, read_recipient_name


def result(*items):
    return dict(
        rec_texts=[i[0] for i in items],
        rec_scores=[0.95] * len(items),
        rec_polys=[[[x, y], [x + w, y], [x + w, y + 15], [x, y + 15]] for _, x, y, w in items],
    )


class BaseTests(unittest.TestCase):
    def test_only_sixth_frame_is_processed_and_camera_released(self):
        frames = [np.full((480, 640, 3), i, np.uint8) for i in range(6)]
        camera = Mock()
        camera.read.side_effect = [(True, f) for f in frames]
        engine = Mock()
        engine.predict.return_value = [
            result(("Jan", 10, 20, 30), ("Smith", 48, 20, 45), ("Anna Berg", 10, 100, 100))
        ]
        with (
            patch.object(cv2, "VideoCapture", return_value=camera) as factory,
            patch("ocr_base._engine", return_value=engine),
        ):
            self.assertEqual(read_recipient_name(target="mac", device=0), "Jan Smith")
        factory.assert_called_once_with(0, cv2.CAP_AVFOUNDATION)
        self.assertEqual(camera.read.call_count, 6)
        self.assertIs(engine.predict.call_args.args[0], frames[5])
        camera.release.assert_called_once()

    def test_failure_does_not_process_an_earlier_frame(self):
        camera = Mock()
        camera.read.side_effect = [(True, np.zeros((10, 10, 3), np.uint8))] * 5 + [(False, None)]
        with (
            patch.object(cv2, "VideoCapture", return_value=camera),
            patch("ocr_base._engine") as engine,
        ):
            with self.assertRaisesRegex(RuntimeError, "frame 6"):
                read_recipient_name()
        engine.assert_not_called()
        camera.release.assert_called_once()

    def test_geometric_first_line_and_separate_words(self):
        raw = result(("Anna Berg", 10, 120, 100), ("Smith", 48, 20, 45), ("Jan", 10, 20, 30))
        self.assertEqual(_first_line(raw), "Jan Smith")

    def test_unknown_first_line_never_falls_back_to_known_second(self):
        line = _first_line(result(("POSTAGE", 10, 10, 100), ("Jan Smith", 10, 100, 100)))
        self.assertEqual(_known_name(line, ["Jan Smith", "Anna Berg"]), "")

    def test_separate_text_on_same_row_is_not_joined(self):
        self.assertEqual(
            _first_line(result(("Jan", 10, 20, 30), ("Smith", 48, 20, 45), ("Other", 300, 20, 80))),
            "Jan Smith",
        )

    def test_tilted_text_is_grouped_along_its_direction(self):
        import math

        def tilt(item, angle):
            c, s_ = math.cos(angle), math.sin(angle)
            return [[x * c - y * s_, x * s_ + y * c] for x, y in item]

        raw = result(("Anna Berg", 10, 120, 100), ("Smith", 48, 20, 45), ("Jan", 10, 20, 30))
        for angle in (0.61, -0.61, 0.3):  # up to about +-35 degrees
            raw["rec_polys"] = [tilt(p, angle) for p in result(
                ("Anna Berg", 10, 120, 100), ("Smith", 48, 20, 45), ("Jan", 10, 20, 30))["rec_polys"]]
            self.assertEqual(_first_line(raw), "Jan Smith", angle)

    def test_tilt_search_goes_0_then_plus_minus_5_10_to_35(self):
        frame = np.zeros((100, 200, 3), np.uint8)
        engine = Mock()
        engine.predict.return_value = [result(("Unknown", 10, 20, 100))]
        with patch("ocr_base._engine", return_value=engine):
            self.assertEqual(read_recipient_name(frame=frame), "")
        self.assertEqual(engine.predict.call_count, 15)
        self.assertIs(engine.predict.call_args_list[0].args[0], frame)

    def test_search_stops_at_the_first_angle_that_reads_a_name(self):
        frame = np.zeros((100, 200, 3), np.uint8)
        engine = Mock()
        engine.predict.side_effect = [[result()], [result()], [result()], [result(("Jan Smith", 10, 20, 100))]]
        with patch("ocr_base._engine", return_value=engine):
            self.assertEqual(read_recipient_name(frame=frame), "Jan Smith")
        self.assertEqual(engine.predict.call_count, 4)  # 0, +5, -5, +10 degrees

    def test_empty_uncertain_and_tied_match(self):
        self.assertEqual(_first_line(result()), "")
        raw = result(("Jan Smith", 10, 20, 100))
        raw["rec_scores"] = [0.1]
        self.assertEqual(_first_line(raw), "")
        self.assertEqual(_known_name("Jan Smith", ["Jan Smith", "Jan Smyth"]), "Jan Smith")
        self.assertEqual(_known_name("Jan Smit", ["Jan Smith", "Jan Smitz"]), "")
