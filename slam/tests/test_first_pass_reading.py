"""Names are read during the first (translation) pass, at the starting distance."""

import unittest
from unittest.mock import patch

import numpy as np

from mailbox_slam.survey import MailboxSurvey, SurveySettings
from tests.test_mailbox_discovery import Rig


class FirstPassReadingTests(unittest.TestCase):
    def rig_with_two_labels(self):
        rig = Rig()
        rig.slam.start(0)
        tags = []
        for y in (-0.1, 0.2):
            tag = rig.tag()
            tag[:3, 3] = [0.6, y, 0.3]
            tags.append(tag)
        return rig, tags

    def test_every_visible_label_is_read_and_confirmed_over_two_frames(self):
        rig, tags = self.rig_with_two_labels()
        texts = {}

        def process_image(image, timestamp, **kwargs):
            results = [rig.result(t) for t in tags]
            for result, text in zip(results, ("Mr Jan Smith", "Ms Anna Berg")):
                texts[tuple(result["label_corners_px"][0])] = text
            for i, result in enumerate(results):  # distinct corners per label, as on an image
                result["label_corners_px"] = [[100 + 200 * i, 100], [160 + 200 * i, 100],
                                              [160 + 200 * i, 150], [100 + 200 * i, 150]]
                texts[tuple(result["label_corners_px"][0])] = ("Mr Jan Smith", "Ms Anna Berg")[i]
            return rig.slam.process_locator_results(timestamp, results, **kwargs)

        def read_label_name(image, corners):
            return dict(ocr_text=texts[tuple(corners[0])], recipient=None, match_score=1.0, ocr_s=0.0)

        with (
            patch.object(rig.slam, "process_image", side_effect=process_image),
            patch.object(rig.slam.locator, "read_label_name", side_effect=read_label_name),
        ):
            rig.advance()
            report = rig.slam.read_visible_names(None, rig.timestamp)
        self.assertEqual(len(report["reads"]), 2)
        names = sorted(m["name"] for m in rig.slam.mailboxes() if m["name"])
        # Each box is detected in a single frame on the first pass: one reading confirms it.
        self.assertEqual(names, ["Mr. Jan Smith", "Ms. Anna Berg"])

    def test_two_confirmations_can_still_be_required(self):
        rig, tags = self.rig_with_two_labels()
        texts = ("Mr Jan Smith", "Ms Anna Berg")

        def process_image(image, timestamp, **kwargs):
            results = [rig.result(t) for t in tags]
            for i, result in enumerate(results):
                result["label_corners_px"] = [[100 + 200 * i, 100], [160 + 200 * i, 100],
                                              [160 + 200 * i, 150], [100 + 200 * i, 150]]
            return rig.slam.process_locator_results(timestamp, results, **kwargs)

        def read_label_name(image, corners):
            return dict(ocr_text=texts[0 if corners[0][0] < 200 else 1], recipient=None,
                        match_score=1.0, ocr_s=0.0)

        with (
            patch.object(rig.slam, "process_image", side_effect=process_image),
            patch.object(rig.slam.locator, "read_label_name", side_effect=read_label_name),
        ):
            rig.advance()
            rig.slam.read_visible_names(None, rig.timestamp, confirmations=2)
            self.assertFalse(any(m["name"] for m in rig.slam.mailboxes()))
            rig.advance()
            rig.slam.read_visible_names(None, rig.timestamp, confirmations=2)
        self.assertEqual(sum(1 for m in rig.slam.mailboxes() if m["name"]), 2)

    def test_unavailable_ocr_does_not_stop_the_pass_and_read_attempts_are_bounded(self):
        rig, tags = self.rig_with_two_labels()
        calls = []

        def read_label_name(image, corners):
            calls.append(1)
            raise RuntimeError("station OCR down")

        with (
            patch.object(rig.slam, "process_image",
                         side_effect=lambda image, timestamp, **kw: rig.slam.process_locator_results(
                             timestamp, [rig.result(t) for t in tags], **kw)),
            patch.object(rig.slam.locator, "read_label_name", side_effect=read_label_name),
        ):
            for _ in range(8):
                rig.advance()
                report = rig.slam.read_visible_names(None, rig.timestamp, max_reads_per_box=2)
        self.assertEqual(report["reads"], [])
        self.assertLessEqual(len(calls), 2 * 2)  # at most 2 attempts per box
        self.assertEqual(len(rig.slam.mailboxes()), 2)  # geometry kept

    def test_setting_is_accepted(self):
        MailboxSurvey(Rig().slam, SurveySettings(read_during_sweep=True))


if __name__ == "__main__":
    unittest.main()
