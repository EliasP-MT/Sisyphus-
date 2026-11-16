"""The base starts facing the wall and slides 1.5 m to the right (no 360 degree turn)."""

import math
import unittest

import numpy as np

from mailbox_slam.geometry import wrap
from mailbox_slam.survey import MailboxSurvey, SurveySettings
from tests.test_mailbox_discovery import Rig

WALL = [(-0.3, "Mr Jan Smith"), (-1.2, "Ms Anna Berg")]  # (y of the label, text on it)


def run(direction=(0.0, -1.0)):
    rig = Rig()
    rig.slam.parallel_wall = True  # as on the robot
    settings = SurveySettings(
        scan_mode="translate",
        scan_direction=direction,
        scan_translate_m=1.5,
        scan_step_m=0.25,
        settle_s=0.05,
        approach_speed_m_s=0.15,
        scan_translate_speed_m_s=0.2,
        image_interval_s=1.0,
        max_visual_age_s=1.1,
        frontal_approach=True,
        read_standoffs_m=(0.50, 0.30),
        max_read_attempts=4,
        second_pass_closer_m=0.05,
    )
    survey = MailboxSurvey(rig.slam, settings)
    survey.start(0)
    max_yaw, sweep_yaws = 0.0, []
    for _ in range(12000):
        proposal = survey.step(rig.timestamp)
        if proposal["capture"]:
            results, texts = [], []
            for y, text in WALL:
                tag = rig.tag()
                tag[:3, 3] = [0.6, y, 0.3]
                result = rig.result(tag)
                p = np.array(result["label_center_m"])
                if p[2] > 0 and abs(np.arctan2(p[0], p[2])) < 0.85:
                    results.append(result)
                    texts.append((tag, text))
            ocr = None
            if proposal["capture"] == "name" and survey.target_id:
                target = rig.slam.graph.landmarks[survey.target_id]["pose"][:2, 3]
                ocr = min(texts, key=lambda item: np.linalg.norm(item[0][:2, 3] - target))[1] if texts else None
            survey.observe_results(results, rig.timestamp, ocr_text=ocr)
        yaw = abs(float(wrap(rig.pose[2])))
        max_yaw = max(max_yaw, yaw)
        if not survey.scan_complete:
            sweep_yaws.append(yaw)
        if survey.phase in ("complete", "incomplete", "fault"):
            break
        rig.advance(proposal["velocity_base"])
    return survey, rig, max_yaw


def biased_map_run(visual):
    """One box at y=-0.3 whose map estimate is wrong by 6 cm after the sweep."""
    rig = Rig()
    rig.slam.parallel_wall = True
    settings = SurveySettings(
        scan_mode="translate", scan_translate_m=0.5, scan_step_m=0.25, settle_s=0.05,
        scan_translate_speed_m_s=0.2, approach_speed_m_s=0.15, image_interval_s=1.0,
        max_visual_age_s=1.1, frontal_approach=True, read_standoffs_m=(0.5,), max_read_attempts=4,
    )
    survey = MailboxSurvey(rig.slam, settings)
    if not visual:
        survey._wall_axes = lambda: None
    survey.start(0)
    biased = False
    for _ in range(12000):
        proposal = survey.step(rig.timestamp)
        if survey.scan_complete and not biased:
            biased = True
            rig.slam.graph.landmarks["mailbox_001"]["pose"][1, 3] += 0.06
            rig.slam.graph.optimize = lambda: None  # keep the wrong estimate: only the image is right
        if proposal["capture"]:
            tag = rig.tag()
            tag[:3, 3] = [0.6, -0.3, 0.3]
            result = rig.result(tag)
            visible = result["label_center_m"][2] > 0
            ocr = "Mr Jan Smith" if proposal["capture"] == "name" else None
            survey.observe_results([result] if visible else [], rig.timestamp, ocr_text=ocr)
        if survey.phase in ("complete", "incomplete", "fault"):
            break
        rig.advance(proposal["velocity_base"])
    return survey, rig


class CenteringTests(unittest.TestCase):
    def test_second_pass_centres_on_the_label_seen_in_the_image(self):
        survey, rig = biased_map_run(visual=True)
        self.assertEqual(survey.phase, "complete", survey.status())
        self.assertAlmostEqual(rig.pose[1], -0.3, delta=0.02)  # in front of the label, not the biased map

    def test_without_the_image_the_robot_follows_the_biased_map(self):
        survey, rig = biased_map_run(visual=False)
        self.assertEqual(survey.phase, "complete", survey.status())
        self.assertAlmostEqual(rig.pose[1], -0.3 + 0.06, delta=0.02)


class WallScanTests(unittest.TestCase):
    def test_translates_along_the_wall_discovers_and_reads_every_box(self):
        survey, rig, max_yaw = run()
        self.assertEqual(survey.phase, "complete", survey.status())
        self.assertTrue(survey.scan_complete)
        self.assertGreaterEqual(survey.travel_progress, 1.5 - 0.02)
        self.assertEqual(survey.yaw_progress, 0.0)  # never turned
        self.assertLess(max_yaw, 0.1)  # cap kept facing the wall
        # The second pass is made 5 cm closer than the first pass distance (measured on the map).
        first = 0.6 - rig.slam._fixed_camera_pose()[0, 3]
        self.assertAlmostEqual(survey.reading_distance, first, delta=0.03)
        self.assertAlmostEqual(survey.second_pass_standoff, survey.reading_distance - 0.05, places=3)
        names = sorted(m["name"] for m in rig.slam.mailboxes())
        self.assertEqual(names, ["Mr. Jan Smith", "Ms. Anna Berg"])
        # Stops in front of the last box it read (nearest-first from the far end), no return to the start.
        self.assertTrue(min(abs(rig.pose[1] - y) for y, _ in WALL) < 0.05, rig.pose)
        self.assertAlmostEqual(rig.pose[0], 0.05, delta=0.03)  # 5 cm closer to the wall than at the start

    def test_wall_scan_settings_are_validated(self):
        with self.assertRaises(ValueError):
            SurveySettings(scan_mode="spin")
        with self.assertRaises(ValueError):
            SurveySettings(scan_mode="translate", scan_step_m=0.8)


if __name__ == "__main__":
    unittest.main()
