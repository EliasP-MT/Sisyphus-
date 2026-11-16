"""Frontal pass: translate in front of a box (no turning towards it), read from the first
standoff, come closer only when the reading fails."""

import math
import unittest

import numpy as np

from mailbox_slam.geometry import wrap
from mailbox_slam.survey import MailboxSurvey, SurveySettings
from tests.test_mailbox_discovery import Rig


def run(readable_within_m, box_y=0.3):
    rig = Rig()
    settings = SurveySettings(
        scan_step_rad=math.pi / 2,
        settle_s=0.05,
        approach_speed_m_s=0.15,
        image_interval_s=1.0,
        max_visual_age_s=1.1,
        frontal_approach=True,
        read_standoffs_m=(0.50, 0.30),
        attempts_per_standoff=2,
        max_read_attempts=4,
    )
    survey = MailboxSurvey(rig.slam, settings)
    survey.start(0)
    name_distances, max_yaw_after_sweep = [], 0.0
    for _ in range(4000):
        proposal = survey.step(rig.timestamp)
        if proposal["capture"]:
            tag = rig.tag()
            tag[1, 3] += box_y  # the box is beside the start position
            result = rig.result(tag)
            p = np.array(result["label_center_m"])
            visible = p[2] > 0 and abs(np.arctan2(p[0], p[2])) < 0.8
            text = None
            if proposal["capture"] == "name":
                name_distances.append(float(np.linalg.norm(p)))
                if np.linalg.norm(p) < readable_within_m:
                    text = "Mr Jan Smith"
            survey.observe_results([result] if visible else [], rig.timestamp, ocr_text=text)
        if survey.scan_complete:
            max_yaw_after_sweep = max(max_yaw_after_sweep, abs(float(wrap(rig.pose[2]))))
        if survey.phase in ("complete", "incomplete", "fault"):
            break
        rig.advance(proposal["velocity_base"])
    return survey, rig, name_distances, max_yaw_after_sweep


class FrontalReadingTests(unittest.TestCase):
    def test_reads_from_the_first_standoff_without_coming_closer(self):
        survey, rig, distances, yaw = run(readable_within_m=2.0)
        self.assertEqual(survey.phase, "complete", survey.status())
        self.assertEqual(len(distances), 2)  # two confirming readings, both from far away
        self.assertTrue(all(d > 0.45 for d in distances))
        self.assertAlmostEqual(rig.pose[1], 0.3, delta=0.03)  # it translated in front of the box
        self.assertLess(yaw, 0.1)  # heading stays along the face normal
        self.assertAlmostEqual(rig.pose[0], 0.4, delta=0.04)  # 50 cm standoff, not 30 cm

    def test_comes_closer_only_after_failed_readings(self):
        survey, rig, distances, yaw = run(readable_within_m=0.45)
        self.assertEqual(survey.phase, "complete", survey.status())
        far = [d for d in distances if d >= 0.45]
        self.assertEqual(len(far), 2)  # two failed readings at the first standoff
        self.assertAlmostEqual(rig.pose[0], 0.6, delta=0.04)  # then the 30 cm standoff
        self.assertLess(yaw, 0.1)

    def test_commanded_wheel_speed_stays_below_the_fraction_of_the_maximum(self):
        rig = Rig()
        survey = MailboxSurvey(rig.slam, SurveySettings(max_wheel_fraction=0.6))
        wheels = rig.slam.wheels
        for twist in ([1.0, 0.0, 0.0], [0.0, 1.0, 2.0], [0.3, -0.2, 3.0]):
            limited = survey._limit_wheel_speed(twist)
            rates = [abs(k @ limited) / w["radius_m"] for k, w in zip(wheels.kinematics, wheels.specs)]
            limits = [0.6 * w["max_rad_s"] for w in wheels.specs]
            self.assertTrue(all(r <= lim + 1e-9 for r, lim in zip(rates, limits)), twist)
            np.testing.assert_allclose(limited / np.linalg.norm(limited), np.array(twist) / np.linalg.norm(twist))
        np.testing.assert_allclose(survey._limit_wheel_speed([0, 0, 0]), [0, 0, 0])


if __name__ == "__main__":
    unittest.main()
