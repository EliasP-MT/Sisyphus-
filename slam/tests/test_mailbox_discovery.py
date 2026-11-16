"""Synthetic fixed-arm survey, anonymous mapping, OCR staging and fault checks."""

import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

from letterbox_locator import HERE, LetterboxLocator, load_json
from mailbox_slam.discovery import MailboxDiscovery
from mailbox_slam.geometry import integrate, inverse, planar
from mailbox_slam.survey import MailboxSurvey, SurveySettings
from mailbox_slam.survey_demo import synthetic_config

GEOMETRY = load_json(HERE / "config/letterbox.json")


class Rig:
    def __init__(self, **kwargs):
        self.slam = MailboxDiscovery(
            synthetic_config(),
            geometry=GEOMETRY,
            recipients=["Mr. Jan Smith", "Ms. Anna Berg"],
            **kwargs,
        )
        self.timestamp = 0.0
        self.pose = np.zeros(3)
        self.ticks = np.ones(3) * 2048
        self.slam.add_wheel_sample(0, self.ticks)
        self.slam.add_joint_sample(0, [2048])

    def advance(self, velocity=(0.0, 0.0, 0.0), dt=0.05, wheel_scale=1.0, joint=2048.0):
        delta = np.array(velocity) * dt
        self.pose = integrate(self.pose, delta)
        distances = self.slam.wheels.kinematics @ (delta * wheel_scale)
        self.ticks = (self.ticks + distances / 0.05 * 4096 / (2 * np.pi)) % 4096
        self.timestamp += dt
        self.slam.add_wheel_sample(self.timestamp, self.ticks)
        self.slam.add_joint_sample(self.timestamp, [joint])

    def tag(self, angle=0.0, distance=1.0):
        pose = np.eye(4)
        r = Rotation.from_rotvec([0, 0, angle]).as_matrix()
        pose[:3, :3] = r @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        pose[:3, 3] = [distance * np.cos(angle), distance * np.sin(angle), 0.3]
        return pose

    def result(self, tag):
        camera = planar(self.pose) @ self.slam._fixed_camera_pose()
        t = inverse(camera) @ tag
        return dict(
            label_center_m=t[:3, 3].tolist(),
            label_rotation=t[:3, :3].tolist(),
            reprojection_px=0.1,
            label_corners_px=[[100, 100], [160, 100], [160, 150], [100, 150]],
        )


class DiscoveryTests(unittest.TestCase):
    def test_start_is_anchored_before_any_rotation_and_keeps_arm_fixed(self):
        rig = Rig()
        rig.slam.start(0)
        for _ in range(10):
            rig.advance((0, 0, 0.2))
        tag = rig.tag()
        report = rig.slam.process_locator_results(rig.timestamp, [rig.result(tag)])
        self.assertAlmostEqual(report["base_pose_xy_yaw"][2], 0.1, places=6)
        np.testing.assert_allclose(rig.slam.graph.frames[0]["pose"], [0, 0, 0])
        np.testing.assert_allclose(
            report["mailboxes"][0]["label_center_xyz_m"], [1, 0, 0.3], atol=1e-6
        )
        self.assertIsNone(report["mailboxes"][0]["name"])
        np.testing.assert_allclose(
            report["mailboxes"][0]["slot_center_xyz_m"],
            [1, -GEOMETRY["slot_from_label_center_m"][0], 0.3],
            atol=1e-6,
        )
        rig.advance(joint=2080.0)
        with self.assertRaisesRegex(ValueError, "Arm moved"):
            rig.slam.process_locator_results(rig.timestamp, [])

    def test_geometry_reassociation_and_drift_correction_without_names(self):
        rig = Rig()
        rig.slam.start(0)
        rig.slam.process_locator_results(0, [rig.result(rig.tag())])
        for _ in range(10):
            rig.advance((0.1, 0, 0), wheel_scale=1.15)
        true_x = rig.pose[0]
        raw_error = abs(rig.slam.wheels.pose[0] - true_x)
        report = rig.slam.process_locator_results(rig.timestamp, [rig.result(rig.tag())])
        self.assertEqual(len(report["mailboxes"]), 1)
        self.assertEqual(report["mailboxes"][0]["observations"], 2)
        self.assertLess(abs(report["base_pose_xy_yaw"][0] - true_x), raw_error)

    def test_names_attach_only_after_independent_database_confirmations(self):
        rig = Rig()
        rig.slam.start(0)
        rig.slam.process_locator_results(0, [rig.result(rig.tag())])
        key = rig.slam.mailboxes()[0]["id"]
        self.assertFalse(rig.slam.assign_name(key, "unreadable")["confirmed"])
        self.assertFalse(rig.slam.assign_name(key, "Mr Jan Sm1th")["confirmed"])
        self.assertFalse(
            rig.slam.assign_name(key, "Mr Jan Sm1th")["confirmed"]
        )  # same frame not two votes
        rig.advance()
        rig.slam.process_locator_results(rig.timestamp, [rig.result(rig.tag())])
        result = rig.slam.assign_name(key, "Mr. Jan Smith")
        self.assertTrue(result["confirmed"])
        self.assertEqual(rig.slam.mailboxes()[0]["name"], "Mr. Jan Smith")

    def test_spatial_ambiguity_does_not_merge_adjacent_boxes(self):
        rig = Rig(geometric_gate_m=0.15, association_margin_m=0.03)
        rig.slam.start(0)
        first = rig.tag()
        second = first.copy()
        second[1, 3] = 0.25
        report = rig.slam.process_locator_results(0, [rig.result(first), rig.result(second)])
        self.assertEqual(len(report["mailboxes"]), 2)
        midpoint = first.copy()
        midpoint[1, 3] = 0.125
        rig.advance()
        report = rig.slam.process_locator_results(rig.timestamp, [rig.result(midpoint)])
        self.assertFalse(report["accepted"])
        self.assertIn("ambiguous", report["rejected"][0]["reason"])

    def test_approach_goal_accounts_for_camera_offset(self):
        rig = Rig()
        rig.slam.start(0)
        rig.slam.process_locator_results(0, [rig.result(rig.tag())])
        goal = rig.slam.approach_pose("mailbox_001", 0.3)
        np.testing.assert_allclose(goal["base_pose_xy_yaw"], [0.6, 0, 0], atol=1e-6)
        camera = planar(goal["base_pose_xy_yaw"]) @ rig.slam._fixed_camera_pose()
        self.assertAlmostEqual(np.linalg.norm(camera[:2, 3] - [1, 0]), 0.3, places=6)
        too_high = rig.tag()
        too_high[2, 3] = 2.0
        rig.slam.graph.landmarks["mailbox_001"]["pose"] = too_high
        with self.assertRaisesRegex(ValueError, "outside"):
            rig.slam.approach_pose("mailbox_001", 0.3)

    def test_locator_detects_all_without_loading_ocr(self):
        locator = LetterboxLocator(synthetic_config()["camera"], GEOMETRY, ocr=True)
        corners = [
            np.float32([[230, 180], [290, 180], [290, 230], [230, 230]]),
            np.float32([[350, 180], [410, 180], [410, 230], [350, 230]]),
        ]
        with (
            patch("letterbox_locator.detect_labels", return_value=corners),
            patch.object(locator, "read_text", side_effect=AssertionError("OCR during discovery")),
        ):
            result = locator.locate_all(np.zeros((480, 640, 3), np.uint8))
        self.assertEqual(len(result), 2)
        self.assertIsNone(locator.ocr)
        self.assertTrue(all("ocr_text" not in item for item in result))

    def test_name_reading_cannot_override_a_duplicate_recipient_rejection(self):
        rig = Rig()
        rig.slam.start(0)
        first = rig.tag(distance=0.5)
        second = first.copy()
        second[1, 3] = 0.25
        rig.slam.process_locator_results(0, [rig.result(first), rig.result(second)])
        rig.slam.assign_name("mailbox_001", "Mr Jan Smith")
        rig.advance()
        rig.slam.process_locator_results(rig.timestamp, [rig.result(first)])
        rig.slam.assign_name("mailbox_001", "Mr Jan Smith")
        rig.advance()
        original = rig.slam.process_image
        rig.slam.process_image = lambda image, timestamp, **kwargs: (
            rig.slam.process_locator_results(timestamp, [rig.result(second)], **kwargs)
        )
        try:
            with patch.object(
                rig.slam.locator,
                "read_label_name",
                return_value=dict(
                    ocr_text="Mr Jan Smith", recipient="Mr. Jan Smith", match_score=1.0
                ),
            ):
                result = rig.slam.read_target_name(None, rig.timestamp, "mailbox_002")
        finally:
            rig.slam.process_image = original
        self.assertIsNone(result["recipient"])
        self.assertFalse(result["confirmed"])
        self.assertIsNone(rig.slam.graph.landmarks["mailbox_002"]["name"])

    def test_geometry_does_not_add_landmarks_after_sweep(self):
        rig = Rig()
        rig.slam.start(0)
        rig.slam.process_locator_results(0, [rig.result(rig.tag())])
        rig.advance()
        report = rig.slam.process_locator_results(
            rig.timestamp, [rig.result(rig.tag(angle=0.8))], allow_new=False
        )
        self.assertEqual(len(report["mailboxes"]), 1)
        self.assertFalse(report["accepted"])
        self.assertIn("discovery disabled", report["rejected"][0]["reason"])

    def test_export_includes_openings_and_origin(self):
        rig = Rig()
        rig.slam.start(0)
        rig.slam.process_locator_results(0, [rig.result(rig.tag())])
        with tempfile.TemporaryDirectory() as directory:
            result = rig.slam.save_map(Path(directory) / "map.json")
        self.assertEqual(result["origin_timestamp"], 0.0)
        self.assertIn("slot_center_xyz_m", result["mailboxes"][0])


class SurveyTests(unittest.TestCase):
    def test_full_encoder_driven_sweep_then_approach_and_read(self):
        rig = Rig()
        settings = SurveySettings(
            scan_step_rad=math.pi / 2,
            settle_s=0.05,
            approach_speed_m_s=0.15,
            approach_standoff_m=0.30,
            image_interval_s=1.0,
            max_visual_age_s=1.1,
        )
        survey = MailboxSurvey(rig.slam, settings)
        survey.start(0)
        captures = []
        for _ in range(2000):
            proposal = survey.step(rig.timestamp)
            if proposal["capture"]:
                tag = rig.tag()
                result = rig.result(tag)
                # Model a finite camera view during the base's rotation.
                p = np.array(result["label_center_m"])
                visible = p[2] > 0 and abs(np.arctan2(p[0], p[2])) < 0.4
                captures.append((survey.phase, proposal["capture"]))
                survey.observe_results(
                    [result] if visible else [],
                    rig.timestamp,
                    ocr_text="Mr Jan Smith" if proposal["capture"] == "name" else None,
                )
            if survey.phase in ("complete", "incomplete", "fault"):
                break
            rig.advance(proposal["velocity_base"])
        self.assertEqual(survey.phase, "complete", survey.status())
        self.assertTrue(survey.scan_complete)
        self.assertGreaterEqual(survey.yaw_progress, 2 * math.pi - settings.yaw_tolerance_rad)
        self.assertTrue(
            all(mode == "geometry" for phase, mode in captures if phase == "scan_capture")
        )
        self.assertEqual(rig.slam.mailboxes()[0]["name"], "Mr. Jan Smith")
        self.assertAlmostEqual(rig.pose[0], 0.6, delta=0.025)
        np.testing.assert_allclose(survey.step(rig.timestamp)["velocity_base"], [0, 0, 0])

    def test_counted_scan_finishes_full_turn_without_approach_or_ocr(self):
        for expected, final_phase in [(3, "complete"), (2, "incomplete"), (4, "incomplete")]:
            with self.subTest(expected=expected):
                rig = Rig()
                settings = SurveySettings(
                    scan_step_rad=math.pi / 2,
                    settle_s=0.05,
                    discovery_only=True,
                    expected_mailboxes=expected,
                )
                survey = MailboxSurvey(rig.slam, settings)
                survey.start(0)
                rig.slam.locator.read_label_name = Mock(
                    side_effect=AssertionError("OCR in geometry scan")
                )
                found_early = False
                for _ in range(2000):
                    proposal = survey.step(rig.timestamp)
                    self.assertEqual(proposal["velocity_base"][:2], [0.0, 0.0])
                    self.assertNotIn(survey.phase, ("approach", "name_capture"))
                    if proposal["capture"]:
                        self.assertEqual(proposal["capture"], "geometry")
                        results = []
                        for angle in (0.0, math.pi / 2, math.pi):
                            result = rig.result(rig.tag(angle=angle))
                            p = np.array(result["label_center_m"])
                            if p[2] > 0 and abs(np.arctan2(p[0], p[2])) < 0.4:
                                results.append(result)
                        survey.observe_results(results, rig.timestamp)
                        if len(rig.slam.mailboxes()) == 3 and survey.yaw_progress < 2 * math.pi:
                            found_early = True
                            self.assertFalse(survey.scan_complete)
                    if survey.phase in ("complete", "incomplete", "fault"):
                        break
                    rig.advance(proposal["velocity_base"])
                self.assertTrue(found_early)
                self.assertEqual(survey.phase, final_phase, survey.status())
                self.assertGreaterEqual(survey.yaw_progress, 2 * math.pi)
                self.assertTrue(survey.scan_complete)
                self.assertEqual(len(rig.slam.mailboxes()), 3)
                self.assertTrue(all(m["name"] is None for m in rig.slam.mailboxes()))
                self.assertEqual(survey.step(rig.timestamp)["velocity_base"], [0.0, 0.0, 0.0])
                rig.slam.locator.read_label_name.assert_not_called()
                for m in rig.slam.mailboxes():
                    self.assertEqual(len(m["label_center_xyz_m"]), 3)
                    self.assertEqual(len(m["slot_center_xyz_m"]), 3)

    def test_invalid_expected_count_is_refused(self):
        for value in (0, -1, 2.5, True):
            with self.assertRaises(ValueError):
                SurveySettings(expected_mailboxes=value)

    def test_stale_encoders_and_arm_motion_produce_stop(self):
        rig = Rig()
        survey = MailboxSurvey(rig.slam)
        survey.start(0)
        proposal = survey.step(1.0)
        self.assertEqual(proposal["phase"], "fault")
        self.assertEqual(proposal["velocity_base"], [0.0, 0.0, 0.0])
        rig = Rig()
        survey = MailboxSurvey(rig.slam)
        survey.start(0)
        rig.advance(joint=2080.0)
        proposal = survey.step(rig.timestamp)
        self.assertEqual(proposal["phase"], "fault")
        self.assertIn("Arm moved", proposal["reason"])

    def test_stationary_dwell_before_first_capture(self):
        rig = Rig()
        survey = MailboxSurvey(rig.slam, SurveySettings(settle_s=0.1))
        survey.start(0)
        self.assertIsNone(survey.step(0)["capture"])
        for _ in range(5):
            rig.advance()
            proposal = survey.step(rig.timestamp)
        self.assertEqual(proposal["capture"], "geometry")
        self.assertEqual(proposal["velocity_base"], [0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
