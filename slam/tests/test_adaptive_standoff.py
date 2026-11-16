import types
import unittest

from mailbox_slam.survey import MailboxSurvey, SurveySettings


class AdaptiveStandoffTests(unittest.TestCase):
    def survey(self, needed):
        def approach_pose(key, standoff):
            if standoff < needed:
                raise ValueError("Label would be outside the fixed camera view at the approach goal")
            return dict(standoff=standoff)

        return MailboxSurvey(types.SimpleNamespace(approach_pose=approach_pose), SurveySettings())

    def test_nominal_standoff_is_kept_when_the_label_stays_in_view(self):
        self.assertEqual(self.survey(0.0)._approach_goal("m")["standoff"], 0.30)

    def test_low_box_is_approached_from_the_smallest_standoff_that_keeps_it_in_view(self):
        self.assertAlmostEqual(self.survey(0.44)._approach_goal("m")["standoff"], 0.45)

    def test_gives_up_beyond_the_maximum_and_other_errors_propagate(self):
        with self.assertRaisesRegex(ValueError, "outside the fixed camera view"):
            self.survey(5.0)._approach_goal("m")

        def broken(key, standoff):
            raise ValueError("Mailbox face is not approximately vertical")

        survey = MailboxSurvey(types.SimpleNamespace(approach_pose=broken), SurveySettings())
        with self.assertRaisesRegex(ValueError, "not approximately vertical"):
            survey._approach_goal("m")


if __name__ == "__main__":
    unittest.main()


class ParallelWallTests(unittest.TestCase):
    def test_goal_ignores_the_noisy_face_normal_when_the_wall_is_known(self):
        import numpy as np

        from tests.test_mailbox_discovery import Rig

        rig = Rig()
        rig.slam.start(0)
        tilted = rig.tag(angle=0.8)  # a face normal estimate that is 46 degrees off
        tilted[:3, 3] = [1.0, 0.3, 0.3]
        rig.slam.process_locator_results(0, [rig.result(tilted)])
        free = rig.slam.approach_pose("mailbox_001", 0.3)
        self.assertGreater(abs(free["base_pose_xy_yaw"][2]), 0.3)  # follows the noisy normal
        rig.slam.parallel_wall = True
        wall = rig.slam.approach_pose("mailbox_001", 0.3)
        np.testing.assert_allclose(wall["base_pose_xy_yaw"][2], 0.0, atol=1e-9)  # keeps facing the wall
        np.testing.assert_allclose(wall["base_pose_xy_yaw"][:2], [0.6, 0.3], atol=1e-6)
