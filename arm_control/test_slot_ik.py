import unittest

import numpy as np

from planner import Arm
from slot_ik import camera_pose, point_in_base, solve_tcp_ik
from test_arm import URDF, calibration


class SlotIkTests(unittest.TestCase):
    def setUp(self):
        self.c = calibration()
        self.arm = Arm(URDF, self.c)
        self.q0 = np.array([-0.3, 0.4, -0.3, 0.9, -0.5, 0.8, 0.3])
        self.offset = np.array([0.0, 0.106, 0.0])  # fingertips, Gripper_Base frame

    def tip(self, q):
        T = self.arm.fk(q)
        return T[:3, 3] + T[:3, :3] @ self.offset

    def test_reaches_a_point_with_the_wrist_pivoted_by_90_degrees(self):
        target = self.tip(self.q0 + np.array([0.15, -0.1, 0.1, -0.1, 0.05, 0.0, 0.0]))
        pinned = {6: self.q0[6] + np.pi / 2}
        q, info = solve_tcp_ik(self.arm, self.q0, target, self.offset, pinned=pinned)
        self.assertLess(np.linalg.norm(self.tip(q) - target), 0.003)
        self.assertAlmostEqual(q[6], self.q0[6] + np.pi / 2, places=6)  # the imposed joint is exact
        self.assertLess(info['position_error_m'], 0.003)

    def test_gripper_axis_follows_the_approach_direction(self):
        q_true = self.q0 + np.array([0.1, 0.05, 0.0, -0.1, 0.0, 0.1, 0.0])
        target = self.tip(q_true)
        axis = (self.arm.fk(q_true)[:3, :3] @ np.array([0, 1.0, 0]))
        q, info = solve_tcp_ik(self.arm, self.q0, target, self.offset, approach_dir=axis, pinned={6: self.q0[6]})
        self.assertLess(info['axis_error_deg'], 12)

    def test_pinned_joint_outside_its_limits_is_refused(self):
        with self.assertRaisesRegex(ValueError, 'outside its limits'):
            solve_tcp_ik(self.arm, self.q0, self.tip(self.q0), self.offset, pinned={6: 9.0})

    def test_unreachable_target_is_refused(self):
        with self.assertRaisesRegex(ValueError, 'No collision-free'):
            solve_tcp_ik(self.arm, self.q0, [1.5, 1.5, 1.5], self.offset, restarts=3)

    def test_camera_pose_and_a_point_in_the_camera_frame(self):
        mount = dict(T_parent_camera=np.eye(4).tolist())
        T = camera_pose(self.arm, self.q0, mount)
        np.testing.assert_allclose(T, self.arm.fk(self.q0))
        shifted = np.eye(4); shifted[:3, 3] = [0.075, 0.03, 0.0]
        T2 = camera_pose(self.arm, self.q0, dict(T_parent_camera=shifted.tolist()))
        np.testing.assert_allclose(point_in_base(T2, [0, 0, 0]),
                                   self.arm.fk(self.q0)[:3, 3] + self.arm.fk(self.q0)[:3, :3] @ [0.075, 0.03, 0])
        # A roll correction turns the whole mount about the gripper's forward axis (+y of Gripper_Base):
        # the camera keeps its heading and its distance along that axis, and its sideways offset
        # moves round the axis (7.5 cm in +x becomes 7.5 cm in +z for -90 degrees).
        forward = np.eye(4)  # a mount like the real one: optical axis = +y of Gripper_Base
        forward[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
        forward[:3, 3] = [0.075, 0.03, 0.0]
        plain = camera_pose(self.arm, self.q0, dict(T_parent_camera=forward.tolist()))
        rolled = camera_pose(self.arm, self.q0, dict(T_parent_camera=forward.tolist(), roll_correction_deg=-90))
        base = self.arm.fk(self.q0)
        np.testing.assert_allclose(rolled[:3, 3], base[:3, 3] + base[:3, :3] @ [0.0, 0.03, 0.075], atol=1e-9)
        np.testing.assert_allclose(rolled[:3, 2], plain[:3, 2], atol=1e-9)  # same optical axis
        with self.assertRaises(ValueError):
            camera_pose(self.arm, self.q0, dict(T_parent_camera=[[1, 0], [0, 1]]))


if __name__ == '__main__':
    unittest.main()
