"""Live integration contracts without ROS, network, models, or actuators."""

import math
import unittest
from types import SimpleNamespace

import numpy as np

from mailbox_slam.config import validate
from mailbox_slam.encoders import ArmKinematics
from mailbox_slam.robot_model import Telemetry, robot_config

PARAMS = dict(
    wheel_ids=[11, 12, 9],
    wheel_mount_deg=[240.0, 0.0, 120.0],
    wheel_signs=[1, 1, 1],
    wheel_radius_m=0.05,
    base_radius_m=0.125,
    max_wheel_raw=1500,
)
CAL = dict(
    hardware_verified=True,
    max_velocity_rad_s=0.2,
    joints=[dict(name="rev_motor_01", zero_ticks=2000, sign=-1, ticks_per_rad=700)],
)
CAM = dict(fx=600, fy=600, cx=320, cy=240, width=640, height=480, distortion=[0.0] * 5)
URDF = """<robot name="test"><link name="base"/><link name="arm"/><link name="camera"/>
<joint name="rev_motor_01" type="revolute"><parent link="base"/><child link="arm"/>
<origin xyz="0 0 0.3"/><axis xyz="0 0 1"/></joint>
<joint name="optical" type="fixed"><parent link="arm"/><child link="camera"/>
<origin xyz="0.1 0 0"/></joint></robot>"""


def config(**kwargs):
    return robot_config(PARAMS, CAL, URDF, CAM, base_link="base", camera_link="camera", **kwargs)


class RobotTests(unittest.TestCase):
    def test_actual_wheel_ids_and_main_kiwi_rows(self):
        c = config()
        self.assertEqual([int(w["name"]) for w in c["wheels"]], [11, 12, 9])
        for w, angle in zip(c["wheels"], PARAMS["wheel_mount_deg"]):
            x, y = w["position_xy_m"]
            dx, dy = w["rolling_direction_xy"]
            np.testing.assert_allclose(
                [dx, dy, -y * dx + x * dy],
                [math.cos(math.radians(angle - 90)), math.sin(math.radians(angle - 90)), 0.125],
            )

    def test_calibrated_fk_preserves_origin_axis_and_scale(self):
        arm = ArmKinematics(validate(config()))
        arm.add(0, [2000])
        np.testing.assert_allclose(arm.camera_pose(0)[:3, 3], [0.1, 0, 0.3])
        self.assertAlmostEqual(
            config()["arm"]["joints"][0]["ticks_per_revolution"], 700 * 2 * math.pi
        )

    def test_mount_must_be_known_and_verified(self):
        with self.assertRaisesRegex(ValueError, "hardware_verified"):
            robot_config(
                PARAMS,
                {**CAL, "hardware_verified": False},
                URDF,
                CAM,
                base_link="base",
                camera_link="camera",
            )
        with self.assertRaisesRegex(ValueError, "No URDF chain"):
            robot_config(PARAMS, CAL, URDF, CAM, base_link="base", camera_link="unknown")
        pose = np.eye(4)
        pose[0, 3] = 0.2
        c = robot_config(
            PARAMS,
            CAL,
            URDF,
            CAM,
            base_link="base",
            camera_link="unknown",
            camera_mount=dict(parent_link="arm", T_parent_camera=pose),
        )
        self.assertAlmostEqual(c["arm"]["T_last_camera"][0][3], 0.2)

    def test_partial_joints_do_not_refresh_missing_joint(self):
        c = config()
        c["arm"]["joints"].append({**c["arm"]["joints"][0], "name": "second"})
        t = Telemetry(c, max_skew=0.08)
        self.assertIsNone(t.joint_sample(0, ["rev_motor_01"], [0.0]))
        self.assertIsNone(t.joint_sample(0.2, ["second"], [0.0]))
        self.assertEqual(t.joint_sample(0.21, ["rev_motor_01"], [0.0]), (0.2, [0.0, 0.0]))
        self.assertIsNone(t.joint_sample(0.22, ["rev_motor_01"], [0.0]))

    def test_wheels_use_per_motor_age_and_reject_faults(self):
        t = Telemetry(config())
        motors = [
            SimpleNamespace(
                motor_id=i,
                ever_received=True,
                fresh=True,
                responding=True,
                fault="",
                age_s=0.02,
                position_ticks=1234,
            )
            for i in [11, 12, 9]
        ]
        self.assertEqual(t.wheel_sample(1.0, motors), (0.98, [1234] * 3))
        self.assertIsNone(t.wheel_sample(1.0, motors))
        motors[1].fresh = False
        with self.assertRaises(ValueError):
            t.wheel_sample(2.0, motors)


class ArmBaseTests(unittest.TestCase):
    def test_nonidentity_arm_base_is_composed_before_arm_fk(self):
        pose = np.eye(4)
        pose[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        pose[:3, 3] = [0.027, 0.0025, 0.093]
        c = config(arm_base={"T_base_arm": pose.tolist()})
        arm = ArmKinematics(validate(c))
        arm.add(0, [2000])
        np.testing.assert_allclose(arm.camera_pose(0)[:3, 3], [0.027, 0.1025, 0.393])
