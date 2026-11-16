import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from scipy.spatial.transform import Rotation

from mailbox_slam.config import validate
from mailbox_slam.constants_example import SLAM_CONFIG
from mailbox_slam.encoders import ArmKinematics, WheelOdometry
from mailbox_slam.geometry import inverse, planar, transform
from mailbox_slam.graph import PoseGraph
from mailbox_slam.station import feed_station_frame, measured_positions


def configuration():
    # SYNTHETIC test rig, not measured ElRobot constants.
    camera_mount = np.eye(4)
    camera_mount[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    camera_mount[:3, 3] = [0.1, 0, 0.3]
    wheels = []
    for i, angle in enumerate(np.radians([0, 120, 240])):
        wheels.append(
            dict(
                name=str(i),
                position_xy_m=[0.15 * np.cos(angle), 0.15 * np.sin(angle)],
                rolling_direction_xy=[-np.sin(angle), np.cos(angle)],
                radius_m=0.05,
                ticks_per_revolution=4096.0,
                sign=1,
                modulus=4096.0,
                max_rad_s=20.0,
            )
        )
    return dict(
        camera=dict(
            fx=700.0,
            fy=710.0,
            cx=320.0,
            cy=240.0,
            width=640,
            height=480,
            distortion=[0.03, -0.02, 0.0, 0.0, 0.0],
        ),
        arm=dict(
            T_base_arm=np.eye(4).tolist(),
            T_last_camera=camera_mount.tolist(),
            joints=[
                dict(
                    name="joint1",
                    T_parent_joint=np.eye(4).tolist(),
                    axis=[0, 0, 1],
                    zero_tick=2048.0,
                    ticks_per_revolution=4096.0,
                    sign=1,
                    modulus=4096.0,
                    max_rad_s=10.0,
                )
            ],
        ),
        wheels=wheels,
        settings=dict(max_sample_age_s=0.15, max_encoder_gap_s=0.5),
    )


def tag_pose():
    t = np.eye(4)
    t[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    t[:3, 3] = [1.0, 0, 0.5]
    return t


def seed(slam, timestamp=0, ticks=None, joint=2048.0):
    slam.add_wheel_sample(timestamp, np.zeros(3) if ticks is None else ticks)
    slam.add_joint_sample(timestamp, [joint])


class EncoderTests(unittest.TestCase):
    def test_required_constants_and_transforms(self):
        with self.assertRaises(ValueError):
            validate(SLAM_CONFIG)
        with self.assertRaises(ValueError):
            transform(np.zeros((4, 4)))
        config = validate(configuration())
        self.assertEqual(config["camera"]["width"], 640)

    def test_kiwi_translation_and_rotation_with_wrap(self):
        config = validate(configuration())
        wheels = WheelOdometry(config)
        ticks = np.ones(3) * 4090
        wheels.add(0, ticks)
        delta = np.array([0.01, -0.005, 0.02])
        distances = wheels.kinematics @ delta
        next_ticks = (ticks + distances / 0.05 * 4096 / (2 * np.pi)) % 4096
        actual = wheels.add(0.1, next_ticks)
        # Translation rotates slightly during the finite motion.
        from mailbox_slam.geometry import integrate

        np.testing.assert_allclose(actual, integrate(np.zeros(3), delta), atol=1e-10)

    def test_stale_wrap_gap_and_nonmonotonic_samples(self):
        wheels = WheelOdometry(validate(configuration()))
        wheels.add(0, [0, 0, 0])
        with self.assertRaises(ValueError):
            wheels.add(0.6, [0, 0, 0])
        with self.assertRaises(ValueError):
            wheels.add(0, [0, 0, 0])
        with self.assertRaises(ValueError):
            wheels.add(0.3, [0, 0, 0])  # At configured max speed wrap is ambiguous.
        with self.assertRaises(ValueError):
            wheels.buffer.at(2)

    def test_joint_forward_kinematics_and_synchronization(self):
        arm = ArmKinematics(validate(configuration()))
        arm.add(0, [0.0], units="radians")
        arm.add(0.1, [0.2], units="radians")
        pose = arm.camera_pose(0.05)
        expected = np.eye(4)
        expected[:3, :3] = Rotation.from_rotvec([0, 0, 0.1]).as_matrix()
        expected = expected @ np.asarray(configuration()["arm"]["T_last_camera"])
        np.testing.assert_allclose(pose, expected)
        with self.assertRaises(ValueError):
            arm.camera_pose(1)

    def test_nonfinite_timestamps_and_unknown_settings_are_rejected(self):
        arm = ArmKinematics(validate(configuration()))
        arm.add(0, [0.0], units="radians")
        with self.assertRaises(ValueError):
            arm.buffer.at(float("nan"))
        config = configuration()
        config["settings"]["typo"] = 1.0
        with self.assertRaisesRegex(ValueError, "Unknown SLAM setting"):
            validate(config)

    def test_unwrapped_radian_samples_do_not_have_tick_wrap_ambiguity(self):
        arm = ArmKinematics(validate(configuration()))
        arm.add(0, [0.0], units="radians")
        arm.add(0.4, [0.1], units="radians")
        self.assertAlmostEqual(arm.buffer.at(0.4)[0], 0.1)


class GraphTests(unittest.TestCase):
    def test_repeat_observations_correct_odometry_drift(self):
        settings = validate(configuration())["settings"]
        settings["observation_translation_sigma_m"] = 0.005
        graph = PoseGraph(settings)
        landmark = tag_pose()
        graph.landmarks["one"] = dict(pose=landmark.copy())
        for index, (truth, odom) in enumerate([(0, 0), (0.1, 0.13), (0.2, 0.26), (0, 0.06)]):
            graph.add_frame(index, np.array([odom, 0, 0.0]))
            graph.observations.append((index, "one", inverse(planar([truth, 0, 0])) @ landmark))
        before = graph.frames[-1]["pose"][0]
        graph.optimize()
        self.assertLess(abs(graph.frames[-1]["pose"][0]), abs(before) * 0.3)
        np.testing.assert_allclose(graph.frames[0]["pose"], [0, 0, 0])
        self.assertLess(
            graph.last_optimization["final_whitened_rms"],
            graph.last_optimization["initial_whitened_rms"],
        )


class StationTests(unittest.TestCase):
    def test_measured_position_only_and_missing_motor(self):
        state = bytearray(80)
        state[0x38:0x3A] = (1234).to_bytes(2, "little")
        state[0x2A:0x2C] = (999).to_bytes(2, "little")
        motor = SimpleNamespace(get_id=lambda: 1, get_state=lambda: state)
        descriptor = SimpleNamespace(get_serial_number=lambda: "follower")
        bus = SimpleNamespace(get_bus=lambda: descriptor, get_motors=lambda: [motor])
        frame = SimpleNamespace(get_buses=lambda: [bus])
        self.assertEqual(measured_positions(frame)[("follower", 1)], 1234)
        slam = Mock()
        feed_station_frame(slam, frame, 1, joint_sources={"joint1": ("follower", 1)})
        slam.add_joint_sample.assert_called_once_with(1, {"joint1": 1234})
        with self.assertRaises(ValueError):
            feed_station_frame(slam, frame, 2, joint_sources={"joint1": ("missing", 1)})


if __name__ == "__main__":
    unittest.main()
