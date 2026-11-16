"""Read main's calibrated robot model without changing or importing its controllers."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import transform


def urdf_origin(joint):
    origin = joint.find("origin")
    pose = np.eye(4)
    if origin is not None:
        pose[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        pose[:3, :3] = Rotation.from_euler(
            "xyz", [float(v) for v in origin.get("rpy", "0 0 0").split()]
        ).as_matrix()
    return transform(pose, "URDF origin")


def robot_config(
    params, calibration, urdf, camera, *, base_link, camera_link, camera_mount=None, arm_base=None
):
    """Construct SLAM constants matching main's Kiwi and calibrated joint conventions.

    camera_mount is {parent_link, T_parent_camera}; only needed when the optical
    camera link is not in the URDF. It must be measured, never inferred.
    arm_base is {T_base_arm}: pose of the URDF base link in the Kiwi robot frame
    (x forward, origin at the wheel-contact centroid). Defaults to identity.
    """
    if calibration.get("hardware_verified") is not True:
        raise ValueError("Arm calibration must be hardware_verified")
    calibrated = {j["name"]: j for j in calibration["joints"]}
    root = ET.fromstring(urdf)
    parents = {j.find("child").get("link"): j for j in root.findall("joint")}
    tail = np.eye(4)
    link = camera_link
    if camera_mount is not None:
        link = camera_mount["parent_link"]
        tail = transform(camera_mount["T_parent_camera"], "measured camera mount")
    path, visited = [], set()
    while link != base_link:
        if link in visited or link not in parents:
            raise ValueError(
                f"No URDF chain from {base_link} to {camera_link}; supply a measured camera mount"
            )
        visited.add(link)
        joint = parents[link]
        path.insert(0, joint)
        link = joint.find("parent").get("link")
    accumulated, joints = np.eye(4), []
    for joint in path:
        accumulated = accumulated @ urdf_origin(joint)
        if joint.get("type") == "fixed":
            continue
        name = joint.get("name")
        if joint.get("type") not in ("revolute", "continuous") or name not in calibrated:
            raise ValueError(f"Uncalibrated/unsupported camera-chain joint: {name}")
        c = calibrated[name]
        joints.append(
            dict(
                name=name,
                T_parent_joint=accumulated.tolist(),
                axis=[float(v) for v in joint.find("axis").get("xyz", "1 0 0").split()]
                if joint.find("axis") is not None
                else [1, 0, 0],
                zero_tick=c["zero_ticks"],
                sign=c["sign"],
                ticks_per_revolution=2 * math.pi * c.get("ticks_per_rad", 4096 / (2 * math.pi)),
                modulus=None,
                max_rad_s=calibration["max_velocity_rad_s"],
            )
        )
        accumulated = np.eye(4)
    if not joints:
        raise ValueError("The selected camera must be attached to the calibrated arm chain")
    wheels = []
    for role, motor_id, mount, sign in zip(
        ("wheel_left", "wheel_back", "wheel_right"),
        params["wheel_ids"],
        params["wheel_mount_deg"],
        params["wheel_signs"],
    ):
        angle = math.radians(mount - 90)
        dx, dy = math.cos(angle), math.sin(angle)
        radius = params["base_radius_m"]
        wheels.append(
            dict(
                name=str(motor_id),
                role=role,
                position_xy_m=[radius * dy, -radius * dx],
                rolling_direction_xy=[dx, dy],
                radius_m=params["wheel_radius_m"],
                sign=sign,
                ticks_per_revolution=4096,
                modulus=4096,
                max_rad_s=params["max_wheel_raw"] * 2 * math.pi / 4096,
            )
        )
    return dict(
        camera=camera,
        arm=dict(
            T_base_arm=(
                transform(arm_base["T_base_arm"], "arm base") if arm_base else np.eye(4)
            ).tolist(),
            joints=joints,
            T_last_camera=(accumulated @ tail).tolist(),
        ),
        wheels=wheels,
    )


def load_robot_config(robot_yaml, calibration_path, urdf_path, camera_path, **kwargs):
    import yaml

    parameters = yaml.safe_load(Path(robot_yaml).read_text())["/robot/core"]["ros__parameters"]
    config = robot_config(
        parameters,
        json.loads(Path(calibration_path).read_text()),
        Path(urdf_path).read_text(),
        json.loads(Path(camera_path).read_text()),
        **kwargs,
    )
    return config, parameters


class Telemetry:
    """Coherent measured samples; never refresh a joint using another joint's stamp."""

    def __init__(self, config, max_skew=0.08):
        self.joint_names = [j["name"] for j in config["arm"]["joints"]]
        self.wheel_ids = [int(w["name"]) for w in config["wheels"]]
        self.max_skew = max_skew
        self.joints = {}
        self.last_joint = self.last_wheel = -float("inf")

    def joint_sample(self, timestamp, names, positions):
        for name, value in zip(names, positions):
            if name in self.joint_names:
                self.joints[name] = timestamp, float(value)
        if not all(name in self.joints for name in self.joint_names):
            return None
        samples = [self.joints[name] for name in self.joint_names]
        oldest, newest = min(s[0] for s in samples), max(s[0] for s in samples)
        if newest - oldest > self.max_skew or oldest <= self.last_joint:
            return None
        self.last_joint = oldest
        return oldest, [value for _, value in samples]

    def wheel_sample(self, timestamp, motors):
        selected = {m.motor_id: m for m in motors if m.motor_id in self.wheel_ids}
        if len(selected) != len(self.wheel_ids):
            return None
        ordered = [selected[i] for i in self.wheel_ids]
        if any(
            not m.ever_received or not m.fresh or not m.responding or m.fault or m.age_s < 0
            for m in ordered
        ):
            raise ValueError("Wheel feedback is missing, stale or faulty")
        stamps = [timestamp - m.age_s for m in ordered]
        if max(stamps) - min(stamps) > self.max_skew:
            raise ValueError("Wheel sample skew exceeds synchronization limit")
        stamp = min(stamps)
        if stamp <= self.last_wheel:
            return None
        self.last_wheel = stamp
        return stamp, [m.position_ticks for m in ordered]
