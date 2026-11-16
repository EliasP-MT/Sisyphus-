"""Shared encoder state and map export for the geometry-first survey."""

import json
import tempfile
from pathlib import Path

import numpy as np

from .config import validate
from .encoders import ArmKinematics, WheelOdometry
from .graph import PoseGraph


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def write_json(path, payload):
    """Serialize before writing, then atomically replace the destination."""
    path = Path(path)
    text = (
        json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False, default=json_value)
        + "\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
        ) as stream:
            temporary = Path(stream.name)
            stream.write(text)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class MappingCore:
    """Single-threaded measured encoder inputs and a sparse pose graph."""

    def __init__(self, config):
        self.config = validate(config)
        self.settings = self.config["settings"]
        self.wheels = WheelOdometry(self.config)
        self.arm = ArmKinematics(self.config)
        self.graph = PoseGraph(self.settings)
        self.raw_observations = []
        self.last_report = None

    def add_wheel_sample(self, timestamp, wheel_positions):
        return self.wheels.add(timestamp, wheel_positions)

    def add_joint_sample(self, timestamp, joint_positions, units="ticks"):
        return self.arm.add(timestamp, joint_positions, units)

    def mailboxes(self):
        return [
            dict(
                id=key,
                name=landmark["name"],
                identity=landmark["identity"],
                position_xyz_m=landmark["pose"][:3, 3].tolist(),
                T_map_tag=landmark["pose"].tolist(),
                outward_normal_xyz=(-landmark["pose"][:3, 2]).tolist(),
                observations=landmark["observations"],
            )
            for key, landmark in self.graph.landmarks.items()
        ]

    def map_payload(self):
        return dict(
            format_version=1,
            map_kind="sparse_3d_mailbox_landmarks",
            axes="x forward, y left, z up; origin at initial robot base",
            units="metres/radians",
            config=self.config,
            mailboxes=self.mailboxes(),
            frames=[
                dict(
                    timestamp=f["timestamp"],
                    pose_xy_yaw=f["pose"].tolist(),
                    odom_xy_yaw=f["odom"].tolist(),
                )
                for f in self.graph.frames
            ],
            observations=self.raw_observations,
            optimization=self.graph.last_optimization,
        )

    def save_map(self, path):
        payload = self.map_payload()
        write_json(path, payload)
        return payload
