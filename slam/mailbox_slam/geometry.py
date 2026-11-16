"""Metric rigid transforms: T_a_b maps coordinates from frame b into frame a."""

import numpy as np
from scipy.spatial.transform import Rotation


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def transform(value, name="transform"):
    t = np.asarray(value, dtype=float)
    if (
        t.shape != (4, 4)
        or not np.isfinite(t).all()
        or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-6)
        or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-5)
    ):
        raise ValueError(f"{name} must be a finite 4x4 rigid transform")
    return t.copy()


def inverse(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def planar(pose):
    x, y, yaw = pose
    out = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    out[:2, :2] = [[c, -s], [s, c]]
    out[:2, 3] = [x, y]
    return out


def se2(t):
    return np.array([t[0, 3], t[1, 3], np.arctan2(t[1, 0], t[0, 0])])


def between(a, b):
    return se2(inverse(planar(a)) @ planar(b))


def integrate(pose, delta):
    """Integrate a body-frame twist displacement using the SE(2) exponential."""
    dx, dy, yaw = delta
    if abs(yaw) > 1e-8:
        a, b = np.sin(yaw) / yaw, (1 - np.cos(yaw)) / yaw
        dx, dy = a * dx - b * dy, b * dx + a * dy
    return se2(planar(pose) @ planar([dx, dy, yaw]))


def pose6(t):
    return np.r_[t[:3, 3], Rotation.from_matrix(t[:3, :3]).as_rotvec()]


def rigid(pose):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_rotvec(pose[3:]).as_matrix()
    out[:3, 3] = pose[:3]
    return out


def residual6(prediction, observation):
    return np.r_[
        prediction[:3, 3] - observation[:3, 3],
        Rotation.from_matrix(observation[:3, :3].T @ prediction[:3, :3]).as_rotvec(),
    ]


def pose_ambiguous(candidates, settings):
    if len(candidates) < 2:
        return False
    a, b = candidates[:2]
    angle = Rotation.from_matrix(a["pose"][:3, :3].T @ b["pose"][:3, :3]).magnitude()
    return (
        b["rms_px"] - a["rms_px"] < settings["ambiguity_error_gap_px"]
        and angle > settings["ambiguity_angle_rad"]
    )
