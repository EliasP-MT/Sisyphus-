"""Validate imported hardware constants; never substitute guessed geometry."""

import importlib
import json
from copy import deepcopy
from pathlib import Path

import numpy as np

from .geometry import transform

DEFAULTS = dict(
    max_sample_age_s=0.5,
    max_encoder_gap_s=0.5,
    odom_translation_sigma_m=0.03,
    odom_yaw_sigma_rad=0.05,
    observation_translation_sigma_m=0.02,
    observation_rotation_sigma_rad=0.12,
    max_reprojection_px=2.0,
    ambiguity_error_gap_px=0.15,
    ambiguity_angle_rad=0.25,
    max_frames=300,
    max_optimizer_evaluations=100,
)


def positive(value, name):
    if value is None or not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a measured positive finite number")
    return float(value)


def validate(config):
    c = deepcopy(config)
    try:
        camera = c["camera"]
        for key in ("fx", "fy", "width", "height"):
            positive(camera[key], f"camera.{key}")
        if any(int(camera[key]) != camera[key] for key in ("width", "height")):
            raise ValueError("Camera dimensions must be integer pixels")
        if any(not np.isfinite(camera[k]) for k in ("cx", "cy")):
            raise ValueError("camera.cx/cy must be finite")
        d = np.asarray(camera["distortion"], float)
        if d.ndim != 1 or d.size not in (4, 5, 8, 12, 14) or not np.isfinite(d).all():
            raise ValueError("camera.distortion must contain 4, 5, 8, 12, or 14 coefficients")
        arm = c["arm"]
        transform(arm["T_base_arm"], "arm.T_base_arm")
        transform(arm["T_last_camera"], "arm.T_last_camera")
        if not arm["joints"]:
            raise ValueError("arm.joints must describe the serial arm chain")
        ids = []
        for j in arm["joints"]:
            ids.append(str(j["name"]))
            transform(j["T_parent_joint"], "joint.T_parent_joint")
            axis = np.asarray(j["axis"], float)
            if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-8:
                raise ValueError("Each joint needs a nonzero finite 3D axis")
            positive(j["ticks_per_revolution"], "joint.ticks_per_revolution")
            positive(j["max_rad_s"], "joint.max_rad_s")
            if (
                j["zero_tick"] is None
                or not np.isfinite(j["zero_tick"])
                or j["sign"] not in (-1, 1)
            ):
                raise ValueError("Each joint needs zero_tick and sign (+1/-1)")
            if j.get("modulus") is not None:
                positive(j["modulus"], "joint.modulus")
        if len(set(ids)) != len(ids):
            raise ValueError("Joint names must be unique")
        wheels = c["wheels"]
        if len(wheels) < 3:
            raise ValueError("Kiwi odometry needs at least three configured wheels")
        rows, names = [], []
        for w in wheels:
            names.append(str(w["name"]))
            positive(w["radius_m"], "wheel.radius_m")
            positive(w["ticks_per_revolution"], "wheel.ticks_per_revolution")
            positive(w["max_rad_s"], "wheel.max_rad_s")
            xy, direction = (
                np.asarray(w["position_xy_m"], float),
                np.asarray(w["rolling_direction_xy"], float),
            )
            if (
                xy.shape != (2,)
                or direction.shape != (2,)
                or not np.isfinite(xy).all()
                or not np.isfinite(direction).all()
                or not np.isclose(np.linalg.norm(direction), 1)
            ):
                raise ValueError(
                    "Wheel position must be finite XY and rolling direction a unit XY vector"
                )
            if w["sign"] not in (-1, 1):
                raise ValueError("wheel.sign must be +1/-1")
            if w.get("modulus") is not None:
                positive(w["modulus"], "wheel.modulus")
            x, y = xy
            a, b = direction
            rows.append([a, b, -y * a + x * b])
        if len(set(names)) != len(names) or np.linalg.matrix_rank(rows) != 3:
            raise ValueError("Wheel names must be unique and geometry must constrain x, y, yaw")
        c["settings"] = {**DEFAULTS, **c.get("settings", {})}
        for key, value in c["settings"].items():
            if key not in DEFAULTS:
                raise ValueError(f"Unknown SLAM setting: {key}")
            positive(value, f"settings.{key}")
        for key in ("max_frames", "max_optimizer_evaluations"):
            if int(c["settings"][key]) != c["settings"][key]:
                raise ValueError(f"settings.{key} must be an integer")
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Missing/invalid hardware constant: {exc}") from exc
    return c


def load_config(path=None, constants_module=None):
    """Load JSON, or import a Python module exposing SLAM_CONFIG."""
    if (path is None) == (constants_module is None):
        raise ValueError("Supply exactly one JSON path or constants module")
    raw = (
        json.loads(Path(path).read_text())
        if path is not None
        else importlib.import_module(constants_module).SLAM_CONFIG
    )
    return validate(raw)
