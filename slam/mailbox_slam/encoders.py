"""Measured encoder conversion, Kiwi kinematics, and time alignment."""

import bisect
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import integrate, transform, wrap


class SampleBuffer:
    def __init__(self, max_age, angles=False):
        self.samples = deque(maxlen=5000)
        self.max_age = max_age
        self.angles = angles

    def add(self, timestamp, values):
        t, values = float(timestamp), np.asarray(values, float)
        if not np.isfinite(t) or not np.isfinite(values).all():
            raise ValueError("Encoder samples must be finite")
        if self.samples and t <= self.samples[-1][0]:
            raise ValueError("Encoder timestamps must increase strictly")
        self.samples.append((t, values.copy()))

    def at(self, timestamp):
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError("Timestamp must be finite")
        if not self.samples:
            raise ValueError("No encoder samples available")
        times = [s[0] for s in self.samples]
        i = bisect.bisect_left(times, timestamp)
        if i < len(times) and times[i] == timestamp:
            return self.samples[i][1].copy()
        if i in (0, len(times)):
            t, value = self.samples[0 if i == 0 else -1]
            if abs(t - timestamp) > self.max_age:
                raise ValueError(
                    "Stale/unavailable encoder sample at image timestamp "
                    f"(t={timestamp:.3f}, buffer {times[0]:.3f}..{times[-1]:.3f}, "
                    f"{len(times)} samples, {'joint angles' if self.angles is False and self.samples and len(self.samples[0][1]) == 7 else 'wheels'}, "
                    f"max_age={self.max_age})"
                )
            return value.copy()
        before, after = self.samples[i - 1], self.samples[i]
        if max(timestamp - before[0], after[0] - timestamp) > self.max_age:
            raise ValueError("Encoder interpolation gap exceeds max_sample_age_s")
        delta = after[1] - before[1]
        if self.angles:  # Only the planar yaw is cyclic; joint values are unwrapped.
            delta = delta.copy()
            delta[2] = wrap(delta[2])
        return before[1] + (timestamp - before[0]) / (after[0] - before[0]) * delta


def values_in_order(values, specs):
    if isinstance(values, dict):
        try:
            values = [values[str(s["name"])] for s in specs]
        except KeyError as exc:
            raise ValueError(f"Missing encoder {exc}") from exc
    result = np.asarray(values, float)
    if result.shape != (len(specs),) or not np.isfinite(result).all():
        raise ValueError("Supply one finite measured position per configured encoder")
    return result


def tick_delta(new, previous, spec):
    delta = new - previous
    modulus = spec.get("modulus")
    if modulus is not None:
        if not 0 <= new < modulus or not 0 <= previous < modulus:
            raise ValueError("Modulo encoder position outside its configured range")
        delta = (delta + modulus / 2) % modulus - modulus / 2
    return delta * (2 * np.pi / spec["ticks_per_revolution"]) * spec["sign"]


def check_interval(previous_t, timestamp, settings):
    dt = timestamp - previous_t
    if dt <= 0 or dt > settings["max_encoder_gap_s"]:
        raise ValueError(
            "Encoder samples out of order or gap too long; restart after resolving telemetry loss"
        )
    return dt


def check_motion(delta, dt, spec, *, modulo=True):
    modulus = spec.get("modulus") if modulo else None
    if modulus is not None:
        half_range = modulus * np.pi / spec["ticks_per_revolution"]
        if dt * spec["max_rad_s"] >= half_range:
            raise ValueError("Encoder interval permits an ambiguous wrap; sample faster")
    # 25 % plus one tick of slack: sample times jitter (age-corrected stamps), which makes a
    # wheel running near its limit look briefly faster than physically possible.
    slack = 2 * np.pi / spec["ticks_per_revolution"] if spec.get("ticks_per_revolution") else 0.0
    if abs(delta) > spec["max_rad_s"] * dt * 1.25 + slack + 1e-6:
        raise ValueError(
            f"Implausible encoder speed for {spec['name']} "
            f"({abs(delta) / dt:.2f} rad/s over {dt * 1000:.0f} ms, limit {spec['max_rad_s']:.2f})"
        )


class WheelOdometry:
    def __init__(self, config):
        self.specs, self.settings = config["wheels"], config["settings"]
        rows = []
        for w in self.specs:
            x, y = w["position_xy_m"]
            a, b = w["rolling_direction_xy"]
            rows.append([a, b, -y * a + x * b])
        self.kinematics = np.asarray(rows)
        self.previous = None
        self.pose = np.zeros(3)
        self.buffer = SampleBuffer(self.settings["max_sample_age_s"], angles=True)

    def add(self, timestamp, positions):
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError("Encoder timestamp must be finite")
        ticks = values_in_order(positions, self.specs)
        for tick, spec in zip(ticks, self.specs):
            if spec.get("modulus") is not None and not 0 <= tick < spec["modulus"]:
                raise ValueError("Wheel tick outside configured modulus")
        pose = self.pose.copy()
        if self.previous is not None:
            previous_t, previous_ticks = self.previous
            dt = check_interval(previous_t, timestamp, self.settings)
            distances = []
            for tick, old, spec in zip(ticks, previous_ticks, self.specs):
                angle = tick_delta(tick, old, spec)
                check_motion(angle, dt, spec)
                distances.append(spec["radius_m"] * angle)
            body_delta = np.linalg.lstsq(self.kinematics, distances, rcond=None)[0]
            pose = integrate(pose, body_delta)
        self.buffer.add(timestamp, pose)
        self.pose, self.previous = pose, (timestamp, ticks)
        return self.pose.copy()


class ArmKinematics:
    def __init__(self, config):
        self.arm, self.settings = config["arm"], config["settings"]
        self.specs = self.arm["joints"]
        self.previous = None
        self.buffer = SampleBuffer(self.settings["max_sample_age_s"])

    def add(self, timestamp, positions, units="ticks"):
        if units not in ("ticks", "radians"):
            raise ValueError("Joint units must be ticks or radians")
        timestamp = float(timestamp)
        values = values_in_order(positions, self.specs)
        if self.previous is not None:
            old_t, old_values, old_angles, old_units = self.previous
            if old_units != units:
                raise ValueError("Do not change joint units during a mapping session")
            dt = check_interval(old_t, timestamp, self.settings)
        angles = []
        for i, (value, spec) in enumerate(zip(values, self.specs)):
            if (
                units == "ticks"
                and spec.get("modulus") is not None
                and not 0 <= value < spec["modulus"]
            ):
                raise ValueError("Joint tick outside configured modulus")
            if self.previous is None:
                angle = tick_delta(value, spec["zero_tick"], spec) if units == "ticks" else value
            else:
                delta = (
                    tick_delta(value, old_values[i], spec)
                    if units == "ticks"
                    else value - old_angles[i]
                )
                check_motion(delta, dt, spec, modulo=units == "ticks")
                angle = old_angles[i] + delta
            angles.append(angle)
        angles = np.asarray(angles)
        self.buffer.add(timestamp, angles)
        self.previous = timestamp, values, angles, units
        return angles.copy()

    def camera_pose(self, timestamp):
        return self.forward(self.buffer.at(timestamp))

    def forward(self, angles):
        angles = values_in_order(angles, self.specs)
        t = transform(self.arm["T_base_arm"])
        for angle, spec in zip(angles, self.specs):
            axis = np.asarray(spec["axis"], float)
            axis /= np.linalg.norm(axis)
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            t = t @ transform(spec["T_parent_joint"]) @ rotation
        return t @ transform(self.arm["T_last_camera"])
