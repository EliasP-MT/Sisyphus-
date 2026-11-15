"""Verified arm calibration: validation and tick/radian conversion without clipping.

Mirrors arm_control/planner.py (require_calibration, ticks_to_joints,
joints_to_ticks) so the robot and the PC planner agree on every conversion.
Only the seven calibrated arm joints get angles; the gripper, wheels and
dispenser stay in raw ticks because no physical calibration exists for them.
"""
import hashlib
import json
import math
from pathlib import Path

JOINTS = tuple(f'rev_motor_{i:02d}' for i in range(1, 8))


class CalibrationError(ValueError):
    pass


class Calibration:
    def __init__(self, data, raw=b''):
        self.data = data
        self.sha256 = hashlib.sha256(raw).hexdigest() if raw else ''
        if data.get('hardware_verified') is not True:
            raise CalibrationError('Calibration is not marked hardware_verified')
        if not data.get('bus') or data['bus'] == 'auto':
            raise CalibrationError('Calibration must name its serial adapter')
        joints = data.get('joints', [])
        if tuple(j.get('name') for j in joints) != JOINTS:
            raise CalibrationError('Ordered rev_motor_01..07 mapping required')
        ids = [j['motor_id'] for j in joints]
        if ids != list(range(1, 8)):
            raise CalibrationError('Arm joints must map to motor IDs 1..7')
        for key in ('servo_speed_ticks_s', 'servo_acceleration', 'tracking_error_rad'):
            if key not in data:
                raise CalibrationError(f'Missing {key}')
        if (type(data['servo_speed_ticks_s']) is not int or not 1 <= data['servo_speed_ticks_s'] <= 3400
                or type(data['servo_acceleration']) is not int or not 1 <= data['servo_acceleration'] <= 254):
            raise CalibrationError('Invalid servo speed/acceleration')
        for j in joints:
            scale = j.get('ticks_per_rad', 4096 / (2 * math.pi))
            if not (isinstance(scale, (int, float)) and math.isfinite(scale) and 100 <= scale <= 5000):
                raise CalibrationError(f"{j['name']}: invalid ticks_per_rad")
            if (j.get('sign') not in (-1, 1) or type(j.get('sign')) is not int
                    or any(type(j.get(k)) is not int for k in ('zero_ticks', 'min_ticks', 'max_ticks'))
                    or not 0 <= j['zero_ticks'] <= 4095
                    or not 0 <= j['min_ticks'] < j['max_ticks'] <= 4095):
                raise CalibrationError(f"{j['name']}: measured zero, sign and limits required")
            if data.get('max_velocity_rad_s', 0) * scale > data['servo_speed_ticks_s']:
                raise CalibrationError(f"{j['name']}: planned velocity exceeds servo speed")
        self.joints = joints

    @classmethod
    def load(cls, path):
        raw = Path(path).read_bytes()
        return cls(json.loads(raw), raw)

    @staticmethod
    def scale(joint):
        return joint.get('ticks_per_rad', 4096 / (2 * math.pi))

    def joint(self, name):
        return self.joints[JOINTS.index(name)]

    def to_radians(self, name, ticks):
        j = self.joint(name)
        return (ticks - j['zero_ticks']) * j['sign'] / self.scale(j)

    def to_ticks(self, name, radians):
        """Convert and REFUSE (never clip) targets outside the calibrated range."""
        if not math.isfinite(radians):
            raise CalibrationError(f'{name}: non-finite target')
        j = self.joint(name)
        ticks = round(j['zero_ticks'] + j['sign'] * radians * self.scale(j))
        if not j['min_ticks'] <= ticks <= j['max_ticks']:
            raise CalibrationError(
                f"{name}: target {ticks} ticks outside {j['min_ticks']}..{j['max_ticks']}")
        return ticks

    def margin_violation(self, name, ticks):
        j = self.joint(name)
        if ticks < j['min_ticks']:
            return f"{name} at {ticks} ticks is below the operating margin {j['min_ticks']}"
        if ticks > j['max_ticks']:
            return f"{name} at {ticks} ticks is above the operating margin {j['max_ticks']}"
        return None
