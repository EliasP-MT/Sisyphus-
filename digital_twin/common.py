"""Configuration, SE(2), and the official LeKiwi wheel convention in SI units."""
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
URDF = ROOT/'arm_control/.sdk/hardware/elrobot/simulation/elrobot_follower.urdf'


def config(path=None):
    result = json.loads(Path(path or HERE/'config.json').read_text())
    if result['timestep_s'] <= 0 or result['control_period_s'] < result['timestep_s']:
        raise ValueError('Pas de simulation/commande invalides')
    if result['wheel_radius_m'] <= 0 or result['base_radius_m'] <= 0:
        raise ValueError('Rayons de roues/châssis positifs requis')
    if len(result['wheel_mount_deg']) != 3 or result['roller_count'] < 8:
        raise ValueError('Trois roues et au moins huit rouleaux par roue requis')
    if result['lidar']['rays'] < 30 or result['lidar']['max_range_m'] <= 0:
        raise ValueError('LiDAR virtuel invalide')
    return result


def wrap(angle):
    return (angle+np.pi) % (2*np.pi)-np.pi


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def compose(pose, increment):
    return np.r_[np.asarray(pose[:2])+rotation(pose[2]) @ increment[:2], wrap(pose[2]+increment[2])]


def relative(a, b):
    return np.r_[rotation(a[2]).T @ (np.asarray(b[:2])-a[:2]), wrap(b[2]-a[2])]


class KiwiKinematics:
    """LeRobot LeKiwi left/back/right mounting and proportional saturation."""
    def __init__(self, c):
        angles = np.deg2rad(np.asarray(c['wheel_mount_deg'])-90)
        self.matrix = np.c_[np.cos(angles), np.sin(angles), np.full(3, c['base_radius_m'])]/c['wheel_radius_m']
        self.inverse = np.linalg.inv(self.matrix)
        self.limit = c['wheel_speed_rad_s']

    def wheels(self, body_twist):
        twist = np.asarray(body_twist, dtype=float)
        if twist.shape != (3,) or not np.all(np.isfinite(twist)):
            raise ValueError('Twist fini [vx m/s, vy m/s, omega rad/s] requis')
        values = self.matrix @ twist
        return values/max(1, np.max(np.abs(values))/self.limit)

    def body(self, wheel_rates):
        return self.inverse @ np.asarray(wheel_rates)
