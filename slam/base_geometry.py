"""Provisional Kiwi geometry for manual ROS drive/odometry; calibrate before use."""

import math

WHEEL_RADIUS = 0.05  # metres; measure before using odometry for navigation.
BASE_RADIUS = 0.125  # metres from base centre to each wheel.
WHEEL_ANGLES_DEG = (150, -90, 30)  # motor IDs 7, 8, 9.


class WheelOdometry:
    """Integrate 12-bit wheel encoder deltas; reject ambiguous jumps and long gaps."""

    def __init__(self, radius=WHEEL_RADIUS, base_radius=BASE_RADIUS, max_ticks_per_s=2500):
        self.radius, self.base_radius, self.max_ticks_per_s = radius, base_radius, max_ticks_per_s
        self.previous = None
        self.x = self.y = self.yaw = 0.0

    def update(self, positions, now):
        if len(positions) != 3 or any(not 0 <= p < 4096 for p in positions):
            raise ValueError("Expected three 12-bit encoder positions")
        old, self.previous = self.previous, (tuple(positions), now)
        if old is None:
            return None
        previous, then = old
        dt = now - then
        if not 0 < dt < 0.3:
            return None
        ticks = [((p - q + 2048) % 4096) - 2048 for p, q in zip(positions, previous)]
        if any(abs(v) > self.max_ticks_per_s * dt + 10 for v in ticks):
            return None
        travel = [t * 2 * math.pi * self.radius / 4096 for t in ticks]
        angles = [math.radians(a) for a in WHEEL_ANGLES_DEG]
        dx = 2 / 3 * sum(math.cos(a) * v for a, v in zip(angles, travel))
        dy = 2 / 3 * sum(math.sin(a) * v for a, v in zip(angles, travel))
        dyaw = sum(travel) / (3 * self.base_radius)
        heading = self.yaw + dyaw / 2
        self.x += math.cos(heading) * dx - math.sin(heading) * dy
        self.y += math.sin(heading) * dx + math.cos(heading) * dy
        self.yaw += dyaw
        return dx / dt, dy / dt, dyaw / dt
