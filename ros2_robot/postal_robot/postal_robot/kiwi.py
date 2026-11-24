"""Three-wheel Kiwi base kinematics, using the LeKiwi convention.

Wheel order is (left, back, right) with mounting angles (240, 0, 120) degrees.
Wheel radius 0.05 m and centre-wheel distance 0.125 m were confirmed by the
user. Servo speed unit: 4096 encoder steps per wheel revolution per second.
"""
import math

STEPS_PER_REV = 4096


class Kiwi:
    def __init__(self, wheel_radius_m=.05, base_radius_m=.125, mount_deg=(240., 0., 120.),
                 signs=(1, 1, 1), max_wheel_raw=1500):
        if wheel_radius_m <= 0 or base_radius_m <= 0 or len(mount_deg) != 3 or len(signs) != 3:
            raise ValueError('Invalid Kiwi geometry')
        if any(s not in (-1, 1) for s in signs) or not 0 < max_wheel_raw <= 3000:
            raise ValueError('Invalid wheel signs or speed limit')
        self.r, self.R, self.signs, self.max_raw = wheel_radius_m, base_radius_m, tuple(signs), max_wheel_raw
        a = [math.radians(d - 90) for d in mount_deg]
        self.rows = [(math.cos(x), math.sin(x), base_radius_m) for x in a]
        self.inverse = _invert3([[c / self.r, s / self.r, R / self.r] for c, s, R in self.rows])

    def wheel_rates(self, vx, vy, wz):
        """Body twist (m/s, m/s, rad/s) -> wheel angular rates (rad/s)."""
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            raise ValueError('Non-finite twist')
        return [(c * vx + s * vy + R * wz) / self.r for c, s, R in self.rows]

    def raw_speeds(self, vx, vy, wz):
        """Twist -> signed servo speeds, saturated proportionally (direction kept)."""
        raw = [rate * STEPS_PER_REV / (2 * math.pi) * sign
               for rate, sign in zip(self.wheel_rates(vx, vy, wz), self.signs)]
        peak = max(abs(v) for v in raw)
        scale = min(1.0, self.max_raw / peak) if peak else 1.0
        return [int(round(v * scale)) for v in raw]

    def twist_from_raw(self, raw_speeds):
        rates = [v * sign * 2 * math.pi / STEPS_PER_REV for v, sign in zip(raw_speeds, self.signs)]
        return [sum(self.inverse[i][k] * rates[k] for k in range(3)) for i in range(3)]


def _invert3(m):
    (a, b, c), (d, e, f), (g, h, i) = m
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise ValueError('Singular Kiwi geometry')
    return [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
            [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
            [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]]
