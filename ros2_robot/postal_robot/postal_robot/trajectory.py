"""Arm trajectory execution logic (pure; the core node does the bus I/O).

Keeps the intent of arm_control/station.py:execute: every setpoint is
converted and range-checked before any write (refused, never clipped); the
measured start must match the plan; each step requires fresh feedback and a
bounded tracking error; a late control loop stops the run; success requires
measured arrival, not merely having sent the last setpoint. Only existing plan
samples are commanded (no synthesized shortcuts).
"""
import math

from .calibration import JOINTS, CalibrationError

RUNNING, SUCCEEDED, ABORTED = 'running', 'succeeded', 'aborted'


class TrajectoryError(ValueError):
    pass


def velocity_scales(calibration_data):
    """Seven per-joint velocity multipliers (default all 1) from calibration joint_limits."""
    values = (calibration_data.get('joint_limits') or {}).get('velocity_scale', [1.0] * 7)
    if (not isinstance(values, list) or len(values) != 7
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 < v <= 4 for v in values)):
        raise TrajectoryError('joint_limits.velocity_scale: seven values within (0, 4] required')
    return [float(v) for v in values]


class TrajectoryRun:
    def __init__(self, calibration, names, times, points, *, start_tolerance=.02,
                 arrival_tolerance=.01, arrival_timeout=20., max_step_gap=.25,
                 velocity_margin=1.25, max_speed_factor=1., compensation=None, settle_time=.4, settle_rad=.003,
                 max_compensation_steps=20, tracking_margin=.1, load_limit_raw=None,
                 load_slow=.65, load_stop=.92, load_min_rate=.3, load_stop_hold_s=.5):
        if tuple(names) != JOINTS:
            raise TrajectoryError('Trajectory must list rev_motor_01..07 in order')
        if len(times) < 2 or len(times) != len(points) or len(times) > 100000:
            raise TrajectoryError('Trajectory needs 2..100000 timed points')
        if times[0] != 0 or any(not math.isfinite(t) for t in times) \
                or any(b <= a for a, b in zip(times, times[1:])):
            raise TrajectoryError('time_from_start must start at 0 and strictly increase')
        c = calibration.data
        self.tracking_error = c['tracking_error_rad']
        if not (math.isfinite(max_speed_factor) and 0 < max_speed_factor <= 3):
            raise TrajectoryError('max_speed_factor must be within (0, 3]')
        # Optional per-joint velocity limits: calibration joint_limits.velocity_scale (7 multipliers).
        self.velocity_scale = velocity_scales(c)
        self.velocity_margin = velocity_margin
        vmax = [c.get('max_velocity_rad_s', float('inf')) * k * max_speed_factor * velocity_margin
                for k in self.velocity_scale]
        goals = []
        for index, point in enumerate(points):
            if len(point) != 7 or any(not math.isfinite(q) for q in point):
                raise TrajectoryError(f'Point {index}: seven finite positions required')
            try:
                goals.append([calibration.to_ticks(n, q) for n, q in zip(JOINTS, point)])
            except CalibrationError as exc:
                raise TrajectoryError(f'Point {index}: {exc}') from exc
        for i in range(1, len(points)):
            dt = times[i] - times[i - 1]
            if any(abs(b - a) > v * dt + 1e-9 for a, b, v in zip(points[i - 1], points[i], vmax)):
                raise TrajectoryError(f'Point {i}: velocity above {max_speed_factor:g}x the calibrated limit')
        self.calibration, self.times, self.points, self.goals = calibration, list(times), points, goals
        # Speed of this plan relative to the calibrated velocity. Servo lag grows
        # with speed, so the tracking bound grows in proportion (1x -> base bound,
        # 1.5x -> 1.5x, 2x -> 2x). The static sag measured at the start is
        # removed from the error (see begin()), so the bound limits lag only.
        peak = max((max(abs(b - a) / k for a, b, k in zip(points[i - 1], points[i], self.velocity_scale))
                    / (times[i] - times[i - 1]) for i in range(1, len(points))), default=0.)
        self.speed_ratio = max(1., peak / c['max_velocity_rad_s']) if c.get('max_velocity_rad_s') else 1.
        # Proportional part + fixed margin so that load/gravity variations do not
        # stop a valid motion; a real stall (error far beyond) still aborts.
        self.tracking_error = self.tracking_error * self.speed_ratio + tracking_margin
        self.bias = [0.] * 7
        self.start_tolerance, self.arrival_tolerance = start_tolerance, arrival_tolerance
        # Immediate success only within the exact tolerance; a looser one (poses next to
        # an obstacle) is accepted only once the arm has settled and compensation is exhausted.
        self.exact = min(arrival_tolerance, .01)
        self.arrival_timeout, self.max_step_gap = arrival_timeout, max_step_gap
        compensation = list(compensation or [0.] * 7)
        if len(compensation) != 7 or any(not (math.isfinite(c) and 0 <= c <= .1) for c in compensation):
            raise TrajectoryError('compensation limits: seven values within 0..0.1 rad')
        self.compensation, self.offset = compensation, [0.] * 7
        self.settle_time, self.settle_rad, self.max_steps = settle_time, settle_rad, max_compensation_steps
        self.steps, self.settled_ref, self.last_motion = 0, None, None
        self.state, self.message = RUNNING, ''
        self.start_time = self.last_step = None
        # Load guard: the servo cuts out at its own overload threshold (a raw load of about 200 on
        # J2). Before that, the plan is played back more slowly (less acceleration torque), and
        # the arm is held while it is still under the threshold if the load stays too high.
        self.load_limit, self.load_slow, self.load_stop = load_limit_raw, load_slow, load_stop
        self.load_min_rate, self.load_stop_hold = load_min_rate, load_stop_hold_s
        self.rate, self.virtual, self.lost, self.guard_since, self.min_rate_seen = 1., 0., 0., None, 1.
        self.commanded = -1
        self.max_tracking = 0.

    def peak_ticks_per_s(self):
        """Peak commanded speed of each joint, in encoder ticks per second."""
        peaks = [0.] * 7
        for i in range(1, len(self.goals)):
            dt = self.times[i] - self.times[i - 1]
            for j in range(7):
                peaks[j] = max(peaks[j], abs(self.goals[i][j] - self.goals[i - 1][j]) / dt)
        return peaks

    @property
    def duration(self):
        return self.times[-1]

    def begin(self, measured, now):
        """measured: seven radians from fresh feedback. Raises when the arm moved."""
        error = max(abs(a - b) for a, b in zip(measured, self.points[0]))
        if error > self.start_tolerance:
            raise TrajectoryError(
                f'Measured start differs from plan by {error:.3f} rad (> {self.start_tolerance}); replan')
        # Static (gravity) sag at the held start pose: measured - planned, per joint,
        # bounded by the start tolerance already enforced above.
        self.bias = [m - p for m, p in zip(measured, self.points[0])]
        self.start_time = self.last_step = now
        return error

    def _load_rate(self, loads, dt, now):
        """Playback rate 0.3..1 from the arm servos' raw loads; aborts when the load stays too high."""
        if not self.load_limit or not loads:
            target = 1.
        else:
            ratios = [abs(v) / self.load_limit for v in loads[:7]]
            peak = max(ratios)
            joint = ratios.index(peak)
            if peak >= self.load_stop:
                self.guard_since = self.guard_since if self.guard_since is not None else now
                if now - self.guard_since >= self.load_stop_hold:
                    self._abort(f'Load guard: J{joint + 1} at {peak * 100:.0f}% of the {self.load_limit} overload '
                                'threshold; arm held before the servo protection trips')
            else:
                self.guard_since = None
            span = max(1e-6, self.load_stop - self.load_slow)
            target = 1. if peak <= self.load_slow else max(
                self.load_min_rate, 1. - (peak - self.load_slow) / span * (1. - self.load_min_rate))
        # fast to slow down, slow to speed up again
        step = min(max(target - self.rate, -2. * dt), .5 * dt)
        self.rate = min(1., max(self.load_min_rate, self.rate + step))
        self.min_rate_seen = min(self.min_rate_seen, self.rate)
        return self.rate

    def step(self, now, measured, loads=None):
        """Return goal ticks to send (or None). measured is None when feedback is not fresh.
        loads: seven raw servo loads (or None) for the load guard."""
        if self.state != RUNNING:
            return None
        if measured is None:
            return self._abort('Arm feedback missing or stale during motion')
        if now - self.last_step > self.max_step_gap:
            return self._abort(f'Control loop late by {now - self.last_step:.3f} s; motion stopped')
        dt = now - self.last_step
        self.last_step = now
        rate = self._load_rate(loads, dt, now)
        if self.state != RUNNING:
            return None
        self.virtual += dt * rate
        self.lost += dt * (1. - rate)
        reference = self.points[max(self.commanded, 0)]  # planned pose, not the offset goal
        errors = [abs(a - b - o) for a, b, o in zip(measured, reference, self.bias)]
        error = max(errors)
        self.max_tracking = max(self.max_tracking, error)
        if error > self.tracking_error:
            return self._abort(f'Tracking error {error:.3f} rad on J{errors.index(error) + 1} exceeds {self.tracking_error:.3f} rad')
        elapsed = self.virtual  # plan time: it runs slower than the clock while the load guard slows it
        index = self.commanded
        while index + 1 < len(self.times) and self.times[index + 1] <= elapsed:
            index += 1
        if index == self.commanded and self.commanded >= 0:
            if self.commanded == len(self.times) - 1:
                return self._arrival(now, measured)
            return None
        index = max(index, 0)
        self.commanded = index
        return self.goals[index]

    def _arrival(self, now, measured):
        """Measured arrival; optional bounded compensation of the servo static error.

        Once the arm has settled, the final goal is shifted by the remaining error
        (gravity sag of a proportional servo), never beyond the per-joint bound
        validated collision-free by the planner. The physical target stays the
        planned final pose; only the commanded goal is offset."""
        final = self.points[-1]
        errors = [b - a for a, b in zip(measured, final)]
        error = max(abs(e) for e in errors)
        used = max(abs(o) for o in self.offset)
        if error <= self.exact:
            return self._arrived(error, used)
        if now - self.start_time - self.lost > self.duration + self.arrival_timeout:
            if error <= self.arrival_tolerance:
                return self._arrived(error, used)
            return self._abort(f'Final position not reached: error {error:.4f} rad after timeout ({self._worst(errors)})'
                               + (f' (compensation {used:.3f} rad)' if self.steps else ''))
        if self.settled_ref is None or max(abs(a - b) for a, b in zip(measured, self.settled_ref)) > self.settle_rad:
            self.settled_ref, self.last_motion = list(measured), now
            return None
        if now - self.last_motion < self.settle_time:
            return None
        if not any(self.compensation) or self.steps >= self.max_steps:
            if error <= self.arrival_tolerance:
                return self._arrived(error, used)
            if not any(self.compensation):
                return None
            return self._abort(f'Final position not reached after {self.steps} compensation steps: error '
                               f'{error:.4f} rad ({self._worst(errors)})')
        offset = [max(-c, min(c, o + e)) if c > 0 and abs(e) > self.exact / 2 else o
                  for o, e, c in zip(self.offset, errors, self.compensation)]
        if offset == self.offset:
            if error <= self.arrival_tolerance:
                return self._arrived(error, used)
            return self._abort(f'Static error {error:.4f} rad: {self._worst(errors)}; the compensation offset is at its '
                               'bound (or the joint does not respond to it: friction, load or an obstacle)')
        try:
            goal = [self.calibration.to_ticks(n, q + o) for n, q, o in zip(JOINTS, final, offset)]
        except CalibrationError as exc:
            return self._abort(f'Compensated goal refused: {exc}')
        self.offset, self.steps, self.last_motion = offset, self.steps + 1, now
        return goal

    def _worst(self, errors):
        """Which joint is off, by how much, and the compensation offset already applied to it."""
        j = max(range(7), key=lambda k: abs(errors[k]))
        return (f'worst joint J{j + 1}: {errors[j]:+.4f} rad, offset {self.offset[j]:+.3f} of '
                f'+/-{self.compensation[j]:.3f}')

    def _arrived(self, error, used):
        self.state = SUCCEEDED
        self.message = (f'Arrived, slowed to {self.min_rate_seen * 100:.0f}% by the load guard; '
                        if self.min_rate_seen < .99 else 'Arrived ') + f'(max joint error {error:.4f} rad' + (
            f', static-error compensation {used:.3f} rad in {self.steps} step(s))' if self.steps else ')')
        return None

    def _abort(self, message):
        self.state, self.message = ABORTED, message
        return None

    def progress(self, now):
        if self.start_time is None:
            return 0.
        return min(1., self.virtual / self.duration)
