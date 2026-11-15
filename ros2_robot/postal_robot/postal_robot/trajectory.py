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


class TrajectoryRun:
    def __init__(self, calibration, names, times, points, *, start_tolerance=.02,
                 arrival_tolerance=.01, arrival_timeout=8., max_step_gap=.25,
                 velocity_margin=1.25, max_speed_factor=1., compensation=None, settle_time=.4, settle_rad=.003,
                 max_compensation_steps=5):
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
        vmax = c.get('max_velocity_rad_s', float('inf')) * max_speed_factor * velocity_margin
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
            if max(abs(b - a) for a, b in zip(points[i - 1], points[i])) > vmax * dt + 1e-9:
                raise TrajectoryError(f'Point {i}: velocity above {max_speed_factor:g}x the calibrated limit')
        self.calibration, self.times, self.points, self.goals = calibration, list(times), points, goals
        self.start_tolerance, self.arrival_tolerance = start_tolerance, arrival_tolerance
        self.arrival_timeout, self.max_step_gap = arrival_timeout, max_step_gap
        compensation = list(compensation or [0.] * 7)
        if len(compensation) != 7 or any(not (math.isfinite(c) and 0 <= c <= .1) for c in compensation):
            raise TrajectoryError('compensation limits: seven values within 0..0.1 rad')
        self.compensation, self.offset = compensation, [0.] * 7
        self.settle_time, self.settle_rad, self.max_steps = settle_time, settle_rad, max_compensation_steps
        self.steps, self.settled_ref, self.last_motion = 0, None, None
        self.state, self.message = RUNNING, ''
        self.start_time = self.last_step = None
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
        self.start_time = self.last_step = now
        return error

    def step(self, now, measured):
        """Return goal ticks to send (or None). measured is None when feedback is not fresh."""
        if self.state != RUNNING:
            return None
        if measured is None:
            return self._abort('Arm feedback missing or stale during motion')
        if now - self.last_step > self.max_step_gap:
            return self._abort(f'Control loop late by {now - self.last_step:.3f} s; motion stopped')
        self.last_step = now
        reference = self.points[max(self.commanded, 0)]  # planned pose, not the offset goal
        error = max(abs(a - b) for a, b in zip(measured, reference))
        self.max_tracking = max(self.max_tracking, error)
        if error > self.tracking_error:
            return self._abort(f'Tracking error {error:.3f} rad exceeds {self.tracking_error} rad')
        elapsed = now - self.start_time
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
        if error <= self.arrival_tolerance:
            self.state = SUCCEEDED
            self.message = f'Arrived (max joint error {error:.4f} rad' + (
                f', static-error compensation {used:.3f} rad in {self.steps} step(s))' if self.steps else ')')
            return None
        if now - self.start_time > self.duration + self.arrival_timeout:
            return self._abort(f'Final position not reached: error {error:.4f} rad after timeout'
                               + (f' (compensation {used:.3f} rad)' if self.steps else ''))
        if self.settled_ref is None or max(abs(a - b) for a, b in zip(measured, self.settled_ref)) > self.settle_rad:
            self.settled_ref, self.last_motion = list(measured), now
            return None
        if now - self.last_motion < self.settle_time or not any(self.compensation):
            return None
        if self.steps >= self.max_steps:
            return self._abort(f'Final position not reached after {self.steps} compensation steps: error {error:.4f} rad')
        offset = [max(-c, min(c, o + e)) if c > 0 and abs(e) > self.arrival_tolerance / 2 else o
                  for o, e, c in zip(self.offset, errors, self.compensation)]
        if offset == self.offset:
            return self._abort(f'Static error {error:.4f} rad exceeds the validated compensation bound')
        try:
            goal = [self.calibration.to_ticks(n, q + o) for n, q, o in zip(JOINTS, final, offset)]
        except CalibrationError as exc:
            return self._abort(f'Compensated goal refused: {exc}')
        self.offset, self.steps, self.last_motion = offset, self.steps + 1, now
        return goal

    def _abort(self, message):
        self.state, self.message = ABORTED, message
        return None

    def progress(self, now):
        if self.start_time is None:
            return 0.
        return min(1., (now - self.start_time) / self.duration)
