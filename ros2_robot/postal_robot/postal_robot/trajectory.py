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
                 arrival_tolerance=.01, arrival_timeout=5., max_step_gap=.25,
                 velocity_margin=1.25):
        if tuple(names) != JOINTS:
            raise TrajectoryError('Trajectory must list rev_motor_01..07 in order')
        if len(times) < 2 or len(times) != len(points) or len(times) > 100000:
            raise TrajectoryError('Trajectory needs 2..100000 timed points')
        if times[0] != 0 or any(not math.isfinite(t) for t in times) \
                or any(b <= a for a, b in zip(times, times[1:])):
            raise TrajectoryError('time_from_start must start at 0 and strictly increase')
        c = calibration.data
        self.tracking_error = c['tracking_error_rad']
        vmax = c.get('max_velocity_rad_s', float('inf')) * velocity_margin
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
                raise TrajectoryError(f'Point {i}: velocity above the calibrated limit')
        self.calibration, self.times, self.points, self.goals = calibration, list(times), points, goals
        self.start_tolerance, self.arrival_tolerance = start_tolerance, arrival_tolerance
        self.arrival_timeout, self.max_step_gap = arrival_timeout, max_step_gap
        self.state, self.message = RUNNING, ''
        self.start_time = self.last_step = None
        self.commanded = -1
        self.max_tracking = 0.

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
        reference = self.points[max(self.commanded, 0)]
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
        final = self.points[-1]
        error = max(abs(a - b) for a, b in zip(measured, final))
        if error <= self.arrival_tolerance:
            self.state, self.message = SUCCEEDED, f'Arrived (max joint error {error:.4f} rad)'
        elif now - self.start_time > self.duration + self.arrival_timeout:
            self._abort(f'Final position not reached: error {error:.4f} rad after timeout')
        return None

    def _abort(self, message):
        self.state, self.message = ABORTED, message
        return None

    def progress(self, now):
        if self.start_time is None:
            return 0.
        return min(1., (now - self.start_time) / self.duration)
