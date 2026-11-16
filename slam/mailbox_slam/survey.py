"""Fixed-arm rotate/discover/approach/read initialization state machine.

Returns short-lived base velocity PROPOSALS. It never writes a motor command.
An application must supply timestamped encoders/images and execute proposals in
an independently cleared demo area, through its existing watchdog controller.
"""

import math
from dataclasses import asdict, dataclass

import numpy as np

from .geometry import planar, se2, wrap


@dataclass(frozen=True)
class SurveySettings:
    scan_step_rad: float = math.radians(30)  # camera HFOV is ~99 deg: wide overlap
    scan_speed_rad_s: float = 0.5
    settle_s: float = 0.2
    stationary_speed_m_s: float = 0.01
    stationary_yaw_rad_s: float = 0.03
    yaw_tolerance_rad: float = 0.025
    position_tolerance_m: float = 0.015
    approach_speed_m_s: float = 0.10
    approach_standoff_m: float = 0.30  # 0.22 m lost the label on the robot (fixed focus, view limits)
    image_interval_s: float = 1.5  # approach drives between pictures instead of stop-and-go
    max_visual_age_s: float = 3.0
    max_standoff_m: float = 0.65  # the arm stays at scan: low boxes need a larger standoff to stay in view
    max_phase_s: float = 120.0
    command_valid_s: float = 0.1
    min_sightings: int = 1
    max_read_attempts: int = 4
    max_missing_target_frames: int = 3
    discovery_only: bool = False
    expected_mailboxes: int | None = None
    # Frontal reading (live robot): after the sweep the base translates in front of each box,
    # heading along the face normal (Kiwi base: no turning towards the label), reads from the
    # first standoff and only comes closer, standoff by standoff, when a reading fails.
    frontal_approach: bool = False
    read_standoffs_m: tuple = ()
    attempts_per_standoff: int = 2
    near_goal_m: float = 0.25  # re-acquire the label geometrically only this close to the goal
    max_wheel_fraction: float = 0.6  # commanded wheel speed vs the wheel's maximum
    read_during_sweep: bool = False  # read names on the first pass, at the starting distance
    # Second pass (boxes still unread after the first one) from this much closer than the first
    # pass distance, which is measured at the end of the sweep. None = read_standoffs_m.
    second_pass_closer_m: float | None = None
    # Wall scan: the base starts facing the boxes and slides sideways (cap unchanged) instead of
    # turning 360 degrees. Direction is in the start frame (x forward, y left): (0, -1) = right.
    scan_mode: str = "rotate"  # or "translate"
    scan_translate_m: float = 1.5
    scan_step_m: float = 0.25
    scan_translate_speed_m_s: float = 0.10
    scan_direction: tuple = (0.0, -1.0)

    def __post_init__(self):
        if type(self.discovery_only) is not bool:
            raise ValueError("discovery_only must be boolean")
        if self.expected_mailboxes is not None and (
            type(self.expected_mailboxes) is not int or self.expected_mailboxes <= 0
        ):
            raise ValueError("expected_mailboxes must be a positive integer")
        for key, value in asdict(self).items():
            if key in (
                "discovery_only",
                "expected_mailboxes",
                "frontal_approach",
                "read_standoffs_m",
                "scan_mode",
                "scan_direction",
                "read_during_sweep",
                "second_pass_closer_m",
            ):
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be positive and finite")
        if self.scan_mode not in ("rotate", "translate"):
            raise ValueError("scan_mode must be rotate or translate")
        if self.scan_mode == "translate" and self.scan_step_m > 0.5:
            raise ValueError("Scan steps must be <=0.5 m; choose overlapping camera views")
        if self.scan_step_rad > math.pi / 2:
            raise ValueError("Scan steps must be <=90 degrees; choose overlapping camera views")
        for key in ("min_sightings", "max_read_attempts", "max_missing_target_frames"):
            if int(getattr(self, key)) != getattr(self, key):
                raise ValueError(f"{key} must be an integer")


class MailboxSurvey:
    """One session: anchor origin, scan 360 degrees, visit unnamed labels, read DB names.

    Public loop: add encoder samples to slam, call step(t) frequently, and call
    observe(frame, capture_t) only when capture is requested. This class assumes
    every mailbox of interest is visible somewhere during the initial sweep.
    It cannot establish that hidden/out-of-view mailboxes do not exist.
    """

    def __init__(self, slam, settings=None):
        self.slam = slam
        self.settings = settings or SurveySettings()
        self.phase = "idle"
        self.reason = None
        self.started = None
        self.phase_started = None
        self.still_since = None
        self.yaw_progress = 0.0
        self.previous_odom_yaw = None
        self.next_scan_angle = 0.0
        self.travel_progress = 0.0  # metres travelled along scan_direction (translate mode)
        self.next_scan_travel = 0.0
        self.previous_odom_xy = None
        self.start_yaw = 0.0
        self.target_id = None
        self.last_image_at = None
        self.last_target_seen = None
        self.name_attempts = 0
        self.missing_target_frames = 0
        self.failed = {}
        self.scan_complete = False
        self.last_step_at = None
        self.last_capture_mode = None
        self.target_started = None
        self.stage = 0
        self.reading_distance = None  # camera-to-wall distance of the first pass (m)
        self.second_pass_standoff = None

    def start(self, timestamp):
        if self.phase != "idle":
            raise ValueError("Survey already started")
        self.slam.start(timestamp)
        self.started = float(timestamp)
        first = self.slam.wheels.buffer.at(timestamp)
        self.previous_odom_yaw = float(first[2])
        self.previous_odom_xy, self.start_yaw = np.array(first[:2]), float(first[2])
        self._enter("scan_settle", timestamp)
        return self.status()

    def _enter(self, phase, timestamp):
        self.phase, self.phase_started = phase, float(timestamp)
        self.still_since = None

    def _stationary(self):
        samples = self.slam.wheels.buffer.samples
        if len(samples) < 2:
            return False
        (t0, p0), (t1, p1) = samples[-2], samples[-1]
        dt = t1 - t0
        return (
            np.linalg.norm(p1[:2] - p0[:2]) / dt <= self.settings.stationary_speed_m_s
            and abs(wrap(p1[2] - p0[2])) / dt <= self.settings.stationary_yaw_rad_s
        )

    def _settled(self, timestamp):
        if not self._stationary():
            self.still_since = None
            return False
        if self.still_since is None:
            self.still_since = timestamp
        return timestamp - self.still_since >= self.settings.settle_s

    def _limit_wheel_speed(self, velocity):
        """Scale a body velocity so no wheel exceeds a fraction of its maximum speed: close to the
        limit the encoder speed check trips on timing jitter, and the servo cannot follow anyway."""
        wheels = getattr(self.slam, "wheels", None)
        v = np.asarray(velocity, float)
        if wheels is None or not v.any():
            return v
        rates = np.array(
            [abs(k @ v) / s["radius_m"] for k, s in zip(wheels.kinematics, wheels.specs)]
        )
        limits = np.array([s["max_rad_s"] for s in wheels.specs]) * self.settings.max_wheel_fraction
        scale = float(np.min(limits / np.maximum(rates, 1e-9)))
        return v * min(1.0, scale)

    def _proposal(self, timestamp, velocity=(0.0, 0.0, 0.0), capture=None, goal=None):
        velocity = self._limit_wheel_speed(velocity)
        return dict(
            phase=self.phase,
            velocity_base=list(map(float, velocity)),
            expires_at=float(timestamp) + self.settings.command_valid_s,
            capture=capture,
            target_id=self.target_id,
            goal=goal,
            reason=self.reason,
            scan_progress_rad=self.yaw_progress,
            motion_is_proposal=True,
        )

    def step(self, timestamp):
        """Return [vx,vy,wz] m/s, m/s, rad/s; all fault/await-image states stop.

        Use actual encoder feedback at each step. Repeated calls are necessary;
        executing an old proposal beyond expires_at is never valid.
        """
        timestamp = float(timestamp)
        try:
            if not np.isfinite(timestamp) or (
                self.last_step_at is not None and timestamp < self.last_step_at
            ):
                raise ValueError("Control timestamps must be finite and nondecreasing")
            self.last_step_at = timestamp
            if self.phase == "idle":
                raise ValueError("Call start(timestamp) first")
            if self.phase in ("complete", "incomplete", "fault"):
                return self._proposal(timestamp)
            pose = se2(self.slam.pose_at(timestamp))  # Includes freshness/fixed-arm checks.
            odom = self.slam.wheels.buffer.at(timestamp)
            delta_yaw = float(wrap(odom[2] - self.previous_odom_yaw))
            self.previous_odom_yaw = float(odom[2])
            if self.phase.startswith("scan_"):
                if self.settings.scan_mode == "rotate":
                    self.yaw_progress += delta_yaw  # Net encoder rotation, never sum abs(delta).
                else:
                    direction = np.asarray(self.settings.scan_direction, float)
                    direction = direction / np.linalg.norm(direction)
                    self.travel_progress += float((np.asarray(odom[:2]) - self.previous_odom_xy) @ direction)
            self.previous_odom_xy = np.asarray(odom[:2], float).copy()
            if timestamp - self.phase_started > self.settings.max_phase_s:
                raise ValueError("Survey phase exceeded its time limit")
            if self.phase == "scan_translate":
                remaining = self.next_scan_travel - self.travel_progress
                if remaining > self.settings.position_tolerance_m:
                    speed = min(
                        self.settings.scan_translate_speed_m_s, max(3.0 * remaining, 0.03)
                    )
                    speed = min(speed, remaining / 0.1)
                    direction = np.asarray(self.settings.scan_direction, float)
                    direction = direction / np.linalg.norm(direction)
                    heading_error = float(wrap(odom[2] - self.start_yaw))
                    wz = float(np.clip(-2.0 * heading_error, -0.3, 0.3))  # keep facing the wall
                    body = planar(pose)[:2, :2].T @ (direction * speed)
                    return self._proposal(timestamp, (*body, wz))
                self._enter("scan_settle", timestamp)
            if self.phase == "scan_rotate":
                remaining = self.next_scan_angle - self.yaw_progress
                if remaining > self.settings.yaw_tolerance_rad:
                    # Proportional with a floor (the wheels stall below ~0.12 rad/s): the old
                    # speed == remaining decayed exponentially and crawled into every stop.
                    speed = min(self.settings.scan_speed_rad_s, max(3.0 * remaining, 0.12))
                    return self._proposal(timestamp, (0, 0, min(speed, remaining / 0.1)))
                self._enter("scan_settle", timestamp)
            if self.phase == "scan_settle":
                if self._settled(timestamp):
                    self._enter("scan_capture", timestamp)
                else:
                    return self._proposal(timestamp)
            if self.phase == "scan_capture":
                return self._proposal(timestamp, capture="geometry")
            if self.phase == "approach_capture":
                return self._proposal(timestamp, capture="geometry")
            if self.phase == "name_capture":
                return self._proposal(timestamp, capture="name")
            if self.phase == "approach_settle":
                if self._settled(timestamp):
                    self._enter("approach_capture", timestamp)
                    return self._proposal(timestamp, capture="geometry")
                return self._proposal(timestamp)
            if self.phase == "name_settle":
                if self._settled(timestamp):
                    self._enter("name_capture", timestamp)
                    return self._proposal(timestamp, capture="name")
                return self._proposal(timestamp)
            if self.phase == "approach":
                if timestamp - self.target_started > self.settings.max_phase_s:
                    self.failed[self.target_id] = "Approach exceeded its time limit"
                    self._choose_next(timestamp, pose)
                    return self._proposal(timestamp)
                try:
                    goal = self._approach_goal(self.target_id)
                except ValueError as exc:
                    self.failed[self.target_id] = str(exc)
                    self._choose_next(timestamp, pose)
                    return self._proposal(timestamp)
                if self.settings.frontal_approach:
                    return self._frontal_step(timestamp, pose, goal)
                desired = np.array(goal["base_pose_xy_yaw"])
                error = desired[:2] - pose[:2]
                desired_yaw = desired[2]
                if np.linalg.norm(error) > 0.1:
                    # Keep the fixed camera aimed at the label during a lateral
                    # approach, then finish normal to the face near the goal.
                    label = self.slam.graph.landmarks[self.target_id]["pose"][:2, 3]
                    camera_forward = self.slam._fixed_camera_pose()[:2, 2]
                    desired_yaw = np.arctan2(label[1] - pose[1], label[0] - pose[0]) - np.arctan2(
                        camera_forward[1], camera_forward[0]
                    )
                yaw_error = float(wrap(desired_yaw - pose[2]))
                if abs(yaw_error) > self.settings.yaw_tolerance_rad:
                    # Turn toward the label before requiring that it is visible.
                    wz = np.clip(
                        yaw_error, -self.settings.scan_speed_rad_s, self.settings.scan_speed_rad_s
                    )
                    return self._proposal(timestamp, (0, 0, wz), goal=goal)
                if (
                    self.last_image_at is None
                    or timestamp - self.last_image_at >= self.settings.image_interval_s
                ):
                    self._enter("approach_settle", timestamp)
                    return self._proposal(timestamp, goal=goal)
                if (
                    self.last_target_seen is None
                    or timestamp - self.last_target_seen > self.settings.max_visual_age_s
                ):
                    self._enter("approach_settle", timestamp)
                    return self._proposal(timestamp, goal=goal)
                if np.linalg.norm(error) <= self.settings.position_tolerance_m:
                    self._enter("name_settle", timestamp)
                    return self._proposal(timestamp, goal=goal)
                velocity_map = error * min(
                    1.0, self.settings.approach_speed_m_s / max(np.linalg.norm(error), 1e-9)
                )
                velocity_body = planar(pose)[:2, :2].T @ velocity_map
                return self._proposal(timestamp, (*velocity_body, 0.0), goal=goal)
            return self._proposal(timestamp)
        except (ValueError, KeyError) as exc:
            self.reason = str(exc)
            self._enter("fault", timestamp if np.isfinite(timestamp) else 0.0)
            return self._proposal(timestamp if np.isfinite(timestamp) else 0.0)

    def observe(self, image, timestamp):
        """Process the requested capture; all discovery/approach frames are geometry-only."""
        phase = self._check_capture(timestamp)
        self._defer_sweep_optimization(phase)
        if phase == "name_capture":
            result = self.slam.read_target_name(image, timestamp, self.target_id)
            self._after_geometry(result["geometry"], timestamp, phase, name_result=result)
        elif phase == "scan_capture" and self.settings.read_during_sweep:
            result = self.slam.read_visible_names(image, timestamp)
            self._after_geometry(result, timestamp, phase)
        else:
            result = self.slam.process_image(image, timestamp, allow_new=phase == "scan_capture")
            self._after_geometry(result, timestamp, phase)
        return result

    def observe_results(self, results, timestamp, *, ocr_text=None):
        """Replay/testing adapter for validated locator results and optional recorded OCR."""
        phase = self._check_capture(timestamp)
        self._defer_sweep_optimization(phase)
        report = self.slam.process_locator_results(
            timestamp, results, allow_new=phase == "scan_capture"
        )
        name_result = None
        if phase == "name_capture":
            if self.target_id not in self.slam.last_associations:
                name_result = dict(confirmed=False, reason="target not visible")
            elif (
                np.linalg.norm(self.slam.last_associations[self.target_id]["label_center_m"])
                > self.slam.max_read_distance_m
            ):
                name_result = dict(confirmed=False, reason="target too far")
            elif ocr_text is None:
                name_result = dict(confirmed=False, reason="recorded OCR text missing")
            else:
                name_result = self.slam.assign_name(self.target_id, ocr_text, frame=report["frame"])
        self._after_geometry(report, timestamp, phase, name_result=name_result)
        return dict(geometry=report, name_result=name_result)

    def _standoffs(self):
        if self.second_pass_standoff is not None:
            return (self.second_pass_standoff,)
        return tuple(self.settings.read_standoffs_m) or (self.settings.approach_standoff_m,)

    def _plan_second_pass(self):
        """Second pass standoff = the distance the first pass was made at, minus the margin."""
        closer = self.settings.second_pass_closer_m
        if closer is None:
            return
        distances = []
        for box in self.slam.mailboxes():
            try:
                distances.append(self.slam.wall_distance(box["id"]))
            except (ValueError, KeyError):
                continue
        if distances:
            self.reading_distance = float(np.median(distances))
            self.second_pass_standoff = round(max(0.15, self.reading_distance - closer), 3)

    def _wall_axes(self):
        """(normal towards the robot, tangent along the wall) when the wall is known, else None."""
        if not getattr(self.slam, "parallel_wall", False):
            return None
        forward = np.asarray(self.slam._fixed_camera_pose()[:2, 2], float)
        norm = np.linalg.norm(forward)
        if norm < 0.2:
            return None
        normal = -forward / norm
        return normal, np.array([-normal[1], normal[0]])

    def _visual_wall_offset(self, pose, tangent):
        """Offset (m, along the wall) of the green label from the camera axis, taken from the
        last image if it showed the target: the label must be in front of the camera, centred,
        and the map position of the box is only an estimate. None when there is no fresh view."""
        if (
            self.last_target_seen is None
            or self.last_image_at is None
            or abs(self.last_target_seen - self.last_image_at) > 1e-9
        ):
            return None
        result = self.slam.last_associations.get(self.target_id)
        if result is None or not self.slam.graph.frames:
            return None
        camera_at_image = planar(self.slam.graph.frames[-1]["pose"]) @ self.slam._fixed_camera_pose()
        label = camera_at_image @ np.append(np.asarray(result["label_center_m"], float), 1.0)
        camera_now = planar(pose) @ self.slam._fixed_camera_pose()
        return float((label[:2] - camera_now[:2, 3]) @ tangent)

    def _frontal_step(self, timestamp, pose, goal):
        """Translate to the front pose (heading along the face normal), refine near the goal."""
        cfg = self.settings
        desired = np.array(goal["base_pose_xy_yaw"])
        error = desired[:2] - pose[:2]
        yaw_error = float(wrap(desired[2] - pose[2]))
        axes = self._wall_axes()
        if axes is not None and np.linalg.norm(error) <= cfg.near_goal_m:
            # Close to the goal: keep the distance from the map, but centre on the label itself.
            offset = self._visual_wall_offset(pose, axes[1])
            if offset is not None:
                error = float(error @ axes[0]) * axes[0] + offset * axes[1]
        distance = float(np.linalg.norm(error))
        near = np.linalg.norm(desired[:2] - pose[:2]) <= cfg.near_goal_m
        if near and (
            self.last_image_at is None
            or timestamp - self.last_image_at >= cfg.image_interval_s
            or self.last_target_seen is None
            or timestamp - self.last_target_seen > cfg.max_visual_age_s
        ):
            self._enter("approach_settle", timestamp)
            return self._proposal(timestamp, goal=goal)
        if distance <= cfg.position_tolerance_m and abs(yaw_error) <= cfg.yaw_tolerance_rad:
            self._enter("name_settle", timestamp)
            return self._proposal(timestamp, goal=goal)
        wz = 0.0
        if abs(yaw_error) > cfg.yaw_tolerance_rad / 2:
            speed = min(cfg.scan_speed_rad_s, max(3.0 * abs(yaw_error), 0.12))
            wz = float(np.sign(yaw_error)) * min(speed, abs(yaw_error) / 0.1)
        velocity_body = np.zeros(2)
        if distance > cfg.position_tolerance_m:
            speed = min(cfg.approach_speed_m_s, max(0.03, 1.5 * distance))
            velocity_body = planar(pose)[:2, :2].T @ (error / distance * min(speed, distance / 0.1))
        return self._proposal(timestamp, (*velocity_body, wz), goal=goal)

    def _scan_fraction(self):
        if self.settings.scan_mode == "translate":
            return float(np.clip(self.travel_progress / self.settings.scan_translate_m, 0.0, 1.0))
        return float(np.clip(self.yaw_progress / (2 * math.pi), 0.0, 1.0))

    def _approach_goal(self, target):
        """Smallest standoff (from the nominal one) at which the label stays inside the fixed
        camera's view: boxes much lower or higher than the camera need to be seen from farther."""
        standoffs = self._standoffs()
        standoff, last = standoffs[min(self.stage, len(standoffs) - 1)], None
        while standoff <= self.settings.max_standoff_m + 1e-9:
            try:
                return self.slam.approach_pose(target, standoff)
            except ValueError as exc:
                if "outside the fixed camera view" not in str(exc) and "incompatible" not in str(exc):
                    raise
                last = exc
                standoff = round(standoff + 0.05, 3)
        raise last

    def _defer_sweep_optimization(self, phase):
        """The pose graph is re-optimized at the end of the sweep (and at every approach/name
        capture), not after each of the sweep's pictures: the visual evidence is the same and the
        optimization is the most expensive step on the robot's CPU."""
        sweeping = (
            self.travel_progress < self.settings.scan_translate_m - self.settings.position_tolerance_m
            if self.settings.scan_mode == "translate"
            else self.yaw_progress < 2 * math.pi
        )
        self.slam.defer_optimization = phase == "scan_capture" and sweeping

    def _check_capture(self, timestamp):
        if self.phase not in ("scan_capture", "approach_capture", "name_capture"):
            raise ValueError("No image was requested in the current survey phase")
        if not np.isfinite(timestamp) or timestamp < self.phase_started:
            raise ValueError("Capture predates the request; use a fresh frame")
        self.slam.check_fixed_arm(timestamp)
        if not self._stationary():
            raise ValueError("Capture requires stopped wheels and a settled fixed arm")
        return self.phase

    def _after_geometry(self, report, timestamp, phase, name_result=None):
        self.last_image_at = float(timestamp)
        self.last_capture_mode = "name" if phase == "name_capture" else "geometry"
        if phase == "scan_capture":
            if self.settings.scan_mode == "translate":
                done = self.travel_progress >= self.settings.scan_translate_m - self.settings.position_tolerance_m
            else:
                done = self.yaw_progress >= 2 * math.pi
            if done:
                self.scan_complete = True
                self._plan_second_pass()
                if self.settings.discovery_only or self.settings.expected_mailboxes is not None:
                    count = len(self.slam.mailboxes())
                    expected = self.settings.expected_mailboxes
                    complete = count > 0 and (expected is None or count == expected)
                    self.reason = (
                        None
                        if complete
                        else f"Expected {expected if expected is not None else 'at least one'} distinct mailboxes; detected {count}"
                    )
                    self._enter("complete" if complete else "incomplete", timestamp)
                else:
                    self._choose_next(timestamp, np.array(report["base_pose_xy_yaw"]))
            elif self.settings.scan_mode == "translate":
                self.next_scan_travel = min(
                    self.settings.scan_translate_m, self.next_scan_travel + self.settings.scan_step_m
                )
                self._enter("scan_translate", timestamp)
            else:
                next_angle = self.next_scan_angle + self.settings.scan_step_rad
                # Compensate stopping tolerance so the measured sweep reaches
                # 2*pi rather than terminating ~1.4 degrees short of a full turn.
                self.next_scan_angle = (
                    2 * math.pi + self.settings.yaw_tolerance_rad
                    if next_angle >= 2 * math.pi - 1e-9
                    else next_angle
                )
                self._enter("scan_rotate", timestamp)
            return
        seen = any(item["id"] == self.target_id for item in report["accepted"])
        if seen:
            self.last_target_seen = float(timestamp)
            self.missing_target_frames = 0
        else:
            self.missing_target_frames += 1
        if self.missing_target_frames >= self.settings.max_missing_target_frames:
            self.failed[self.target_id] = "Target could not be reacquired geometrically"
            self._choose_next(timestamp, np.array(report["base_pose_xy_yaw"]))
            return
        if phase == "name_capture":
            self.name_attempts += 1
            if name_result and name_result.get("confirmed"):
                self._choose_next(timestamp, np.array(report["base_pose_xy_yaw"]))
            elif self.name_attempts >= self.settings.max_read_attempts:
                self.failed[self.target_id] = (name_result or {}).get(
                    "reason", "Name not confirmed in recipient database"
                )
                self._choose_next(timestamp, np.array(report["base_pose_xy_yaw"]))
            elif (
                len(self._standoffs()) > 1
                and self.name_attempts % self.settings.attempts_per_standoff == 0
                and self.stage + 1 < len(self._standoffs())
            ):
                self.stage += 1  # not readable from here: come closer to this box
                self._enter("approach", timestamp)
            else:
                self._enter("name_settle", timestamp)
        else:
            self._enter("approach", timestamp)

    def _choose_next(self, timestamp, base_pose):
        candidates = [
            m
            for m in self.slam.mailboxes()
            if not m["name_confirmed"]
            and m["id"] not in self.failed
            and m["observations"] >= self.settings.min_sightings
        ]
        if not candidates:
            all_boxes = self.slam.mailboxes()
            complete = bool(all_boxes) and all(m["name_confirmed"] for m in all_boxes)
            self.target_id = None
            self.reason = (
                None if complete else "Some discovered boxes remain unread, or no boxes were found"
            )
            self._enter("complete" if complete else "incomplete", timestamp)
            return
        chosen = min(
            candidates,
            key=lambda m: np.linalg.norm(np.array(m["label_center_xyz_m"][:2]) - base_pose[:2]),
        )
        self.target_id = chosen["id"]
        self.target_started = float(timestamp)
        self.name_attempts, self.missing_target_frames, self.stage = 0, 0, 0
        self.last_target_seen = None
        self._enter("approach", timestamp)

    def status(self):
        return dict(
            phase=self.phase,
            scan_complete=self.scan_complete,
            discovery_only=self.settings.discovery_only
            or self.settings.expected_mailboxes is not None,
            expected_mailboxes=self.settings.expected_mailboxes,
            detected_mailboxes=len(self.slam.mailboxes()),
            count_matches=(
                None
                if self.settings.expected_mailboxes is None
                else len(self.slam.mailboxes()) == self.settings.expected_mailboxes
            ),
            scan_progress_rad=self.yaw_progress,
            scan_fraction=self._scan_fraction(),
            reading_distance_m=self.reading_distance,
            second_pass_standoff_m=self.second_pass_standoff,
            scan_mode=self.settings.scan_mode,
            travel_m=self.travel_progress,
            target_id=self.target_id,
            reason=self.reason,
            failed=dict(self.failed),
            mailboxes=self.slam.mailboxes(),
        )
