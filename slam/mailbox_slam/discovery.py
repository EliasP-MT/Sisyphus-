"""Geometry-first mailbox SLAM: anonymous landmarks first, database names later."""

from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation

from letterbox_locator import LetterboxLocator, load_json, match_recipient
from recipient_names import normalize_name

from .geometry import between, inverse, planar, pose_ambiguous, transform, wrap
from .mapping import MappingCore

ROOT = Path(__file__).resolve().parents[1]


class MailboxDiscovery(MappingCore):
    """Static-arm mapping with anonymous spatial association and deferred OCR.

    start(timestamp) anchors the map BEFORE rotation. The arm stays fixed in this
    session. process_image performs geometry only; read_target_name is explicit.
    Serialize calls from your hardware adapter. All camera and encoder stamps share a clock.
    """

    def __init__(
        self,
        config,
        *,
        geometry=None,
        recipients=None,
        locator=None,
        geometric_gate_m=0.12,
        association_margin_m=0.025,
        rotation_gate_rad=0.6,
        fixed_arm_tolerance_rad=0.03,
        name_confirmations=2,
        max_read_distance_m=0.8,
    ):
        geometry = (
            load_json(ROOT / "config/letterbox.json") if geometry is None else deepcopy(geometry)
        )
        recipients = (
            load_json(ROOT / "config/recipients.json") if recipients is None else list(recipients)
        )
        for key in ("label_width_m", "label_height_m", "slot_width_m", "slot_height_m"):
            if not np.isfinite(geometry[key]) or geometry[key] <= 0:
                raise ValueError(f"{key} must be a measured positive size")
        offset = np.asarray(geometry["slot_from_label_center_m"], float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("slot_from_label_center_m must be a finite 3D offset")
        identities = [normalize_name(name) for name in recipients]
        if (
            not identities
            or any(not name for name in identities)
            or len(set(identities)) != len(identities)
        ):
            raise ValueError("Recipient database must have unique nonempty normalized names")
        for value in (
            geometric_gate_m,
            association_margin_m,
            rotation_gate_rad,
            fixed_arm_tolerance_rad,
            max_read_distance_m,
        ):
            if not np.isfinite(value) or value <= 0:
                raise ValueError("Discovery thresholds must be positive finite numbers")
        if int(name_confirmations) != name_confirmations or name_confirmations < 1:
            raise ValueError("name_confirmations must be a positive integer")
        config = deepcopy(config)
        # These are the actual dimensions of the contour detected by the locator.
        config["tag"] = dict(width_m=geometry["label_width_m"], height_m=geometry["label_height_m"])
        super().__init__(config)
        self.geometry, self.recipients = geometry, recipients
        self.locator = (
            locator
            if locator is not None
            else LetterboxLocator(self.config["camera"], geometry, recipients, ocr=True)
        )
        self.geometric_gate_m = float(geometric_gate_m)
        self.association_margin_m = float(association_margin_m)
        self.rotation_gate_rad = float(rotation_gate_rad)
        self.fixed_arm_tolerance_rad = float(fixed_arm_tolerance_rad)
        self.max_read_distance_m = float(max_read_distance_m)
        self.name_confirmations = int(name_confirmations)
        self.origin_timestamp = None
        self.fixed_angles = None
        self.last_image_timestamp = None
        self.last_associations = {}
        self.name_evidence = {}
        self.defer_optimization = False  # set by the survey during the sweep
        # All boxes sit on one wall that the base faces at the start: their faces are parallel to
        # it. A face normal estimated from one 6 cm label is far noisier (tens of degrees) than that.
        self.parallel_wall = False
        self.sweep_reads = {}

    def start(self, timestamp):
        if self.graph.frames:
            raise ValueError("Mapping already started")
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError("Timestamp must be finite")
        odom = self.wheels.buffer.at(timestamp)
        self.fixed_angles = self.arm.buffer.at(timestamp)
        self.arm.camera_pose(timestamp)  # Check availability before creating origin.
        self.graph.add_frame(timestamp, odom)
        self.origin_timestamp = timestamp
        return dict(timestamp=timestamp, base_pose_xy_yaw=[0.0, 0.0, 0.0])

    def check_fixed_arm(self, timestamp):
        if self.fixed_angles is None:
            raise ValueError("Call start(timestamp) before rotating")
        angles = self.arm.buffer.at(timestamp)
        if np.max(abs(angles - self.fixed_angles)) > self.fixed_arm_tolerance_rad:
            raise ValueError("Arm moved from its fixed survey pose")

    def pose_at(self, timestamp):
        """Corrected current planar base estimate propagated with wheel encoders."""
        self.check_fixed_arm(timestamp)
        if timestamp < self.graph.frames[-1]["timestamp"]:
            raise ValueError("Control timestamp precedes the latest mapped frame")
        odom = self.wheels.buffer.at(timestamp)
        previous = self.graph.frames[-1]
        return planar(previous["pose"]) @ planar(between(previous["odom"], odom))

    def _capture_state(self, timestamp):
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError("Image timestamp must be finite")
        if self.origin_timestamp is None:
            raise ValueError("Call start(timestamp) at the robot starting position before scanning")
        if timestamp < self.origin_timestamp or (
            self.last_image_timestamp is not None and timestamp <= self.last_image_timestamp
        ):
            raise ValueError("Image timestamps must increase and follow the map origin")
        self.check_fixed_arm(timestamp)
        return self.wheels.buffer.at(timestamp), self.arm.camera_pose(timestamp)

    def process_image(self, image, timestamp, *, allow_new=True):
        """Geometry only: no OCR initialization or database lookup during scanning."""
        self._capture_state(timestamp)
        return self.process_locator_results(
            timestamp, self.locator.locate_all(image, read_names=False), allow_new=allow_new
        )

    def process_observations(self, timestamp, observations, *, allow_new=True):
        """Validated pose input for adapters/simulation; names are intentionally ignored."""
        results = []
        for item in observations:
            pose = transform(item["T_camera_tag"], "T_camera_tag")
            results.append(
                dict(
                    label_center_m=pose[:3, 3].tolist(),
                    label_rotation=pose[:3, :3].tolist(),
                    reprojection_px=float(item.get("reprojection_px", 0.0)),
                )
            )
        return self.process_locator_results(timestamp, results, allow_new=allow_new)

    def process_locator_results(self, timestamp, results, *, allow_new=True):
        """Consume all label geometry from LetterboxLocator, without textual IDs."""
        odom, camera = self._capture_state(timestamp)
        prepared, rejected = [], []
        for detection_index, result in enumerate(results):
            candidates = []
            for candidate in result.get("pose_candidates", [result]):
                error = float(candidate["reprojection_px"])
                if (
                    not np.isfinite(error)
                    or error < 0
                    or error > self.settings["max_reprojection_px"]
                ):
                    continue
                pose = np.eye(4)
                pose[:3, :3] = candidate["label_rotation"]
                pose[:3, 3] = candidate["label_center_m"]
                pose = transform(pose, "T_camera_tag")
                if pose[2, 3] <= 0:
                    continue
                candidates.append(dict(pose=pose, rms_px=error))
            if not candidates:
                rejected.append(dict(detection=detection_index, reason="no valid label pose"))
                continue
            candidates.sort(key=lambda c: c["rms_px"])
            prepared.append(dict(index=detection_index, candidates=candidates, result=result))
        # start() may have added the origin at this exact first image timestamp.
        if timestamp == self.graph.frames[-1]["timestamp"]:
            frame = len(self.graph.frames) - 1
        else:
            frame = self.graph.add_frame(float(timestamp), odom)
        self.last_image_timestamp = float(timestamp)
        world_camera = planar(self.graph.frames[frame]["pose"]) @ camera
        ids = list(self.graph.landmarks)
        costs = np.full((len(prepared), len(ids)), np.inf)
        choices = {}
        for i, observation in enumerate(prepared):
            for j, key in enumerate(ids):
                landmark = self.graph.landmarks[key]["pose"]
                scores = []
                for candidate_index, candidate in enumerate(observation["candidates"]):
                    predicted = world_camera @ candidate["pose"]
                    distance = float(np.linalg.norm(predicted[:3, 3] - landmark[:3, 3]))
                    angle = Rotation.from_matrix(predicted[:3, :3].T @ landmark[:3, :3]).magnitude()
                    if distance <= self.geometric_gate_m and angle <= self.rotation_gate_rad:
                        scores.append((distance, angle, candidate_index))
                if scores:
                    distance, _, candidate_index = min(scores)
                    costs[i, j] = distance
                    choices[(i, j)] = candidate_index
        ambiguous_rows = set()
        for i in range(len(prepared)):
            distances = np.sort(costs[i][np.isfinite(costs[i])])
            if len(distances) > 1 and distances[1] - distances[0] < self.association_margin_m:
                ambiguous_rows.add(i)
        matching = {}
        if len(prepared) and ids:
            augmented = np.full((len(prepared), len(ids) + len(prepared)), 10.0)
            augmented[:, : len(ids)] = np.where(np.isfinite(costs), costs, 100.0)
            for i in ambiguous_rows:
                augmented[i, : len(ids)] = 100.0
            rows, columns = linear_sum_assignment(augmented)
            matching = {
                i: j for i, j in zip(rows, columns) if j < len(ids) and augmented[i, j] < 10
            }
        accepted = []
        self.last_associations = {}
        for i, observation in enumerate(prepared):
            if i in ambiguous_rows:
                rejected.append(
                    dict(detection=observation["index"], reason="ambiguous spatial association")
                )
                continue
            if i in matching:
                j = matching[i]
                key = ids[j]
                chosen = observation["candidates"][choices[(i, j)]]
            else:
                if np.isfinite(costs[i]).any():
                    rejected.append(
                        dict(
                            detection=observation["index"],
                            reason="duplicate sighting of assigned landmark",
                        )
                    )
                    continue
                chosen = observation["candidates"][0]
                if not allow_new:
                    rejected.append(
                        dict(
                            detection=observation["index"],
                            reason="unknown landmark; discovery disabled",
                        )
                    )
                    continue
                if pose_ambiguous(observation["candidates"], self.settings):
                    rejected.append(
                        dict(
                            detection=observation["index"],
                            reason="ambiguous planar pose; another viewpoint required",
                        )
                    )
                    continue
                predicted = world_camera @ chosen["pose"]
                if any(
                    np.linalg.norm(predicted[:3, 3] - existing["pose"][:3, 3])
                    < self.geometric_gate_m
                    for existing in self.graph.landmarks.values()
                ):
                    rejected.append(
                        dict(
                            detection=observation["index"],
                            reason="near existing landmark with inconsistent pose",
                        )
                    )
                    continue
                key = f"mailbox_{len(self.graph.landmarks) + 1:03d}"
                self.graph.landmarks[key] = dict(
                    pose=predicted, name=None, identity=None, observations=0
                )
            landmark = self.graph.landmarks[key]
            measurement = camera @ chosen["pose"]
            self.graph.observations.append((frame, key, measurement))
            landmark["observations"] += 1
            self.raw_observations.append(
                dict(
                    frame=frame,
                    landmark_id=key,
                    name=landmark["name"],
                    T_base_tag=measurement.tolist(),
                    T_base_camera=camera.tolist(),
                    corners_px=observation["result"].get("label_corners_px"),
                )
            )
            accepted.append(
                dict(id=key, detection=observation["index"], reprojection_rms_px=chosen["rms_px"])
            )
            self.last_associations[key] = observation["result"]
        if accepted and not self.defer_optimization:
            self.graph.optimize()
        self.last_report = dict(
            timestamp=float(timestamp),
            frame=frame,
            accepted=accepted,
            rejected=rejected,
            base_pose_xy_yaw=self.graph.frames[frame]["pose"].tolist(),
            mailboxes=self.mailboxes(),
            tracking="visual_update" if accepted else "odometry_only",
            optimization=deepcopy(self.graph.last_optimization),
        )
        return self.last_report

    def mailboxes(self):
        result = super().mailboxes()
        offset = np.r_[self.geometry["slot_from_label_center_m"], 1.0]
        for item in result:
            pose = np.asarray(item["T_map_tag"])
            item.update(
                label_center_xyz_m=item["position_xyz_m"],
                slot_center_xyz_m=(pose @ offset)[:3].tolist(),
                slot_push_direction_xyz=pose[:3, 2].tolist(),
                slot_width_m=self.geometry["slot_width_m"],
                slot_height_m=self.geometry["slot_height_m"],
                slot_source="known_rigid_offset",
                name_confirmed=item["name"] is not None,
                name_source="recipient_database" if item["name"] is not None else None,
                name_confirmations=len(self.name_evidence.get(item["id"], {}).get("frames", [])),
                name_match_score=self.graph.landmarks[item["id"]].get("name_match_score"),
            )
        return result

    def approach_pose(self, key, standoff_m=0.30):
        """Base XY/yaw placing the FIXED camera standoff_m in front of the label.

        This is a geometric goal, not a collision-free/reachability-certified path.
        Projection checks reject labels outside the fixed camera's view at goal.
        """
        if not np.isfinite(standoff_m) or standoff_m <= 0:
            raise ValueError("Standoff must be positive")
        tag = self.graph.landmarks[key]["pose"]
        if self.parallel_wall:
            # Outward normal = opposite of the fixed camera's heading at the start pose, i.e. the
            # wall is perpendicular to where the camera looked when the survey began.
            start_forward = self._fixed_camera_pose()[:2, 2]
            if np.linalg.norm(start_forward) < 0.2:
                raise ValueError("Fixed camera does not point horizontally enough for base approach")
            normal = np.array([*(-start_forward / np.linalg.norm(start_forward)), 0.0])
        else:
            normal = -tag[:3, 2]
        horizontal = np.linalg.norm(normal[:2])
        if horizontal < 0.5:
            raise ValueError("Mailbox face is not approximately vertical")
        direction = normal[:2] / horizontal
        # Mailbox fronts are vertical: a tilted estimate is pose noise from an oblique view
        # (planar ambiguity), so only the horizontal direction of the normal is used.
        normal = np.array([direction[0], direction[1], 0.0])
        camera = (
            self.arm.camera_pose(self.origin_timestamp)
            if self.fixed_angles is None
            else self._fixed_camera_pose()
        )
        forward = camera[:2, 2]
        if np.linalg.norm(forward) < 0.2:
            raise ValueError("Fixed camera does not point horizontally enough for base approach")
        yaw = wrap(np.arctan2(-direction[1], -direction[0]) - np.arctan2(forward[1], forward[0]))
        rotation = planar([0, 0, yaw])[:2, :2]
        # Enforce actual camera-to-plane perpendicular distance, including camera height.
        # n.(camera-tag)=standoff. XY direction accounts for any face tilt.
        distance_xy = (standoff_m - normal[2] * (camera[2, 3] - tag[2, 3])) / horizontal
        if distance_xy <= 0:
            raise ValueError(
                "Requested camera-plane standoff is incompatible with fixed camera height"
            )
        xy = tag[:2, 3] + distance_xy * direction - rotation @ camera[:2, 3]
        base_goal = planar([xy[0], xy[1], yaw])
        camera_tag = inverse(base_goal @ camera) @ tag
        self._check_label_visible(camera_tag)
        return dict(
            base_pose_xy_yaw=[float(xy[0]), float(xy[1]), float(yaw)],
            camera_plane_standoff_m=float(standoff_m),
            label_id=key,
        )

    def wall_distance(self, key):
        """Perpendicular camera-to-wall distance at the survey start (the first pass distance)."""
        if not self.parallel_wall:
            raise ValueError("Wall distance needs the parallel-wall assumption")
        forward = self._fixed_camera_pose()[:2, 2]
        normal = -forward / np.linalg.norm(forward)
        camera = self._fixed_camera_pose()[:2, 3]
        return float(normal @ (camera - self.graph.landmarks[key]["pose"][:2, 3]))

    def _fixed_camera_pose(self):
        return self.arm.forward(self.fixed_angles)

    def _check_label_visible(self, camera_tag):
        import cv2

        from letterbox_locator import camera_matrix

        w, h = self.geometry["label_width_m"], self.geometry["label_height_m"]
        points = np.array(
            [[-w / 2, -h / 2, 0], [w / 2, -h / 2, 0], [w / 2, h / 2, 0], [-w / 2, h / 2, 0]]
        )
        if np.any((points @ camera_tag[:3, :3].T + camera_tag[:3, 3])[:, 2] <= 0):
            raise ValueError("Label is behind the camera at the approach goal")
        k, d = camera_matrix(self.config["camera"])
        pixels, _ = cv2.projectPoints(
            points, Rotation.from_matrix(camera_tag[:3, :3]).as_rotvec(), camera_tag[:3, 3], k, d
        )
        pixels = pixels.reshape(-1, 2)
        c = self.config["camera"]
        if (
            np.any(pixels < 5)
            or np.any(pixels[:, 0] > c["width"] - 5)
            or np.any(pixels[:, 1] > c["height"] - 5)
        ):
            raise ValueError("Label would be outside the fixed camera view at the approach goal")

    def assign_name(self, key, text, *, frame=None, required=None):
        """Attach only an unambiguous database name after independent confirmations."""
        landmark = self.graph.landmarks[key]
        frame = len(self.graph.frames) - 1 if frame is None else frame
        if not 0 <= frame < len(self.graph.frames):
            raise ValueError("Name evidence must reference an existing mapping frame")
        if not any(
            f == frame and landmark_id == key for f, landmark_id, _ in self.graph.observations
        ):
            raise ValueError(
                "Name evidence requires a geometric sighting of this mailbox in this frame"
            )
        name, score = match_recipient(text, self.recipients)
        if name is None:
            self.name_evidence.pop(key, None)
            return dict(
                id=key,
                recipient=None,
                confirmed=False,
                match_score=score,
                reason="no unambiguous database match",
            )
        identity = normalize_name(name)
        if any(
            other != key and item["identity"] == identity
            for other, item in self.graph.landmarks.items()
        ):
            return dict(
                id=key,
                recipient=None,
                confirmed=False,
                match_score=score,
                reason="recipient already assigned to another mailbox",
            )
        if landmark["name"] is not None and landmark["identity"] != identity:
            return dict(
                id=key,
                recipient=None,
                confirmed=False,
                match_score=score,
                reason="conflicts with confirmed mailbox name",
            )
        evidence = self.name_evidence.get(key)
        if evidence is None or evidence["name"] != name:
            evidence = dict(name=name, frames=set())
            self.name_evidence[key] = evidence
        evidence["frames"].add(frame)
        needed = self.name_confirmations if required is None else int(required)
        confirmed = len(evidence["frames"]) >= needed
        if confirmed:
            landmark["name"], landmark["identity"] = name, identity
            landmark["name_match_score"] = score
        return dict(
            id=key,
            recipient=name,
            confirmed=confirmed,
            match_score=score,
            confirmations=len(evidence["frames"]),
            required=needed,
        )

    def read_target_name(self, image, timestamp, key):
        """Re-localize geometrically first, then OCR only the matched target label."""
        report = self.process_image(image, timestamp, allow_new=False)
        if key not in self.last_associations:
            return dict(
                id=key,
                confirmed=False,
                recipient=None,
                reason="target not geometrically visible",
                geometry=report,
            )
        result = self.last_associations[key]
        if np.linalg.norm(result["label_center_m"]) > self.max_read_distance_m:
            return dict(
                id=key,
                confirmed=False,
                recipient=None,
                reason="target too far for name reading",
                geometry=report,
            )
        if "label_corners_px" not in result:
            raise ValueError("Name reading requires label corners, not just injected poses")
        read = self.locator.read_label_name(image, result["label_corners_px"])
        assigned = self.assign_name(key, read["ocr_text"], frame=report["frame"])
        return {**read, **assigned, "geometry": report}

    def read_visible_names(self, image, timestamp, *, max_reads_per_box=4, confirmations=1):
        """First-pass reading: geometry for every label in view, then OCR of each label that
        has no confirmed name yet (the base keeps its reading distance while it slides along the
        wall). Reads that fail stay unconfirmed; the survey falls back to a closer visit."""
        report = self.process_image(image, timestamp, allow_new=True)
        reads = self.sweep_reads
        outcomes = []
        for key, result in list(self.last_associations.items()):
            if self.graph.landmarks[key]["name"] is not None:
                continue
            if reads.get(key, 0) >= max_reads_per_box:
                continue
            if np.linalg.norm(result["label_center_m"]) > self.max_read_distance_m:
                continue
            corners = result.get("label_corners_px")
            if corners is None:
                continue
            reads[key] = reads.get(key, 0) + 1
            try:
                read = self.locator.read_label_name(image, corners)
            except (RuntimeError, ValueError):
                continue  # OCR service unavailable: geometry still counts, names come later
            outcomes.append(
                {
                    **read,
                    **self.assign_name(
                        key, read["ocr_text"], frame=report["frame"], required=confirmations
                    ),
                }
            )
        return {**report, "reads": outcomes}

    def map_payload(self):
        payload = super().map_payload()
        payload.update(
            map_origin="robot base at explicit survey start, before rotation",
            origin_timestamp=self.origin_timestamp,
            letterbox_geometry=self.geometry,
            fixed_joint_angles_rad=None
            if self.fixed_angles is None
            else self.fixed_angles.tolist(),
            recipients=self.recipients,
        )
        return payload
