"""Sparse pose-graph SLAM: planar base poses and full 3D mailbox poses."""

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from .geometry import between, inverse, planar, pose6, residual6, rigid, wrap


class PoseGraph:
    def __init__(self, settings):
        self.settings = settings
        self.frames = []
        self.landmarks = {}
        self.odometry = []
        self.observations = []
        self.last_optimization = None

    def add_frame(self, timestamp, odom):
        if len(self.frames) >= self.settings["max_frames"]:
            raise ValueError("Mapping session reached max_frames; save map and finish session")
        if self.frames:
            previous = self.frames[-1]
            delta = between(previous["odom"], odom)
            pose = np.array(
                [
                    *(planar(previous["pose"]) @ planar(delta))[:2, 3],
                    wrap(previous["pose"][2] + delta[2]),
                ]
            )
            self.odometry.append((len(self.frames) - 1, len(self.frames), delta))
        else:
            pose = np.zeros(3)  # Gauge: first observed base pose defines map frame.
        self.frames.append(dict(timestamp=timestamp, odom=odom.copy(), pose=pose))
        return len(self.frames) - 1

    def optimize(self):
        n, ids = len(self.frames), list(self.landmarks)
        if not self.observations:
            return
        offsets = {key: 3 * (n - 1) + 6 * i for i, key in enumerate(ids)}
        x0 = np.concatenate(
            [
                *(f["pose"] for f in self.frames[1:]),
                *(pose6(self.landmarks[key]["pose"]) for key in ids),
            ]
        )
        settings = self.settings
        odom_sigma = np.array(
            [settings["odom_translation_sigma_m"]] * 2 + [settings["odom_yaw_sigma_rad"]]
        )
        visual_sigma = np.array(
            [settings["observation_translation_sigma_m"]] * 3
            + [settings["observation_rotation_sigma_rad"]] * 3
        )

        def unpack(x):
            poses = [np.zeros(3)] + [x[3 * i : 3 * i + 3] for i in range(n - 1)]
            landmarks = {key: rigid(x[offsets[key] : offsets[key] + 6]) for key in ids}
            return poses, landmarks

        def residual_reference(x):
            poses, landmarks = unpack(x)
            residuals = []
            for a, b, measured in self.odometry:
                error = between(poses[a], poses[b]) - measured
                error[2] = wrap(error[2])
                residuals.extend(error / odom_sigma)
            for frame, key, measured in self.observations:
                predicted = inverse(planar(poses[frame])) @ landmarks[key]
                residuals.extend(residual6(predicted, measured) / visual_sigma)
            return np.asarray(residuals)

        # Same residual, vectorized: the loop above is the readable definition (kept for the
        # equivalence test); this one is ~20x faster, which matters on the RDK.
        odo_a = np.array([a for a, _, _ in self.odometry], dtype=int)
        odo_b = np.array([b for _, b, _ in self.odometry], dtype=int)
        odo_m = np.array([m for _, _, m in self.odometry], dtype=float).reshape(-1, 3)
        obs_f = np.array([f for f, _, _ in self.observations], dtype=int)
        obs_k = np.array([ids.index(k) for _, k, _ in self.observations], dtype=int)
        obs_t = np.array([m[:3, 3] for _, _, m in self.observations], dtype=float).reshape(-1, 3)
        obs_r = np.array([m[:3, :3] for _, _, m in self.observations], dtype=float).reshape(-1, 3, 3)

        def residual(x):
            frames = np.zeros((n, 3))
            frames[1:] = x[: 3 * (n - 1)].reshape(n - 1, 3)
            lm = x[3 * (n - 1) :].reshape(-1, 6)
            parts = []
            if len(odo_a):
                pa, pb = frames[odo_a], frames[odo_b]
                c, s = np.cos(pa[:, 2]), np.sin(pa[:, 2])
                d = pb[:, :2] - pa[:, :2]
                error = np.stack(
                    [c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], pb[:, 2] - pa[:, 2]], 1
                )
                error = error - odo_m
                error[:, 2] = wrap(error[:, 2])
                parts.append((error / odom_sigma).ravel())
            if len(obs_f):
                pf = frames[obs_f]
                c, s = np.cos(pf[:, 2]), np.sin(pf[:, 2])
                rotations = Rotation.from_rotvec(lm[:, 3:]).as_matrix()[obs_k]
                t = lm[obs_k, :3] - np.column_stack([pf[:, 0], pf[:, 1], np.zeros(len(pf))])
                predicted_t = np.stack([c * t[:, 0] + s * t[:, 1], -s * t[:, 0] + c * t[:, 1], t[:, 2]], 1)
                rz = np.zeros((len(pf), 3, 3))
                rz[:, 0, 0], rz[:, 0, 1], rz[:, 1, 0], rz[:, 1, 1], rz[:, 2, 2] = c, s, -s, c, 1.0
                predicted_r = rz @ rotations
                rot = Rotation.from_matrix(np.swapaxes(obs_r, 1, 2) @ predicted_r).as_rotvec()
                parts.append((np.hstack([predicted_t - obs_t, rot]) / visual_sigma).ravel())
            return np.concatenate(parts)

        self._residual_pair = (residual, residual_reference, x0)

        sparsity = lil_matrix(
            (3 * len(self.odometry) + 6 * len(self.observations), len(x0)), dtype=int
        )
        row = 0
        for a, b, _ in self.odometry:
            for i in (a, b):
                if i:
                    sparsity[row : row + 3, 3 * (i - 1) : 3 * i] = 1
            row += 3
        for frame, key, _ in self.observations:
            if frame:
                sparsity[row : row + 6, 3 * (frame - 1) : 3 * frame] = 1
            sparsity[row : row + 6, offsets[key] : offsets[key] + 6] = 1
            row += 6
        before = residual(x0)
        result = least_squares(
            residual,
            x0,
            jac_sparsity=sparsity.tocsr(),
            loss="soft_l1",
            f_scale=1.0,
            max_nfev=int(settings["max_optimizer_evaluations"]),
        )
        if not np.isfinite(result.x).all():
            raise RuntimeError("Graph optimization returned nonfinite state")
        poses, landmarks = unpack(result.x)
        for frame, pose in zip(self.frames, poses):
            frame["pose"] = pose.copy()
        for key, pose in landmarks.items():
            self.landmarks[key]["pose"] = pose
        self.last_optimization = dict(
            converged=bool(result.success),
            evaluations=result.nfev,
            initial_whitened_rms=float(np.sqrt(np.mean(before**2))),
            final_whitened_rms=float(np.sqrt(np.mean(result.fun**2))),
            message=str(result.message),
        )
