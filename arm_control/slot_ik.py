"""Reach a Cartesian point with the gripper fingertips while imposing joint positions.

Pure functions on top of planner.Arm (forward kinematics, joint bounds, collision world):
  * camera_pose(arm, q, mount)     T_base_camera from the arm pose and the measured camera mount
  * solve_tcp_ik(...)             IK of a point attached to the gripper (the fingertips), with some
                                  joints imposed (the 90 degree wrist pivot) and the gripper axis
                                  aligned with the direction towards the target
Frames: base_link = arm base; Gripper_Base frame: +y is the gripper's forward axis (J7 rolls about it).
"""
import numpy as np
from scipy.optimize import least_squares

FORWARD = np.array([0.0, 1.0, 0.0])  # gripper forward axis in the Gripper_Base frame (J7 roll axis)


def camera_pose(arm, q, mount):
    """4x4 T_base_camera (optical frame: x right, y down, z forward)."""
    parent = np.asarray(mount['T_parent_camera'], float)
    if parent.shape != (4, 4) or not np.isfinite(parent).all():
        raise ValueError('camera mount must be a finite 4x4 transform')
    roll = float(mount.get('roll_correction_deg', 0.0))
    if not -180 <= roll <= 180:
        raise ValueError('roll_correction_deg must be within +/-180')
    if roll:
        # The gripper is physically rolled by `roll` about its forward axis (+y of Gripper_Base, which
        # is also the camera's optical axis) relative to the model: a J7 zero offset. The whole mount
        # turns with it, position included (a camera 7.5 cm to the side of the model's axis is really
        # 7.5 cm above it), not only the image orientation.
        c, s = np.cos(np.radians(roll)), np.sin(np.radians(roll))
        about_forward = np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]])
        parent = about_forward @ parent
    return arm.fk(q) @ parent


def point_in_base(T_base_camera, point_camera):
    return (T_base_camera @ np.append(np.asarray(point_camera, float), 1.0))[:3]


def solve_tcp_ik(arm, seed, target_xyz, tcp_offset_m, approach_dir=None, pinned=None, *,
                 axis_weight=0.03, tolerance_m=0.003, restarts=12, rng_seed=0, regularization=0.02,
                 axis_tolerance_deg=6.0):
    """Joint angles (7) putting the point `tcp_offset_m` (in the Gripper_Base frame) at target_xyz.

    pinned: {joint index: angle in rad} held exactly. approach_dir: unit vector in base_link that
    the gripper's forward axis should point along (soft: aligned within axis_tolerance_deg, the
    exact alignment would cost a large swing of the arm). The arm has more freedom than the
    target needs, so among the solutions that reach it (within tolerance_m, collision-free) the one
    closest to the seed wins: a small move of the fingertips must not become a large swing of the
    arm (the gripper turns about other axes). Returns (q, info) with info = position error (m),
    axis error (deg), largest joint change (deg). Raises ValueError when no solution within the
    joint bounds reaches the point.
    """
    seed = np.asarray(seed, float)
    target = np.asarray(target_xyz, float)
    offset = np.asarray(tcp_offset_m, float)
    pinned = dict(pinned or {})
    lo, hi = arm.bounds[:, 0], arm.bounds[:, 1]
    for index, value in pinned.items():
        if not lo[index] <= value <= hi[index]:
            raise ValueError(f'J{index + 1} pinned at {value:.3f} rad is outside its limits '
                             f'[{lo[index]:.3f}, {hi[index]:.3f}]')
    free = [i for i in range(7) if i not in pinned]
    direction = None
    if approach_dir is not None:
        direction = np.asarray(approach_dir, float)
        direction = direction / np.linalg.norm(direction)

    def assemble(x):
        q = seed.copy()
        q[free] = x
        for index, value in pinned.items():
            q[index] = value
        return q

    def tip(q):
        T = arm.fk(q)
        return T[:3, 3] + T[:3, :3] @ offset, T[:3, :3] @ FORWARD

    def residual_with(weight):
      def residual(x):
        point, axis = tip(assemble(x))
        out = list(point - target)
        if direction is not None:
            out += list(axis_weight * (axis - direction))
        out += list(weight * (x - seed[free]))  # stay near the start
        return np.array(out)
      return residual

    rng = np.random.default_rng(rng_seed)
    best = None
    starts = [seed[free]] + [np.clip(seed[free] + rng.normal(0, 0.4, len(free)), lo[free], hi[free])
                             for _ in range(restarts)]
    for x0 in starts:
        x0 = np.clip(x0, lo[free] + 1e-9, hi[free] - 1e-9)
        # Two passes: with a pull towards the seed (picks the nearest of the many solutions), then
        # exact (the pull would bias the position by a few millimetres).
        rough = least_squares(residual_with(regularization), x0, bounds=(lo[free], hi[free]))
        centre = np.clip(rough.x, lo[free] + 1e-9, hi[free] - 1e-9)
        # trust region (+-0.12 rad per joint) so the polish cannot jump to another solution branch
        result = least_squares(residual_with(0.0), centre,
                               bounds=(np.maximum(lo[free], centre - 0.12), np.minimum(hi[free], centre + 0.12)))
        q = assemble(result.x)
        point, axis = tip(q)
        error = float(np.linalg.norm(point - target))
        axis_error = float(np.degrees(np.arccos(np.clip(axis @ direction, -1, 1)))) if direction is not None else 0.0
        change = float(np.degrees(np.abs(q - seed).max()))
        # reach first, then alignment (within tolerance it no longer matters), then the smallest swing
        score = 100 * error + 0.1 * max(0.0, axis_error - axis_tolerance_deg) + 0.02 * change
        valid = arm.collision.valid(q) if getattr(arm, 'collision', None) else True
        if error <= tolerance_m and valid and (best is None or score < best[0]):
            best = (score, q, error, axis_error, change)
            if axis_error <= axis_tolerance_deg and change <= 40:
                break  # aligned within tolerance and close to the seed: no need to look further
    if best is None:
        raise ValueError('No collision-free arm configuration puts the fingertips on the target with '
                         f'J{", J".join(str(i + 1) for i in pinned) or "-"} imposed '
                         '(unreachable, out of joint limits, or in collision)')
    _, q, error, axis_error, change = best
    return q, dict(position_error_m=error, axis_error_deg=axis_error, max_joint_change_deg=change)
