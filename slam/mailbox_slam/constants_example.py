"""Copy into your own importable module and fill ALL hardware placeholders.

None values intentionally prevent accidental use of invented robot geometry.
Transforms are 4x4 matrices; see mailbox_slam/README.md for conventions.
"""

SLAM_CONFIG = {
    "camera": {
        "fx": None,
        "fy": None,
        "cx": None,
        "cy": None,
        "width": None,
        "height": None,
        "distortion": None,
    },
    "arm": {
        "T_base_arm": None,
        "T_last_camera": None,  # Last link -> camera OPTICAL frame.
        "joints": [
            # Replace with one entry per physical revolute joint, base to camera.
            {
                "name": "replace_joint_name",
                "T_parent_joint": None,
                "axis": None,
                "zero_tick": None,
                "ticks_per_revolution": None,
                "sign": None,
                "modulus": None,
                "max_rad_s": None,
            },
        ],
    },
    "wheels": [
        {
            "name": name,
            "position_xy_m": None,
            "rolling_direction_xy": None,
            "radius_m": None,
            "ticks_per_revolution": None,
            "sign": None,
            "modulus": None,
            "max_rad_s": None,
        }
        for name in ("replace_wheel_1", "replace_wheel_2", "replace_wheel_3")
    ],
    "settings": {},  # Software noise/gating defaults; tune against measurements.
}
