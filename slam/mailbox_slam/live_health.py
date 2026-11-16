"""ROS-independent readiness diagnostics; never establishes or extends a lease."""


def readiness_blockers(supervisor, *, runtime_verified, supervisor_age_s, camera_age_s):
    reasons = []
    if not runtime_verified:
        reasons.append("runtime parameters unverified")
    if supervisor is None:
        reasons.append("no supervisor state")
    else:
        if not 0 <= supervisor_age_s < 0.5:
            reasons.append("supervisor state stale")
        if supervisor.state != 1:
            reasons.append(
                f"supervisor state {supervisor.state_name}; keep Station connected and acknowledge in its UI"
            )
        for blocked, message in (
            (not supervisor.base_enabled, "base disabled"),
            (not supervisor.arm_torque, "arm torque off"),
            (supervisor.arm_busy, "arm busy"),
            (supervisor.dispenser_running, "dispenser running"),
            (bool(supervisor.faults), f"faults {list(supervisor.faults)}"),
        ):
            if blocked:
                reasons.append(message)
    if not 0 <= camera_age_s < 1.0:
        reasons.append("arm camera stale or missing")
    return reasons


def geometry_warnings(camera, mount, arm_base):
    warnings = []
    for name, document in (
        ("arm intrinsics", camera),
        ("camera mount", mount),
        ("arm-base transform", arm_base),
    ):
        if document is not None and not (
            document.get("validated") is True or document.get("hardware_verified") is True
        ):
            warnings.append(
                f"{name} not marked validated/hardware_verified; verify against a known physical label"
            )
    return warnings
