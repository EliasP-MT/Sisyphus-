string name
uint8[] motor_ids            # 1..8 (8 = gripper, raw ticks only)
int32[] ticks
string[] joint_names         # calibrated arm joints (rev_motor_01..07)
float64[] joint_positions    # radians from the verified calibration
string recorded_at
string calibration_sha256    # calibration used at capture; empty for imported poses
string[] warnings            # e.g. outside operating margin (never clipped)
