from postal_robot.calibration import JOINTS, Calibration

ZEROS = [2075, 2065, 2024, 2045, 1888, 1910, 2240]
LIMITS = [(1060, 3036), (1012, 3084), (929, 3167), (926, 3170), (154, 3942), (1069, 3027), (135, 3961)]


def calibration_data():
    return {'hardware_verified': True, 'bus': '5B8E114718', 'servo_speed_ticks_s': 100,
            'servo_acceleration': 10, 'tracking_error_rad': .1, 'max_velocity_rad_s': .1457,
            'joints': [{'name': n, 'motor_id': i + 1, 'zero_ticks': z, 'sign': 1, 'min_ticks': lo,
                        'max_ticks': hi, 'ticks_per_rad': 651.8986469044033}
                       for i, (n, z, (lo, hi)) in enumerate(zip(JOINTS, ZEROS, LIMITS))]}


def calibration():
    return Calibration(calibration_data(), b'fixture')
