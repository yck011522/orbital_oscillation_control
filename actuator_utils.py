import numpy as np
import time
from actuator_ik_table05 import create_actuator_lookup_function
lookup_actuator_position = create_actuator_lookup_function()


from motor_control import (
    send_absolute_position_mm,
    read_position_mm,
)

def actuator_length(theta_eff):
    return lookup_actuator_position(theta_eff)

def polar_to_tilt_components(elevation, azimuth):
    theta_x = - np.arctan(np.tan(elevation) * np.cos(azimuth))
    theta_y = - np.arctan(np.tan(elevation) * np.sin(azimuth))
    return theta_x, theta_y

def limit_change(current, target, rate):
    delta = target - current
    if abs(delta) > rate:
        delta = rate * np.sign(delta)
    return current + delta

def send_actuator_positions(ser, positions, speed_rpm, send_func):
    for idx, pos in enumerate(positions, 1):
        send_func(ser, pos, idx, speed_rpm=speed_rpm)


def validate_full_travel(ser, max_deg=1.1, settle_mm=21.0, speed_rpm=200, tolerance_mm=0.1, timeout=10.0):
    """
    Validates each motor's travel to 4 extreme tilt directions.
    """
    def all_motors_reached(targets, timeout):
        start_time = time.time()
        while time.time() - start_time < timeout:
            all_good = True
            for motor_id, target_mm in targets.items():
                pos_mm = read_position_mm(ser, address=motor_id)
                if pos_mm is None or abs(pos_mm - target_mm) > tolerance_mm:
                    all_good = False
                    break
            if all_good:
                return True
            time.sleep(0.05)
        return False

    def send_and_wait(theta_x_deg, theta_y_deg):
        theta_x = np.radians(theta_x_deg)
        theta_y = np.radians(theta_y_deg)
        targets = {
            1: actuator_length(+theta_x),
            2: actuator_length(-theta_y),
            3: actuator_length(-theta_x),
            4: actuator_length(+theta_y),
        }
        for motor_id, mm in targets.items():
            send_absolute_position_mm(ser, mm, address=motor_id, speed_rpm=speed_rpm)
        if not all_motors_reached(targets, timeout):
            print(f"⚠️ Travel to (θx={theta_x_deg}°, θy={theta_y_deg}°) failed or timed out.")
        else:
            print(f"✅ Travel to (θx={theta_x_deg}°, θy={theta_y_deg}°) succeeded.")

    print("=== Starting travel validation ===")
    test_angles = [
        ( max_deg,  0.0),
        ( 0.0,  max_deg),
        (-max_deg,  0.0),
        ( 0.0, -max_deg),
    ]

    for theta_x_deg, theta_y_deg in test_angles:
        send_and_wait(theta_x_deg, theta_y_deg)
        time.sleep(0.5)

    # Return to neutral (0, 0)
    print("Returning to neutral position...")
    neutral_targets = {
        1: actuator_length(0.0),
        2: actuator_length(0.0),
        3: actuator_length(0.0),
        4: actuator_length(0.0),
    }
    for motor_id, mm in neutral_targets.items():
        send_absolute_position_mm(ser, mm, address=motor_id, speed_rpm=speed_rpm)
    all_motors_reached(neutral_targets, timeout)
    print("✅ Validation complete. Motors returned to resting position.")
