import cv2
import math
import time

from motor_control import (
    send_absolute_position_mm,
    open_serial,
    home_all_motors
)
from actuator_utils import (
    actuator_length,
    polar_to_tilt_components,
)

# --- Configurable constants ---
ACTUATOR_OFFSET = 0.2
DEFAULT_RPM = 200
PITCH_MM = 2.0
SPEED_ADJUSTMENT = 1.0

# --- Actuator positions memory for estimating speed ---
prev_positions = {1: None, 2: None, 3: None, 4: None}
prev_time = time.time()

# --- Motor Setup ---
ser = open_serial()
if not home_all_motors(ser, settle_position_mm=28.1):
    print("Homing failed. Exiting.")
    exit()

# --- OpenCV UI Setup ---
cv2.namedWindow("Manual Tilt Control")

# Trackbars: scale tilt from 0–120 (maps to 0.0–1.2°), azimuth 0–360
cv2.createTrackbar("Tilt (x100)", "Manual Tilt Control", 0, 120, lambda x: None)
cv2.createTrackbar("Azimuth (deg)", "Manual Tilt Control", 0, 360, lambda x: None)

def mm_per_s_to_rpm(v_mm_s, pitch_mm=PITCH_MM):
    return abs(v_mm_s * 60 / pitch_mm)

try:
    while True:
        now = time.time()
        dt = now - prev_time
        prev_time = now

        # Read slider values
        tilt_deg = cv2.getTrackbarPos("Tilt (x100)", "Manual Tilt Control") / 100.0
        azimuth_deg = cv2.getTrackbarPos("Azimuth (deg)", "Manual Tilt Control")

        # Convert to radians
        tilt_rad = math.radians(tilt_deg)
        azimuth_rad = math.radians(azimuth_deg)

        theta_x, theta_y = polar_to_tilt_components(tilt_rad, azimuth_rad)

        # Compute actuator positions
        L_x_plus  = actuator_length(theta_x) + ACTUATOR_OFFSET
        L_x_minus = actuator_length(-theta_x) + ACTUATOR_OFFSET
        L_y_plus  = actuator_length(theta_y) + ACTUATOR_OFFSET
        L_y_minus = actuator_length(-theta_y) + ACTUATOR_OFFSET

        target_positions = {
            1: L_x_plus,
            2: L_y_minus,
            3: L_x_minus,
            4: L_y_plus
        }

        estimated_rpm = {}
        for motor_id, pos in target_positions.items():
            prev_pos = prev_positions[motor_id]
            if prev_pos is not None and dt > 0:
                velocity_mm_s = (pos - prev_pos) / dt
                rpm = mm_per_s_to_rpm(velocity_mm_s)
                estimated_rpm[motor_id] = int(min(rpm * SPEED_ADJUSTMENT, 1000))
            else:
                estimated_rpm[motor_id] = DEFAULT_RPM
        prev_positions = target_positions

        # Send to motors
        send_absolute_position_mm(ser, L_x_plus, 1, speed_rpm=estimated_rpm[1])
        send_absolute_position_mm(ser, L_y_minus, 2, speed_rpm=estimated_rpm[2])
        send_absolute_position_mm(ser, L_x_minus, 3, speed_rpm=estimated_rpm[3])
        send_absolute_position_mm(ser, L_y_plus, 4, speed_rpm=estimated_rpm[4])

        print(f"Tilt: {tilt_deg:.2f}°, Azimuth: {azimuth_deg:.2f}° | "
              f"Lx+: {L_x_plus:.2f}, Lx-: {L_x_minus:.2f}, "
              f"Ly+: {L_y_plus:.2f}, Ly-: {L_y_minus:.2f}")
        
        # GUI wait
        key = cv2.waitKey(30)
        if key == 27:  # ESC to exit
            break

finally:
    print("Resetting actuators and closing...")
    send_absolute_position_mm(ser, 21.5, 0, speed_rpm=200)
    ser.close()
    cv2.destroyAllWindows()
