import cv2
import numpy as np
from collections import deque
import math
import time
import os


from motor_control import (
    send_absolute_position_mm,
    open_serial,
    home_all_motors
)


from actuator_utils import (
    actuator_length,
    polar_to_tilt_components,
    limit_change,
    send_actuator_positions,
)

last_time = time.time()

# Serial connection for motor control
ser = open_serial()
if not home_all_motors(ser, settle_position_mm=28.1):
    print("Homing failed. Exiting.")
    exit()


# Motion Control Variables 
prev_time = time.time()
prev_angle = None
prev_velocity_filtered = None


MAX_TILT_DEG = 1.2      # max tilt angle in degrees
GAIN = 0.03           # maps deg/sec to tilt degrees
DEAD_ZONE = 2        # deg/s: don't respond below this speed
lead_angle = 90  # degrees: lead angle for tilt direction


angle_history = deque(maxlen=100)
velocity_history = deque(maxlen=100)
time_history = deque(maxlen=100)

def print_average_frequency(time_history, window_size=50):
    if len(time_history) >= 2:
        # Only use the latest `window_size` timestamps if available
        times = list(time_history)[-window_size:]
        if len(times) >= 2:
            total_time = times[-1] - times[0]
            avg_freq = (len(times) - 1) / total_time if total_time > 0 else 0
            print(f"⏱ Average frequency over {len(times)} frames: {avg_freq:.2f} Hz")
            
tilt_azimuth  = 0.0  # Initial azimuth angle in degrees
angular_velocity_degree_s = 60.0
tilt_magnitude = 0.8 # Tilt magnitude in degrees
actuator_value_offset = 0.5
speed_rpm = 400

try:
    while True:
        # Compute time difference
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        tilt_azimuth += angular_velocity_degree_s * dt  # Update azimuth angle
        tilt_azimuth = tilt_azimuth % 360
        
        
        
        # Set actuator target positions
        target_elevation = np.radians(tilt_magnitude)
        target_azimuth = np.radians(tilt_azimuth)
        print(f"Target Elevation: {np.degrees(target_elevation):8.3f} degrees , Azimuth: {np.degrees(target_azimuth):8.3f} degrees")

        # Apply rate limiting filter
        # current_elevation = limit_change(current_elevation, target_elevation, filter_rate_elevation)
        # current_azimuth = limit_change(current_azimuth, target_azimuth, filter_rate_azimuth)

        theta_x, theta_y = polar_to_tilt_components(target_elevation, target_azimuth)

        L_x_plus  = actuator_length(theta_x) + actuator_value_offset
        L_x_minus = actuator_length(-theta_x) + actuator_value_offset
        L_y_plus  = actuator_length(theta_y) + actuator_value_offset
        L_y_minus = actuator_length(-theta_y) + actuator_value_offset

        wait_for_ack = False
        send_absolute_position_mm(ser, L_x_plus, 1, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
        send_absolute_position_mm(ser, L_y_minus, 2, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
        send_absolute_position_mm(ser, L_x_minus, 3, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
        send_absolute_position_mm(ser, L_y_plus, 4, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)

        print (f"Actuator Positions: L_x_plus={L_x_plus:.2f}, L_y_minus={L_y_minus:.2f}, L_x_minus={L_x_minus:.2f}, L_y_plus={L_y_plus:.2f}")

        now = time.time()
        dt = now - last_time
        # Compute frequency of updates using last 10 time_history
        print_average_frequency(time_history, window_size=50)
        time_history.append(now)
        last_time = now


except KeyboardInterrupt:
    print("\nExiting...")
finally:
    send_absolute_position_mm(ser, 21.5, 0, speed_rpm=200)
    ser.close()
    cv2.destroyAllWindows()

