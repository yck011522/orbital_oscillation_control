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

# OpenCV Constants

# Global canvas and overlays
canvas_size = 400
radius_max = 180  # mm (adjustable later)
scale = (canvas_size // 2 - 20) / radius_max  # mm to pixels
center = (canvas_size // 2, canvas_size // 2)
scale = (canvas_size // 2 - 20) / 180  # adjust radius_max as needed

# Global elements
trail = deque(maxlen=10)
reversal_angles = deque(maxlen=2)  # store last two reversal points

def create_base_canvas():
    canvas = np.ones((canvas_size, canvas_size, 3), dtype=np.uint8) * 255
    # Grid circles
    for r_mm in range(30, 181, 30):
        r_px = int(r_mm * scale)
        cv2.circle(canvas, center, r_px, (200, 200, 200), 1)
    # Cross lines
    cv2.line(canvas, (center[0], 0), (center[0], canvas_size), (180, 180, 180), 1)
    cv2.line(canvas, (0, center[1]), (canvas_size, center[1]), (180, 180, 180), 1)
    return canvas

def draw_position_trail(img, angle_deg, radius_mm):
    if angle_deg is not None:
        angle_rad = math.radians(angle_deg)
        x = int(center[0] + radius_mm * scale * math.cos(angle_rad))
        y = int(center[1] - radius_mm * scale * math.sin(angle_rad))
        trail.append((x, y))
    else:
        if trail:
            trail.popleft()
    for pt in trail:
        cv2.circle(img, pt, 3, (255, 0, 0), -1)
    if angle_deg is not None:
        cv2.circle(img, (x, y), 6, (0, 0, 255), -1)

def draw_control_vector(img, azimuth_deg, magnitude_deg):
    length = int(magnitude_deg * scale * 50)  # scale factor
    angle_rad = math.radians(azimuth_deg)
    x2 = int(center[0] + length * math.cos(angle_rad))
    y2 = int(center[1] - length * math.sin(angle_rad))
    cv2.arrowedLine(img, center, (x2, y2), (0, 128, 0), 2, tipLength=0.2)

def draw_reversal_markers(img):
    for angle in reversal_angles:
        angle_rad = math.radians(angle)
        rx = int(center[0] + 180 * scale * math.cos(angle_rad))
        ry = int(center[1] - 180 * scale * math.sin(angle_rad))
        cv2.circle(img, (rx, ry), 6, (0, 0, 255), 2)
        cv2.putText(img, f"{int(angle)}°", (rx+5, ry-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        
def render_frame(angle=None, radius=None, azimuth=None, magnitude=None):
    img = create_base_canvas()
    draw_position_trail(img, angle, radius)
    if azimuth is not None and magnitude is not None:
        draw_control_vector(img, azimuth, magnitude)
    draw_reversal_markers(img)
    cv2.imshow("Weight Position (Polar View)", img)
    cv2.waitKey(1)


# Table Control Variables
actuator_value_offset = 0.0
speed_rpm = 300
# filter_rate_elevation = 0.002
# filter_rate_azimuth = 0.5
# filter_rate_elevation = 0.01
# filter_rate_azimuth = 0.9
# target_elevation = 0.0
# target_azimuth = 0.0
# current_elevation = 0.0
# current_azimuth = 0.0


# Motion Control Variables 
prev_time = time.time()
prev_angle = None
prev_velocity_filtered = None
angular_velocity_filtered = 0.0
angular_acceleration = 0.0
tilt_x_filtered = 0.0
tilt_y_filtered = 0.0
tilt_alpha = 0.1  # filtering factor for tilt angles
velocity_alpha = 0.1  # filtering factor (0 = slow, 1 = no filtering)
acceleration_alpha = 0.1  # filtering factor for acceleration

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
try:
    while True:


        # Compute time difference
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        tilt_azimuth += 0.2 # Simulate angle change for testing
        tilt_azimuth = tilt_azimuth % 360
        tilt_magnitude = 0.5
        
        
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
        last_time = now


except KeyboardInterrupt:
    print("\nExiting...")
finally:
    send_absolute_position_mm(ser, 21.5, 0, speed_rpm=200)
    ser.close()
    cv2.destroyAllWindows()

