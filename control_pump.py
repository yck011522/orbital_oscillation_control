import cv2
import numpy as np
from collections import deque
import math
import time
import os


from motor_control import send_absolute_position_mm, open_serial, home_all_motors

# Import sensor utilities and constants
from sensor_utils import (
    find_modbus_device,
    read_all_channels_block,
    channels,
    SLOPE,
    TARE,
    SENSOR_POSITIONS,
)

from actuator_utils import (
    actuator_length,
    polar_to_tilt_components,
    limit_change,
    send_actuator_positions,
)


# Connect to Modbus device
client = find_modbus_device()

if not client:
    print("Modbus client not initialized.")
    exit(1)
    
# Uncomment the following line to use a specific port directly
# client = ModbusSerialClient(
#     port="COM18", baudrate=38400, bytesize=8, parity="N", stopbits=1, timeout=1
# )

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
velocity_alpha = 0.2  # filtering factor (0 = slow, 1 = no filtering)
acceleration_alpha = 0.2  # filtering factor for acceleration

MAX_TILT_DEG = 1.2      # max tilt angle in degrees
GAIN = 0.03           # maps deg/sec to tilt degrees
DEAD_ZONE = 2        # deg/s: don't respond below this speed
lead_angle = 90  # degrees: lead angle for tilt direction


angle_history = deque(maxlen=100)
velocity_history = deque(maxlen=100)
time_history = deque(maxlen=100)


try:
    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print("Live Load Cell Readout (kg):\n")
        raw_values = read_all_channels_block(client)
        weights = {}
        if raw_values is None:
            print("Modbus read error.")
            continue

        for ch_name, raw in zip(channels.keys(), raw_values):
            zero = TARE.get(ch_name, 0)
            slope = SLOPE.get(ch_name, 0)
            kg = (raw - zero) * slope * 0.001
            print(f"{ch_name}: {kg:8.3f} kg (raw: {raw})")
            weights[ch_name] = kg

        weighted_sum_x = sum(weights[ch] * SENSOR_POSITIONS[ch][0] for ch in weights)
        weighted_sum_y = sum(weights[ch] * SENSOR_POSITIONS[ch][1] for ch in weights)
        total = sum(weights.values())
        print(f"\nTotal: {total:8.3f} kg")

        # Calculate weighted average position
        position_x = weighted_sum_x / total
        position_y = weighted_sum_y / total
        distance_from_center = math.sqrt(position_x**2 + position_y**2)
        angle_from_x_axis = math.degrees(math.atan2(position_y, position_x))

        # Normalize angle to [0, 360)
        angle_from_x_axis = angle_from_x_axis % 360

        # Compute time difference
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Calculate angular velocity
        if prev_angle is not None and dt > 0:
            # Compute angular difference, handle wraparound
            raw_diff = angle_from_x_axis - prev_angle
            if raw_diff > 180:
                raw_diff -= 360
            elif raw_diff < -180:
                raw_diff += 360

            # Angular velocity in deg/sec
            angular_velocity = raw_diff / dt

            # Apply exponential smoothing filter
            angular_velocity_filtered = (
                velocity_alpha * angular_velocity +
                (1 - velocity_alpha) * angular_velocity_filtered
            )

            
        # Store current angle for next cycle
        prev_angle = angle_from_x_axis
        
        # Calculate angular acceleration
        if prev_velocity_filtered is not None and dt > 0:
            angular_acceleration = (angular_velocity_filtered - prev_velocity_filtered) / dt
            angular_acceleration_filtered = (
                acceleration_alpha * angular_acceleration +
                (1 - acceleration_alpha) * angular_acceleration
            )
        # Store current velocity for next cycle
        prev_velocity_filtered = angular_velocity_filtered

        print(f"Angular velocity: {angular_velocity_filtered:.2f} deg/s. Angular acceleration: {angular_acceleration:.2f} deg/s²")
        
        angle_history.append(angle_from_x_axis)
        velocity_history.append(angular_velocity_filtered)
        time_history.append(time.time())

        if len(velocity_history) >= 3:
            v1, v2, v3 = velocity_history[-3], velocity_history[-2], velocity_history[-1]
            a1, a2, a3 = angle_history[-3], angle_history[-2], angle_history[-1]

            # Detect zero-crossing (velocity sign change)
            if v1 > 0 and v2 <= 0:
                print(f"🔁 CW to CCW reversal at angle {a2:.1f}")
                reversal_angles.append(a2)
            elif v1 < 0 and v2 >= 0:
                print(f"🔁 CCW to CW reversal at angle {a2:.1f}")

            # Detect angular extrema (local maxima/minima)
            if (a2 > a1 and a2 > a3) or (a2 < a1 and a2 < a3):
                print(f"📍 Oscillation extremum at angle {a2:.1f}")

        # # Skip control if not enough motion
        # if abs(angular_velocity_filtered) < DEAD_ZONE:
        #     tilt_magnitude = 0
        #     tilt_azimuth = 0
        # else:
        #     # Limit tilt angle to MAX_TILT_DEG
        #     tilt_magnitude = min(abs(angular_velocity_filtered) * GAIN, MAX_TILT_DEG)

        #     # 90 deg lead based on direction
        #     lead = lead_angle * math.copysign(1, angular_velocity_filtered)

        #     tilt_azimuth = (angle_from_x_axis + lead) % 360
        
        # if angular_velocity_filtered < 0:
        #     tilt_magnitude = 0
        DEAD_ZONE = 0
        if abs(angular_velocity_filtered) < DEAD_ZONE:
            tilt_magnitude = 0
            tilt_azimuth = 0
        else:
            # Limit tilt angle to MAX_TILT_DEG
            tilt_magnitude = min(abs(angular_acceleration) * GAIN, MAX_TILT_DEG)

            # 90 deg lead based on direction
            lead = lead_angle * math.copysign(1, angular_acceleration)

            tilt_azimuth = (angle_from_x_axis + lead) % 360

        if angular_acceleration < 0:
            tilt_magnitude = 0

        # Convert tilt azimuth to radians and calculate tilt vector components
        tilt_azimuth_rad = math.radians(tilt_azimuth)
        tilt_x = tilt_magnitude * math.cos(tilt_azimuth_rad)
        tilt_y = tilt_magnitude * math.sin(tilt_azimuth_rad)
        print(f"Tilt X: {tilt_x:8.3f} mm , Y: {tilt_y:8.3f} mm")

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

        send_absolute_position_mm(ser, L_x_plus, 1, speed_rpm=speed_rpm)
        send_absolute_position_mm(ser, L_y_minus, 2, speed_rpm=speed_rpm)
        send_absolute_position_mm(ser, L_x_minus, 3, speed_rpm=speed_rpm)
        send_absolute_position_mm(ser, L_y_plus, 4, speed_rpm=speed_rpm)

        if total > 0.1:  # Ignore near-zero weight
            print(f"Position X: {position_x:8.3f} mm , Y: {position_y:8.3f} mm")
            print(f"Distance from center: {distance_from_center:8.3f} mm , Angle: {angle_from_x_axis:8.3f} degrees")
            # Update the display with the current position
            # update_cv_polar(angle_from_x_axis, distance_from_center, tilt_x*50, tilt_y*50)
        else:
            print("No significant weight detected.")
            # update_cv_polar() 
        render_frame(angle_from_x_axis, distance_from_center, tilt_azimuth, tilt_magnitude)

        now = time.time()
        dt = now - last_time
        if dt > 0:
            freq = 1.0 / dt
            print(f"Update frequency: {freq:.2f} Hz")
        last_time = now

        time.sleep(0.001)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    send_absolute_position_mm(ser, 21.5, 0, speed_rpm=200)
    ser.close()
    client.close()
    cv2.destroyAllWindows()

