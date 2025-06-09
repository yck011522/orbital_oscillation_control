import cv2
import numpy as np
from collections import deque
import math
import serial.tools.list_ports

import time
import os
import math
from pymodbus.client import ModbusSerialClient
from pymodbus.client.mixin import ModbusClientMixin

from actuator_ik_table05 import create_actuator_lookup_function
from motor_control import send_absolute_position_mm, open_serial, home_all_motors

lookup_actuator_position = create_actuator_lookup_function()




def list_serial_ports():
    return [port.device for port in serial.tools.list_ports.comports()]

def find_modbus_device(baudrate=38400, test_slave_id=1, test_address=3, test_count=1):
    for port in list_serial_ports():
        try:
            client = ModbusSerialClient(
                port=port,
                baudrate=baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1,
            )
            if client.connect():
                # Try a simple Modbus read to check for valid response
                result = client.read_holding_registers(address=test_address, count=test_count, slave=test_slave_id)
                if result and not result.isError():
                    print(f"✅ Found device on {port}")
                    return client  # Return connected client
                client.close()
        except Exception as e:
            print(f"⚠️ Failed on {port}: {e}")
    print("❌ No Modbus device found.")
    return None

client = find_modbus_device()

if not client:
    print("Modbus client not initialized.")
    exit(1)
    
# Uncomment the following line to use a specific port directly
# client = ModbusSerialClient(
#     port="COM18", baudrate=38400, bytesize=8, parity="N", stopbits=1, timeout=1
# )

dp_raw = client.read_holding_registers(address=3, count=1, slave=1)
decimal_places = dp_raw.registers[0]
print(f"Decimal places: {decimal_places}")

unit_raw = client.read_holding_registers(address=14, count=1, slave=1)
unit_mode = unit_raw.registers[0]
print(f"Unit mode: {unit_mode}")

last_time = time.time()


def read_all_channels_block():
    # Read 8 registers starting at 300 (40301) for 4 channels
    result = client.read_holding_registers(address=300, count=8, slave=1)
    if result.isError():
        return None

    # Convert each 32-bit value from result.registers
    raw_values = []
    for i in range(0, 8, 2):
        val = client.convert_from_registers(
            registers=result.registers[i : i + 2],
            data_type=ModbusClientMixin.DATATYPE.INT32,
            word_order="big",
        )
        raw_values.append(val)

    return raw_values


# Channel register map (40301–40308 => offsets 300–306)
channels = {
    "CH1": 300,
    "CH2": 302,
    "CH3": 304,
    "CH4": 306,
}

# Serial connection
client.connect()

# Serial connection for motor control
ser = open_serial()
if not home_all_motors(ser, settle_position_mm=28.1):
    print("Homing failed. Exiting.")
    exit()


# Slope unit is in gramForce per raw value count    
SLOPE = {"CH1": 0.000160217269343611, "CH2": 0.00015715760370146, "CH3": 0.000158163217397836, "CH4": 0.000160217269343611}  # measured zero values
TARE = {"CH1": 4390912, "CH2": 3538944, "CH3": 4915200, "CH4": 3342336}  # measured zero values

SENSOR_POSITIONS = {
    "CH1": (176.662, 64.300),
    "CH2": (64.300, -176.662),
    "CH3": (-176.662,0  -64.300),
    "CH4": (-64.300, 176.662),
}

# OpenCV Constants
canvas_size = 400
center = (canvas_size // 2, canvas_size // 2)
radius_max = 180  # mm
scale = (canvas_size // 2 - 20) / radius_max  # mm to pixels
trail = deque(maxlen=10)

cv2.namedWindow("Weight Position (Polar View)", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Weight Position (Polar View)", 500, 500)

# Prepare static background (grid + cross lines)
def create_background():
    bg = np.ones((canvas_size, canvas_size, 3), dtype=np.uint8) * 255

    # Draw grid circles
    for r_mm in range(30, radius_max + 1, 30):
        r_px = int(r_mm * scale)
        cv2.circle(bg, center, r_px, (200, 200, 200), 1)

    # Draw cross lines
    cv2.line(bg, (center[0], 0), (center[0], canvas_size), (180, 180, 180), 1)
    cv2.line(bg, (0, center[1]), (canvas_size, center[1]), (180, 180, 180), 1)

    return bg

background_img = create_background()

def update_cv_polar(angle_deg=None, radius_mm=None, tilt_x=None, tilt_y=None):
    global trail
    img = background_img.copy()  # Use pre-rendered background

    # If valid position is provided
    if angle_deg is not None and radius_mm is not None:
        angle_rad = math.radians(angle_deg)
        x = int(center[0] + radius_mm * scale * math.cos(angle_rad))
        y = int(center[1] - radius_mm * scale * math.sin(angle_rad))
        trail.append((x, y))
    else:
        if trail:
            trail.popleft()

    # Draw trail (blue)
    for pt in list(trail):
        cv2.circle(img, pt, 3, (255, 0, 0), -1)

    # Draw current point (red) only if valid
    if angle_deg is not None and radius_mm is not None:
        cv2.circle(img, (x, y), 6, (0, 0, 255), -1)

    # Draw tilt vector (red arrow), if provided
    if tilt_x is not None and tilt_y is not None:
        # Convert to pixel vector
        dx = tilt_x * scale
        dy = -tilt_y * scale  # negative because OpenCV y-axis goes down
        tip = (int(center[0] + dx), int(center[1] + dy))
        cv2.arrowedLine(img, center, tip, (0, 0, 255), 2, tipLength=0.2)

    cv2.imshow("Weight Position (Polar View)", img)
    cv2.waitKey(1)

# Table Control Variables

P = 8.75  # mm
alpha_deg = 7.766
alpha_rad = np.radians(alpha_deg)
max_elevation_deg = 1.2
max_elevation_rad = np.radians(max_elevation_deg)
actuator_value_offset = 0.0
speed_rpm = 300


# filter_rate_elevation = 0.002
# filter_rate_azimuth = 0.5
filter_rate_elevation = 0.01
filter_rate_azimuth = 0.9
target_elevation = 0.0
target_azimuth = 0.0
current_elevation = 0.0
current_azimuth = 0.0


def actuator_length(theta_eff):
    return lookup_actuator_position(theta_eff)

def polar_to_tilt_components(elevation, azimuth):
    theta_x = np.arctan(np.tan(elevation) * np.cos(azimuth))
    theta_y = - np.arctan(np.tan(elevation) * np.sin(azimuth))
    return theta_x, theta_y

def limit_change(current, target, rate):
    delta = target - current
    if abs(delta) > rate:
        delta = rate * np.sign(delta)
    return current + delta


# Motion Control Variables 
prev_angle = None
prev_time = time.time()
angular_velocity_filtered = 0.0
velocity_alpha = 0.3  # filtering factor (0 = slow, 1 = no filtering)

MAX_TILT_DEG = 1.0      # max tilt angle in degrees
GAIN = 0.03           # maps deg/sec to tilt degrees
DEAD_ZONE = 10        # deg/s: don't respond below this speed

try:
    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print("Live Load Cell Readout (kg):\n")
        raw_values = read_all_channels_block()
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

            print(f"Angular velocity: {angular_velocity_filtered:.2f} deg/s")
        prev_angle = angle_from_x_axis

        # Skip control if not enough motion
        if abs(angular_velocity_filtered) < DEAD_ZONE:
            tilt_magnitude = 0
            tilt_azimuth = 0
        else:
            # Limit tilt angle to MAX_TILT_DEG
            tilt_magnitude = min(abs(angular_velocity_filtered) * GAIN, MAX_TILT_DEG)

            # 90 deg lead based on direction
            lead = 90 * math.copysign(1, angular_velocity_filtered)
            tilt_azimuth = (angle_from_x_axis + lead) % 360

        # Convert tilt azimuth to radians and calculate tilt vector components
        tilt_azimuth_rad = math.radians(tilt_azimuth)
        tilt_x = tilt_magnitude * math.cos(tilt_azimuth_rad)
        tilt_y = tilt_magnitude * math.sin(tilt_azimuth_rad)
        print(f"Tilt X: {tilt_x:8.3f} mm , Y: {tilt_y:8.3f} mm")

        # Set actuator target positions
        target_elevation = np.radians(tilt_magnitude)
        target_azimuth = np.radians(tilt_azimuth)

        # Apply rate limiting filter
        current_elevation = limit_change(current_elevation, target_elevation, filter_rate_elevation)
        current_azimuth = limit_change(current_azimuth, target_azimuth, filter_rate_azimuth)

        theta_x, theta_y = polar_to_tilt_components(current_elevation, current_azimuth)

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
            update_cv_polar(angle_from_x_axis, distance_from_center, tilt_x*50, tilt_y*50)
        else:
            print("No significant weight detected.")
            # Clear the display slowly
            update_cv_polar() 

        # Print update frequency
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

