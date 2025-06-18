import cv2
import numpy as np
from collections import deque
import math

import time
import os
import math


from sensor_utils import (
    find_modbus_device,
    read_all_channels_weights_kg,
    read_cop_weight_position_angle_distance,
)

client = find_modbus_device()

if not client:
    print("Modbus client not initialized.")
    exit(1)
    

dp_raw = client.read_holding_registers(address=3, count=1, slave=1)
decimal_places = dp_raw.registers[0]
print(f"Decimal places: {decimal_places}")

unit_raw = client.read_holding_registers(address=14, count=1, slave=1)
unit_mode = unit_raw.registers[0]
print(f"Unit mode: {unit_mode}")


client.connect()


# Constants
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

def update_cv_polar(angle_deg=None, radius_mm=None):
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

    cv2.imshow("Weight Position (Polar View)", img)
    cv2.waitKey(1)

# Motion Control code 
last_time = time.time()
prev_time = time.time()
freq = 0.0
freq_alpha = 0.8 # 0.0 is no smoothing

try:
    while True:
        # os.system("cls" if os.name == "nt" else "clear")
        print("Live Load Cell Readout (kg):\n")
        weights = read_all_channels_weights_kg(client)
        total_weight, cop_position, angle_degrees, distance = read_cop_weight_position_angle_distance(weights)
        print(f"\nTotal: {total_weight:8.3f} kg")

        if total_weight > 0.1:  # Ignore near-zero weight

            print(f"Position X: {cop_position[0]:8.3f} mm , Y: {cop_position[1]:8.3f} mm")
            print(f"Distance from center: {distance:8.3f} mm , Angle: {angle_degrees:8.3f} degrees")
            update_cv_polar(angle_degrees, distance) # Update the display with the current position
        else:
            print("No significant weight detected.")
            update_cv_polar() # Clear the display slowly

        # Print update frequency
        now = time.time()
        dt = now - last_time
        if dt > 0:
            freq = freq * (1- freq_alpha) + (1.0 / dt) * freq_alpha
            print(f"Update frequency: {freq:.2f} Hz")
        last_time = now

        time.sleep(0.001)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    client.close()
    cv2.destroyAllWindows()

