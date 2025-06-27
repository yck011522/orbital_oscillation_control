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


# Motion Control code 
last_time = time.time()
prev_time = time.time()
freq = 0.0
freq_alpha = 0.9 # 0.0 is no smoothing

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
        else:
            print("No significant weight detected.")
            print(weights)

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

