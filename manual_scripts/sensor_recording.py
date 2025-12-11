import csv
import cv2
import numpy as np
from collections import deque
import math
import time
import os

from sensor_utils import (
    find_modbus_device,
    read_all_channels_weights_kg,
    read_cop_weight_position_angle_distance
)

client = find_modbus_device()
if not client:
    print("Modbus client not initialized.")
    exit(1)

start_time = time.time()
time_history = deque(maxlen=20)
current_time = start_time
total_record_time = 60.0
file_name = "sensor_recording/glass04.csv"

# Store data in memory: each entry is a dict or list
data_log = []

while current_time - start_time < total_record_time:
    current_time = time.time()
    time_history.append(current_time)
    
    weights = read_all_channels_weights_kg(client)
    total_weight, cop_position, angle_degrees, distance = read_cop_weight_position_angle_distance(weights)
    
    fps_average = len(time_history) / (current_time - time_history[0]) if len(time_history) > 1 else 0
    
    # Save the data
    data_log.append([
        current_time - start_time,
        weights['CH1'],
        weights['CH2'],
        weights['CH3'],
        weights['CH4'],
        total_weight,
        cop_position[0],
        cop_position[1],
        angle_degrees,
        distance
    ])
    
    print(f"Time: {current_time - start_time:.3f} s, Weights: {weights}, FPS: {fps_average:.3f}")
    print(f"Total Weight: {total_weight} kg, COP Position: {cop_position}, Angle: {angle_degrees} degrees, Distance: {distance} m")
    
    # os.system("cls" if os.name == "nt" else "clear")

# Save to CSV after loop

with open(file_name, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        "Time (s)", 
        "Weight1 (kg)", "Weight2 (kg)", "Weight3 (kg)", "Weight4 (kg)",
        "TotalWeight (kg)", "COP_X (m)", "COP_Y (m)",
        "Angle (deg)", "Distance (m)"
    ])
    writer.writerows(data_log)

print(f"Data saved to {file_name}")
