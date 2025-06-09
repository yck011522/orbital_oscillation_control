import numpy as np
from actuator_ik_table05 import create_actuator_lookup_function
lookup_actuator_position = create_actuator_lookup_function()

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

def send_actuator_positions(ser, positions, speed_rpm, send_func):
    for idx, pos in enumerate(positions, 1):
        send_func(ser, pos, idx, speed_rpm=speed_rpm)