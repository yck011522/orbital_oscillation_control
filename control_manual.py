import numpy as np
import cv2
from actuator_ik_table05 import create_actuator_lookup_function
lookup_actuator_position = create_actuator_lookup_function()
from motor_control import send_absolute_position_mm
from motor_control import read_position_mm
from motor_control import open_serial
from motor_control import home_all_motors

# Constants
P = 8.75  # distance from roller to ramp (in mm)
alpha_deg = 7.766
alpha_rad = np.radians(alpha_deg)
max_elevation_deg = 1.2 # 0.6 would be nice but 1.2 is easier to control
max_elevation_rad = np.radians(max_elevation_deg)
canvas_size = 600
center = canvas_size // 2
radius = canvas_size // 2 - 20
filter_rate_elevation = 0.002  # max radians change per frame
filter_rate_azimuth = 0.5  # max radians change per frame

# Initial states
target_elevation = 0.0
target_azimuth = 0.0
current_elevation = 0.0
current_azimuth = 0.0

speed_rpm = 200 # 180 would be quite quiet. 500 is the max speed of the motors

# tuning
actuator_value_offset = 0.0

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

# Mouse callback
def mouse_event(event, x, y, flags, param):
    global target_elevation, target_azimuth
    if event == cv2.EVENT_LBUTTONDOWN or (flags & cv2.EVENT_FLAG_LBUTTON):
        dx = x - center
        dy = y - center
        r = np.hypot(dx, dy)
        if r <= radius:
            angle = np.arctan2(dy, dx)
            mag = min(r / radius, 1.0)
            target_elevation = mag * max_elevation_rad
            target_azimuth = angle

# Create OpenCV window
cv2.namedWindow("Tilt Control")
cv2.setMouseCallback("Tilt Control", mouse_event)


with open_serial() as ser:
    homing_success = home_all_motors(ser, settle_position_mm=28.1)
    if not homing_success:
        print("Homing failed. Exiting.")
        exit()

    while True:
        # Apply rate limiting filter
        current_elevation = limit_change(current_elevation, target_elevation, filter_rate_elevation)
        current_azimuth = limit_change(current_azimuth, target_azimuth, filter_rate_azimuth)

        # Compute tilt components
        theta_x, theta_y = polar_to_tilt_components(current_elevation, current_azimuth)

        # Compute actuator lengths
        L_x_plus = actuator_length(theta_x) + actuator_value_offset
        L_x_minus = actuator_length(-theta_x) + actuator_value_offset
        L_y_plus = actuator_length(theta_y) + actuator_value_offset
        L_y_minus = actuator_length(-theta_y) + actuator_value_offset

        # Send actuator positions
        
        send_absolute_position_mm(ser, L_x_plus, 1, speed_rpm=speed_rpm)
        send_absolute_position_mm(ser, L_y_minus, 2, speed_rpm=speed_rpm)
        send_absolute_position_mm(ser, L_x_minus, 3, speed_rpm=speed_rpm)
        send_absolute_position_mm(ser, L_y_plus, 4, speed_rpm=speed_rpm)

        # Create canvas
        canvas = np.ones((canvas_size, canvas_size, 3), dtype=np.uint8) * 255

        # Draw polar grid
        cv2.circle(canvas, (center, center), radius, (200, 200, 200), 1)
        for r in range(1, 4):
            cv2.circle(canvas, (center, center), r * radius // 4, (230, 230, 230), 1)
        for a in range(0, 360, 30):
            rad = np.radians(a)
            x1 = int(center + np.cos(rad) * radius)
            y1 = int(center + np.sin(rad) * radius)
            cv2.line(canvas, (center, center), (x1, y1), (230, 230, 230), 1)

        # Draw current input dot
        rx = int(center + np.cos(current_azimuth) * (current_elevation / max_elevation_rad) * radius)
        ry = int(center + np.sin(current_azimuth) * (current_elevation / max_elevation_rad) * radius)
        cv2.circle(canvas, (rx, ry), 5, (0, 0, 255), -1)

        # Draw current and target values
        display_text = [
            f"Target Elevation: {np.degrees(target_elevation):.2f}°",
            f"Current Elevation: {np.degrees(current_elevation):.2f}°",
            f"Target Azimuth: {np.degrees(target_azimuth):.2f}°",
            f"Current Azimuth: {np.degrees(current_azimuth):.2f}°"
        ]

        for i, txt in enumerate(display_text):
            cv2.putText(canvas, txt, (10, canvas_size - 80 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Display actuator values
        text_y = 20
        for label, value in [
            ("L_x_plus", L_x_plus), ("L_x_minus", L_x_minus),
            ("L_y_plus", L_y_plus), ("L_y_minus", L_y_minus)
        ]:
            cv2.putText(canvas, f"{label}: {value:.2f} mm", (10, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            text_y += 20

        cv2.imshow("Tilt Control", canvas)
        key = cv2.waitKey(30)
        if key == 27:  # ESC to quit
            break

    send_absolute_position_mm(ser, 21.5, 0, speed_rpm=200)

    cv2.destroyAllWindows()
