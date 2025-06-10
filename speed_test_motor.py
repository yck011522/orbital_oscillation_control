import time

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


# Serial connection for motor control
ser = open_serial()
if not home_all_motors(ser, settle_position_mm=28.1):
    print("Homing failed. Exiting.")
    exit()

# Set actuator positions
speed_rpm = 200
target = 28.1
wait_for_ack = False # Change this setting on the motor too.

t0 = time.time()
send_absolute_position_mm(ser, 28.1, 1, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
send_absolute_position_mm(ser, 28.1, 2, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
send_absolute_position_mm(ser, 28.1, 3, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
send_absolute_position_mm(ser, 28.1, 4, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
t1 = time.time()


attempts = 100
start_position = 10.0
end_position = 28.1
for i in range(attempts):
    target = (end_position - start_position) * i / (attempts - 1) + start_position
    send_absolute_position_mm(ser, target, 1, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
    send_absolute_position_mm(ser, target, 2, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
    send_absolute_position_mm(ser, target, 3, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
    send_absolute_position_mm(ser, target, 4, speed_rpm=speed_rpm, wait_for_ack=wait_for_ack)
t2 = time.time()


print(f"Set four actuator positions: {t1-t0:.5f}s")
print(f"Set four actuator positions {attempts} attempts average: {(t2-t1)/attempts:.5f}s per attempt. Freq: {attempts / (t2-t1):.2f} Hz")

"""
Set four actuator positions: 0.01166s
Set four actuator positions 100 attempts average: 0.01049s per attempt. Freq: 95.31 Hz
"""