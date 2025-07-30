
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
    validate_full_travel,
)


# Serial connection for motor control
ser = open_serial()
if not home_all_motors(ser, settle_position_mm=28.1, skip_if_homed=False):
    print("Homing failed. Exiting.")
    exit()

# Validate travel
validate_full_travel(ser, 0.9)
