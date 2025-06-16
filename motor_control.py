import serial
import time

# === Mechanical Constants ===
MICROSTEPS = 16
STEPS_PER_REV = 200 * MICROSTEPS
LEADSCREW_PITCH_MM = 2.0
STEPS_PER_MM = STEPS_PER_REV / LEADSCREW_PITCH_MM  # = 1600

# === Serial Settings ===
SERIAL_PORT = 'COM19'
BAUDRATE = 115200


def open_serial():
    return serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)


def send_absolute_position_mm(ser, target_mm, address=1, direction=0x00, speed_rpm=200, acc=0x00, wait_for_ack=False):
    assert 0 <= address <= 4, "Motor address must be between 1 and 4"
    target_steps = int(round(target_mm * STEPS_PER_MM))
    speed_hi = (speed_rpm >> 8) & 0xFF
    speed_lo = speed_rpm & 0xFF
    pulse_bytes = target_steps.to_bytes(4, byteorder='big', signed=False)

    cmd = [
        address,     # motor ID
        0xFD,        # position control command
        direction,
        speed_hi, speed_lo,
        acc,
        *pulse_bytes,
        0x01,        # absolute mode
        0x00,        # no sync
        0x6B         # fixed checksum
    ]

    ser.write(bytes(cmd))
    if wait_for_ack:
        response = ser.read(4)
        if len(response) == 4 and response[0] == address and response[1] == 0xFD and response[3] == 0x6B:
            if response[2] == 0x02:
                # print(f"✅ Motor {address}: Move command accepted to {target_mm:.2f} mm.")
                pass
            elif response[2] == 0xE2:
                print(f"⚠️ Motor {address}: Condition not met (maybe not enabled or blocked).")
            else:
                print(f"❌ Motor {address}: Unknown response status: {response[2]:02X}")
        else:
            print(f"❌ Motor {address}: Invalid or no response:", response)
    else:
        # delay for modbus command spacing
        time.sleep(0.002) 


def read_position_mm(ser, address=1):
    assert 1 <= address <= 4, "Motor address must be between 1 and 4"

    ser.write(bytes([address, 0x36, 0x6B]))  # Request current position
    response = ser.read(8)

    if len(response) != 8 or response[0] != address or response[1] != 0x36 or response[-1] != 0x6B:
        print(f"❌ Motor {address}: Invalid position response:", response)
        return None

    sign = -1 if response[2] == 0x01 else 1
    raw_val = int.from_bytes(response[3:7], byteorder='big', signed=False)

    steps = sign * (raw_val * STEPS_PER_REV / 65536)
    mm = steps / STEPS_PER_MM
    return mm

def motor_needs_homing(ser, motor_id, threshold_mm=0.05):
    pos = read_position_mm(ser, address=motor_id)
    time.sleep(0.1) # Allow time for serial response
    if pos is None:
        print(f"Motor {motor_id}: unable to read position. Assuming it needs homing.")
        raise RuntimeError(f"Motor {motor_id}: failed to read position.")
        # return True
    if abs(pos) < threshold_mm:
        print(f"Motor {motor_id}: position is {pos:.2f} mm — assumed unhomed.")
        return True
    print(f"Motor {motor_id}: position is {pos:.2f} mm — assumed already homed.")
    return False



def home_all_motors(ser, home_speed_rpm=200, settle_position_mm=21.0, delay_after_move=1.0):
    """
    Homes all four motors one by one and moves each to the predefined zero position.
    """
    for motor_id in range(1, 5):
        if not motor_needs_homing(ser, motor_id):
            print(f"Motor {motor_id}: already homed. Skipping.")
            continue

        print(f"Motor {motor_id}: requires homing. Starting...")

        # Trigger homing: 0x9A + [mode] + [sync] + 0x6B
        # Mode 00 = single-turn nearest zero
        # Trigger senseless homing: 0x9A + mode 0x02 + sync 0x00 + checksum 0x6B
        cmd = [motor_id, 0x9A, 0x02, 0x00, 0x6B]
        ser.write(bytes(cmd))
        # response = ser.read(4)

        # if len(response) == 4 and response[0] == motor_id and response[1] == 0x9A and response[3] == 0x6B:
        #     if response[2] == 0x02:
        #         print(f"Motor {motor_id} homing started.")
        #     elif response[2] == 0xE2:
        #         print(f"Motor {motor_id} failed to start homing: condition not met.")
        #         return False
        #     else:
        #         print(f"Motor {motor_id} returned unknown status: {response[2]:02X}")
        #         return False
        # else:
        #     print(f"Motor {motor_id} no or invalid response:", response)
        #     return False

        # Wait for motor to report it's done homing
        timeout = 30.0
        poll_interval = 0.1
        elapsed = 0.0

        while elapsed < timeout:
            ser.write(bytes([motor_id, 0x3B, 0x6B]))  # read homing status
            status_response = ser.read(4)
            if len(status_response) == 4 and status_response[0] == motor_id and status_response[1] == 0x3B:
                flags = status_response[2]
                homing_done = (flags & 0x04) == 0 and (flags & 0x08) == 0
                if homing_done:
                    print(f"Motor {motor_id} finished homing.")
                    break
            time.sleep(poll_interval)
            elapsed += poll_interval
        else:
            print(f"Motor {motor_id} did not complete homing in time.")
            return False

        # Move to predefined settle position after homing
        time.sleep(delay_after_move)
        send_absolute_position_mm(ser, target_mm=settle_position_mm, address=motor_id, speed_rpm=home_speed_rpm)
        time.sleep(delay_after_move)
    
    return True


if __name__ == "__main__":

    with open_serial() as ser:
        home_all_motors(ser)
