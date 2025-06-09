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


def send_absolute_position_mm(ser, target_mm, address=1, direction=0x00, speed_rpm=200, acc=0x00):
    assert 1 <= address <= 4, "Motor address must be between 1 and 4"
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
    response = ser.read(4)

    if len(response) == 4 and response[0] == address and response[1] == 0xFD and response[3] == 0x6B:
        if response[2] == 0x02:
            print(f"✅ Motor {address}: Move command accepted.")
        elif response[2] == 0xE2:
            print(f"⚠️ Motor {address}: Condition not met (maybe not enabled or blocked).")
        else:
            print(f"❌ Motor {address}: Unknown response status: {response[2]:02X}")
    else:
        print(f"❌ Motor {address}: Invalid or no response:", response)



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