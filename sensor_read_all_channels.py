import time
import os
import math
from pymodbus.client import ModbusSerialClient
from pymodbus.client.mixin import ModbusClientMixin

client = ModbusSerialClient(
    port="COM18", baudrate=38400, bytesize=8, parity="N", stopbits=1, timeout=1
)

dp_raw = client.read_holding_registers(address=3, count=1, slave=1)
decimal_places = dp_raw.registers[0]
print(f"Decimal places: {decimal_places}")

unit_raw = client.read_holding_registers(address=14, count=1, slave=1)
unit_mode = unit_raw.registers[0]
print(f"Unit mode: {unit_mode}")


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

client.connect()

# Slope unit is in gramForce per raw value count    
SLOPE = {"CH1": 0.000160217269343611, "CH2": 0.00015715760370146, "CH3": 0.000158163217397836, "CH4": 0.000160217269343611}  # measured zero values
TARE = {"CH1": 4390912, "CH2": 3538944, "CH3": 4915200, "CH4": 3342336}  # measured zero values

SENSOR_DISTANCE_FROM_CENTER = 189 # mm
try:
    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print("Live Load Cell Readout (kg):\n")
        raw_values = read_all_channels_block()
        weights = {}
        if raw_values is None:
            print("Modbus read error.")
        else:
            for ch_name, raw in zip(channels.keys(), raw_values):
                zero = TARE.get(ch_name, 0)
                slope = SLOPE.get(ch_name, 0)
                kg = (raw - zero) * slope * 0.001
                print(f"{ch_name}: {kg:8.3f} kg (raw: {raw})")
                weights[ch_name] = kg

        # Calculate total weight
        total = sum(weights.values())
        print(f"\nTotal: {total:8.3f} kg") 
        time.sleep(0.01)

        # Compute weight location
        position_x = SENSOR_DISTANCE_FROM_CENTER * (weights["CH2"] - weights["CH4"]) / total
        position_y = SENSOR_DISTANCE_FROM_CENTER * (weights["CH1"] - weights["CH3"]) / total
        distance_from_center = (position_x**2 + position_y**2) ** 0.5
        angle_from_x_axis = math.atan2(position_y, position_x) * 180 / math.pi
        print(f"Position X: {position_x:8.3f} mm , Y: {position_y:8.3f} mm")
        print(f"Distance from center: {distance_from_center:8.3f} mm , Angle: {angle_from_x_axis:8.3f} degrees")

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    client.close()
