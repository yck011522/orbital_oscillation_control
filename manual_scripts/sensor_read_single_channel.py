import time
import os
from pymodbus.client import ModbusSerialClient
from pymodbus.client.mixin import ModbusClientMixin

client = ModbusSerialClient(
    port="COM7", baudrate=38400, bytesize=8, parity="N", stopbits=1, timeout=1
)

dp_raw = client.read_holding_registers(address=3, count=1, slave=1)
decimal_places = dp_raw.registers[0]
print(f"Decimal places: {decimal_places}")

unit_raw = client.read_holding_registers(address=14, count=1, slave=1)
unit_mode = unit_raw.registers[0]
print(f"Unit mode: {unit_mode}")


def read_channel(register_address):
    result = client.read_holding_registers(address=register_address, count=2, slave=1)
    if result.isError():
        return None
    return client.convert_from_registers(
        registers=result.registers,
        data_type=ModbusClientMixin.DATATYPE.INT32,
        word_order="big",
    )


# Channel register map (40301–40308 => offsets 300–306)
channels = {
    "CH1": 300,
    "CH2": 302,
    "CH3": 304,
    "CH4": 306,
}

client.connect()

try:
    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print("Live Load Cell Readout (Raw ADC Values):\n")
        for name, addr in channels.items():
            value = read_channel(addr)
            if value is not None:
                print(f"{name}: {value:>10}")
            else:
                print(f"{name}: Error")
        time.sleep(0.01)  # 10Hz update rate
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    client.close()
