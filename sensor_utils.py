import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient
from pymodbus.client.mixin import ModbusClientMixin

channels = {
    "CH1": 300,
    "CH2": 302,
    "CH3": 304,
    "CH4": 306,
}

SLOPE = {
    "CH1": 0.000160217269343611,
    "CH2": 0.00015715760370146,
    "CH3": 0.000158163217397836,
    "CH4": 0.000160217269343611,
}

TARE = {
    "CH1": 4390912,
    "CH2": 3538944,
    "CH3": 4915200,
    "CH4": 3342336,
}

SENSOR_POSITIONS = {
    "CH1": (176.662, 64.300),
    "CH2": (64.300, -176.662),
    "CH3": (-176.662, -64.300),
    "CH4": (-64.300, 176.662),
}

def list_serial_ports():
    return [port.device for port in serial.tools.list_ports.comports()]

def find_modbus_device(baudrate=38400, test_slave_id=1, test_address=3, test_count=1):
    for port in list_serial_ports():
        try:
            client = ModbusSerialClient(
                port=port,
                baudrate=baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1,
            )
            if client.connect():
                result = client.read_holding_registers(address=test_address, count=test_count, slave=test_slave_id)
                if result and not result.isError():

                    print(f"✅ Found device on {port}")
                    dp_raw = client.read_holding_registers(address=3, count=1, slave=1)
                    decimal_places = dp_raw.registers[0]
                    print(f"Decimal places: {decimal_places}")

                    unit_raw = client.read_holding_registers(address=14, count=1, slave=1)
                    unit_mode = unit_raw.registers[0]
                    print(f"Unit mode: {unit_mode}")
                    
                    return client
                client.close()
        except Exception as e:
            print(f"⚠️ Failed on {port}: {e}")
    print("❌ No Modbus device found.")  

    return None

def read_all_channels_block(client):
    result = client.read_holding_registers(address=300, count=8, slave=1)
    if result.isError():
        return None

    raw_values = []
    for i in range(0, 8, 2):
        val = client.convert_from_registers(
            registers=result.registers[i : i + 2],
            data_type=ModbusClientMixin.DATATYPE.INT32,
            word_order="big",
        )
        raw_values.append(val)

    return raw_values