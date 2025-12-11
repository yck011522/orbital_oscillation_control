import time
from sensor_utils import (
    find_modbus_device,
    read_all_channels_block,
    channels,
    SLOPE,
    TARE,
    SENSOR_POSITIONS,
)
t0 = time.time()
client = find_modbus_device()
if not client:
    raise RuntimeError("No Modbus device found. Exiting.")
t1 = time.time()
raw_values = read_all_channels_block(client)
t2 = time.time()

attempts = 1000
for i in range(attempts):
    raw_values = read_all_channels_block(client)

t3 = time.time()
print(f"Connected to Modbus device: {t1-t0:.5f}s")
print(f"Read all channels: {t2-t1:.5f}s")
print(f"Read {attempts} attempts average: {(t3-t2)/attempts:.5f}s per attempt. Freq {attempts / (t3-t2):.2f} Hz")

"""
Connected to Modbus device: 0.04663s
Read all channels: 0.01353s
Read 1000 attempts average: 0.01376s per attempt. Freq 72.69 Hz
"""