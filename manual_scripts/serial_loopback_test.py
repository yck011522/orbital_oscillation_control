import serial

ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)
ser.write(b'hello\n')
response = ser.readline()
print("Received:", response)
ser.close()