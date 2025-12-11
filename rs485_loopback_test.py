import serial
import time

# Configure serial ports
sender_port = 'COM21'
receiver_port = 'COM22'
baudrate = 9600
timeout = 1  # seconds

# Message to send
message = b'Hello, RS485 loopback!\n'

# Open sender and receiver serial ports
with serial.Serial(sender_port, baudrate, timeout=timeout) as sender, \
    serial.Serial(receiver_port, baudrate, timeout=timeout) as receiver:

    # Flush input/output buffers
    sender.reset_input_buffer()
    sender.reset_output_buffer()
    receiver.reset_input_buffer()
    receiver.reset_output_buffer()

    # Send message
    sender.write(message)
    print(f"Sent: {message}")

    # Wait for data to arrive
    time.sleep(0.1)

    # Read message
    received = receiver.read(len(message))
    print(f"Received: {received}")

    # Check if received matches sent
    if received == message:
       print("Loopback test PASSED.")
    else:
       print("Loopback test FAILED.")