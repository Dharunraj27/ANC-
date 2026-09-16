import serial
import time
import numpy as np

PORT = "COM5"
BAUD = 921600
FRAME_SIZE = 512

print("Connecting to ESP32...")

esp = serial.Serial(
    PORT,
    BAUD,
    timeout=5,
    write_timeout=5
)

time.sleep(2)

esp.reset_input_buffer()
esp.reset_output_buffer()

print("ESP32 connected.")

# Send command
print("Sending N...")
esp.write(b"N")

# Create one test audio frame
frame = np.zeros(FRAME_SIZE, dtype=np.int16)

print("Sending 512 samples...")
esp.write(frame.tobytes())

print("Waiting for ESP32 ACK...")

ack = esp.read(1)

if ack == b"A":
    print()
    print("==============================")
    print("SUCCESS!")
    print("ESP32 received the audio frame.")
    print("==============================")
else:
    print()
    print("==============================")
    print("FAILED")
    print("Received:", repr(ack))
    print("==============================")

esp.close()