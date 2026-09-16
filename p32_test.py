import serial
import time

PORT = "COM5"       # CHANGE THIS
BAUD = 115200
print("Connecting to ESP32...")

esp = serial.Serial(
    PORT,
    BAUD,
    timeout=2
)

time.sleep(2)

print("Connected.")

esp.reset_input_buffer()

esp.write(b'H')

time.sleep(1)

response = esp.readline().decode(
    errors="ignore"
).strip()

print("ESP32 response:")
print(response)

esp.close()