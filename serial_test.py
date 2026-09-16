import serial
import time

PORT = "COM5"
BAUD = 115200

print("Opening ESP32...")

esp = serial.Serial(
    PORT,
    BAUD,
    timeout=1
)

# Opening the serial port can reset ESP32.
time.sleep(4)

# Remove all old boot/reset messages
esp.reset_input_buffer()

print("Sending H...")

esp.write(b"H")
esp.flush()

time.sleep(1)

print("Reading response...")

data = esp.read_all()

print("Received:", repr(data))

if b"HELLO_FROM_ESP32" in data:
    print()
    print("==============================")
    print("SUCCESS!")
    print("ESP32 SERIAL COMMUNICATION OK")
    print("==============================")
else:
    print()
    print("==============================")
    print("NO CORRECT RESPONSE")
    print("==============================")

esp.close()