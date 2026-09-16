import serial
import time
import numpy as np
import sounddevice as sd

# ==============================
# SETTINGS
# ==============================

PORT = "COM5"
BAUD = 115200

FS = 16000
FRAME_SIZE = 512

# ==============================
# CONNECT ESP32
# ==============================

print("Connecting to ESP32...")

esp = serial.Serial(
    PORT,
    BAUD,
    timeout=5,
    write_timeout=5
)

# Opening COM port may reset ESP32
time.sleep(3)

esp.reset_input_buffer()

print("ESP32 connected.")

# ==============================
# RECORD ONE FRAME
# ==============================

print()
print("==============================")
print("MICROPHONE TEST")
print("==============================")

print()
print("Get ready...")

time.sleep(1)

print("Speak now!")

audio = sd.rec(
    FRAME_SIZE,
    samplerate=FS,
    channels=1,
    dtype="float32"
)

sd.wait()

audio = audio.flatten()

print("Recording completed.")

# ==============================
# CONVERT TO INT16
# ==============================

maximum = np.max(np.abs(audio))

if maximum > 0:
    audio = audio / maximum * 0.8

audio_int16 = (audio * 32767).astype(np.int16)

# ==============================
# SEND TO ESP32
# ==============================

print("Sending audio frame to ESP32...")

esp.write(b"A")
esp.flush()

esp.write(audio_int16.tobytes())
esp.flush()

print("Waiting for ESP32...")

response = esp.read(1)

# ==============================
# RESULT
# ==============================

if response == b"K":

    print()
    print("==============================")
    print("SUCCESS!")
    print("==============================")
    print("Laptop microphone")
    print("       ↓")
    print("Python")
    print("       ↓")
    print("ESP32")
    print()
    print("ESP32 successfully received")
    print("the microphone audio frame.")

else:

    print()
    print("==============================")
    print("FAILED")
    print("==============================")
    print("Received:", repr(response))

esp.close()