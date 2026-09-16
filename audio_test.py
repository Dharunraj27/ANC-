import sounddevice as sd
import numpy as np

FS = 16000
DURATION = 5

print("Recording...")
audio = sd.rec(
    int(FS * DURATION),
    samplerate=FS,
    channels=1,
    dtype="float32"
)

sd.wait()

print("Recording complete.")
print("Playing...")

sd.play(audio, FS)
sd.wait()

print("Done.")