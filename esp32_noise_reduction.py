import serial
import time
import numpy as np
import sounddevice as sd
import wave
import struct


# =====================================================
# SETTINGS
# =====================================================

PORT = "COM5"

# Must match ESP32
BAUD = 460800

FS = 16000

RECORD_TIME = 6
NOISE_TIME = 2

FFT_SIZE = 1024
HOP_SIZE = 256


# =====================================================
# PROTOCOL
# =====================================================

HEADER = b"\xA5\x5A"

RESPONSE_HEADER = b"\x5A\xA5"

ACK = b"A"
READY = b"R"
FINISHED = b"F"
ERROR = b"E"


# =====================================================
# PRINT HEADER
# =====================================================

print()
print("================================================")
print(" ESP32 HIGH QUALITY SPEECH NOISE REDUCTION")
print("================================================")


# =====================================================
# CONNECT
# =====================================================

print()
print("Connecting to ESP32...")


esp = serial.Serial(
    PORT,
    BAUD,
    timeout=20,
    write_timeout=20
)


# Opening COM port resets ESP32
time.sleep(3)


esp.reset_input_buffer()
esp.reset_output_buffer()


print("ESP32 connected.")


# =====================================================
# SEND COMMAND
# =====================================================

def send_command(command):

    packet = (
        HEADER +
        bytes([ord(command)])
    )

    esp.write(packet)
    esp.flush()


# =====================================================
# SEND AUDIO PACKET
# =====================================================

def send_audio_packet(frame):

    data = frame.astype(
        np.int16
    ).tobytes()


    length = len(data)


    checksum = 0

    for byte in data:
        checksum ^= byte


    packet = (
        HEADER
        +
        struct.pack(
            "<H",
            length
        )
        +
        data
        +
        bytes([checksum])
    )


    esp.write(packet)
    esp.flush()


# =====================================================
# RECEIVE RESPONSE
# =====================================================

def receive_response():

    # Find response header
    while True:

        byte1 = esp.read(1)

        if not byte1:
            raise RuntimeError(
                "ESP32 response timeout."
            )


        if byte1 != RESPONSE_HEADER[:1]:
            continue


        byte2 = esp.read(1)

        if not byte2:
            raise RuntimeError(
                "ESP32 response timeout."
            )


        if byte2 == RESPONSE_HEADER[1:2]:
            break


    response = esp.read(1)


    if not response:
        raise RuntimeError(
            "ESP32 response incomplete."
        )


    return response


# =====================================================
# RECORD AUDIO
# =====================================================

print()
print("================================================")
print(" RECORDING")
print("================================================")

print()
print("FIRST 2 SECONDS : KEEP QUIET")
print("NEXT 4 SECONDS  : SPEAK")

input("\nPress ENTER to start...")


print()
print("Recording...")


audio = sd.rec(
    int(
        FS *
        RECORD_TIME
    ),
    samplerate=FS,
    channels=1,
    dtype="float32"
)


sd.wait()


audio = audio.flatten()


print("Recording completed.")


# =====================================================
# REMOVE DC FROM ORIGINAL
# =====================================================

audio -= np.mean(audio)


# =====================================================
# NORMALIZE
# =====================================================

peak = np.max(
    np.abs(audio)
)


if peak > 0:

    audio = (
        audio /
        peak *
        0.85
    )


# FLOAT → INT16

audio_int16 = (
    audio *
    32767
).astype(
    np.int16
)


# =====================================================
# NOISE AUDIO
# =====================================================

noise_samples = int(
    NOISE_TIME *
    FS
)


noise_audio = audio_int16[
    :noise_samples
]


# =====================================================
# NUMBER OF NOISE FRAMES
# =====================================================

noise_frames = (
    1
    +
    (
        len(noise_audio)
        -
        FFT_SIZE
    )
    //
    HOP_SIZE
)


print()
print(
    "Noise frames:",
    noise_frames
)


# =====================================================
# START NOISE ESTIMATION
# =====================================================

print()
print("Starting noise estimation...")


send_command("N")


# =====================================================
# SEND NOISE FRAMES
# =====================================================

for k in range(
    noise_frames
):

    start = (
        k *
        HOP_SIZE
    )


    end = (
        start +
        FFT_SIZE
    )


    frame = noise_audio[
        start:end
    ]


    # Send
    send_audio_packet(
        frame
    )


    # Wait for framed ACK
    response = receive_response()


    if response == ERROR:

        esp.close()

        raise RuntimeError(
            f"ESP32 reported an error "
            f"at noise frame "
            f"{k + 1}/{noise_frames}"
        )


    if response != ACK:

        esp.close()

        raise RuntimeError(
            f"Noise ACK failed at "
            f"{k + 1}/{noise_frames}. "
            f"Received={response!r}"
        )


    if (
        k == 0
        or
        (k + 1) % 10 == 0
        or
        k == noise_frames - 1
    ):

        print(
            f"Noise processing: "
            f"{k + 1}/{noise_frames}"
        )


# =====================================================
# WAIT FOR READY
# =====================================================

response = receive_response()


if response != READY:

    esp.close()

    raise RuntimeError(
        "ESP32 did not finish "
        "noise estimation. "
        f"Received={response!r}"
    )


print()
print("Noise estimation completed.")


# =====================================================
# PREPARE AUDIO FOR PROCESSING
# =====================================================

num_frames = int(
    np.ceil(
        (
            len(audio_int16)
            -
            FFT_SIZE
        )
        /
        HOP_SIZE
    )
) + 1


processed_length = (
    (
        num_frames -
        1
    )
    *
    HOP_SIZE
    +
    FFT_SIZE
)


padded = np.zeros(
    processed_length,
    dtype=np.int16
)


padded[
    :len(audio_int16)
] = audio_int16


print()
print("================================================")
print(" ESP32 HIGH QUALITY PROCESSING")
print("================================================")

print(
    "FFT size:",
    FFT_SIZE
)

print(
    "Hop size:",
    HOP_SIZE
)

print(
    "Frames:",
    num_frames
)


# =====================================================
# OUTPUT BUFFERS
# =====================================================

clean_audio = np.zeros(
    processed_length,
    dtype=np.float32
)


window_sum = np.zeros(
    processed_length,
    dtype=np.float32
)


# Hann window

window = np.hanning(
    FFT_SIZE
)


# =====================================================
# START PROCESSING
# =====================================================

send_command("P")


# Send frame count

esp.write(
    struct.pack(
        "<I",
        num_frames
    )
)

esp.flush()


# =====================================================
# PROCESS FRAMES
# =====================================================

for k in range(
    num_frames
):

    start = (
        k *
        HOP_SIZE
    )


    end = (
        start +
        FFT_SIZE
    )


    frame = padded[
        start:end
    ]


    # ---------------------------------------------
    # Send frame
    # ---------------------------------------------

    send_audio_packet(
        frame
    )


    # ---------------------------------------------
    # Receive processed frame
    # ---------------------------------------------

    # Find response audio packet header
    while True:

        b1 = esp.read(1)

        if not b1:

            esp.close()

            raise RuntimeError(
                "ESP32 stopped responding "
                f"at frame "
                f"{k + 1}/{num_frames}"
            )


        if b1 != HEADER[:1]:
            continue


        b2 = esp.read(1)

        if not b2:

            esp.close()

            raise RuntimeError(
                "ESP32 response incomplete."
            )


        if b2 == HEADER[1:2]:
            break


    # ---------------------------------------------
    # Receive payload length
    # ---------------------------------------------

    length_data = esp.read(2)


    if len(length_data) != 2:

        esp.close()

        raise RuntimeError(
            "ESP32 packet length missing."
        )


    payload_length = struct.unpack(
        "<H",
        length_data
    )[0]


    if payload_length != (
        FFT_SIZE * 2
    ):

        esp.close()

        raise RuntimeError(
            "Invalid processed audio "
            f"packet size: "
            f"{payload_length}"
        )


    # ---------------------------------------------
    # Receive audio
    # ---------------------------------------------

    data = bytearray()


    while len(data) < payload_length:

        chunk = esp.read(
            payload_length -
            len(data)
        )


        if not chunk:

            esp.close()

            raise RuntimeError(
                "ESP32 stopped sending "
                f"audio at frame "
                f"{k + 1}/{num_frames}"
            )


        data.extend(chunk)


    # ---------------------------------------------
    # Receive checksum
    # ---------------------------------------------

    checksum_data = esp.read(1)


    if len(checksum_data) != 1:

        esp.close()

        raise RuntimeError(
            "ESP32 checksum missing."
        )


    received_checksum = checksum_data[0]


    calculated_checksum = 0


    for byte in data:

        calculated_checksum ^= byte


    if (
        received_checksum
        !=
        calculated_checksum
    ):

        esp.close()

        raise RuntimeError(
            "Audio packet checksum "
            f"failed at frame "
            f"{k + 1}/{num_frames}"
        )


    # ---------------------------------------------
    # Convert audio
    # ---------------------------------------------

    output_frame = np.frombuffer(
        data,
        dtype=np.int16
    ).astype(
        np.float32
    )


    output_frame /= 32767.0


    # ---------------------------------------------
    # Overlap-add
    # ---------------------------------------------

    clean_audio[
        start:end
    ] += (
        output_frame *
        window
    )


    window_sum[
        start:end
    ] += (
        window ** 2
    )


    # ---------------------------------------------
    # Wait for processing ACK
    # ---------------------------------------------

    response = receive_response()


    if response == ERROR:

        esp.close()

        raise RuntimeError(
            f"ESP32 processing error "
            f"at frame "
            f"{k + 1}/{num_frames}"
        )


    if response != ACK:

        esp.close()

        raise RuntimeError(
            f"Processing ACK failed "
            f"at frame "
            f"{k + 1}/{num_frames}. "
            f"Received={response!r}"
        )


    if (
        k == 0
        or
        (k + 1) % 20 == 0
        or
        k == num_frames - 1
    ):

        print(
            f"Processing: "
            f"{k + 1}/{num_frames}"
        )


# =====================================================
# PROCESSING COMPLETE
# =====================================================

response = receive_response()


if response != FINISHED:

    esp.close()

    raise RuntimeError(
        "ESP32 did not report "
        "processing complete. "
        f"Received={response!r}"
    )


esp.close()


# =====================================================
# OVERLAP-ADD NORMALIZATION
# =====================================================

valid = (
    window_sum >
    1e-8
)


clean_audio[
    valid
] /= window_sum[
    valid
]


# Remove padding

clean_audio = clean_audio[
    :len(audio)
]


# =====================================================
# REMOVE DC
# =====================================================

clean_audio -= np.mean(
    clean_audio
)


# =====================================================
# OUTPUT NORMALIZATION
# =====================================================

peak = np.max(
    np.abs(clean_audio)
)


if peak > 0:

    clean_audio = (
        clean_audio /
        peak *
        0.85
    )


# =====================================================
# SAVE WAV
# =====================================================

output_int16 = (
    clean_audio *
    32767
).astype(
    np.int16
)


filename = (
    "ESP32_HighQuality_Clean_Voice.wav"
)


with wave.open(
    filename,
    "wb"
) as wav:

    wav.setnchannels(1)

    wav.setsampwidth(2)

    wav.setframerate(FS)

    wav.writeframes(
        output_int16.tobytes()
    )


# =====================================================
# AUDIO LEVELS
# =====================================================

input_rms = np.sqrt(
    np.mean(
        audio ** 2
    )
)


noise_float = (
    noise_audio.astype(
        np.float32
    )
    /
    32767.0
)


noise_rms = np.sqrt(
    np.mean(
        noise_float ** 2
    )
)


output_rms = np.sqrt(
    np.mean(
        clean_audio ** 2
    )
)


input_db = (
    20 *
    np.log10(
        input_rms +
        1e-12
    )
)


noise_db = (
    20 *
    np.log10(
        noise_rms +
        1e-12
    )
)


output_db = (
    20 *
    np.log10(
        output_rms +
        1e-12
    )
)


# =====================================================
# RESULTS
# =====================================================

print()
print("================================================")
print(" RESULTS")
print("================================================")

print(
    f"Input level  : "
    f"{input_db:.2f} dB"
)

print(
    f"Noise level  : "
    f"{noise_db:.2f} dB"
)

print(
    f"Output level : "
    f"{output_db:.2f} dB"
)

print()
print(
    "Saved:",
    filename
)


# =====================================================
# PLAY
# =====================================================

print()
print(
    "Playing cleaned audio..."
)


sd.play(
    clean_audio,
    FS
)


sd.wait()


print()
print("================================================")
print(" COMPLETE")
print("================================================")