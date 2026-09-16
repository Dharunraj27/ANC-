"""
================================================================
 ESP32 AUDIO NOISE REDUCTION
================================================================

 Serial:
     COM3
     921600 baud

 Audio:
     16 kHz
     16-bit
     Mono

 DSP:
     FFT = 512
     Hop = 256

 Protocol:

 HOST -> ESP32

     H = handshake
     N = noise start
     n = noise data
     P = process start
     p = process data

 ESP32 -> HOST

     R = ready / ACK
     O = processed output
     D = done
     E = error

================================================================
"""

import os
import time
import struct

import numpy as np
import serial
import sounddevice as sd

from scipy.io.wavfile import write


# ================================================================
# CONFIGURATION
# ================================================================

PORT = "COM3"

BAUD = 921600

FS = 16000

FFT_SIZE = 512

HOP_SIZE = 256

RECORD_SECONDS = 6

NOISE_SECONDS = 2


# ================================================================
# PACKET TYPES
# ================================================================

PKT_HANDSHAKE = ord("H")

PKT_NOISE_START = ord("N")

PKT_NOISE_DATA = ord("n")

PKT_PROCESS_START = ord("P")

PKT_PROCESS_DATA = ord("p")


RESP_READY = ord("R")

RESP_OUTPUT = ord("O")

RESP_DONE = ord("D")

RESP_ERROR = ord("E")


# ================================================================
# SERIAL
# ================================================================

MAX_PAYLOAD = 1024

SERIAL_TIMEOUT = 3.0


# ================================================================
# CHECKSUM
# ================================================================

def calculate_checksum(
    packet_type,
    sequence,
    length,
    payload
):

    checksum = 0

    checksum ^= packet_type

    checksum ^= (
        sequence &
        0xFF
    )

    checksum ^= (
        (sequence >> 8) &
        0xFF
    )

    checksum ^= (
        length &
        0xFF
    )

    checksum ^= (
        (length >> 8) &
        0xFF
    )

    for value in payload:

        checksum ^= int(value)

    return checksum


# ================================================================
# SEND PACKET
# ================================================================

def send_packet(
    ser,
    packet_type,
    sequence,
    payload=b""
):

    payload = bytes(payload)

    length = len(payload)

    if length > MAX_PAYLOAD:

        raise ValueError(
            f"Payload too large: {length}"
        )


    checksum = calculate_checksum(
        packet_type,
        sequence,
        length,
        payload
    )


    header = struct.pack(
        "<2sBHH",
        b"\xAA\x55",
        packet_type,
        sequence,
        length
    )


    packet = (
        header +
        payload +
        bytes([checksum])
    )


    ser.write(packet)


# ================================================================
# READ EXACT BYTES
# ================================================================

def read_exact(
    ser,
    count,
    timeout
):

    data = bytearray()

    start = time.perf_counter()


    while len(data) < count:

        elapsed = (
            time.perf_counter() -
            start
        )

        if elapsed >= timeout:

            raise TimeoutError(
                f"Serial timeout. "
                f"Expected {count} bytes, "
                f"received {len(data)}."
            )


        chunk = ser.read(
            count - len(data)
        )


        if chunk:

            data.extend(chunk)


    return bytes(data)


# ================================================================
# FIND HEADER
# ================================================================

def find_header(
    ser,
    timeout
):

    start = time.perf_counter()

    state = 0


    while (
        time.perf_counter() -
        start
    ) < timeout:

        byte = ser.read(1)


        if not byte:

            continue


        value = byte[0]


        if state == 0:

            if value == 0xAA:

                state = 1


        else:

            if value == 0x55:

                return

            elif value == 0xAA:

                state = 1

            else:

                state = 0


    raise TimeoutError(
        "ESP32 response header timeout."
    )


# ================================================================
# RECEIVE PACKET
# ================================================================

def receive_packet(
    ser,
    timeout=SERIAL_TIMEOUT
):

    find_header(
        ser,
        timeout
    )


    header = read_exact(
        ser,
        5,
        timeout
    )


    packet_type = header[0]


    sequence = struct.unpack_from(
        "<H",
        header,
        1
    )[0]


    length = struct.unpack_from(
        "<H",
        header,
        3
    )[0]


    if length > MAX_PAYLOAD:

        raise RuntimeError(
            f"Invalid ESP32 payload "
            f"length: {length}"
        )


    payload = read_exact(
        ser,
        length,
        timeout
    )


    received_checksum = read_exact(
        ser,
        1,
        timeout
    )[0]


    calculated_checksum = (
        calculate_checksum(
            packet_type,
            sequence,
            length,
            payload
        )
    )


    if (
        received_checksum !=
        calculated_checksum
    ):

        raise RuntimeError(
            "Checksum error from ESP32."
        )


    return (
        packet_type,
        sequence,
        payload
    )


# ================================================================
# WAIT FOR RESPONSE
# ================================================================

def wait_for_response(
    ser,
    expected_type,
    expected_sequence=None,
    timeout=SERIAL_TIMEOUT
):

    packet_type, sequence, payload = (
        receive_packet(
            ser,
            timeout
        )
    )


    if packet_type == RESP_ERROR:

        raise RuntimeError(
            f"ESP32 returned ERROR "
            f"(sequence={sequence})"
        )


    if packet_type != expected_type:

        raise RuntimeError(
            f"Unexpected ESP32 response: "
            f"{chr(packet_type)} "
            f"expected "
            f"{chr(expected_type)}"
        )


    if (
        expected_sequence is not None and
        sequence != expected_sequence
    ):

        raise RuntimeError(
            f"Sequence mismatch: "
            f"received {sequence}, "
            f"expected {expected_sequence}"
        )


    return (
        sequence,
        payload
    )


# ================================================================
# CONNECT ESP32
# ================================================================

def connect_esp32():

    print()
    print("=" * 65)
    print("                       ESP32 CONNECTION")
    print("=" * 65)
    print()


    print(
        f"Connecting to {PORT} "
        f"at {BAUD} baud..."
    )


    try:

        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            timeout=0.1,
            write_timeout=3
        )


    except Exception as e:

        raise RuntimeError(
            f"Could not open {PORT}.\n"
            f"Check that:\n"
            f"1. ESP32 is connected\n"
            f"2. COM3 is correct\n"
            f"3. Serial Monitor is closed\n\n"
            f"Error: {e}"
        )


    # Opening serial usually resets ESP32.
    time.sleep(1.5)


    ser.reset_input_buffer()

    ser.reset_output_buffer()


    print(
        "ESP32 connected."
    )

    print(
        "Starting handshake..."
    )


    # ------------------------------------------------------------
    # HANDSHAKE
    # ------------------------------------------------------------

    for attempt in range(5):

        try:

            ser.reset_input_buffer()


            send_packet(
                ser,
                PKT_HANDSHAKE,
                attempt,
                b""
            )


            wait_for_response(
                ser,
                RESP_READY,
                expected_sequence=attempt,
                timeout=2
            )


            print(
                "Handshake successful."
            )


            return ser


        except Exception as e:

            print(
                f"Handshake "
                f"{attempt + 1}/5 failed: "
                f"{e}"
            )


            time.sleep(0.2)


    ser.close()


    raise RuntimeError(
        "ESP32 handshake failed."
    )


# ================================================================
# RECORD AUDIO
# ================================================================

def record_audio():

    print()
    print("=" * 65)
    print("                         RECORDING")
    print("=" * 65)
    print()


    print(
        f"Duration: "
        f"{RECORD_SECONDS} seconds"
    )


    print()


    print(
        f"FIRST {NOISE_SECONDS} SECONDS:"
    )

    print(
        "Keep quiet so the ESP32 "
        "can estimate background noise."
    )


    print()


    print(
        f"NEXT "
        f"{RECORD_SECONDS - NOISE_SECONDS} "
        f"SECONDS:"
    )

    print(
        "Speak normally."
    )


    print()


    input(
        "Press ENTER to start..."
    )


    print()

    print(
        "Recording..."
    )


    start = time.perf_counter()


    audio = sd.rec(
        int(
            RECORD_SECONDS *
            FS
        ),

        samplerate=FS,

        channels=1,

        dtype="int16"
    )


    sd.wait()


    elapsed = (
        time.perf_counter() -
        start
    )


    audio = audio[:, 0]


    print()

    print(
        f"Recording completed in "
        f"{elapsed:.3f} seconds."
    )


    return audio


# ================================================================
# PLAY AUDIO
# ================================================================

def play_audio(
    audio,
    name
):

    print()

    print(
        f"Playing {name}..."
    )


    data = (
        audio.astype(
            np.float32
        ) /
        32768.0
    )


    sd.play(
        data,
        FS
    )


    sd.wait()


    print(
        f"{name} playback complete."
    )


# ================================================================
# SAVE WAV
# ================================================================

def save_wav(
    filename,
    audio
):

    directory = os.path.dirname(
        os.path.abspath(__file__)
    )


    path = os.path.join(
        directory,
        filename
    )


    write(
        path,
        FS,
        audio.astype(np.int16)
    )


    return path


# ================================================================
# FRAME COUNT
# ================================================================

def calculate_frame_count(
    samples
):

    if samples <= FFT_SIZE:

        return 1


    return (
        int(
            np.ceil(
                (
                    samples -
                    FFT_SIZE
                ) /
                HOP_SIZE
            )
        )
        + 1
    )


# ================================================================
# GET FRAME
# ================================================================

def get_frame(
    audio,
    frame_index
):

    # First frame
    if frame_index == 0:

        start = 0

        end = FFT_SIZE

        frame = audio[
            start:end
        ]


        if len(frame) < FFT_SIZE:

            frame = np.pad(
                frame,
                (
                    0,
                    FFT_SIZE -
                    len(frame)
                )
            )


        return frame.astype(
            np.int16
        )


    # Remaining frames
    start = (
        frame_index *
        HOP_SIZE
    )

    end = (
        start +
        HOP_SIZE
    )


    frame = audio[
        start:end
    ]


    if len(frame) < HOP_SIZE:

        frame = np.pad(
            frame,
            (
                0,
                HOP_SIZE -
                len(frame)
            )
        )


    return frame.astype(
        np.int16
    )


# ================================================================
# CREATE NOISE FRAMES
# ================================================================

def create_noise_frames(
    audio
):

    noise_samples = int(
        NOISE_SECONDS *
        FS
    )


    noise_audio = audio[
        :noise_samples
    ]


    frame_count = (
        calculate_frame_count(
            len(noise_audio)
        )
    )


    frames = []


    for i in range(
        frame_count
    ):

        frame = get_frame(
            noise_audio,
            i
        )


        if i == 0:

            frames.append(
                frame[:FFT_SIZE]
            )

        else:

            frames.append(
                frame[:HOP_SIZE]
            )


    return frames


# ================================================================
# SEND NOISE ESTIMATION
# ================================================================

def send_noise_estimation(
    ser,
    audio
):

    print()
    print("=" * 65)
    print("                    NOISE ESTIMATION")
    print("=" * 65)
    print()


    frames = create_noise_frames(
        audio
    )


    frame_count = len(frames)


    print(
        f"Noise frames: "
        f"{frame_count}"
    )


    # ------------------------------------------------------------
    # Tell ESP32 noise estimation is starting
    # ------------------------------------------------------------

    start_payload = struct.pack(
        "<I",
        frame_count
    )


    send_packet(
        ser,
        PKT_NOISE_START,
        0,
        start_payload
    )


    # ESP32 says READY
    wait_for_response(
        ser,
        RESP_READY,
        expected_sequence=0,
        timeout=5
    )


    print(
        "ESP32 ready for noise frames."
    )


    # ------------------------------------------------------------
    # Send frames ONE AT A TIME
    #
    # This is important.
    #
    # Python waits for ESP32 ACK after
    # each frame, preventing RX overflow.
    # ------------------------------------------------------------

    for i, frame in enumerate(frames):


        if i == 0:

            data = frame[
                :FFT_SIZE
            ]

        else:

            data = frame[
                :HOP_SIZE
            ]


        payload = data.astype(
            "<i2"
        ).tobytes()


        send_packet(
            ser,
            PKT_NOISE_DATA,
            i,
            payload
        )


        # Wait for this exact frame
        wait_for_response(
            ser,
            RESP_READY,
            expected_sequence=i,
            timeout=5
        )


        if (
            i == 0 or
            (i + 1) % 10 == 0 or
            i == frame_count - 1
        ):

            percent = (
                (i + 1) /
                frame_count *
                100
            )


            print(
                f"\rNoise estimation: "
                f"{i + 1}/"
                f"{frame_count} "
                f"({percent:5.1f}%)",
                end=""
            )


    print()


    # ------------------------------------------------------------
    # Wait for final DONE
    # ------------------------------------------------------------

    wait_for_response(
        ser,
        RESP_DONE,
        expected_sequence=frame_count,
        timeout=5
    )


    print(
        "Noise estimation completed."
    )


# ================================================================
# PROCESS AUDIO
# ================================================================

def process_audio_on_esp32(
    ser,
    audio
):

    print()
    print("=" * 65)
    print("                    ESP32 PROCESSING")
    print("=" * 65)
    print()


    frame_count = (
        calculate_frame_count(
            len(audio)
        )
    )


    print(
        f"Processing frames: "
        f"{frame_count}"
    )


    print(
        f"Expected output hops: "
        f"{frame_count + 1}"
    )


    # ------------------------------------------------------------
    # Start processing
    # ------------------------------------------------------------

    start_payload = struct.pack(
        "<I",
        frame_count
    )


    send_packet(
        ser,
        PKT_PROCESS_START,
        0,
        start_payload
    )


    # Wait for ESP32
    wait_for_response(
        ser,
        RESP_READY,
        expected_sequence=0,
        timeout=5
    )


    print(
        "ESP32 ready for audio frames."
    )


    # ------------------------------------------------------------
    # Allocate output
    # ------------------------------------------------------------

    output_length = (
        (
            frame_count +
            1
        )
        *
        HOP_SIZE
    )


    output = np.zeros(
        output_length,
        dtype=np.int16
    )


    processing_start = (
        time.perf_counter()
    )


    # ------------------------------------------------------------
    # Send/process frames
    # ------------------------------------------------------------

    for i in range(
        frame_count
    ):

        frame = get_frame(
            audio,
            i
        )


        if i == 0:

            data = frame[
                :FFT_SIZE
            ]

        else:

            data = frame[
                :HOP_SIZE
            ]


        payload = data.astype(
            "<i2"
        ).tobytes()


        # --------------------------------------------------------
        # Send input
        # --------------------------------------------------------

        send_packet(
            ser,
            PKT_PROCESS_DATA,
            i,
            payload
        )


        # --------------------------------------------------------
        # Receive processed output
        # --------------------------------------------------------

        packet_type, sequence, payload = (
            receive_packet(
                ser,
                timeout=5
            )
        )


        if packet_type == RESP_ERROR:

            raise RuntimeError(
                f"ESP32 processing error "
                f"at sequence {sequence}"
            )


        if packet_type != RESP_OUTPUT:

            raise RuntimeError(
                f"Unexpected ESP32 packet: "
                f"{chr(packet_type)}"
            )


        if sequence != i:

            raise RuntimeError(
                f"Output sequence mismatch: "
                f"received {sequence}, "
                f"expected {i}"
            )


        expected_bytes = (
            HOP_SIZE *
            2
        )


        if len(payload) != expected_bytes:

            raise RuntimeError(
                f"Invalid output length: "
                f"{len(payload)}"
            )


        hop = np.frombuffer(
            payload,
            dtype="<i2"
        ).copy()


        start = (
            i *
            HOP_SIZE
        )


        end = (
            start +
            HOP_SIZE
        )


        output[
            start:end
        ] = hop


        # --------------------------------------------------------
        # Progress
        # --------------------------------------------------------

        if (
            i == 0 or
            (i + 1) % 10 == 0 or
            i == frame_count - 1
        ):

            percent = (
                (i + 1) /
                frame_count *
                100
            )


            print(
                f"\rProcessing: "
                f"{i + 1}/"
                f"{frame_count} "
                f"({percent:5.1f}%)",
                end=""
            )


    print()


    # ------------------------------------------------------------
    # Receive final tail
    # ------------------------------------------------------------

    packet_type, sequence, payload = (
        receive_packet(
            ser,
            timeout=5
        )
    )


    if packet_type != RESP_OUTPUT:

        raise RuntimeError(
            "Expected final output packet."
        )


    if sequence != frame_count:

        raise RuntimeError(
            f"Final sequence mismatch: "
            f"{sequence} != "
            f"{frame_count}"
        )


    if len(payload) != (
        HOP_SIZE * 2
    ):

        raise RuntimeError(
            "Invalid final output size."
        )


    final_hop = np.frombuffer(
        payload,
        dtype="<i2"
    ).copy()


    start = (
        frame_count *
        HOP_SIZE
    )


    end = (
        start +
        HOP_SIZE
    )


    output[
        start:end
    ] = final_hop


    # ------------------------------------------------------------
    # Wait for DONE
    # ------------------------------------------------------------

    wait_for_response(
        ser,
        RESP_DONE,
        expected_sequence=frame_count,
        timeout=5
    )


    processing_time = (
        time.perf_counter() -
        processing_start
    )


    print()

    print(
        f"ESP32 processing completed in "
        f"{processing_time:.3f} seconds."
    )


    return output


# ================================================================
# AUDIO INFORMATION
# ================================================================

def print_audio_information(
    audio
):

    noise_samples = int(
        NOISE_SECONDS *
        FS
    )


    noise_frames = len(
        create_noise_frames(
            audio
        )
    )


    processing_frames = (
        calculate_frame_count(
            len(audio)
        )
    )


    print()
    print("=" * 65)
    print("                       AUDIO INFORMATION")
    print("=" * 65)
    print()


    print(
        f"Audio samples       : "
        f"{len(audio)}"
    )


    print(
        f"Sample rate         : "
        f"{FS} Hz"
    )


    print(
        f"FFT size            : "
        f"{FFT_SIZE}"
    )


    print(
        f"Hop size            : "
        f"{HOP_SIZE}"
    )


    print(
        f"Frame duration      : "
        f"{FFT_SIZE / FS * 1000:.2f} ms"
    )


    print(
        f"Hop duration        : "
        f"{HOP_SIZE / FS * 1000:.2f} ms"
    )


    print(
        f"Noise samples       : "
        f"{noise_samples}"
    )


    print(
        f"Noise frames        : "
        f"{noise_frames}"
    )


    print(
        f"Processing frames   : "
        f"{processing_frames}"
    )


    print(
        f"Expected output hops: "
        f"{processing_frames + 1}"
    )


# ================================================================
# MAIN
# ================================================================

def main():

    print()
    print("=" * 65)
    print("       ESP32 OPTIMIZED AUDIO NOISE REDUCTION")
    print("=" * 65)
    print()


    print(
        f"Serial port : {PORT}"
    )


    print(
        f"Baud rate   : {BAUD}"
    )


    print(
        f"Sample rate : {FS}"
    )


    print(
        f"FFT size    : {FFT_SIZE}"
    )


    print(
        f"Hop size    : {HOP_SIZE}"
    )


    # ============================================================
    # RECORD
    # ============================================================

    audio = record_audio()


    # ============================================================
    # SAVE ORIGINAL
    # ============================================================

    original_path = save_wav(
        "Original_Audio_4.wav",
        audio
    )


    print()

    print(
        "Original saved:"
    )

    print(
        original_path
    )


    # ============================================================
    # PLAY ORIGINAL
    # ============================================================

    play_audio(
        audio,
        "original audio"
    )


    # ============================================================
    # INFORMATION
    # ============================================================

    print_audio_information(
        audio
    )


    ser = None


    try:

        # ========================================================
        # CONNECT
        # ========================================================

        ser = connect_esp32()


        # ========================================================
        # NOISE ESTIMATION
        # ========================================================

        send_noise_estimation(
            ser,
            audio
        )


        # ========================================================
        # ESP32 PROCESSING
        # ========================================================

        cleaned = (
            process_audio_on_esp32(
                ser,
                audio
            )
        )


        # ========================================================
        # TRIM TO ORIGINAL LENGTH
        # ========================================================

        cleaned = cleaned[
            :len(audio)
        ]


        # ========================================================
        # SAVE CLEANED AUDIO
        # ========================================================

        cleaned_path = save_wav(
            "Cleaned_Audio_ESP32.wav",
            cleaned
        )


        print()
        print("=" * 65)
        print("                         RESULTS")
        print("=" * 65)
        print()


        print(
            "Cleaned audio saved:"
        )


        print(
            cleaned_path
        )


        # ========================================================
        # PLAY CLEANED
        # ========================================================

        play_audio(
            cleaned,
            "ESP32 cleaned audio"
        )


        # ========================================================
        # RMS COMPARISON
        # ========================================================

        original_rms = np.sqrt(
            np.mean(
                audio.astype(
                    np.float64
                ) ** 2
            )
        )


        cleaned_rms = np.sqrt(
            np.mean(
                cleaned.astype(
                    np.float64
                ) ** 2
            )
        )


        print()
        print("=" * 65)
        print("                         SUMMARY")
        print("=" * 65)
        print()


        print(
            f"Original RMS : "
            f"{original_rms:.2f}"
        )


        print(
            f"Cleaned RMS  : "
            f"{cleaned_rms:.2f}"
        )


        print()


        print(
            "Processing completed successfully."
        )


    except KeyboardInterrupt:

        print()

        print(
            "Stopped by user."
        )


    except Exception as e:

        print()
        print("=" * 65)
        print("                           ERROR")
        print("=" * 65)
        print()


        print(
            str(e)
        )


        print()
        print(
            "Check:"
        )


        print(
            "1. ESP32 code was uploaded successfully."
        )


        print(
            "2. Arduino Serial Monitor is CLOSED."
        )


        print(
            "3. PORT = COM3."
        )


        print(
            "4. ESP32 baud = 921600."
        )


        print(
            "5. ArduinoFFT 2.0.4 is installed."
        )


    finally:

        if ser is not None:

            try:

                ser.close()

            except Exception:

                pass


# ================================================================
# START
# ================================================================

if __name__ == "__main__":

    main()