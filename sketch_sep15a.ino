/*
================================================================
 ESP32 AUDIO NOISE REDUCTION
================================================================

 Board:
   ESP32 Dev Module

 ArduinoFFT:
   2.0.4

 Serial:
   921600 baud

 Audio:
   16 kHz
   16-bit
   Mono

 DSP:
   FFT size = 512
   Hop size = 256
   50% overlap

 Protocol:

   HOST -> ESP32

   H = Handshake
   N = Noise estimation start
   n = Noise audio frame
   P = Processing start
   p = Processing audio frame

   ESP32 -> HOST

   R = Ready / ACK
   O = Processed output
   D = Done
   E = Error

================================================================
*/

#include <Arduino.h>
#include <arduinoFFT.h>
#include <math.h>
#include <string.h>


// ================================================================
// CONFIGURATION
// ================================================================

#define BAUD_RATE       921600
#define SAMPLE_RATE     16000

#define FFT_SIZE        512
#define HOP_SIZE        256
#define HALF_FFT        (FFT_SIZE / 2)

#define MAX_PAYLOAD     1024

#define PACKET_TIMEOUT  3000


// ================================================================
// PACKET TYPES
// ================================================================

// Host -> ESP32
#define PKT_HANDSHAKE       'H'

#define PKT_NOISE_START     'N'
#define PKT_NOISE_DATA      'n'

#define PKT_PROCESS_START   'P'
#define PKT_PROCESS_DATA    'p'


// ESP32 -> Host
#define RESP_READY          'R'
#define RESP_OUTPUT         'O'
#define RESP_DONE           'D'
#define RESP_ERROR          'E'


// ================================================================
// NOISE REDUCTION PARAMETERS
// ================================================================

#define MIN_PROCESS_FREQ    70.0f
#define MAX_PROCESS_FREQ    7500.0f

#define NOISE_FACTOR        1.25f

#define MIN_GAIN            0.08f
#define MAX_GAIN            1.00f

#define SMOOTHING           0.75f


// ================================================================
// FFT BUFFERS
// ================================================================

float vReal[FFT_SIZE];
float vImag[FFT_SIZE];


// ================================================================
// NOISE MODEL
// ================================================================

float noisePower[HALF_FFT];

float previousGain[HALF_FFT];


// ================================================================
// WINDOW
// ================================================================

float hannWindow[FFT_SIZE];


// ================================================================
// AUDIO BUFFERS
// ================================================================

int16_t frameBuffer[FFT_SIZE];

int16_t hopBuffer[HOP_SIZE];

int16_t outputBuffer[HOP_SIZE];


// ================================================================
// OVERLAP-ADD BUFFERS
// ================================================================

float olaBuffer[FFT_SIZE];

float olaWeight[FFT_SIZE];


// ================================================================
// PACKET BUFFER
// ================================================================

uint8_t packetPayload[MAX_PAYLOAD];


// ================================================================
// FFT OBJECT
// ================================================================

ArduinoFFT<float> FFT(
    vReal,
    vImag,
    FFT_SIZE,
    SAMPLE_RATE
);


// ================================================================
// CHECKSUM
// ================================================================

uint8_t calculateChecksum(
    uint8_t type,
    uint16_t sequence,
    uint16_t length,
    const uint8_t *payload
) {

    uint8_t checksum = 0;

    checksum ^= type;

    checksum ^= (uint8_t)(sequence & 0xFF);

    checksum ^= (uint8_t)(
        (sequence >> 8) & 0xFF
    );

    checksum ^= (uint8_t)(length & 0xFF);

    checksum ^= (uint8_t)(
        (length >> 8) & 0xFF
    );

    for (
        uint16_t i = 0;
        i < length;
        i++
    ) {

        checksum ^= payload[i];
    }

    return checksum;
}


// ================================================================
// SEND PACKET
// ================================================================

void sendPacket(
    uint8_t type,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t length
) {

    uint8_t checksum =
        calculateChecksum(
            type,
            sequence,
            length,
            payload
        );

    Serial.write(0xAA);
    Serial.write(0x55);

    Serial.write(type);

    Serial.write(
        (uint8_t)(sequence & 0xFF)
    );

    Serial.write(
        (uint8_t)(
            (sequence >> 8) & 0xFF
        )
    );

    Serial.write(
        (uint8_t)(length & 0xFF)
    );

    Serial.write(
        (uint8_t)(
            (length >> 8) & 0xFF
        )
    );

    if (
        length > 0 &&
        payload != nullptr
    ) {

        Serial.write(
            payload,
            length
        );
    }

    Serial.write(checksum);
}


// ================================================================
// SEND EMPTY RESPONSE
// ================================================================

void sendResponse(
    uint8_t type,
    uint16_t sequence
) {

    sendPacket(
        type,
        sequence,
        nullptr,
        0
    );
}


// ================================================================
// WAIT FOR EXACT NUMBER OF BYTES
// ================================================================

bool readBytes(
    uint8_t *buffer,
    uint16_t count,
    uint32_t timeout
) {

    uint32_t start =
        millis();

    uint16_t received = 0;

    while (received < count) {

        if (Serial.available()) {

            buffer[received++] =
                Serial.read();

        } else {

            if (
                millis() - start >
                timeout
            ) {

                return false;
            }

            delayMicroseconds(50);
        }
    }

    return true;
}


// ================================================================
// FIND PACKET HEADER
// ================================================================

bool findHeader(
    uint32_t timeout
) {

    uint32_t start =
        millis();

    uint8_t state = 0;

    while (
        millis() - start <
        timeout
    ) {

        if (!Serial.available()) {

            delayMicroseconds(50);

            continue;
        }

        uint8_t value =
            Serial.read();

        if (state == 0) {

            if (value == 0xAA) {

                state = 1;
            }

        } else {

            if (value == 0x55) {

                return true;
            }

            if (value == 0xAA) {

                state = 1;

            } else {

                state = 0;
            }
        }
    }

    return false;
}


// ================================================================
// RECEIVE PACKET
// ================================================================

bool receivePacket(
    uint8_t &type,
    uint16_t &sequence,
    uint8_t *payload,
    uint16_t &length,
    uint32_t timeout
) {

    // ------------------------------------------------------------
    // Header
    // ------------------------------------------------------------

    if (!findHeader(timeout)) {

        return false;
    }


    // ------------------------------------------------------------
    // Type
    // ------------------------------------------------------------

    if (
        !readBytes(
            &type,
            1,
            timeout
        )
    ) {

        return false;
    }


    // ------------------------------------------------------------
    // Sequence
    // ------------------------------------------------------------

    uint8_t seqBytes[2];

    if (
        !readBytes(
            seqBytes,
            2,
            timeout
        )
    ) {

        return false;
    }

    sequence =
        (uint16_t)seqBytes[0] |
        ((uint16_t)seqBytes[1] << 8);


    // ------------------------------------------------------------
    // Length
    // ------------------------------------------------------------

    uint8_t lengthBytes[2];

    if (
        !readBytes(
            lengthBytes,
            2,
            timeout
        )
    ) {

        return false;
    }

    length =
        (uint16_t)lengthBytes[0] |
        ((uint16_t)lengthBytes[1] << 8);


    // ------------------------------------------------------------
    // Validate length
    // ------------------------------------------------------------

    if (
        length >
        MAX_PAYLOAD
    ) {

        sendResponse(
            RESP_ERROR,
            sequence
        );

        return false;
    }


    // ------------------------------------------------------------
    // Payload
    // ------------------------------------------------------------

    if (length > 0) {

        if (
            !readBytes(
                payload,
                length,
                timeout
            )
        ) {

            return false;
        }
    }


    // ------------------------------------------------------------
    // Checksum
    // ------------------------------------------------------------

    uint8_t receivedChecksum;

    if (
        !readBytes(
            &receivedChecksum,
            1,
            timeout
        )
    ) {

        return false;
    }


    uint8_t calculatedChecksum =
        calculateChecksum(
            type,
            sequence,
            length,
            payload
        );


    if (
        receivedChecksum !=
        calculatedChecksum
    ) {

        sendResponse(
            RESP_ERROR,
            sequence
        );

        return false;
    }


    return true;
}


// ================================================================
// CREATE HANN WINDOW
// ================================================================

void createHannWindow() {

    for (
        int i = 0;
        i < FFT_SIZE;
        i++
    ) {

        hannWindow[i] =
            0.5f *
            (
                1.0f -
                cosf(
                    2.0f *
                    PI *
                    (float)i /
                    (float)(FFT_SIZE - 1)
                )
            );
    }
}


// ================================================================
// CLEAR DSP STATE
// ================================================================

void clearDSPState() {

    memset(
        noisePower,
        0,
        sizeof(noisePower)
    );

    memset(
        previousGain,
        0,
        sizeof(previousGain)
    );

    memset(
        olaBuffer,
        0,
        sizeof(olaBuffer)
    );

    memset(
        olaWeight,
        0,
        sizeof(olaWeight)
    );

    memset(
        frameBuffer,
        0,
        sizeof(frameBuffer)
    );

    memset(
        hopBuffer,
        0,
        sizeof(hopBuffer)
    );

    memset(
        outputBuffer,
        0,
        sizeof(outputBuffer)
    );
}


// ================================================================
// LOAD FIRST 512-SAMPLE FRAME
// ================================================================

bool loadFirstFrame(
    const uint8_t *payload,
    uint16_t length
) {

    if (
        length !=
        FFT_SIZE * sizeof(int16_t)
    ) {

        return false;
    }

    memcpy(
        frameBuffer,
        payload,
        FFT_SIZE *
        sizeof(int16_t)
    );

    return true;
}


// ================================================================
// LOAD NEXT 256 SAMPLES
// ================================================================

bool loadNextHop(
    const uint8_t *payload,
    uint16_t length
) {

    if (
        length !=
        HOP_SIZE * sizeof(int16_t)
    ) {

        return false;
    }


    memcpy(
        hopBuffer,
        payload,
        HOP_SIZE *
        sizeof(int16_t)
    );


    // Shift old frame left by 256
    memmove(
        frameBuffer,
        frameBuffer + HOP_SIZE,
        (FFT_SIZE - HOP_SIZE) *
        sizeof(int16_t)
    );


    // Append new 256 samples
    memcpy(
        frameBuffer +
        FFT_SIZE -
        HOP_SIZE,

        hopBuffer,

        HOP_SIZE *
        sizeof(int16_t)
    );


    return true;
}


// ================================================================
// NOISE FRAME FFT
// ================================================================

void analyzeNoiseFrame() {

    // ------------------------------------------------------------
    // Calculate DC
    // ------------------------------------------------------------

    float mean = 0.0f;

    for (
        int i = 0;
        i < FFT_SIZE;
        i++
    ) {

        mean +=
            (float)frameBuffer[i];
    }

    mean /=
        (float)FFT_SIZE;


    // ------------------------------------------------------------
    // Prepare FFT
    // ------------------------------------------------------------

    for (
        int i = 0;
        i < FFT_SIZE;
        i++
    ) {

        vReal[i] =
            (
                (float)frameBuffer[i] -
                mean
            ) *
            hannWindow[i];

        vImag[i] = 0.0f;
    }


    // ------------------------------------------------------------
    // FFT
    // ------------------------------------------------------------

    FFT.compute(
        FFTDirection::Forward
    );


    // ------------------------------------------------------------
    // Calculate noise power
    //
    // IMPORTANT:
    // Do NOT use complexToMagnitude()
    // here because we need the complex values.
    // ------------------------------------------------------------

    for (
        int k = 0;
        k < HALF_FFT;
        k++
    ) {

        float realPart =
            vReal[k];

        float imagPart =
            vImag[k];

        float power =
            realPart * realPart +
            imagPart * imagPart;

        noisePower[k] +=
            power;
    }
}


// ================================================================
// FINISH NOISE ESTIMATION
// ================================================================

void finalizeNoise(
    uint16_t frameCount
) {

    if (frameCount == 0) {

        frameCount = 1;
    }

    for (
        int k = 0;
        k < HALF_FFT;
        k++
    ) {

        noisePower[k] /=
            (float)frameCount;

        if (
            noisePower[k] <
            1.0f
        ) {

            noisePower[k] = 1.0f;
        }
    }
}


// ================================================================
// PROCESS AUDIO FRAME
// ================================================================

void processAudioFrame() {

    // ------------------------------------------------------------
    // DC removal
    // ------------------------------------------------------------

    float mean = 0.0f;

    for (
        int i = 0;
        i < FFT_SIZE;
        i++
    ) {

        mean +=
            (float)frameBuffer[i];
    }

    mean /=
        (float)FFT_SIZE;


    // ------------------------------------------------------------
    // Prepare FFT
    // ------------------------------------------------------------

    for (
        int i = 0;
        i < FFT_SIZE;
        i++
    ) {

        vReal[i] =
            (
                (float)frameBuffer[i] -
                mean
            ) *
            hannWindow[i];

        vImag[i] = 0.0f;
    }


    // ------------------------------------------------------------
    // Forward FFT
    // ------------------------------------------------------------

    FFT.compute(
        FFTDirection::Forward
    );


    // ------------------------------------------------------------
    // Noise suppression
    // ------------------------------------------------------------

    for (
        int k = 1;
        k < HALF_FFT;
        k++
    ) {

        float frequency =
            (
                (float)k *
                SAMPLE_RATE
            ) /
            FFT_SIZE;


        // --------------------------------------------------------
        // Outside processing range
        // --------------------------------------------------------

        if (
            frequency <
            MIN_PROCESS_FREQ ||
            frequency >
            MAX_PROCESS_FREQ
        ) {

            continue;
        }


        // --------------------------------------------------------
        // Complex FFT value
        // --------------------------------------------------------

        float realPart =
            vReal[k];

        float imagPart =
            vImag[k];


        // --------------------------------------------------------
        // Power
        // --------------------------------------------------------

        float power =
            realPart * realPart +
            imagPart * imagPart;


        // --------------------------------------------------------
        // Noise estimate
        // --------------------------------------------------------

        float estimatedNoise =
            noisePower[k] *
            NOISE_FACTOR;


        // --------------------------------------------------------
        // Estimate speech power
        // --------------------------------------------------------

        float speechPower =
            power -
            estimatedNoise;

        if (
            speechPower < 0.0f
        ) {

            speechPower = 0.0f;
        }


        // --------------------------------------------------------
        // SNR
        // --------------------------------------------------------

        float snr =
            speechPower /
            (
                estimatedNoise +
                1.0f
            );


        // --------------------------------------------------------
        // Wiener gain
        // --------------------------------------------------------

        float gain =
            snr /
            (1.0f + snr);


        // --------------------------------------------------------
        // Gain limits
        // --------------------------------------------------------

        if (
            gain <
            MIN_GAIN
        ) {

            gain =
                MIN_GAIN;
        }

        if (
            gain >
            MAX_GAIN
        ) {

            gain =
                MAX_GAIN;
        }


        // --------------------------------------------------------
        // Temporal smoothing
        // --------------------------------------------------------

        gain =
            SMOOTHING *
            previousGain[k]
            +
            (1.0f - SMOOTHING) *
            gain;


        previousGain[k] =
            gain;


        // --------------------------------------------------------
        // Apply gain
        // --------------------------------------------------------

        vReal[k] =
            realPart *
            gain;

        vImag[k] =
            imagPart *
            gain;


        // --------------------------------------------------------
        // Conjugate mirror
        // --------------------------------------------------------

        int mirror =
            FFT_SIZE - k;

        vReal[mirror] =
            vReal[k];

        vImag[mirror] =
            -vImag[k];
    }


    // ------------------------------------------------------------
    // DC
    // ------------------------------------------------------------

    vReal[0] = 0.0f;
    vImag[0] = 0.0f;


    // ------------------------------------------------------------
    // Nyquist
    // ------------------------------------------------------------

    vImag[HALF_FFT] = 0.0f;


    // ------------------------------------------------------------
    // Inverse FFT
    // ------------------------------------------------------------

    FFT.compute(
        FFTDirection::Reverse
    );


    // ------------------------------------------------------------
    // Synthesis window + OLA
    // ------------------------------------------------------------

    for (
        int i = 0;
        i < FFT_SIZE;
        i++
    ) {

        float window =
            hannWindow[i];

        float sample =
            vReal[i] *
            window;


        olaBuffer[i] +=
            sample;


        olaWeight[i] +=
            window *
            window;
    }


    // ------------------------------------------------------------
    // Extract 256 samples
    // ------------------------------------------------------------

    for (
        int i = 0;
        i < HOP_SIZE;
        i++
    ) {

        float sample = 0.0f;

        float weight =
            olaWeight[i];


        if (
            weight >
            0.000001f
        ) {

            sample =
                olaBuffer[i] /
                weight;
        }


        // --------------------------------------------------------
        // Saturation
        // --------------------------------------------------------

        if (
            sample >
            32767.0f
        ) {

            sample =
                32767.0f;
        }


        if (
            sample <
            -32768.0f
        ) {

            sample =
                -32768.0f;
        }


        outputBuffer[i] =
            (int16_t)sample;
    }


    // ------------------------------------------------------------
    // Shift OLA
    // ------------------------------------------------------------

    memmove(
        olaBuffer,
        olaBuffer + HOP_SIZE,
        (FFT_SIZE - HOP_SIZE) *
        sizeof(float)
    );

    memmove(
        olaWeight,
        olaWeight + HOP_SIZE,
        (FFT_SIZE - HOP_SIZE) *
        sizeof(float)
    );


    // Clear last half
    memset(
        olaBuffer +
        FFT_SIZE -
        HOP_SIZE,

        0,

        HOP_SIZE *
        sizeof(float)
    );


    memset(
        olaWeight +
        FFT_SIZE -
        HOP_SIZE,

        0,

        HOP_SIZE *
        sizeof(float)
    );
}


// ================================================================
// SEND OUTPUT HOP
// ================================================================

void sendOutput(
    uint16_t sequence
) {

    sendPacket(
        RESP_OUTPUT,
        sequence,

        (uint8_t *)outputBuffer,

        HOP_SIZE *
        sizeof(int16_t)
    );
}


// ================================================================
// HANDLE NOISE ESTIMATION
// ================================================================

void handleNoiseEstimation(
    uint16_t frameCount
) {

    clearDSPState();


    // ------------------------------------------------------------
    // Tell Python we are ready for frames
    // ------------------------------------------------------------

    sendResponse(
        RESP_READY,
        0
    );


    // ------------------------------------------------------------
    // Receive frames
    // ------------------------------------------------------------

    for (
        uint16_t frame = 0;
        frame < frameCount;
        frame++
    ) {

        uint8_t type;
        uint16_t sequence;
        uint16_t length;


        bool success =
            receivePacket(
                type,
                sequence,
                packetPayload,
                length,
                PACKET_TIMEOUT
            );


        if (!success) {

            sendResponse(
                RESP_ERROR,
                frame
            );

            return;
        }


        // --------------------------------------------------------
        // Validate packet type
        // --------------------------------------------------------

        if (
            type !=
            PKT_NOISE_DATA
        ) {

            sendResponse(
                RESP_ERROR,
                sequence
            );

            return;
        }


        // --------------------------------------------------------
        // Validate sequence
        // --------------------------------------------------------

        if (
            sequence !=
            frame
        ) {

            sendResponse(
                RESP_ERROR,
                sequence
            );

            return;
        }


        // --------------------------------------------------------
        // First frame = 512 samples
        // --------------------------------------------------------

        if (frame == 0) {

            if (
                !loadFirstFrame(
                    packetPayload,
                    length
                )
            ) {

                sendResponse(
                    RESP_ERROR,
                    frame
                );

                return;
            }

        }

        // --------------------------------------------------------
        // Remaining frames = 256 samples
        // --------------------------------------------------------

        else {

            if (
                !loadNextHop(
                    packetPayload,
                    length
                )
            ) {

                sendResponse(
                    RESP_ERROR,
                    frame
                );

                return;
            }
        }


        // --------------------------------------------------------
        // FFT noise analysis
        // --------------------------------------------------------

        analyzeNoiseFrame();


        // --------------------------------------------------------
        // ACK THIS FRAME
        // --------------------------------------------------------

        sendResponse(
            RESP_READY,
            frame
        );
    }


    // ------------------------------------------------------------
    // Finalize noise model
    // ------------------------------------------------------------

    finalizeNoise(
        frameCount
    );


    // ------------------------------------------------------------
    // Noise estimation complete
    // ------------------------------------------------------------

    sendResponse(
        RESP_DONE,
        frameCount
    );
}


// ================================================================
// HANDLE PROCESSING
// ================================================================

void handleProcessing(
    uint16_t frameCount
) {

    // Reset only OLA and temporal state.
    // Keep noisePower!
    memset(
        previousGain,
        0,
        sizeof(previousGain)
    );

    memset(
        olaBuffer,
        0,
        sizeof(olaBuffer)
    );

    memset(
        olaWeight,
        0,
        sizeof(olaWeight)
    );


    // ------------------------------------------------------------
    // Tell Python we are ready
    // ------------------------------------------------------------

    sendResponse(
        RESP_READY,
        0
    );


    // ------------------------------------------------------------
    // Receive/process frames
    // ------------------------------------------------------------

    for (
        uint16_t frame = 0;
        frame < frameCount;
        frame++
    ) {

        uint8_t type;
        uint16_t sequence;
        uint16_t length;


        bool success =
            receivePacket(
                type,
                sequence,
                packetPayload,
                length,
                PACKET_TIMEOUT
            );


        if (!success) {

            sendResponse(
                RESP_ERROR,
                frame
            );

            return;
        }


        // --------------------------------------------------------
        // Packet type
        // --------------------------------------------------------

        if (
            type !=
            PKT_PROCESS_DATA
        ) {

            sendResponse(
                RESP_ERROR,
                sequence
            );

            return;
        }


        // --------------------------------------------------------
        // Sequence
        // --------------------------------------------------------

        if (
            sequence !=
            frame
        ) {

            sendResponse(
                RESP_ERROR,
                sequence
            );

            return;
        }


        // --------------------------------------------------------
        // Load frame
        // --------------------------------------------------------

        if (frame == 0) {

            if (
                !loadFirstFrame(
                    packetPayload,
                    length
                )
            ) {

                sendResponse(
                    RESP_ERROR,
                    frame
                );

                return;
            }

        }

        else {

            if (
                !loadNextHop(
                    packetPayload,
                    length
                )
            ) {

                sendResponse(
                    RESP_ERROR,
                    frame
                );

                return;
            }
        }


        // --------------------------------------------------------
        // DSP
        // --------------------------------------------------------

        processAudioFrame();


        // --------------------------------------------------------
        // Send processed hop
        // --------------------------------------------------------

        sendOutput(
            frame
        );
    }


    // ------------------------------------------------------------
    // Flush OLA tail
    // ------------------------------------------------------------

    memset(
        frameBuffer +
        FFT_SIZE -
        HOP_SIZE,

        0,

        HOP_SIZE *
        sizeof(int16_t)
    );


    memmove(
        frameBuffer,
        frameBuffer + HOP_SIZE,
        (FFT_SIZE - HOP_SIZE) *
        sizeof(int16_t)
    );


    processAudioFrame();


    sendOutput(
        frameCount
    );


    // ------------------------------------------------------------
    // Done
    // ------------------------------------------------------------

    sendResponse(
        RESP_DONE,
        frameCount
    );
}


// ================================================================
// SETUP
// ================================================================

void setup() {

    // Set buffers BEFORE Serial.begin
    Serial.setRxBufferSize(4096);
    Serial.setTxBufferSize(4096);

    Serial.begin(
        BAUD_RATE
    );


    delay(1000);


    createHannWindow();

    clearDSPState();


    /*
     * IMPORTANT:
     *
     * Do NOT send READY automatically here.
     *
     * Python will send:
     *
     *     H
     *
     * ESP32 replies:
     *
     *     R
     *
     * This prevents the startup race condition.
     */
}


// ================================================================
// MAIN LOOP
// ================================================================

void loop() {

    uint8_t type;
    uint16_t sequence;
    uint16_t length;


    bool received =
        receivePacket(
            type,
            sequence,
            packetPayload,
            length,
            100
        );


    if (!received) {

        return;
    }


    // ============================================================
    // HANDSHAKE
    // ============================================================

    if (
        type ==
        PKT_HANDSHAKE
    ) {

        sendResponse(
            RESP_READY,
            sequence
        );

        return;
    }


    // ============================================================
    // NOISE START
    // ============================================================

    if (
        type ==
        PKT_NOISE_START
    ) {

        if (length != 4) {

            sendResponse(
                RESP_ERROR,
                sequence
            );

            return;
        }


        uint32_t frameCount =
            ((uint32_t)packetPayload[0]) |
            ((uint32_t)packetPayload[1] << 8) |
            ((uint32_t)packetPayload[2] << 16) |
            ((uint32_t)packetPayload[3] << 24);


        if (
            frameCount == 0 ||
            frameCount > 1000
        ) {

            sendResponse(
                RESP_ERROR,
                sequence
            );

            return;
        }


        handleNoiseEstimation(
            (uint16_t)frameCount
        );

        return;
    }


    // ============================================================
    // PROCESS START
    // ============================================================

    if (
        type ==
        PKT_PROCESS_START
    ) {

        if (length != 4) {

            sendResponse(
                RESP_ERROR,
                sequence
            );

            return;
        }


        uint32_t frameCount =
            ((uint32_t)packetPayload[0]) |
            ((uint32_t)packetPayload[1] << 8) |
            ((uint32_t)packetPayload[2] << 16) |
            ((uint32_t)packetPayload[3] << 24);


        if (
            frameCount == 0 ||
            frameCount > 1000
        ) {

            sendResponse(
                RESP_ERROR,
                sequence
            );

            return;
        }


        handleProcessing(
            (uint16_t)frameCount
        );

        return;
    }
}