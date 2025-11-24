# KV4P HT – Python App Communication Setup

**Project:** kv4p-ht  
**Purpose:** Communicate with KV4P HT microcontroller (ESP32) over USB serial  
**App Goal:** Set frequency, control PTT (Push-To-Talk), send/receive audio, and get status.

---

## 1. Connection (Transport Layer)

- **Interface:** USB Serial (CDC ACM)
- **Typical Device:** `/dev/ttyACM0`, `/dev/ttyUSB0`, or `COMxx` on Windows
- **Baud Rate:** `115200`

---

## 2. Protocol Overview

- **Delimiters:** Each data packet starts with the fixed byte sequence:
  - `0xDE, 0xAD, 0xBE, 0xEF`
- **Frame Structure:**

| Field               | Size (bytes) | Description                                   |
|---------------------|--------------|-----------------------------------------------|
| Command Delimiter   | 4            | Fixed value (`0xDEADBEEF`)                    |
| Command             | 1            | Command code (see below)                      |
| Parameter Length    | 2            | Length of Parameters field (uint16_t LE)      |
| Parameters          | 0–2048       | Command-specific data (binary payload)        |

- **Byte Order:** Little-endian for multi-byte fields
- **MTU (Max Frame Payload):** 2048 bytes

---

## 3. Command Codes

**Incoming (Host → ESP32):**
- `0x01` – PTT Down
- `0x02` – PTT Up
- `0x03` – Set Group (frequencies, tones, squelch)
- `0x04` – Set Filters
- `0x05` – Stop Current Operation
- `0x06` – Send Config (High/Low power etc.)
- `0x07` – Transmit OPUS Audio
- `0x08` – Set High/Low power
- `0x09` – Enable/Disable RSSI reporting

**Outgoing (ESP32 → Host):**
- `0x53` – S-meter Report (RSSI)
- `0x44` – Physical PTT Down
- `0x55` – Physical PTT Up
- `0x01` – Debug Info/Error/Warning/Debug/Trace
- `0x06` – Hello handshake
- `0x07` – Receive OPUS Audio
- `0x08` – Send firmware version info
- `0x09` – Window update (flow control)

---

## 4. Flow Control

- **Window-based:** ESP32 reports available buffer window.
- **Host must not send more data than the window allows.**
- **Host replenishes window on receiving COMMAND_WINDOW_UPDATE.**

---

## 5. Example Config for Your Python App

```yaml
serial:
  port: /dev/ttyACM0         # Replace with your COM port
  baudrate: 115200
  timeout: 1.0
protocol:
  delimiter: [0xDE, 0xAD, 0xBE, 0xEF]
  mtu: 2048
  little_endian: true
  frame:
    - delimiter
    - command: uint8
    - parameter_length: uint16
    - parameters: bytes
app:
  overview: |
    Python app should:
      - Establish USB Serial connection to ESP32
      - Wait for handshake (COMMAND_HELLO)
      - Send config and group (frequency) settings
      - Handle PTT, audio, and receive status messages
  references:
    - https://github.com/malakaisbest/kv4p-ht/blob/main/microcontroller-src/kv4p_ht_esp32_wroom_32/readme.md
    - https://github.com/malakaisbest/kv4p-ht/blob/main/android-src/KV4PHT/app/src/main/java/com/vagell/kv4pht/radio/Protocol.java
requirements:
  - pyserial
  - (optional) opuslib or pyogg if audio is needed
  - struct, threading
```

---

## 6. Short Description

This configuration describes the protocol the Python app must speak to the ESP32 microcontroller in the KV4P HT radio project.  
Every message is framed with a special binary delimiter, has a command code, and may include payload/parameters.  
The app starts by connecting over USB serial, waits for a HELLO from the ESP32, and then uses the above command set to operate radio features and transmit/receive data.

---

**For protocol details and payload structures, see:**  
[Full Protocol Documentation (README)](https://github.com/malakaisbest/kv4p-ht/blob/main/microcontroller-src/kv4p_ht_esp32_wroom_32/readme.md)
