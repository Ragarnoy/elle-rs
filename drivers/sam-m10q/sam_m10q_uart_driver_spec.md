# SAM-M10Q (u-blox M10) — UART Host Driver Spec & Requirements

Target: a robust, idiomatic Rust driver for the **u-blox SAM-M10Q** GNSS antenna module over **UART**, focusing on transport and message framing (UBX + NMEA). [file:32]

> This document is based on the SAM-M10Q datasheet (UBX-22013293 R05). It does **not** replace the u-blox M10 *Interface description*; for the full UBX protocol (message classes/IDs, configuration keys, etc.) consult the referenced u-blox document. [file:32]

## 1. Scope and assumptions

### In scope
- UART transport layer.
- Streaming decode of mixed **UBX** (binary) and **NMEA** (ASCII) on the same UART. [file:32]
- Minimal send helpers for UBX and NMEA.
- Optional integration with a GPIO reset pin.

### Out of scope (v1)
- Full UBX message catalog and configuration database.
- High-level PVT/nav solution model.

## 2. Hardware / electrical constraints (driver-relevant)

### Pins and functions (for documentation)
- UART pins: `TXD` (module output) and `RXD` (module input). [file:32]
- Reset: `RESET_N` is active low and must be held low for at least **1 ms** to trigger a reset. [file:32]
- `V_IO` defines digital IO levels. [file:32]

### Operating conditions (for system docs)
- `VCC`: 2.7–3.6 V. [file:32]
- `V_IO`: 2.7 V up to `VCC` (max 3.6 V). [file:32]

## 3. UART requirements

- Baud rate range: **9600 to 921600 bit/s**. [file:32]
- Hardware flow control: **not supported**. [file:32]
- Default UART settings: **9600 baud, 8 data bits, no parity, 1 stop bit (8-N-1)**. [file:32]
- Default protocol configuration on UART:
  - Input messages: **NMEA and UBX**.
  - Output messages: NMEA **GGA, GLL, GSA, GSV, RMC, VTG, TXT**. [file:32]

## 4. Supported protocols (UART payload)

- UBX: input/output, binary, u-blox proprietary. [file:32]
- NMEA: versions 2.1, 2.3, 4.0, 4.10 and 4.11 (default). [file:32]

## 5. Driver architecture (Rust)

### 5.1 Design goals
- `no_std` friendly.
- Works with any HAL UART implementation via `embedded-io` / `embedded-hal`-style traits (exact trait choice is up to your stack; keep it generic).
- Incremental decode: accept arbitrary byte chunks and emit frames.

### 5.2 Core types
- `struct SamM10q<UART> { uart: UART, decoder: Decoder, /* optional config */ }`
- `enum Frame<'a> { Ubx(&'a [u8]), Nmea(&'a [u8]) }`

### 5.3 Decoder requirements
Implement a streaming state machine:
- UBX framing:
  - Detect UBX sync bytes.
  - Read header, payload length, payload bytes.
  - Verify UBX checksum.
- NMEA framing:
  - Detect `$` start.
  - Read until `\r\n`.
  - Verify optional `*hh` checksum if present.

The decoder **MUST** handle:
- Frame split across reads.
- Garbage bytes between frames.
- Back-to-back UBX and NMEA frames.

### 5.4 Sending
- `send_ubx(payload)` helper that wraps with sync/header/length/checksum.
- `send_nmea(sentence)` helper that ensures `\r\n` line ending.

### 5.5 Error handling
Define `enum Error<E>`:
- `Uart(E)`
- `Checksum`
- `FrameTooLarge`
- `Timeout` (if providing blocking reads with deadlines)
- `Utf8` (only if you choose to parse NMEA as UTF-8 string; otherwise keep it as bytes)

## 6. Functional requirements

### 6.1 Bring-up API
**MUST**
- `poll()`-style method that reads from UART and returns `Option<Frame>` when a full frame is available.
- Ability to send raw bytes.

**SHOULD**
- Provide a convenience constructor for datasheet defaults (9600 8N1), while letting the application configure UART externally. [file:32]

### 6.2 Reset handling
**MUST**
- Document reset timing: `RESET_N` low for >= 1 ms. [file:32]

**SHOULD**
- Optional `reset(&mut self, reset_pin, delay)` method, where `reset_pin` is a user-provided GPIO and `delay` can wait >=1 ms.

## 7. Test requirements

**MUST**
- Unit tests for:
  - UBX checksum verification.
  - NMEA checksum verification (when present).
  - Chunk boundary splitting (frame assembled from multiple reads).
  - Mixed stream (NMEA then UBX then NMEA).

**SHOULD**
- Golden test vectors that reflect the datasheet default NMEA outputs (GGA/GLL/GSA/GSV/RMC/VTG/TXT). [file:32]

## 8. References
- SAM-M10Q Data sheet, UBX-22013293 R05. [file:32]
- u-blox M10 SPG 5.10 Interface description (UBX-21035062) for UBX protocol/message definitions. [file:32]
