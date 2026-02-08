# MMC5616WA (MEMSIC) — Rust I2C Driver Spec & Requirements

Target: a robust, idiomatic Rust driver for the **MEMSIC MMC5616WA** 3-axis AMR magnetometer, **I2C only**. [file:1]

## 1. Goals / non-goals

### Goals
- Provide blocking (and, potentially non blocking) APIs for:
  - One-shot magnetic measurement (XYZ), optional SET/RESET compensation. [file:1]
  - One-shot temperature measurement. [file:1]
  - Continuous mode start/stop + configuration (ODR, BW). [file:1]
  - Device identity reads (Chip ID, Product ID). [file:1]
- Follow common `embedded-hal` driver structure: HAL-agnostic, testable with mocks, `no_std` friendly. [web:12][web:15]
- Correct register protocol: write register address then read sequential bytes. [file:1]

### Non-goals (v1)
- I3C support. [file:1]
- Motion detector / factory-use bits. [file:1]
- FIFO interrupt usage (datasheet notes Data_ready interrupt is I3C-only). [file:1]

## 2. Datasheet facts (I2C-relevant)

- I2C slave supports Fast-mode up to **400 kHz**. [file:1]
- Device 7-bit address shown as `0110000` (= **0x30**), with up to 8 factory-programmable address variants. [file:1]
- Power-on: device operable after **tOp = 5 ms** once VDD is valid. [file:1]
- Minimum interval between SET/RESET and other operations: **tSR = 1 ms**. [file:1]
- Measurement time `tTM` depends on BW bits:
  - BW=00: 6.6 ms; BW=01: 3.5 ms; BW=10: 2.0 ms; BW=11: 1.2 ms. [file:1]
- Sensitivity (20-bit): **16384 counts/G**; Null field output (20-bit): **524288 counts**. [file:1]
- Temperature output `Tout`: unsigned, `0` corresponds to **-75°C**, ~**0.8°C/LSB**. [file:1]

## 3. Register map (subset)

### Outputs
- `Xout0..1`: `0x00..0x01` (X[19:4]). [file:1]
- `Yout0..1`: `0x02..0x03` (Y[19:4]). [file:1]
- `Zout0..1`: `0x04..0x05` (Z[19:4]). [file:1]
- `Xout2`: `0x06` (X[3:0] in [7:4], fifo_usage in [3:0]). [file:1]
- `Yout2`: `0x07` (Y[3:0] in [7:4], fifo flags in [1:0]). [file:1]
- `Zout2`: `0x08` (Z[3:0] in [7:4]). [file:1]
- `Tout`: `0x09`. [file:1]

### Status
- `Status1`: `0x18` includes `Meas_m_done` and `Meas_t_done`; `Meas_m_done` resets when any magnetic output reg is read, `Meas_t_done` resets when `Tout` is read. [file:1]

### Control
- `ODR`: `0x1A`. [file:1]
- `Internal Control 0`: `0x1B` (TM_M, TM_T, Do Set, Do Reset, Auto_SR_en, Cmm_freq_en, …). [file:1]
- `Internal Control 1`: `0x1C` (BW1:BW0, Sw_reset, …). [file:1]
- `Internal Control 2`: `0x1D` (Cmm_en, periodic set fields, …). [file:1]
- `Chip ID`: `0x21` (Rev I -> 0xD2 per doc). [file:1]
- `Product ID`: `0x39`. [file:1]

## 4. Rust driver design (good practices)

### 4.1 Traits and generics
- Use `embedded-hal` I2C traits (blocking in v1). [web:15]
- Provide `new(i2c, address)` and `destroy()` returning the bus.
- Address must be configurable (default constant `DEFAULT_ADDR = 0x30`). [file:1]

### 4.2 Errors
Define `enum Error<E>`:
- `I2c(E)`
- `Timeout`
- `BadParam`
- `InvalidChipId(u8)` (optional helper)

### 4.3 Register access
Provide low-level helpers:
- `write_reg(reg, val)`
- `read_reg(reg) -> u8`
- `read_regs(start, buf)`

Use read-modify-write (or a shadow cache) for control registers to avoid clobbering unrelated bits. [web:2][web:4]

### 4.4 Control register caching
Maintain a shadow of writable control registers to make RMW safe and reduce bus reads:
- `odr (0x1A)`, `ctrl0 (0x1B)`, `ctrl1 (0x1C)`, `ctrl2 (0x1D)`. [file:1]

### 4.5 Timing / delays
- Provide `init(delay)` enforcing `tOp >= 5ms`, or document caller responsibility. [file:1]
- Enforce `tSR >= 1ms` when SET/RESET is used. [file:1]
- Prefer polling status bits with timeout; delay-only mode should be documented using BW-dependent `tTM`. [file:1]

## 5. Functional requirements (MUST/SHOULD)

### 5.1 Initialization
**MUST**
- `new(i2c, addr)` and `destroy()`
- `init(delay)` (or explicit documentation) for `tOp`. [file:1]

**SHOULD**
- `validate()` reading `Chip ID (0x21)` and optionally checking Rev I value. [file:1]

### 5.2 One-shot magnetic measurement (20-bit)
**MUST**
- Trigger measurement by writing `Internal Control 0 (0x1B)` with `Take_meas_M=1` and commonly `Auto_SR_en=1` (datasheet example uses `0b0010_0001`). [file:1]
- Poll `Status1 (0x18)` until `Meas_m_done` is set. [file:1]
- Burst-read `0x00..0x08` and reconstruct 20-bit X/Y/Z. [file:1]

**MUST — 20-bit reconstruction**
- X = `(x0<<12) | (x1<<4) | (x2>>4)` where `x2` is `Xout2`. [file:1]
- Y = `(y0<<12) | (y1<<4) | (y2>>4)` where `y2` is `Yout2`. [file:1]
- Z = `(z0<<12) | (z1<<4) | (z2>>4)` where `z2` is `Zout2`. [file:1]

**SHOULD**
- Provide signed counts helper using Null field code: `signed = raw20 as i32 - 524_288`. [file:1]
- Provide conversion: `gauss = signed_counts / 16384.0`. [file:1]

### 5.3 SET/RESET compensated measurement
**SHOULD**
- Implement optional SET→MEAS, RESET→MEAS, then `H = (Out1 - Out2)/2` to remove offset. [file:1]
- Respect `tSR` between SET/RESET and subsequent operations. [file:1]

### 5.4 Temperature
**MUST**
- Trigger `Take_meas_T` in `0x1B`, poll `Meas_t_done`, read `Tout (0x09)`. [file:1]

**SHOULD**
- Convert to °C: `temp_c ≈ -75.0 + 0.8 * Tout`. [file:1]

### 5.5 Continuous mode
**MUST**
- `set_odr(1..=255)` writes `ODR (0x1A)`; `0` means not active. [file:1]
- Start sequence: write ODR, set `Cmm_freq_en` in `0x1B`, set `Cmm_en` in `0x1D`. [file:1]
- Stop: clear `Cmm_en`. [file:1]

### 5.6 Bandwidth
**MUST**
- Set `BW1:BW0` in `0x1C`. [file:1]
- Document `tTM` table per BW. [file:1]

### 5.7 Soft reset
**MUST**
- `soft_reset()` sets `Sw_reset` in `0x1C`. [file:1]
- Document: clears regs and re-reads OTP; datasheet mentions 20 ms power-on time after SW reset. [file:1]

## 6. Patterns borrowed from MMC5983-rs (STYLE ONLY)

The following are recommended *structural* patterns inspired by the MMC5983-rs Rust driver, **without copying any device-specific behavior**. [page:1]

- Split modules: `interface.rs` (I2C primitives), `registers.rs` (addresses/masks), `types.rs` (domain types), `mode.rs` (config enums), `error.rs` (error types). [page:1]
- Keep a single device struct that owns the bus and provides high-level methods (trigger/ready/read style). [page:1]

### Guardrail
- **Do not** reuse MMC5983 register addresses, bitfields, scaling constants, or measurement sequences; MMC5616WA datasheet is authoritative for all hardware behavior. [file:1]

## 7. Test requirements

**MUST**
- Unit tests for 20-bit reconstruction and signed conversion around the null-field code. [file:1]

**SHOULD**
- Transaction-level tests using an I2C mock verifying:
  - write `0x1B` to trigger,
  - poll `0x18`,
  - read `0x00..0x08` in one burst read. [file:1]

## 8. References
- MEMSIC MMC5616WA datasheet (rev 01/2024). [file:1]
- embedded-hal docs (traits and ecosystem). [web:15]
- Embedded Rust Book (HAL patterns). [web:12]
- MMC5983-rs crate docs (structure inspiration only). [page:1]
