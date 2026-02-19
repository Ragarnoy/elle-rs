# Elle Flight Controller

## Project Structure

Cargo workspace with embedded firmware and host tooling:

- **`crates/elle-eagle/`** — Main firmware binary (RP2350, Embassy async, `no_std`)
- **`crates/elle-system/`** — System-level firmware support (RPC transport, supervisor, etc.)
- **`crates/elle-rpc-icd/`** — Shared RPC Interface Control Document (types, endpoints, topics)
- **`crates/elle-hardware/`** — Hardware drivers (IMU, LED, PWM, CRSF, flash)
- **`crates/elle-control/`** — Flight control algorithms
- **`crates/elle-config/`** — Configuration constants
- **`crates/elle-error/`** — Error types
- **`crates/elle-ulog/`** — ULog flight data recording
- **`tools/elle-rpc-host/`** — Host CLI tool (TUI dashboard + direct probe commands)

## Building

### Firmware

Target is RP2350 (`thumbv8m.main-none-eabihf`), configured in `.cargo/config.toml`.

```sh
cd crates/elle-eagle
cargo build --release                          # default features (CRSF/ELRS flight mode)
cargo build --release --features rpc-control   # with RPC server (ground test mode)
cargo run --release --features rpc-control     # flash via probe-rs
```

Key feature flags for elle-eagle:
- `rpc-control` — postcard-RPC server over RTT (ground test mode)
- `defmt-logging` — defmt log output (default)
- `performance-monitoring` — Timing instrumentation
- `ulog-logging` — Flash-based flight data recording
- `gnss` — SAM-M10Q GNSS receiver support (requires `rpc-control`)
- `crsf-telemetry` — CRSF telemetry TX to radio via PIN_20/UART1 TX (attitude, flight mode, GPS, baro altitude)
- `imu-save-calibration` — Persist IMU calibration to flash (default, currently no-op with ICM-42686)
- `legacy-ctrl` — Legacy mixing functions (mutually exclusive with default mixing)

### Host Tool

Must specify target explicitly (workspace `.cargo/config.toml` defaults to thumbv8m):

```sh
cargo build -p elle-rpc-host --target x86_64-unknown-linux-gnu
```

## Architecture

### RPC Transport (postcard-RPC over RTT)

RTT channel layout:
- **Up channel 0**: defmt logs (NoBlockSkip)
- **Up channel 1**: RPC TX — firmware to host (BlockIfFull, COBS encoded)
- **Down channel 0**: RPC RX — host to firmware (NoBlockSkip, COBS encoded)

Transport implementation: `crates/elle-system/src/rpc.rs`
- `RttTx` — WireTx impl using blocking_mutex + double-buffered COBS encoding
- `RttRx` — WireRx impl with COBS frame reassembly, polling with 100us yield
- `ElleWireSpawn` — Minimal WireSpawn stub (all handlers are blocking)
- `init_rtt_rpc()` — Sets up RTT channels and returns `RttChannels { tx, rx }`

### RPC Dispatch (define_dispatch! macro)

The firmware uses postcard-rpc's `define_dispatch!` macro for type-safe dispatch.
No extra postcard-rpc features needed — the macro works with elle's own WireTx/WireRx.

- **ICD**: `crates/elle-rpc-icd/src/lib.rs` — Uses `endpoints!`/`topics!` macros generating `ENDPOINT_LIST`, `TOPICS_IN_LIST`, `TOPICS_OUT_LIST`
- **Dispatch**: `crates/elle-eagle/src/rpc_app.rs` — `define_dispatch!` with `ElleApp` type, `RpcContext`, and 25 blocking handler functions
- **Server task**: `crates/elle-eagle/src/main.rs` `rpc_server_task()` — Creates `ElleApp`, runs `Server::new().run()` loop

### RPC Protocol

25 endpoints + 1 outgoing topic defined in the ICD:

**Endpoints** (request/response):
- Control: SetThrottle, SetElevons, SetControlMode, SetPidGains, SetAttitudeSetpoint
- Safety: Arm, Disarm, EmergencyStop
- Query: GetStatus, GetAttitude, GetPerformance, ResetPerformance, GetMagnetometer, GetBarometer, GetGnss, GetRcChannels, GetControllerOutput
- ULog: StartULog, StopULog, ReadULogChunk, EraseULog, GetULogInfo
- System: Ping, GetVersion, GetTime

**Topics** (device -> host, streaming):
- LogTopic — device-side log messages (level + numeric code)

### Shared State

- **`FlightState`** (`crates/elle-eagle/src/flight_state.rs`) — Signal carrying `{ armed, failsafe, mode }`, published by the RPC control loop after each `fc.update()`, read by `handle_get_status` in `rpc_app.rs`
- **`RpcCommand`** (`crates/elle-eagle/src/rpc_handlers.rs`) — Enum + channel for RPC handler → main loop communication (includes StartULog, StopULog, ReadULogChunk, PopAndPeekULog, EraseULog)
- **`BaroReading`** / **`BARO_SIGNAL`** (`crates/elle-hardware/src/imu.rs`) — BMP390 barometer data (pressure, temperature, altitude), polled at ~2 Hz on Core1
- **`LogMsg`** (`crates/elle-eagle/src/log_channel.rs`) — Channel for firmware events → `log_publisher_task` → LogTopic
- **`ULOG_ENABLED`** (`crates/elle-eagle/src/rpc_app.rs`) — AtomicBool flag controlling ULog recording in RPC mode. Set by StartULog/StopULog RPC commands.
- **`ULOG_STATE`** / **`ULOG_ITEM_SIGNAL`** (`crates/elle-eagle/src/rpc_app.rs`) — Atomic state machine (IDLE→READING→READY→EMPTY) + Signal for ULog extraction, bridging blocking RPC handlers to async flash operations

### Host Tool (`tools/elle-rpc-host/`)

Two modes:
- **Default (no subcommand)**: TUI monitoring dashboard with polled attitude/status/mag/baro/gnss + log streaming
- **`direct <cmd>`**: Single RPC commands for scripting

Key modules:
- `probe.rs` — probe-rs connection, RTT attach, blocking I/O worker thread
- `wire.rs` — WireTx/WireRx/WireSpawn bridging tokio mpsc channels to HostClient
- `tui/` — ratatui dashboard (mod.rs event loop, state.rs, ui.rs, commands.rs)
- `direct.rs` — Single-command mode using HostClient

TUI polling rates: attitude 10Hz, status 0.5Hz, magnetometer 5Hz, barometer 1Hz, GNSS 1Hz.

**Important**: Host `probe.rs` reads from RTT up channel 1 (index 1), not channel 0 (which is defmt).

### Logging Systems

Three independent logging systems coexist, each serving a different purpose:

| System | Transport | Rate | Persistence | Audience |
|--------|-----------|------|-------------|----------|
| **defmt** | RTT channel 0 | Event-driven | Only if host captures | Developer at debug probe |
| **RPC LogTopic** | RTT channel 1 (postcard-RPC) | Event-driven (6 call sites) | No — streaming | Host TUI dashboard |
| **ULog** | Flash storage | 77Hz attitude+commands, 7.7Hz status | Yes — survives power loss | Post-flight analysis |

- defmt macros (`info!`, `warn!`, etc.) are always compiled in; the transport (`defmt-rtt`) is gated on `defmt-logging` (default on). Without the transport, macros become no-ops.
- RPC LogTopic carries `(level: u8, code: u16)` — numeric event codes mapped to strings on the host side in `tui/ui.rs::log_code_text()`. ULog-related codes: 30=recording started, 31=init failed, 32=not compiled in, 33=recording stopped, 34=flash erased.
- ULog records full-fidelity flight data (attitude, commands, status) to flash via `elle-hardware::ULogLogger`, gated on `ulog-logging`. Recording is explicitly started/stopped — in RPC mode via `ulog start`/`ulog stop` TUI commands, in flight mode via RC aux channel switch (CH7, threshold 1500).

### Control Loop Architecture

The firmware main loop runs at 77Hz (13ms ticker):

**Flight mode** (default, no `rpc-control`):
1. Reads CRSF/ELRS commands from dedicated receiver task
2. Updates FlightController with attitude + pilot commands
3. Failsafe check, LED pattern updates
4. ULog recording controlled by RC aux channel switch (edge detection, CH7 > 1500 = on)

**RPC mode** (`rpc-control` feature):
1. Reads RPC commands from `RPC_CMD_CHANNEL`
2. Builds `PilotCommands::Normalized` from accumulated RPC state
3. Updates FlightController with attitude data from IMU
4. Publishes `FlightState` signal for RPC query handlers
5. Periodic LED pattern updates
6. ULog recording gated on `ULOG_ENABLED` flag (set via StartULog/StopULog RPC commands)
7. Handles ULog extraction commands (ReadULogChunk, PopAndPeekULog, EraseULog) via `FLASH_REQUEST_SIGNAL`

RPC handlers send commands to the main loop via `RPC_CMD_CHANNEL` — they never directly control hardware.

**Important**: ULog extraction uses `FLASH_REQUEST_SIGNAL` which is single-valued. Recording must be stopped before extraction to avoid signal contention. The host `ulog extract` command auto-sends `StopULog` first.

## Key Dependencies

| Crate | Firmware | Host | Purpose |
|-------|----------|------|---------|
| postcard-rpc 0.12 | server (define_dispatch!) | host_client | RPC framework |
| embassy-* (git) | yes | - | Async embedded runtime |
| probe-rs 0.30 | - | yes | Debug probe + RTT access |
| icm426xx 0.4 | yes | - | ICM-42686-P IMU driver (SPI, FIFO, 20-bit) |
| ahrs 0.8 | yes | - | Madgwick AHRS sensor fusion (no_std) |
| nalgebra 0.34 | yes | - | Linear algebra (no_std + libm) |
| bmp390 0.4 | yes (sync) | - | BMP390 barometer driver |
| embedded-hal-bus 0.2 | yes | - | I2C/SPI bus sharing (RefCellDevice, ExclusiveDevice) |
| cobs 0.5 | yes | yes | Frame encoding |
| ratatui 0.30 | - | yes | TUI dashboard |
| rtt-target 0.6 | yes | - | RTT channel API |

## Recent Work / Resume Points

### Completed
- Field-readiness cleanup: removed `rtt-control`, `mag-test`, `disable-imu`, `error-strings` features
- Removed `TelemetryTopic` from ICD (polling is sufficient for TUI; avoids BlockIfFull backpressure)
- TUI rewired from dead telemetry subscription to polled `GetAttitudeEndpoint` at 10Hz
- Extracted inline modules to files: `rpc_handlers.rs`, `log_channel.rs`, `flight_state.rs`
- `FlightState` signal wired into RPC control loop and `handle_get_status`
- `handle_get_performance` reads real `PERFORMANCE_MONITOR` data (cfg-gated)
- `handle_reset_performance` actually calls `pm.reset_all()`
- `TimingMeasurement` + timing helpers exported unconditionally from `elle_system` (no-op stubs when `performance-monitoring` disabled)
- Supervisor simplified: removed RTT join4 path, always join3
- TUI monitoring dashboard with polled attitude/status/mag/gnss + log streaming
- Direct mode shares probe.rs/wire.rs with TUI
- Firmware dispatch via `define_dispatch!` macro
- ICD uses batch `endpoints!`/`topics!` macros
- CRSF telemetry TX: attitude, flight mode, GPS frames to radio via PIN_20 (feature `crsf-telemetry`)
- CRSF telemetry log events forwarded to RPC LogTopic (codes 20-23) for TUI visibility
- Host TUI/direct mode: proper RTT worker shutdown via `AtomicBool` flag + `JoinHandle::join()`
- `CrsfReceiver::new()` refactored to accept `UartRx` (UART split for TX telemetry)
- MMC5616WA magnetometer wired into `disable-imu` stub — reads at ~10 Hz, populates `MAG_SIGNAL` for TUI/RPC. Always-on (no feature gate), the chip is physically on the board.
- BMP390 barometer driver integrated via `embedded-hal-bus::RefCellDevice` for I2C0 bus sharing with MMC5616WA. Polls at ~2 Hz, populates `BARO_SIGNAL`. Init tries both addresses (0x77, 0x76). Always-on (no feature gate).
- I2C bus sharing: I2C0 wrapped in `RefCell` + `StaticCell`, creates `RefCellDevice` handles for MMC5616WA and BMP390. Safe because both run in a single task on Core1.
- CRSF telemetry expanded to 4 slots: attitude → flight_mode → GPS → baro (~12.5 Hz each at 50 Hz tick)
- `GetBarometerEndpoint` RPC endpoint returns pressure (hPa), temperature (°C), barometric altitude (m)
- Host TUI displays barometer data (pressure, temperature, altitude) at 1 Hz poll rate
- **ICM-42686-P IMU integrated via SPI0** — replaced BNO055 (I2C) with ICM-42686 (SPI) + Madgwick AHRS sensor fusion. Renamed `BnoImu` → `Imu`. Removed `bno055`/`mint` deps, added `icm426xx`/`ahrs`/`nalgebra`.
- **AHRS sensor fusion**: `ahrs` crate (Madgwick filter) fuses ICM accel+gyro at 1 kHz with MMC5616WA mag at 10 Hz for 9-DOF attitude estimation. Falls back to 6-DOF (no mag) until first mag reading.
- **SPI0 pin assignments**: MISO=PIN_0, CS=PIN_1, SCLK=PIN_2, MOSI=PIN_3, INT1=PIN_5 (unused — FIFO polled)
- **Blocking SPI on Core1**: Uses blocking SPI (polled, no DMA) since DMA interrupt handlers are registered on Core0's NVIC. 24-byte FIFO read at 1 MHz SPI takes ~200µs.
- **I2C bus always RefCell-wrapped**: Both real and stub paths now use `RefCell<I2c>` for I2C0, since mag+baro share the bus.

### Known TODOs in Firmware
- **`disable-imu` stub generates synthetic test data** (slow sine waves) — for debugging without ICM-42686 hardware
- ~~BMP390 hardware issue~~ — **resolved**: resoldered, now working on I2C0.
- **Axis mapping**: ICM-42686 → AHRS Euler angles may need sign adjustment depending on chip orientation on PCB. Start with identity mapping, verify in TUI.
- **`imu-save-calibration` feature**: Now a no-op (ICM-42686 is factory-calibrated, no user calibration to persist). Flash calibration infrastructure remains for potential future use.

### ICM-42686-P resoldered and functional

**Status**: ICM-42686-P has been resoldered and is working on SPI0. Software integration complete, compiles on all feature combos.

**Hardware verification TODO**:
1. Flash: `cargo run -p elle-eagle --release --features rpc-control,crsf-telemetry`
2. Expected log: `ICM-42686: initialized (WHO_AM_I OK, 1 kHz ODR)` — WHO_AM_I should be `0x44`
3. Verify attitude in TUI (pitch/roll/yaw should track board motion)
4. **Axis mapping**: if pitch/roll are swapped or inverted, adjust signs in `imu.rs` `run()` loop (search for "axis sign adjustment")
5. Verify mag + baro still work alongside ICM (shared I2C0 bus via RefCell)

### ULog Extraction & Erasure via RPC

**Status**: Implemented. ULog data can be extracted from flash, recording can be started/stopped, and flash can be erased via TUI commands.

TUI commands:
- `ulog start` — start ULog recording (initializes logger on first call, sets `ULOG_ENABLED`)
- `ulog stop` — stop ULog recording (flushes buffer, clears `ULOG_ENABLED`)
- `ulog extract [file]` — downloads all queued ULog data (auto-stops recording first, fragments 4KB queue items into 512B RPC chunks, default filename `flight_YYYYMMDD_HHMMSS.ulg`)
- `ulog erase` — erases entire ULog flash region (0x210000–0xFFFFFF, auto-stops recording)

Architecture:
- Flash manager migrated to sequential-storage v7.1.0 API (QueueStorage/MapStorage structs with destroy() for ownership recovery)
- RPC handlers use atomic state machine (IDLE→READING→READY→EMPTY) bridging blocking dispatch to async flash operations
- Extraction runs as a background task in the TUI (non-blocking UI)
- Flight mode: RC aux channel (CH7, `ULOG_ENABLE_CH`) controls recording via edge detection (high ~2047 = on)

### Next Steps
1. **Verify ICM-42686 on hardware** — test attitude in TUI and CRSF telemetry
2. **Tune AHRS** — adjust Madgwick beta (currently 0.033), verify axis mapping
3. **Test ULog start/stop/extract/erase on hardware** — record data via RC switch or TUI command, extract via TUI, verify .ulg file
4. **Explore async SPI** — If IMU task moves to Core0, or DMA cross-core routing is confirmed, enable `async` feature on `icm426xx` for DMA-based SPI
