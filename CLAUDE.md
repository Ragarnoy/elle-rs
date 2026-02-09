# Elle Flight Controller

## Project Structure

Cargo workspace with embedded firmware and host tooling:

- **`crates/elle-eagle/`** — Main firmware binary (RP2350, Embassy async, `no_std`)
- **`crates/elle-system/`** — System-level firmware support (RPC transport, supervisor, etc.)
- **`crates/elle-rpc-icd/`** — Shared RPC Interface Control Document (types, endpoints, topics)
- **`crates/elle-hardware/`** — Hardware drivers (IMU, LED, PWM, SBUS, flash)
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
cargo build --release                          # default features (SBUS flight mode)
cargo build --release --features rpc-control   # with RPC server (ground test mode)
cargo run --release --features rpc-control     # flash via probe-rs
```

Key feature flags for elle-eagle:
- `rpc-control` — postcard-RPC server over RTT (ground test mode)
- `defmt-logging` — defmt log output (default)
- `performance-monitoring` — Timing instrumentation
- `ulog-logging` — Flash-based flight data recording
- `gnss` — SAM-M10Q GNSS receiver support (requires `rpc-control`)
- `crsf-telemetry` — CRSF telemetry TX to radio via PIN_20/UART1 TX (attitude, flight mode, GPS)
- `imu-save-calibration` — Persist IMU calibration to flash (default)
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
- **Dispatch**: `crates/elle-eagle/src/rpc_app.rs` — `define_dispatch!` with `ElleApp` type, `RpcContext`, and 17 blocking handler functions
- **Server task**: `crates/elle-eagle/src/main.rs` `rpc_server_task()` — Creates `ElleApp`, runs `Server::new().run()` loop

### RPC Protocol

17 endpoints + 1 outgoing topic defined in the ICD:

**Endpoints** (request/response):
- Control: SetThrottle, SetElevons, SetControlMode
- Safety: Arm, Disarm, EmergencyStop
- Trim/Cal: AdjustTrim, SaveCalibration, ClearCalibration
- Query: GetStatus, GetAttitude, GetPerformance, ResetPerformance, GetMagnetometer, GetGnss
- System: Ping, GetVersion

**Topics** (device -> host, streaming):
- LogTopic — device-side log messages (level + numeric code)

### Shared State

- **`FlightState`** (`crates/elle-eagle/src/flight_state.rs`) — Signal carrying `{ armed, failsafe, mode }`, published by the RPC control loop after each `fc.update()`, read by `handle_get_status` in `rpc_app.rs`
- **`RpcCommand`** (`crates/elle-eagle/src/rpc_handlers.rs`) — Enum + channel for RPC handler → main loop communication
- **`LogMsg`** (`crates/elle-eagle/src/log_channel.rs`) — Channel for firmware events → `log_publisher_task` → LogTopic

### Host Tool (`tools/elle-rpc-host/`)

Two modes:
- **Default (no subcommand)**: TUI monitoring dashboard with polled attitude/status/mag/gnss + log streaming
- **`direct <cmd>`**: Single RPC commands for scripting

Key modules:
- `probe.rs` — probe-rs connection, RTT attach, blocking I/O worker thread
- `wire.rs` — WireTx/WireRx/WireSpawn bridging tokio mpsc channels to HostClient
- `tui/` — ratatui dashboard (mod.rs event loop, state.rs, ui.rs, commands.rs)
- `direct.rs` — Single-command mode using HostClient

TUI polling rates: attitude 10Hz, status 0.5Hz, magnetometer 5Hz, GNSS 1Hz.

**Important**: Host `probe.rs` reads from RTT up channel 1 (index 1), not channel 0 (which is defmt).

### Logging Systems

Three independent logging systems coexist, each serving a different purpose:

| System | Transport | Rate | Persistence | Audience |
|--------|-----------|------|-------------|----------|
| **defmt** | RTT channel 0 | Event-driven | Only if host captures | Developer at debug probe |
| **RPC LogTopic** | RTT channel 1 (postcard-RPC) | Event-driven (6 call sites) | No — streaming | Host TUI dashboard |
| **ULog** | Flash storage | 77Hz attitude+commands, 7.7Hz status | Yes — survives power loss | Post-flight analysis |

- defmt macros (`info!`, `warn!`, etc.) are always compiled in; the transport (`defmt-rtt`) is gated on `defmt-logging` (default on). Without the transport, macros become no-ops.
- RPC LogTopic carries `(level: u8, code: u16)` — numeric event codes mapped to strings on the host side in `tui/ui.rs::log_code_text()`.
- ULog records full-fidelity flight data (attitude, commands, status) to flash via `elle-hardware::ULogLogger`, gated on `ulog-logging`.

### Control Loop Architecture

The firmware main loop runs at 77Hz (13ms ticker):

**SBUS mode** (default, no `rpc-control`):
1. Reads SBUS commands from dedicated receiver task
2. Updates FlightController with attitude + pilot commands
3. Failsafe check, LED pattern updates

**RPC mode** (`rpc-control` feature):
1. Reads RPC commands from `RPC_CMD_CHANNEL`
2. Builds `PilotCommands::Normalized` from accumulated RPC state
3. Updates FlightController with attitude data from IMU
4. Publishes `FlightState` signal for RPC query handlers
5. Periodic LED pattern updates

RPC handlers send commands to the main loop via `RPC_CMD_CHANNEL` — they never directly control hardware.

## Key Dependencies

| Crate | Firmware | Host | Purpose |
|-------|----------|------|---------|
| postcard-rpc 0.12 | server (define_dispatch!) | host_client | RPC framework |
| embassy-* (git) | yes | - | Async embedded runtime |
| probe-rs 0.30 | - | yes | Debug probe + RTT access |
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

### Known TODOs in Firmware
- `RpcCommand::SetMode`: not implemented (TODO in main loop)
- `RpcCommand::AdjustTrim`: logged but not implemented
- `RpcCommand::SaveCalibration` / `ClearCalibration`: logged but not implemented
- **`disable-imu` stub generates synthetic test data** (slow sine waves) — remove once ICM42686P is connected
- I2C0 bus plan: MMC5616WA (mag) + BMP390 (baro, driver not yet written). ICM42686P (IMU) will use SPI.

### Next Steps
1. **Test end-to-end** — Flash firmware with `rpc-control,disable-imu`, run host TUI, verify mag data in TUI
2. **Implement remaining RPC commands** — Mode switching, trim adjust, calibration save/clear
3. **Wire MMC5616WA into real IMU path** — when ICM42686P is on SPI, I2C0 still available for mag reading in the real `BnoImu::run()` loop
4. **BMP390 barometer driver** — second I2C0 device, will need shared bus (I2C bus mutex or separate task)
