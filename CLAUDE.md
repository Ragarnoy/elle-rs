# Elle Flight Controller

## Project Structure

Cargo workspace with embedded firmware and host tooling:

- **`crates/elle-eagle/`** — Main firmware binary, twin-engine flying wing (RP2350, Embassy async, `no_std`)
- **`crates/elle-dart/`** — Dart firmware binary, single-engine platform (same stack, `single-engine` feature)
- **`crates/elle-app/`** — The application both binaries run: flight and RPC control loops, RPC dispatch, boot sequence, ULog per-tick logging, shared tasks. The binaries keep only pins, engine setup, `bind_interrupts!`, `led_task` and `main()` — **make behaviour changes here, once**, not in a binary.
- **`crates/elle-system/`** — System-level firmware support (RPC transport, supervisor, etc.)
- **`crates/elle-rpc-icd/`** — Shared RPC Interface Control Document (types, endpoints, topics)
- **`crates/elle-hardware/`** — Hardware drivers (IMU, LED, PWM, CRSF, flash)
- **`crates/elle-control/`** — Flight control algorithms
- **`crates/elle-config/`** — Configuration constants
- **`crates/elle-error/`** — Error types
- **`crates/elle-ulog/`** — ULog flight data recording
- **`crates/elle-nav/`** — Navigation math placeholder (not yet a workspace member)
- **`drivers/`** — Vendored sensor drivers: `mmc5616wa` (mag), `sam-m10q` (GNSS), `bmp390` (baro, patched over crates.io)
- **`tools/elle-rpc-host/`** — Host CLI tool (TUI dashboard + direct probe commands)

## Building

### Firmware

Target is RP2350 (`thumbv8m.main-none-eabihf`), configured in `.cargo/config.toml`.

```sh
cd crates/elle-eagle
cargo build --release                                              # default features (CRSF/ELRS flight mode)
cargo build --release --no-default-features --features rpc-control,gnss    # with RPC server (ground test mode)
cargo build --release --no-default-features --features rpc-control,rpc-rc,gnss # RPC monitoring + RC flight control
cargo run --release --no-default-features --features rpc-control,gnss      # flash via probe-rs
```

**Do not omit `gnss` from RPC builds.** It is in `default`, and
`--no-default-features` drops it — the GNSS task is then never compiled or
spawned, so the TUI shows no satellites and no GNSS log lines at all, which
looks exactly like a hardware or reception failure.

Key feature flags (both binaries; they forward to `elle-app`):
- `rpc-control` — postcard-RPC server over RTT (ground test mode). **Mutually exclusive with `defmt-logging`** (both define `_SEGGER_RTT`); build with `--no-default-features --features rpc-control`. Enforced via `compile_error!`.
- `rpc-rc` — RC/CRSF flight control in RPC mode. When combined with `rpc-control`, pilot commands come from the RC transmitter instead of RPC accumulators. The TUI still provides full monitoring. **Requires `rpc-control`** (enforced via `compile_error!` in `elle-app`). Mitigates a build-specific DShot PIO issue in the RPC binary (see memory). Available on the dart too since the `elle-app` split.
- `defmt-logging` — defmt log output (default)
- `performance-monitoring` — Timing instrumentation
- `gnss` — SAM-M10Q GNSS receiver support (in `default`, both eagle and dart; works in both flight and RPC modes; provides ULog GPS logging + CRSF telemetry GPS frames). **RPC builds use `--no-default-features`, so `gnss` must be listed explicitly or there is no GNSS task at all.** Enables `elle-hardware/gnss`, which carries the shared `gnss` module.
- `gyro-raw-log` — bench vibration capture: every 1 kHz gyro sample (unfiltered, bias-corrected, airframe frame) to ULog as `gyro_raw`, to size `GYRO_RATE_LPF_HZ` from a real spectrum. ~25 kB/s more on the SD card; not for flight builds.
- CRSF telemetry TX is always compiled in (no feature gate) — attitude, flight mode, GPS, baro altitude, battery voltage/current to radio via PIN_20/UART1 TX

ULog flash recording is always compiled in (no feature gate). Recording is idle until explicitly started.

### Feature Powerset Check

Use `cargo hack` to verify all valid feature combinations compile:

```sh
cargo hack check -p elle-eagle --feature-powerset --exclude-features defmt-logging,default,rpc-rc --release
cargo hack check -p elle-dart  --feature-powerset --exclude-features defmt-logging,default,rpc-rc --release
```

Excluded features: `defmt-logging`/`default` (mutually exclusive with `rpc-control`), `rpc-rc` (requires `rpc-control`, not valid standalone). The `compile_error!` guards (`defmt-logging` × `rpc-control` in each `main.rs`, `rpc-rc` in `elle-app`) enforce these constraints.

### Host Tool

Must specify target explicitly (workspace `.cargo/config.toml` defaults to thumbv8m):

```sh
cargo build -p elle-rpc-host --target x86_64-unknown-linux-gnu
```

## Pin Map

| Pin | Function | Bus/Peripheral |
|-----|----------|----------------|
| PIN_0 | SPI0 MISO | ICM-42686 IMU |
| PIN_1 | SPI0 CS | ICM-42686 IMU |
| PIN_2 | SPI0 SCLK | ICM-42686 IMU |
| PIN_3 | SPI0 MOSI | ICM-42686 IMU |
| PIN_5 | IMU INT1 | DATA_RDY (async GPIO) |
| PIN_8 | I2C0 SDA | MMC5616WA mag + BMP390 baro |
| PIN_9 | I2C0 SCL | MMC5616WA mag + BMP390 baro |
| PIN_10 | WS2812B LED | PIO0 SM2 + DMA_CH2 |
| PIN_11 | DShot right engine | PIO2 |
| PIN_12 | Elevon right PWM | PWM slice 6 A |
| PIN_13 | Elevon left PWM | PWM slice 6 B |
| PIN_14 | DShot left engine | PIO1 |
| PIN_20 | UART1 TX | CRSF telemetry (DMA_CH4) |
| PIN_21 | UART1 RX | CRSF receiver (DMA_CH3) |
| PIN_23 | SD CARD_DETECT | GPIO input (active-low) |
| PIN_24 | SPI1 MISO | SD card |
| PIN_25 | SPI1 CS | SD card |
| PIN_26 | SPI1 SCK | SD card |
| PIN_27 | SPI1 MOSI | SD card |
| PIN_29 | UART0 RX | SAM-M10Q GNSS (BufferedUart/UART0_IRQ, feature `gnss`) |

DMA channels: CH1=Flash async, CH2=LED, CH3=CRSF RX, CH4=CRSF TX, CH5=SD SPI1 TX, CH6=SD SPI1 RX. CH0 and CH7 are free — the GNSS UART is interrupt-buffered (`BufferedUart`), not DMA-driven, because only the buffered variant implements `embedded-io-async`.

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
- **Dispatch**: `crates/elle-app/src/rpc_app.rs` — `define_dispatch!` with `ElleApp` type, `RpcContext`, and 34 blocking handler functions
- **Server task**: `crates/elle-app/src/tasks.rs` `rpc_server_task()` — Creates `ElleApp`, runs `Server::new().run()` loop

### RPC Protocol

34 endpoints + 1 outgoing topic defined in the ICD:

**Endpoints** (request/response):
- Control: SetThrottle, SetElevons, SetControlMode, SetPidGains, SetHeadingHold
- Safety: Arm, Disarm, EmergencyStop
- Query: GetStatus, GetAttitude, GetPerformance, ResetPerformance, GetMagnetometer, GetBarometer, GetGnss, GetRcChannels, GetControllerOutput, GetEngine
- ULog: StartULog, StopULog, ReadULogChunk, EraseULog, GetULogInfo
- Autotune: StartAutotune, AbortAutotune
- Mag calibration: StartMagCal, ClearMagCal, GetMagCal
- Level calibration: StartLevelCal, ClearLevelCal, GetLevelCal
- System: Ping, GetVersion, GetTime

**Topics** (device -> host, streaming):
- LogTopic — device-side log messages (level + numeric code)

### Shared State

**`SignalCache<T>`** (`crates/elle-hardware/src/signal_cache.rs`) — Generic wrapper combining `Signal<CriticalSectionRawMutex, T>` + `Mutex<CriticalSectionRawMutex, Cell<T>>` for publish/subscribe with non-consuming cache reads. Methods: `publish()`, `read_cached()`, `try_take()`. Used throughout the firmware for inter-task communication.

- **`ATTITUDE`** (`crates/elle-hardware/src/imu.rs`) — `SignalCache<AttitudeData>` — pitch/roll/yaw from AHRS at 1kHz, consumed by control loop via `try_take()`
- **`MAG`** (`crates/elle-hardware/src/imu.rs`) — `SignalCache<MagReading>` — magnetometer XYZ counts at 10Hz, read by CRSF telemetry and RPC via `read_cached()`
- **`BARO`** (`crates/elle-hardware/src/imu.rs`) — `SignalCache<BaroReading>` — pressure/temp/altitude at 2Hz, read by CRSF telemetry and RPC via `read_cached()`
- **`FLIGHT_STATE`** (`crates/elle-app/src/flight_state.rs`) — `SignalCache<FlightState>` — `{ armed, failsafe, mode, rc_age_ms }`, published by both control loops, read by `handle_get_status`
- **`CONTROLLER_OUTPUT`** (`crates/elle-app/src/flight_state.rs`) — `SignalCache<ControllerOutput>` — PID corrections/setpoints/servo μs, published by RPC control loop
- **`RpcCommand`** (`crates/elle-app/src/rpc_handlers.rs`) — Enum + channel for RPC handler → main loop communication (includes ULog, autotune, PID save commands)
- **`EVENT_CHANNEL`** (`crates/elle-hardware/src/event.rs`) — firmware events (`elle_event!`) → `log_publisher_task` (`crates/elle-app/src/tasks.rs`) → `LogMsg` on LogTopic
- **`ULOG_ENABLED`** (`crates/elle-app/src/rpc_app.rs`) — AtomicBool flag controlling ULog recording in RPC mode. Set by StartULog/StopULog RPC commands.
- **`DSHOT_THROTTLE`** (`crates/elle-hardware/src/dshot.rs`) — Signal carrying `(u32, u32)` eRPM target values from control loop to dedicated 1kHz DShot send task (governor converts to DShot commands)
- **`ENGINE_CACHE`** (`crates/elle-hardware/src/dshot.rs`) — Mutex-based non-consuming cache for `EngineReading` (`{ left: EngineUnitReading, right: EngineUnitReading }` — per-engine eRPM, throttle, validity, EDT temperature/voltage/current). Written by `dshot_task` at 1kHz, read by RPC handler, CRSF telemetry, and ULog logger.
- **`ULogState`** / **`ULOG_ITEM_SIGNAL`** (`crates/elle-app/src/rpc_app.rs`) — `#[repr(u8)]` enum state machine (Idle→Reading→Ready→Empty) + Signal for ULog extraction, bridging blocking RPC handlers to async flash operations

### Host Tool (`tools/elle-rpc-host/`)

Two modes:
- **Default (no subcommand)**: TUI monitoring dashboard with polled attitude/status/mag/baro/gnss + log streaming
- **`direct <cmd>`**: Single RPC commands for scripting

See `tools/elle-rpc-host/README.md` for the dashboard layout, the horizon widget's
`ROLL_SIGN` caveat, and the list of UI improvements still outstanding.

Key modules:
- `probe.rs` — probe-rs connection, RTT attach, blocking I/O worker thread
- `wire.rs` — WireTx/WireRx/WireSpawn bridging tokio mpsc channels to HostClient
- `tui/` — ratatui dashboard (mod.rs event loop, state.rs, ui.rs, commands.rs)
- `direct.rs` — Single-command mode using HostClient

TUI polling rates: attitude 10Hz, status 0.5Hz, magnetometer 5Hz, barometer 1Hz, GNSS 1Hz, engine 5Hz, RC channels 20Hz, controller output 10Hz.

**Important**: Host `probe.rs` reads from RTT up channel 1 (index 1), not channel 0 (which is defmt).

### GNSS (`crates/elle-hardware/src/gnss.rs`)

One shared task for both airframes, replacing the byte-identical `gnss_task` and
`gnss_signal` module that used to be duplicated in each `main.rs`.

- **Primary source: UBX-NAV-PVT** at 5 Hz — position, velocity NED, ground speed,
  course over ground, and `hAcc`/`vAcc`/`sAcc` accuracy estimates. Parsed by the
  `ublox` crate (`sam_m10q::ubx::nav::parse_pvt`), so field offsets and scaling
  are not hand-transcribed.
- **Fallback: NMEA GGA.** If configuration fails, or PVT goes stale for 3 s, GGA
  drives position again and an event fires. A misconfigured module still flies.
- **Boot sequence**: cold start → CFG-VALSET (Airborne <4g dynamic model, 5 Hz
  solution, NAV-PVT on, GLL/GSA/GSV/VTG/RMC off) → ACK check → baud to 115200 →
  probe for traffic; on no answer, fall back to 9600 and NMEA. 9600 is 960 B/s,
  which the default sentence set nearly saturates at 1 Hz — hence the switch.
- **RAM layer only** (`LAYER_RAM`). Config is reapplied every boot from the
  module's known power-on defaults rather than inherited from flash, and the
  part sees no config-write wear. Same reasoning as ESC spin direction.
  RAM is also the only layer whose *validity* the receiver checks
  (UBX-21035062 §3.10.5.1), which is why keys that constrain one another —
  `DYNMODEL` and `FIXMODE` — must be sent in a single VALSET. Config is applied
  in groups, one message each, so a rejection names the group rather than
  losing the whole batch; the accepted set is reported as `cfg_mask`.
- **`gnss-gsv` feature** (enabled by `rpc-control`): asks for NMEA GSV so
  `sats_in_view` is populated. The fix reports only satellites *used*, which
  reads zero throughout acquisition, so GSV is the only way to watch a receiver
  acquire. Costs ~2.4 kB/s at 5 Hz — 29% of the 115200 link but 69% of a 9600
  one, so it is requested only when the fast link was achieved.
- `GNSS_SIGNAL` carries `GnssData` to ULog, the RPC `GetGnss` handler, and CRSF
  telemetry. `hdop` is only meaningful on the GGA path; `h_acc_m` is the real
  fix-quality gate.
- UART is `BufferedUart`, not the DMA `Uart`: only the interrupt-buffered variant
  implements `embedded-io-async`, and its partial reads suit a bursty stream.

### Logging Systems

Three independent logging systems coexist, each serving a different purpose:

| System | Transport | Rate | Persistence | Audience |
|--------|-----------|------|-------------|----------|
| **defmt** | RTT channel 0 | Event-driven | Only if host captures | Developer at debug probe |
| **RPC LogTopic** | RTT channel 1 (postcard-RPC) | Event-driven (`elle_event!` sites throughout firmware) | No — streaming | Host TUI dashboard |
| **ULog** | SD card (FAT32 over SPI1) | 83Hz attitude+commands+controller+engine, 8.3Hz status, PID gains on change | Yes — survives power loss | Post-flight analysis |

- defmt macros (`info!`, `warn!`, etc.) are always compiled in; the transport (`defmt-rtt`) is gated on `defmt-logging` (default on). Without the transport, macros become no-ops.
- RPC LogTopic carries `(level: u8, code: u16)` — numeric event codes mapped to strings on the host side in `tui/ui.rs::log_code_text()`. ULog-related codes: 30=recording started, 31=init failed, 33=recording stopped, 34=flash erased.
- ULog records full-fidelity flight data (attitude, commands, controller, pid_gains, engine, status, baro, mag) via `elle-hardware::ULogLogger` → `ULOG_WRITE_CHANNEL` → `sd_writer_task` (FAT32 on SD card). Always compiled in (no feature gate). In flight mode recording auto-starts when the SD card is ready (`SD_READY`) and runs until power-off; in RPC mode it is started/stopped via `ulog start`/`ulog stop` TUI commands. The flash ULog region (0x210000–0xFFFFFF) is a legacy store — extraction/erase RPC still targets it, but new recordings go to SD.
- `commands` logs pilot input and the *pre-filter* setpoint; `controller` logs what the attitude PID actually used and did each tick: measured `dt_us`, attitude sample age `att_age_us`, the filtered + rate-limited setpoint, scaled P/I/D terms per axis (they sum to the correction), mixer `saturation` bits (pitch_up, pitch_down, roll_right, roll_left, LSB first) and the elevon pulses actually output after trim/inversion. `pid_gains` is written once per file and whenever `FlightController::gains_version()` changes.

### Control Loop Architecture

The firmware main loop runs at 83Hz (12ms ticker; `CONTROL_LOOP_PERIOD_MS` is the source of truth and `CONTROL_LOOP_FREQUENCY_HZ`/`CONTROL_LOOP_DT` derive from it). Engine output is decoupled: the control loop publishes throttle values via `DSHOT_THROTTLE` Signal, and a dedicated `dshot_task` resends them at ~1kHz. The DShot task also handles ESC arming on startup (2s MotorStop burst), eliminating any init gap.

**DShot task** (`crates/elle-hardware/src/dshot.rs`):
- Spawned on Core0 before supervisor init, takes ownership of PIO1+PIO2 engine peripherals
- Arms ESCs (2s), then runs 1kHz ticker: `try_take()` from `DSHOT_THROTTLE`, resend current values
- Uses `throttle_with_telemetry()` for bidirectional eRPM reading; writes `ENGINE_CACHE` every iteration
- Per-engine auto-fallback: after 100 consecutive telemetry failures, switches to `throttle_async()` (fire-and-forget) and sets `valid=false`
- Defaults to (0, 0) = MotorStop until the control loop publishes its first value

Where it lives: `elle_app::boot::run` (IMU wait, supervisor barrier, PID / mag-cal / level-cal loads) hands off to `elle_app::flight::run_flight` or `elle_app::rpc::run_rpc`. Both loops publish engine output through `elle_app::engines::publish_engine_output`, which always sends both eRPM targets (the single-engine task reads only the left).

**Flight mode** (default, no `rpc-control`):
1. Reads CRSF/ELRS commands from dedicated receiver task
2. Updates FlightController with attitude + pilot commands
3. Publishes engine output via `DSHOT_THROTTLE` signal
4. Autotune state machine (RC CH7 3-position switch with debounce: off/pitch/roll)
5. Failsafe check, LED pattern updates
6. ULog recording auto-starts once the SD card is ready (`SD_READY`), runs until power-off
7. Auto-saves PID gains to flash on autotune completion

**Arming** (`elle-control/src/arming.rs`): RC auto-arm needs a deliberate gesture — throttle above `ARM_THROTTLE_HIGH_RAW` (~30 %), then back to zero thrust (where the throttle curve outputs 0, raw ≤ `THROTTLE_DEADZONE`). Needed at boot and again after every disarm, kill switch or failsafe. It used to arm below 1100 µs on the 1000–1600 µs scale (raw 341, ~16.7 % stick), which the throttle curve already turns into ~5,400 RPM on the eagle. RPC `Arm` is refused (event 18) while the RPC throttle is non-zero.

RC aux channel map (`elle-config/src/lib.rs`): CH5 = heading hold (2-pos), CH6 = flight mode (3-pos: Manual/Stabilized/AltitudeHold), CH7 = autotune (3-pos: off/pitch/roll), CH8 = kill switch (2-pos, high = disarm).

**RPC mode** (`rpc-control` feature):
1. Reads RPC commands from `RPC_CMD_CHANNEL`
2. Builds `PilotCommands::Normalized` from accumulated RPC state
3. Updates FlightController with attitude data from IMU
4. Publishes engine output via `DSHOT_THROTTLE` signal
5. Autotune state machine (triggered via StartAutotune/AbortAutotune RPC)
6. Publishes `FlightState` + `ControllerOutput` signals and caches for RPC query handlers
7. Periodic LED pattern updates
8. ULog recording gated on `ULOG_ENABLED` flag (set via StartULog/StopULog RPC commands)
9. Handles ULog extraction commands (ReadULogChunk, PopAndPeekULog, EraseULog) via `FLASH_REQUEST_SIGNAL`

RPC handlers send commands to the main loop via `RPC_CMD_CHANNEL` — they never directly control hardware.

**Important**: ULog extraction uses `FLASH_REQUEST_SIGNAL` which is single-valued. Recording must be stopped before extraction to avoid signal contention. The host `ulog extract` command auto-sends `StopULog` first.

## Key Dependencies

| Crate | Firmware | Host | Purpose |
|-------|----------|------|---------|
| postcard-rpc 0.12 | server (define_dispatch!) | host_client | RPC framework |
| embassy-* (git) | yes | - | Async embedded runtime |
| probe-rs 0.31 | - | yes | Debug probe + RTT access |
| icm426xx 0.4 (git, upstream `ProfFan/icm426xx` rev `7e22a5a`) | yes | - | ICM-42686-P IMU driver (SPI, FIFO, 20-bit). Pinned to a git rev, not crates.io: the 42686-P generic `Device` support (PR #10, merged 2026-02-11) landed after 0.4.0 was published (2025-10-28), and upstream has not cut a release since. Switch to the registry once they do. |
| ahrs 0.8 | yes | - | Madgwick AHRS sensor fusion (no_std) |
| nalgebra 0.34 | yes | - | Linear algebra (no_std + libm) |
| bmp390 (vendored, drivers/bmp390) | yes (sync) | - | BMP390 barometer driver, patched over crates.io 0.4 |
| embassy-dshot 0.5 | yes | - | DShot ESC driver over PIO (own crate, published to crates.io). 0.5 shares one `BidirDshotProgram` per PIO block and makes every send fallible |
| embedded-hal-bus 0.3 | yes | - | I2C/SPI bus sharing (RefCellDevice, ExclusiveDevice) |
| cobs 0.5 | yes | yes | Frame encoding |
| ratatui 0.30 | - | yes | TUI dashboard |
| rtt-target 0.6 | yes | - | RTT channel API |

## Recent Work / Resume Points

### Completed
- Field-readiness cleanup: removed `rtt-control`, `mag-test`, `error-strings` features; `disable-imu` (stub IMU with synthetic data) removed for good in July 2026 once both platforms flew with the real ICM-42686
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
- CRSF telemetry TX: attitude, flight mode, GPS frames to radio via PIN_20
- CRSF telemetry log events forwarded to RPC LogTopic (codes 20-23) for TUI visibility
- Host TUI/direct mode: proper RTT worker shutdown via `AtomicBool` flag + `JoinHandle::join()`
- `CrsfReceiver::new()` refactored to accept `UartRx` (UART split for TX telemetry)
- MMC5616WA magnetometer reads at ~10 Hz, populates `MAG_SIGNAL` for TUI/RPC. Always-on (no feature gate), the chip is physically on the board.
- BMP390 barometer driver integrated via `embedded-hal-bus::RefCellDevice` for I2C0 bus sharing with MMC5616WA. Polls at ~2 Hz, populates `BARO_SIGNAL`. Init tries both addresses (0x77, 0x76). Always-on (no feature gate).
- I2C bus sharing: I2C0 wrapped in `RefCell` + `StaticCell`, creates `RefCellDevice` handles for MMC5616WA and BMP390. Safe because both run in a single task on Core1.
- CRSF telemetry expanded to 5 slots: attitude → flight_mode → GPS → baro → battery (~10 Hz each at 50 Hz tick)
- `GetBarometerEndpoint` RPC endpoint returns pressure (hPa), temperature (°C), barometric altitude (m)
- Host TUI displays barometer data (pressure, temperature, altitude) at 1 Hz poll rate
- **ICM-42686-P IMU integrated via SPI0** — replaced BNO055 (I2C) with ICM-42686 (SPI) + Madgwick AHRS sensor fusion. Renamed `BnoImu` → `Imu`. Removed `bno055`/`mint` deps, added `icm426xx`/`ahrs`/`nalgebra`.
- **AHRS sensor fusion**: `ahrs` crate (Madgwick filter) fuses ICM accel+gyro at 1 kHz with MMC5616WA mag at 10 Hz for 9-DOF attitude estimation. Falls back to 6-DOF (no mag) until first mag reading.
- **SPI0 pin assignments**: MISO=PIN_0, CS=PIN_1, SCLK=PIN_2, MOSI=PIN_3, INT1=PIN_5 (DATA_RDY — awaited via `wait_for_high()`)
- **Blocking SPI on Core1**: Uses blocking SPI (polled, no DMA) since DMA interrupt handlers are registered on Core0's NVIC. 24-byte FIFO read at 1 MHz SPI takes ~200µs.
- **I2C bus always RefCell-wrapped**: Both real and stub paths now use `RefCell<I2c>` for I2C0, since mag+baro share the bus.

- **RPC autotune endpoints**: StartAutotune/AbortAutotune RPC endpoints + TUI commands (`autotune pitch`, `autotune roll`, `autotune abort`, `savepid`)
- **Relay-based autotuner** (`crates/elle-control/src/autotune.rs`): Oscillation detection, Tyreus-Luyben/Ziegler-Nichols/SomeOvershoot tuning rules, safety timeout/amplitude limits
- **Flash PID persistence**: `SavedGains` serialized to flash via sequential-storage MapStorage. Auto-loads on boot with f32 validation (finite + range checks). Auto-saves on autotune completion.
- **`SignalCache<T>` abstraction**: Replaced 5 separate Signal+Mutex<Cell<>> pairs (`ATTITUDE`, `MAG`, `BARO`, `FLIGHT_STATE`, `CONTROLLER_OUTPUT`) with unified `SignalCache<T>` wrapper in `elle-hardware/src/signal_cache.rs`. Single `publish()` call replaces dual signal+cache writes.
- **`EngineUnit` restructure**: Replaced 14 flat `left_`/`right_` fields in `EngineReading` (firmware) and `EngineResp` (ICD) with per-engine `EngineUnitReading`/`EngineUnit` sub-structs. Unified `apply_edt_left`/`apply_edt_right` into `EngineUnitReading::apply_edt()`, consolidated duplicated left/right telemetry update blocks into `update_engine_unit()`.
- **RPC handler deduplication**: `send_cmd()` helper in `rpc_app.rs` (14 firmware handlers), `handle_ack()` helper in `tui/commands.rs` (14 host handlers), `save_pid_to_flash()` in `main.rs` (4 flash save sites), `mark_poll_success()` in `tui/state.rs` (8 polling branches)
- **Named constants**: `RAD_TO_CDEG` replacing 6 magic `5729.578` literals, `ULOG_FLASH_END_EXCL` replacing 5 hardcoded `0x1000000` values
- **ULog always compiled in**: Removed `ulog-logging` feature gate. ULog support is always available; recording starts only when explicitly triggered (RC switch or TUI command).
- **Code quality pass**: clippy pedantic/nursery fixes, f64→f32 atan2, named constants for magic numbers (AHRS_BETA, sensor rate ticks), `ULogState` enum replacing magic constants, event drain throttling, setpoint filter skip in Manual mode
- **Dedicated 1kHz DShot send task**: Decoupled ESC frame sending from the control loop into `dshot_task` running at ~1kHz via `DSHOT_THROTTLE` Signal. Task owns PIO1+PIO2 engines, handles arming, and continuously resends latest throttle values. `DshotEngines` made private (concrete PIO1/PIO2 types, no longer generic).
- **EDT RPM telemetry**: DShot task uses `throttle_with_telemetry()` for bidirectional eRPM reading. `ENGINE_CACHE` (Mutex<Cell<>>) carries `EngineReading` (eRPM, throttle, validity per engine). Auto-fallback to `throttle_async()` after 100 consecutive telemetry failures per engine. ULog `engine_data` message at the control-loop rate. `GetEngineEndpoint` RPC + TUI 5Hz polling + `direct engine` command.
- **Failsafe state machine**: `RcLinkState` enum (Ok/Warning/Lost) replaces simple threshold check. `RC_WARNING_MS=200` → `RC_TIMEOUT_MS=300` two-stage detection. Events (codes 13-15) fire on transitions only, not every iteration. `signal_restored()` clears failsafe on recovery. `rc_age_ms` in `StatusResp`, `FlightState`, ULog `system_status`. LED warning pattern (FastBlink/orange) at Warning, RapidFlash/orange at Lost.
- **Interrupt-driven IMU reads**: INT1 (PIN_5) configured for DATA_RDY (`ui_drdy_int1_en=1`). `Imu::run()` awaits `int1.wait_for_high()` (async GPIO; the embassy-rp multicore executor wakes Core1 across cores), then drains the FIFO: every queued sample is fused in order and only the newest attitude is published, capped at `IMU_MAX_DRAIN` per wake-up. Reading one sample per DATA_RDY used to leave any backlog from a slow iteration (mag/baro I2C) queued for good, ratcheting attitude latency up until a FIFO-overflow flush. A multi-sample drain fires event 45 (rate-limited to 1/s). INT2 (GPIO4) free for crash detection SMD interrupt.
- **Code review cleanup**: Replaced `Result<(), ()>` in `ULogLogger` with proper `ULogError` enum (4 variants: NotInitialized, InitFailed, BufferFull, FlushFailed). Extracted `buffer_writer_output()` helper deduplicating extend+flush across 8 log methods. Removed unused `failsafe: bool` parameter from `ArmingState::update()`. Removed misleading `const` from 9 methods across `arming.rs` and `system.rs` (`const fn` with `&mut self` compiles but is semantically wrong). Extracted 9 named constants from magic numbers in main loop divisors (`ULOG_STATUS_DIVISOR`, `ULOG_MAG_DIVISOR`, `ULOG_BARO_DIVISOR`, `ULOG_GNSS_DIVISOR`, `STALE_EVENT_DRAIN_DIVISOR`, `LED_UPDATE_INTERVAL`, `PERF_LOG_INTERVAL`, `GNSS_ERROR_LOG_INITIAL`, `GNSS_ERROR_LOG_INTERVAL`). Reviewed and dismissed 4 theoretical overflow risks (eRPM u32 multiplication fits, governor feedforward clamped by upstream DShot range, deadband discontinuity is 0.25% step, attitude rate i16 overflow only at 327°/s crash tumble in telemetry-only path).
- **RPM governor windup fix**: `RpmGovernor::update()` (`elle-control/src/governor.rs`) was clamping its PI correction/anti-windup against `DSHOT_THROTTLE_MAX` (1999) instead of each platform's measured stall/saturation boundary — at full throttle (target eRPM sitting right at the peak), any normal RPM dip wound the integrator past that boundary, pushing DShot into a region where more throttle means *less* RPM (at the time, eagle's right engine saturated above DShot 1473; current ceilings: eagle 1498 from the 2026-09-26 sweep, dart 1998 — see the prop entry below), causing runaway progressive RPM sag. Added platform-specific `GOVERNOR_DSHOT_MAX` const (`elle-config/src/lib.rs`) matching each platform's last `GOVERNOR_FF_TABLE` entry and used it for all governor clamps.
- **Governor MotorStop latch fix**: dropping throttle 40%→10% in the TUI left the engine commanded off permanently (`cmd:0`, `0 RPM`, against a live 1336 RPM target); 20%→10% was fine. Three pieces closed a loop: `dshot.rs` sent `MotorStop` whenever the *governor output* was 0 (a governor 0 is DShot frame 48 = minimum spin, **not** a stop) and then fabricated `erpm = 0, valid = true` for `ENGINE_CACHE`; the governor's spike filter rejected that fabricated 0 (jump > `GOVERNOR_ERPM_MAX_JUMP`) and kept the stale pre-chop `last_measured` forever, since no telemetry is read while MotorStop goes out; the frozen huge negative error clamped output back to 0. Trigger threshold is `KP × |Δ eRPM| > ff(target)` — ~2,070 RPM of drop at that operating point, which is why the small step escaped. Fixes: `MotorStop` and the fabricated zero reading are both now gated on `target_erpm == 0` rather than on the governor output, and `MAX_CONSECUTIVE_SPIKE_REJECTS` (20 ticks) bounds the spike filter so a stale `last_measured` can never latch it shut. Regression tests in `crates/elle-control/tests/governor_step_response.rs` (host: `cargo test -p elle-control --target x86_64-unknown-linux-gnu --features platform-dart`); `elle-control` gained a `platform-dart` passthrough feature for them. Affected flight mode too, not just the TUI — an RC throttle chop of the same size cut the engine with no recovery short of throttle-to-zero-and-back.
- **Dart prop and spin direction**: the dart runs a 3-blade prop with the ESC reversed. (An opposite-handed 2-blade flew briefly in Sep 2026 and broke on the 13th.) `crates/elle-hardware/src/dshot.rs` asserts spin direction at arm time on every boot (`SPIN_DIRECTION_CMD`, 6 transmissions, driven by `elle_config::ENGINE_SPIN_REVERSED`) — deliberately **without** `SettingsSave`, so there is no ESC EEPROM wear and the direction survives an ESC swap or factory reset. The dart `GOVERNOR_FF_TABLE` (`elle-config/src/lut.rs`) is from a 2026-09-26 `rpm_range` sweep on the 3-blade, reversed, 3S at 11.5 V under load: RPM rises all the way to DShot 1998 (15,360 RPM), so `GOVERNOR_DSHOT_MAX = 1998`, `MAX_RPM = 15_300`, `MAX_ERPM = 107_100`. The same sweep in the normal direction peaked at DShot ~1773 and fell off, which is how to recognise a wrong-direction run. `const _: () = assert!(GOVERNOR_DSHOT_MAX == lut::governor_ff_max_dshot())` makes ceiling/table drift a compile error. Recalibrate with the `governor-calibration` skill (`.claude/skills/`).
- **Roll axis sign fix**: `Imu::run()` (`elle-hardware/src/imu/driver.rs`) now negates `roll`/`roll_rate` from the AHRS quaternion — field-confirmed the identity mapping had roll backwards (aircraft rolled the wrong way in response to PID correction) on this PCB orientation.
- **Autotune measurement invert bug fix**: The per-tick measurement fed to `Autotuner::update()` (both `main.rs` files, flight-mode + RPC-mode tasks) was multiplying `att.pitch`/`att.roll` by `PITCH_INVERT`/`ROLL_INVERT`, but the real `FlightController` PID loop compares its setpoint against raw, uninverted attitude (`PITCH_INVERT` only corrects the pilot stick mapping, per the comment at `elle-system/src/system.rs:462`). With `PITCH_INVERT = -1.0`, this turned the pitch relay test into positive feedback instead of a self-correcting oscillation. Removed the invert multiplication from all 4 sites per platform file. Roll was unaffected only because `ROLL_INVERT = 1.0` was a no-op. Untested since the fix — retest pitch autotune specifically before trusting it.
- **Autotune Ku formula fix**: `finish_relay()` (`elle-control/src/autotune.rs`) computed `Ku = 4h/(πa)` as if the excitation were a pure relay, but the setpoint-relay method drives the plant with `u = K·(r − y)` (`K = scale·TEST_KP`) — a relay of amplitude `h = K·d` **in parallel with proportional feedback K**. The oscillation condition is `(4h/(πa) + K)·G = −1`, so Ku must include `+ K` (5.0 in effective units with `PID_SCALE = 5.0`); omitting it underestimated Ku by ~40% when the measured amplitude ≈ relay amplitude, making all previously autotuned gains softer than the tuning rules intended. Any gains autotuned before this fix should be re-tuned.
- **Autotune setpoint filter bypass**: `FlightController::update()` (`elle-system/src/system.rs`) now skips `SETPOINT_FILTER_ALPHA` smoothing when `setpoint_override` is active — the autotune relay is an intentional square wave, and the filter (τ ≈ 87 ms at 77 Hz) was attenuating/phase-lagging the excitation that the Ku/Tu math assumes reaches the plant unmodified.
- **Frozen-mag fusion fix**: `Imu::run()` mag error path (`elle-hardware/src/imu/driver.rs`) now clears `has_mag` alongside `mag_ok` on I2C error — previously the AHRS kept fusing the stale `last_mag` vector forever after the mag died mid-session, dragging yaw toward a fixed garbage heading regardless of calibration.
- **Mag cal progress counter fix**: `MAG_CAL_SAMPLES` in both `rpc_app.rs` files was read by `handle_get_mag_cal` but only ever written as 0 — Core1's real sample count never left the driver, so `mag cal` status showed 0 samples throughout collection. Replaced with `elle_hardware::imu::MAG_CAL_PROGRESS` (AtomicU16), written by the Core1 driver during collection, read by the RPC handlers. Note: `GetMagnetometer`/TUI mag panel intentionally shows **raw** counts (offsets only apply to the AHRS feed), so calibration is only observable via yaw behavior and the `mag cal` status offsets.

- **No out-of-tree path dependencies**: `embassy-dshot` was a `path = "../dshot-pio"` dep and `icm426xx` a `path = "../icm426xx"` dep, so the workspace only built on a machine with those sibling checkouts. `embassy-dshot` 0.3.0 was published to crates.io (upgraded to `embassy-rp` 0.10 / `embassy-time` 0.5.1, no `[patch.crates-io]` in the library) and is now a registry dep; `icm426xx` is pinned to upstream rev `7e22a5a` — the 42686-P support is merged into `ProfFan/icm426xx` main but postdates the published 0.4.0, so a git rev is needed until upstream releases again. A fresh clone now builds all four configurations (eagle, dart, eagle `rpc-control`, host tool) with no local checkouts.

- **GNSS via UBX-NAV-PVT + shared task**: `drivers/sam-m10q` gained an async API
  (`sam_m10q::asynch::SamM10q<RX, TX>`) over `embedded-io-async`, NAV-PVT parsing and
  CFG-VALSET building delegated to the `ublox` crate (0.10, `ubx_proto33`, no_std,
  ~1 KB flash), and ACK/NAK handling. The two byte-identical `gnss_task` copies
  collapsed into `elle_hardware::gnss`. Module now runs at 115200/5 Hz with the
  Airborne <4g dynamic model, GGA kept as a fallback. Velocity NED, ground speed,
  course and accuracy estimates plumbed through `GnssData`, `elle_ulog::GnssMessage`
  (now 58 bytes, with `const` assertions guarding the hand-written `FORMAT_MSG`
  length prefix that nothing checked before), `GnssResp`, the TUI panel and
  `direct gnss`. CRSF GPS groundspeed was hardcoded to 0 and is now real.
  **Not yet bench-tested** — the baud/rate switch needs hardware verification.

- **Stabilization latency fixes** (from an external stabilization review):
  - *Elevons on hardware PWM.* `PioPwm` queued commands in a 4-deep PIO TX FIFO drained once per 20 ms frame; written every tick it stayed full, dropped each new write, and servos ran ~70–80 ms behind the controller. `PwmOutputs` (`elle-hardware/src/pwm.rs`) now drives PWM slice 6 (PIN_12 = A right, PIN_13 = B left) at a 1 MHz count, whose compare latches at wrap: always the latest command, ≤ 1 frame old.
  - *IMU FIFO drained per DATA_RDY* (see the interrupt-driven IMU entry): backlog from slow iterations no longer persists.
  - *Loop timing.* `1000 / 77` gave a 12 ms ticker while `CONTROL_LOOP_DT` stayed 0.013, so PID integral, heading hold and autotune Tu were 8% off. Everything now derives from `CONTROL_LOOP_PERIOD_MS = 12` (83 Hz); autotune timeouts and debounce ticks derive from the rate.
  - Gains fitted or autotuned before these fixes were compensating for the delays: re-run autotune before trusting them. Not yet bench-verified (scope PIN_12/13, watch event 45).

- **Stabilization follow-ups** (same review, second round):
  - *Gyro bias at boot.* `elle_control::gyro_bias::GyroBiasEstimator` averages the first still second (restarts when any axis spreads > `GYRO_BIAS_MAX_SPREAD_RAD_S`; gives up after 10 s or on |bias| > `GYRO_BIAS_MAX_RAD_S`). Core1 subtracts it from every sample before level cal, mount and AHRS. `IMU_STATUS.calibrated` now means "bias measured" (LED green / TUI "Cal"); on failure the IMU flies with zero bias, stays uncalibrated. Events 46 (measured) / 47 (failed). Keep the aircraft still for ~1 s after power-up.
  - *Setpoint rate limit.* Each EMA step of the attitude setpoint is capped at `MAX_SETPOINT_RATE_DEG_S` (90°/s). Autotune override stays unfiltered.
  - *Mixer-aware anti-windup.* `mix_elevons` reports `MixSaturation`; the PID holds an axis's integral when its error would push further into a blocked direction (previous tick's flags), and always lets it unwind.
  - *Controller logging.* New `controller` and `pid_gains` ULog messages (see Logging Systems).
  - Bench-verified on the eagle (LOG_0032/0033): dt 12.0 ms p50, attitude age < 1.7 ms, anti-windup never grew into a blocked direction, setpoint ramp ~90°/s.
- **Gyro rate filter + eagle gains** (from LOG_0033):
  - EDFs running put 20–30 °/s of roll-rate noise on the gyro (0.1 °/s stopped), aliased into the 83 Hz loop through the D term: 100–200 µs elevon jitter with centred sticks. A 2nd-order Butterworth low-pass (`elle_control::filter`, `GYRO_RATE_LPF_HZ` = 30) now runs on Core1 at 1 kHz on the rates published for the PID; the AHRS integrates the unfiltered gyro. The ICM's own AAF is 488 Hz at 1 kHz ODR. Resize the cutoff from a `gyro-raw-log` capture.
  - Eagle default gains moved to the dart's flown values (pitch 0.45/0.020/0.16, roll 0.25/0.012/0.07); the old 1.0/0.1/0.25 pitch D sustained a 5–8 Hz elevon/airframe oscillation on the bench with engines off.

- **Eagle/dart deduplication (`elle-app`)**: the two `main.rs` files (~2,200 lines each, ~88 % identical) and their identical `rpc_app` / `rpc_handlers` / `flight_state` collapsed into the `elle-app` crate; each `main.rs` is now ~270 lines of hardware setup, and `diff` between them shows only engine setup and the PIO2 IRQ binding. Divergences resolved to the superset: dart gained `rpc-rc`; the eagle gained the `IGNORE_PID_FLASH` gating (const is `false` there); `SetPidGains` carries a `PidConfig`. Loops take `epoch_ms` from `main()` rather than calling `compile_time::unix!()` (a library would bake in its own, possibly stale, build time). Async-fn gotcha found on the way: passing `FlightController` and `ULogLogger` down by value duplicated their storage in each enclosing future (+7.3 kB .bss); the controller is built in `boot::run` and lent to the loops, and each loop owns its logger. Post-refactor binaries: .bss +72..88 B, .text +1..2.5 kB vs. before.

- **Visibility / const passes and the bugs they surfaced**: `pub` items cut ~36 % across the firmware crates so rustc's dead-code lint can see unused code (~100 dead items removed); ~33 `const fn`, 26 new compile-time checks, ULog `FORMAT_MSG` bytes derived from `FORMAT` at compile time. Bugs found and fixed:
  - dart yaw cut its only engine's thrust up to 20 % (the yaw LUT ignored `YAW_TO_DIFF_GAIN`);
  - legacy flash ULog counters were never incremented (now counted once when the flash manager goes idle after boot, decremented with saturation);
  - IMU/flash timings were never recorded (`elle_hardware::timing`, behind `performance-monitoring`, is read by the summary and `GetPerformance`);
  - double-tap event 120 was never emitted;
  - an unknown autotune axis started roll tuning (axis values are now `elle_rpc_icd::AUTOTUNE_AXIS_*`, unknown ones NAK'd with `AUTOTUNE_ERR_BAD_AXIS`);
  - `ARM_DURATION_MS` was unused; `GetVersion` was hard-coded.

### Known TODOs in Firmware
None currently tracked — see `TODO.md` for the feature backlog (waypoint navigation, pitot tube).

### ULog Recording

Always compiled in. TUI commands:
- `ulog start` — start ULog recording (initializes logger on first call, sets `ULOG_ENABLED`)
- `ulog stop` — stop ULog recording (flushes buffer, clears `ULOG_ENABLED`)
- `ulog extract [file]` — downloads all queued ULog data (auto-stops recording first, fragments 4KB queue items into 512B RPC chunks, default filename `flight_YYYYMMDD_HHMMSS.ulg`)
- `ulog erase` — erases entire ULog flash region (0x210000–0xFFFFFF, auto-stops recording)

Flight mode: recording auto-starts when the SD card is detected and initialized (`SD_READY`), and runs until power-off — no RC switch. `ulog extract`/`ulog erase` operate on the legacy flash region; new recordings land on the SD card as FAT32 files.

### PID Autotune

Relay-based autotuner with 3-position RC switch (CH7: off/pitch/roll) in flight mode, or RPC commands in ground test mode.

TUI commands:
- `autotune pitch [relay_deg] [cycles] [tl|zn|so]` — start pitch autotune
- `autotune roll [relay_deg] [cycles] [tl|zn|so]` — start roll autotune
- `autotune abort` — abort active autotune, restore original gains
- `savepid` — save current PID gains to flash without autotuning

Computed gains are auto-saved to flash and auto-loaded on next boot.

### Magnetometer Hard-Iron Calibration

Compensates PCB hard-iron offsets on the MMC5616WA magnetometer by tracking min/max readings over all orientations, then subtracting the midpoint (hard-iron bias). Uses flash MapStorage key=2 for persistence (same infrastructure as PID gains, key=1).

**Calibration flow:**
1. `mag cal start` → firmware resets min/max trackers, collects 300 mag samples (~30s at 10Hz)
2. User rotates board through all orientations during collection
3. After 300 samples: validates axis ranges (≥5000 counts each), computes offsets, auto-saves to flash
4. Offsets are subtracted from raw mag counts before feeding AHRS (raw counts still available in MAG_CACHE for diagnostics)

**TUI commands:**
- `mag cal start` — start calibration (rotate board for ~30s)
- `mag cal clear` — clear calibration (zero offsets, clear flash)
- `mag cal` — show calibration status and offsets

**Direct CLI:** `direct mag-cal start|clear|status`

**Cross-core signals:** `MAG_CAL_START_SIGNAL` (Core0→Core1 start), `MAG_CAL_RESULT_SIGNAL` (Core1→Core0 result), `MAG_CALIBRATION_SIGNAL` (Core0→Core1 loaded offsets at boot)

**Statics for RPC visibility:** `MAG_CAL_OFFSET`, `MAG_CAL_STATUS` (0=uncalibrated, 1=collecting, 2=calibrated) in `rpc_app.rs`; live sample count in `elle_hardware::imu::MAG_CAL_PROGRESS` (written by Core1 driver)

**Event codes:** 110–116 (started, complete, failed, saved, cleared, loaded, load empty)

Offsets auto-load on boot and survive power cycles.

### Level Calibration (IMU mounting offset)

Attitude comes from the flight controller's own IMU, so without this 0° means the *circuit board* is level, not the airframe — Stabilized and AltitudeHold hold whatever tilt the board is mounted at. Level calibration measures that tilt once and corrects for it.

**How it works:** with the aircraft still at its reference attitude, Core1 skips 500 samples (so a triggering tap dies out), then averages 2000 raw accelerometer samples (2 s at 1 kHz). The rotation taking that gravity vector onto +Z is the "mount" quaternion (`elle_control::level_cal::compute_mount`). It is applied to the accel, gyro and (offset-corrected) mag vectors **before** the AHRS, so attitude *and* rates come out in the airframe frame at every attitude. Rejected if the gyro magnitude ever exceeds `LEVEL_CAL_MAX_GYRO_RAD_S` (moving) or the tilt exceeds `LEVEL_CAL_MAX_TILT_DEG` = 15° (not at the reference attitude; an upside-down board is caught here too). On failure the previous mount stays.

**Reported angles** use the attitude telemetry's sign convention: what the *uncorrected* attitude reads with the airframe level. "pitch −2.1°" = board sits 2.1° nose-down.

**Storage:** flash MapStorage key=3, 16 bytes (quaternion w, i, j, k). "Clear" stores the identity; on load, identity, non-finite, non-unit or >15° values all mean "uncorrected". Auto-loads on boot.

**Triggers:**
- TUI: `level cal start` · `level cal clear` · `level cal` (status). Refused while armed.
- Direct CLI: `direct level-cal start|clear|status`
- Field gesture (flight mode, and `rpc-rc`): the mag-cal double-tap with the **CH7 autotune switch out of off** (pitch or roll position) starts level cal instead of mag cal. Kill switch on, throttle low, gyro quiet, as for mag cal. Safe: autotune only starts while armed and only on an off→on transition. LED fast-blinks cyan while collecting.

**Code:** math in `crates/elle-control/src/level_cal.rs` (host tests in `crates/elle-control/tests/level_cal.rs`); Core1 collection and the mount rotation in `elle-hardware/src/imu/driver.rs` (`level_cal_step`); Core0 plumbing — signals (`LEVEL_CALIBRATION_SIGNAL`, `LEVEL_CAL_START_SIGNAL`, `LEVEL_CAL_RESULT_SIGNAL`), `STATUS` for the RPC handler, and the flash round-trips — in `elle-hardware/src/imu/level_cal.rs`, called identically from both airframes and both modes (`load_from_flash`, `start`, `clear`, `poll_result`).

**Event codes:** 150–158 (started, complete, failed-moving/refused-while-armed, failed-tilted, saved, save failed, cleared, loaded, load empty). 140–149 belong to GNSS.

### Next Steps
See `TODO.md` for prioritized task list (waypoint navigation, pitot tube).
