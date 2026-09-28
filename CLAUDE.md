# Elle Flight Controller

Firmware for two RP2350 flying wings (Embassy async, `no_std`) plus a host tool.
This file is the developer/Claude reference. **How to fly, arm, calibrate and read
events is in [`docs/OPERATIONS.md`](docs/OPERATIONS.md)** — keep the two in sync when
behaviour changes.

## Documentation map

| Doc | What it is for |
|-----|----------------|
| [`README.md`](README.md) | Project overview, quick start |
| [`docs/OPERATIONS.md`](docs/OPERATIONS.md) | Operator guide: arming, failsafe, RC map, modes, calibration, autotune, ULog, LED and event code tables |
| [`TEST_PLAN.md`](TEST_PLAN.md) | Bench and field test procedures |
| [`STATE_DIAGRAMS.md`](STATE_DIAGRAMS.md) | Arming, failsafe, mode, autotune, calibration state machines |
| [`TODO.md`](TODO.md) | Feature backlog and items still awaiting hardware verification |
| [`docs/DART_PID.md`](docs/DART_PID.md) | How the current PID gains were derived from dart flight logs |
| [`docs/NAVIGATION_PLAN.md`](docs/NAVIGATION_PLAN.md) | Waypoint navigation design (not started) |
| [`docs/embassy-rp-sio-irq-fifo-flash-bug.md`](docs/embassy-rp-sio-irq-fifo-flash-bug.md) | Why the flash manager masks the SIO FIFO IRQ |
| [`tools/elle-rpc-host/README.md`](tools/elle-rpc-host/README.md) | Host TUI and `direct` commands |
| [`crates/elle-ulog/README.md`](crates/elle-ulog/README.md) | ULog writer and message set |
| [`.claude/skills/governor-calibration/SKILL.md`](.claude/skills/governor-calibration/SKILL.md) | Re-sweeping the RPM governor table |
| [`docs/archive/`](docs/archive/) | Superseded design specs (sensor drivers), kept for reference |

## Project Structure

Cargo workspace:

- **`crates/elle-eagle/`** — twin-engine flying wing binary
- **`crates/elle-dart/`** — single-engine binary (same stack, `platform-dart` config)
- **`crates/elle-app/`** — the application both binaries run: boot sequence, flight and RPC control loops, RPC dispatch, ULog per-tick logging, shared tasks. The binaries keep only pins, engine setup, `bind_interrupts!`, `led_task` and `main()` — **make behaviour changes here, once**, not in a binary.
- **`crates/elle-system/`** — `FlightController` (arming, failsafe, modes, PID, mixing, heading hold) and the RPC transport
- **`crates/elle-hardware/`** — drivers and tasks: `crsf`, `dshot`, `event`, `flash`, `gnss`, `imu`, `led`, `pwm`, `sd_writer`, `timing`, `watchdog`, the ULog logger
- **`crates/elle-control/`** — pure algorithms, host-testable: `arming`, `autotune`, `commands`, `filter`, `governor`, `gyro_bias`, `heading`, `level_cal`, `mixing`, `pid`
- **`crates/elle-config/`** — every tunable constant and per-platform value (`lib.rs`, `lut.rs`, `profile.rs`)
- **`crates/elle-rpc-icd/`** — shared RPC Interface Control Document
- **`crates/elle-ulog/`** — `no_std` ULog encoder
- **`crates/elle-error/`** — error types
- **`crates/elle-nav/`** — navigation math placeholder (workspace member, not yet used by the firmware)
- **`drivers/`** — vendored sensor drivers: `mmc5616wa` (mag), `sam-m10q` (GNSS), `bmp390` (baro, local fork patched over crates.io)
- **`tools/elle-rpc-host/`** — host CLI (TUI dashboard + `direct` commands)

## Building

### Firmware

Target is RP2350 (`thumbv8m.main-none-eabihf`), set in `.cargo/config.toml`. Same
commands in `crates/elle-dart`.

```sh
cd crates/elle-eagle
cargo build --release                                                          # flight mode (CRSF/ELRS)
cargo build --release --no-default-features --features rpc-control,gnss        # RPC ground test mode
cargo build --release --no-default-features --features rpc-control,rpc-rc,gnss # RPC monitoring + RC flight control
cargo run --release --no-default-features --features rpc-control,gnss          # flash via probe-rs
```

**Do not omit `gnss` from RPC builds.** It is in `default`, and
`--no-default-features` drops it — the GNSS task is then never compiled or spawned,
so the TUI shows no satellites and no GNSS log lines at all, which looks exactly like
a hardware or reception failure.

Feature flags (both binaries; they forward to `elle-app`):
- `rpc-control` — postcard-RPC server over RTT. **Mutually exclusive with `defmt-logging`** (both define `_SEGGER_RTT`), enforced by `compile_error!` in each `main.rs`.
- `rpc-rc` — in RPC mode, pilot commands come from the RC receiver (gesture arming, kill switch, RC failsafe) while the TUI monitors. **Requires `rpc-control`** (`compile_error!` in `elle-app`).
- `defmt-logging` — defmt over RTT (default). The macros are always compiled; without this feature they are no-ops.
- `gnss` — SAM-M10Q task (default). Enables `elle-hardware/gnss`. `rpc-control` also enables `gnss-gsv` (satellites in view).
- `performance-monitoring` — timing instrumentation (`TimingMeasurement` is a no-op stub without it).
- `gyro-raw-log` — every 1 kHz gyro sample to ULog as `gyro_raw` (~25 kB/s), for sizing `GYRO_RATE_LPF_HZ`. Bench only.

CRSF telemetry TX and ULog are always compiled in.

### Feature powerset check

```sh
cargo hack check -p elle-eagle --feature-powerset --exclude-features defmt-logging,default,rpc-rc --release
cargo hack check -p elle-dart  --feature-powerset --exclude-features defmt-logging,default,rpc-rc --release
```

### Host tool and tests

The workspace defaults to thumbv8m, so host crates need an explicit target:

```sh
cargo build -p elle-rpc-host --target x86_64-unknown-linux-gnu
cargo test  -p elle-control  --target x86_64-unknown-linux-gnu                          # eagle config
cargo test  -p elle-control  --target x86_64-unknown-linux-gnu --features platform-dart  # dart config
```

### CI

`.github/workflows/ci.yaml` on every PR: release builds of eagle and dart × flight /
`rpc-control,gnss` / `rpc-control,rpc-rc,gnss`; `cargo fmt --check`; both powerset
checks; clippy `-D warnings` on both airframes (flight and RPC+RC) and the host tool;
`elle-control` tests for both platforms (governor step response, level cal, autotune,
arming, …).

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
| PIN_11 | DShot right engine | PIO2 (eagle only) |
| PIN_12 | Elevon right | PWM slice 6 A |
| PIN_13 | Elevon left | PWM slice 6 B |
| PIN_14 | DShot left / single engine | PIO1 |
| PIN_20 | UART1 TX | CRSF telemetry (DMA_CH4) |
| PIN_21 | UART1 RX | CRSF receiver (DMA_CH3) |
| PIN_23 | SD CARD_DETECT | GPIO input (active-low) |
| PIN_24 | SPI1 MISO | SD card |
| PIN_25 | SPI1 CS | SD card |
| PIN_26 | SPI1 SCK | SD card |
| PIN_27 | SPI1 MOSI | SD card |
| PIN_28 | UART0 TX | SAM-M10Q GNSS (config writes) |
| PIN_29 | UART0 RX | SAM-M10Q GNSS (`BufferedUart`, UART0_IRQ) |

DMA: CH1 flash, CH2 LED, CH3 CRSF RX, CH4 CRSF TX, CH5 SD TX, CH6 SD RX. CH0 and CH7
are free — the GNSS UART is interrupt-buffered because only `BufferedUart` implements
`embedded-io-async`.

## Architecture

### Cores and tasks

- **Core 0, thread executor:** control loop (`elle_app::boot::run` → `flight::run_flight` or `rpc::run_rpc`), flash manager, SD writer, LED, CRSF RX/TX, GNSS, supervisor, RPC server.
- **Core 0, interrupt executor** (`EXECUTOR_DSHOT` on SWI_IRQ_0, priority P2, set up in each binary): the DShot task alone, so thread-executor stalls can't delay its frames. Flash operations still pause it (interrupts off), which is fine because they only happen disarmed. This makes `executor-interrupt` and the SIO FIFO masking load-bearing ([embassy-rp bug note](docs/embassy-rp-sio-irq-fifo-flash-bug.md)).
- **Core 1:** `imu_task` — ICM-42686 over **blocking** SPI0 (DMA IRQs are bound on Core 0's NVIC), MMC5616WA + BMP390 sharing I2C0 through `RefCell` + `embedded-hal-bus::RefCellDevice`, Madgwick AHRS, calibrations.

`boot::run` waits for the IMU and the supervisor barrier, loads PID / mag cal / level
cal from flash, builds the `FlightController`, then hands it by `&mut` to the loop.

### Control loop

83 Hz: `CONTROL_LOOP_PERIOD_MS = 12` is the source of truth; `CONTROL_LOOP_FREQUENCY_HZ`
and `CONTROL_LOOP_DT` derive from it. After a stall (flash write) the ticker resyncs
instead of bursting (`support::resync_after_stall`).

Each tick: read commands (CRSF in flight mode, `RPC_CMD_CHANNEL` + accumulated RPC state
in RPC mode), kill switch, `FlightController::update` with the latest attitude, publish
engine output via `engines::publish_engine_output` (always both eRPM targets; the
single-engine task reads only the left), autotune step, failsafe check, heading hold,
LED, ULog, deferred flash saves. RPC handlers only send `RpcCommand`s to the loop; they
never touch hardware.

Arming, failsafe and kill-switch *behaviour* is described in
[`docs/OPERATIONS.md`](docs/OPERATIONS.md#arming-and-disarming). Code:
`elle-control/src/arming.rs` (gesture: `ARM_THROTTLE_HIGH_RAW`, then zero thrust via
`throttle_curve_lut`), `FlightController::check_failsafe` / `RcLinkState`
(`RC_WARNING_MS` 200 / `RC_TIMEOUT_MS` 300), kill switch at the top of the loop in
`flight.rs` / `rpc.rs`. In pure RPC mode `explicit_arming_only` disables the gesture and
the failsafe ages `elle_system::rpc::HOST_LAST_RX_MS` instead of RC frames.

### Controller (`elle-system/src/system.rs`, `elle-control`)

- Stabilized: stick → attitude setpoint (±25° pitch, ±45° roll), EMA (`SETPOINT_FILTER_ALPHA`) with each step capped at `MAX_SETPOINT_RATE_DEG_S` (90°/s). The autotune override bypasses both.
- AltitudeHold is a 0°/0° level hold for now.
- Heading hold (`elle-control/src/heading.rs`) replaces the roll setpoint in Stabilized.
- PID outputs are scaled by `PID_SCALE` (5.0). `mix_elevons` reports `MixSaturation`; the PID freezes an axis's integral when its error would push further into a saturated direction (previous tick's flags) and always lets it unwind.
- `PITCH_INVERT` / `ROLL_INVERT` apply to **stick input only** (`elle-control/src/commands.rs`); the PID compares against raw attitude. Anything fed to the autotuner must use raw attitude too.
- Autotune (`elle-control/src/autotune.rs`): `update(pitch_deg, roll_deg, tick)` measures its own axis and enforces the ±20° envelope on both from settling on. Both loops call `support::guard_autotune` before it every tick; it aborts and restores gains once the run loses the aircraft (`check_run_conditions`: kill, disarm/failsafe, attitude controller off, no attitude). A run never changes axis. Crossings and relay flips use `AUTOTUNE_HYSTERESIS_DEG`; `validate_result` also caps the tuned axis's Kp/Kd change at `AUTOTUNE_MAX_GAIN_RATIO` (Ki is exempt, see its doc).
- Elevons are on hardware PWM slice 6 at a 1 MHz count (`elle-hardware/src/pwm.rs`), 200 Hz frames (`REFRESH_INTERVAL_US` = 5 000; digital servos on both airframes); the compare latches at wrap, so output is always the latest command, ≤ 1 frame (5 ms) old.

### IMU pipeline (`elle-hardware/src/imu/driver.rs`)

1. INT1 DATA_RDY wakes the task; it drains the FIFO, fusing every sample in order, up to `IMU_MAX_DRAIN` (32) per wake-up, publishing only the newest attitude (event 45 on multi-sample drains, rate-limited).
2. Gyro bias (`elle_control::gyro_bias`) is measured over the first still second and subtracted from every sample. `IMU_STATUS.calibrated` means "bias measured".
3. Level-cal mount quaternion rotates accel, gyro and (offset-corrected) mag into the airframe frame before the AHRS.
4. Madgwick AHRS at 1 kHz, 9-DOF once a mag reading exists; on a mag I2C error `has_mag` is cleared so a stale vector is never fused.
5. **Roll and roll rate are negated** after the quaternion → Euler conversion (this PCB orientation).
6. Rates published for the PID pass through a 2nd-order Butterworth low-pass (`GYRO_RATE_LPF_HZ` = 30); the AHRS integrates unfiltered gyro.

Any I2C error on mag or baro takes **both** off the bus (`i2c_bus_failed`, event 48).
Mag ~10 Hz, baro ~20 Hz (`MAG_READ_INTERVAL_TICKS` / `BARO_READ_INTERVAL_TICKS` at 1 kHz).

### DShot and RPM governor (`elle-hardware/src/dshot.rs`, `elle-control/src/governor.rs`)

- `dshot_task` (eagle, PIO1 + PIO2) / `dshot_single_task` (dart, PIO1) owns the engines: 2 s MotorStop burst to arm the ESCs (`ARM_DURATION_MS`), spin direction asserted every boot from `ENGINE_SPIN_REVERSED` (6 commands, **no** `SettingsSave` — no ESC EEPROM wear), then a 1 kHz loop that takes eRPM targets from `DSHOT_THROTTLE` and sends via `read_extended_telemetry()` (both engines joined), falling back per engine to fire-and-forget while running after 100 telemetry failures (re-enabled when the ESC answers at idle). If an ESC doesn't answer right after the boot configuration, it is configured when it first does. Arm/disarm beeps come in via `BEEP_SIGNAL`.
- The governor converts target eRPM to DShot with a feed-forward table (`GOVERNOR_FF_TABLE` in `elle-config/src/lut.rs`) plus PI. **Governor output 0 is DShot 48 = minimum spin, not a stop**: `MotorStop` and the fabricated zero reading are gated on `target_erpm == 0` only.
- All governor clamps use `GOVERNOR_DSHOT_MAX` — the platform's stall/saturation boundary, not `DSHOT_THROTTLE_MAX`. A compile-time assert ties it to the last table entry. Current: eagle 1498 (`MAX_ERPM` 142,100), dart 1998 (107,100; 3-blade prop, ESC reversed).
- The telemetry spike filter (`GOVERNOR_ERPM_MAX_JUMP`) is bounded by `MAX_CONSECUTIVE_SPIKE_REJECTS` (20) so a stale reading can never latch the output shut. Regression tests: `crates/elle-control/tests/governor_step_response.rs`.
- The send loop is paced by `elle_control::dshot_pace::next_deadline_us`, not a `Ticker`: after a stall it skips missed ticks instead of replaying them back to back, and frames stay ≥ `DSHOT_MIN_FRAME_GAP_US` apart. A push while the previous frame is still on the wire truncates it (embassy-dshot issue #8), and the ESC may read the splice as a beep or a throttle pulse.
- A stopped engine gets `MotorStop` **with** a telemetry request (`command_with_extended_telemetry`), so EDT flows at idle. `elle_control::esc_link::EscLink` counts unanswered requests: `ESC_SILENT_FRAMES` in a row → event 160/161; when the ESC answers again (or answers for the first time after missing the boot configuration), it is re-sent spin direction + EDT once it has been stopped and answering for `ESC_RECONFIGURE_SETTLE_FRAMES` (162/163). Reply, timeout, corrupt-reply and re-configuration counts go to ULog `esc_health`.
- Recalibrate with the `governor-calibration` skill. A sweep in the wrong spin direction peaks early and falls off.

### Watchdog and supervisor

- Hardware watchdog (`elle_hardware::watchdog`), `WATCHDOG_TIMEOUT_MS` = 500, fed every control tick. The flash manager feeds `FLASH_OP_WATCHDOG_MS` (6 s) before each flash request and before the idle ULog count.
- The supervisor sequences startup through the `SUP_*_READY` / `SUP_START_*` signals (LED, then IMU, then flight controller). Core 1 sends a heartbeat; `check_core1_health` disables the attitude controller when it stops (events 70/71).

### Flash persistence (`elle-hardware/src/flash/`)

- Profile region 0x200000–0x20FFFF: `sequential-storage` MapStorage, `MAP_KEY_SLOTS = 4`. Entries are `elle_config::profile::ProfileEntry` — **1** PID gains (32 B), **2** mag cal offsets (12 B), **3** level cal quaternion (16 B); never renumber. A new entry needs `MAP_KEY_SLOTS` raised.
- **Policy: one setting per key, all access through `sequential-storage`.** Clearing a setting (`clearpid`, `mag cal clear`, `level cal clear`) is `FlashRequest::ClearProfileEntry`, which calls `remove_item` on that key only (the RP flash driver is `MultiwriteNorFlash`); boot then falls back to firmware defaults. Never erase the profile region directly. The loaders still treat zero mag offsets and an identity mount as "not calibrated", which is what older firmware stored on clear.
- ULog region 0x210000–0xFFFFFF: legacy queue, still served by extract/erase RPC.
- **No flash write while armed.** A write pauses Core 1 and blocks Core 0 (DShot included). Autotune saves are deferred until disarm; calibration results are only collected when disarmed; RPC commands with `writes_flash()` are refused while armed (event 63).
- `IGNORE_PID_FLASH` (true on the dart only): PID gains are never loaded or saved; firmware defaults always apply.
- The flash manager masks the SIO FIFO IRQ around writes — see the [embassy-rp bug note](docs/embassy-rp-sio-irq-fifo-flash-bug.md).

### Shared state

**`SignalCache<T>`** (`elle-hardware/src/signal_cache.rs`) — `Signal` + `Mutex<Cell<T>>`: `publish()`, `read_cached()`, `try_take()`.

- `ATTITUDE` (`elle-hardware/src/imu.rs`) — AHRS attitude + filtered rates at 1 kHz; the control loop `try_take()`s it.
- `MAG`, `BARO` (same file) — raw mag counts (~10 Hz), pressure/temp/altitude (~20 Hz); read by CRSF telemetry, RPC and ULog.
- `GNSS_SIGNAL` (`elle-hardware/src/gnss.rs`) — `GnssData` for ULog, RPC and CRSF.
- `FLIGHT_STATE`, `CONTROLLER_OUTPUT` (`elle-app/src/flight_state.rs`) — published by the RPC loop for the status and controller-output handlers.
- `DSHOT_THROTTLE` / `ENGINE_CACHE` / `BEEP_SIGNAL` (`elle-hardware/src/dshot.rs`) — eRPM targets in, per-engine telemetry out.
- `RPC_CMD_CHANNEL` + `RpcCommand` (`elle-app/src/rpc_handlers.rs`) — RPC handler → loop.
- `EVENT_CHANNEL` (`elle-hardware/src/event.rs`) — `elle_event!` → `log_publisher_task` → `LogTopic`.
- `ULOG_ENABLED`, `ULogState` / `ULOG_ITEM_SIGNAL` (`elle-app/src/rpc_app.rs`) — RPC ULog control and legacy flash extraction. `FLASH_REQUEST_SIGNAL` is single-valued: recording must stop before extraction (the host does this).
- Calibration signals: `MAG_CAL_*` / `MAG_CALIBRATION_SIGNAL` and `LEVEL_CAL_*` / `LEVEL_CALIBRATION_SIGNAL` cross Core 0 ↔ Core 1; `MAG_CAL_PROGRESS` is the live sample count. Level-cal Core 0 plumbing is in `elle-hardware/src/imu/level_cal.rs`.

### RPC transport and protocol

postcard-RPC over RTT (`elle-system/src/rpc.rs`):
- Up channel 0: defmt (NoBlockSkip) · Up channel 1: RPC TX (BlockIfFull, COBS) · Down channel 0: RPC RX (COBS).
- `RttTx` (blocking mutex, double-buffered COBS), `RttRx` (frame reassembly, 100 µs poll; stamps `HOST_LAST_RX_MS`), `ElleWireSpawn` stub (all handlers blocking).
- ICD in `crates/elle-rpc-icd/src/lib.rs` (`endpoints!` / `topics!`); dispatch via `define_dispatch!` in `elle-app/src/rpc_app.rs`; server loop in `elle-app/src/tasks.rs` `rpc_server_task()`.

Endpoints — control: SetThrottle, SetElevons, SetControlMode, SetPidGains, SetHeadingHold ·
safety: Arm, Disarm, EmergencyStop · query: GetStatus, GetAttitude, GetPerformance,
ResetPerformance, GetMagnetometer, GetBarometer, GetGnss, GetRcChannels,
GetControllerOutput, GetEngine · ULog: StartULog, StopULog, ReadULogChunk, EraseULog,
GetULogInfo · autotune: StartAutotune (axis `0xFF`/`0xFE` = save/erase PID), AbortAutotune · calibration: StartMagCal, ClearMagCal, GetMagCal, StartLevelCal,
ClearLevelCal, GetLevelCal · system: Ping, GetVersion, GetTime.
One outgoing topic: `LogTopic` `(level: u8, code: u16)`.

**Host `probe.rs` reads RTT up channel 1, not 0.**

### Host tool (`tools/elle-rpc-host/`)

TUI dashboard (default) or `direct <cmd>`. `probe.rs` (probe-rs + RTT worker thread),
`wire.rs` (tokio ↔ HostClient), `tui/` (event loop, state, ui, commands), `direct.rs`
(one-shot commands; holds the link with 100 ms pings after commands that leave
something moving). Event code labels: `tui/ui.rs` `log_code_text()` — add one for every
new `EVT_*`. Poll rates: attitude 10 Hz, status 0.5 Hz, mag 5 Hz, baro 1 Hz, GNSS 1 Hz,
engine 5 Hz, RC 20 Hz, controller output 10 Hz.

### GNSS (`elle-hardware/src/gnss.rs`)

- Primary source UBX-NAV-PVT at 5 Hz (parsed by the `ublox` crate); NMEA GGA is the fallback if configuration fails or PVT is stale for 3 s.
- Boot: cold start → CFG-VALSET groups (Airborne <4g, 5 Hz, NAV-PVT on, GLL/GSA/GSV/VTG/RMC off) → ACK → 115200 baud → probe; no answer falls back to 9600 + NMEA. The accepted groups are reported as `cfg_mask`.
- **RAM layer only**: reapplied every boot, no config-write wear. `DYNMODEL` and `FIXMODE` constrain each other and must go in one VALSET (UBX-21035062 §3.10.5.1).
- `gnss-gsv` requests GSV only on the 115200 link (it would eat 69 % of 9600).
- `hdop` is only meaningful on the GGA path; `h_acc_m` is the real quality gate.

### Logging

| System | Transport | Audience |
|--------|-----------|----------|
| defmt | RTT up 0 | Developer at the probe (flight builds) |
| `LogTopic` | RTT up 1 (RPC) | TUI log panel; numeric event codes |
| ULog | SD card, FAT32 over SPI1 | Post-flight analysis |

Event codes are listed in [`docs/OPERATIONS.md`](docs/OPERATIONS.md#event-codes); the
constants and their ranges are in `elle-hardware/src/event.rs`. ULog goes
`ULogLogger` (`elle-hardware`) → `ULOG_WRITE_CHANNEL` → `sd_writer_task` (its SPI device is wrapped in `YieldingSpi`: sdspi's busy-card poll otherwise never yields and stalls every thread-mode task for tens of ms); message set
and sizes in [`crates/elle-ulog/README.md`](crates/elle-ulog/README.md). `commands`
logs the pre-filter setpoint; `controller` logs what the PID used and did each tick
(dt, attitude age, filtered setpoint, scaled P/I/D per axis, saturation bits, final
elevon pulses); `pid_gains` is written per file and whenever
`FlightController::gains_version()` changes.

## Conventions

- **Behaviour changes go in `elle-app` (or below), once.** A binary change is a hardware change.
- **Constants live in `elle-config`**, with `const _: () = assert!(…)` for invariants; per-platform values are `#[cfg(feature = "platform-dart")]` pairs.
- **Lend large structs to async fns, don't move them.** Passing `FlightController` or `ULogLogger` by value duplicates their storage in every enclosing future (+7.3 kB .bss once); `boot::run` builds the controller and lends `&mut`.
- No `const fn` on `&mut self` methods.
- New event: add the `EVT_*` constant in `event.rs`, a host label in `ui.rs`, and a row in the OPERATIONS.md table. Retired codes stay reserved.
- New pure logic goes in `elle-control` with host tests in `crates/elle-control/tests/`.
- Commit and PR text carries no AI attribution.

## Key Dependencies

| Crate | Firmware | Host | Purpose |
|-------|----------|------|---------|
| embassy-* (git) | yes | - | Async runtime, RP2350 HAL |
| postcard-rpc 0.12 | server (`define_dispatch!`) | `HostClient` | RPC framework |
| probe-rs 0.32 | - | yes | Debug probe + RTT |
| rtt-target 0.6 | yes | - | RTT channels |
| cobs 0.5 | yes | yes | Frame encoding |
| ratatui 0.30 | - | yes | TUI |
| icm426xx (git `ProfFan/icm426xx` rev `7e22a5a`) | yes | - | ICM-42686-P driver. The 42686-P support postdates the 0.4.0 release; switch to crates.io once upstream releases again. |
| ahrs 0.8 · nalgebra 0.34 | yes | - | Madgwick AHRS, linear algebra (`libm`) |
| embassy-dshot 0.5.1 | yes | - | DShot over PIO (own crate). One `BidirDshotProgram` per PIO block; every send is fallible. 0.5.1 adds `command_with_extended_telemetry` (EDT from a stopped motor) and fixes issue #8: every push waits out the previous frame's cycle. |
| ublox 0.10 (`ubx_proto33`) | yes | - | UBX parsing and CFG-VALSET building |
| sequential-storage 8.0 | yes | - | Flash MapStorage (profile) and queue (legacy ULog) |
| embedded-fatfs / sdspi (git `MabezDev/embedded-fatfs` rev `919e569f`) | yes | - | FAT32 on the SD card |
| embedded-hal-bus 0.3 | yes | - | I2C/SPI bus sharing |
| bmp390 (`drivers/bmp390`) | yes | - | Barometer, local fork patched over crates.io |
