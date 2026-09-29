# Elle

[![CI](https://github.com/Ragarnoy/elle-rs/actions/workflows/ci.yaml/badge.svg)](https://github.com/Ragarnoy/elle-rs/actions/workflows/ci.yaml)

Flight controller firmware for two small flying wings, in Rust on the RP2350 with
[Embassy](https://embassy.dev) (async, `no_std`):

- **Eagle** — twin EDF, differential thrust
- **Dart** — single engine

Both run the same application: ICM-42686 IMU with Madgwick AHRS on core 1, 200 Hz
attitude control on core 0, elevons on hardware PWM, bidirectional DShot with a closed-loop
RPM governor, CRSF/ELRS receiver and telemetry, SAM-M10Q GNSS, BMP390 baro, MMC5616WA
magnetometer, and ULog flight recording to an SD card. A host tool drives and monitors
the board over a debug probe.

## Layout

| Path | Contents |
|------|----------|
| `crates/elle-eagle`, `crates/elle-dart` | The two firmware binaries (hardware setup only) |
| `crates/elle-app` | The application: boot, control loops, RPC dispatch |
| `crates/elle-system` | Flight controller and RPC transport |
| `crates/elle-hardware` | Drivers and hardware tasks |
| `crates/elle-control` | Control algorithms (PID, governor, autotune, calibration), host-tested |
| `crates/elle-config` | Every tunable constant, per platform |
| `crates/elle-rpc-icd`, `crates/elle-ulog`, `crates/elle-error`, `crates/elle-nav` | RPC contract, ULog encoder, errors, navigation (observation mode: computed and logged, not applied) |
| `drivers/` | Vendored sensor drivers |
| `tools/elle-rpc-host` | Host TUI and command-line tool |
| `tools/elle-replay` | Replays raw IMU logs through the attitude pipeline on the host |

## Quick start

```sh
# Firmware, flight mode (flashes through probe-rs)
cd crates/elle-eagle    # or crates/elle-dart
cargo run --release

# Firmware, ground-test mode with the RPC server (keep `gnss`)
cargo run --release --no-default-features --features rpc-control,gnss

# Host dashboard (the workspace defaults to the thumbv8m target)
cargo run -p elle-rpc-host --target x86_64-unknown-linux-gnu
```

## Documentation

| Doc | For |
|-----|-----|
| [`docs/OPERATIONS.md`](docs/OPERATIONS.md) | **Flying it:** arming, failsafe, RC channels, modes, calibration, autotune, recording, LED and event codes |
| [`CLAUDE.md`](CLAUDE.md) | **Developing it:** architecture, features, pins, conventions, CI |
| [`TEST_PLAN.md`](TEST_PLAN.md) | Bench and field checks before flight |
| [`STATE_DIAGRAMS.md`](STATE_DIAGRAMS.md) | State machines |
| [`TODO.md`](TODO.md) | Backlog and items awaiting hardware verification |
| [`tools/elle-rpc-host/README.md`](tools/elle-rpc-host/README.md) | Host tool |
| [`crates/elle-ulog/README.md`](crates/elle-ulog/README.md) | Flight log format |
| [`docs/DART_PID.md`](docs/DART_PID.md) | How the PID gains were derived |
| [`docs/NAVIGATION_PLAN.md`](docs/NAVIGATION_PLAN.md) | Navigation plan |
| [`docs/ATTITUDE.md`](docs/ATTITUDE.md) | Attitude estimation and turn compensation |
| [`docs/embassy-rp-sio-irq-fifo-flash-bug.md`](docs/embassy-rp-sio-irq-fifo-flash-bug.md) | An embassy-rp multicore flash bug and its workaround |
