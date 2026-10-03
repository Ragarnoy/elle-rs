# Changelog

All notable changes to Elle. Format: [Keep a Changelog](https://keepachangelog.com);
versions follow [`docs/VERSIONING.md`](docs/VERSIONING.md). Breaking entries name
the surface they break: operator behaviour, RPC ICD, flash profile, ULog or event codes.

## [Unreleased]

### Added
- Magnetometer health (operator behaviour, event codes): boot self-test (event 49,
  mag not used on failure); median-of-3 spike filter (event 119) ahead of the
  calibration and the health check;
  `MagHealth` reports a corrected field outside 0.15–0.85 G or frozen for 2 s
  (117) and its recovery (118). Reported only: the AHRS fuses the raw corrected
  mag exactly as before (`MAG_GATE_ENFORCED` off) until a proposal decides from
  flight logs.

### Changed
- Dart roll Kd 0.07 → 0.04 (tuning): 0.25 / 0.012 / 0.07 flew a ~4 Hz roll limit
  cycle that grows with throttle (LOG_0050). It flew again at 0.04 (LOG_0052)
  unchanged; see docs/DART_PID.md. Roll Kp 0.25 → 0.18 for the next flight
  (0.125 flew without the oscillation). Pitch unchanged.
- Dart elevon trim (tuning, operator behaviour): both elevons 95 µs more nose-up
  at neutral, in every mode. LOG_0052 held ~257 µs nose-up in calm Stabilized
  flight and was at the nose-up limit 43% of the time.

### Fixed
- Mag calibration accepted a disturbed field: it now also requires the readings to
  sit on an earth-sized sphere (RMS radius 0.15–0.85 G) around the fitted centre,
  and filters spikes first. Dart logs had four "successful" calibrations with a
  3.3–7.6 G radius; the eagle's spikes (~(0, +5000, −6000) counts, 2–3 % of reads)
  alone satisfied the old 5000-count span check.
- MMC5616WA status masks: `MEAS_M_DONE` / `MEAS_T_DONE` named bits 0 and 1, which
  are the I3C `_int` flags (datasheet p12). Renamed to `*_INT` (one-shot polling
  unchanged); bits 4–7 added under their real names.
- Eagle attitude signs (operator behaviour, proposal 0003): pitch and roll both read
  reversed because the eagle's controller is mounted turned round, and the firmware
  had one sensor mapping for both airframes. New per-platform `BOARD_YAW_DEG`
  (eagle 180°, dart 0°) rotates accel, gyro and mag before the level-cal tilt. The
  dart is unchanged bit for bit; the eagle's stored level cal stays valid, its
  reported angles are now in the airframe's frame. `imu_raw_ctx.mount` holds the
  composed rotation.
- GNSS configuration on boot (operator behaviour): waiting for each CFG-VALSET
  acknowledgement gave up on the first UART error, and the framing / overrun
  flags left by the baud switch failed every key within milliseconds, so the
  module ran its defaults (1 Hz, default dynamic model, NMEA on). Read errors are
  now skipped until the 400 ms timeout, and event 140's text counts them. On the
  eagle a power-on boot now configures fully (`cfg_mask` 0x3FF, 5 Hz).
- GNSS boot after an MCU-only reset (flash, `cargo run`): the task now listens
  at 115200 for up to 1.2 s first and, when the module is already there, skips
  the 9600-baud reset and baud switch, which reached it as garbage and could
  abandon configuration. A power-on boot reaches GNSS up to 1.2 s later.
- GNSS baud probe: every baud change now discards the RX ring first. Bytes left
  over from 9600 still decoded as valid frames at 115200, so a 9600 module could
  pass for one already at 115200 and lose GNSS for the session.
- GNSS configuration mask after an MCU-only reset: the module does not answer a
  CFG-VALSET that changes nothing, so the dynamic-model group (already set from
  the last session) was reported lost (`cfg_mask` 0x3FE). An unanswered group is
  now read back with CFG-VALGET and counted as applied when the values are in
  place (`sam-m10q`: `build_valget`, `valget_matches`, `poll_matches`).
- RPC transmit over RTT (`elle_system::rpc::RttTx`): encoding a reply no longer
  runs inside a critical section, which held off every interrupt (the DShot
  executor included) for the whole send; a reply near 1 KB no longer panics the
  firmware (the COBS buffer was smaller than the worst-case encoding); a frame and
  its delimiter are written in one RTT write, so a full channel drops whole frames
  instead of gluing two together; an oversized message is an error instead of
  being dropped silently. Internal; no compatibility surface changes.

## [0.3.0] - 2026-10-02

### Added
- Yaw damper (`elle_control::yaw_damper`, proposal 0001): washed-out yaw rate into
  the eagle's differential thrust in Stabilized and AltitudeHold. Ships **inert**
  (`YAW_DAMPER_GAIN` = 0 on both airframes), so flight behaviour is unchanged.

### ULog
- **Breaking:** `nav.status` bit 5 now means *coasting*: no usable fix for more
  than `NAV_COAST_AFTER_MS` (300 ms), until position drops at 1 s. It used to be
  set whenever velocity was valid, which on a live stream was almost always. The
  bit is renamed `elle_nav::status::COASTING`. **RPC ICD:** `NavResp.status` bit 5
  changes meaning in the same way (layout unchanged).
- `controller` gains `yaw_damp` (float, the damper's command, 0 while it is off);
  57 B per message, up from 53. Added field, not breaking.

### Fixed
- Builds with Rust 1.99: `fetch_update` → `try_update` in the flash manager
  (clippy `-D warnings` failed on the deprecation).

## [0.2.0] - 2026-10-01

First tagged version. Everything before it was `0.1.0`; see `git log` for the
history. This entry records the baseline that later breaking changes are measured
against.

### Operator behaviour
- Gesture arming (throttle high, then zero thrust), kill switch, RC failsafe
  (warning 200 ms, timeout 300 ms); explicit arming only in pure RPC mode, where the
  failsafe ages host messages instead.
- Stabilized (±25° pitch, ±45° roll), AltitudeHold as a level hold, heading hold.
- Relay autotune with a ±20° envelope; saves deferred until disarm.
- Mag and level calibration; no flash write while armed.

### RPC ICD
- Control, safety, query, ULog, autotune, calibration and system endpoints as listed
  in `CLAUDE.md`, plus the `LogTopic` event topic. `GetBuildInfo` reports platform,
  features, turn compensation, loop rate and `git describe`.

### Flash profile
- Keys: 1 PID gains, 2 mag cal offsets, 3 level cal quaternion (`MAP_KEY_SLOTS` 4).

### ULog
- Message set and sizes as in `crates/elle-ulog/README.md`, including `nav`
  (observation mode), `esc_health`, `core1_load`, `loop_stages` and the
  `imu-raw-log` messages.

### Event codes
- As in `docs/OPERATIONS.md#event-codes`.

[Unreleased]: https://github.com/Ragarnoy/elle-rs/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/Ragarnoy/elle-rs/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/Ragarnoy/elle-rs/releases/tag/v0.2.0
