# Changelog

All notable changes to Elle. Format: [Keep a Changelog](https://keepachangelog.com);
versions follow [`docs/VERSIONING.md`](docs/VERSIONING.md). Breaking entries name
the surface they break: operator behaviour, RPC ICD, flash profile, ULog or event codes.

## [Unreleased]

### Added
- Yaw damper (`elle_control::yaw_damper`, proposal 0001): washed-out yaw rate into
  the eagle's differential thrust in Stabilized and AltitudeHold. Ships **inert**
  (`YAW_DAMPER_GAIN` = 0 on both airframes), so flight behaviour is unchanged.

### ULog
- `controller` gains `yaw_damp` (float, the damper's command, 0 while it is off);
  57 B per message, up from 53. Added field, not breaking.

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

[Unreleased]: https://github.com/Ragarnoy/elle-rs/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/Ragarnoy/elle-rs/releases/tag/v0.2.0
