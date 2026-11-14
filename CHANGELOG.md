# Changelog

All notable changes to Elle. Format: [Keep a Changelog](https://keepachangelog.com);
versions follow [`docs/VERSIONING.md`](docs/VERSIONING.md). Breaking entries name
the surface they break: operator behaviour, RPC ICD, flash profile, ULog or event codes.

## [Unreleased]

### Changed
- Eagle elevon trims left +50 → +150 µs, right +100 → −100 µs, `MAX_TRIM_US`
  100 → 150 (tuning, eagle only; round A): after linkage changes both elevons sat
  low at their neutrals (right ~7 mm); the new values are flush on the bench. Upward
  travel now stops at mixer 0.7 left and 0.8 right (`ELEVON_*_MIX_MAX`).
- Eagle pitch gains 0.45 / 0.020 / 0.16 → 0.35 / 0.020 / 0.02 (tuning, eagle only;
  round A): LOG_0082 flew a 3–4 Hz pitch limit cycle, which a closed-loop sim puts
  on kd against the ~55 ms loop delay (estimate). A saved flash PID profile still
  overrides these.
- Control law moved into `elle_control::flight_law::FlightLaw` (internal, no
  surface; proposal 0007): `FlightController::update()` is now one `FlightLaw::step`
  plus the PWM write and events. Behaviour is unchanged, shown by running the old and
  new controllers side by side on a scripted and a flight-derived input stream, bit
  for bit. On target the PWM write and the attitude-age read move by a few µs
  (`controller.pwm_counter_us`, `att_age_us`).
- Control tick phase-locked to the elevon PWM (timing): ticks start 1.5 ms
  (`CONTROL_PWM_LEAD_US`) before a PWM wrap, so a command reaches the servos ~1.3 ms
  after it is written on every boot, instead of anywhere from 0 to 5 ms (fixed per
  boot, re-drawn at each stall resync). The ticker realigns after a stall.
- ULog timing fields (ULog): `attitude_data` is stamped when the IMU sample was
  read (DATA_RDY wake) instead of when it was logged, so it now leads the tick's
  `controller` record by `att_age_us`; `att_age_us` also counts the SPI read and the
  AHRS (~0.1–0.4 ms more). `controller` gains `pwm_counter_us` (59 B): the PWM
  counter at the write, so each tick's latch delay is `5000 − pwm_counter_us`.
- Flight builds no longer count the legacy flash ULog queue ~5 s after boot
  (timing): the count held Core 0 for ~26 ms (LOG_0065, `dt_us` 26 ms at 5.0 s), in
  flight if armed and launched that early. RPC builds still count it for `ulog info`.
- Held-attitude limit 100 → 20 ms (operator behaviour, event codes): when no new
  IMU sample arrives, Stabilized and AltitudeHold keep flying on the last one for
  20 ms instead of 100 ms, then fall back to the sticks. New event 83 when a
  sample is held for 2 ticks; event 80 now also fires once when the held sample
  expires.
- Anti-windup sees trim-limited elevon travel (tuning, both airframes): the mixer
  clamps each elevon and sets the saturation flags at the mix value where its
  trimmed pulse reaches 1000/2000 µs (eagle: left 0.9, right −0.8; dart: left 0.8,
  right 0.81) instead of at ±1, so the I term stops winding while an elevon sits on
  its stop. Output pulses are unchanged (within the LUT's 1 µs step).
- Eagle Stabilized pitch limit ±25° → ±15° (operator behaviour, tuning): full pitch
  stick now asks for 15°. LOG_0082's three departures all began from a 22–25°
  setpoint at a full-throttle launch, where the nose rides above the setpoint and
  full nose-down elevon can't recover it. The dart keeps ±25°.
- Eagle elevon trim (tuning, operator behaviour, every mode): left 100 → 50 µs,
  right −50 → 100 µs, so neutral is 25 µs nose-down and 75 µs left roll instead of
  75 µs nose-up. LOG_0082 held ~100 µs nose-down and ~50 µs left roll in level
  flight, gliding included.

### Added
- Build id in the ULog header (ULog, additive): an info record `ver_sw_git` holds
  the build's `git describe`, `-dirty` when the tree had uncommitted changes
  (now also caught for unstaged source edits). `ver_sw` is unchanged.
- AM32 ESC configuration from the TUI, `elle direct esc …` and `elle mcp`
  (operator behaviour, RPC ICD, event codes; proposal 0002). RPC builds can reboot
  into an ESC-config mode: no DShot, the engine pins carry a one-wire UART to each
  ESC's bootloader, arming is locked (172), flash writes and level cal are refused
  (new event 178), LED double blink orange, CRSF `ESCCFG`. New endpoints `elle/esc/mode`,
  `elle/esc/mode/enter`, `elle/esc/mode/exit`, `elle/esc/connect`,
  `elle/esc/eeprom/read`, `elle/esc/eeprom/write` (verified by read-back),
  `elle/esc/name`, `elle/esc/read`, `elle/esc/run`, with `ESC_MODE_REFUSED_ARMED` /
  `ESC_MODE_BUSY` ack codes; events 170–178. Host: settings decoding and the
  esc-config-rs host policy, raw 256-byte backups in the relay CLI's format, a TUI
  ESC panel, and MCP tools `esc_mode`, `esc_connect`, `esc_read`, `esc_backup`,
  `esc_compare`, `esc_set`, `esc_restore`, `esc_run` (writes need
  `--dangerously-allow-esc-write`). Depends on esc-config-rs by git rev. Not yet run
  on hardware (TEST_PLAN 6.11).

- Magnetometer health (operator behaviour, event codes): boot self-test (event 49,
  mag not used on failure); median-of-3 spike filter (event 119) ahead of the
  calibration and the health check;
  `MagHealth` reports a corrected field outside 0.15–0.85 G or frozen for 2.5 s
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
- Disarm stops the engines at once (operator behaviour, proposal 0008): the DShot
  task played the disarm beep (with ~300 ms of sleeps) before reading the new target,
  so after every disarm, kill switch and failsafe included, the engines kept their
  last throttle for ~330 ms (bench LOG_0094). The target is now read first and
  `MotorStop` goes out on the next frame; beeps play only with zero targets and the
  engines at rest, and a throttle-up cancels one.
- SD writes are batched into multi-block writes (ULog recording): the writer wrote
  each queued chunk as it came, which left the file position off the sector grid, so
  every 512 B cost a sector read, a single-block write and its busy wait. An
  `imu-raw-log` build produced ~47 kB/s and got ~27 kB/s onto the card (LOG_0091:
  7924 dropouts, every stream at ~57 % of its rate); a normal build at ~33 kB/s
  ran at the edge (LOG_0090). Chunks are now collected up to `SD_WRITE_BATCH_SIZE`
  (16 KB) and written as whole sectors in one card command; the tail of less than a
  sector waits in RAM for the next write or the file close. The ~1 s flush now also
  runs while the writer is working through a backlog. +16 KB static RAM. Bench,
  eagle, `imu-raw-log`, armed at 30-70 %: 4 KB still lost 2-8 % of records while
  armed (LOG_0092); 16 KB lost none, and `elle-replay` reproduced the attitude
  exactly (LOG_0093).
- `elle-replay`: a ULog dropout stops the replay's sync until the next context,
  since a lost mag change or context leaves no break in the sample indices.
- A watchdog reset no longer leaves the FC dead (operator behaviour): the AON
  timer survives it, and the boot's `set_counter` on a running timer panicked
  into HardFault (no RTT, no DShot, no elevons) until a power cycle. Any watchdog
  reset hit it, a watchdog timeout included; the ESC-config mode switch made it
  show. The boot now stops the timer before reseeding it.
- ULog keeps its last data on `ulog stop` (ULog): the SD writer now writes the
  chunks still queued for the session before closing the file instead of
  discarding them.
- Mag calibration accepted a disturbed field: it now also requires the readings to
  sit on an earth-sized sphere (RMS radius 0.15–0.85 G) around the fitted centre,
  and filters spikes first. Dart logs had four "successful" calibrations with a
  3.3–7.6 G radius; the eagle's spikes (~(0, +5000, −6000) counts, 2–3 % of reads)
  alone satisfied the old 5000-count span check.
- MMC5616WA status masks: `MEAS_M_DONE` / `MEAS_T_DONE` named bits 0 and 1, which
  are the I3C `_int` flags (datasheet p12). Renamed to `*_INT` (one-shot polling
  unchanged); bits 4–7 added under their real names.
- Mag cal ignored at boot (operator behaviour): stored hard-iron offsets were
  accepted only below 100 000 counts, a 16-bit assumption, while the MMC5616WA
  reads signed 20-bit counts (±524 288). The eagle's x offset (~242 000) was
  rejected on every boot ("invalid data in flash"), so it ran without mag cal and
  its yaw locked onto the uncorrected field. The bound is now the sensor's full
  scale, and a rejection logs the values.
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
