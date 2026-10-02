# elle-ulog

`no_std`, allocation-free encoder for the [PX4 ULog](https://docs.px4.io/main/en/dev_log/ulog_file_format.html)
format, used by the Elle flight controller for flight recording. Output opens in
PlotJuggler, pyulog (`ulog_info`, `ulog2csv`) and PX4 Flight Review.

This crate only **encodes**. Buffering, timing and storage live in `elle-hardware`:

```
elle-app control loop ──log_*()──► elle_hardware::ULogLogger ──512 B chunks──► ULOG_WRITE_CHANNEL
      (200 Hz)                       (2 kB buffer, flush at 75 %)  64 slots, ~1.1 s  │
                                                                                     ▼
                                                                   sd_writer_task → LOG_NNNN.ulg
                                                                   (FAT32 on the SD card, SPI1)
```

The on-board flash ULog region (0x210000–0xFFFFFF) is a legacy store: RPC extract/erase
still target it, but new recordings go to the SD card.

## Messages

| Name | Size (B) | Rate | Content |
|------|---------:|------|---------|
| `attitude_data` | 32 | 200 Hz | pitch/roll/yaw (rad), rates (rad/s, filtered as the PID sees them) |
| `commands` | 49 | 100 Hz | pilot inputs, mode, **pre-filter** setpoints, PID corrections, elevon µs |
| `controller` | 57 | 200 Hz | loop `dt_us`, attitude age, filtered setpoints, scaled P/I/D per axis, `saturation` bits (pitch_up, pitch_down, roll_right, roll_left, LSB first), final elevon pulses, `yaw_damp` (yaw damper command into the differential thrust, normalized yaw, positive slows the left engine; 0 when off) |
| `engine_data` | 46 | 100 Hz | per engine: eRPM, DShot throttle, target eRPM, EDT temperature/voltage/current |
| `system_status` | 24 | 8 Hz | loop time, IMU errors, calibrated, armed, CPU load, RC age |
| `magnetometer_data` | 20 | ~10 Hz | raw counts |
| `barometer_data` | 24 | 5 Hz | pressure, temperature, altitude, vario |
| `gnss_data` | 59 | 5 Hz (per solution) | timestamped with the solution's receive time: fix, lat/lon (degrees × 10⁷), velocity NED, ground speed, course, accuracies, sats, `pvt_active`; velocity and accuracies are NaN on the NMEA fallback |
| `pid_gains` | 40 | per file + on change | Kp/Ki/Kd per axis, I limit, scale |
| `log_event` | 11 | on event | level + event code (see [`docs/OPERATIONS.md`](../../docs/OPERATIONS.md#event-codes)) |
| `autotune_status` | 24 | while tuning | phase, axis, relay state, setpoint, measurement, cycles, amplitude |
| `esc_health` | 44 | ~1 Hz | per ESC, cumulative: telemetry replies, timeouts, corrupt replies (GCR/CRC), re-configurations, extended-telemetry frames |
| `core1_load` | 32 | ~1 Hz | Core 1 IMU task over the window: wake-ups, mean and max busy µs per wake (deadline 1 ms), longest mag and baro read (own task, not part of the busy time), largest FIFO drain |
| `loop_stages` | 46 | 8 Hz | flight loop only: mean and max µs per stage over 25 ticks (intake, update, outputs, switches, autotune, log, tail), DShot executor interrupt time and runs on Core 0 |
| `nav` | 64 | 25 Hz | navigator, observation mode (`gnss` builds): `status` bits (`elle_nav::status`), fix age, position/velocity north-east of home, baro and GNSS height above home, distance/bearing to home, track error, lateral acceleration and bank demand for a loiter around home (not applied), measured roll; invalid fields NaN |
| `imu_raw` | 195 | 100 Hz (10 samples each) | `imu-raw-log` builds: first sample index, 10 × gyro + accel as the 20-bit FIFO integers (24-bit LE), IMU temperature; timestamp = when the first sample was read |
| `imu_raw_mag` | 25 | on change (~10 Hz) | `imu-raw-log` builds: mag vector as fed to the AHRS (airframe frame), from sample `index` on |
| `imu_raw_ctx` | 85 | 1 Hz + on change | `imu-raw-log` builds: AHRS quaternion entering sample `index`, gyro bias and mount it was fused with (board orientation × level-cal tilt, proposal 0003), encode round-trip errors, turn compensation state (`aid_state`) and the build's `turn_comp` mode and `gate_g` |
| `imu_raw_fix` | 25 | ~5 Hz, turn compensation on | `imu-raw-log` builds: each GNSS fix handed to the attitude pipeline (receive time, first sample `index`, NED velocity, `pvt`) |

Sizes are the payload after the 3-byte message header. In `imu-raw-log` builds
`attitude_data` is logged at 50 Hz (a replay regenerates every sample).

The whole header (definitions, info, one subscription per message) is written into
the writer's 4 KB buffer before the first flush; `tests/header.rs` fails when less
than 256 B would be left (3.7 KB used today). Rates are set in `elle-app`
(`ULOG_*_DIVISOR` in `elle-config`).

Each message struct carries its ULog format string as `FORMAT`; the `'F'` definition
record `FORMAT_MSG` is derived from it at compile time by `format::format_msg`, which
checks the length prefix, so a field change can't leave a stale header behind. Add a
field by editing the struct, its `FORMAT`, `SIZE` and `to_bytes()` together.

## API

- `ULogWriter` (this crate): `initialize(start)`, `write_definitions(...)`,
  `add_subscription(name) -> msg_id`, one `write_*` per message into an internal buffer,
  then `buffer()` / `clear_buffer()`. Synchronous and `no_std`.
- `elle_hardware::ULogLogger` (the one the firmware uses): `initialize(epoch_ms).await`
  writes the header, definitions, info and subscriptions; `log_*()` and `flush()` are
  synchronous and never block the control loop. If the channel is full the flush is
  dropped and the next one starts with a ULog dropout (`'O'`) record covering the gap.

## File layout

```
Header (16 B)      "ULog" magic, version 1, start timestamp (µs)
Definitions        flag bits 'B', formats 'F', info 'I' (sys_name, ver_hw, ver_sw, sys_start_time_utc_ms)
Data               subscriptions 'A', then data 'D', dropouts 'O'
```

Little-endian throughout. Timestamps are µs since boot; the wall-clock start time
comes from the AON timer (seeded from the build time) and names the file date on the SD
card.
