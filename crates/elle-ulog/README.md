# elle-ulog

`no_std`, allocation-free encoder for the [PX4 ULog](https://docs.px4.io/main/en/dev_log/ulog_file_format.html)
format, used by the Elle flight controller for flight recording. Output opens in
PlotJuggler, pyulog (`ulog_info`, `ulog2csv`) and PX4 Flight Review.

This crate only **encodes**. Buffering, timing and storage live in `elle-hardware`:

```
elle-app control loop ──log_*()──► elle_hardware::ULogLogger ──512 B chunks──► ULOG_WRITE_CHANNEL
      (83 Hz)                        (2 kB buffer, flush at 75 %)                    │
                                                                                     ▼
                                                                   sd_writer_task → LOG_NNNN.ulg
                                                                   (FAT32 on the SD card, SPI1)
```

The on-board flash ULog region (0x210000–0xFFFFFF) is a legacy store: RPC extract/erase
still target it, but new recordings go to the SD card.

## Messages

| Name | Size (B) | Rate | Content |
|------|---------:|------|---------|
| `attitude_data` | 32 | 83 Hz | pitch/roll/yaw (rad), rates (rad/s, filtered as the PID sees them) |
| `commands` | 49 | 83 Hz | pilot inputs, mode, **pre-filter** setpoints, PID corrections, elevon µs |
| `controller` | 53 | 83 Hz | loop `dt_us`, attitude age, filtered setpoints, scaled P/I/D per axis, `saturation` bits (pitch_up, pitch_down, roll_right, roll_left, LSB first), final elevon pulses |
| `engine_data` | 46 | 83 Hz | per engine: eRPM, DShot throttle, target eRPM, EDT temperature/voltage/current |
| `system_status` | 24 | 8.3 Hz | loop time, IMU errors, calibrated, armed, CPU load, RC age |
| `magnetometer_data` | 20 | ~10 Hz | raw counts |
| `barometer_data` | 24 | ~4.4 Hz | pressure, temperature, altitude, vario |
| `gnss_data` | 58 | ~1 Hz | fix, position, velocity NED, ground speed, course, accuracies, sats |
| `pid_gains` | 40 | per file + on change | Kp/Ki/Kd per axis, I limit, scale |
| `log_event` | 11 | on event | level + event code (see [`docs/OPERATIONS.md`](../../docs/OPERATIONS.md#event-codes)) |
| `autotune_status` | 24 | while tuning | phase, axis, relay state, setpoint, measurement, cycles, amplitude |
| `esc_health` | 44 | ~1 Hz | per ESC, cumulative: telemetry replies, timeouts, corrupt replies (GCR/CRC), re-configurations, extended-telemetry frames |
| `core1_load` | 32 | ~1 Hz | Core 1 IMU task over the window: wake-ups, mean and max busy µs per wake (deadline 1 ms), longest mag and baro read (own task, not part of the busy time), largest FIFO drain |
| `loop_stages` | 46 | 8.3 Hz | flight loop only: mean and max µs per stage over ~10 ticks (intake, update, outputs, switches, autotune, log, tail), DShot executor interrupt time and runs on Core 0 |
| `gyro_raw` | 20 | 1 kHz | unfiltered gyro, `gyro-raw-log` builds only |

Sizes are the payload after the 3-byte message header. Rates are set in `elle-app`
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
