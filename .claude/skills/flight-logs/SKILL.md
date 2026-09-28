---
name: flight-logs
description: Read and analyse Elle ULog flight logs (LOG_NNNN.ulg from the SD card) — find the right files, summarise a session, check control-loop and Core 1 timing, ESC link health, sensor rates, events, and compare two builds. Use whenever the user says a new log is on the card, asks what happened in a flight or bench run, or wants a timing/regression comparison.
---

# Reading Elle flight logs

The firmware records every session to the SD card as `LOG_NNNN.ulg` (PX4 ULog,
FAT32, one file per boot; logs from older firmware are `LOG_NNNN.ULG`). Message
set and sizes: [`crates/elle-ulog/README.md`](../../../crates/elle-ulog/README.md).
Event codes: the host labels in `tools/elle-rpc-host/src/tui/ui.rs`
(`log_code_text`), also tabled in `docs/OPERATIONS.md`.

## 1. Get the files

The card mounts at `/run/media/$USER/<label>/` (it has been `F22D-C1B3`). If
nothing is mounted, `lsblk` shows whether the reader sees a card at all — ask the
user to reseat it rather than guessing. Copy the files you need into the repo's
`logs/` (gitignored, like `*.ulg`) so they survive the card being pulled:

```sh
mkdir -p logs && cp /run/media/$USER/*/LOG_00{52..54}.ulg logs/
```

File dates come from the AON clock seeded at build time, so they are only
roughly right; the number is the reliable order. Power cycles and `cargo run`
each start a new file, so a test session is often several files: pick them with
`list` (step 3), not by guessing.

## 2. Tooling

```sh
python3 -m venv logs/.venv && logs/.venv/bin/pip install -q pyulog numpy   # once
PY=logs/.venv/bin/python; LOG=.claude/skills/flight-logs/elle_log.py
```

`elle_log.py` covers the analyses that keep coming up. Every subcommand takes one
or more files and degrades gracefully on older logs that lack newer messages:

| Command | Shows |
|---|---|
| `list` | duration, armed time, mag change rate, which newer messages exist — use it to tell sessions and builds apart |
| `summary` | armed intervals, mode share, event counts and timeline (labels from `ui.rs`), ULog dropouts |
| `timing` | tick period and late ticks (> 24 ms) per 10 s, `loop_time_us` by armed/mode/engines, Core 1 load and FIFO backlogs |
| `esc` | per-ESC replies / timeouts / corrupt replies / re-configurations, target-while-disarmed check, whether EDT (voltage) ever arrived |
| `sensors` | mag and baro logged rate vs value-change rate, GNSS sats/fix, attitude ranges |
| `stages` | flight-loop time per stage (`loop_stages`: intake, update, outputs, switches, autotune, log, tail) by armed/mode/engines, and the DShot executor's share of Core 0 |
| `window FILE T0 T1` | events, late ticks, loop time, RC age, modes between two times (seconds from the file's first record) |

For anything else, load the file directly — `pyulog.ULog(path).data_list` gives one
numpy dict per message — and keep the ad-hoc script in the scratchpad.

## 3. Identify the build

Messages were added over time, so their presence dates a log:

| Present | Build has |
|---|---|
| `controller` | per-tick controller internals (before it, use `commands` timestamps for tick timing) |
| `esc_health` | DShot timing work: idle telemetry, ESC re-configuration |
| `core1_load` | Core 1 load logging (and the async mag/baro task) |
| mag value changes ≈ 10/s | the MMC5616WA `Cmm_freq_en` fix; ≈ 1/s means before it |

When comparing two builds, ask the user to run both under the same procedure
(same transmitter settings, same probe state, power-cycled after flashing, the
same minutes in each mode) and say which files are which only after checking with
`list`.

## 4. Reading the numbers correctly

- **Time.** Timestamps are µs since boot; the tool reports seconds from the file's
  first record, which is 0.1–5 s after boot (the SD card has to mount first).
- **`loop_time_us`** (`system_status`, 8.3 Hz) is wall time from tick start to the
  ULog write. It includes interrupt and DShot-executor preemption. While armed the
  flight loop has no `await` before that point, so other thread tasks are not in
  it; while disarmed, the level-cal poll can be.
- **Late ticks.** A tick gap over 24 ms is what `support::resync_after_stall`
  warns about. Bursts of them mean a thread-mode task stopped yielding (the SD
  busy-wait was one); check `esc` to see whether DShot kept 1000 replies/s through
  them. A gap with a ULog dropout inside it is lost log data, not a stall: the
  SD writer fell behind and records were discarded while the loop kept running.
  `timing` and `window` report those separately as logging gaps.
- **Core 1** (`core1_load`, ~1 Hz windows): the first window covers boot — skip it.
  `busy_*` is per IMU wake-up against a 1 ms deadline; `max_drain` > 1 means samples
  queued. Mag/baro durations come from the separate I2C task and include waiting
  for the IMU task.
- **ESC telemetry.** At zero target the ESC is sent MotorStop with a telemetry
  request, so eRPM at idle is real (older logs fabricated 0). `replies` counts any
  valid reply; EDT fields (voltage, temperature) only arrive when extended
  telemetry is enabled — a session with thousands of replies and voltage 0 means
  the EDT enable did not take. Newer logs count EDT frames directly
  (`esc_health` `*_edt_frames`), and the firmware re-sends the configuration when
  they don't arrive (event 162/163, then 164/165 if it gives up).
- **Sentinels and quirks.** `controller.att_age_us` = 4294967295 means no attitude
  age; `controller.dt_us` accumulates while the kill switch blocks updates; event 45
  is rate-limited to once per second; events 2 and 7 (and 23) are periodic
  re-announcements, not state changes; a stale mag/baro value shows as a low
  value-change rate, not a gap.
- **Modes.** `commands.attitude_mode`: 0 Manual, 1 Stabilized, 2 AltitudeHold.

## 5. Report

Lead with the answer to the user's question, then the numbers that support it in
a small table (file, build, condition, p50/p90). Say which files you used and why
the others were excluded. Distinguish measured from inferred, and say when a
difference could be setup (probe, transmitter, GNSS state) rather than code.
