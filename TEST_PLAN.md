# Ground Test Plan

Pre-flight verification checklist. Complete ALL tests before flight.

Written for the eagle. On the **dart** (single engine) skip the differential-thrust rows
(1.2, 5.2 #4) and 3.4.3 — `IGNORE_PID_FLASH` means tuned gains are never saved. How
arming, failsafe, LEDs and event codes work is in
[`docs/OPERATIONS.md`](docs/OPERATIONS.md).

## Prerequisites

- BetaFPV Pro transmitter bound and linked
- CH5 (2-pos): Heading hold
- CH6 (3-pos switch left): Manual / Stabilized / AltitudeHold
- CH7 (3-pos switch right): Autotune off / pitch / roll (flight firmware only)
- CH8 (2-pos switch right): Kill switch (high = disarm)
- Props removed for all ground tests
- SD card inserted (FAT32)
- Probe-rs debug probe connected (RPC tests only)
- Keep the aircraft still for ~1 s after every power-up (gyro bias; LED goes solid green,
  or solid purple in RPC builds)
- **Arming** is a gesture: throttle above ~30 %, then back to zero. Written "arm (gesture)"
  below. Kill, failsafe and every disarm require a new gesture.

## Build Commands

```sh
# Flight firmware (default — CRSF/ELRS)
cd crates/elle-eagle          # or crates/elle-dart
cargo run --release

# RPC firmware (ground test mode). Always list gnss: --no-default-features drops it.
cargo run --release --no-default-features --features rpc-control,gnss

# RPC + RC firmware (probe monitoring with RC control)
cargo run --release --no-default-features --features rpc-control,rpc-rc,gnss

# Host TUI
cargo run -p elle-rpc-host --target x86_64-unknown-linux-gnu
```

## Running it through `elle mcp`

`elle mcp` ([`tools/elle-rpc-host/README.md`](tools/elle-rpc-host/README.md#mcp-server-elle-mcp))
lets an agent run most of this plan: it flashes builds, reads every query endpoint,
waits for conditions and events while the operator acts, fetches the SD card logs,
analyses them and records each result (`test_record`, in `logs/test-runs/<date>.jsonl`).
The `test-plan-runner` skill walks a Part through it. How each section runs:

- **auto**: the agent does it alone.
- **assisted**: the operator does something physical (tilt, switch, TX off, walk); the
  agent tells them what, then verifies with `wait_for`, `wait_event` or `sample`.
- **log**: checked afterwards from the SD card log (`copy_logs`, `analyse_log`, `replay`).
- **manual**: needs eyes, ears, a scope or a flight; the agent only records what the
  operator reports.

Engines only spin with `--dangerously-allow-motors` on the server (rows marked
**motors**), props off. The probe has one owner: the TUI and the server cannot run
together, so TUI rows run as their `elle mcp` equivalent. Flight builds have no RPC:
their rows are assisted through the defmt log (`log`, `wait_log`) or checked from the log.

| Section | How | Tools and notes |
|---|---|---|
| 0.1 | log; rows 1, 2, 4 assisted on an RPC build | `sample attitude` (drift, 10 min), `wait_for pitch_deg`; row 3 as 1.3 |
| 0.2 | log; row 5 manual (scope) | operator runs the armed session on the flight build; `analyse_log timing / stages / list` |
| 0.3 | assisted outdoors; row 1 manual (radio text) | `read nav` (bits), `read gnss` |
| 0.4 | log | `analyse_log timing`, `replay` (exact), `replay compare` |
| 0.5 | manual (flight) | afterwards: log, as 7.3 |
| 1.1, 1.2 | assisted, motors for 1.2 | RPC+RC build; operator moves sticks, agent reads `rc` and `controller` pulses / `engine`; operator confirms the physical direction |
| 1.3 | assisted | operator tilts; `wait_for`, then the correction signs in `controller` |
| 1.4 | assisted, motors | `wait_event` 10 / 16 / 17; beeps by ear |
| 2.1–2.5 | assisted | `read status` (mode), `controller` (setpoints, corrections); 2.5 rows 3–4 with `autotune` |
| 3.1, 3.2 | auto + log | `ulog start/stop`, `reset_target`, `copy_logs`, `analyse_log list` |
| 3.3 | assisted + log | `autotune pitch/abort`, `read status`; rows 5–7 from the log |
| 3.4.1–3.4.5 | assisted | operator tilts to follow the elevons; `wait_event` 90–94, `read controller`; 3.4.3 with `reset_target` for the power cycle |
| 3.4.6 | assisted on the flight build | `wait_log`; CH7 by the operator |
| 4.1 | auto (`set_mode` per mode) | the TUI's command parser itself is not covered |
| 4.2 | auto, row 5 assisted | `set_mode`, `arm` (motors), `set_elevons`, `read controller` |
| 5.x | log | `analyse_log`; plots by hand in PlotJuggler where a row says plot |
| 6.1 | assisted, motors | RPC+RC build; TX off; `wait_event` 13 / 14 / 15 |
| 6.2 | rows 1–2 auto (motors); rows 3–6 manual | the server always disarms before letting go, so the host-loss rows run with the TUI and `direct` (Part 8 row 8 covers the server) |
| 6.3 | auto, motors (arming at throttle 0) | `arm`, `autotune save_pid`, `mag_cal start`, `level_cal start`, `wait_event` 63 / 100 / 152; power cycles as `reset_target` |
| 6.4 | row 1 auto, row 2 assisted | `reset_target`, `wait_event` 46 / 47 |
| 6.5, 6.6 | assisted | `wait_event` 130 / 131, 48; `read mag` stops changing |
| 6.7 | manual (scope); row 2 auto | `events` 45 / 41 |
| 6.8 | rows 1–2 manual (scope); 3, 6 log; 4–5 assisted | `wait_event` 160–163, `analyse_log esc` |
| 6.9 | rows 1–3 assisted outdoors (laptop and probe on the aircraft); 4–5 log | `read nav`, `read gnss`, `analyse_log nav` |
| 6.10 | log; row 5 manual | `replay`, `analyse_log timing` |
| 7.1 | auto | `cargo test` from a shell; `replay simulate` |
| 7.2–7.4 | log | `replay compare score_by_rate` (7.2), `replay compare` (7.3) |
| 7.5–7.7 | as the rows they repeat; flight manual | |
| 8 | auto / assisted | validates the server itself: run it first |

---

## Part 0: Untested Changes (bench session before the next flight)

**Nothing merged since the 200 Hz loop (#33) has run on the aircraft.** That covers the
200 Hz control loop and the 100 Hz logging it now uses (#33), navigation observation and
the reworked GNSS data (#34), and the `imu-replay` branch, which moved the attitude
fusion into `elle-control`, swapped the filter library (`ahrs` → uf-ahrs) and added turn
compensation (off). That changes the code Stabilized flies on. Host tests show the
fusion is unchanged (bit-identical after the move, within 3e-5° after the swap, and
compensation off is the plain filter bit for bit), but only the aircraft can confirm it.
Turning compensation on is Part 7, after this session. The `elle mcp` server has
not touched the hardware either: Part 8, before using it for anything else.

Run this session in order on the **eagle**, props off, then 0.1–0.4 on the dart. Stop at
the first failure. **No flight until 0.1–0.4 pass.** Copy the logs into `logs/` and
note the file numbers in each row. Baselines are from LOG_0065 (200 Hz loop, before
these changes): armed loop time p50 237 µs / p99 949 µs, Core 1 busy mean 373 µs /
max 869 µs.

```sh
PY=logs/.venv/bin/python; LOG=.claude/skills/flight-logs/elle_log.py
```

### 0.1 Attitude (normal flight build, SD card in)

| # | Test | Expected | Log | Pass |
|---|------|----------|-----|------|
| 1 | Power up flat and still, leave it 10 min disarmed | Pitch and roll within ±0.5° of the pre-change reading on the same surface, drift < 0.5° over the 10 min (`$PY $LOG sensors`) | | [ ] |
| 2 | Tilt nose up, nose down, right wing down, left wing down, ~20° each | Pitch positive nose up, roll positive right wing down (CRSF attitude on the radio, then `attitude_data`) | | [ ] |
| 3 | Part 1.3 in full (Stabilized, armed, in hand) | All seven rows as before: corrects against the tilt, D damps quick rotations | | [ ] |
| 4 | Rotate 360° in yaw on the bench, slowly | Yaw follows and returns to within ~5° of the start; no jump when the mag reading updates | | [ ] |

### 0.2 Timing and logging (normal flight build)

| # | Test | Expected | Log | Pass |
|---|------|----------|-----|------|
| 1 | Armed 15+ min: idle, throttle steps, Manual and Stabilized, until the log passes ~4 MB | `$PY $LOG timing`: median tick 5.00 ms, no late ticks after boot, **no ULog dropouts** | | [ ] |
| 2 | Same log | Loop time p50/p99 within ~10 % of the baseline; `$PY $LOG stages`: `log` stage not noticeably larger (the navigator runs there, 25 Hz) | | [ ] |
| 3 | Same log | `core1_load` busy mean/max within ~10 % of the baseline | | [ ] |
| 4 | Same log | `commands` and `engine_data` at ~100 Hz, `attitude_data` and `controller` at ~200 Hz, `gnss_data` at ~5 Hz (`$PY $LOG list`) | | [ ] |
| 5 | Scope PIN_12/13 against a stick step (6.7 row 1) | New pulse within one 5 ms frame | | [ ] |

### 0.3 GNSS and navigation observation (outdoors, normal flight build)

| # | Test | Expected | Log | Pass |
|---|------|----------|-----|------|
| 1 | Power up outdoors, wait for the fix | Radio FM text `NOHOME` → `WAIT H` once hAcc ≤ 5 m and ≥ 6 satellites | | [ ] |
| 2 | 6.9 rows 1–3 (home, lock while armed, GNSS covered 5 s) | As in 6.9 | | [ ] |
| 3 | TUI (RPC build) GNSS panel | Lat/lon as before (now from integer degrees × 10⁷); Spd/Trk `---` if it falls back to NMEA | | [ ] |

### 0.4 Raw IMU capture and replay (`--features imu-raw-log`)

| # | Test | Expected | Log | Pass |
|---|------|----------|-----|------|
| 1 | Repeat 0.2 row 1 with this build | No ULog dropouts at ~50 kB/s over 4+ MB; Core 1 busy within a few µs of 0.2 row 3 | | [ ] |
| 2 | Same log: 6.10 rows 2 and 4 | `imu_raw` indices without gaps, `roundtrip_errors` 0; `elle-replay LOG` prints `OK` (exact) | | [ ] |
| 3 | Same log, `elle-replay LOG --compare` | Runs; reference coverage printed (on the bench mostly "level", nothing scored) | | [ ] |
| 4 | Before the flight: Part 7.2 (vehicle test) with this build | See 7.2 | | [ ] |

### 0.5 First flight after this session

Only after 0.1–0.4 pass. If 0.4 passed, fly the `imu-raw-log` build (it flies the same
code, only logs more); otherwise the normal build. Turn compensation stays off: this
flight is the data for Part 7. Manual take-off, then Stabilized with
Manual ready. Collect, in this order, stopping at anything unusual:

1. Straight legs both ways, 20 s each.
2. Steady turns at ~15°, ~30°, ~45°, each direction, 20 s each.
3. 80 m circles around home, clockwise then anticlockwise (6.9 rows 4–5).

Then the flight rows of TODO's pending list: the 200 Hz loop's autotune pass, and the
autotune checks. That log also feeds the attitude-filter comparison (`elle-replay
--compare`, and the turn-correction work).

---

## Part 1: Axis Verification (props off, Manual mode)

**Firmware: RPC+RC** (`--features rpc-control,rpc-rc,gnss`) + TUI for monitoring.
**Critical: verify all axes move the correct direction before any other test.**

### 1.1 Elevon Direction (Manual mode, armed)

Hold the aircraft from behind, looking forward along the fuselage.

| # | Stick input         | Left elevon           | Right elevon | Pass |
|---|---------------------|-----------------------|--------------|------|
| 1 | Pitch stick forward | Down                  | Down         | [x]  |
| 2 | Pitch stick back    | Up                    | Up           | [x]  |
| 3 | Roll stick left     | Up                    | Down         | [x]  |
| 4 | Roll stick right    | Down                  | Up           | [x]  | 
| 5 | Sticks centered     | Both at trim (center) |              | [x]  | 

If any row is wrong: adjust `PITCH_INVERT` or `ROLL_INVERT` in `elle-config/src/lib.rs`, or swap `elevon_left`/
`elevon_right` pins.

### 1.2 Differential Thrust Direction (Manual mode, armed)

| # | Stick input      | Left engine | Right engine | Expected yaw | Pass |
|---|------------------|-------------|--------------|--------------|------|
| 1 | Yaw stick left   | Slower      | Faster       | Turn left    | [x]  |
| 2 | Yaw stick right  | Faster      | Slower       | Turn right   | [x]  |
| 3 | Yaw stick center | Equal       | Equal        | Straight     | [x]  |

If reversed: flip `YAW_INVERT` in `elle-config/src/lib.rs`.

### 1.3 Stabilized PID Direction (Stabilized mode, armed)

Hold board in hand. Verify PID corrects **against** the tilt, not with it.

| # | Action               | Expected elevon response                    | Pass |
|---|----------------------|---------------------------------------------|------|
| 1 | Tilt nose up         | Elevons push nose down (both down)          | [x]  |
| 2 | Tilt nose down       | Elevons push nose up (both up)              | [x]  |
| 3 | Tilt roll left       | Elevons correct right (left down, right up) | [x]  |
| 4 | Tilt roll right      | Elevons correct left (left up, right down)  | [x]  |
| 5 | Hold steady tilt 15° | Sustained correction, not oscillating       | [x]  |
| 6 | Quick pitch rotation | D-term damps the motion (opposes rate)      | [x]  |
| 7 | Quick roll rotation  | D-term damps the motion (opposes rate)      | [x]  |

If P-term is inverted (corrects wrong way at steady angle): the attitude sign for that axis is
wrong. Roll and roll rate are already negated for this PCB in `AttitudePipeline::fuse`
(`crates/elle-control/src/attitude.rs`); fix the sign there, for angle **and** rate.
`PITCH_INVERT`/`ROLL_INVERT` only flip the sticks (`elle-control/src/commands.rs`) and
cannot fix a PID sign.
If D-term is inverted (accelerates rotation): the rate sign is wrong at the same place.

### 1.4 Arming Gesture, Kill Switch + Beep

| # | Action                                   | Expected                                           | Pass |
|---|------------------------------------------|----------------------------------------------------|------|
| 1 | Power on with throttle already low       | Stays disarmed, no beep                            | [ ]  |
| 2 | Throttle to ~50 %                        | Still disarmed, motors stay off                    | [ ]  |
| 3 | Throttle back to zero                    | Arms: single beep, event 10                        | [ ]  |
| 4 | Throttle up                              | Motors spin normally                               | [ ]  |
| 5 | CH8 high (kill)                          | Motors stop, elevons centre, two beeps, event 16   | [ ]  |
| 6 | CH8 still high, throttle gesture         | Stays disarmed (kill blocks everything)            | [ ]  |
| 7 | CH8 low (event 17), throttle at zero     | Stays disarmed: the old gesture was cleared        | [ ]  |
| 8 | Throttle up, then zero                   | Re-arms, single beep                               | [ ]  |

---

## Part 2: Mode Tests (props off, RPC+RC)

### 2.1 Mode Switch Mapping (CH6 3-position)

| # | CH6 position             | Expected mode | Pass |
|---|--------------------------|---------------|------|
| 1 | Position 1 (low, ~306)   | Manual        | [x]  |
| 2 | Position 2 (mid, ~1000)  | Stabilized    | [x]  |
| 3 | Position 3 (high, ~1694) | AltitudeHold  | [x]  |

### 2.2 Stabilized Mode — Stick Response

Hold board in hand, armed.

| # | Stick input                           | Expected servo response                         | Pass |
|---|---------------------------------------|-------------------------------------------------|------|
| 1 | CH6 mid (Stabilized), sticks centered | Elevons hold trim, PID corrects for hand tilt   | [x]  |
| 2 | Full pitch stick forward              | Elevons deflect to nose-down attitude (~25°)    | [x]  |
| 3 | Full pitch stick back                 | Elevons deflect to nose-up (~25°)               | [x]  |
| 4 | Full roll stick left                  | Elevons split for left roll (~45°)              | [x]  |
| 5 | Full roll stick right                 | Elevons split for right roll (~45°)             | [x]  |
| 6 | Release sticks (center)               | Elevons return to level hold (0°/0°)            | [x]  |
| 7 | Throttle stick                        | Throttle responds directly (no PID on throttle) | [x]  |
| 8 | Yaw stick                             | Differential thrust responds directly           | [x]  |

### 2.3 AltitudeHold Mode — Wings Level

| # | Stick input               | Expected                                            | Pass |
|---|---------------------------|-----------------------------------------------------|------|
| 1 | CH6 high, sticks centered | Elevons hold level (0°/0°)                          | [x]  |
| 2 | Full pitch/roll stick     | Elevons do NOT follow stick (setpoint locked 0°/0°) | [x]  |
| 3 | Tilt board                | PID corrects to level                               | [x]  |
| 4 | Throttle/yaw sticks       | Respond normally (manual)                           | [x]  |

### 2.4 Manual Mode — Direct Pass-Through

| # | Stick input              | Expected                              | Pass |
|---|--------------------------|---------------------------------------|------|
| 1 | CH6 low, sticks centered | Elevons at trim, no PID               | [x]  |
| 2 | Move pitch stick         | Elevons follow stick directly via LUT | [x]  |
| 3 | Tilt board               | No servo correction (PID off)         | [x]  |

### 2.5 Manual Escape Under Load

| # | Scenario                                          | Expected                                           | Pass |
|---|---------------------------------------------------|----------------------------------------------------|------|
| 1 | Stabilized, board tilted 30°, PID fighting        | -                                                  | [x]  |
| 2 | Flip CH6 to Manual                                | Instant: elevons snap to stick position, PID stops | [x]  |
| 3 | Stabilized, TUI: `autotune pitch`, relay running  | -                                                  | [x]  |
| 4 | TUI: `autotune abort`, then flip CH6 to Manual    | Autotune aborts, PID off, full manual control      | [x]  |

---

## Part 3: SD Card + ULog

### 3.1 Auto-Start Recording

**Firmware: RPC+RC** — ULog must be started manually via TUI `ulog start`.
ULog auto-starts only in flight firmware.

| # | Action                    | Expected                                            | Pass |
|---|---------------------------|-----------------------------------------------------|------|
| 1 | Power on with SD inserted | TUI: REC is dark gray (not recording yet)           | [x]  |
| 2 | TUI: `ulog start`         | TUI: REC blinks red                                 | [x]  |
| 3 | Power off, pull SD card   | LOG_0000.ulg exists with data                       | [x]  |
| 4 | Power on again            | Next file is LOG_0001.ulg (not overwritten)         | [x]  |

### 3.2 ULog Data Validation

| # | Check                    | Expected                             | Pass |
|---|--------------------------|--------------------------------------|------|
| 1 | `ulog_info LOG_NNNN.ULG` | All message types present, no errors | [x]  |
| 2 | attitude_data rate       | ~200 Hz                              | [ ]  |
| 3 | File has real timestamp  | Correct date/time (not 1970/1980)    | [x]  |

### 3.3 Autotune ULog + Performance (RPC+RC mode)

| # | Steps                              | Expected                                                 | Pass |
|---|------------------------------------|----------------------------------------------------------|------|
| 1 | CH6 mid (Stabilized), arm (gesture) | Armed in Stabilized (TUI shows ARMED + Stabilized)      | [x]  |
| 2 | TUI: `autotune pitch`              | Autotune starts, TUI header shows `AT PITCH`             | [x]  |
| 3 | Let it run for ~10s                | No "attitude data lost" abort event                      | [x]  |
| 4 | TUI: `autotune abort`              | Autotune aborts normally, `AT PITCH` disappears          | [ ]  |
| 5 | Pull SD, check ULog                | `autotune_status` message present with phase transitions | [x]  |
| 6 | Check `system_status.loop_time_us` | All values < 5000µs (5ms budget)                         | [ ]  |
| 7 | Check attitude_data rate           | Still ~200 Hz during autotune (no drops)                 | [ ]  |

### 3.4 Autotune Oscillation Verification (RPC+RC mode, props off)

**Critical: verify autotune relay direction is correct before attempting in flight.**
Use `--features rpc-control,rpc-rc,gnss` so TUI monitors while RC controls.

**Note:** Autotune cannot produce real oscillation on the ground — there is no airflow,
so elevons produce no aerodynamic force. The board will not move on its own.
You must **manually tilt the board to simulate the aircraft's response**:

- When the setpoint is positive, tilt the board in the positive direction (nose up / roll right)
- When the setpoint flips negative, tilt the other way
- You are simulating what aerodynamic forces would do in flight
- Tilt just enough to cross zero — precision doesn't matter

The computed gains will be meaningless (hand period ≠ flight period) and will be
overwritten by in-flight autotune. The goal is only to verify the relay direction
and sign conventions are correct (setpoint flips when measurement crosses zero,
elevons push the right way, cycle counter increments, run completes).

#### 3.4.1 Pitch Autotune — Oscillation Forms

Hold board level in hand. Switch to Stabilized (CH6 mid), arm (gesture).
**Start autotune via TUI command** (CH7 switch does NOT work in RPC+RC mode).

| # | Action                      | Expected                                              | Pass |
|---|-----------------------------|-------------------------------------------------------|------|
| 1 | TUI: `autotune pitch`       | TUI header: `AT PITCH`, radio: `AT P`                 | [x]  |
| 2 | Wait 2s (settling)          | Setpoint held at 0°, elevons hold trim                | [x]  |
| 3 | Relay starts                | Pitch setpoint alternates ±5° (visible in TUI)        | [x]  |
| 4 | Observe elevons             | Elevons oscillate in pitch (both up/both down rhythm) | [x]  |
| 5 | Observe cycle count         | Cycle counter increments in TUI autotune log          | [x]  |
| 6 | Wait ~10-15s for completion | `DONE` on radio, gains printed in TUI                 | [x]  |

**If it aborts with ERR! after ~10s:** oscillation never formed — sign mismatch between
autotuner measurement and PID. Both must use the **raw** attitude: the autotuner step in
`crates/elle-app/src/flight.rs` / `rpc.rs` must not apply `PITCH_INVERT` (that constant is
for sticks only).

**If it ends with event 94 (rejected):** the run completed but the result failed
validation (amplitude < max(0.5°, 20 % of relay), period outside 0.1–5 s, or gains out of
range). Original gains are restored; nothing is saved.

**If oscillation grows until 20° safety abort:** TEST_KP (1.0) × scale (5.0) is too aggressive.
Retry with `autotune pitch 3` (3° relay amplitude).

**If `UnstablePeriod` abort:** oscillation period varies >30% between cycles. Reduce hand
shake — rest board on a surface with pitch axis free to rotate.

#### 3.4.2 Roll Autotune — Oscillation Forms

| # | Action                      | Expected                                                   | Pass |
|---|-----------------------------|------------------------------------------------------------|------|
| 1 | TUI: `autotune roll`        | TUI header: `AT ROLL`, radio: `AT R`                       | [x]  |
| 2 | Wait 2s (settling)          | Setpoint held at 0°                                        | [x]  |
| 3 | Relay starts                | Roll setpoint alternates ±5°                               | [x]  |
| 4 | Observe elevons             | Elevons oscillate in split (left up/right down, then swap) | [x]  |
| 5 | Wait ~10-15s for completion | `DONE` on radio, gains printed in TUI                      | [x]  |

#### 3.4.3 Gains Survive Reboot (eagle only)

| # | Action                      | Expected                                                    | Pass |
|---|-----------------------------|-------------------------------------------------------------|------|
| 1 | Note gains from 3.4.1/3.4.2 | Record Kp/Ki/Kd for pitch and roll                          | [ ]  |
| 2 | Still armed                 | No flash write yet (no event 100)                           | [ ]  |
| 3 | Disarm                      | Event 100 "PID: saved to flash"                             | [ ]  |
| 4 | Power cycle the board       | —                                                           | [ ]  |
| 5 | Check TUI log + ULog         | Event 102 "PID: loaded from flash"; `pid_gains` matches     | [ ]  |

#### 3.4.4 Safety Abort — Amplitude Limit

| # | Action                        | Expected                                    | Pass |
|---|-------------------------------|---------------------------------------------|------|
| 1 | TUI: `autotune pitch`         | Autotune starts                             | [ ]  |
| 2 | During relay, tilt board >20° | Immediate abort, `ERR!` on radio            | [ ]  |
| 3 | Elevons return to normal      | Stabilized mode resumes with original gains | [ ]  |
| 4 | `autotune roll`, tilt **pitch** >20° during settling (first 2 s) | Abort, event 93 | [ ]  |

#### 3.4.4b Safety Abort — Loss of Control Authority

Each row starts from a fresh `autotune pitch` during the relay; each must end with event
93, `ERR!` on the radio, and the original gains in `pid_gains`.

| # | Action                               | Expected                                          | Pass |
|---|--------------------------------------|---------------------------------------------------|------|
| 1 | CH6 to Manual                        | Abort; back to Stabilized does **not** resume it  | [ ]  |
| 2 | CH8 kill                             | Abort                                             | [ ]  |
| 3 | TX off (RC failsafe)                 | Abort                                             | [ ]  |
| 4 | TUI: `autotune pitch 5 20`           | Refused: "autotune cycles must be 1-14"           | [ ]  |

#### 3.4.5 Safety Abort — TUI Command

| # | Action                             | Expected                                                  | Pass |
|---|------------------------------------|-----------------------------------------------------------|------|
| 1 | TUI: `autotune pitch`              | Autotune starts, `AT PITCH` in TUI, `AT P` on radio       | [ ]  |
| 2 | TUI: `autotune abort` during relay | Immediate abort, `ERR!` on radio, original gains restored | [ ]  |

#### 3.4.6 Safety Abort — RC Switch (flight firmware only)

**Firmware: flight mode** (`--features gnss`). This test verifies the CH7 switch abort
path which is only available in flight firmware, not RPC+RC.

| # | Action                     | Expected                                                  | Pass |
|---|----------------------------|-----------------------------------------------------------|------|
| 1 | CH7 mid (pitch autotune)   | Autotune starts, `AT P` on radio                          | [ ]  |
| 2 | CH7 low (off) during relay | Immediate abort, `ERR!` on radio, original gains restored | [ ]  |
| 3 | CH7 mid, then straight to high during relay | Abort (event 92); roll does not start until CH7 goes off and back | [ ]  |

---

## Part 4: RPC Mode Tests (probe connected)

Flash `--no-default-features --features rpc-control,gnss`, launch TUI.

### 4.1 TUI Command Acceptance

| # | Command             | Expected                                        | Pass |
|---|---------------------|-------------------------------------------------|------|
| 1 | `mode stab`         | "Mode: Stabilized"                              | [ ]  |
| 2 | `mode stabilized`   | "Mode: Stabilized"                              | [ ]  |
| 3 | `mode althold`      | "Mode: AltitudeHold"                            | [ ]  |
| 4 | `mode altitudehold` | "Mode: AltitudeHold"                            | [ ]  |
| 5 | `mode manual`       | "Mode: Manual"                                  | [ ]  |
| 6 | `mode mixed`        | Error: "Mode must be: manual, stab, or althold" | [ ]  |

### 4.2 RPC Stabilized Mode

| # | Command sequence          | Expected in controller output panel       | Pass |
|---|---------------------------|-------------------------------------------|------|
| 1 | `mode stab`, `arm`        | Attitude controller active                | [ ]  |
| 2 | `elevon 0 0`              | Setpoint near 0/0, PID corrections small  | [ ]  |
| 3 | `elevon 50 0`             | Pitch setpoint ~12.5° (50% of 25°)        | [ ]  |
| 4 | `elevon 0 50`             | Roll setpoint ~22.5° (50% of 45°)         | [ ]  |
| 5 | `elevon 0 0` → tilt board | PID corrections respond to attitude error | [ ]  |

---

## Part 5: ULog Post-Validation

After completing Parts 1-4, pull the SD card and validate the ULog file(s).
Each test leaves a signature in the data that can be checked after the fact.

**Tools**: `ulog_info`, pyulog, or PlotJuggler.

### 5.1 Message Presence

| # | Check                                | Expected                         | Pass |
|---|--------------------------------------|----------------------------------|------|
| 1 | `attitude_data` messages present     | Yes, ~200 Hz rate                | [ ]  |
| 2 | `commands` messages present          | Yes, ~100 Hz rate                | [ ]  |
| 3 | `controller` messages present        | Yes, ~200 Hz rate                | [ ]  |
| 4 | `engine_data` messages present       | Yes, ~100 Hz rate                | [ ]  |
| 5 | `system_status` messages present     | Yes, ~8 Hz rate                  | [ ]  |
| 6 | `barometer_data` messages present    | Yes, ~5 Hz rate                  | [ ]  |
| 7 | `magnetometer_data` messages present | Yes, ~10 Hz rate                 | [ ]  |
| 8 | `gnss_data` messages present         | Yes, ~5 Hz (one per solution)    | [ ]  |
| 8b | `nav` messages present (GNSS builds) | Yes, ~25 Hz                     | [ ]  |
| 9 | `pid_gains` messages present         | Once at file start, then on change | [ ]  |
| 10 | `log_event` messages present        | Yes (arm/disarm/kill events)     | [ ]  |
| 11 | `autotune_status` messages present  | Yes (if autotune was run in 3.4) | [ ]  |

### 5.2 Axis Verification (validates Part 1)

Plot `commands.pitch` vs `commands.elevon_left_us` and `commands.elevon_right_us`:

| # | Check                                    | Expected                               | Pass |
|---|------------------------------------------|----------------------------------------|------|
| 1 | Pitch stick forward (commands.pitch < 0) | Both elevon_us decrease (deflect down) | [x]  |
| 2 | Pitch stick back (commands.pitch > 0)    | Both elevon_us increase (deflect up)   | [x]  |
| 3 | Roll stick right (commands.roll > 0)     | Left elevon down, right elevon up      | [x]  |
| 4 | Yaw stick left → engine_data             | left_erpm < right_erpm                 | [ ]  |

### 5.3 PID Direction (validates Part 1.3)

Plot `attitude_data.pitch` vs `commands.pitch_correction` during Stabilized mode
(`commands.attitude_mode == 1`):

Note: positive pitch_correction = elevons UP = nose-down push (correct). The PID
works on raw attitude; `PITCH_INVERT` only flips the stick. `controller` has the same
correction split into P/I/D terms.

| # | Check                            | Expected                                             | Pass |
|---|----------------------------------|------------------------------------------------------|------|
| 1 | Positive pitch (nose up tilt)    | Positive pitch_correction (elevons up = nose down)   | [x]  |
| 2 | Positive roll (right tilt)       | Correction opposes tilt direction                    | [x]  |
| 3 | Corrections track tilt magnitude | Larger tilt = larger correction                      | [x]  |

### 5.4 Kill Switch (validates Part 1.4)

Find `log_event` with kill switch event codes in the timeline:

| # | Check                   | Expected                            | Pass |
|---|-------------------------|-------------------------------------|------|
| 1 | At kill event timestamp | system_status.armed transitions 1→0 | [x]  |
| 2 | After kill event        | engine_data left/right_throttle = 0 | [x]  |
| 3 | After kill event        | elevon_us returns to center (~1500) | [x]  |

### 5.5 Mode Transitions (validates Part 2)

| # | Check                                                  | Expected                            | Pass |
|---|--------------------------------------------------------|-------------------------------------|------|
| 1 | commands.attitude_mode changes 0→1→2→0                 | Mode transitions visible in data    | [x]  |
| 2 | In mode 0 (Manual): pitch_correction = 0               | PID inactive                        | [x]  |
| 3 | In mode 1 (Stabilized): pitch_correction ≠ 0 with tilt | PID active                          | [x]  |
| 4 | In mode 2 (AltHold): setpoint locked 0°/0°             | pitch/roll_setpoint_deg stay near 0 | [ ]  |

### 5.6 Autotune (validates Part 3.4)

Plot `autotune_status` fields:

| # | Check                                            | Expected                    | Pass |
|---|--------------------------------------------------|-----------------------------|------|
| 1 | phase transitions: 1→2→3                         | Settling → Relay → Complete | [x]  |
| 2 | setpoint_deg alternates ±5° during phase 2       | Clean relay switching       | [x]  |
| 3 | measurement_deg crosses zero between relay flips | Oscillation is forming      | [x]  |
| 4 | cycles_done increments to 8                      | 2 discard + 6 measured      | [x]  |
| 5 | amplitude_deg stays < 20°                        | Within safety limit         | [x]  |

### 5.7 Performance

| # | Check                                      | Expected                               | Pass |
|---|--------------------------------------------|----------------------------------------|------|
| 1 | system_status.loop_time_us: max            | < 5000 µs (5ms budget)                 | [ ]  |
| 2 | system_status.loop_time_us: average        | < 5000 µs (comfortable margin)         | [ ]  |
| 3 | controller.dt_us                           | ~12000, no gaps > 24000                | [ ]  |
| 4 | controller.att_age_us                      | < 2000 (fresh attitude every tick)     | [ ]  |
| 5 | During autotune: loop_time_us not elevated | < 12000 µs (no performance regression) | [ ]  |

---

## Part 6: Safety and Recent Changes (props off)

### 6.1 RC Failsafe (flight or RPC+RC)

| # | Action                               | Expected                                                        | Pass |
|---|--------------------------------------|-----------------------------------------------------------------|------|
| 1 | Arm (gesture), throttle ~20 %        | Motors spin                                                     | [ ]  |
| 2 | Switch the transmitter off           | Event 13 then 14 within ~300 ms; motors stop, elevons centre, LED rapid orange | [ ]  |
| 3 | Transmitter back on                  | Event 15; stays disarmed                                        | [ ]  |
| 4 | Throttle at zero                     | Stays disarmed (the gesture was cleared)                        | [ ]  |
| 5 | Arm (gesture)                        | Arms normally                                                   | [ ]  |

### 6.2 Host-Link Failsafe (pure RPC, `rpc-control,gnss`)

| # | Action                                            | Expected                                                   | Pass |
|---|---------------------------------------------------|------------------------------------------------------------|------|
| 1 | TUI: `throttle 20`, then `arm`                    | Refused: event 18 (throttle not at zero)                   | [ ]  |
| 2 | TUI: `throttle 0`, `arm`, `throttle 20`           | Arms, motors spin                                          | [ ]  |
| 3 | Kill the TUI process (or unplug the probe)        | Within ~300 ms: disarm, motors stop                        | [ ]  |
| 4 | Restart the TUI                                   | Disarmed, throttle reads 0 (not the old 20 %)              | [ ]  |
| 5 | `direct throttle 0`, `direct arm`                 | `direct arm` stays running, pinging, motors armed          | [ ]  |
| 6 | Ctrl-C the `direct` process                       | Sends throttle 0 + disarm, exits; aircraft disarmed        | [ ]  |

### 6.3 No Flash Writes While Armed

| # | Action (RPC mode, armed)        | Expected                                              | Pass |
|---|---------------------------------|-------------------------------------------------------|------|
| 1 | `savepid`                       | Refused, event 63                                     | [ ]  |
| 2 | `mag cal start`                 | Refused, event 63                                     | [ ]  |
| 3 | `level cal start`               | Refused (event 152)                                   | [ ]  |
| 4 | Disarm, `savepid`               | Event 100; control loop keeps running (no watchdog reset) | [ ]  |

Each flash setting is cleared on its own (eagle; on the dart skip rows 2–3, `clearpid`
does nothing there):

| # | Action (RPC mode, disarmed)                         | Expected                                              | Pass |
|---|-----------------------------------------------------|-------------------------------------------------------|------|
| 1 | Mag cal and level cal done, `savepid`               | Events 113, 154, 100                                  | [ ]  |
| 2 | `clearpid`, power cycle                             | Event 103 (no PID) **and** 115 + 157 (both cals load) | [ ]  |
| 3 | `savepid`, `mag cal clear`, power cycle             | Events 102 + 157, and 116 (mag cal gone)              | [ ]  |
| 4 | `level cal clear`, power cycle                      | Event 158 (level cal gone), 102 still                 | [ ]  |

### 6.4 Gyro Bias at Boot

| # | Action                                   | Expected                                                   | Pass |
|---|------------------------------------------|------------------------------------------------------------|------|
| 1 | Power on, aircraft still                 | Event 46 within ~1 s; LED solid green (purple in RPC); TUI "Cal" | [ ]  |
| 2 | Power on while moving it for > 10 s      | Event 47; LED keeps pulsing; TUI not "Cal"                 | [ ]  |

### 6.5 Heading Hold (CH5, Stabilized)

| # | Action                                   | Expected                                                     | Pass |
|---|------------------------------------------|--------------------------------------------------------------|------|
| 1 | Stabilized, CH5 on                       | Event 130 after ~0.5 s; LED pulses blue when armed           | [ ]  |
| 2 | Yaw the aircraft by hand ~30°            | Elevons command a bank back toward the captured heading (≤ 25°) | [ ]  |
| 3 | Switch CH6 to Manual                     | Event 131, direct control                                    | [ ]  |
| 4 | Back to Stabilized with CH5 still on     | Event 130 again, new heading captured                        | [ ]  |

### 6.6 I2C Fault

| # | Action                                           | Expected                                               | Pass |
|---|--------------------------------------------------|--------------------------------------------------------|------|
| 1 | Disturb the I2C0 bus (short SDA briefly) on the bench | Event 48; mag and baro stop updating in the TUI     | [ ]  |
| 2 | Watch attitude                                   | IMU keeps running at 1 kHz (6-DOF), no Core 1 restart  | [ ]  |

### 6.7 Elevon Latency (scope)

| # | Action                                          | Expected                                        | Pass |
|---|-------------------------------------------------|-------------------------------------------------|------|
| 1 | Scope PIN_12 and PIN_13, Manual mode, stick step | Frames every 5 ms (200 Hz); new pulse width within one frame of the step (compare with `commands` in ULog) | [ ]  |
| 2 | TUI log during engines-on bench run              | Event 45 rare (catch-up drains), no event 41    | [ ]  |

### 6.8 DShot Timing and ESC Link (props off, eagle first)

| # | Action | Expected | Pass |
|---|--------|----------|------|
| 1 | Scope PIN_14 and PIN_11 at idle, disarmed | One complete MotorStop frame + ESC reply per ms; no frame cut short | [ ]  |
| 2 | Same, across a thread-executor stall (boot, SD init) | Frames keep their 1 ms cadence through it (DShot's own executor); never back-to-back or cut-off frames | [ ]  |
| 3 | Disarmed, check ULog | `engine_data` voltage/temperature update at idle; `esc_health` replies ≈ 1000/s per ESC, `bad_frames` 0 | [ ]  |
| 4 | FC on USB first, then plug the battery | Events 162 and 163 about 1 s after the ESCs finish their tones; EDT present; correct spin direction on first spool-up | [ ]  |
| 5 | Briefly cut one ESC's power at idle | 160 (or 161) for that side only, then 162 (or 163) once it is back | [ ]  |
| 6 | Idle soak 15–30 min | No twitches or odd beeps; `bad_frames` stays 0 (a steady rate points at wiring) | [ ]  |

---

### 6.9 Navigation Observation (outdoors, GNSS fix)

The navigator only logs; nothing it computes reaches the elevons. Read the results
with `elle_log.py nav` and `elle_log.py sensors`.

| # | Test | Expected | Pass |
|---|------|----------|------|
| 1 | Power up outdoors, wait for a 3D fix, disarmed | Radio FM text goes `NOHOME` → `WAIT H`; `nav.status` has home (bit 0) and position (bit 2); `fix_age_ms` < 250 | [ ] |
| 2 | Arm, carry the aircraft ~50 m, disarm | Home locked (bit 1) while armed; `home_dist_m` grows to ~50 m and bearing points back; after disarm home follows the aircraft again | [ ] |
| 3 | Cover the antenna (or unplug GNSS) for 5 s while armed | Position drops (bit 2 clear) ~1 s after the last fix; extrapolated (bit 5) only for the first 400 ms; `bank_demand_deg` NaN | [ ] |
| 4 | Flight in Stabilized: circle the field clockwise at ~80 m, then anticlockwise | Clockwise: `bank_demand_deg` and `roll_deg` both positive and close; anticlockwise: they disagree in sign (demand still asks for a right turn). Confirms the sign conventions | [ ] |
| 5 | Same flight | `gnss_data` ~5/s with `pvt_active` = 1; baro and GNSS height above home within a few metres | [ ] |

### 6.10 Raw IMU Capture (bench, `imu-raw-log` build)

Build with `--features imu-raw-log` (eagle flight build first). Nothing flies
differently; the build records more and logs `attitude_data` at 50 Hz.

| # | Test | Expected | Pass |
|---|------|----------|------|
| 1 | Power up, arm, run the engines at idle and a few throttle steps for 10+ min (props off) | No ULog dropouts (`elle_log.py timing`) over ~4 MB or more | [ ] |
| 2 | Same log | `imu_raw` at ~100/s, first indices consecutive (step 10, no gaps); `imu_raw_ctx` ~1/s, `roundtrip_errors` 0 | [ ] |
| 3 | Same log | `core1_load` busy mean/max within a few µs of a normal build's | [ ] |
| 4 | Same log, `elle-replay FILE` | Every `attitude_data` sample while synced matches the replay exactly | [ ] |
| 5 | Stabilized on the stand, stick steps and disturbances | Feels and responds as before (the fusion code moved, bit-identical on the host) | [ ] |

## Part 7: Attitude Estimation and Turn Compensation

Background, settings and the simulated numbers: [`docs/ATTITUDE.md`](docs/ATTITUDE.md).
The firmware ships with `AHRS_TURN_COMP = Off` and no accel gate; nothing in 7.1–7.4
changes how the aircraft flies. 7.5 onwards only after 7.4 picks a mode.

```sh
R="cargo run -q --release -p elle-replay --target x86_64-unknown-linux-gnu --"
```

### 7.1 Host checks (every PR, CI)

| # | Check | Expected | Pass |
|---|-------|----------|------|
| 1 | `cargo test -p elle-control --target x86_64-unknown-linux-gnu` | Attitude pipeline, turn compensation, raw capture: all pass (includes exact replay per mode and across a gap, and Off = plain Madgwick bit for bit) | [ ] |
| 2 | `cargo test -p elle-replay --target x86_64-unknown-linux-gnu` | Simulator, reference, scoring, vehicle test: all pass | [ ] |
| 3 | `$R --simulate /tmp/s.ulg --compare --wind-east 6 --vibration 2 --gyro-bias-dps 0.03` | `OK` (exact); reference ≤ 0.2° off the truth; `madgwick-ce` ≈ 1.6° roll RMS, firmware ≈ 5.6° | [ ] |

### 7.2 Vehicle ground test (real sensors, nothing flown)

The aircraft strapped **level** in a car (nose forward, props off, engines disarmed),
`imu-raw-log` build, compensation off. The car turns without banking, so the truth is
level while the accel feels the sideways acceleration. GNSS fix outdoors first.

| # | Test | Expected | Log | Pass |
|---|------|----------|-----|------|
| 1 | Park 60 s, then drive straight 20 s at ≥ 25 km/h (7 m/s) | Straight stretches for the reference to anchor on | | [ ] |
| 2 | Two or three laps of a roundabout each way, ≥ 25 km/h, straight 10 s between | | | [ ] |
| 3 | `$R LOG --compare --score-by-rate` | `OK` (exact); reference covers the curves | | [ ] |
| 4 | Same | `firmware` leans in curves (several degrees RMS, sign opposite per direction); `madgwick-cc` and `madgwick-ce` within ~1–2° of level. If a compensated variant is **worse** than the firmware, the forward axis or a sign is wrong: stop and investigate | | [ ] |
| 5 | Same, `--csv /tmp/car.csv` in PlotJuggler | Roll of each variant through the curves; reference flat | | [ ] |

### 7.3 Data flights (compensation off, `imu-raw-log` build)

Part 0.5's flight, then one more on a windier day. Straight legs of ≥ 5 s between every
manoeuvre (the reference anchors there): 20 s straight both ways; steady turns at ~15°,
~30°, ~45° each way, 20 s each; 80 m circles both ways (also TEST_PLAN 6.9).

| # | Check | Expected | Log | Pass |
|---|-------|----------|-----|------|
| 1 | `$R LOG --compare` | `OK`; no gaps (or few), `roundtrip_errors` 0 | | [ ] |
| 2 | Reference coverage | ≥ 50 % of the turn time scored | | [ ] |
| 3 | Firmware row | Record its roll/pitch RMS in turns and the per-direction mean: this is the size of the problem on the real aircraft (simulated: ~5–7°) | | [ ] |

### 7.4 Decision

Criteria in [`docs/ATTITUDE.md`](docs/ATTITUDE.md#from-data-to-the-aircraft): the mode
with the lowest roll RMS in turns, if on **every** data flight it halves the firmware's
roll RMS (and by ≥ 2°) in both directions and does not worsen pitch RMS by > 0.5°.
Record the table from each flight in the PR that enables it. Otherwise stay `Off`.

### 7.5 Enable: bench regression (props off)

Set `AHRS_TURN_COMP` to the chosen mode, build flight + `imu-raw-log`.

| # | Test | Expected | Log | Pass |
|---|------|----------|-----|------|
| 1 | Part 0.1 rows 1–4 | Unchanged (below 6 m/s there is no compensation) | | [ ] |
| 2 | 10 min armed idle outdoors with a GNSS fix | `core1_load` busy mean/max within ~10 µs of an `Off` build | | [ ] |
| 3 | Same log, `$R LOG` | `OK` (exact) with `imu_raw_fix` records present (~5/s) and `imu_raw_ctx.turn_comp` set | | [ ] |
| 4 | Repeat 7.2 with this build | The **firmware** now stays level in the curves, like the variant did in 7.2 | | [ ] |

### 7.6 Enable: first flight

Stabilized, Manual ready, calm day. Straight, then turns at ~15° and ~30° each way, then
circles.

| # | Check | Expected | Log | Pass |
|---|-------|----------|-----|------|
| 1 | Feel | Turns hold bank without the slow tightening; no oscillation; wings level after roll-out | | [ ] |
| 2 | `$R LOG --compare` | `OK`; the firmware row now matches the chosen variant's row from 7.4 | | [ ] |
| 3 | 6.9 row 4 (circling) | Bank demand vs measured roll agree better than before | | [ ] |

### 7.7 Enable: retune

The PID sees a different (better) attitude in turns: rerun the autotune (both axes) and
compare gains with the previous set. Then a windy-day flight repeating 7.6.

## Part 8: `elle mcp` on the hardware

**Never run on the aircraft yet.** The server is tested against a fake flight controller
on the real RPC client path; these rows check the parts only hardware has: the probe,
flashing, RTT timing and the engine gates. Run it on the eagle, props off, before using
the server for any other Part. Start it from Claude Code with `.mcp.json` (see the
host README).

| # | Test | Expected | Pass |
|---|------|----------|------|
| 1 | `build_and_flash` eagle `rpc` | Builds, flashes, reattaches; `read build`: platform eagle, features include rpc-control and gnss, `git` = `git describe` of the checkout | [ ] |
| 1b | Power the board, start the server, `connect` at once (board still booting) | Connects within the 30 s default; note `connect_timing` (RTT and first reply, s) here: the defaults assume ≤ ~20 s | [ ] |
| 2 | `read` every source; `sample attitude` 60 s at 20 Hz | All answer; no request timeouts; ~1200 samples | [ ] |
| 3 | `reset_target`, then `wait_event` 46 | Link comes back by itself; event 46 within ~2 s of the reset | [ ] |
| 4 | Server **without** `--dangerously-allow-motors`: `arm`, `set_throttle 10` | Both refused by the server; no event 10 | [ ] |
| 5 | Restart with `--dangerously-allow-motors --max-armed-s 15`: `arm` without `props_off_confirmed`, then with it | Refused, then armed (event 10); `set_throttle 50` refused (cap 30); `set_throttle 10` spins | [ ] |
| 6 | Armed, wait 15 s without `extend_armed` | Throttle 0 and disarm (event 11) at 15 s | [ ] |
| 7 | Armed, quit Claude Code (the client closes stdin) | Disarmed on the way out (event 11), not by the firmware's host failsafe | [ ] |
| 8 | Armed, `kill -9` the server process | Firmware host-link failsafe within ~300 ms: engines stop, disarmed | [ ] |
| 9 | `disconnect`, run the TUI, quit it, `connect` | TUI works while the server is disconnected; the server reconnects after | [ ] |
| 10 | `build_and_flash` eagle `flight`, then `wait_log` `115200` after `reset_target` | defmt lines arrive decoded; GNSS baud line found | [ ] |
| 11 | Card in the host's reader: `card_logs`, `copy_logs` | Lists the card's files; copies the newest into `logs/`; a second copy skips it | [ ] |
| 12 | `analyse_log summary` and `replay` on that file (an `imu-raw-log` build for `replay`) | Same output as running the tools by hand | [ ] |
| 13 | `test_record` a row, `test_report` | The row, its note and the build info in `logs/test-runs/<date>.jsonl` | [ ] |

## Abort Criteria

Stop testing and investigate if any of these occur:

- Any axis in 1.1/1.2/1.3 is reversed — fix inversions before proceeding
- Kill switch does not stop motors — do NOT fly
- Manual mode escape does not immediately disable PID
- PID oscillates uncontrollably in Stabilized mode (reduce `PID_SCALE` in `elle-config/src/lib.rs`)
- Any Part 6 failsafe test fails — do NOT fly
- Mode switch has no effect (check CH6 wiring / thresholds)
- SD card not mounting or ULog not auto-starting

## Post-Ground Sign-Off

All tests in Part 1 through Part 6 passed: [ ]
Reviewed by: _______________
Date: _______________
