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
wrong. Roll and roll rate are already negated for this PCB in `Imu::run()`
(`crates/elle-hardware/src/imu/driver.rs`); fix the sign there, for angle **and** rate.
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
| 2 | attitude_data rate       | ~83 Hz                               | [ ]  |
| 3 | File has real timestamp  | Correct date/time (not 1970/1980)    | [x]  |

### 3.3 Autotune ULog + Performance (RPC+RC mode)

| # | Steps                              | Expected                                                 | Pass |
|---|------------------------------------|----------------------------------------------------------|------|
| 1 | CH6 mid (Stabilized), arm (gesture) | Armed in Stabilized (TUI shows ARMED + Stabilized)      | [x]  |
| 2 | TUI: `autotune pitch`              | Autotune starts, TUI header shows `AT PITCH`             | [x]  |
| 3 | Let it run for ~10s                | No "attitude data lost" abort event                      | [x]  |
| 4 | TUI: `autotune abort`              | Autotune aborts normally, `AT PITCH` disappears          | [ ]  |
| 5 | Pull SD, check ULog                | `autotune_status` message present with phase transitions | [x]  |
| 6 | Check `system_status.loop_time_us` | All values < 12000µs (12ms budget)                       | [ ]  |
| 7 | Check attitude_data rate           | Still ~83 Hz during autotune (no drops)                  | [ ]  |

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
| 1 | `attitude_data` messages present     | Yes, ~83 Hz rate                 | [ ]  |
| 2 | `commands` messages present          | Yes, ~83 Hz rate                 | [ ]  |
| 3 | `controller` messages present        | Yes, ~83 Hz rate                 | [ ]  |
| 4 | `engine_data` messages present       | Yes, ~83 Hz rate                 | [ ]  |
| 5 | `system_status` messages present     | Yes, ~8.3 Hz rate                | [ ]  |
| 6 | `barometer_data` messages present    | Yes, ~4.4 Hz rate                | [ ]  |
| 7 | `magnetometer_data` messages present | Yes, ~10 Hz rate                 | [ ]  |
| 8 | `gnss_data` messages present         | Yes, ~1 Hz rate                  | [ ]  |
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
| 1 | system_status.loop_time_us: max            | < 12000 µs (12ms budget)               | [ ]  |
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
| 1 | Scope PIN_12 and PIN_13, Manual mode, stick step | New pulse width within one 20 ms frame of the step (compare with `commands` in ULog) | [ ]  |
| 2 | TUI log during engines-on bench run              | Event 45 rare (catch-up drains), no event 41    | [ ]  |

### 6.8 DShot Timing and ESC Link (props off, eagle first)

| # | Action | Expected | Pass |
|---|--------|----------|------|
| 1 | Scope PIN_14 and PIN_11 at idle, disarmed | One complete MotorStop frame + ESC reply per ms; no frame cut short | [ ]  |
| 2 | Same, across a loop stall (boot, SD init) | After the stall: one frame ≥ 200 µs later, then 1 ms spacing — never back-to-back frames | [ ]  |
| 3 | Disarmed, check ULog | `engine_data` voltage/temperature update at idle; `esc_health` replies ≈ 1000/s per ESC, `bad_frames` 0 | [ ]  |
| 4 | FC on USB first, then plug the battery | Events 162 and 163 about 1 s after the ESCs finish their tones; EDT present; correct spin direction on first spool-up | [ ]  |
| 5 | Briefly cut one ESC's power at idle | 160 (or 161) for that side only, then 162 (or 163) once it is back | [ ]  |
| 6 | Idle soak 15–30 min | No twitches or odd beeps; `bad_frames` stays 0 (a steady rate points at wiring) | [ ]  |

---

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
