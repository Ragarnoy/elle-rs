# Ground Test Plan

Pre-flight verification checklist. Complete ALL tests before flight.

## Prerequisites

- BetaFPV Pro transmitter bound and linked
- CH5 (2-pos switch left): Kill switch (high = disarm)
- CH6 (3-pos switch left): Manual / Stabilized / AltitudeHold
- CH7 (3-pos switch right): Autotune off / pitch / roll
- Props removed for all ground tests
- SD card inserted (FAT32)
- Probe-rs debug probe connected (RPC tests only)

## Build Commands

```sh
# Flight firmware (default — CRSF/ELRS)
cd crates/elle-eagle
cargo run --release --features gnss

# RPC firmware (ground test mode)
cargo run --release --no-default-features --features rpc-control

# RPC + RC firmware (probe monitoring with RC control)
cargo run --release --no-default-features --features rpc-control,rpc-rc

# RPC + RC + GNSS firmware (autotune ground test with full monitoring)
cargo run --release --no-default-features --features rpc-control,rpc-rc,gnss

# Host TUI
cargo run -p elle-rpc-host --target x86_64-unknown-linux-gnu
```

---

## Part 1: Axis Verification (props off, Manual mode)

**Critical: verify all axes move the correct direction before any other test.**
Use default flight firmware or RPC+RC with TUI for monitoring.

### 1.1 Elevon Direction (Manual mode, armed)

Hold the aircraft from behind, looking forward along the fuselage.

| # | Stick input            | Left elevon | Right elevon | Pass |
|---|------------------------|-------------|--------------|------|
| 1 | Pitch stick forward    | Down        | Down         | [ ]  |
| 2 | Pitch stick back       | Up          | Up           | [ ]  |
| 3 | Roll stick left        | Up          | Down         | [ ]  |
| 4 | Roll stick right       | Down        | Up           | [ ]  |
| 5 | Sticks centered        | Both at trim (center)  | | [ ]  |

If any row is wrong: adjust `PITCH_INVERT` or `ROLL_INVERT` in `elle-config/src/lib.rs`, or swap `elevon_left`/`elevon_right` pins.

### 1.2 Differential Thrust Direction (Manual mode, armed)

| # | Stick input       | Left engine | Right engine | Expected yaw | Pass |
|---|-------------------|-------------|--------------|--------------|------|
| 1 | Yaw stick left    | Slower      | Faster       | Turn left    | [ ]  |
| 2 | Yaw stick right   | Faster      | Slower       | Turn right   | [ ]  |
| 3 | Yaw stick center  | Equal       | Equal        | Straight     | [ ]  |

If reversed: flip `YAW_INVERT` in `elle-config/src/lib.rs`.

### 1.3 Stabilized PID Direction (Stabilized mode, armed)

Hold board in hand. Verify PID corrects **against** the tilt, not with it.

| # | Action                   | Expected elevon response             | Pass |
|---|--------------------------|--------------------------------------|------|
| 1 | Tilt nose up             | Elevons push nose down (both down)   | [ ]  |
| 2 | Tilt nose down           | Elevons push nose up (both up)       | [ ]  |
| 3 | Tilt roll left           | Elevons correct right (left down, right up) | [ ]  |
| 4 | Tilt roll right          | Elevons correct left (left up, right down)  | [ ]  |
| 5 | Hold steady tilt 15°     | Sustained correction, not oscillating | [ ]  |
| 6 | Quick pitch rotation     | D-term damps the motion (opposes rate) | [ ]  |
| 7 | Quick roll rotation      | D-term damps the motion (opposes rate) | [ ]  |

If P-term is inverted (corrects wrong way at steady angle): negate the AHRS measurement for that axis in `system.rs`.
If D-term is inverted (accelerates rotation): negate the AHRS rate for that axis in `system.rs`.

### 1.4 Kill Switch + Beep

| # | Action                        | Expected                                    | Pass |
|---|-------------------------------|----------------------------------------------|------|
| 1 | Throttle low (auto-arm)       | Single beep from motors (arm confirmation)   | [ ]  |
| 2 | Throttle up                   | Motors spin normally                         | [ ]  |
| 3 | CH5 high (kill)               | Motors stop immediately, two beeps (disarm)  | [ ]  |
| 4 | CH5 still high, throttle low  | Motors stay off (no re-arm, no beep)         | [ ]  |
| 5 | CH5 low, throttle low         | Re-arms, single beep again                  | [ ]  |
| 6 | Throttle up                   | Motors spin normally                         | [ ]  |

---

## Part 2: Mode Tests (props off)

### 2.1 Mode Switch Mapping (CH6 3-position)

| # | CH6 position             | Expected mode | Pass |
|---|--------------------------|---------------|------|
| 1 | Position 1 (low, ~306)   | Manual        | [ ]  |
| 2 | Position 2 (mid, ~1000)  | Stabilized    | [ ]  |
| 3 | Position 3 (high, ~1694) | AltitudeHold  | [ ]  |

### 2.2 Stabilized Mode — Stick Response

Hold board in hand, armed.

| # | Stick input                           | Expected servo response                           | Pass |
|---|---------------------------------------|---------------------------------------------------|------|
| 1 | CH6 mid (Stabilized), sticks centered | Elevons hold trim, PID corrects for hand tilt     | [ ]  |
| 2 | Full pitch stick forward              | Elevons deflect to nose-down attitude (~25°)      | [ ]  |
| 3 | Full pitch stick back                 | Elevons deflect to nose-up (~25°)                 | [ ]  |
| 4 | Full roll stick left                  | Elevons split for left roll (~45°)                | [ ]  |
| 5 | Full roll stick right                 | Elevons split for right roll (~45°)               | [ ]  |
| 6 | Release sticks (center)               | Elevons return to level hold (0°/0°)              | [ ]  |
| 7 | Throttle stick                        | Throttle responds directly (no PID on throttle)   | [ ]  |
| 8 | Yaw stick                             | Differential thrust responds directly             | [ ]  |

### 2.3 AltitudeHold Mode — Wings Level

| # | Stick input               | Expected                                            | Pass |
|---|---------------------------|-----------------------------------------------------|------|
| 1 | CH6 high, sticks centered | Elevons hold level (0°/0°)                          | [ ]  |
| 2 | Full pitch/roll stick     | Elevons do NOT follow stick (setpoint locked 0°/0°) | [ ]  |
| 3 | Tilt board                | PID corrects to level                               | [ ]  |
| 4 | Throttle/yaw sticks       | Respond normally (manual)                           | [ ]  |

### 2.4 Manual Mode — Direct Pass-Through

| # | Stick input              | Expected                              | Pass |
|---|--------------------------|---------------------------------------|------|
| 1 | CH6 low, sticks centered | Elevons at trim, no PID               | [ ]  |
| 2 | Move pitch stick         | Elevons follow stick directly via LUT | [ ]  |
| 3 | Tilt board               | No servo correction (PID off)         | [ ]  |

### 2.5 Manual Escape Under Load

| # | Scenario                                   | Expected                                           | Pass |
|---|--------------------------------------------|----------------------------------------------------|------|
| 1 | Stabilized, board tilted 30°, PID fighting | -                                                  | [ ]  |
| 2 | Flip CH6 to Manual                         | Instant: elevons snap to stick position, PID stops | [ ]  |
| 3 | Stabilized, autotune running               | -                                                  | [ ]  |
| 4 | Flip CH6 to Manual                         | Autotune pauses, PID off, full manual control      | [ ]  |

---

## Part 3: SD Card + ULog

### 3.1 Auto-Start Recording

| # | Action                    | Expected                                   | Pass |
|---|---------------------------|---------------------------------------------|------|
| 1 | Power on with SD inserted | defmt: "SD: FAT32 mounted"                 | [ ]  |
| 2 | Wait 5s                   | defmt: "ULog recording started (auto)"     | [ ]  |
| 3 | Power off, pull SD card   | LOG_0000.ULG exists with data              | [ ]  |
| 4 | Power on again            | Next file is LOG_0001.ULG (not overwritten) | [ ]  |

### 3.2 ULog Data Validation

| # | Check                       | Expected                              | Pass |
|---|------------------------------|---------------------------------------|------|
| 1 | `ulog_info LOG_NNNN.ULG`    | All message types present, no errors  | [ ]  |
| 2 | attitude_data rate           | ~77 Hz                                | [ ]  |
| 3 | File has real timestamp      | Correct date/time (not 1970/1980)     | [ ]  |

### 3.3 Autotune ULog + Performance (RPC mode)

| # | Steps                      | Expected                                    | Pass |
|---|----------------------------|---------------------------------------------|------|
| 1 | `mode stab`, `arm`         | Armed in Stabilized                         | [ ]  |
| 2 | `autotune pitch`           | Autotune starts                             | [ ]  |
| 3 | Let it run for ~10s        | No "attitude data lost" abort event         | [ ]  |
| 4 | `autotune abort`           | Autotune aborts normally                    | [ ]  |
| 5 | Pull SD, check ULog        | `autotune_status` message present with phase transitions | [ ]  |
| 6 | Check `system_status.loop_time_us` | All values < 13000µs (13ms budget) | [ ]  |
| 7 | Check attitude_data rate   | Still ~77 Hz during autotune (no drops)     | [ ]  |

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

Hold board level in hand. Switch to Stabilized (CH6 mid), let it arm.

| # | Action                       | Expected                                              | Pass |
|---|------------------------------|-------------------------------------------------------|------|
| 1 | TUI: `autotune pitch`        | CRSF radio shows `AT P`                              | [ ]  |
| 2 | Wait 2s (settling)           | Setpoint held at 0°, elevons hold trim                | [ ]  |
| 3 | Relay starts                 | Pitch setpoint alternates ±5° (visible in TUI)        | [ ]  |
| 4 | Observe elevons              | Elevons oscillate in pitch (both up/both down rhythm) | [ ]  |
| 5 | Observe cycle count          | Cycle counter increments in TUI autotune log          | [ ]  |
| 6 | Wait ~10-15s for completion  | `DONE` on radio, gains printed in TUI                 | [ ]  |

**If it aborts with ERR! after ~10s:** oscillation never formed — sign mismatch between
autotuner measurement and PID. Check `PITCH_INVERT` is applied in both the PID measurement
path (`system.rs`) and the autotuner measurement path (`main.rs`).

**If oscillation grows until 20° safety abort:** TEST_KP (1.0) × scale (5.0) is too aggressive.
Retry with `autotune pitch 3` (3° relay amplitude).

**If `UnstablePeriod` abort:** oscillation period varies >30% between cycles. Reduce hand
shake — rest board on a surface with pitch axis free to rotate.

#### 3.4.2 Roll Autotune — Oscillation Forms

| # | Action                       | Expected                                              | Pass |
|---|------------------------------|-------------------------------------------------------|------|
| 1 | TUI: `autotune roll`         | CRSF radio shows `AT R`                              | [ ]  |
| 2 | Wait 2s (settling)           | Setpoint held at 0°                                   | [ ]  |
| 3 | Relay starts                 | Roll setpoint alternates ±5°                          | [ ]  |
| 4 | Observe elevons              | Elevons oscillate in split (left up/right down, then swap) | [ ]  |
| 5 | Wait ~10-15s for completion  | `DONE` on radio, gains printed in TUI                 | [ ]  |

#### 3.4.3 Gains Survive Reboot

| # | Action                       | Expected                                              | Pass |
|---|------------------------------|-------------------------------------------------------|------|
| 1 | Note gains from 3.4.1/3.4.2 | Record Kp/Ki/Kd for pitch and roll                    | [ ]  |
| 2 | Power cycle the board        | —                                                     | [ ]  |
| 3 | Check TUI startup logs       | "PID loaded P(...) R(...) s=... il=..." with matching gains | [ ]  |

#### 3.4.4 Safety Abort — Amplitude Limit

| # | Action                        | Expected                                             | Pass |
|---|-------------------------------|------------------------------------------------------|------|
| 1 | TUI: `autotune pitch`         | Autotune starts                                     | [ ]  |
| 2 | During relay, tilt board >20° | Immediate abort, `ERR!` on radio                    | [ ]  |
| 3 | Elevons return to normal      | Stabilized mode resumes with original gains          | [ ]  |

#### 3.4.5 Safety Abort — RC Switch

| # | Action                        | Expected                                             | Pass |
|---|-------------------------------|------------------------------------------------------|------|
| 1 | CH7 mid (pitch autotune)      | Autotune starts, `AT P` on radio                    | [ ]  |
| 2 | CH7 low (off) during relay    | Immediate abort, `ERR!` on radio, original gains restored | [ ]  |

---

## Part 4: RPC Mode Tests (probe connected)

Flash `--features rpc-control`, launch TUI.

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

---

## Part 5: ULog Post-Validation

After completing Parts 1-4, pull the SD card and validate the ULog file(s).
Each test leaves a signature in the data that can be checked after the fact.

**Tools**: `ulog_info`, pyulog, or PlotJuggler.

### 5.1 Message Presence

| # | Check                              | Expected                                    | Pass |
|---|------------------------------------|---------------------------------------------|------|
| 1 | `attitude_data` messages present   | Yes, ~77 Hz rate                            | [ ]  |
| 2 | `commands` messages present        | Yes, ~77 Hz rate                            | [ ]  |
| 3 | `engine_data` messages present     | Yes, ~77 Hz rate                            | [ ]  |
| 4 | `system_status` messages present   | Yes, ~7.7 Hz rate                           | [ ]  |
| 5 | `barometer_data` messages present  | Yes, ~4 Hz rate                             | [ ]  |
| 6 | `magnetometer_data` messages present | Yes, ~9.6 Hz rate                         | [ ]  |
| 7 | `log_event` messages present       | Yes (arm/disarm/kill events)                | [ ]  |
| 8 | `autotune_status` messages present | Yes (if autotune was run in 3.4)            | [ ]  |

### 5.2 Axis Verification (validates Part 1)

Plot `commands.pitch` vs `commands.elevon_left_us` and `commands.elevon_right_us`:

| # | Check                                                   | Expected                                  | Pass |
|---|---------------------------------------------------------|-------------------------------------------|------|
| 1 | Pitch stick forward (commands.pitch < 0)                | Both elevon_us decrease (deflect down)    | [ ]  |
| 2 | Pitch stick back (commands.pitch > 0)                   | Both elevon_us increase (deflect up)      | [ ]  |
| 3 | Roll stick right (commands.roll > 0)                    | Left elevon down, right elevon up         | [ ]  |
| 4 | Yaw stick left → engine_data                            | left_erpm < right_erpm                    | [ ]  |

### 5.3 PID Direction (validates Part 1.3)

Plot `attitude_data.pitch` vs `commands.pitch_correction` during Stabilized mode
(`commands.attitude_mode == 1`):

| # | Check                                                   | Expected                                  | Pass |
|---|---------------------------------------------------------|-------------------------------------------|------|
| 1 | Positive pitch (nose up tilt)                           | Negative pitch_correction (pushes down)   | [ ]  |
| 2 | Positive roll (right tilt)                              | Negative roll_correction (pushes left)    | [ ]  |
| 3 | Corrections track tilt magnitude                        | Larger tilt = larger correction           | [ ]  |

### 5.4 Kill Switch (validates Part 1.4)

Find `log_event` with kill switch event codes in the timeline:

| # | Check                                                   | Expected                                  | Pass |
|---|---------------------------------------------------------|-------------------------------------------|------|
| 1 | At kill event timestamp                                 | system_status.armed transitions 1→0       | [ ]  |
| 2 | After kill event                                        | engine_data left/right_throttle = 0       | [ ]  |
| 3 | After kill event                                        | elevon_us returns to center (~1500)       | [ ]  |

### 5.5 Mode Transitions (validates Part 2)

| # | Check                                                   | Expected                                  | Pass |
|---|---------------------------------------------------------|-------------------------------------------|------|
| 1 | commands.attitude_mode changes 0→1→2→0                  | Mode transitions visible in data          | [ ]  |
| 2 | In mode 0 (Manual): pitch_correction = 0                | PID inactive                              | [ ]  |
| 3 | In mode 1 (Stabilized): pitch_correction ≠ 0 with tilt | PID active                                | [ ]  |
| 4 | In mode 2 (AltHold): setpoint locked 0°/0°             | pitch/roll_setpoint_deg stay near 0       | [ ]  |

### 5.6 Autotune (validates Part 3.4)

Plot `autotune_status` fields:

| # | Check                                                   | Expected                                  | Pass |
|---|---------------------------------------------------------|-------------------------------------------|------|
| 1 | phase transitions: 1→2→3                                | Settling → Relay → Complete               | [ ]  |
| 2 | setpoint_deg alternates ±5° during phase 2              | Clean relay switching                     | [ ]  |
| 3 | measurement_deg crosses zero between relay flips        | Oscillation is forming                    | [ ]  |
| 4 | cycles_done increments to 8                             | 2 discard + 6 measured                    | [ ]  |
| 5 | amplitude_deg stays < 20°                               | Within safety limit                       | [ ]  |

### 5.7 Performance

| # | Check                                                   | Expected                                  | Pass |
|---|---------------------------------------------------------|-------------------------------------------|------|
| 1 | system_status.loop_time_us: max                         | < 13000 µs (13ms budget)                  | [ ]  |
| 2 | system_status.loop_time_us: average                     | < 5000 µs (comfortable margin)            | [ ]  |
| 3 | attitude_data sample interval                           | Consistent ~13ms, no gaps > 26ms          | [ ]  |
| 4 | During autotune: loop_time_us not elevated              | < 13000 µs (no performance regression)    | [ ]  |

---

## Abort Criteria

Stop testing and investigate if any of these occur:
- Any axis in 1.1/1.2/1.3 is reversed — fix inversions before proceeding
- Kill switch does not stop motors — do NOT fly
- Manual mode escape does not immediately disable PID
- PID oscillates uncontrollably in Stabilized mode (reduce `config.scale` in system.rs)
- Mode switch has no effect (check CH6 wiring / thresholds)
- SD card not mounting or ULog not auto-starting

## Post-Ground Sign-Off

All tests in Part 1 through Part 4 passed: [ ]
Reviewed by: _______________
Date: _______________
