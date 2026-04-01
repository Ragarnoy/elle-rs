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

### 1.4 Kill Switch

| # | Action                        | Expected                                   | Pass |
|---|-------------------------------|---------------------------------------------|------|
| 1 | Armed, throttle up, CH5 high  | Motors stop immediately                     | [ ]  |
| 2 | CH5 still high, throttle low  | Motors stay off (no re-arm)                 | [ ]  |
| 3 | CH5 low, throttle low         | Re-arms (throttle-low auto-arm)             | [ ]  |
| 4 | Throttle up                   | Motors spin normally                        | [ ]  |

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

### 3.3 Autotune ULog (RPC mode)

| # | Steps                      | Expected                                    | Pass |
|---|----------------------------|---------------------------------------------|------|
| 1 | `mode stab`, `arm`         | Armed in Stabilized                         | [ ]  |
| 2 | `autotune pitch`           | Autotune starts                             | [ ]  |
| 3 | `autotune abort`           | Autotune aborts                             | [ ]  |
| 4 | Pull SD, check ULog        | `autotune_status` message present with phase transitions | [ ]  |

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
