# Ground Test Plan — Flight Mode Rework

Pre-flight verification for Stabilized/AltitudeHold mode changes.
Test date target: 2026-03-29.

## Prerequisites

- BetaFPV Pro transmitter bound and linked
- CH6 (3-pos switch left): Manual / Stabilized / AltitudeHold
- Props removed for all ground tests
- Probe-rs debug probe connected (RPC tests only)

## Build Commands

```sh
# Flight firmware (default — CRSF/ELRS)
cd crates/elle-eagle
cargo run --release

# RPC firmware (ground test mode)
cargo run --release --no-default-features --features rpc-control

# RPC + RC firmware (probe monitoring with RC control)
cargo run --release --no-default-features --features rpc-control,rpc-rc

# Host TUI
cargo run -p elle-rpc-host --target x86_64-unknown-linux-gnu
```

---

## Part 1: RPC Mode Tests (probe connected, no RC needed)

Flash `--features rpc-control`, launch TUI.

### 1.1 TUI Command Acceptance

| # | Command             | Expected                                        | Pass |
|---|---------------------|-------------------------------------------------|------|
| 1 | `mode stab`         | "Mode: Stabilized"                              | [ ]  |
| 2 | `mode stabilized`   | "Mode: Stabilized"                              | [ ]  |
| 3 | `mode althold`      | "Mode: AltitudeHold"                            | [ ]  |
| 4 | `mode altitudehold` | "Mode: AltitudeHold"                            | [ ]  |
| 5 | `mode manual`       | "Mode: Manual"                                  | [ ]  |
| 6 | `mode mixed`        | Error: "Mode must be: manual, stab, or althold" | [ ]  |
| 7 | `mode auto`         | Same error                                      | [ ]  |
| 8 | `setpoint 5 5`      | Error: unknown command                          | [ ]  |

### 1.2 Mode Display in TUI

| # | Action         | Expected header display   | Pass |
|---|----------------|---------------------------|------|
| 1 | `mode manual`  | Mode shows "Manual"       | [ ]  |
| 2 | `mode stab`    | Mode shows "Stabilized"   | [ ]  |
| 3 | `mode althold` | Mode shows "AltitudeHold" | [ ]  |

### 1.3 Stabilized Mode — Stick-to-Setpoint (RPC elevon input)

With mode set to `stab`, the elevon command maps to pitch/roll stick which becomes the attitude setpoint.

| # | Command sequence          | Expected in controller output panel       | Pass |
|---|---------------------------|-------------------------------------------|------|
| 1 | `mode stab`, `arm`        | Attitude controller active                | [ ]  |
| 2 | `elevon 0 0`              | Setpoint near 0/0, PID corrections small  | [ ]  |
| 3 | `elevon 50 0`             | Pitch setpoint ~12.5° (50% of 25°)        | [ ]  |
| 4 | `elevon 0 50`             | Roll setpoint ~22.5° (50% of 45°)         | [ ]  |
| 5 | `elevon 100 100`          | Pitch setpoint ~25°, roll ~45° (max)      | [ ]  |
| 6 | `elevon -100 -100`        | Pitch setpoint ~-25°, roll ~-45°          | [ ]  |
| 7 | `elevon 0 0` → tilt board | PID corrections respond to attitude error | [ ]  |

Note: elevon maps to pitch = (left+right)/2, roll = (right-left)/2.

### 1.4 AltitudeHold Mode — Level Hold

| # | Command sequence      | Expected                                          | Pass |
|---|-----------------------|---------------------------------------------------|------|
| 1 | `mode althold`, `arm` | Setpoint locked at 0°/0°                          | [ ]  |
| 2 | `elevon 100 100`      | Setpoint still 0°/0° (stick ignored for setpoint) | [ ]  |
| 3 | Tilt board            | PID corrections oppose tilt                       | [ ]  |
| 4 | Return board level    | Corrections return to ~0                          | [ ]  |

### 1.5 Manual Mode — No PID

| # | Command sequence     | Expected                            | Pass |
|---|----------------------|-------------------------------------|------|
| 1 | `mode manual`, `arm` | PID corrections stay 0              | [ ]  |
| 2 | `elevon 50 -50`      | Elevons move, no PID engagement     | [ ]  |
| 3 | Tilt board           | No PID response, corrections stay 0 | [ ]  |

### 1.6 Manual Escape

| # | From mode                 | Action        | Expected                                     | Pass |
|---|---------------------------|---------------|----------------------------------------------|------|
| 1 | `stab` (armed, tilted)    | `mode manual` | PID immediately stops, corrections drop to 0 | [ ]  |
| 2 | `althold` (armed, tilted) | `mode manual` | Same — instant PID disengage                 | [ ]  |

### 1.7 Autotune in Stabilized Mode

| # | Command sequence   | Expected                                  | Pass |
|---|--------------------|-------------------------------------------|------|
| 1 | `mode stab`, `arm` | Armed in Stabilized                       | [ ]  |
| 2 | `throttle 30`      | Motors spin (if connected)                | [ ]  |
| 3 | `autotune pitch`   | Autotune starts, setpoint override active | [ ]  |
| 4 | `autotune abort`   | Autotune aborts, original gains restored  | [ ]  |

### 1.8 ULog Setpoint Recording

| # | Steps                                | Expected                                       | Pass |
|---|--------------------------------------|------------------------------------------------|------|
| 1 | `mode stab`, `arm`, `elevon 50 0`    | -                                              | [ ]  |
| 2 | `ulog start`                         | Recording started (code 30)                    | [ ]  |
| 3 | Wait 3s, `ulog stop`, `ulog extract` | File saved                                     | [ ]  |
| 4 | Open .ulg, check `pilot_commands`    | `pitch_setpoint_deg` ~12.5° (not 0 or garbage) | [ ]  |

---

## Part 2: RC Mode Tests (transmitter required)

All inputs via RC sticks/switches. Two firmware options:

- **Default firmware** (`cargo run --release`) — RC only, verify via defmt logs or servo movement
- **RPC+RC firmware** (`--no-default-features --features rpc-control,rpc-rc`) — RC controls flight, TUI provides live monitoring (attitude, setpoints, mode, engine). Recommended for ground testing since you can see PID state.

Transmitter must be bound. Props off for all tests.

### 2.1 Mode Switch Mapping (CH6 3-position)

| # | CH6 position             | Expected mode | Pass |
|---|--------------------------|---------------|------|
| 1 | Position 1 (low, ~306)   | Manual        | [ ]  |
| 2 | Position 2 (mid, ~1000)  | Stabilized    | [ ]  |
| 3 | Position 3 (high, ~1694) | AltitudeHold  | [ ]  |

Verify via TUI (rpc-rc build) or defmt log output.

### 2.2 Stabilized Mode — Stick Feel

Hold board in hand, props off, armed.

| # | Stick input                           | Expected servo response                           | Pass |
|---|---------------------------------------|---------------------------------------------------|------|
| 1 | CH6 mid (Stabilized), sticks centered | Elevons hold trim position, PID corrects for tilt | [ ]  |
| 2 | Full pitch stick forward              | Elevons deflect to nose-down attitude (~25°)      | [ ]  |
| 3 | Full pitch stick back                 | Elevons deflect to nose-up (~25°)                 | [ ]  |
| 4 | Full roll stick left                  | Elevons split for left roll (~45°)                | [ ]  |
| 5 | Full roll stick right                 | Elevons split for right roll (~45°)               | [ ]  |
| 6 | Release sticks (center)               | Elevons return to level hold (0°/0°)              | [ ]  |
| 7 | Stick centered, tilt board nose-up    | PID pushes elevons to correct back to level       | [ ]  |
| 8 | Throttle stick                        | Throttle responds directly (no PID on throttle)   | [ ]  |
| 9 | Yaw stick                             | Differential thrust responds directly             | [ ]  |

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

### 2.6 ULog Recording via RC Switch

| # | Steps                                     | Expected                                                 | Pass |
|---|-------------------------------------------|----------------------------------------------------------|------|
| 1 | CH5 high (ULog on), fly Stabilized for 5s | -                                                        | [ ]  |
| 2 | CH5 low (ULog off)                        | Recording stopped                                        | [ ]  |
| 3 | Extract via probe or next RPC session     | `pitch_setpoint_deg` in ULog matches stick-derived angle | [ ]  |

### 2.7 RC Autotune (CH7 3-position)

| # | Steps                                    | Expected                                        | Pass |
|---|------------------------------------------|-------------------------------------------------|------|
| 1 | CH6 mid (Stabilized), armed, throttle up | -                                               | [ ]  |
| 2 | CH7 mid (pitch autotune)                 | Autotune starts, oscillation visible on elevons | [ ]  |
| 3 | CH7 low (off)                            | Autotune aborts                                 | [ ]  |
| 4 | CH6 low (Manual) during autotune         | PID off, autotune paused, full manual           | [ ]  |

---

## Abort Criteria

Stop testing and investigate if any of these occur:
- Manual mode escape does not immediately disable PID
- Elevons do not respond in Manual mode
- PID oscillates uncontrollably in Stabilized mode (reduce gains)
- Mode switch has no effect (check CH6 wiring / thresholds)
- ULog setpoint values are 0 when stick is deflected (logging bug)

## Post-Ground Sign-Off

All tests in Part 1 and Part 2 passed: [ ]
Reviewed by: _______________
Date: _______________
