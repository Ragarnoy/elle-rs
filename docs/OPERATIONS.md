# Operating Elle

How to arm, fly, calibrate, tune and record with the Elle flight controller, and how to
read what the aircraft tells you (LED, beeps, event codes). Applies to both airframes
(eagle: twin engine, dart: single engine) unless noted.

Build and architecture details are in [`CLAUDE.md`](../CLAUDE.md); the host tool is
described in [`tools/elle-rpc-host/README.md`](../tools/elle-rpc-host/README.md).

The constants quoted here live in `crates/elle-config/src/lib.rs` unless another file is
named. If this page and the code disagree, the code is right — fix the page.

## Firmware modes

| Build | Pilot input | Arming |
|-------|-------------|--------|
| Flight (default features) | CRSF/ELRS receiver | Throttle gesture |
| `rpc-control,gnss` | Host TUI / `direct` over the debug probe | RPC `arm` command only |
| `rpc-control,rpc-rc,gnss` | CRSF receiver, host monitors | Throttle gesture (RPC `arm` also accepted) |

## Arming and disarming

### Throttle gesture (RC)

The aircraft boots **disarmed** and never arms on its own. To arm:

1. Raise the throttle to at least ~30 % (`ARM_THROTTLE_HIGH_RAW` = 614 raw CRSF).
2. Bring it back down to **zero thrust** — the bottom of the stick, where the throttle
   curve outputs 0 (raw ≤ `THROTTLE_DEADZONE` = 200).

The aircraft arms on step 2, so it always arms with the engines at zero. Booting with
the stick already low does nothing: the "high" half of the gesture has to be seen first.

The gesture is forgotten on every disarm — kill switch, RC failsafe, RPC disarm or
emergency stop — so each re-arm needs a fresh up-and-down. It cannot complete while
failsafe is active. Code: `crates/elle-control/src/arming.rs`.

### Home position (GNSS builds)

Home is set automatically; there is nothing to press. While disarmed it follows every
good fix (h_acc ≤ 5 m, ≥ 6 satellites) with the baro altitude at that moment; arming
locks it for the flight, and disarming releases it. Arm where you want home to be,
after the fix is good. The radio's flight-mode text says whether arming would lock one:

| FM text (disarmed) | Meaning |
|---|---|
| `WAIT H` | Home set: arming locks it |
| `NOHOME` | No fix good enough yet; a flight armed now has no home |
| `WAIT` | Build without GNSS |

An EdgeTX logical switch on the FM sensor can announce the change. The radio's own
GPS "home" (first fix it received) is a different point.

Navigation only observes today (logged as ULog `nav`, never applied), so a missing
home costs nothing but the navigation log for that flight.

### RPC (`rpc-control` without `rpc-rc`)

Arming is explicit: `arm` in the TUI or `direct arm`. The gesture is not used. `arm` is
**refused while the commanded throttle is above zero** (event 18) — set throttle 0 first.
`disarm` also zeroes the commanded throttle, so the next `arm` never spins the engines
back up. `stop` (emergency stop, event 12) disarms and zeroes throttle and surfaces.

### Kill switch (CH8)

CH8 high (> 1500 µs) disarms on every control tick, centres the elevons, sends zero to
the engines and skips the controller entirely. Releasing it does **not** re-arm — do the
throttle gesture again. Events 16 (engaged) / 17 (released). Honoured in flight mode and
with `rpc-rc`.

### Beeps

The ESCs beep through the motors: one beep sequence on arm, a double beep on disarm.
Engine arming at power-up is a separate 2 s MotorStop burst (`ARM_DURATION_MS`) that
the ESCs need before they accept throttle; it is not the flight-controller arm.

## Failsafe

**RC link (flight mode and `rpc-rc`).** Link age is measured from the last CRSF frame:

| Age | State | Effect |
|-----|-------|--------|
| > 200 ms (`RC_WARNING_MS`) | Warning | Event 13, LED fast-blinks orange while armed |
| > 300 ms (`RC_TIMEOUT_MS`) | Lost | Event 14: disarm, zero thrust, PID reset, surfaces centred, LED rapid-flashes orange |
| Frames resume | Restored | Event 15. Still disarmed — re-arm with the gesture |

**Host link (pure RPC mode).** The same thresholds apply to the time since the host last
sent anything (`HOST_LAST_RX_MS` in `crates/elle-system/src/rpc.rs`). A crashed TUI,
unplugged probe or killed `direct` disarms the aircraft and forgets the commanded
throttle and surfaces. The TUI polls continuously, so it holds the link by itself.
`direct throttle` (non-zero), `direct elevon` and `direct arm` then ping every 100 ms and hold the
link until Ctrl-C, then send throttle 0 and disarm before exiting.

## RC channel map

| Channel | Function | Positions |
|---------|----------|-----------|
| CH1–CH4 | Roll, pitch, throttle, yaw | — |
| CH5 | Heading hold | 2-pos: > 1024 = on (0.5 s debounce) |
| CH6 | Flight mode | < 500 Manual · < 1300 Stabilized · above AltitudeHold |
| CH7 | Autotune | < 500 off · < 1300 pitch · above roll |
| CH8 | Kill switch | > 1500 = kill (disarm) |

Radio-side setup (LiteRadio 3 Pro / EdgeTX) is not in the repo.

## Flight modes

- **Manual** — sticks drive the elevons directly; attitude controller off.
- **Stabilized** — sticks command an attitude: up to ±25° pitch
  (`STABILIZED_MAX_PITCH_DEG`) and ±45° roll (`STABILIZED_MAX_ROLL_DEG`). Setpoints are
  smoothed and slewed at no more than 90°/s.
- **AltitudeHold** — currently a **level hold** (0° pitch, 0° roll), throttle stays
  manual. There is no barometric altitude loop yet.

### Heading hold (CH5, Stabilized only)

Switching CH5 on while in Stabilized captures the current yaw as the target (event 130)
and replaces the roll stick with a P heading controller: bank limited to ±25°
(`HEADING_HOLD_MAX_ROLL_DEG`), bank command slewed at 15°/s. Leaving Stabilized or
switching CH5 off disengages it (event 131). Needs a calibrated magnetometer to hold a
true heading. In RPC mode the `SetHeadingHold` endpoint sets an explicit target heading
instead (event 132).

## Boot

Keep the aircraft **still for about a second after power-up**. Core 1 averages the
gyro to measure its bias (`GYRO_BIAS_SAMPLES` = 1 s at 1 kHz), restarting whenever it
sees movement, and gives up after 10 s.

- Success: event 46, LED goes solid green (purple in RPC mode), TUI shows "Cal".
- Failure (never still, or bias implausibly large): event 47, the IMU runs with zero
  bias and the LED keeps pulsing. Power-cycle and try again rather than fly it.

The LED slow-blinks blue while the IMU initialises.

## Calibration

Both calibrations are stored in flash, reload at every boot, and are **refused while
armed**. Collection runs only while disarmed. Each is stored separately: clearing one
(`… clear`) removes only that one, and `clearpid` touches neither.

### Magnetometer (hard iron)

Removes the PCB's own magnetic offset. Collect 300 samples (~30 s at 10 Hz) while
rotating the aircraft through every orientation; each axis must span ≥ 5000 counts or the
result is rejected. The offsets are subtracted before the AHRS; the TUI mag panel keeps
showing **raw** counts, so check the result with `mag cal` and by watching yaw.

- TUI: `mag cal start` · `mag cal clear` · `mag cal` (status)
- Direct: `direct mag-cal start|clear|status`
- Field: double-tap the airframe (see below) with CH7 **off**.
- LED fast-blinks yellow while collecting. Events 110–116.

### Level (IMU mounting offset)

Without it, 0° means the circuit board is level, not the airframe. Hold the aircraft
still at its reference (level-flight) attitude: Core 1 waits 0.5 s for the trigger to
settle, then averages the accelerometer for 2 s. Rejected if it moves or if the board is
tilted more than 15° (which also catches an upside-down board); the previous correction
stays.

Reported angles are what the *uncorrected* attitude reads with the airframe level:
"pitch −2.1°" means the board sits 2.1° nose-down.

- TUI: `level cal start` · `level cal clear` · `level cal`
- Direct: `direct level-cal start|clear|status`
- Field: double-tap with CH7 in **pitch or roll**.
- LED fast-blinks cyan while collecting. Events 150–158.

### Field double-tap

Available in flight mode and with `rpc-rc`. Requires all of:

- kill switch **on** and aircraft disarmed;
- throttle low (raw < 200);
- gyro quiet (< 0.5 rad/s) — hold it still, then tap twice sharply.

Event 120 confirms the tap. CH7 selects which calibration starts. This cannot start an
autotune: autotune only starts while armed, on an off→pitch/roll transition.

## PID autotune

A relay test: the controller swings the attitude setpoint ± the relay amplitude and
measures the resulting oscillation, then derives gains (Tyreus–Luyben by default).

**RC (flight mode):** armed, in Stabilized or AltitudeHold, move CH7 from off to pitch
or roll. Relay 5°, 6 cycles. Back to off — or straight across to the other axis — aborts
and restores the previous gains (event 92); a run never changes axis. After a successful pitch tune, pitch is locked until reboot so that parking CH7 in
the middle position doesn't retune it; roll can be run again.

**TUI (RPC mode):** `autotune pitch|roll [relay_deg] [cycles] [tl|zn|so]` (defaults 5°,
6, `tl`; cycles 1–14, the most one run can measure) · `autotune abort` · `savepid` (save the current gains) · `clearpid` (back to
firmware defaults on next boot; calibrations are kept). `clearpid` does nothing on the
dart.

**Outcome:**

- Zero crossings (and relay flips) use a ±0.5° hysteresis band, so attitude noise near
  level neither chatters the relay nor counts as oscillation. Noise alone ends in the
  no-oscillation timeout (event 93).
- The result is validated before use: oscillation amplitude ≥ max(0.5°, 20 % of the
  relay), period 0.1–5 s, gains inside the sane range, and the tuned axis's Kp and Kd
  within 3× (up or down) of the gains flown before the run. Ki is not ratio-limited —
  the flown Ki is deliberately far below what the tuning rules give. A failed check
  restores the previous gains and fires event 94 (*rejected*). To move gains further
  than 3×, set them by hand (`pid`) or tune again from the new result.
- Safety abort, event 93, gains restored and setpoint override cleared:
  - timeout (60 s, or 10 s after settling with no oscillation);
  - pitch or roll beyond ±20° (or non-finite) at any point, settling included;
  - the run loses the aircraft: kill switch, disarm (including failsafe), Manual mode or
    Core 1 unhealthy (attitude controller off), or no valid attitude.
- Success (event 91): the new gains apply **immediately**, and are written to flash
  **after the next disarm** — never in the air, since a flash write stalls both cores.
- On the dart, `IGNORE_PID_FLASH` is set: tuned gains are used until power-off but never
  saved or loaded; the firmware defaults are always used at boot.

Going back to Stabilized after an abort does not resume the test; start a new run.

## Flight recording (ULog)

Recordings go to the **SD card** as `.ulg` files (open with PlotJuggler, pyulog or
Flight Review).

- **Flight mode:** starts automatically once the SD card is detected and initialised,
  and runs until power-off. Insert the card before power-up.
- **RPC mode:** `ulog start` / `ulog stop` in the TUI.

Logged at the 200 Hz control rate: attitude and controller internals (PID terms,
setpoints, saturation, elevon pulses, loop dt); pilot commands and engine telemetry at
100 Hz; status at 8 Hz; baro and mag at their sensor rates; every GNSS solution (5 Hz); PID gains
once per file and on every change. The navigator (`nav`, 25 Hz, GNSS builds) runs in
observation mode: it logs what it would bank for a loiter around home and never moves
a surface (home: see [Home position](#home-position-gnss-builds)). ESC link health (`esc_health`, ~1 Hz) counts DShot telemetry replies, timeouts,
corrupt replies and re-configurations per ESC: corrupt replies point at wiring noise, and
timeouts climbing at idle mean an ESC is silent. Core 1 load (`core1_load`, ~1 Hz) gives
the IMU task's mean and max busy time per 1 ms sample and its longest mag and baro reads.
The `gyro-raw-log` build adds every
1 kHz gyro sample for vibration analysis.
See [`crates/elle-ulog/README.md`](../crates/elle-ulog/README.md).

`ulog extract [file]` and `ulog erase` operate on the legacy on-board flash store only.

## LED

| Situation | Flight mode | RPC mode |
|-----------|-------------|----------|
| Booting, IMU initialising | Slow blink blue | Slow blink blue |
| Disarmed, gyro bias measured | Solid green | Solid purple |
| Disarmed, bias not measured | Pulse cyan | Pulse purple |
| Armed, Manual | Double blink green | Double blink purple |
| Armed, Stabilized / AltitudeHold | Pulse cyan | Double blink purple |
| Armed, heading hold | Pulse blue | Pulse blue |
| Armed, link warning | Fast blink orange | Fast blink orange |
| Failsafe (link lost) | Rapid flash orange | Rapid flash orange |
| Mag cal collecting | Fast blink yellow | — |
| Level cal collecting | Fast blink cyan | Fast blink cyan |

Code: `led_pattern` logic in `crates/elle-app/src/flight.rs` and `rpc.rs`.

## Event codes

Firmware events travel to the TUI log panel as numeric codes
(`crates/elle-hardware/src/event.rs`; host labels in
`tools/elle-rpc-host/src/tui/ui.rs` `log_code_text()`). They also appear in defmt output
when built with `defmt-logging`.

| Code | Meaning |
|------|---------|
| 1–9 | GNSS: fix, configuration, baud, fallback to NMEA |
| 10 / 11 | Armed / disarmed |
| 12 | Emergency stop (RPC) |
| 13 / 14 / 15 | Link warning / link lost (failsafe) / link restored |
| 16 / 17 | Kill switch engaged / released |
| 18 | RPC arm refused: throttle not at zero |
| 20–23 | CRSF telemetry TX |
| 30 / 31 / 33 / 34 | ULog started (RPC) / init failed / stopped / flash erased |
| 40–44 | IMU init failed, FIFO overflow, read errors, mag init failed, baro init failed |
| 45 | IMU catch-up: more than one sample drained per wake-up |
| 46 / 47 | Gyro bias measured / failed |
| 48 | I2C bus failed: mag and baro both taken offline |
| 50 / 51 | First CRSF frame / CRSF UART error |
| 61 / 62 | ULog flash erase failed / flash write timeout |
| 63 | Flash write refused while armed |
| 70 / 71 | Core 1 (IMU) unhealthy / restored |
| 80 | Attitude data stale |
| 81 | ULog auto-started on SD (flight mode) |
| 90–94 | Autotune started / complete / aborted / safety abort / rejected |
| 100–103 | PID saved / save failed / loaded / nothing saved |
| 110–116 | Mag cal started, complete, failed, saved, cleared, loaded, nothing saved |
| 120 | Double-tap detected |
| 130 / 131 / 132 | Heading hold engaged / disengaged / target set |
| 140 / 141 | GNSS configuration timeout / partially accepted |
| 150–158 | Level cal started, complete, failed moving (or refused armed), failed tilted, saved, save failed, cleared, loaded, nothing saved |
| 160 / 161 | Left / right ESC stopped replying to DShot telemetry (power loss or restart) |
| 162 / 163 | Left / right ESC re-sent spin direction and extended telemetry: it reappeared (restart, late power), or it answers but sent no EDT |
| 164 / 165 | Left / right ESC still sends no EDT after the retries: its spin direction is unconfirmed |

Codes 32, 60 and 82 are retired and never sent.
