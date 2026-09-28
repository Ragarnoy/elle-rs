# Elle TODO

Open work only — finished work is in git history. Current behaviour is documented in
[`CLAUDE.md`](CLAUDE.md) (developer reference) and
[`docs/OPERATIONS.md`](docs/OPERATIONS.md) (operator guide).

When proposing new event codes, take a free range: `event.rs` uses 1–9, 10–18, 20–23,
30–34, 40–48, 50–51, 60–63, 70–71, 80–82, 90–94, 100–103, 110–116, 120, 130–132,
140–141, 150–158 and 160–163; **170 and up is free**. Flash MapStorage keys 1–3 are taken (PID,
mag cal, level cal) and `MAP_KEY_SLOTS = 4`, so a new key means raising it.

## Pending verification

Implemented, but not yet confirmed on hardware:

- [ ] **RC failsafe, TX off** (dart): expect event 14 within ~300 ms, disarm, orange rapid flash; then re-arm needs the gesture.
- [ ] **Host-link failsafe**: kill the TUI / unplug the probe while armed in RPC mode → disarm within ~300 ms. `direct throttle` holds the link until Ctrl-C.
- [ ] **Arming gesture** on both airframes: boot with stick low does not arm; up-then-down arms at zero thrust; kill and failsafe need a new gesture.
- [ ] **GNSS UBX path**: 115200 baud switch, 5 Hz NAV-PVT, velocity/accuracy fields, GGA fallback.
- [ ] **Elevon latency**: scope PIN_12/13 against a stick step — expect ≤ one 5 ms frame (200 Hz servo PWM). Servos confirmed fine at 200 Hz (no jitter, cool).
- [ ] **Pitch autotune** since the measurement-invert fix, and autotune in general since the latency fixes (gains derived before them were compensating for delay). Check the save lands after disarm and that a bad run is rejected (event 94).
- [ ] **Governor** at sustained full throttle on the dart's re-swept table (2026-09-26): RPM should hold flat.
- [ ] **Autotune hysteresis and gain cap** in flight: a normal run still completes with ±0.5° hysteresis (not timing out), and the 3× Kp/Kd cap doesn't reject reasonable results (check `autotune_status` and event 94 in ULog).
- [ ] **Autotune ownership aborts**: Manual, kill, failsafe, CH7 pitch→roll and a 20° off-axis excursion each abort the run and restore gains (TEST_PLAN 3.4.4, 3.4.4b, 3.4.6).
- [ ] **Per-entry flash clear**: `clearpid`, `mag cal clear`, `level cal clear` each remove only their own entry (TEST_PLAN 6.3).
- [ ] **DShot ESC re-configuration** (TEST_PLAN 6.8 rows 1, 2, 4, 5): scope the DShot lines across a stall; FC on USB then battery → events 162/163; ESC power cut → 160/161 then 162/163. Already confirmed (LOG_0043, 13 min armed idle): no twitches or beeps, 1000 frames/s per ESC, 0 timeouts / corrupt replies.
- [ ] **Mag rate** (branch `mag-odr-fix`): `magnetometer_data` changes on every ~10 Hz read (it changed once a second before: `CMM_FREQ_EN` was the wrong bit); log shows `MMC5616WA: ... continuous mode at 50 Hz`; mag cal still completes. The async I2C task itself is verified (LOG_0047: mag/baro at their rates, no event 48).
- [ ] **GNSS after `cargo run`** (branch `gnss-baud-probe`): run twice without a power cycle; the second boot should log `GNSS: 115200 baud, 200 ms solution`, with no steady `Framing error` stream and no unanswered config keys.
- [ ] **EDT retry** (branch `esc-edt-retry`): across several boots, `esc_health` `*_edt_frames` climbs on both ESCs every time, and voltage/temperature appear in `engine_data`; a session where the boot configuration missed shows event 162/163 about 5 s after boot. On the dart, confirm the spin direction on each boot.
- [ ] **ULog buffering** (branch `ulog-buffering`): a long armed log shows no ULog dropouts after boot (`elle_log.py timing` reports no logging gaps; LOG_0058 had four of 95–180 ms with 8 channel slots).
- [ ] **200 Hz control loop** (branch `loop-200hz`): bench log shows a 5.00 ms median tick, no late ticks after boot, no ULog dropouts at ~30 kB/s over a session long enough to reach the card's slow phase (LOG_0065 dropped 1.4 s ~3.3 MB in at ~40 kB/s, before `commands`/`engine_data` went to 100 Hz); Stabilized on the stand feels unchanged and doesn't oscillate; elevon pulse within one 5 ms frame of a stick step (TEST_PLAN 6.7). Then a cautious first flight and an autotune pass.
- [ ] **I2C fault handling**: an I2C error drops both mag and baro (event 48) without stalling Core 1.

## Near term

**1. Pre-arm checks (~1 h)**
- Attitude freshness: `ATTITUDE` age < 100 ms (proves Core 1 and the IMU are alive).
- Gyro bias measured (`IMU_STATUS.calibrated`) — refuse to arm on an unmeasured bias.
- ESC arming complete: gate on an `ESCS_READY` flag set after the 2 s MotorStop burst.
- A specific error code per failed check (RPC `ArmEndpoint` already returns `error_code`), for both gesture and RPC arming.
- Not EDT voltage (reads high, 0 before the first frame) — handle that as a runtime failsafe.

**2. Low battery failsafe (~1 h)**
- Voltage thresholds in `elle-config` (per cell or absolute), from EDT voltage in `ENGINE_CACHE`.
- Warning (LED + event) at the first threshold, failsafe at the critical one.

**3. LED coverage (~30 min)**
- Autotune active, ULog recording and low battery have no LED pattern yet.

**4. Stick-to-setpoint latch**
- A switch that makes the pitch/roll sticks set target angles and latches them when released (hands-off hold). CH5–CH8 are all taken; needs a new channel.

## DShot follow-up

- **Beep use cases** — arm/disarm beeps exist (`BEEP_SIGNAL`); still missing: lost-model alarm (after failsafe + timeout), low battery on the ground, calibration done/failed. Never while armed or with throttle > 0.
- **Motor health detection** — alert when the governor can't correct: DShot at `GOVERNOR_DSHOT_MAX` with RPM still low (obstruction, dying motor), RPM to zero with DShot > 0 (desync), sustained integrator windup. New event range, optional `health: u8` in the ICD `EngineUnit`.

## Field readiness

**Crash detection / auto-disarm (~2–3 h)**
- ICM-42686 APEX Wake-on-Motion + Significant Motion Detection routed to INT2 (GPIO4, wired, unused).
- Core 1 waits on GPIO4, confirms `INT_STATUS3.smd_int`, signals Core 0 → disarm + event + MotorStop. Only while armed; threshold in `elle-config` high enough to ignore flight loads.
- `crash_detected` in `FlightState` / ULog.

**Ground level reference / AGL (~30 min)**
- `SetGroundLevel` RPC (or auto on arm) captures baro altitude; `agl_m` in `StatusResp` and ULog. Prerequisite for the navigation safety layers.

**Battery mAh tracking (~1.5 h)** — integrate EDT current in the DShot task; fill the zeroed CRSF capacity/remaining fields; expose over RPC. (The dart's ESC has no current sensor.)

**Flight timer + boot counter (~1 h)** — armed time in `StatusResp` and ULog; persistent boot counter in flash (new key).

**Flash crash blackbox** — ULog now goes to SD only. A low-rate status + events stream in the legacy flash region would survive SD failure or ejection.

**Servo trim via RPC (~1.5 h)** — `SetTrimReq { left_us, right_us }`, persisted (new flash key), TUI `trim left 10`.

**Expo curves (~1.5 h)** — per-axis expo on pitch/roll/yaw via compile-time LUTs.

**EdgeTX Lua telemetry script (~2–3 h)** — mode, RPM, battery, ULog state, link quality on the LiteRadio 3 Pro from the CRSF frames already sent.

## Waypoint navigation

See [`docs/NAVIGATION_PLAN.md`](docs/NAVIGATION_PLAN.md). `crates/elle-nav` exists as a
workspace member (sguaba, nalgebra) with no firmware users yet. Heading hold already
provides the heading → roll outer loop.

1. Nav math (`elle-nav`): coordinates, bearing, distance
2. ~~Heading controller~~ — done (heading hold, `elle-control/src/heading.rs`)
3. Altitude hold: baro altitude error → pitch setpoint (AltitudeHold is a level hold today)
4. Fly-to-point (Guided mode)
5. Waypoint sequencing
6. L1 path following
7. Loiter / RTL
8. Safety layers: geofence, minimum AGL floor, upset recovery (bank > 60° or pitch < −30° → wings level + climb), oscillation detection, RTL on failsafe
9. Mission upload over RPC + TUI
10. TECS

## Pitot tube / airspeed

Needed for safe autonomous flight: GPS groundspeed is the only speed reference today and
breaks down in wind. Enables stall protection, wind-aware guidance and proper TECS. Low
priority until navigation reaches the outer loops.
