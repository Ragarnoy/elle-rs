# Elle TODO

## Completed

- [x] RPC autotune trigger (StartAutotune/AbortAutotune endpoints, TUI commands)
- [x] Flash persistence for PID gains (MapStorage on 0x200000-0x20FFFF, boot-time load, auto-save on completion, `savepid` TUI command)
- [x] DShot300 bidirectional motor protocol replacing PWM for engines (PIO1 left engine, PIO2 right engine, LED moved to PIO0 SM2)
- [x] RPM telemetry storage and display (EDT bidir reading, `ENGINE_CACHE`, `GetEngineEndpoint` RPC, TUI 5Hz poll, `direct engine`, ULog `engine_data` at 77Hz, per-engine auto-fallback)
- [x] Throttle range rework (removed µs intermediate, control path outputs DShot 0-1999 directly)
- [x] Governor mode: closed-loop RPM control via per-engine PI in 1kHz DShot task (feedforward + anti-windup, telemetry-loss fallback, target eRPM observability in ICD/ULog/TUI)
- [x] Extended DShot telemetry (EDT): ESC temperature, voltage, current via `read_extended_telemetry()`, best-effort (0 if unsupported)
- [x] Magnetometer hard-iron calibration: min/max tracking over 300 samples, flash persistence (MapStorage key=2), auto-load on boot, TUI/direct CLI commands, cross-core signals
- [x] Failsafe improvements: `RcLinkState` state machine (Ok→Warning→Lost), transition-based events (codes 13-15), `rc_age_ms` in StatusResp/FlightState/ULog, signal restoration, LED warning pattern (FastBlink/orange)
- [x] Interrupt-driven IMU reads: INT1 (PIN_5) DATA_RDY replaces 500µs FIFO polling — deterministic latency, no wasted SPI reads, true CPU sleep between samples
- [x] CRSF telemetry battery frame: EDT voltage/current to radio OSD via Battery Sensor frame (0x08)
- [x] ULog flash crash fix: SIO_IRQ_FIFO masking around flash ops (executor-interrupt handler on Core0 was stealing PAUSE_TOKEN during pause_core1)
- [x] ULog write batching: flash manager drains channel and pushes batched data in single queue.push() — reduces in_ram() pause frequency ~4x
- [x] Governor feedforward: measured RPM-to-DShot LUT replaces linear estimate
- [x] RC channel reassignment: CH5=ULog, CH6=attitude mode (3-pos), CH7=autotune (3-pos), CH8=reserved
- [x] Flash persistence across firmware updates: `restore_unwritten_bytes = true` in probe-rs config

## Notes

- `rpc-control` and `defmt-logging` are mutually exclusive (`defmt-rtt` and `rtt-target` both define `_SEGGER_RTT`). Build RPC mode with `--no-default-features --features rpc-control`.
- `rc` feature: enables RC/CRSF flight control within RPC mode (monitoring + RC). Default on in flight mode, off in RPC mode.

---

## Planned

### 1. Near-Term (pre/post test flight)

**1. CH8 modifier: stick-to-setpoint in Autopilot mode**
- When Autopilot + CH8 high, pitch/roll sticks set target angles instead of direct control
- CH8 low latches the last setpoint (hands-off hold)
- Eliminates need for knobs/pots for setpoint control

**2. Pre-arm checks (~1h)**
- **Attitude signal freshness**: `ATTITUDE` cache age < 100ms — proves Core1 alive + IMU producing data (single check replaces separate Core1/IMU gates; ICM-42686 is factory-calibrated so `calibrated` flag is always true and useless as a gate)
- **Throttle edge detection**: require `seen_throttle_high` before low-throttle can trigger arm — prevents instant arming on boot when stick is already at 0. Add `seen_throttle_high: bool` to `ArmingState`, set when throttle > ~1300µs
- **ESC arming complete**: `dshot_task` runs 2s MotorStop burst on boot; gate arming on `static ESCS_READY: AtomicBool` set after sequence completes
- Return specific error code per failed check (RPC `ArmEndpoint` already returns `error_code`)
- Apply to both RC throttle-based arming and RPC arm command
- Skip EDT voltage as pre-arm gate (reads higher than actual, 0 before first EDT frame) — handle as runtime failsafe instead

**3. Low battery voltage failsafe (~1h)**
- Configurable voltage threshold in `elle-config` (per-cell or absolute)
- Warning state (LED + event code) at first threshold, failsafe trigger at critical threshold
- Uses EDT voltage from `ENGINE_CACHE` — no new hardware needed

**4. LED state mapping (~30min)**
- Autotune active → distinct pattern (e.g., FastBlink/Yellow)
- ULog recording → distinct pattern (e.g., SlowBlink/Red)
- Low battery warning → pattern overlay
- Currently only armed/failsafe/calibration have LED feedback

### 2. DShot Follow-Up

DShot300 bidirectional is fully implemented. RPM telemetry, governor mode, EDT, and throttle
rework are done. Remaining items are incremental.

#### Architecture
```
PIO0 SM0 -> PIN_12 (elevon_left)   — PWM
PIO0 SM1 -> PIN_14 (elevon_right)  — PWM
PIO0 SM2 -> PIN_10 (WS2812B LED)   — WS2812
PIO1     -> PIN_11 (engine_left)   — BidirDShot300
PIO2     -> PIN_15 (engine_right)  — BidirDShot300
```

**1. Motor beep feedback via DShot commands (~1.5h)**
- DShot commands 1-5 (Beep1-Beep5) play tones through the motors — no buzzer hardware needed
- **Ground-only**: beep commands replace throttle on the wire, cannot be sent while motors spin
- `dshot_task` needs a `DSHOT_COMMAND` signal: when throttle is zero and a beep is requested, pause throttle loop, send beep via `send_command_repeated_async` (10 frames, ~10ms), resume
- Use cases:
  - Arming confirmation (short beep pattern after arm sequence completes)
  - Disarm confirmation
  - Lost model alarm (continuous beep after failsafe + disarm + timeout, e.g. 30s)
  - Low battery warning on the ground (periodic beep when disarmed + voltage below threshold)
  - Calibration complete / error feedback
- Safety: ignore beep requests if throttle > 0 or motors armed
- Event codes 130+ for beep triggers

**2. Motor health / failure detection (anomaly alerting on governor state)**
- Governor already compares target vs actual RPM — this is a thin alerting layer on top
- Detect uncorrectable conditions: DShot saturated at max but RPM still low (obstruction/dying motor), DShot at minimum but RPM overshooting (prop damage/blade loss), RPM drops to zero while DShot > 0 (ESC desync), sustained integrator windup
- Event codes + optional `health: u8` field in `EngineUnit` ICD for RPC/ULog visibility
- ~half day: state machine in `dshot_task`, event codes 120+, ICD/TUI/ULog updates

---

### 3. Field Readiness

Incremental improvements for safe real-world flight.

#### Safety

**1. Crash detection / auto-disarm (~2-3h)**
- Use ICM-42686 hardware APEX features for zero-CPU-cost impact detection
- **INT1 = GPIO5, INT2 = GPIO4** — both wired, INT2 currently unused
- **Wake-on-Motion (WoM)**: ICM compares accel samples against a programmable threshold internally; fires interrupt on exceedance. Configure via `SMD_CONFIG` register (`wom_mode`, `wom_int_mode`) + WoM threshold register. No firmware polling needed — GPIO interrupt wakes handler.
- **Significant Motion Detection (SMD)**: two WoM events within 1s (short) or 3s (long) = confirmed impact, not just a bump. `smd_mode` in `SMD_CONFIG`, status in `INT_STATUS3.smd_int`.
- **Implementation**: configure WoM threshold (~4-8g), enable SMD short mode, route SMD interrupt to INT2. On Core1 (IMU task), `embassy_rp::gpio::Input::wait_for_*()` on GPIO4 → read `INT_STATUS3` to confirm `smd_int` → signal Core0 via a static Signal → auto-disarm + event code + DShot MotorStop.
- **Safety**: only act when armed. Threshold must be high enough to ignore normal flight loads (banking, gusts) but catch ground impact. Configurable threshold in `elle-config`.
- Event codes 140+ (crash detected, auto-disarm)
- Expose `crash_detected: bool` in `FlightState` / ULog for post-flight analysis

**2. Ground level reference / AGL datum (~30min)**
- RPC endpoint `SetGroundLevel` captures current barometric altitude as ground reference
- TUI command: `set ground` (or auto-set on arm if no explicit set)
- Store as static `GROUND_ALT_M: Mutex<Cell<f32>>`, compute AGL = `baro_alt - ground_alt`
- Expose `agl_m` in `StatusResp` and ULog `system_status` for post-flight analysis
- Prerequisite for nav safety layers (minimum AGL floor, upset recovery)

#### Observability

**3. Battery mAh consumption tracking (~1.5h)**
- Integrate EDT current over time in `dshot_task` (already runs at 1kHz)
- Fill the hardcoded-zero CRSF battery capacity/remaining% fields
- Expose consumed mAh via RPC (extend `EngineResp` or new endpoint)
- Battery capacity constant in `elle-config`

**4. Flight timer + boot counter (~1h)**
- Armed duration since boot in `StatusResp` (use AON timer)
- Persistent boot counter in flash (MapStorage key=3)
- Flight timer in ULog status message

**5. SD card flight logger via SPI1 (~4-5h)**
- Hardware SPI1 (free): MISO=PIN_24, CS=PIN_25, SCK=PIN_26, MOSI=PIN_27
- Async SPI on Core0 — DMA works here (unlike Core1 IMU SPI0)
- FAT32 via `embedded-sdmmc` crate — files readable on any computer, no extraction tool needed
- SD card slot on board (unsoldered, needs populating)
- Init sequence: CMD0→CMD8→ACMD41, then CMD24/CMD25 block writes

**Logging strategy: SD as primary, flash as crash blackbox**
- **SD (primary)**: Full-rate ULog stream (attitude 77Hz, commands 77Hz, engine 77Hz, status 7.7Hz, baro/mag/events). Dedicated `sd_logger_task` receives ULog chunks via channel, writes .ulg files. Buffer 8-16KB to absorb SD write stalls (50-250ms). Unlimited capacity, no wear concern.
- **Flash (blackbox)**: Reduced to safety-critical subset only — status + events at ~1-2Hz. Tiny data volume, negligible wear, survives power loss and SD failure/ejection. Enough to reconstruct what happened in a crash.
- Current full-fidelity flash logging is the main source of wear and capacity pressure. Dropping to events-only makes flash practically unlimited.
- Graceful SD failure: if SD init fails or write errors accumulate, fall back to flash-only full-rate logging (current behavior).

#### Usability

**6. Servo trim via RPC + flash persistence (~1.5h)**
- `SetTrimReq { left_us: i16, right_us: i16 }` endpoint with range validation
- Store in flash (MapStorage key=4), auto-load on boot (same pattern as PID gains)
- TUI commands: `trim left 10`, `trim right -5`
- Eliminates recompile for trim adjustment

**7. Expo curves on pitch/roll/yaw (~1.5h)**
- Compile-time LUT generation for common expo values (0%, 20%, 40%)
- Configurable per-axis via `elle-config` constant or runtime RPC
- Significantly improves stick feel — linear input is too aggressive near center

**8. EdgeTX Lua script for BetaFPV radio (~2-3h)**
- Custom telemetry display: flight mode, engine RPM L/R, battery V/A/mAh, ULog recording status, RC link quality, rc_age_ms
- Reads existing CRSF telemetry frames (attitude, GPS, battery sensor) already sent by firmware
- Stretch: bidirectional CRSF commands for ULog start/stop, mode switching from radio UI (CRSF device parameter protocol or extended frames)
- Depends on: CRSF telemetry TX (done), battery mAh tracking (planned)
- Target radio: BetaFPV Lite Radio 3 Pro (EdgeTX)

**9. Control loop rate increase (~1h, after first flight)**
- Current: 77Hz (13ms). Estimated CPU usage ~3-4%, massive margin available.
- Target: ~150-200Hz. Needs real timing data from `performance-monitoring` feature to confirm budget.
- Benefits: fresher derivative term, smoother integral accumulation, better disturbance rejection.
- Servo limit: elevons are 50Hz PWM, so commands oversample — but PID still benefits from faster sensing.
- **Requires PID gain adjustment**: Ki and Kd scale with dt. Either retune or apply proportional correction (halve dt → halve Ki, double Kd). Update `CONTROL_LOOP_DT` constant.
- Do this after first flight, not before — current gains are tuned for 13ms dt.

---

### 4. Waypoint Navigation

Full autonomous navigation system. See [docs/NAVIGATION_PLAN.md](docs/NAVIGATION_PLAN.md) for the detailed plan.

**Stub crate created** at `crates/elle-nav/` with deps (sguaba, nalgebra). Not in workspace members yet — add when implementation begins.

**Build order summary:**

1. Nav math library (`elle-nav` crate) — coordinate/bearing/distance functions
2. Heading controller — outer loop: heading error -> roll setpoint
3. Altitude hold — outer loop: baro altitude error -> pitch setpoint
4. Fly-to-point (Guided mode) — combine heading + altitude to reach a GPS coordinate
5. Waypoint sequencing — chain multiple points
6. L1 path following — smoother path tracking
7. Loiter / RTL modes
8. Safety layers — geofence, failsafes, stall protection, upset recovery, minimum AGL floor
9. Mission upload protocol — RPC endpoints + TUI commands
10. TECS — total energy control (replaces separate speed + altitude PIDs)

**Safety layers detail (step 8):**
- **Minimum AGL floor**: If AGL (from ground level reference) drops below threshold, override pitch setpoint to safe climb angle + minimum throttle. Requires working PID + ground level datum.
- **Upset recovery**: If bank >60° or pitch < -30°, override setpoints to wings-level + slight climb regardless of pilot input. The PID is the recovery mechanism, not something you disable.
- **PID output rate limiter**: Clamp correction rate-of-change (e.g. max 20°/s) so bad gains produce sluggish-but-safe output instead of oscillation.
- **Oscillation detection**: Count PID output sign reversals; if excessive, reduce authority gradually rather than hard-disable.
- Geofence, stall protection, RTL-on-failsafe.

**Key dependency**: Pitot tube / airspeed sensor for safe autonomous flight in wind.

---

### 5. Pitot Tube / Airspeed Sensor

Required for safe autonomous navigation. Without it, GPS groundspeed is the only speed
reference, which breaks down in wind (headwind climb can stall even within pitch limits).

Enables: stall protection, wind-aware guidance, proper TECS, accurate turn coordination.

Low priority until navigation work reaches the outer control loops (heading/altitude).
