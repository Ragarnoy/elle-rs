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

## Notes

- `rpc-control` and `defmt-logging` are mutually exclusive (`defmt-rtt` and `rtt-target` both define `_SEGGER_RTT`). Build RPC mode with `--no-default-features --features rpc-control`.

---

## Planned

### 1. DShot Follow-Up: Remaining Items

DShot300 bidirectional is fully implemented. RPM telemetry, governor mode, and throttle
rework are done. Remaining items are incremental.

#### Architecture
```
PIO0 SM0 -> PIN_12 (elevon_left)   — PWM
PIO0 SM1 -> PIN_14 (elevon_right)  — PWM
PIO0 SM2 -> PIN_10 (WS2812B LED)   — WS2812
PIO1     -> PIN_11 (engine_left)   — BidirDShot300
PIO2     -> PIN_15 (engine_right)  — BidirDShot300
```

#### Remaining tasks

**~~1. Extended DShot telemetry (temperature, voltage, current)~~ DONE**
- `ExtendedTelemetryEnable` sent 6x during ESC arming
- `read_extended_telemetry()` replaces `throttle_with_telemetry()` in 1kHz loop
- EDT frames (temp/voltage/current) stored in `EngineReading`, exposed via RPC, logged to ULog
- Best-effort: if ESC doesn't support EDT, plain eRPM frames still work, EDT fields stay 0

**~~2. CRSF telemetry battery frame~~ DONE**
- Battery Sensor frame (0x08) sends EDT voltage/current to radio OSD
- 5th slot in 50Hz round-robin (~10Hz update), reads `ENGINE_CACHE` for both engines
- Voltage: max of both engines (same battery), current: sum of both engines

**3. Motor health / failure detection (anomaly alerting on governor state)**
- Governor already compares target vs actual RPM — this is a thin alerting layer on top
- Detect uncorrectable conditions: DShot saturated at max but RPM still low (obstruction/dying motor), DShot at minimum but RPM overshooting (prop damage/blade loss), RPM drops to zero while DShot > 0 (ESC desync), sustained integrator windup
- Event codes + optional `health: u8` field in `EngineUnit` ICD for RPC/ULog visibility
- ~half day: state machine in `dshot_task`, event codes 120+, ICD/TUI/ULog updates

---

### 2. Field Readiness

Incremental improvements for safe real-world flight. Grouped by theme.

#### Safety

**1. Pre-arm checks (~1h)**
- Block arming if: IMU not calibrated, EDT battery voltage below threshold, Core1 unhealthy, autotune in progress
- Return specific error code per failed check (RPC `ArmEndpoint` already returns `error_code`)
- Apply to both RC throttle-based arming and RPC arm command

**2. Low battery voltage failsafe (~1h)**
- Configurable voltage threshold in `elle-config` (per-cell or absolute)
- Warning state (LED + event code) at first threshold, failsafe trigger at critical threshold
- Uses EDT voltage from `ENGINE_CACHE` — no new hardware needed

**~~3. Failsafe improvements~~ DONE**
- `RcLinkState` enum (Ok/Warning/Lost) with transition-based state machine in `check_failsafe()`
- Event codes 13 (RC warning), 14 (RC signal lost), 15 (RC restored) — fire only on transitions
- `rc_age_ms` field in `StatusResp`, `FlightState`, ULog `system_status` message
- `signal_restored()` called on recovery (clears failsafe, does not re-arm)
- LED warning: FastBlink(orange) when armed+warning, RapidFlash(orange) on lost
- TUI displays RC age with color coding (green <100ms, yellow <200ms, red >=200ms)

#### Observability

**4. Battery mAh consumption tracking (~1.5h)**
- Integrate EDT current over time in `dshot_task` (already runs at 1kHz)
- Fill the hardcoded-zero CRSF battery capacity/remaining% fields
- Expose consumed mAh via RPC (extend `EngineResp` or new endpoint)
- Battery capacity constant in `elle-config`

**5. Flight timer + boot counter (~1h)**
- Armed duration since boot in `StatusResp` (use AON timer)
- Persistent boot counter in flash (MapStorage key=3)
- Flight timer in ULog status message

**6. LED state mapping (~30min)**
- Autotune active → distinct pattern (e.g., FastBlink/Yellow)
- ULog recording → distinct pattern (e.g., SlowBlink/Red)
- Low battery warning → pattern overlay
- Currently only armed/failsafe/calibration have LED feedback

#### Usability

**7. Servo trim via RPC + flash persistence (~1.5h)**
- `SetTrimReq { left_us: i16, right_us: i16 }` endpoint with range validation
- Store in flash (MapStorage key=4), auto-load on boot (same pattern as PID gains)
- TUI commands: `trim left 10`, `trim right -5`
- Eliminates recompile for trim adjustment

**8. Expo curves on pitch/roll/yaw (~1.5h)**
- Compile-time LUT generation for common expo values (0%, 20%, 40%)
- Configurable per-axis via `elle-config` constant or runtime RPC
- Significantly improves stick feel — linear input is too aggressive near center

**9. Wire RPC SetControlMode handler (~20min)**
- Endpoint exists in ICD but handler is not implemented
- Trivial: send `RpcCommand::SetMode` through command channel

---

### 3. Waypoint Navigation

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
8. Safety layers — geofence, failsafes, stall protection
9. Mission upload protocol — RPC endpoints + TUI commands
10. TECS — total energy control (replaces separate speed + altitude PIDs)

**Key dependency**: Pitot tube / airspeed sensor for safe autonomous flight in wind.

---

### 4. Pitot Tube / Airspeed Sensor

Required for safe autonomous navigation. Without it, GPS groundspeed is the only speed
reference, which breaks down in wind (headwind climb can stall even within pitch limits).

Enables: stall protection, wind-aware guidance, proper TECS, accurate turn coordination.

Low priority until navigation work reaches the outer control loops (heading/altitude).
