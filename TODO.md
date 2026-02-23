# Elle TODO

## Completed

- [x] RPC autotune trigger (StartAutotune/AbortAutotune endpoints, TUI commands)
- [x] Flash persistence for PID gains (MapStorage on 0x200000-0x20FFFF, boot-time load, auto-save on completion, `savepid` TUI command)
- [x] DShot300 bidirectional motor protocol replacing PWM for engines (PIO1 left engine, PIO2 right engine, LED moved to PIO0 SM2)

## Notes

- `rpc-control` and `defmt-logging` are mutually exclusive (`defmt-rtt` and `rtt-target` both define `_SEGGER_RTT`). Build RPC mode with `--no-default-features --features rpc-control`.

---

## Planned

### 1. DShot Follow-Up: RPM Telemetry & Throttle Rework

DShot300 bidirectional is implemented via `embassy-dshot` crate. Both engines use
`BidirDshotPio` (PIO1 for left, PIO2 for right). Elevon PWM remains on PIO0 SM0/SM1,
LED moved to PIO0 SM2.

#### Current architecture
```
PIO0 SM0 -> PIN_12 (elevon_left)   — PWM
PIO0 SM1 -> PIN_14 (elevon_right)  — PWM
PIO0 SM2 -> PIN_10 (WS2812B LED)   — WS2812
PIO1     -> PIN_11 (engine_left)   — BidirDShot300
PIO2     -> PIN_15 (engine_right)  — BidirDShot300
```

The control path still uses µs internally (1000-1600) and `DshotEngines::set_throttle()`
converts to DShot 0-1999 via `us_to_dshot_throttle()`. This works but is an unnecessary
conversion step.

#### Remaining tasks

**1. RPM telemetry storage and display**
- Add `RPM_CACHE` (Mutex-based, like `MAG_CACHE`/`BARO_CACHE`) in `elle-hardware`
- Switch from `throttle_async()` (fire-and-forget) to `throttle_with_telemetry()` in the main loop
- Store eRPM values in the cache after each DShot send
- Add `GetRpmEndpoint` to RPC ICD + handler
- Add RPM display to TUI dashboard
- Log RPM to ULog (new message type)
- CRSF telemetry RPM frame

**2. Extended DShot telemetry (temperature, voltage, current)**
- Send `ExtendedTelemetryEnable` command sequence during ESC arming
- Use `read_extended_telemetry()` periodically
- Store in a cache, expose via RPC, log to ULog

**3. ~~Throttle range rework (remove µs intermediate representation)~~ DONE**
- Control path now outputs DShot 0-1999 directly
- Removed `ENGINE_IDLE_PULSE_US`, `ENGINE_DSHOT_RANGE` constants
- `THROTTLE_LUT` outputs `u16` DShot values, differential thrust works in DShot space
- `DshotEngines::set_throttle()` takes `u16` directly — no conversion

**4. Closed-loop RPM control (governor mode)**
- Instead of commanding raw DShot values, command target RPM per engine
- Read eRPM telemetry back from bidirectional DShot (already supported by `BidirDshotPio`)
- Per-engine PI loop: target RPM → DShot adjustment
- Compensates for battery voltage sag (same DShot = less RPM as voltage drops)
- More linear thrust response (thrust ∝ RPM²)
- Better differential thrust precision (real RPM guarantee, not open-loop)

**Implementation sketch:**
- Add `MOTOR_POLES` constant to `elle-config` (needed for eRPM → mechanical RPM)
- Switch `set_throttle()` from `throttle_async()` (fire-and-forget) to reading telemetry response
- Per-engine PI controller struct in `elle-hardware` or `elle-control` (target RPM, measured RPM, DShot output)
- Open-loop startup ramp: below a threshold RPM or when telemetry is absent, pass DShot directly until telemetry starts reporting, then switch to closed-loop
- Telemetry dropout handling: hold last DShot value, freeze integrator, don't wind up on CRC errors/timeouts
- eRPM period encoding resolution check: the 3-bit shift + 9-bit base gets coarse at high RPM — verify usable range for your motors
- `RPM_CACHE` for observability (RPC, TUI, ULog, CRSF telemetry)
- Fallback: if telemetry is persistently absent (wiring issue, ESC doesn't support bidir), degrade gracefully to open-loop DShot

---

### 2. Magnetometer Hard-Iron Calibration

#### Problem
The MMC5616WA magnetometer feeds raw counts into the Madgwick AHRS for yaw/heading.
Without hard-iron calibration, permanent magnetic fields on the PCB (traces, battery,
motor wires, screws) offset the readings, causing a constant heading bias.

Currently yaw=0 is arbitrary at startup (gyro-only 6-DOF), then the Madgwick filter
gradually converges toward magnetic north once `has_mag` flips true (~100ms into the
run loop). The convergence speed depends on beta (currently 0.033 — conservative).

#### What hard-iron calibration does
1. Raw mag XYZ plotted over all orientations should form a sphere centered at (0,0,0)
2. PCB magnetic fields shift the sphere off-center by (offset_x, offset_y, offset_z)
3. Calibration finds those offsets, then subtracts them before feeding into AHRS

#### Implementation plan

**1. Calibration data structure**
Replace the BNO055-specific `CalibrationData` in `sequential_flash_manager.rs`:
- Current: `profile_data: [u8; BNO055_CALIB_SIZE]` + `quality: CalibrationLevels`
- New: `mag_offset: [f32; 3]` (12 bytes) + optional `mag_scale: [f32; 3]` (soft-iron)
- Update `BNO055_CALIB_SIZE` -> `MAG_CALIB_SIZE` in `elle-config`
- Update `CalibrationKey` identifier (change from 0x42 to avoid loading stale BNO055 data)

**2. Calibration routine (in `imu.rs`)**
Add a `calibrate_mag()` method to `Imu`:
- Enter calibration mode (could be triggered via RPC `SaveCalibration` command)
- Collect raw mag samples for ~30 seconds while user rotates the board
- Track min/max per axis: `min_x, max_x, min_y, max_y, min_z, max_z`
- Compute offsets: `offset_x = (max_x + min_x) / 2`, same for Y and Z
- Store offsets via `request_save_calibration()` flash API

**3. Apply offsets in run loop**
In `imu.rs` `run()`, mag read section:
```rust
self.last_mag = nalgebra::Vector3::new(
    data.x as f32 - self.mag_offset.x,
    data.y as f32 - self.mag_offset.y,
    data.z as f32 - self.mag_offset.z,
);
```

**4. Load on boot**
In `imu.rs` `initialize()`, after mag init succeeds:
- Call `request_load_calibration()` to read offsets from flash
- If found, populate `self.mag_offset`
- If not found, use zero offsets (uncalibrated, still works but biased)

**5. Wire up RPC commands**
The `SaveCalibration` and `ClearCalibration` RPC commands already exist in the ICD
but are logged-and-ignored. Wire them to:
- `SaveCalibration` -> trigger `calibrate_mag()` routine
- `ClearCalibration` -> erase stored offsets, reset to zero

#### Files to modify
- `crates/elle-config/src/lib.rs` — rename `BNO055_CALIB_SIZE`, add `MAG_CALIB_SIZE`
- `crates/elle-config/src/profile.rs` — update `FlashRequest`/`FlashResponse` types
- `crates/elle-hardware/src/sequential_flash_manager.rs` — new `CalibrationData` format
- `crates/elle-hardware/src/imu.rs` — add `mag_offset` field, `calibrate_mag()`, apply offsets
- `crates/elle-eagle/src/rpc_handlers.rs` — wire `SaveCalibration`/`ClearCalibration`

#### Soft-iron calibration (future, lower priority)
Soft-iron distortion (nearby ferromagnetic material squashes the sphere into an
ellipsoid) requires a 3x3 correction matrix instead of just offsets. More complex
to compute (needs ellipsoid fitting), usually less impactful than hard-iron.
Not needed for initial heading accuracy.

---

### 3. Waypoint Navigation

Full autonomous navigation system. See [docs/NAVIGATION_PLAN.md](docs/NAVIGATION_PLAN.md) for the detailed plan.

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

---

### 5. RPM Telemetry Integration

Bidirectional DShot300 hardware layer is done (`embassy-dshot` BidirDshotPio). The ESCs
respond with GCR-encoded eRPM after each frame. Currently using `throttle_async()`
(fire-and-forget) — switching to reading telemetry enables RPM readback.

See task 1.1 (storage/display) and task 1.4 (closed-loop RPM control) above.

#### What RPM telemetry enables

- **Motor health / failure detection**: Compare expected RPM vs actual. Prop damage = RPM too high, obstruction = RPM too low, ESC desync = RPM drops to zero.
- **Closed-loop RPM control (governor mode)**: Per-engine PI loop for consistent thrust across battery voltage sag and load changes. See task 1.4 for implementation plan.
- **ULog recording**: Log RPM alongside attitude and commands.
- **CRSF telemetry**: Send RPM to radio for OSD display.
