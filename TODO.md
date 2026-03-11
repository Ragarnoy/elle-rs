# Elle TODO

## Completed

- [x] RPC autotune trigger (StartAutotune/AbortAutotune endpoints, TUI commands)
- [x] Flash persistence for PID gains (MapStorage on 0x200000-0x20FFFF, boot-time load, auto-save on completion, `savepid` TUI command)
- [x] DShot300 bidirectional motor protocol replacing PWM for engines (PIO1 left engine, PIO2 right engine, LED moved to PIO0 SM2)
- [x] RPM telemetry storage and display (EDT bidir reading, `ENGINE_CACHE`, `GetEngineEndpoint` RPC, TUI 5Hz poll, `direct engine`, ULog `engine_data` at 77Hz, per-engine auto-fallback)
- [x] Throttle range rework (removed µs intermediate, control path outputs DShot 0-1999 directly)
- [x] Governor mode: closed-loop RPM control via per-engine PI in 1kHz DShot task (feedforward + anti-windup, telemetry-loss fallback, target eRPM observability in ICD/ULog/TUI)
- [x] Extended DShot telemetry (EDT): ESC temperature, voltage, current via `read_extended_telemetry()`, best-effort (0 if unsupported)

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

**2. CRSF telemetry RPM frame**
- Send RPM to radio for OSD display (not yet implemented)

**3. Motor health / failure detection**
- Compare expected RPM (governor target) vs actual
- Prop damage = RPM too high for given DShot, obstruction = RPM too low, ESC desync = RPM drops to zero

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
