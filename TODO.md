# Elle TODO

## Completed

- [x] RPC autotune trigger (StartAutotune/AbortAutotune endpoints, TUI commands)
- [x] Flash persistence for PID gains (MapStorage on 0x200000-0x20FFFF, boot-time load, auto-save on completion, `savepid` TUI command)

## Notes

- `rpc-control` and `defmt-logging` are mutually exclusive (`defmt-rtt` and `rtt-target` both define `_SEGGER_RTT`). Build RPC mode with `--no-default-features --features rpc-control`.

---

## Planned

### 1. DShot Motor Protocol (replacing PWM for engines) — BLOCKING

Hardware is being upgraded to DShot ESCs. PWM engines will no longer work.
Steps 1-4 (unidirectional DShot TX) are required to fly again.

#### Current motor output architecture
```
PIO0 SM0 -> PIN_12 (elevon_left)   — PWM, stays PWM
PIO0 SM1 -> PIN_14 (elevon_right)  — PWM, stays PWM
PIO0 SM2 -> PIN_11 (engine_left)   — PWM -> DShot
PIO0 SM3 -> PIN_15 (engine_right)  — PWM -> DShot
```

`set_engines(left_us, right_us)` takes pulse widths in us (1000-1600).
Throttle pipeline: RC 0-2047 -> `throttle_curve_lut` -> us -> `differential_thrust_lut` -> us pair.

#### DShot protocol
Digital protocol, each frame is 16 bits:
- 11 bits: throttle value (0 = disarm, 48-2047 = throttle range)
- 1 bit: telemetry request
- 4 bits: CRC

DShot300 = 53.3us/frame, DShot600 = 26.7us/frame. Both fit within the 13ms control loop.

Key difference: DShot sends a **value** (0-2047) not a pulse width. The throttle LUTs
would output 48-2047 directly, eliminating the intermediate us representation.

#### PIO resource plan

**Recommended: move engines to PIO1** (currently unused).
- PIO0 SM0/SM1: elevon PWM (unchanged)
- PIO1 SM0: engine_left DShot (PIN_11)
- PIO1 SM1: engine_right DShot (PIN_15)

Keeps motor protocol isolated from servo PWM. PIO1 has its own 32-slot instruction
memory — important if bidir DShot needs ~25 instructions. Frees PIO0 SM2/SM3.

#### Interface changes

Current:
```rust
pub fn set_engines(&mut self, left_us: u32, right_us: u32)
```

DShot:
```rust
pub fn set_engines(&mut self, left: u16, right: u16)  // 0-2047 DShot throttle value
```

Cleaner approach — `MotorOutput` trait with compile-time selection:
```rust
pub trait MotorOutput {
    fn set_engines(&mut self, left: u16, right: u16);
    async fn arm_sequence(&mut self);
}
```

`PwmMotors` and `DshotMotors` both implement it. Feature-gated, zero dynamic dispatch.
`PwmOutputs` splits into `ServoOutputs` (elevons only) + `impl MotorOutput`.

#### ESC init with DShot
Simpler than PWM: send throttle value `0` for ~1 second. No min/idle/min dance.

Special DShot commands (values 0-47 are reserved):
- 0 = disarm / motor stop
- 1-5 = beep patterns
- 6 = ESC info request
- 7/8 = spin direction
- 12 = save settings

#### Implementation order

**Must-have (to fly again):**

1. Normal DShot TX — PIO1 program, get motor control working digitally
2. Refactor PwmOutputs — split into `ServoOutputs` + `DshotMotors`
3. Update throttle LUTs — output 48-2047 instead of 1000-1600us
4. ESC init — simplify to "send 0 for 1 second"

**Nice-to-have (later):**

5. Bidir DShot RX — add RPM reception and GCR decoding
6. RPM signal — wire into shared state (`RPM_SIGNAL`)
7. RPM logging — ULog message + CRSF telemetry frame

#### Notes
- If the DShot crate doesn't materialize, the PIO program is small (~12 instructions
  for TX, ~25 for bidir). Writing it with `pio_proc::pio_asm!` is feasible.
- `ENGINE_RIGHT_OFFSET_US` trim goes away with DShot (digital protocol, no analog drift).
  If motors still need individual trim, apply it as a DShot value offset instead.

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

### 5. Bidirectional DShot (eRPM telemetry)

Depends on DShot TX (#1) being done first. Not needed to fly — purely observability and future closed-loop thrust control.

After the FC sends a 16-bit frame, the ESC responds with a 21-bit GCR-encoded eRPM
frame on the **same wire**. PIO handles both TX and RX by flipping pin direction
mid-program.

Timing per motor: ~53us TX + ~30us response + ~10us gap = 93us.
Two motors sequentially: ~186us. Well within 13ms loop.

#### What RPM telemetry enables

- **Motor health / failure detection**: Compare expected RPM vs actual. Prop damage = RPM too high, obstruction = RPM too low, ESC desync = RPM drops to zero.
- **Thrust linearization**: Thrust is proportional to RPM squared. Close the loop for linear throttle response across battery voltage sag.
- **Governor mode**: Hold constant RPM regardless of load/maneuver.
- **ULog recording**: Log RPM alongside attitude and commands.
- **CRSF telemetry**: Send RPM to radio for OSD display.
