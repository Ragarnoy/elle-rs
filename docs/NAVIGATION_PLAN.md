# Navigation & Waypoint System Plan

## Current State

- Inner attitude PID (pitch/roll to elevons) at 200 Hz (5 ms)
- AHRS heading (Madgwick, magnetometer + gyro), hard-iron and level calibration
- GNSS: SAM-M10Q, UBX-NAV-PVT at 5 Hz over 115200 baud (position, velocity NED, ground
  speed, course, accuracy estimates); NMEA GGA fallback
- Barometric altitude at ~20 Hz (BMP390)
- Modes: Manual, Stabilized, AltitudeHold (currently a 0°/0° **level** hold — no altitude loop yet)
- **Heading hold** (CH5, Stabilized): P controller from heading error to bank setpoint,
  `elle-control/src/heading.rs` — this is the "simpler alternative" heading controller
  of Layer 2 and step 2 of the build order
- Eagle: differential thrust (2 engines); dart: single engine
- `crates/elle-nav` is a workspace member but has no firmware users yet
- No airspeed sensor (pitot tube planned)

---

## Layer 0 — Navigation Math Library

Pure math, no hardware. Can be written and unit-tested on the host.

- **Coordinate conversions**: lat/lon (WGS84) to local NED frame (North/East/Down in meters relative to a reference point). Equirectangular approximation is sufficient under ~10km.
- **Bearing**: Great-circle bearing between two lat/lon points.
- **Distance**: Haversine distance between two points.
- **Cross-track error**: Perpendicular distance from current position to the line between two waypoints.
- **Magnetic declination**: Offset between magnetic heading (AHRS) and true heading (GPS course). Can be a hardcoded constant for a known flying field or a simple lookup table.

**Crate**: `elle-nav` (new, `no_std`, testable on host)

---

## Layer 1 — State Estimation Upgrades

- **Altitude estimator**: Complementary filter blending baro (smooth, drifty) with GPS altitude (noisy, absolute). Output: estimated altitude + climb rate.
- **Position rate**: GNSS runs at 5 Hz (NAV-PVT). Between fixes, dead-reckon using AHRS heading + last known groundspeed for smoother position estimates; the SAM-M10Q can go to 10 Hz if needed.
- **Groundspeed/track**: GPS provides these directly. Apply smoothing filter to reject outlier fixes.
- **Wind estimation** (nice-to-have): Assuming roughly constant airspeed, the difference between heading vector (mag) and GPS track vector gives wind. Helps predict stall risk in turns.

---

## Layer 2 — Outer Control Loops

Two new PIDs sitting outside the existing attitude loop.

### Heading / Course Controller

```
Desired Track -> [L1 or heading PID] -> Bank Angle Setpoint -> [Existing Roll PID] -> Elevons
```

- **L1 navigation controller**: Industry standard for fixed-wing path following. Computes lateral acceleration demand based on cross-track error and closing angle. Converts to bank angle via `bank = atan(a_lateral / g)`.
- **Simpler alternative**: P controller on heading error with cross-track correction term. Less elegant but easier to implement and tune first.
- **Turn coordination**: Bank angle must be appropriate for speed. `bank = atan(v^2 / (r * g))`. Without airspeed, use GPS groundspeed.
- **Bank limiting**: Cap at 30-40 degrees for safety.

### Altitude Controller

```
Target Alt -> [Alt PID] -> Pitch Setpoint -> [Existing Pitch PID] -> Elevons
```

- P or PD controller on altitude error, output clamped to safe pitch range (e.g. +/-15 degrees).
- Use baro altitude for the control loop (smoother, lower latency). GPS for drift correction.
- Rate-limit pitch setpoint changes to prevent jerky maneuvers.

### Speed / Throttle Controller

```
Target Speed -> [Speed PID] -> Throttle %
```

- V1: Fixed cruise throttle with minimum-speed protection (increase throttle if groundspeed drops below threshold).
- V2: PID on GPS groundspeed error.
- V3: Full TECS (Total Energy Control System) coordinating pitch + throttle for total energy management.

---

## Layer 3 — Waypoint Data Model

- **Waypoint struct**: `{ lat: f32, lon: f32, alt_m: f32, wp_type: WaypointType, radius_m: f32, speed: Option<f32> }`
- **WaypointType**: `Flyover` (pass directly over), `Flyby` (begin turning early), `Loiter` (circle), `Land`, `RTL`
- **Mission**: Ordered list of waypoints. Fixed-size array for `no_std` (e.g. `heapless::Vec<Waypoint, 32>`).
- **Home position**: Captured on arm from current GPS fix. RTL target.
- **Storage**: RAM for active mission (uploaded via RPC). Optional: persist to flash in profile region.

---

## Layer 4 — Guidance / Mission Sequencer

The brain that decides what to do each tick.

- **Waypoint sequencing**: Advance to next waypoint when within acceptance radius, or when crossing the bisector plane perpendicular to the path at the waypoint.
- **Segment tracking**: Current segment = line from WP[n-1] to WP[n]. Feed to L1 controller.
- **Altitude profile**: Immediate climb/descend on segment start, or follow a slope (gradual altitude change proportional to distance along segment).
- **Loiter**: Orbit around a point at set radius. Constant bank angle calculated for radius + speed.
- **Mission complete**: Configurable behavior — loiter at last waypoint or RTL.

---

## Layer 5 — Navigation Modes

Extend `ControlMode` beyond Manual/Mixed/Autopilot:

| Mode | Lateral | Vertical | Throttle |
|------|---------|----------|----------|
| Manual | Stick to elevons | Stick to elevons | Stick |
| Stabilized | Stick to attitude setpoint | Stick to attitude setpoint | Stick |
| AltHold | Stick to heading rate | Hold altitude | Cruise |
| Guided | RPC target point | Target altitude | Auto |
| Auto | Follow mission | Follow mission | Auto |
| RTL | Fly to home | Descend to safe alt | Auto |
| Loiter | Circle current pos | Hold altitude | Cruise |

Each mode selects which outer loops are active and where setpoints come from.

---

## Layer 6 — RPC / Mission Protocol

New endpoints for mission management:

- `UploadWaypoint { index, lat, lon, alt, type, radius }` — set one waypoint
- `ClearMission` — wipe all waypoints
- `GetMissionInfo` — count, current index
- `GetWaypoint { index }` — read back a waypoint
- `StartMission` / `PauseMission` / `ResumeMission`
- `SetGuidedTarget { lat, lon, alt }` — fly to a single point (no mission)
- `GetNavStatus` — current WP index, distance to WP, ETA, cross-track error, groundspeed

TUI commands: `wp add 48.123 2.456 150`, `wp list`, `wp clear`, `mission start`, `nav status`, etc.

---

## Layer 7 — Safety Systems

Non-negotiable for autonomous flight.

- **Geofence**: Cylindrical (max radius + max altitude from home). Breach triggers forced RTL.
- **GPS loss failsafe**: Hold last heading + altitude for N seconds. If no fix, RTL via dead reckoning, then loiter/descend.
- **RC loss failsafe**: Already have CRSF failsafe detection. Escalation: RC lost -> RTL -> timeout -> loiter descend.
- **Minimum altitude floor**: Never command below e.g. 30m AGL.
- **Stall protection**: If groundspeed drops below threshold, pitch down + increase throttle regardless of altitude command.
- **Nav loop watchdog**: If guidance produces insane outputs, revert to stabilized mode.

---

## Layer 8 — Future Additions

- **Pitot tube / airspeed sensor**: Biggest safety gain for autonomous nav. Enables real stall protection, wind-aware guidance, proper TECS.
- **Auto-takeoff**: Full throttle, pitch up to climb angle, climb to first waypoint altitude.
- **Auto-landing**: Glide slope, flare, throttle cut. Hardest part of the whole system.
- **Terrain following**: Requires terrain database or downward-facing sensor.
- **MAVLink bridge**: Compatibility with QGroundControl / Mission Planner ground stations.

---

## Suggested Build Order

| Step | Deliverable | What It Enables |
|------|-------------|-----------------|
| 1 | Nav math library (`elle-nav`) | Unit-testable coordinate/bearing/distance functions |
| 2 | ~~Heading controller~~ (done: heading hold) | Fly a commanded heading (yaw -> roll setpoint) |
| 3 | Altitude hold | Outer loop on baro altitude -> pitch setpoint |
| 4 | Fly-to-point (Guided mode) | Combine heading + alt to reach a single GPS coordinate |
| 5 | Waypoint sequencing | Chain multiple points together |
| 6 | L1 path following | Replace simple heading controller for smoother tracking |
| 7 | Loiter / RTL | Special guidance modes |
| 8 | Safety layers | Geofence, failsafes, stall protection |
| 9 | Mission upload protocol | Full RPC endpoints + TUI commands |
| 10 | TECS | Proper energy management replacing separate speed + alt PIDs |

Steps 1-4 get "fly to a GPS point" — already very useful for testing.
Steps 5-8 produce a real autopilot.
Steps 9-10 make it production-quality.

---

## Key Risk

Without an airspeed sensor, GPS groundspeed is a poor substitute in wind. The biggest danger is stall during climbs and turns when flying into a headwind. Pitch limits and minimum groundspeed protection are the only defense until a pitot tube is added.
