# Attitude Estimation

How the attitude the PID flies on is estimated, why it goes wrong in sustained turns,
what was built to measure and fix that, and how a fix gets from a replay to the
aircraft. **Nothing here has flown yet**: the turn compensation is in the firmware but
off, and every number below comes from simulation until TEST_PLAN Part 7 replaces it
with flight data.

## The problem

The AHRS (Madgwick, β = 0.033, 1 kHz) integrates the gyro and slowly pulls the tilt
towards "the accelerometer's direction is down". That is right when the aircraft is
not accelerating. In a coordinated turn the accelerometer reads gravity plus the
centripetal acceleration, which together point through the belly: the filter is pulled
towards wings level at up to 2β rad/s (~3.8°/s). In simulation a sustained 15° turn
reads ~5° of bank after 15 s, and a 30° turn held for 15 s is ~8° off in roll and ~11° in
pitch (the mag pulls the tilt once roll is wrong).

If that holds on the aircraft, the controller sees too little bank in any long turn and
adds more, and the navigation loiter (a constant turn) flies on a wrong attitude.

## Pipeline

`elle_control::attitude::AttitudePipeline`, run by the IMU task on Core 1 and, the same
code, by every host replay:

1. subtract the boot gyro bias; level-cal step (driver);
2. rotate gyro and accel by the level-cal mount;
3. rate low-pass for the PID (30 Hz Butterworth);
4. **turn compensation** (`AHRS_TURN_COMP`, off by default) and **accel gate**
   (`AHRS_ACCEL_GATE_G`, off by default);
5. Madgwick update (uf-ahrs): 9-DOF with the mag, gyro only while gated;
6. Euler angles, roll negated for this board.

**Filter frame.** From the hardware-verified conventions (pitch positive nose up, roll
positive right wing down after step 6, +1 g on z when level), the filter's body axes are
forward −x, right +y, up +z, and with the mag its earth frame is north-west-up.
`attitude::forward()` holds the forward axis. This is derived, not measured: a wrong
sign would show in a replay (and in the vehicle test, 7.2) as more error, not less.

## Settings (`elle-config`)

| Setting | Default | Effect |
|---|---|---|
| `AHRS_TURN_COMP` | `Off` | `Centripetal`: subtract gyro × GNSS ground speed along the nose. `GnssAccel`: subtract the GNSS velocity change between fixes, rotated into the body with the current attitude. |
| `AHRS_TURN_COMP_MIN_SPEED_MS` | 6 m/s | Below this ground speed: no compensation (on the ground, walking). |
| `AHRS_TURN_COMP_MAX_AGE_MS` | 300 ms | Older GNSS solutions are not used. The GNSS acceleration also needs two NAV-PVT fixes at most 500 ms apart. |
| `AHRS_TURN_COMP_RAMP_S` | 1 s | Compensation fades in and out, never steps. |
| `AHRS_ACCEL_GATE_G` | `None` | Skip the accel (gyro-only update) while its 5 Hz low-passed magnitude is further than this from 1 g. |

With `Off` and no gate the pipeline is the plain Madgwick update, bit for bit (a test
checks it). With a mode on, the IMU task hands the pipeline each new GNSS solution from
the GNSS cache (`gnss` builds only; without GNSS there is nothing to compensate with).
Cost: 1.7 kB flash, 136 B RAM; Core 1 time per sample to be measured (7.5).

**Why two modes.** Ground speed is not airspeed. In wind, `Centripetal` is wrong by
turn rate × wind speed (0.2 rad/s × 6 m/s ≈ 1.2 m/s², several degrees, varying around
a circle). `GnssAccel` measures the actual acceleration, so wind does not matter, but
it is 5 Hz, delayed by the receiver (~100 ms), and rotated with the heading, which is
magnetic (declination is not applied: a few percent of the correction).

## Raw capture and exact replay

`imu-raw-log` builds record every 1 kHz sample as the IMU's 20-bit integers (`imu_raw`),
the mag vector as fed (`imu_raw_mag`), each GNSS fix handed to the pipeline
(`imu_raw_fix`), and a context (`imu_raw_ctx`: quaternion, bias, mount, turn
compensation state, the build's modes) every second and on change. See
[`crates/elle-ulog/README.md`](../crates/elle-ulog/README.md).

`elle_control::imu_raw::Replayer` reproduces the firmware's angles **bit for bit** from
any context on, in every mode (tests: `crates/elle-control/tests/imu_raw.rs`). Limits:

- after a gap, exact again only from the next context (≤ 1 s);
- the filtered rates are not in the context: they converge within tens of ms;
- a lost `imu_raw_fix` record changes the GNSS aid, so the replay may drift until the
  next context resets the compensation state.

`elle-replay LOG` checks it on every log: each `attitude_data` must be reproduced
exactly; any mismatch fails the run, and nothing else it reports is trusted.

## Comparing filters: `elle-replay --compare`

Runs, on the same inputs as the firmware (logged bias, mount, mag, GNSS fixes), seeded
from the firmware's state: uf-ahrs Madgwick, Mahony and VQF, each without and with turn
compensation (`-cc` centripetal, `-ce` GNSS acceleration) and accel gating (`-gated`).
The compensation and gate are the firmware's own code.

**The reference.** Differences between filters say nothing about which is right; the
reference does. In straight, level, unaccelerated flight the accel's direction is
gravity, so it gives pitch and roll directly. From such a stretch (≥ 2 s before, and
0.5 s after, so it is never taken as a manoeuvre starts) the reference integrates the
gyro alone for up to 20 s. At the next straight stretch the drift it gathered is known
and spread back over the run (exact for a constant residual bias). Scored samples:
reference bank > 10° (flight), or turn rate > 6°/s with `--score-by-rate` (a vehicle,
which turns without banking).

In simulation the reference is within 0.03° of truth in turns, 0.12° with 2 m/s² of
vibration and 0.03°/s of residual gyro bias, and ranks the filters exactly as truth
does. It needs straight-and-level stretches between manoeuvres; its coverage is printed.

## Simulated results

`elle-replay --simulate out.ulg --compare [--wind-east 6] [--vibration 2 --gyro-bias-dps 0.03]`
flies turns both ways at 15°, 30° and 45°, a pull-up and two circles each way at 30°
(`sim::standard_profile`), at 15 m/s. Roll RMS error in turns against the truth
(pitch RMS in brackets), degrees:

| Filter | Calm | Wind 6 m/s | Vibration + bias | Both |
|---|---|---|---|---|
| firmware (Madgwick, no compensation) | 7.2 (7.4) | 7.2 (7.4) | 5.6 (7.4) | 5.6 (7.4) |
| madgwick-gated | 7.1 (7.1) | 7.1 (7.1) | 5.5 (7.1) | 5.5 (7.1) |
| **madgwick-cc** (`Centripetal`) | **0.1** (0.1) | 6.7 (3.2) | **0.1** (0.1) | 4.8 (4.6) |
| **madgwick-ce** (`GnssAccel`) | 2.0 (2.5) | **2.0** (2.5) | 1.6 (1.2) | **1.6** (1.2) |
| vqf, no compensation | 14.2 (20.7) | 14.2 (20.7) | 19.3 (18.1) | 19.3 (18.1) |
| vqf-cc | 0.1 (0.1) | 3.5 (4.6) | 0.1 (0.1) | 4.9 (6.3) |
| vqf-ce | 2.6 (1.7) | 2.6 (1.7) | 4.0 (3.0) | 4.0 (3.0) |

The vehicle ground test (7.2), simulated at 7 m/s round 25°/s curves: firmware > 5° RMS
off level, `-cc` < 1°, `-ce` < 3° (`tests/sim.rs`).

What the simulation says, to be confirmed by flight data:

- Compensation, not the filter, is what matters: uncompensated VQF and Mahony are
  worse than the current Madgwick; gating alone barely helps.
- In calm air `Centripetal` is near perfect; in 6 m/s of wind it keeps only its pitch
  gain. `GnssAccel` gives ~2° whatever the wind.
- What the simulation leaves out: GNSS latency and noise are modelled simply; there is
  no sideslip, no angle of attack, no turbulence, and speed is constant.

## From data to the aircraft

1. **Host** (CI): everything above is tested on every PR.
2. **Vehicle test** (TEST_PLAN 7.2): real sensors, no flight, normal build with
   `imu-raw-log`. Confirms the sign and size of both corrections.
3. **Data flight** (Part 0.5, 7.3) with `imu-raw-log`, compensation off.
4. **Decision** (7.4): criteria below, on at least two flights, one of them windy.
5. **Enable** (7.5–7.7): set `AHRS_TURN_COMP`, bench regression, vehicle test with the
   firmware itself compensating, cautious flight, then autotune.

**Decision criteria** (7.4). Pick the mode with the lowest roll RMS against the reference
in turns, if on every data flight it:

- cuts roll RMS in turns by at least half and by at least 2°, both directions;
- does not make pitch RMS in turns worse by more than 0.5°;
- keeps the replay exact and the reference coverage ≥ 50 % of turn time.

Otherwise keep `Off`, and look at what the data shows (latency, sideslip, a wrong
axis) before changing anything.

## Open

- **Smooth accel weighting** (β scaled with the accel error rather than on/off) needs
  `set_params` in uf-ahrs: [jettify/uf-ahrs#48](https://github.com/jettify/uf-ahrs/pull/48),
  pending.
- **Airspeed sensor**: would make `Centripetal` right in wind.
- **Magnetic declination** for `GnssAccel`.
- **VQF in the firmware**: its mag handling (heading-only, disturbance rejection) is
  still attractive, but uncompensated it is worse in turns; revisit with flight data.
