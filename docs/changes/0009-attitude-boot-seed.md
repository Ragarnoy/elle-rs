# 0009: Seed the attitude at boot, and refuse arming until it has settled

| | |
|-|-|
| Status | Draft |
| Airframes | both |
| Date | 2026-11-14 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules), 0.x): a fix plus one arming refusal and one event; no RPC, flash or ULog layout change |
| Surfaces | operator behaviour (arming refused while the attitude settles), event codes (19 added) |

## Motivation

For about the first minute after every boot the attitude is wrong by tens of degrees
while the aircraft sits still. Arming is allowed after the gyro bias is measured
(~2 s), so Stabilized can be armed, and launched, holding a tilt that is not there.

Evidence, eagle, every log checked, aircraft still on the bench (1 s means of
`attitude_data`, from the first record):

| Log | Build | Pitch after boot | Back within 1.5° |
|-----|-------|------------------|------------------|
| LOG_0082 (round-A reference flight) | v0.3.0 | 0 → **+39°** at 13 s | ~45 s |
| LOG_0094 | round-a 2993f3b | 0 → **−44°** at 10 s | ~45 s |
| LOG_0095 | round-a 7be55fb | 0 → **−45°** at 10 s | ~45 s |
| LOG_0106 | round-a 642b9ae | 0 → **+41°** at 11 s | ~40 s |
| LOG_0107 | round-a 5edd559 | 0 → **−42°** at 13 s | ~45 s |

Its size varies from boot to boot on the same bench at the same heading (final yaw
−155 to −166° every time), and so does the raw mag reading; the heading always
swings from ~0° to there:

| Log | Boot | Worst pitch / roll, first 60 s | Raw mag z, 1–10 s |
|-----|------|-------------------------------|-------------------|
| LOG_0094, LOG_0095 | flash reset | 44° / 12° | +55k, +57k |
| LOG_0103–0105 | flash reset | 17–22° / 9° | +99k to +100k |
| LOG_0106 | flash reset | 41° / 10° | +135k |
| LOG_0109 | flash reset | 1.4° / 3.0° | +104k |
| LOG_0110 | power cycle | 2.6° / 3.2° | +104k |

LOG_0109/0110: a skid was off the table, so roll sat at ≈ −2.9° (−1.4° otherwise);
their roll figure is that static tilt, not the transient.

So the leak depends on the field the filter sees, not only on the heading error,
and a pilot cannot predict it.

The raw field swinging between sessions had its own cause, found on 2026-11-14: a
magnet in the eagle's hatch, 1–2 cm from the MMC5616WA, gave ~20 G at the sensor
(16–24 G in every eagle log since LOG_0054; a SET/RESET measurement showed it is a
real field, sensor offset ~0.4 G). It sat differently each time the hatch closed,
so the stored hard-iron offsets never matched. It is removed and the mag
recalibrated. **The transient stays without it:** LOG_0119 (magnet out, mag
recalibrated, `imu-raw-log`, still for 87 s) holds pitch −17 to −18° for ~40 s and
is back within 1° at ~54 s while the heading swings −4° → −164°.

**And with the magnetometer axes fixed** (proposal 0010, LOG_0126, 2026-11-14, eagle,
clean field, fresh calibration): a handled boot rather than a still one, so it is
not in the replay table below. Yaw read 2.9° at 4 s with the nose NE (134° expected),
and the tilt error against the accelerometer was 12–19° at every rest in the first
~30 s. The heading error fell −131° → −68° → −54° → −34° → −15° over 4–32 s while
the aircraft was turned through N, W, S and E, and was −1° to −3° once settled.

Roll moves too (up to ~17°). Once settled the attitude is good: LOG_0107 then holds
pitch 0.0–0.1°, roll −1.4° for minutes.

In Stabilized the controller acts on it even disarmed: LOG_0107 switched to
Stabilized ~22 s after boot and both elevons went to their stops (1999 / 1001 µs at
25 s) against a phantom −34° nose-down, easing back as the attitude settled (1635 /
1385 µs at 50 s). Armed, that is a full nose-up command at launch. (The 2026-11-14
trim sessions, LOG_0096–0104, were in Manual: surfaces at the trim values
throughout, so those trims are unaffected.)

**Cause.** The heading converges at the same time: in LOG_0107 yaw goes from −7° to
−155° over 50 s while pitch makes its excursion, and settles when pitch does; LOG_0089
the same (−6° → −170°). The raw mag counts stay steady throughout (no offset or field
change mid-transient), so the filter, not the sensor, is moving. The Madgwick filter starts from the identity quaternion, i.e. it assumes
the airframe is level **and pointing north**. With a magnetometer reading (9-DOF) it
turns towards the real heading at the rate `AHRS_BETA` (0.033) allows, and the
gradient step leaks part of that heading correction into pitch and roll until the
heading has converged. A host run of `AttitudePipeline::with_board(board_rotation())`
(eagle), held perfectly level and still with a 63°-inclination field, for 60 s:

| Real heading | 9-DOF worst pitch/roll | still off at 55 s | 6-DOF (no mag) |
|--------------|------------------------|-------------------|----------------|
| north | 0° | 0° | 0° |
| east / west | 11.6° | 0° (settled ~30 s) | 0° |
| south | 18° | −14° pitch | 0° |

The size depends on the field's dip as well as the heading: a static host run
(`elle-replay --example seed_static`, level, constant field) peaks at 44° from
identity with a 20° dip and the field 164° from north, 20° at a 60° dip. The 20 G
hatch magnet gave the eagle a field of its own dip, hence the spread between
boots in the tables above. **Replay confirms a seed fixes the real case** (see
Verification: worst tilt 19.3° as logged, 0.7° seeded, on LOG_0119). The
board orientation is not the cause: the heading error at power-up is, so **the dart
is affected the same way** (its logs are to be checked, Verification).

Not new: LOG_0082 (v0.3.0, flown) shows it. A pilot who powers up and waits before
arming never sees it, which is likely why it has not bitten yet.

## Behaviour delta

| Situation | Before | After |
|-----------|--------|-------|
| Boot, still | Attitude starts at 0/0/north and wanders up to ~45° in pitch over ~45 s | Attitude seeded from accel and mag once the gyro bias is done (~2 s after boot): correct from then on |
| Boot, no mag (I2C failed, event 43/48) | 6-DOF from level, converges on tilt only | Seeded from accel (tilt); heading stays at the mag-less default, as now |
| Arm (any path) while the AHRS tilt and the accel tilt disagree by > `ARM_ATTITUDE_MAX_ERR_DEG` | Arms | Refused, event 19, LED and CRSF as for the other arming refusals |
| Arm with the aircraft moving (handling) | Arms | Arms if the tilts agree (the check uses a filtered accel and only runs disarmed); refused otherwise until it is held still |
| Armed | — | Unchanged: the check never runs armed and never disarms |

OPERATIONS.md "Arming and disarming": one more refusal reason and its event.
STATE_DIAGRAMS.md arming diagram: the guard "attitude settled" on Disarmed → Armed.

## Design

Two parts. Part A removes the cause; Part B is the safety net that also catches any
other divergence (a bad seed, a mag disturbance at boot, a Core 1 fault).

### A. Seed (`elle-control/src/attitude.rs`, `elle-hardware/src/imu/driver.rs`)

- New pure function `elle_control::attitude::seed(accel, mag) ->
  Option<UnitQuaternion<f32>>` (written, on this branch), in the airframe frame the
  AHRS uses (after `mount`): roll and pitch from the accel vector, heading from the
  mag vector projected on the horizontal plane (TRIAD), body to uf-ahrs' earth frame
  (north, west, up). `None` for a zero or parallel pair. Still to add: a tilt-only
  variant for the no-mag case (heading 0). Host check so far: with constant inputs
  at dips −60…+75° and three headings, the seed is the filter's fixed point (no
  drift over 120 s), and an identity start converges to the seed's heading.
- The driver calls `pipeline.set_quat(seed(…))` once, when the gyro bias estimate
  completes (the aircraft has just been still for a second, so the averaged accel is
  gravity), using the bias estimator's mean accel and the latest `MAG_FIELD` with the
  flash mag offsets already applied. **They are not applied from the first sample:**
  boot sends them over `MAG_CALIBRATION_SIGNAL` once it has read flash, and in
  LOG_0113 and LOG_0119 the first ~88 fused samples (3 mag readings) still carry
  the raw field. With the 20 G magnet a seed from that first reading came out 150°
  off in heading and the seeded run drifted 21° (replay). So the seed waits for
  both: the bias done **and** a mag reading taken after the offsets arrived (or
  after boot has said none are stored). It re-seeds, disarmed and still, whenever the mag offsets or the level
  mount change (mag cal, level cal, their clears), since each of those otherwise
  recreates the transient.
  `imu-raw-log` already records the AHRS quaternion in `imu_raw_ctx` on change, so a
  replay picks the seed up from the log and stays exact (to check: the seed must
  land in a ctx record before the next sample).
- Nothing changes after the seed: same filter, same `AHRS_BETA`, same 1 kHz step.

### B. Arming guard (`elle-control/src/arming.rs`, `elle-system/src/system.rs`)

- New pure check `attitude_settled(ahrs_pitch, ahrs_roll, accel_lp) -> bool`: the
  tilt implied by a low-passed accel (time constant `ARM_ATTITUDE_ACCEL_TAU_S`, 0.5 s)
  against the AHRS pitch and roll, each within `ARM_ATTITUDE_MAX_ERR_DEG` (5°). The
  accel tilt is only meaningful when still, so the check refuses rather than passes
  when the accel magnitude is off 1 g by more than 10 % (being handled).
- **Latched once per boot.** The check gates arming only until it has passed once
  since boot; from then on it never refuses. A re-arm after a failsafe or a kill
  switch, which can happen in the air (round-A B6: a fresh gesture re-arms after RC
  restore), is therefore never blocked: in a turn or glide the accel tilt differs
  from the AHRS tilt and |a| is not 1 g, so an unlatched check would leave the pilot
  without engines.
- `FlightController::arm()` (every arm path: gesture, RPC `Arm`, `rpc-rc`) refuses
  with event 19 while the latch is not set, the same way as
  `EVT_ARM_REFUSED_THROTTLE`. Once armed the check is not evaluated.
- Core 1 publishes the low-passed accel with the attitude (`AttitudeData` gains
  `accel_tilt_pitch`, `accel_tilt_roll`, f32, computed on Core 1 at 1 kHz: a few
  multiplies). Core 0 compares at arm time only.

### Constants (`elle-config`)

`ARM_ATTITUDE_MAX_ERR_DEG` 5.0, `ARM_ATTITUDE_ACCEL_TAU_S` 0.5,
`ARM_ATTITUDE_ACCEL_G_TOL` 0.10; asserts: positive, error < the pitch setpoint limit.

### Timing

Seed: once, a few hundred float ops on Core 1. Guard: one low-pass per sample on
Core 1 (~10 ops), one comparison per arm attempt on Core 0. Nothing per tick.

## Interfaces

- **Events:** 19 `EVT_ARM_REFUSED_ATTITUDE` ("Arming refused: attitude not settled"),
  host label, OPERATIONS.md row. Optionally an info event when the seed is applied
  (in the IMU block, 40–49 is full: next free code there to be chosen).
- **RPC:** none. `GetAttitude` is unchanged; the accel tilt is internal.
- **Flash:** none.
- **ULog:** none required. (`attitude_data` could carry the accel tilt for
  diagnosis; left out to keep the layout.)
- **CRSF / LED:** the existing arming-refused pattern.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| Seed from a bad mag (disturbed field at boot) | Heading wrong; tilt leak as today, but from a smaller error | Part B refuses arming until the tilt agrees; the seed is never worse than identity in tilt (tilt comes from the accel) |
| Seed taken while moving | Wrong tilt seed | Taken only when the bias estimator has accepted a still second; otherwise no seed (behaviour as today) and Part B holds arming |
| Guard refuses a legitimate arm | Pilot cannot arm | 5° is well above the settled error (≤1.5° in the logs); refusal is visible (event, LED); holding the aircraft still clears it |
| Guard false pass | Arms with a wrong attitude | Only possible if accel tilt and AHRS agree on a wrong value, i.e. the accel itself is wrong: no worse than today |
| Hand launch: arm, then throw | — | The check runs at arm time only, never after: no effect on the launch |
| Re-arm in the air after failsafe or kill switch | Pilot cannot restart engines | Latched once per boot (Design B): once it has passed on the ground it never refuses again |
| **Reboot in the air** (watchdog, brown-out) | The new boot has no latch and no still second: no seed, and the guard would refuse every arm for the rest of the flight | **Open, the user's decision:** (a) skip the guard after a watchdog reset (`ResetReason::TimedOut`), (b) let it time out (e.g. pass after `ARM_ATTITUDE_LATCH_TIMEOUT_S`), or (c) accept it (an in-air reboot today also restarts with a wrong attitude). The seed itself is harmless here: without a still second it does not run |
| Sensor loss, RC loss, Core 1 stall, flash write | — | Seed: only at boot (and on cal changes, disarmed). Guard: disarmed only, latched |

## Verification

- **Host tests:** `seed` at headings 0/90/180/270 and tilts ±20°: seeded pipeline
  error < 0.5° over 60 s still (the table above as a regression test, with and
  without the seed); `attitude_settled` true when still and level, false at a 10°
  disagreement, false while the accel magnitude is off 1 g.
- **Replay (done 2026-11-14, `cargo run -p elle-replay --example seed -- LOG`;
  `SEED_AT_S` starts every run that long after the first synced sample):** each
  `imu-raw-log` boot capture of a still aircraft, run three ways on the logged
  inputs: as logged (exact replay), restarted at identity (what a boot does) and
  restarted at `seed`. Worst tilt error against the settled attitude:

  | Log | Field | Logged | Identity | Seeded at 5 s | Seeded at 0.1 s |
  |-----|-------|--------|----------|---------------|-----------------|
  | LOG_0109 | magnet in | 1.6° | 3.1° | 0.2° | — |
  | LOG_0113–0115 | magnet in, shifted | 6.6–7.4° | 9.3° | 0.1–0.2° | 0.2° (0113) |
  | LOG_0119 | magnet out, recalibrated | 19.3° (> 1° until 47 s) | 19.4° | 0.7° (never > 1°) | 1.3° (> 1° until 3.2 s) |

  The identity run reproduces the logged transient, so it stands in for a boot.
  Seeded at the very first sample (before the offsets) LOG_0113–0115 got worse
  (21°), which is the ordering requirement in Design A. LOG_0110 is 2.9° in all
  three runs at 252 s (handling late in the log, not the boot).
  Every log in this table predates proposal 0010: the mag is fused 90° off its true
  axes. LOG_0109 and 0113–0115 also carry the hatch magnet, and LOG_0119's corrected
  field reads 0.17 G, so its stored offsets did not fit the magnet-free board. For a
  still, level boot that doesn't change the result: the filter sees one constant
  vector, whatever its mapping, and the seed is its fixed point. For a tilted boot
  it does: the mis-mapped field is inconsistent with gravity there (0010,
  Motivation 5), so the tilted-arm bench row and the 5° threshold of Part B are to
  be checked on a build with 0010.
  Still to do: the seeded firmware's replay is exact (ctx record); rerun this table
  with `elle-replay --mag-axes 90` once 0010 is merged (both branches add
  `Input.mag` to `elle-replay` and a row to the proposals index: a trivial conflict
  for whichever merges second).
- **Bench:** new TEST_PLAN 0.1 row 5: power up pointing south, still; `attitude_data`
  stays within 1.5° of the settled value from 3 s after boot. Arming within the first
  second (gesture) → refused, event 19; after → arms. Tilt the aircraft 20° by hand
  and arm while holding it still → arms (tilts agree). Check one dart log for the
  same boot transient first.
- **Flight:** none beyond confirming the first 60 s of the next flight log.

## Rollback

Part B: `ARM_ATTITUDE_MAX_ERR_DEG` set to 180 disables the refusal. Part A: revert
its commit (a constant `AHRS_SEED_AT_BOOT` could gate it if wanted). Nothing stored.

## Until this lands

Field card for every flight: **power up and leave the aircraft still until the
radio's CRSF attitude reads level (pitch and roll within ~2° of the bench value),
then arm.** In the logs that took 40–50 s; how long it takes depends on the heading,
so watch the reading rather than a clock. Don't switch to Stabilized before then
either: it drives the surfaces to their stops while disarmed.

## Doc updates

- [ ] `docs/OPERATIONS.md`: arming refusal and event 19; boot wait until merged
- [ ] `STATE_DIAGRAMS.md`: arming guard
- [ ] `TEST_PLAN.md`: 0.1 row 5
- [ ] `CLAUDE.md`: IMU pipeline step (seed), arming
- [ ] `crates/elle-ulog/README.md`: none
- [ ] `CHANGELOG.md`: operator behaviour, event codes
