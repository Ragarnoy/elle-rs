# 0010: Magnetometer axis alignment

| | |
|-|-|
| Status | Draft, implemented on branch `mag-axes`: bench 0.1.5 a passed; b and c to redo in one session (c shows the alignment in the dip but missed its yaw and tilt criteria); Accepted after both |
| Airframes | both (eagle first; dart after its own gate, below) |
| Date | 2026-11-14 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules), 0.x): a fix. No RPC type, flash entry, event code or ULog field changes meaning (`imu_raw_mag` stays "the vector as fed"); one ULog info entry is added |
| Surfaces | operator behaviour (eagle yaw and heading, and pitch/roll wherever the mag pulls on them), ULog (one info entry added) |

## Motivation

The firmware fuses the MMC5616WA's axes as if they were the ICM-42686's: one mount
rotation (`AttitudePipeline::mount`, board × level cal) is applied to accel, gyro
and mag alike. On the rev A board the two chips are a quarter turn apart about the
vertical. Five independent pieces of evidence, all from the eagle, 2026-11-14:

1. **Gyro consistency.** For a constant earth field, each mag sample must equal the
   previous one rotated by the gyro in between. Over the 48 signed axis
   permutations, with the hard-iron offset fitted jointly, on three mag-cal tumbles
   (raw IMU capture, `imu-raw-log`):

   | Log, window | Best map | Residual | Identity (today) | Next best |
   |---|---|---|---|---|
   | LOG_0121, 31–62 s | imu = (−my, +mx, +mz) | 0.005 G | 0.036 G | 0.031 G |
   | LOG_0117, 440–470 s | same | 0.009 G | 0.055 G | 0.035 G |
   | LOG_0117, 612–642 s | same | 0.005 G | 0.029 G | 0.020 G |

   LOG_0121 again from `attitude_data` rates instead of the raw capture: same map,
   0.005 G against 0.037 G. The fit cannot tell a map from its exact negative
   (+my, −mx, −mz), a mirror; 2 and 3 settle it.
2. **Datasheet.** MMC5616WA datasheet (MEMSIC, 5/2023), p. 6, "Marking
   illustration": +X runs from ball A2 to A1, +Y from row A to row B, +Z out of the
   top face. Right-handed, so not the mirror.
3. **Layout.** elle-pcb rev A (`master` fbea092): IC1 on the top side, rotated 180°
   from its footprint's default. Installed in the eagle, A1 is nose-port, A2
   nose-starboard, B1 aft-port. So the chip's X points to port, Y aft, Z up. Item 1
   then says the IMU's sensor axes point forward, port, up, and the eagle's 180°
   board rotation (proposal [0003](0003-board-orientation.md)) turns them into the
   firmware's airframe frame (see "Heading reference" below).
4. **A compass.** Bench reading level, raw (3034, −803, −7189) counts, stored
   offsets (1167.5, 984, −2386.5): corrected and mapped as above, the nose points
   46° east of magnetic north with a dip of 61.7° down (France: ~60–62°). The
   operator's phone compass read NE. With today's mapping the same reading gives a
   heading 90° off.
5. **Dip through large tilts** (added after the bench, LOG_0126, build with the
   alignment). The dip is the angle between the field and the horizontal: with the
   right mapping it is the same at every attitude. At five hand-held holds tilted
   57–69°, the aligned mapping gives 59.4–66.4° (61.8° level); the old mapping,
   replayed on the same samples (`--mag-axes -90`), gives −16.5° to +28.9°.

What a quarter-turn error does: when level it is a constant 90° heading error. Tilted,
the mis-mapped field is no longer a rotated copy of the true one, so the angle
between field and gravity changes with attitude. Live poses with today's mapping
showed it: dip 62.5° level, 37.6° nose up, 38.5° nose down (bench, probe attached,
so partly the bench's own distortion). Madgwick fuses the mag through the whole
quaternion, so this inconsistency can pull pitch and roll, most in banked turns and
tilted boots: the cases proposal 0009 and [`ATTITUDE.md`](../ATTITUDE.md) chase. How
much is not settled; in steady state a Madgwick mag term mostly constrains heading.
Static tilts on the bench (Verification, LOG_0126) lean towards the alignment; turns
and flight are not measured yet: every eagle capture with engines is from the
hatch-magnet era.

The eagle's mag readings before 2026-11-14 also carried a ~20 G hatch magnet,
since removed. The dart carries a magnet too (operator, 2026-11-14), still to be
removed; see the dart gate.

## Behaviour delta

| Situation | Before | After |
|-----------|--------|-------|
| Eagle, yaw and heading (`attitude_data.yaw`, `GetAttitude`, CRSF attitude) | ~90° off magnetic, and moving as the aircraft tilts | Consistent with the field; still in the existing heading reference (below) |
| Eagle, yaw while tilting at a fixed heading | Moves with the tilt | Stays within a few degrees |
| Eagle, pitch/roll | Can be pulled by the inconsistent field when tilted or turning | Not pulled by it. Bench tilted rests (LOG_0126) lean that way but missed the 0.1.5 c criteria; to redo, then the next flight capture |
| Eagle, heading hold | Captures a heading on engage; works on differences | Same; a jump of ~90° in the absolute value at the first boot of the new build |
| Eagle, mag calibration | — | Unchanged; the stored offsets (key 2) stay valid, no recalibration needed |
| Raw mag (`MAG`, `GetMagnetometer`, ULog `magnetometer_data`, TUI and CRSF raw-mag heading) | Chip frame | Unchanged, chip frame |
| Dart | — | Unchanged, bit for bit, until its gate passes |

OPERATIONS.md: one line under mag calibration that the chip frame is turned into
the IMU's before fusion, and that raw mag values stay in the chip frame. No state
machine changes.

## Design

- **`elle-config`:** `MAG_AXES_YAW_DEG: f32`, per platform: eagle `90.0`, dart
  `0.0` (until its gate; then `90.0`, the same board). The rotation about the
  vertical that takes the mag chip's axes onto the IMU's sensor axes. Same
  quarter-turn assert as `BOARD_YAW_DEG`. It describes the chips on the PCB, so it
  sits in front of the board rotation, never folded into it.
- **`elle-control/src/attitude.rs`:**
  - `pub fn mag_axes(m: Vector3<f32>, yaw_deg: f32) -> Vector3<f32>`: an exact axis
    swap, no trigonometry: 0 → (x, y, z), 90 → (−y, x, z), 180 → (−x, −y, z),
    270 → (y, −x, z). Exact, so the dart (0) fuses bit for bit as before.
  - `AttitudePipeline` gains a `mag_axes_deg` field, set by a new
    `with_mag_axes(self, deg) -> Self` builder; `with_board(board)` keeps its
    signature, so its callers (`driver.rs`, `tests/board_orientation.rs`, and 0009's
    `tests/attitude_seed.rs` on `attitude-boot-seed`) are untouched. `new()` /
    `with_modes()` keep 0, so the host tests and `elle-replay --simulate` (sensor
    frame = airframe) are unchanged.
  - `mag_to_airframe(mag)` becomes `mount × mag_axes(mag)`.
- **After the offset, not on the raw reading.** `MAG_FIELD` is the offset-corrected
  field in the chip frame (`i2c_sensors.rs`). The swap happens where it is mounted,
  in the IMU task. The hard-iron offsets therefore stay in the chip frame, and the
  stored calibration stays valid. Calibration (sphere fit) and `MagHealth` (magnitude
  band, frozen check) don't depend on the axis order, so they are untouched.
- **`elle-hardware/src/imu/driver.rs`:** builds the pipeline with
  `with_board(board_rotation()).with_mag_axes(elle_config::MAG_AXES_YAW_DEG)`. No
  other change.
- **Raw IMU replay:** `imu_raw_mag` records the vector as fed (after
  `mag_to_airframe`), so a log from the new build replays exactly as before. A new
  ULog info entry, `elle_mag_axes_deg`, records the constant, written right after
  the standard ones (`ULogWriter::write_info_i32`) so it stays in the definitions
  section. `elle-replay --mag-axes <deg>` runs a what-if, relative to whatever the
  log did (90 on every log so far): it un-mounts the fed vector with the logged
  `imu_raw_ctx` mount, swaps the axes, and mounts it again. The what-if filter is
  seeded at the heading the remapped field gives (`variants::reseat_heading`), so it
  measures the mapping, not a 90° heading convergence. `--compare` adds a "against
  the firmware replay" table (mean, RMS, max per axis for every variant).
  `examples/mag_axes_still.rs` scores the what-if where the answer is known: tilt
  against the accel at still rests, and the dip at each rest.
- **Seed (proposal 0009):** `attitude::seed` takes the airframe-frame mag, so it gets
  the fix with no change.
- **Timing:** a few sign flips per mag update on Core 1 (~10 Hz). Nothing
  measurable.

### Heading reference (not changed here)

`attitude::angles` reports unnegated Euler pitch as nose up and negates roll.
Together with the nose-up-positive and right-wing-down-positive results of TEST_PLAN
0.1.2 on both airframes, that makes the firmware's airframe frame **x aft, y
starboard, z up**. Yaw is therefore the bearing of the **tail**, counted
anticlockwise from the AHRS's earth x axis, and with a north-west-up earth frame
yaw ≈ 180° − nose compass bearing. A check of that model: the bench sample above,
through **today's** mapping into this frame, gives yaw −136°; the firmware reported
−141° at that pose. With this change the same pose (nose 46°) should read
yaw ≈ +134°. Proposal 0006 (yaw sign; Draft, on branch `yaw-sign`, not yet on master) owns
the yaw sign and where yaw is zero. It already lists the zero as unchecked, and this
finding gives the answer to check there. Heading hold and the yaw damper use differences only.
This proposal changes neither the sign nor the zero.

## Interfaces

- **Events:** none
- **RPC:** none. `GetAttitude` yaw changes value on the eagle (same type).
- **Flash:** none. Mag offsets (key 2) keep their meaning (chip frame).
- **ULog:** no layout change. `imu_raw_mag` documents "after the mag axis swap and the
  mount"; info entry `elle_mag_axes_deg` added (header must still fit the 4 KB buffer).
- **CRSF telemetry / LED:** CRSF attitude yaw changes on the eagle. The CRSF GPS-frame
  heading (`crsf/telemetry.rs`, `atan2` of raw mag y and x) and the TUI's mag heading
  are raw chip-frame values with no offset or tilt correction. They are wrong today
  and stay wrong. A follow-up should replace them with the AHRS yaw once 0006 settles
  the convention.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| Wrong direction (270 instead of 90) | Heading 180° off and the tilt inconsistency still there, worse than today | Host test on the real bench sample: (3034, −803, −7189) minus the stored offsets, eagle mapping → yaw 134° ± 1° (nose 46°), dip 61.7° ± 1° down (`tests/mag_axes.rs`); bench 0.1.5 b reads the four headings in order |
| The dart gets the eagle's value before its magnet is gone | A remap on a field dominated by a magnet; nothing gained, results unreadable | Dart stays at 0 until its gate |
| Pitch/roll change in flight on the eagle | The PID sees a different attitude in turns | No clean-field flight capture exists to replay. Gate instead: 0.1.5 c (tilted rests, tilt error against the accel ≤ 1°), then Part 1.3 and 0.1 on the bench, then a Manual flight with `imu-raw-log` (or the 7.2 vehicle test) replayed with `--compare` and `--mag-axes -90` before the eagle flies Stabilized on this build |
| Heading hold engaged across the update | — | It captures on engage; the new build's first boot starts from the new heading |
| A magnet or bench steel near the chip | The field is still wrong, and now it looks correct | Bench rows run away from the bench, PSU and probe; `MagHealth` and the dip check catch a strong field |
| Sensor loss, RC loss, Core 1 stall, flash write, reboot in the air | Unchanged: a constant swap on an existing path; nothing stored or timed | — |

## Verification

Done so far (2026-11-14, branch `mag-axes`):

- **Host:** `crates/elle-control/tests/mag_axes.rs` passes on both platforms. It
  covers each quarter turn, the platform value, the bench sample (yaw 134°, dip
  61.7°; today's mapping −136°), a full pipeline converging to that heading from
  north, and 0 bit for bit. The `elle-control`, `elle-replay` and `elle-ulog` suites
  pass, and the eagle and dart firmware pass clippy (flight and RPC + RC) and build
  with `imu-raw-log`.
- **Replay, what there is.** Every eagle raw capture before 2026-11-14 is from the
  hatch-magnet era: LOG_0092, 0093 and 0109–0117 read 5–23 G, so they say nothing
  about the mapping. That includes LOG_0092–0093, the only eagle captures with
  engines. LOG_0118–0120 come after the magnet but with stale offsets (0.17 G); both
  mappings show proposal 0009's 15–18° boot transient. LOG_0121 after its
  calibration (61.6 s) has five rests, all level:

  | | Firmware (today) | `--mag-axes 90` |
  |---|---|---|
  | Yaw | −135° … −144° | +131° … +143° |
  | Tilt error vs accel | 0.10–0.32° | 0.12–0.37° |
  | Dip | 66.3–71.6° | 65.0–71.9° |

  +131…+143° is a nose at 37…49°, the phone's NE. Pitch and roll can't move at a
  level rest. Tilted rests came later, on the bench (0.1.5 c, LOG_0126, below);
  turns are not in any log yet: the next flight.

The heading result is a consistency check at the same pose as the compass reading
in Motivation, not independent evidence.

- **Bench, TEST_PLAN 0.1.5** (flight build with `imu-raw-log`, `round-a` +
  `mag-axes` at `dcde7c7`, no probe; `logs/test-runs/2026-11-14.jsonl`):
  - **a, pass** (LOG_0125): mag cal tumble 123–153 s saved (113, offsets 772, 1011,
    −2428); `mag_gyro_fit.py` on it picks (−my, +mx, +mz) at 0.0064 G, next 0.0265 G,
    identity 0.0384 G: a fourth tumble agreeing. Settled level rest nose NE (phone):
    yaw 134.0° ± 0.3° (expected 134°; the old mapping replayed reads −140°, nose
    320°). The same spot and pose as Motivation 4.
  - **b, to redo** (LOG_0126): holds at NE, N, W, S, E (anticlockwise) from 4 to
    ~32 s after boot, hand-held. Yaw rose 104°, 110°, 109° per 90° turn. Those
    steps come from the gyro and show only the direction. The error against
    180° − bearing shrank at each hold (−131°, −68°, −54°, −34°, −15°) to −1° … −3°
    at the settled NE rest at the end: the boot convergence (proposal 0009), and
    consistent with a correct heading, but the absolute heading at N, W, S and E is
    not measured. Redo with the aircraft settled (≥ 90 s after boot), each hold
    rested ~10 s.
  - **c, to redo; missed all three criteria, dip clearly right** (LOG_0126, holds tilted
    57–69°, 2–4 s each, `STILL_DPS=6 STILL_S=1`):

    | Hold (s after boot) | 38.6 | 47.0 | 53.0 | 61.5 | 66.2 | level 76.0 |
    |---|---|---|---|---|---|---|
    | Dip, aligned | 66.4° | 61.4° | 59.4° | 63.3° | 65.8° | 61.8° |
    | Dip, old mapping | −6.4° | 26.0° | −16.5° | 28.9° | −1.2° | 62.2° |
    | Tilt error vs accel, aligned | 5.25° | 5.13° | 0.13° | 0.46° | 1.13° | 0.30° |
    | Tilt error vs accel, old mapping | 1.55° | 8.13° | 1.75° | 5.77° | 4.23° | 8.34° |
    | Yaw, aligned | 93.6° | 121.3° | 121.5° | 126.8° | 121.9° | 131.0° |

    Holds 1–2 are 5° off in tilt with the alignment. Hold 1 may still be settling
    after the heading holds; hold 2 is not explained (same yaw as the later ones).
    Yaw at the tilts sits 4–10° from level, outside the ± 5° row. Candidate causes,
    none checked: holds too short to settle; the table (0.35 G lying on it against
    0.41–0.44 G in the hand); the calibration, whose offsets differ from the gyro fit
    on the same tumble (891, 1296, −1995) by up to 433 counts (~0.026 G).
    Against the row: yaw 4–10° from level (± 5°), tilt error 5° at holds 1–2 (≤ 1°),
    dip −2.4° to +4.6° from level (± 3°). Redo in the same session as b, rested
    ~10 s off the table at 30°.

    The level rest at 76 s, 10 s after the last tilt, is the nearest thing yet to a
    pitch/roll answer: the old mapping replayed is 8.3° off in tilt there, the
    alignment 0.3°. It is a transient, not a steady state: the old-mapping filter's
    yaw there (−175.5°) is still ~35° from where it settles at a level rest (−140°,
    LOG_0125), so the inconsistent field during the tilts has dragged it and it has
    not recovered; at a settled level rest both mappings agree within 0.4°
    (LOG_0121, LOG_0125). The what-if starts from the same seed and sees the same
    samples, so it is not an artefact of the replay; whether a turn drags the old
    mapping as far is the flight question.
  - ULog: one start-up dropout per file (LOG_0125, LOG_0126), as before the merge.

The gyro-consistency fit is `tools/elle-replay/scripts/mag_gyro_fit.py` (raw
`magnetometer_data` plus `attitude_data` rates, so any build). Raw mag stays in the
chip frame on every build, so on the new build too it must pick the map matching
`MAG_AXES_YAW_DEG`, (−my, +mx, +mz) on the eagle, not identity.

Still to do:

- **Replay:**
  - The first eagle flight with `imu-raw-log` and a clean field: `--compare`
    tilt error through turns against the gyro-only reference. On the new build the
    firmware track is the aligned one; `--mag-axes -90` gives the old mapping for
    comparison.
  - Proposal 0009's seed table, rerun with the remap. Its tilted rows used the wrong
    mapping.
- **Bench** (new TEST_PLAN 0.1.5, flight build with `imu-raw-log`, SD card,
  **no probe**, ~1 m from the bench, PSU and any steel). The cal comes first: the
  stored offsets and the ones the gyro fit finds differ by ~400–500 counts
  (~0.03 G), several degrees of heading, so rows 2–3 run on a fresh cal.
  1. Done (pass, above). Mag-cal tumble at the test spot (event 113):
     `mag_gyro_fit.py` on it picks (−my, +mx, +mz) (or its mirror, which always
     ties), residual ≤ 0.01 G.
  2. At least 90 s after boot, level, rested ~10 s, nose at each of four
     phone-compass bearings (N, E, S, W): yaw ≈ 180° − bearing (180°, 90°, 0°,
     −90°), each within ± 10° (before 0006 lands). A 270° mapping reads 180° off at
     every bearing.
  3. At one heading, rested ~10 s each, off the table: level, nose up 30°, nose down 30°, each wing
     down 30°: yaw within ± 5° of level, dip (from accel and mag) within ± 3°,
     tilt error against the accel ≤ 1° (`examples/mag_axes_still.rs`, `REST_TRACE=1`;
     on a new-build log its firmware column is the aligned mapping and
     `--mag-axes -90` gives the old one).
  4. 0.1.4 (slow 360° yaw) as before.
- **Dart gate** (before its constant becomes 90): magnet removed; mag cal; a ground
  tumble with `imu-raw-log`; the gyro-consistency fit picks (−my, +mx, +mz) by a
  margin of at least 3× over identity. Its flight logs so far lean the same way
  (3 of 4 logs) but don't settle it: 0.04–0.06 G residuals in flight, with a fitted
  offset of ~1.2–1.6 G from the magnet.
- **Flight:** the eagle's next flight logs `imu-raw-log`; check yaw against GNSS
  track in straight legs, and roll in turns against the replay reference.

## Rollback

Set `MAG_AXES_YAW_DEG` to 0 for the platform: the old fusion, bit for bit. Nothing is
stored, so nothing is left behind.

## Doc updates

- [x] `docs/OPERATIONS.md`: mag calibration note (chip frame, swap before fusion)
- [x] `STATE_DIAGRAMS.md`: none
- [x] `TEST_PLAN.md`: 0.1.5
- [x] `CLAUDE.md`: IMU pipeline step 3 (mag axis swap before the mount)
- [x] `crates/elle-ulog/README.md`: `imu_raw_mag` meaning, `elle_mag_axes_deg`
- [x] `CHANGELOG.md`: `Unreleased`, operator behaviour (eagle heading), ULog
- [x] `docs/ATTITUDE.md`: mag axes, and the results of the replay above
