# 0003: Per-airframe board orientation

| | |
|-|-|
| Status | Implemented |
| Airframes | both (eagle changes; dart identical bit for bit) |
| Date | 2026-10-02 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules)) |
| Surfaces | operator behaviour (eagle attitude signs) |

## Motivation

On the eagle, pitch and roll both read with the wrong sign. TEST_PLAN 0.1.2 on the
bench, 2026-10-02 (RPC build `v0.3.0-6-g1cb6063`, `read attitude` over `elle mcp`,
recorded in `logs/test-runs/2026-10-02.jsonl`):

| Tilt (~20°) | Expected | Measured |
|-------------|----------|----------|
| Nose up | pitch > 0 | pitch −13° → −20° |
| Right wing down | roll > 0 | roll −22° … −26° |

The yaw axis is unaffected: level, the aircraft reads pitch ≈ 2°, roll ≈ 1°. Both
horizontal axes reversed and the vertical one intact is a 180° rotation about the
vertical. The eagle's flight controller is not mounted the same way as the dart's,
and the firmware has a single sensor-to-airframe mapping for both airframes
(`elle_control::attitude`: `pitch`, `−roll`, confirmed on the dart).

`GetAttitude` passes `ATTITUDE` through unchanged, and `ATTITUDE` is what the PID
compares against. In Stabilized the eagle would correct every pitch and roll error
the wrong way and diverge. It has not flown in Stabilized. This is an abort
criterion ("any axis reversed"): no Stabilized on the eagle until it is fixed.

The stick inversions (`PITCH_INVERT` / `ROLL_INVERT`) cannot fix it: they act on
pilot input only, and the PID works on raw attitude. The rotation has to happen
before the filter.

## Behaviour delta

| Situation | Before | After |
|-----------|--------|-------|
| Eagle, nose up | pitch negative | pitch positive |
| Eagle, right wing down | roll negative | roll positive |
| Eagle, pitch/roll rates | reversed | correct sign (same rotation) |
| Eagle, yaw / heading | 180° from the board's frame | 180° different from before; heading hold works on yaw differences and is unaffected |
| Eagle, Stabilized | would diverge | corrects toward the setpoint (to be confirmed, Part 1.3) |
| Eagle, level-cal angles (`GetLevelCal`, TUI) | in the board's frame | in the airframe's frame: the stored offset is unchanged, its displayed roll/pitch signs flip |
| Dart | — | unchanged, bit for bit |

Everything that reads `ATTITUDE` follows: the PID, CRSF attitude telemetry, RPC, ULog
`attitude_data` / `controller`, autotune, the navigator's bank comparison.
[`OPERATIONS.md`](../OPERATIONS.md) gains a line on the board orientation constant
under level calibration. No state machine changes.

## Design

- **`elle-config`:** `BOARD_YAW_DEG: f32`, per platform: dart `0.0`, eagle `180.0`.
  This is the rotation about the vertical that takes the sensor's axes onto the
  airframe's, applied before the level-cal tilt. Invariant:
  `const _: () = assert!(BOARD_YAW_DEG == 0.0 || BOARD_YAW_DEG == 90.0 || BOARD_YAW_DEG == 180.0 || BOARD_YAW_DEG == 270.0)`.
  A mounting is a multiple of 90°; anything else is a typo.
- **`elle-control/src/attitude.rs`:** `AttitudePipeline` keeps a single
  `mount` field, now the *effective* rotation: `board × level`. Changes:
  - `AttitudePipeline::with_board(board)`: start with `mount = board`.
  - `set_level_mount(level: Option<UnitQuaternion>)`: `mount = board × level`
    (identity level when `None`).
  - `new()` and `with_modes()` stay identity-board, so the host tests and
    `elle-replay --simulate` keep their sensor-frame = airframe convention.
  - `pub fn board_rotation() -> UnitQuaternion<f32>` builds the platform's
    rotation from `BOARD_YAW_DEG` (not `const`: nalgebra's constructors are not).
  The per-sample work is unchanged: one rotation per vector, as today. The
  composition costs one quaternion product when the mount changes (boot, level cal
  applied or cleared).
- **`elle-hardware/src/imu/driver.rs`:** the IMU task builds its pipeline with
  `with_board(board_rotation())`. The three places that set the mount (flash load or
  clear via `LEVEL_CALIBRATION_SIGNAL`, a level-cal result in `level_cal_step`) call
  `set_level_mount` instead of writing `pipeline.mount`.
- **Level calibration is unchanged.** `compute_mount` works on raw sensor accel and
  returns a pure tilt (gravity onto +Z), and a rotation about +Z does not change
  `up.z`, so the tilt limit and the result are the same. The **stored eagle level
  cal stays valid**: it is a sensor-frame tilt, applied before the board rotation.
- **Level-cal display:** `mount_to_display_deg` reports the offset in the airframe's
  frame by conjugating it through the board rotation (`board × level × board⁻¹`). On
  the dart this is the identity.
- **Mag:** `MAG_FIELD` is offset-corrected in the sensor frame and the pipeline
  rotates it by `mount` (`attitude.rs`), so it picks up the board rotation with no
  change. Mag calibration offsets stay valid.
- **Raw IMU replay:** `imu_raw_ctx.mount` logs `pipeline.mount`, which is now the
  effective rotation, so `elle-replay` reproduces an eagle log exactly without
  knowing the platform. Logs written before this change carry their own (level-only)
  mount and replay as before.

## Interfaces

- **Events:** none
- **RPC:** none. `GetAttitude` values change sign on the eagle; `GetLevelCal` angles
  change sign on the eagle (same stored data).
- **Flash:** none. The level-cal entry (key 3) keeps its meaning (sensor-frame tilt).
- **ULog:** no layout change. `imu_raw_ctx.mount` now documents "board rotation ×
  level-cal mount"; on the eagle it includes the 180°.
- **CRSF telemetry / LED:** CRSF attitude on the radio reads with the right signs on
  the eagle.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| Wrong `BOARD_YAW_DEG` for an airframe | Axes reversed or swapped, Stabilized diverges | Host test pins dart = 0°, eagle = 180°; bench 0.1.2 (all four tilts) and Part 1.3 in full before any Stabilized flight |
| Dart affected by the change | Regression on the airframe that flies today | Board 0° composes to the identity; host test that the dart pipeline fuses bit for bit as before; replay of a dart `imu-raw-log` log still exact |
| Level cal composed in the wrong order | Small tilt error mirrored on the eagle | Host test: a tilted board with a level cal reads level after the board rotation; bench: after re-seating, level reads within the level-cal tolerance |
| Yaw axis also reversed on the eagle (not yet measured) | Yaw rate and heading wrong; yaw damper (inert) and heading hold sign | 0.1.4 (slow 360° yaw) and a yaw-rate sign check on the bench; the 180° rotation leaves the vertical axis as it is, so a reversed yaw would mean the board is upside down, which the level reading (gravity +Z) rules out |
| Sensor loss, RC loss, Core 1 stall, flash write, reboot in the air | Unchanged: the rotation is a constant applied with the existing mount; nothing new is stored or timed | — |

## Verification

- **Host tests** (`crates/elle-control/tests/attitude.rs`, both platform configs):
  - a 180° board: physical nose up (sensor sees `−x, −y` of the dart's mapping) reads
    pitch > 0; physical right wing down reads roll > 0; rates follow
  - converge from level, not from a seed, so a sign error cannot hide in a slow
    filter (the existing tests start at the answer)
  - a board with a level-cal tilt and the 180° rotation reads level
  - `board_rotation()` is the identity on the dart and 180° about Z on the eagle
  - `mount_to_display_deg` on the eagle: a stored offset reads with flipped signs
- **Replay:** `cargo test -p elle-replay` (synthetic log end to end, exact) on both
  platforms; an existing dart `imu-raw-log` log replays exactly.
- **Bench (eagle, props off):** TEST_PLAN 0.1.2 all four tilts; 0.1.4 (yaw, heading
  roughly matches a compass); `read level_cal` angles plausible; then Part 1.3 in full
  (Stabilized correction signs, D damping).
- **Bench (dart):** 0.1.2 nose up and right wing down unchanged.
- **Flight:** none for this change alone; the eagle's first Stabilized flight follows
  Part 1.3 passing.

## Rollback

Set the eagle's `BOARD_YAW_DEG` to `0.0`: the pipeline returns to today's mapping.
Nothing is stored, so a rollback leaves nothing behind.

## Doc updates

- [x] `docs/OPERATIONS.md`: board orientation under level calibration; level-cal
      angles are in the airframe's frame
- [x] `STATE_DIAGRAMS.md`: none
- [x] `TEST_PLAN.md`: 0.1.2 runs all four tilts on each airframe; note the rotation
      in Part 1.3's preamble
- [x] `CLAUDE.md`: IMU pipeline step 3 (board rotation, then level-cal mount)
- [x] `crates/elle-ulog/README.md`: `imu_raw_ctx.mount` is board × level cal
- [x] `CHANGELOG.md`: Fixed, eagle attitude signs (operator behaviour)
