# 0004: Yaw sign: positive nose right

| | |
|-|-|
| Status | Draft |
| Airframes | both |
| Date | 2026-10-02 |
| Version impact | breaking ([rules](../VERSIONING.md#bump-rules)): 0.x minor |
| Surfaces | operator behaviour (heading hold, radio heading) / RPC ICD (meaning of `AttitudeResp` yaw fields) / ULog (meaning of `attitude_data.yaw`, `yaw_rate`) |

## Motivation

The firmware publishes yaw and yaw rate with the opposite sign to the one its
consumers assume. TEST_PLAN 0.1.4 on the eagle, 2026-10-02 (RPC build
`v0.3.0-3-g765df4a`, `sample attitude` over `elle mcp`, recorded in
`logs/test-runs/2026-10-02.jsonl`): one slow full turn clockwise seen from above,
nose going right, integrated to a yaw rate of **−368.6°** and a net yaw change of
**−368.7°**. Nose right reads negative.

Heading hold (`elle_control::heading`) and the yaw damper (`yaw_damper.rs`,
proposal [0001](0001-eagle-yaw-damper.md)) both take **positive = nose right**. That
was never confirmed on hardware: proposal 0001 says so, and TEST_PLAN 1.2a says that
if the sign is wrong the fix belongs in `AttitudePipeline::fuse`, not in the
consumers.

With the sign as it is:

- **Heading hold diverges.** A target to the right gives a positive error, the roll
  setpoint banks right, the nose goes right, the published yaw *falls*, and the
  error grows. It turns away from the captured heading, up to its bank limit.
- **The yaw damper would amplify yaw.** It ships with `YAW_DAMPER_GAIN` = 0 on both
  airframes, so it is inert today; gate 1.2a would catch it before a gain goes in.
- **The radio's heading turns the wrong way.** CRSF attitude carries yaw, which
  EdgeTX shows as heading.

Not affected: Manual, and the Stabilized pitch and roll loops
(`AttitudeController::update` receives the yaw rate but ignores it). The navigator
works from GNSS track, not AHRS yaw.

Cause: the AHRS runs in a z-up body frame, where a positive rotation about +Z is
counter-clockwise seen from above (nose left). `AttitudePipeline::fuse` negates
roll for this PCB but passes yaw and yaw rate through. The sensor's vertical axis is
the same on both boards (proposal [0003](0003-board-orientation.md) rotates about
it), so the dart is expected to show the same sign; that is the first thing to check
(Verification).

## Behaviour delta

| Situation | Before | After |
|-----------|--------|-------|
| Nose turning right | `yaw_rate` < 0, yaw falling | `yaw_rate` > 0, yaw rising |
| Heading hold, aircraft yawed off the captured heading | banks away from it | banks back toward it (TEST_PLAN 6.5 row 2) |
| Yaw damper (gain > 0, bench only) | amplifies | opposes (TEST_PLAN 1.2a) |
| Radio heading (CRSF attitude) | counter-clockwise | clockwise, like a compass |
| `GetAttitude` / ULog `attitude_data` yaw, yaw rate | nose left positive | nose right positive |
| Manual, Stabilized pitch/roll | — | unchanged |

[`OPERATIONS.md`](../OPERATIONS.md) gains the yaw convention next to pitch and
roll. Heading hold's behaviour as documented (bank back toward the captured heading)
does not change; it starts being true. No state machine changes.

## Design

- **`elle-control/src/attitude.rs`**, `AttitudePipeline::fuse`: publish
  `yaw: -yaw` and `yaw_rate: -rates.z`, next to the existing roll negation, with the
  convention stated on `Attitude` ("yaw positive nose right"). The AHRS quaternion,
  turn compensation and the accel gate work on the quaternion and body vectors, not
  on the published Euler angles, and do not change.
- **Heading hold, yaw damper, CRSF, RPC, ULog:** no code change. They read the
  published values, and their assumed convention becomes true.
- **Where yaw is zero:** after the negation, yaw is the angle clockwise from the
  AHRS's reference direction. Whether that is magnetic north depends on uf-ahrs's
  earth frame (NWU vs ENU), which is not checked here; heading hold uses yaw
  differences only, so it does not care. Verification measures it against a
  compass, and if the zero is not north the doc says so (a separate change would
  add an offset).
- **`elle-replay`:**
  - `reference.rs` builds the AHRS quaternion from the published yaw
    (`from_euler_angles(roll_f, pitch_f, a.yaw)`): it must use the AHRS-convention
    yaw, i.e. negate it back.
  - `sim.rs` truth yaw is "as the filter reports it": it follows the new sign.
  - **Old logs:** the faithfulness check (`lib.rs`) compares replayed yaw against
    the logged `attitude_data.yaw`. A log written before this change has the old
    sign. Each new log carries a ULog info entry `char[] elle_yaw` =
    `nose_right_positive`; its absence marks a legacy log, and replay negates the
    logged yaw before comparing and reports `legacy_yaw_sign` in its JSON.
- **`elle-ulog`:** one more info entry in the header. `tests/header.rs` checks it
  still fits the 4 KB buffer.
- **Timing:** two negations per sample on Core 1; nothing measurable.

## Interfaces

- **Events:** none
- **RPC:** no type change; `AttitudeResp.yaw_cdeg` and `yaw_rate_cdeg` change sign
  (breaking in meaning). The TUI and `elle mcp` show the new values as they come.
- **Flash:** none
- **ULog:** no layout change; `attitude_data.yaw` and `yaw_rate` change sign
  (breaking in meaning, named in `CHANGELOG.md`); new info entry `elle_yaw`.
- **CRSF telemetry / LED:** the radio's heading turns the right way.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| The sign is not the same on both airframes | Heading hold right on one, divergent on the other | Bench check on the **dart** (0.1.4 turn) before writing code; if it differs, this becomes a per-platform constant like 0003 |
| Heading hold tuned or flown around the wrong sign | Behaviour changes in flight | Check the logs for heading-hold use (events 130/131) on both airframes before merging; none known on the eagle (never flown in Stabilized) |
| A hidden second negation in a consumer | Two wrongs cancel today and break after | Audit done for this proposal: heading hold, damper, CRSF, RPC, ULog, PID (ignores yaw rate), nav (uses GNSS) carry no sign of their own |
| Replay of old logs fails faithfulness | False "replay mismatch" on every existing raw log | Legacy detection via the `elle_yaw` info entry (Design); `cargo test -p elle-replay` plus one existing `imu-raw-log` log replayed |
| Yaw damper enabled before 1.2a | Positive feedback in yaw | Unchanged rule: gain stays 0 until 1.2a passes |
| Sensor loss, RC loss, Core 1 stall, flash write, reboot in the air | Unchanged: a sign in the published output, no state | — |

## Verification

- **Before code — bench (dart):** TEST_PLAN 0.1.4, a slow clockwise turn: confirm
  the yaw rate is negative on the dart too.
- **Host tests** (`crates/elle-control/tests/attitude.rs`, both platforms):
  - a positive rotation about the airframe's vertical seen as clockwise from above
    (nose right) gives `yaw_rate` > 0 and rising yaw, converging from level
  - heading hold closes the loop in a host simulation: start 30° left of the
    target, integrate a yaw rate proportional to bank, and reach the target
  - `nose_right_rate_slows_left_engine` (yaw damper) unchanged and passing
- **Replay:** `cargo test -p elle-replay` (synthetic log, exact, new sign); a
  pre-change `imu-raw-log` log replays as legacy with `problems` empty.
- **Bench (eagle and dart, props off):** 0.1.4 (nose right → `yaw_rate` > 0; yaw
  against a phone compass at four headings, to document where zero is); 6.5 rows 1–4
  (heading hold banks back toward the captured heading); 1.2a only when a gain is
  trialled.
- **Flight:** 6.5 in the air once the bench rows pass; check ULog `controller` for
  the roll setpoint opposing heading error.

## Rollback

Revert the commit (the two negations and the replay handling). There is no constant
to flip: a per-platform switch would only be added if the dart turns out different.
Logs written while the change was in carry `elle_yaw` and replay correctly either
way.

## Doc updates

- [ ] `docs/OPERATIONS.md`: yaw convention (positive nose right; where zero is,
      once measured)
- [ ] `STATE_DIAGRAMS.md`: none
- [ ] `TEST_PLAN.md`: 0.1.4 expects `yaw_rate` > 0 for nose right; 1.2a note
      updated (the sign was wrong and is fixed)
- [ ] `CLAUDE.md`: IMU pipeline step 5 (roll **and yaw** negated)
- [ ] `crates/elle-ulog/README.md`: `attitude_data` yaw convention; `elle_yaw` info
- [ ] `CHANGELOG.md`: breaking, ULog and RPC meaning, operator behaviour
- [ ] `docs/changes/0001-eagle-yaw-damper.md`: the sign assumption is now confirmed
      and enforced
