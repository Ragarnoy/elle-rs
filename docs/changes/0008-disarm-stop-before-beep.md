# 0008: Stop the engines before the disarm beep

| | |
|-|-|
| Status | Implemented |
| Airframes | both |
| Date | 2026-11-13 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules), 0.x): a fix; no RPC, flash, ULog or event change |
| Surfaces | operator behaviour (when the arm and disarm beeps sound) |

## Motivation

The DShot task played a beep pattern **before** it read the new engine target, and
the pattern blocked the task: the disarm beep was 10 × `Beep2`, 100 ms, 10 × `Beep2`,
then a 200 ms sleep. Every disarm signals the disarm beep in the same tick it
publishes the zero target (`flight.rs`, `rpc.rs`), so for ~330 ms after **any**
disarm (gesture, RPC `Disarm`, kill switch, RC or host failsafe) the engines got no
frame at all. An AM32 ESC keeps its last throttle without a new frame, so they kept
running at whatever they were doing.

Evidence: round-A bench B6 (eagle, flight build, LOG_0094). The pilot killed at
473.944 s; `engine_data` holds the 29 784 eRPM target and the engines stay at speed
for 330 ms before the stop frames resume. The arm beep had the same shape (10 ms of
beep then 200 ms asleep) on the way up, so throttle right after arming was also
held back ~210 ms. The bug dates from the beep's introduction (b8384e5 / 39d81a4).

## Behaviour delta

[`OPERATIONS.md`](../OPERATIONS.md#beeps) "Beeps"; the arming diagram in
[`STATE_DIAGRAMS.md`](../../STATE_DIAGRAMS.md) keeps its transitions.

| Situation | Before | After |
|-----------|--------|-------|
| Disarm (any path) with engines running | Engines kept their last throttle ~330 ms, then stopped; beeps while still spinning | `MotorStop` on the next 1 ms frame; the double beep sounds once the engines have spun down (below `BEEP_AT_REST_ERPM`, or `BEEP_SPINDOWN_FALLBACK_MS` after the stop without telemetry) |
| Disarm with engines stopped | Double beep at once | Same |
| Arm | Single beep, then throttle ignored ~210 ms | Single beep; a throttle-up during it cancels the rest of the beep and the throttle frame goes out at once |
| Beep never possible (engine never comes to rest) | — | Dropped after `BEEP_MAX_WAIT_MS` (4 s) |

## Design

- **`elle_control::beep`** (pure, host-tested in `crates/elle-control/tests/beep.rs`):
  `BeepSequencer::next(now_ms, spin_commanded, at_rest)` decides one frame slot at a
  time. A beep starts only with every target zero and every engine at rest, and a
  non-zero target cancels a playing or pending beep. Bursts of 10 frames; the
  disarm beep's two bursts are `BEEP_GAP_MS` apart, during which normal (stop)
  frames go out. `engine_at_rest` judges rest by telemetry when it is valid, else by
  time since the target went to zero.
- **`elle-hardware/src/dshot.rs`**: both tasks take `DSHOT_THROTTLE` **first**, then
  ask the `Beeper` whether this slot is a beep. If it is, they send that one command
  (both engines joined on the eagle), pace, and continue; otherwise the normal frame
  path runs unchanged. No sleep remains in the beep path: the send loop's pacing
  (`pace`, 1 kHz) is the only wait.
- **Constants** (`elle-config`): `BEEP_AT_REST_ERPM` 2 000, `BEEP_SPINDOWN_FALLBACK_MS`
  2 000, `BEEP_MAX_WAIT_MS` 4 000, `BEEP_GAP_MS` 100.
- **Timing:** one sequencer step per DShot frame (a few compares). A beep slot skips
  that frame's telemetry read, so `engine_data` is not refreshed for those ~20 slots;
  the targets are zero then.

## Interfaces

- **Events / RPC / flash / ULog / LED / CRSF:** none. `BEEP_SIGNAL` and `BeepPattern`
  keep their names and callers.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| A beep takes a frame slot while thrust is wanted | Throttle frame lost | A beep plays only with every target zero; a non-zero target cancels it in the same slot |
| Beep starts while an engine still turns | Beep commands on a spinning ESC (AM32 ignores them while running) | Starts only at rest: telemetry below `BEEP_AT_REST_ERPM`, or 2 s after the stop without telemetry |
| No beep heard after a disarm | Operator unsure it disarmed | LED and event codes unchanged; the beep is late (spin-down), not gone, unless the engine never reads at rest within 4 s |

RC loss, failsafe, kill switch: all disarm paths now stop the engines on the next
frame, which is the point. Core 1 stall, flash write, reboot: unchanged.

## Verification

- **Host tests:** `crates/elle-control/tests/beep.rs` (rest gating, burst counts and
  gap, cancel on a non-zero target mid-burst and while pending, drop after the
  deadline, replacement, telemetry vs time rest).
- **Bench:** TEST_PLAN 1.4 rows 3–5 and 8 (beeps by ear), and a repeat of round-A B6
  at power: in `engine_data`, the target reaches 0 and the eRPM starts falling within
  one control tick of the disarm event; the double beep follows the spin-down.
- **Flight:** none beyond the round-A plan.

## Rollback

Revert the commit. Nothing is stored.

## Doc updates

- [x] `docs/OPERATIONS.md`: Beeps
- [x] `STATE_DIAGRAMS.md`: beep notes on the arming diagram
- [x] `TEST_PLAN.md`: 1.4 row 5
- [ ] `CLAUDE.md`: none needed beyond the DShot paragraph
- [ ] `crates/elle-ulog/README.md`: none
- [x] `CHANGELOG.md`: `Unreleased`, Fixed
