# 0004: Throttle-scheduled attitude gains

| | |
|-|-|
| Status | Draft, in two steps like [0001](0001-eagle-yaw-damper.md): (1) the code ships **inert** (`GAIN_SCHED_*_MIN` = 1.0 on both airframes); (2) lowering the dart's roll factor waits for gate 1 |
| Airframes | dart first; eagle compiled in, inert |
| Date | 2026-10-03 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules), 0.x): tuning inside Stabilized and AltitudeHold, two ULog fields added |
| Surfaces | operator behaviour (Stabilized / AltitudeHold feel at high throttle), ULog (`controller` field added) |

## Motivation

The dart flies a **~4 Hz roll limit cycle** at roll Kp 0.25, and it gets worse with
throttle. Fast roll motion (roll-rate RMS in the 3–40 Hz band) in Stabilized flight,
by throttle band ([`DART_PID.md`](../DART_PID.md#what-actually-ran)):

| Log | Roll Kp / Kd | 0.25–0.5 | 0.5–0.75 | 0.75+ |
|-----|--------------|----------|----------|-------|
| 0050 | 0.25 / 0.07 | 51 dps, 6% > 100 | 92 dps, 5% | 120 dps, 16% |
| 0052 | 0.25 / 0.04 | 54 dps, 7% | 82 dps, 12% | 101 dps, 15% |
| 0033 / 0035 | 0.125 / 0.03 | 34–53 dps, 2–3% | 41–44 dps, 2–3% | 40–87 dps, 3–4% |

Elevon effectiveness grows with the square of airspeed, so a gain that is right at a
slow cruise is too high at full throttle. One fixed gain has to choose between the two
failures: 0.25 oscillates when fast; 0.125 leaves ~20° of wing drop when holding a bank
([`DART_PID.md`](../DART_PID.md#fit-to-the-logs)). The current default, 0.18, splits the
difference and is the fixed-gain test this proposal waits on (gate 1).

Throttle is the only speed signal there is: GNSS has had no fix on either airframe
since late September (TODO, *Pending verification*), and there is no pitot.

## Behaviour delta

| Situation | Before | After (dart, once enabled) |
|-----------|--------|-------|
| Manual | Sticks pass straight through | Unchanged |
| Stabilized / AltitudeHold, throttle ≤ breakpoint | P and D at the configured gains | Unchanged |
| Stabilized / AltitudeHold, throttle above breakpoint | P and D at the configured gains | P and D scaled down linearly with throttle, to `GAIN_SCHED_ROLL_MIN` × gain at full throttle |
| Throttle chopped after a fast run | Gains unchanged | Gains return to full over `GAIN_SCHED_RECOVER_S` (the aircraft is still fast) |
| Autotune running | Configured gains | Factor held at 1.0 (see Risks) |
| Glide or dive at zero throttle | Configured gains | Unchanged: full gains (see Risks) |

Nothing changes in [`STATE_DIAGRAMS.md`](../../STATE_DIAGRAMS.md): no new state or
transition. [`OPERATIONS.md`](../OPERATIONS.md) gets a paragraph on the feel at high
throttle.

## Design

- **Pure logic, `elle-control/src/gain_schedule.rs`:**
  `GainSchedule::update(throttle, dt) -> (pitch_factor, roll_factor)`.
  - Target factor per axis: 1.0 for `throttle ≤ GAIN_SCHED_BREAKPOINT`, then linear to
    `GAIN_SCHED_{PITCH,ROLL}_MIN` at throttle 1.0.
  - The applied factor follows the target **at once when the target drops** (throttle
    up: less gain right away, the safe direction) and **with a first-order lag of
    `GAIN_SCHED_RECOVER_S` when it rises** (throttle chop: the airspeed is still high
    for a few seconds).
  - Input is a normalised 0–1 "speed proxy", so ground speed or airspeed can replace
    throttle later without touching the controller.
- **Applied in `AttitudeController::update`** (`elle-control/src/pid.rs`): the factor
  multiplies the P and D terms of its axis, **not I**. The integral holds trim, which
  changes the other way with speed, and scaling it would make every throttle change
  step the trim.
- **Caller, `elle-system/src/system.rs`:** `norm.throttle` (the pilot's normalised
  stick, before the throttle curve) feeds the schedule each tick. It resets to 1.0
  wherever the PID resets (disarm, low throttle, PID off), and is held at 1.0 while an
  autotune override is active.
- **Constants, `elle-config`:** `GAIN_SCHED_BREAKPOINT` (dart 0.5), `GAIN_SCHED_ROLL_MIN`
  and `GAIN_SCHED_PITCH_MIN` (1.0 = inert on both airframes when shipped),
  `GAIN_SCHED_RECOVER_S` (2.0). Asserts: breakpoint in [0, 1), minimums in (0, 1],
  recovery time > 0.
- **Timing:** two multiplies, one compare and one EMA step per axis per 5 ms tick.
  Negligible.

### First enabled values (step 2, dart)

Roll only; pitch stays 1.0 until a log shows pitch oscillating on its own (in 0052 the
pitch motion during roll episodes came through the shared elevons).
`GAIN_SCHED_ROLL_MIN` = 0.6 with roll Kp back at 0.25: full throttle then runs Kp 0.15,
Kd 0.024, below the 0.18 fixed gain under test, while cruise below half throttle keeps
0.25. Gate 1 sets the final numbers.

## Interfaces

- **Events:** none
- **RPC:** none. `GetControllerOutput` could carry the factor later; not needed for the
  flight test.
- **Flash:** none. The schedule constants are firmware-only, like `PID_SCALE`.
- **ULog:** `controller` gains a trailing `float roll_gain_factor` (and
  `pitch_gain_factor`), appended like `yaw_damp`. The logged `roll_p` / `roll_d` are
  after scaling, so existing analyses stay correct.
- **CRSF telemetry / LED:** none

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| Factor too low at full throttle | Sluggish roll, wing drop at speed | Floor at `GAIN_SCHED_ROLL_MIN` (0.6 first); Manual is unchanged and remains the escape |
| Dive or glide at zero throttle: fast, but full gain | The limit cycle can still appear in a power-off dive, as it can today | No regression against today. Real fix needs a speed signal (GNSS once it works, pitot); the speed-proxy input is there for it |
| Throttle chop at high speed | Gains jump back while still fast | Recovery lag `GAIN_SCHED_RECOVER_S` |
| Autotune measuring a scaled loop | Wrong gains saved | Factor held at 1.0 during autotune; autotune runs are short and at a set throttle anyway |
| RC loss / failsafe | Throttle goes to zero → factor recovers to 1.0 | The failsafe disarms; the PID is off. No new path |
| Core 1 stall, flash write, reboot in the air | Same as today | The schedule is Core 0, stateless across reboots (starts at 1.0) |
| Wrong sign or wiring | Gains scaled up instead of down | Host test: factor never > 1.0; the factor is logged |

## Verification

- **Gate 1 (before step 2):** fly the fixed roll Kp 0.18 (already on the branch) with
  the SD card in. Compare 3–40 Hz roll-rate RMS by throttle band against the table
  above (`logs/dart`, same script as `DART_PID.md`). If 0.18 is quiet at every
  throttle and the wing drop is acceptable, **withdraw** this proposal. If it still
  oscillates at high throttle, or holds banks poorly at low throttle, go to step 2.
- **Host tests (`crates/elle-control/tests/gain_schedule.rs`):** factor 1.0 at and
  below the breakpoint; linear above; equals the minimum at throttle 1.0; never above
  1.0 or below the minimum; drops immediately on a throttle step up; recovers with the
  set time constant on a step down; with minimums of 1.0 the controller output is
  bit-identical to today's.
- **Bench (props off, new TEST_PLAN row next to 1.3):** Stabilized, armed, tilt the
  aircraft and sweep the throttle; the elevon correction for the same tilt shrinks
  above the breakpoint and returns over ~2 s after a chop. Read `roll_gain_factor` in
  the ULog.
- **Flight (step 2):** the same throttle-band table from the new log. Pass: 0.75+
  band at or below the 0033/0035 level (~90 dps, < 5% over 100 dps), and the bank
  hold at low throttle no worse than 0052.

## Rollback

Set `GAIN_SCHED_ROLL_MIN` (and `_PITCH_MIN`) back to 1.0: the controller is then
bit-identical to today's. Nothing is left in flash.

## Doc updates

- [ ] `docs/OPERATIONS.md` (Stabilized feel at high throttle)
- [ ] `STATE_DIAGRAMS.md`: none
- [ ] `TEST_PLAN.md` (bench row next to 1.3)
- [ ] `CLAUDE.md` (Controller section)
- [ ] `crates/elle-ulog/README.md` (`controller` fields)
- [ ] `CHANGELOG.md`
- [ ] `docs/DART_PID.md` (gate 1 and step 2 results)
