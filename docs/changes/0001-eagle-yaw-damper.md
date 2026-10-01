# 0001: Yaw damper on the eagle

| | |
|-|-|
| Status | Accepted, in two steps: (1) the code ships **inert** (`YAW_DAMPER_GAIN` = 0 on both airframes); (2) raising the eagle gain waits for gate 1 |
| Airframes | eagle (dart: compiled out, single engine) |
| Date | 2026-10-01 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules)): adds behaviour inside Stabilized and AltitudeHold, removes none; one ULog field added |
| Surfaces | operator behaviour (Stabilized / AltitudeHold yaw feel), ULog (`controller` field added) |

## Motivation

The attitude controller closes loops on pitch and roll only. `AttitudeController::update`
receives the yaw rate and drops it (`crates/elle-control/src/pid.rs:199`). In Stabilized
the yaw stick passes straight through to differential thrust and the small elevon yaw
term (`crates/elle-system/src/system.rs:557`). Nothing damps yaw.

A flying wing has no vertical tail, or only a small one, so its directional stability is
weak and its Dutch roll is lightly damped. The roll PID sees only the roll half of a
Dutch roll oscillation and fights it with elevons, which also produces adverse yaw. The
eagle has a yaw actuator, differential thrust, that only the pilot uses today.

**No evidence yet.** No eagle flight log exists to check for a yaw problem (see the
gate 1 result), and none is expected soon. So the change is split:

1. **Inert code, now.** The damper, its tests, the wiring and the ULog field ship with
   `YAW_DAMPER_GAIN` = 0.0 on both airframes. The output is then exactly 0 every tick,
   and the aircraft flies as before. The only visible change is the new
   `controller.yaw_damp` field, which logs 0.
2. **Enabling it, later.** Raising the eagle gain above 0 is the flight-safety change.
   It waits for gate 1 (a flight log that shows a lightly damped Dutch roll) and gate
   2 (engine lag). If gate 1 shows a well-damped yaw mode, the gain stays 0 and the
   code can be removed.

## Behaviour delta

| Situation | Before | After |
|-----------|--------|-------|
| Manual | Yaw stick → differential thrust | Unchanged. Manual stays pure passthrough and remains the escape. |
| Stabilized / AltitudeHold, yaw stick centred, yaw disturbance | Engines equal | The engines differ to oppose the yaw rate (washed out, see Design) |
| Stabilized, yaw stick deflected | Differential set by the stick | Stick plus the damper term. Briefly resists the yaw the stick starts, then the washout lets the turn through. |
| Steady coordinated turn (stick or heading hold) | Engines equal | Engines equal again once the washout settles (~3 τ) |
| Disarmed, failsafe, kill switch, zero throttle | Engines 0 | Unchanged: `apply_differential_thrust_lut` returns (0, 0) when base thrust is 0 |
| Dart | No differential thrust | Unchanged (`YAW_DAMPER_GAIN` = 0, asserted) |

The table is step 2, the eagle with a gain above 0. In step 1 every row is "unchanged".

OPERATIONS.md: the Stabilized and AltitudeHold mode descriptions gain one line each.
STATE_DIAGRAMS.md: no new state or transition. The damper is active in the existing
Stabilized and AltitudeHold states and resets on entry.

## Design

**Control law** (new pure module `elle-control/src/yaw_damper.rs`, host-tested):

```
r_hp[k]  = washout(r[k])                       // 1st-order high-pass, τ = YAW_DAMPER_WASHOUT_S
cmd      = clamp(YAW_DAMPER_GAIN · r_hp, ±YAW_DAMPER_MAX)
yaw_diff = clamp(pilot_yaw + cmd, ±1)          // into the differential thrust LUT only
```

- `r` is `att.yaw_rate`, the 30 Hz Butterworth-filtered body z rate already published
  with the attitude. No new IMU work and nothing on Core 1.
- **Washout** keeps the damper from fighting steady turns. In a coordinated turn the yaw
  rate is `g·tan(φ)/V`, which is constant. Without a high-pass the damper would oppose
  every heading-hold turn and every turn the pilot holds. Start at τ = 1.0 s, well below
  the heading-hold time scale and long compared with the Dutch roll period, which is
  expected to be about 1 s (to be measured, gate 1).
- **Elevons see the pilot's yaw only.** The damper term goes into the differential
  thrust command and not into `mix_elevons`. Fed through `YAW_TO_ELEVON_GAIN` it would
  become a roll command, and the roll PID would fight it.
- **Sign.** The firmware's convention for normalized yaw needs care. With
  `YAW_INVERT` = -1, a right stick gives a negative normalized yaw, which slows the
  right engine and turns the nose right (TEST_PLAN 1.2). So a positive command slows
  the left engine, a nose-left moment. The old comment in
  `generate_yaw_differential_lut` ("Right turn: reduce left engine") said the opposite;
  it is fixed. A positive `yaw_rate` is taken to be nose right, from heading hold's
  convention (a right bank raises AHRS yaw). TEST_PLAN 6.5 hasn't confirmed that on
  the hardware yet. With both conventions, opposing the motion is `cmd = +gain · r_hp`.
  The host test `nose_right_rate_slows_left_engine` pins this through
  `apply_differential_thrust_lut`. Bench row 1.2a checks the gyro half on the
  hardware, and must pass before any flight with a gain above 0.
- **Authority** is set by the existing LUT. Full differential takes 20 % off one engine,
  so the moment scales with throttle and the damper has no authority at idle.
  `YAW_DAMPER_MAX` = 0.5 (10 % differential) leaves the pilot's stick at least half the
  range at all times. The LUT only ever *reduces* an engine, so damping costs up to 10 %
  thrust on one side and never adds thrust. A symmetric ± mixer would remove that cost,
  but it would also change the pilot's yaw feel in Manual. Out of scope.
- **Actuator lag.** The command becomes an eRPM target and goes through the governor's
  feed-forward plus PI and the EDF's spool-up. If that lag is a large fraction of the
  Dutch roll period, the damper adds phase and can make the oscillation worse. Gate 2
  measures it before any flight.
- **Reset**, like the PID: zeroed on entering Stabilized or AltitudeHold, on disarm, and
  when `low_throttle` (< 5 %) holds. The washout state starts from the current rate so
  there is no step.
- **Autotune**: the damper stays on during a run, so the gains are tuned with the yaw
  dynamics the aircraft actually flies with. The relay excites roll, not yaw. If a run
  shows yaw coupling into the measurement, revisit this.

**Implemented:** `elle_control::yaw_damper::YawDamper` (`from_config`, `update`,
`reset`); tests in `crates/elle-control/tests/yaw_damper.rs`;
`elle_control::mixing::yaw::normalized_yaw_to_rc` converts the command for the LUT.

**Wiring** (`crates/elle-system/src/system.rs`, Stabilized / AltitudeHold branch only):
`FlightController` holds a `YawDamper`. It is updated next to the attitude PID with
`att.yaw_rate`, and its output is added when `yaw_rc` is built for
`apply_differential_thrust_lut` (`system.rs:580`). When there is no attitude, the
existing fallback to manual inputs also drops the damper. `update_fast_path_raw`
(Manual) is not touched.

**Constants** (`elle-config/src/lib.rs`, with `const _: () = assert!(…)`):

| Constant | Eagle | Dart | Invariant |
|----------|------:|-----:|-----------|
| `YAW_DAMPER_GAIN` (normalized yaw per rad/s) | to be set (start ~0.3) | 0.0 | dart == 0; eagle ≥ 0 |
| `YAW_DAMPER_WASHOUT_S` | 1.0 | 1.0 | > 0, > 2·`CONTROL_LOOP_DT` |
| `YAW_DAMPER_MAX` | 0.5 | 0.5 | in (0, 1] |

The washout coefficient is derived from `CONTROL_LOOP_DT` at compile time, as
`SETPOINT_FILTER_ALPHA` is.

**Timing:** one high-pass, one multiply and two clamps per 5 ms tick, under 1 µs. Core 1
is not affected.

## Interfaces

- **Events:** none
- **RPC:** none. The gain is a compile-time constant until it is proven. Putting it
  into `SetPidGains` / `ProfileEntry` 1 would be a flash-profile break; a later proposal
  can do that if it is needed.
- **Flash:** none
- **ULog:** `controller` gains `yaw_damp` (f32, normalized damper command after the
  clamp, 0 when inactive). It is an addition, so not breaking. The payload goes from
  53 B to 57 B; `crates/elle-ulog/tests/header.rs` must still pass. `attitude_data`
  already logs the yaw rate and `engine_data` the per-engine targets.
- **CRSF telemetry / LED:** none

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| Sign inverted | Positive feedback: the yaw oscillation grows | Host test on the mixer sign; bench row 1.2a before any flight; first flight with the gain low; Manual escape (CH6) leaves the damper out entirely |
| Actuator lag too large | The damper adds phase and the oscillation gets worse | Gate 2 measures the governor and EDF response; keep the gain low or withdraw if the lag is more than ¼ of the Dutch roll period |
| Damper fights intended turns | Sluggish yaw stick, heading hold slow to start a turn | Washout; ULog `yaw_damp` against yaw rate; flight row checks heading hold |
| Thrust loss while damping | Up to 10 % less thrust on one engine | `YAW_DAMPER_MAX`; it only matters while the damper is working |
| Noisy yaw rate (vibration) | Engine eRPM jitter, ESC heating | Rate is already 30 Hz low-passed; check `engine_data` target jitter in the bench run with props on |
| Sensor loss / stale attitude | — | No attitude → the existing manual fallback with no damper term |
| RC loss, kill switch | — | Engines go to 0 through the existing paths; the damper cannot add thrust from 0 |
| Core 1 stall | — | `check_core1_health` disables the attitude controller; the damper is gated on the same flag |
| Flash write | — | Only while disarmed, when the damper is reset |
| Reboot in the air | — | Boots disarmed, the same as today |

## Verification

**Gate 1, evidence (before any code):** go through the existing eagle ULogs
(`flight-logs` skill) for Stabilized segments. Look for an oscillation in `yaw_rate`
coupled with `roll_rate`, and estimate its period and damping ratio. No lightly damped
mode found → withdraw. Mode found → its period sets the washout τ and the lag budget.

**Gate 1 result (2026-10-01): not answerable yet.** No eagle log has controlled flight
in it.

- `logs/LOG_0055–0065`: Eagle header, all bench runs. There is no GNSS ground speed,
  roll stays within 14° and the yaw rate p99 is ≤ 16°/s.
- `~/Downloads/logs/logs/LOG_0000–0035`: dart. `LOG_0000` says Eagle, but only the
  left engine ever ran, and it predates the per-platform `ver_hw` (fc3b456).
- The March `flight_*.ulg` RPC extractions: bench.
- `~/Downloads/LOG_0013.ULG` (old firmware, 83 Hz) is the only eagle log with the
  engines at full power. It has six runs of 4–7 s:
  - At 526 s and 685 s, in Stabilized, the nose falls from about −12° to −85° within
    about 2 s of full power, and the wing then rolls through ±180°.
  - The yaw rate stays within ±25°/s until pitch is already past −60°.
  - At 955 s, in Manual, it rolls off to about 90° bank.
  - None of the runs has a stretch of wings-level flight to fit a Dutch roll to. The
    departures start in pitch, not yaw.
  - Left and right eRPM differ by a steady 2–3 % at full throttle (about 131k vs
    127k). That is a constant yaw moment, not a damping problem.

Gate 1 needs an eagle data flight with the current firmware: Stabilized, stick-free
straight segments of ≥ 5 s, a few yaw-stick doublets, and ideally an `imu-raw-log`
build. Until then this proposal stays `Draft`.

**Gate 2, actuator lag (bench, props on, restrained):** step the yaw stick in
Stabilized at mid throttle and measure, from `engine_data`, the time from target eRPM
to 63 % of measured eRPM on each engine. Record it here.

- **Host tests:** new `crates/elle-control/tests/yaw_damper.rs`: washout settles to 0
  on a constant rate (steady turn); sign (nose-right rate → left engine slows, through
  `apply_differential_thrust_lut`); clamp at `YAW_DAMPER_MAX`; the pilot's stick still
  reaches full deflection; reset on mode entry; zero output with gain 0 (dart).
  Existing `elle-control` tests for both platforms still pass.
- **Replay / simulation:** none. `elle-replay` covers attitude estimation, not
  closed-loop dynamics.
- **Bench (props off):**
  - TEST_PLAN 1.2a (added), with a temporary build where the gain is above 0:
    - Stabilized: a nose-right twist slows the left engine; a nose-left twist slows
      the right engine.
    - The engines equalise within ~3 s once the aircraft is still.
    - Manual, and throttle under 5 %, give no damper output.
  - 2.2 / 2.4: existing rows still pass.
- **Flight:**
  - 0.5 extended: first flight with `YAW_DAMPER_GAIN` at half its planned value. Fly
    yaw stick doublets in Stabilized. In the ULog, compare the decay of `yaw_rate` with
    the gate 1 baseline, and check that `yaw_damp` is opposite in sign to the
    high-passed rate.
  - 6.5 heading hold: a 90° heading change completes. Compare the time with the
    pre-damper baseline if one exists.

## Rollback

Set `YAW_DAMPER_GAIN` to 0.0 for the eagle and rebuild. The module stays, the output is
identically 0, and `yaw_damp` logs 0. Nothing is stored in flash. Removing it entirely
is a revert of the implementing commit.

## Doc updates

- [ ] `docs/OPERATIONS.md` (Stabilized, AltitudeHold): with step 2, no behaviour changes before it
- [ ] `STATE_DIAGRAMS.md` (note on the Stabilized and AltitudeHold states): with step 2
- [x] `TEST_PLAN.md` (1.2a; the flight row comes with step 2)
- [x] `CLAUDE.md` (Controller section)
- [x] `crates/elle-ulog/README.md` (`controller` field and size)
- [x] `CHANGELOG.md`

## Open questions

1. Does the eagle have a lightly damped Dutch roll at all (gate 1)? Open: no eagle
   flight log yet (see the gate 1 result).
2. Is the governor plus EDF lag short enough (gate 2)?
3. Should the damper also run in a future rate or acro mode? Not in scope; none exists.
