# 0007: Host-buildable flight law

| | |
|-|-|
| Status | Accepted |
| Airframes | both (identical bit for bit by intent) |
| Date | 2026-11-11 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules)) |
| Surfaces | none |

## Motivation

The control law lives in `FlightController::update()` (`crates/elle-system/src/system.rs`).
It covers mode tracking, the arming gesture, setpoints and their filter, heading hold,
the PID, the yaw damper, mixing and thrust. The same function also drives hardware: it
writes `PwmOutputs`, reads `Instant::now()` and emits `elle_event!`. `elle-system`
builds only for thumbv8m, so the law can't run on the host. The pieces are tested on
the host (`elle-control/tests/`), but their wiring isn't: the order of the steps, which
state each mode resets, the failsafe hold, autotune override precedence, and the Core 1
health gate. That wiring has caused real bugs. The Core 1 verdict was being
overwritten every tick, and the PID integrators survived leaving Manual. Each fix is
now only a comment in `update()`.

A host-buildable law opens three things:

- a closed loop in `elle-replay --simulate`: the real controller flying the simulated
  airframe, instead of an open-loop attitude trace;
- replaying recorded pilot inputs through the controller that flies;
- host tests of the interplay between modes, failsafe, autotune and heading hold.

This proposal is only the move. Behaviour is unchanged and proven identical. Anything
the move makes possible comes later, under its own change.

## Behaviour delta

None.

| Situation | Before | After |
|-----------|--------|-------|
| Any pilot input, mode, arming, failsafe, autotune or heading-hold sequence | `update()` output | Same elevon command, engine output, `ControllerOutputSnapshot` and events, bit for bit (see Verification) |

The only intended differences are two timing shifts of a few µs on target, described
under Design → Clock: the clock read (ULog `controller.att_age_us`, the hold-expiry
edge) and the moment of the PWM write (`controller.pwm_counter_us`). Neither reaches a
control output. No section of `OPERATIONS.md` or `STATE_DIAGRAMS.md` changes.

## Design

### Split

```
elle-control::flight_law::FlightLaw        pure, host-tested
    step(&LawInputs, dt) -> LawOutputs
elle-system::FlightController              hardware and supervisor
    owns FlightLaw, PwmOutputs, watchdog, Core 1 health, RC link state
    update(commands, attitude):
        stamp dt_us, read now
        out = law.step(inputs, CONTROL_LOOP_DT)
        log the mode change, emit out.events
        last_output = out.write(&mut pwm, dt_us)   // PWM write, snapshot completed
```

### `FlightLaw` (new, `crates/elle-control/src/flight_law.rs`)

`FlightLaw` owns the state that `update()` touches today:

- `ArmingState`, `AttitudeController`, `YawDamper`
- `HeadingController`, `heading_hold_active`, `locked_heading_rad`
- the filtered pitch and roll setpoints, and `last_saturation`
- `last_attitude`, `held_ticks`, `attitude_dropped`
- `current_control_mode`, `setpoint_override`, `explicit_arming_only`, `gains_version`

`ControlMode` and `ControllerOutputSnapshot` move to `elle-control`, and `elle-system`
re-exports both so existing callers are unaffected. The law also keeps the previous
tick's snapshot, because the fast path and the failsafe hold carry unset fields over
from it.

Inputs, per tick:

```rust
pub struct LawInputs<'a> {
    pub now: Instant,                 // one clock read per tick
    pub commands: &'a PilotCommands,
    pub attitude: Option<&'a Attitude>,
    pub core1_healthy: bool,          // the supervisor's verdict, today `core1_health.is_healthy`
}
```

`Attitude` is `elle_hardware::imu::AttitudeData` moved to `elle-control`: the angles,
the rates and the `Instant` timestamp, with `zero()`. `elle-control` already depends on
`embassy-time`. `elle_hardware::imu::AttitudeData` is now a re-export of it, so Core 1
and `ATTITUDE` don't change. `is_attitude_valid` gets a pure twin,
`is_attitude_valid_at(attitude, max_age, now)`.

Outputs, per tick:

```rust
pub enum ElevonCommand {
    Trimmed { left_us: u32, right_us: u32 },  // set_elevons_with_trim (fast path and normal path)
    Safe,                                     // set_safe_positions (failsafe hold)
}
pub struct LawOutputs {
    pub elevons: ElevonCommand,
    pub snapshot: ControllerOutputSnapshot, // engines included; pulses, counter, dt_us not yet
    pub events: LawEvents,       // mode_change(from, to), attitude_held_ms, attitude_stale_ms
}
pub trait ElevonOutput {         // implemented by elle_hardware::pwm::PwmOutputs
    fn set_elevons_with_trim(&mut self, left_us: u32, right_us: u32) -> (u32, u32, u16);
    fn set_safe_positions(&mut self) -> u16;
}
impl LawOutputs {
    // Make the write, fill elevon_*_pulse_us, pwm_counter_us and dt_us.
    pub fn write<O: ElevonOutput>(&self, out: &mut O, dt_us: u32) -> ControllerOutputSnapshot;
}
```

`write` lives in `elle-control`, so the step from law output to the published
snapshot is shared code: the firmware and the oracle both run it.

`FlightController` keeps these jobs:

- the PWM hardware (`PwmOutputs`: trim, right-servo inversion, `pwm_counter_us`),
  driven through `out.write`, and the measured `dt_us` it passes in;
- emitting `EVT_ATTITUDE_HELD` and `EVT_ATTITUDE_STALE`, and logging the mode change;
- the watchdog, `check_core1_health` and the supervisor;
- `check_failsafe` and `RcLinkState`, with events `EVT_RC_*`. On loss it calls
  `law.signal_loss()`, which does today's `arming.signal_loss()`, PID reset and
  saturation reset. On restore it calls `law.signal_restored()`;
- `note_rc_packet` and `last_packet_time`.

These methods keep their signatures and delegate to the law: `arm`, `disarm`,
`set_explicit_arming`, `set_pid_gains`, `pid_config`, `gains_version`,
`apply_saved_gains`, `get_pid_gains`, the setpoint override, heading-hold
engage/disengage, and the `is_*` queries. `elle-app` doesn't change.

### Core 1 health

Today `check_core1_health` sets `attitude_controller.enabled = false` directly, and
`update_normalized` recomputes `enabled` as `mode && armed && is_healthy`. After the
move, the law computes `enabled` from `inputs.core1_healthy` alone, and
`FlightController` keeps no handle on the PID. The direct clear in
`check_core1_health` becomes a call to `law.disable_attitude()`, made at the same
point between ticks, because `is_attitude_enabled()` is read between ticks (RPC
status, autotune guard). The oracle (Verification) checks this ordering.

### `dt`

`step` takes `dt`, as asked. No arithmetic in `update()` uses the measured dt today:

- the PID integrates with `CONTROL_LOOP_DT` (`pid.rs:64`);
- heading hold uses `CONTROL_LOOP_DT` for its integral and slew (`heading.rs:79`,
  `heading.rs:85`);
- the setpoint filter uses `SETPOINT_FILTER_ALPHA` and a `CONTROL_LOOP_DT` step cap;
- the yaw damper's weight is fixed at construction.

So `dt` is the period the law integrates over, and `FlightController` passes the
constant `CONTROL_LOOP_DT`. That keeps every result bit for bit, and a simulation can
pass its own. Inside the law, `dt` replaces the `CONTROL_LOOP_DT` uses listed above:
`max_step`, plus PID and heading calls that take a dt argument. The constructor-time
alpha and damper weight stay as they are. With `dt == CONTROL_LOOP_DT` this computes
the same floats in the same order. The measured `dt_us` is never fed into the law's
maths: that would change behaviour and needs its own proposal.

### Clock

`update()` reads the clock four times. `Instant::now()` is read once for the `dt_us`
stamp and again for `att_age_us`. `elapsed()` is called in `is_attitude_valid` (hold
validity) and in the event text. After the move, `now` is read once and passed as
`inputs.now`. On target, `att_age_us` and the 20 ms (`ATTITUDE_HOLD_MAX_MS`)
hold-expiry edge can therefore shift by the few µs the old body took between reads.

The PWM write also moves. The old body wrote the elevons right after mixing, then
computed the thrust and the snapshot. Now the law computes all of that, the events
are emitted, and then `write` makes the PWM write. The write therefore lands a few
µs later in the tick, which shows in `controller.pwm_counter_us`. The pulses latch
at the frame wrap 1.5 ms later either way (`CONTROL_PWM_LEAD_US`), so they reach the
servos at the same moment.

These two are the only expected differences. The oracle runs on a mock clock that
doesn't advance within a tick, and its PWM stub returns counter 0, so it can't see
either one; they are stated here instead.

### Timing

The Manual fast path (`update_fast_path_raw`) is `#[inline(always)]` today, and it
moves across a crate boundary. The release profile has `lto = true` and
`codegen-units = 1`, so inlining should survive. The cost is measured, not assumed:
ULog `loop_stages` `Update` mean and max, before and after, on one airframe (bench,
Manual and Stabilized). The law adds no per-tick work. A one-tick `LawOutputs` copy
is the only new data movement.

### Memory

`FlightController` shrinks by what moves into `FlightLaw`, and `FlightLaw` lives
inside it, so the total is roughly unchanged. The controller is still lent `&mut`
from `boot::run` (Conventions: lend large structs).

## Interfaces

- **Events:** none new. `EVT_ATTITUDE_HELD`, `EVT_ATTITUDE_STALE` and `EVT_RC_*` are
  still emitted by `FlightController`, with the same conditions and text.
- **RPC:** none.
- **Flash:** none.
- **ULog:** none. `controller` and `commands` keep their fields. Only the timing
  notes above apply.
- **CRSF telemetry / LED:** none.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| A state transfer is missed in the move (PID reset on leaving Manual, failsafe resets, Core 1 gate, heading-hold disengage on disarm) | Wrong surface or engine output in an edge case | The oracle's scripted stream covers every branch, compared bit for bit |
| `enabled` is read between ticks with a different value than before | Autotune guard or RPC status disagrees for one tick | The oracle compares `is_attitude_enabled()` after every between-tick call, not only after `step` |
| Elevon write path chosen wrongly (Trimmed vs Safe) | Surfaces not centred in failsafe | `ElevonCommand` is part of the compared output |
| The fast path loses inlining | `Update` stage slower in Manual | `loop_stages` before and after; must stay within today's max |
| A partial port leaves two copies of the law | They drift | The move is one commit; the old body is deleted in it |

Sensor loss, RC loss, a Core 1 stall, a flash write and a reboot in the air all follow
today's paths. None of them is changed; the scripted stream exercises the first three.

## Verification

### The no-change proof

The proof uses the same recorded input stream, run through the law before and after
the refactor, on the same commit, and requires identical outputs.

**Oracle.** The refactor commit adds `crates/elle-control/tests/flight_law_equivalence.rs`.
It compiles `tests/flight_law_equivalence/legacy_system.rs`, a **byte-identical copy** of
`crates/elle-system/src/system.rs` from the base commit. The base commit's hash is
written in the test. A test checks the copy's SHA-256 against a constant taken from
`git show <base>:crates/elle-system/src/system.rs`, so the copy can't be edited into
agreement. It also includes today's `crates/elle-system/src/system.rs` (as `current`).
Both build on the host against the same shim modules, under the original paths:

- `elle_hardware::pwm::PwmOutputs`: the same trim and inversion arithmetic, recording
  each write and which path took it;
- `elle_hardware::elle_event!`: records the level, code and formatted message;
- `elle_hardware::imu::{AttitudeData, CORE1_HEARTBEAT, is_attitude_valid}`;
- `elle_hardware::watchdog`, `embassy_rp::watchdog::Watchdog`: no-ops.

Clock: embassy-time's `mock-driver` feature, present at the pinned rev `340bf1de`. It
is enabled as a dev-dependency feature only and advanced once per tick, so the old
and new code read the same time. The shims are the only hand-written part. They
replace hardware, not law, and each one is a few lines.

**Stream.** The stream is a fixture of per-tick records. Each record holds the
`PilotCommands` (Raw or Normalized), the attitude (or none), the time step, and the
calls `elle-app` makes between ticks:

- `arm`, `disarm`, `set_explicit_arming`;
- `set_pid_gains`, the override set and clear, heading engage and disengage;
- `check_failsafe` (driven by the packet age);
- Core 1 heartbeat loss and return;
- kill-switch ticks where `update` is skipped (`flight.rs`, `rpc.rs`).

There are two sources:

- **(a) Scripted.** A scenario that covers every branch: the RC arm gesture and its
  failsafe block; Manual ↔ Stabilized ↔ AltitudeHold; the Raw fast path; Normalized
  RPC commands with explicit arming; failsafe hold and recovery (no re-arm); autotune
  override; heading hold with and without a locked target or attitude; fresh, held,
  and expired attitude (both events, each once); saturation anti-windup; low-throttle
  integrator reset; Core 1 loss with the attitude controller forced off.
- **(b) From a flight.** A stream rebuilt from a real flight ULog. `commands` is
  logged at 100 Hz and held per tick; `attitude_data` supplies the attitude. The rebuild
  doesn't reproduce exactly what flew, because `commands` runs at half the tick rate.
  It gives realistic inputs, and both implementations get the same ones.

**Compare.** After every operation, not only ticks, it compares:

- the elevon writes (values, path and output pulses), in order;
- the events (level, code, message), in order;
- the whole `ControllerOutputSnapshot`, destructured field by field so a missed field
  doesn't compile;
- `engine_output()`;
- the queries: `is_armed`, `is_failsafe`, `is_attitude_enabled`,
  `current_control_mode`, `is_heading_hold_active`, `rc_link_state`,
  `rc_signal_age_ms`, `gains_version`, `pid_config`;
- `supervisor_check`'s return value. Floats are compared with `to_bits()`, with no tolerance.
It runs for both configs:

- `cargo test -p elle-control --target x86_64-unknown-linux-gnu`
- the same with `--features platform-dart`

**After Implemented.** The oracle, the copy and the shims are deleted. They pin the
old text, so any later intentional change to the law would have to delete them
anyway. The scripted stream stays, with the law's outputs from the proof run as a
golden file: a regression test that later changes update on purpose.

### Other checks

- **Host tests:** every existing `elle-control`, `elle-nav`, `elle-replay` and
  `elle-ulog` test passes unchanged.
- **Build:** CI as it is (both airframes × three feature sets, powerset, clippy
  `-D warnings`).
- **Bench:** TEST_PLAN 1.1 (elevon direction), 1.2 (differential thrust), 1.3
  (Stabilized PID direction) and 1.4 (arming gesture, kill switch) on one airframe.
  On the same airframe, compare `loop_stages` `Update` against a log from the base
  commit.
- **Flight:** none required by this change. The first flight after it reads the
  usual `controller` fields; nothing new to look for.

## Decisions

Both questions were settled on 2026-11-11.

1. **What counts as "recorded":** (a)+(b). The scripted stream covers the branches,
   and the stream rebuilt from a flight ULog adds realistic values. There is no
   on-target capture, so no new firmware and no ULog surface change.
2. **Base commit:** `3438ced` (master after this proposal merged). The oracle copies
   `crates/elle-system/src/system.rs` from it: blob `4d3f845f`, SHA-256
   `992eef7f5f758edd03a78b329143460d08ee5d522840757f2d0105ce5de2783d`. If master's
   `system.rs` changes before the refactor starts, the base moves to that commit
   and both hashes are taken again.

## As implemented

Implementation (branch `flight-law`) departs from the accepted text in these
ways. The sections above are corrected to match.

- **The oracle runs the new `system.rs` too.** The accepted design compared the old
  file with the new `FlightLaw`, which would have needed a hand-written copy of the
  new controller's glue in the test. Instead, `current` includes
  `crates/elle-system/src/system.rs` itself (the file the firmware builds) through the
  same shims as `legacy`. Both real files run every operation of the stream through
  their public API.
- **`ElevonOutput` and `LawOutputs::write` are in `elle-control`,** so the PWM write
  and the snapshot completion are shared code. There is no `engines` field: the
  engine values are in the snapshot, as `engine_output()` reads them.
- **`dt` plumbing is its own commit.** `AttitudeController::update_dt` and
  `HeadingController::update_dt` take `dt`; `update` passes `CONTROL_LOOP_DT`. The
  legacy copy calls `update` and so runs the same component code as the new law. The
  oracle therefore proves the wiring, not this change. The change only replaces the
  constant with a parameter, and the existing anti-windup tests cover the PID.
  Heading hold has no host test.
- **Event messages are compared,** not only their codes: the shim formats each
  `elle_event!` message, so the held/stale ages the law now computes are checked.
- **The streams.**
  - `scripted.stream`: 1,675 ticks written by `scenario.rs`. A test fails if the
    file and the scenario disagree. The run asserts that the stream reaches every
    event (13, 14, 15, 70, 71, 80, 83), the failsafe hold, mixer saturation, heading
    hold and running integrators.
  - `flight.stream`: 4,000 ticks (20 s) of dart `LOG_0052.ulg` from 1015 s, armed and
    flying in Stabilized. The converter that writes it is
    `tools/elle-replay/examples/law_stream.rs`. That log carries no raw RC channels,
    so this stream drives the normalized path only.
- **The oracle can fail.** `tests/flight_law_equivalence/mutate.py` applies 25 single
  mutations, one at a time, to the new law (`flight_law.rs`, including
  `LawOutputs::write`) and to the new `system.rs`, and runs the oracle on each. 24
  fail the test, on both airframe configs. Two groups:
  - **In the law:**
    - the resets on leaving Manual and on signal loss;
    - ignoring the Core 1 verdict, and `disable_attitude` doing nothing;
    - heading hold kept through normalized Manual or disarm;
    - the hold counters and the stale latch;
    - the override going through the filter;
    - the failsafe hold leaving the PID on or the engines running;
    - the low-throttle threshold;
    - the cached-attitude fallback;
    - explicit arming;
    - the damper reset;
    - wrong pulses for the Safe write, and `write` dropping `dt_us`.
  - **In `system.rs`:**
    - restore not clearing the failsafe;
    - loss not centring the elevons;
    - the Core 1 verdict not passed in;
    - the events emitted after the write instead of before;
    - the measured dt not passed;
    - the held event dropped.

  The 25th, the fast path not storing its mixer saturation, is equivalent: every way
  out of Manual resets that value before anything reads it. Writing the mutations
  exposed three gaps in the first scripted stream, now closed:
  - a re-arm by command at cruise throttle after a link loss;
  - normalized Manual with heading hold on;
  - throttle between 0.04 and 0.05.
- **The scenario uses `libm`'s `sinf`/`cosf`,** not std's (the system libm), so the
  committed stream regenerates bit for bit on any machine.
- **The flight window is the log's own:** the firmware logged `saturation = 9` (pitch-up
  and roll-left blocked) on all 4,000 ticks. The dart was holding a 20–54° bank with
  the left elevon at its 2000 µs stop. The oracle's runs show the same saturation.
- **Fixtures:** `scripted.stream` ~320 KB, `flight.stream` ~640 KB.
- **Limits.**
  - `YAW_DAMPER_GAIN` is 0 on both airframes (asserted), so the damper's state is
    invisible: its output is always ±0. The damper-reset mutation was caught only
    through the sign of that zero.
  - The mode-change message is a defmt log, not an event, so it isn't compared. The
    mode itself is, through `current_control_mode()`.
- **Observed, unchanged:** on the RC fast path (raw commands in Manual), heading hold
  stays engaged; only the normalized Manual path disengages it. This was already the
  behaviour, and the move keeps it.

## Rollback

Revert the refactor commit. It leaves nothing behind: no flash entries, no ULog,
ICD or event changes.

## Doc updates

- [ ] `docs/OPERATIONS.md`: none
- [ ] `STATE_DIAGRAMS.md`: none
- [ ] `TEST_PLAN.md`: none
- [ ] `CLAUDE.md`: Controller section (the law in `elle-control::flight_law`, what
  `FlightController` keeps), and `flight_law` in the `elle-control` crate list
- [ ] `crates/elle-ulog/README.md`: none
- [ ] `CHANGELOG.md`: `Unreleased` line, internal, no surface
