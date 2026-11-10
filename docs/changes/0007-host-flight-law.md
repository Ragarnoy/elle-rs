# 0007: Host-buildable flight law

| | |
|-|-|
| Status | Draft |
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

The only intended difference is the clock-read timing described under Design →
Clock. It shows in ULog `controller.att_age_us` and never in a control output. No
section of `OPERATIONS.md` or `STATE_DIAGRAMS.md` changes.

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
        PWM write per out.elevons -> pulses, pwm_counter_us
        emit out.events
        fill ControllerOutputSnapshot
```

### `FlightLaw` (new, `crates/elle-control/src/flight_law.rs`)

`FlightLaw` owns the state that `update()` touches today:

- `ArmingState`, `AttitudeController`, `YawDamper`
- `HeadingController`, `heading_hold_active`, `locked_heading_rad`
- the filtered pitch and roll setpoints, and `last_saturation`
- `last_attitude`, `held_ticks`, `attitude_dropped`
- `current_control_mode`, `setpoint_override`, `explicit_arming_only`, `gains_version`

`ControlMode` moves to `elle-control`, and `elle-system` re-exports it so existing
callers are unaffected.

Inputs, per tick:

```rust
pub struct LawInputs<'a> {
    pub now: Instant,                 // one clock read per tick
    pub commands: &'a PilotCommands,
    pub attitude: Option<&'a Attitude>,
    pub core1_healthy: bool,          // the supervisor's verdict, today `core1_health.is_healthy`
}
```

`Attitude` is the pure half of `elle_hardware::imu::AttitudeData`: the angles, the
rates and the `Instant` timestamp. `elle-control` already depends on `embassy-time`.
`AttitudeData` becomes a re-export or a `From`, so Core 1 and `ATTITUDE` don't change.
`is_attitude_valid` gets a pure twin that takes `now`.

Outputs, per tick:

```rust
pub enum ElevonCommand {
    Trimmed { left_us: u32, right_us: u32 },  // set_elevons_with_trim (fast path and normal path)
    Safe,                                     // set_safe_positions (failsafe hold)
}
pub struct LawOutputs {
    pub elevons: ElevonCommand,
    pub engines: (u16, u16),
    pub snapshot: LawSnapshot,   // every ControllerOutputSnapshot field the law computes
    pub events: LawEvents,       // attitude_held, attitude_stale (with age), mode_changed(from, to)
}
```

`FlightController` keeps these jobs:

- the PWM write: trim, right-servo inversion, `pwm_counter_us`;
- the snapshot fields that come from that write (`elevon_*_pulse_us`,
  `pwm_counter_us`) and the measured `dt_us`;
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
`inputs.now`. On target, `att_age_us` and the 50 ms hold-expiry edge can therefore
shift by the few µs the old body took between reads. That is the only expected
difference. The oracle runs on a mock clock that doesn't advance within a tick, so
it can't see this; it is stated here instead.

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
- **ULog:** none. `controller` and `commands` keep their fields. Only the clock note
  above applies.
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
agreement. The copy builds on the host against shim modules with the original paths:

- `elle_hardware::pwm::PwmOutputs`: the same trim and inversion arithmetic, recording
  each write and which path took it;
- `elle_hardware::elle_event!`: records the event codes;
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

**Compare.** Every tick compares the elevon write (value and path), the engine output,
the whole `ControllerOutputSnapshot`, the event codes with their counts, and the
queries (`is_armed`, `is_failsafe`, `is_attitude_enabled`, `current_control_mode`,
`is_heading_hold_active`). Floats are compared with `to_bits()`, with no tolerance.
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

## Open questions

1. **What counts as "recorded".** Existing ULogs can't give the exact per-tick input
   stream: `commands` is 100 Hz, and the 16 raw channels the fast path reads aren't
   logged. The options are (a)+(b) above, or an on-target capture: a feature that logs
   every `update()` call's arguments and the between-tick calls. That capture is extra
   firmware and a ULog surface change. **Recommendation: (a)+(b).** The scripted
   stream is what covers the branches, and (b) adds realistic values.
2. **Base commit.** The oracle copies `system.rs` from the commit the refactor starts
   on. PR #45 (proposal 0006, yaw sign) doesn't touch `system.rs`, so its merge order
   doesn't matter to the proof, but the base hash must be recorded.

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
