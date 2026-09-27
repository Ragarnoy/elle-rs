---
name: governor-calibration
description: Recalibrate the RPM governor for eagle or dart by running the embassy-dshot `rpm_range` EDT sweep on the hardware, then rewriting GOVERNOR_FF_TABLE and the ceiling constants (MAX_RPM, MAX_ERPM, GOVERNOR_DSHOT_MAX) from the measured curve. Use after a prop, motor, ESC or battery change, or when the governor table is marked stale.
---

# RPM governor calibration

The governor (`elle-control/src/governor.rs`) is feedforward from
`GOVERNOR_FF_TABLE` plus a PI trim. The table maps eRPM to DShot, measured on this
airframe **under its real load**. When the prop or EDF changes, the table is wrong
and the PI has to cover the whole difference. This skill re-measures the curve and
rewrites the table.

What gets updated:

| Item | File | Meaning |
|---|---|---|
| `GOVERNOR_FF_TABLE` | `crates/elle-config/src/lut.rs` | measured (eRPM, DShot) curve; the first row (DShot 48) is the **min** end |
| `MAX_RPM` / `MAX_ERPM` | `crates/elle-config/src/lib.rs` | **max**: the full-stick target |
| `GOVERNOR_DSHOT_MAX` | `crates/elle-config/src/lib.rs` | the DShot ceiling the PI may never exceed; a compile-time assert requires it to equal the table's last row |

All three are `#[cfg(feature = "platform-dart")]`-split. Only edit the block for the platform being calibrated.

## 1. Confirm the setup with the user before anything spins

The sweep takes the motor to **full throttle** (DShot 48 → 1998, about 40 s per engine).
Ask the user to confirm, and do not flash until they have:

- **Prop/EDF fitted**, aircraft **restrained** and clear of people. The
  `rpm_range` banner says "remove propeller". Ignore that here: a table swept with
  no prop fitted doesn't describe the flying load and is useless for the governor.
- **Charged pack.** Pack voltage shapes the whole curve. Record it (the sweep
  prints it), because the table is only right near that voltage.
- **Which platform, which prop.** This goes into the table's doc comment.
- The probe is connected to the flight controller. The sweep **replaces the elle
  firmware**, so it has to be reflashed afterwards (step 6).

## 2. Run the sweep

`run_sweep.sh` patches the example's pin and spin direction, runs it, stops probe-rs
when the sweep finishes, and restores the example source (it refuses to run if
`rpm_range.rs` has uncommitted edits). Write logs to the scratchpad.

| Platform | Engines | Pin | Direction (`ENGINE_SPIN_REVERSED` in `elle-config/src/lib.rs`) |
|---|---|---|---|
| dart | 1 | 14 | check the constant: currently `reversed` |
| eagle | left | 14 | check the constant: currently `normal` |
| eagle | right | 11 | same as left |

Always read `ENGINE_SPIN_REVERSED` rather than trusting this table. The wrong
direction on a handed prop measures the wrong load curve without any error.

```sh
S=.claude/skills/governor-calibration
SCRATCH=<session scratchpad dir>
$S/run_sweep.sh 14 reversed $SCRATCH/dart.log            # dart
$S/run_sweep.sh 14 normal   $SCRATCH/eagle_left.log      # eagle, then:
$S/run_sweep.sh 11 normal   $SCRATCH/eagle_right.log
```

Run it in the background and wait for it to finish. For eagle, confirm with the
user again between engines. Let the ESCs cool and swap in a fresh pack if the first
run drained it: both engines should be swept at the same voltage.

If a run fails, read the log. `No valid RPM readings` or a wall of
`(unstable)` steps means no bidirectional telemetry is coming back (wiring, ESC
firmware, wrong pin). Don't write a table from that.

## 3. Parse

```sh
python3 $S/parse_sweep.py $SCRATCH/dart.log
python3 $S/parse_sweep.py $SCRATCH/eagle_left.log $SCRATCH/eagle_right.log
```

It prints each engine's peak, a ready-made `GOVERNOR_FF_TABLE`, and the three
constants. The rules it applies (see its docstring):

- **Ceiling** = the lowest engine's peak-RPM step. Above a peak, more throttle
  means *less* RPM, and a PI that winds into that region runs away (the 2026-07
  windup bug). A curve still rising at DShot 1998 has no peak, so the ceiling is
  the last step.
- Rows every 100 DShot from 48, plus the ceiling; twin engines averaged.
- The last row is pinned to `MAX_ERPM` = the slowest engine's RPM at the ceiling ×
  pole pairs, so full stick targets what both engines can actually reach.

Before writing anything, sanity-check the numbers against the current table and
show the user the comparison. A different prop moves the curve by roughly 10–30%.
A 3× change, or a curve that isn't rising, means a bad sweep, not a new prop.

## 4. Update the code

In `crates/elle-config/src/lut.rs`, replace **only this platform's**
`GOVERNOR_FF_TABLE` with the parsed one, and rewrite its doc comment: date, prop,
pack voltage, "rpm_range sweep, 200 settle + 500 measured samples per step",
where the ceiling came from (peak, or still rising at 1998). Remove any
`STALE` note the new data replaces.

In `crates/elle-config/src/lib.rs`, in this platform's cfg block, set `MAX_RPM`
and `GOVERNOR_DSHOT_MAX` to the parsed values (`MAX_ERPM` is derived from
`MAX_RPM`; just check its trailing `// = ...` comment). Rewrite the motor-spec
comment above `MAX_RPM` and the `GOVERNOR_DSHOT_MAX` doc comment to match the
new data. Any history they carry (prop, date, pack voltage, why the ceiling sits
where it does) must describe the sweep you just ran.

Dart only: `crates/elle-control/tests/governor_step_response.rs` hardcodes
`MAX_ERPM` "matching `MAX_ERPM` for the dart". Update it too.

## 5. Verify

```sh
(cd crates/elle-dart  && cargo build --release)   # compile-time assert: GOVERNOR_DSHOT_MAX == last table row
(cd crates/elle-eagle && cargo build --release)
cargo test -p elle-control --target x86_64-unknown-linux-gnu --features platform-dart
cargo test -p elle-control --target x86_64-unknown-linux-gnu
```

CI runs the same `elle-control` tests for both platforms, so a stale test plant fails
the PR.

If a governor test fails, work out whether the test's plant model assumed the old
curve before touching the test. Don't loosen a test just to make it pass.

## 6. Reflash and hand back

The board is still running `rpm_range`. Reflash the platform's firmware (see
CLAUDE.md for build commands; with `cargo run --release` from the crate
directory). Then bench-check the governor at a few stick positions before flying.

Update the ceilings in `CLAUDE.md` ("DShot and RPM governor") and the dart prop memory
if the calibration changes what they say. Don't commit unless the user asks.
