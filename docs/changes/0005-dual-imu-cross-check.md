# 0005: Dual-IMU cross-check

| | |
|-|-|
| Status | Draft |
| Airframes | both (hardware: Rev B PCB, both eagle and dart) |
| Date | 2026-10-05 |
| Version impact | minor |
| Surfaces | ULog, event codes |

## Motivation

Rev B of the PCB adds a second ICM-45686 (`U6`) on the shared SPI0 bus, with its own
chip-select/interrupt (GPIO6/GPIO7) and a shared `CLKIN` so both IMUs' sample clocks
stay locked together. This is a hardware-side proposal already captured in the PCB
repo; this document is the firmware side — what, if anything, to do with the second
sensor, and why it needs a proposal before any code is written.

Today the attitude path is entirely single-sensor: one `AttitudePipeline`, one gyro
bias estimate, one `ATTITUDE` `SignalCache`, one `IMU_STATUS.calibrated` flag.
`check_core1_health` only detects Core 1 dying outright — nothing detects a single
IMU feeding bad data (stuck FIFO, a sensor fault short of a bus error) while Core 1
keeps running.

Three options were considered for what the second IMU is for:

1. **Cross-validation / fault detection** — run both IMUs' AHRS independently,
   compare attitude each tick, flag divergence beyond a threshold. No combined
   estimate, no voting; lowest integration risk.
2. **Sensor fusion / voting** — combine both into one attitude estimate with a
   disagreement policy. Highest value, highest complexity and failure-mode surface
   (what happens when they disagree, does a bad second IMU drag down a good first
   one).
3. **Ground-truth reference only, not flown** — use the second IMU purely during the
   already-planned turn-compensation data flights (`docs/ATTITUDE.md`, TEST_PLAN
   Part 7) as an independent check, without it ever being a flight dependency.

This proposal is for **option 1**. It's the smallest change that gets real value
(catching a single-IMU fault that the current heartbeat check can't see) without
taking on a combined-estimate failure mode. Option 3 can be done as a one-off
replay/log-analysis exercise without firmware changes once raw dual-IMU logging
exists, and doesn't need its own proposal. Option 2 is explicitly **not** proposed
here — if cross-validation data from flights on this design shows the two IMUs
agree well and a case emerges for combining them, that's a separate future proposal.

Not a fix for vibration or EMI: both ICM-45686 sit on the same rigid board a few mm
apart and will see essentially the same vibration spectrum, so this doesn't reduce
vibration-induced attitude error — that's mechanical isolation work, tracked in the
PCB repo, unrelated to this proposal.

## Behaviour delta

No change to arming, failsafe, modes, or anything the pilot commands. The only
pilot-visible effect is a new failsafe trigger and its LED/event signature.

| Situation | Before | After |
|-----------|--------|-------|
| Second IMU disagrees with the primary beyond threshold, while armed | Nothing detects this; flight continues on a possibly-bad attitude | New event fires; disarms via the existing attitude-controller-off path (same effect as `check_core1_health` losing the heartbeat today) |
| Second IMU fails to initialize at boot, or its FIFO stalls | Not detected (no second IMU exists today) | Logged (new event), does not block arming on its own — primary IMU is still the sole input to control; this is detection, not redundancy |

## Design

- New pure logic in `elle-control`: a small comparison module (e.g.
  `elle_control::imu_crosscheck`) that takes both pipelines' attitude output and
  returns a disagreement metric, host-tested the same way `AttitudePipeline` is.
  Keeps the actual threshold/decision logic out of `elle-hardware`, consistent with
  "new pure logic goes in `elle-control`."
- `elle-hardware/src/imu/driver.rs`: `imu_task` grows a second `AttitudePipeline`
  instance driven by `U6` over the same blocking SPI0, its own CS/INT per the PCB
  design. **Does not feed the second IMU into control** — its output is compared and
  logged only; `ATTITUDE` (the `SignalCache` the control loop reads) stays sourced
  from the primary IMU exactly as today.
- Gyro bias: the second IMU needs its own bias measurement over the same first-still-
  second window, independent of the primary's.
- Timing: this is the main open risk. Two blocking SPI0 devices drained on Core 1 at
  1 kHz must both fit the existing budget — check against
  `elle_hardware::timing::CORE1_LOAD` (longest FIFO drain, mag/baro read durations)
  before committing to running both every tick. If the full-rate second pipeline
  doesn't fit, consider running its AHRS at a decimated rate (it's a cross-check, not
  a control input, so it doesn't need 1 kHz) — but get a timing measurement first
  rather than assume.
- Constants: a new `elle-config` entry for the divergence threshold (degrees),
  per-platform if the two airframes' mounting turns out to need different values.

## Interfaces

- **Events:** two new codes in the next free range (170+): divergence detected
  while armed (disarm path), and second-IMU init/stall failure (log-only). Add the
  host label in `tools/elle-rpc-host/src/events.rs` and a row in
  `docs/OPERATIONS.md`'s event table.
- **RPC:** none planned — this is log/event only, no new query endpoint unless a
  review decides a live secondary-attitude readout is useful for bench debugging.
- **Flash:** none.
- **ULog:** new message carrying the second IMU's attitude and the divergence
  metric, at a rate to be decided once the Core 1 timing check is done (likely
  decimated, not 1 kHz, matching `attitude_data`'s existing 50 Hz pattern under
  `imu-raw-log`).
- **CRSF telemetry / LED:** divergence-while-armed disarm reuses the existing
  Core-1-health-loss LED pattern; no new pattern needed unless review wants one to
  distinguish the two causes.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| Second IMU's SPI reads blow the 1 kHz Core 1 budget | Late or dropped primary-IMU samples, degraded control | Measure against `CORE1_LOAD` before flight; decimate the second pipeline's rate if needed |
| Divergence threshold too tight | Nuisance disarms on legitimate vibration/transient disagreement | Tune from bench + flight log data before enabling the disarm path; log-only first, gate the disarm behind a flight log review (same gated-rollout pattern as the yaw damper, proposal 0001) |
| Divergence threshold too loose | Fails to catch a real single-IMU fault, false sense of coverage | Bench-fault-inject (disconnect/short one IMU under test) before trusting the detector |
| Second IMU's gyro bias measurement runs into the same first-still-second window and takes longer than the primary's | Arming delayed, or proceeds with one bias unmeasured | Confirm both complete within the existing arming timing before this reaches flight |

## Verification

- **Host tests:** `elle_control::imu_crosscheck` divergence metric, pure function,
  deterministic cases (agreement, injected offset, injected noise).
- **Replay / simulation:** not directly applicable (`elle-replay` takes one IMU's
  raw log); if useful, a follow-up could feed two recorded raw logs through the
  comparison logic offline, but that's not required to land this.
- **Bench:** new TEST_PLAN row — both IMUs initialize, agree within threshold at
  rest and under hand-applied motion, `core1_load` stays within budget with both
  running, a bench fault injection (block one IMU's CS or starve its FIFO) raises
  the new event.
- **Flight:** log-only first (no disarm path enabled) for at least one flight on
  each airframe to collect real divergence data before gating the disarm behind a
  threshold, per the risk table above.

## Rollback

Both the comparison task and the disarm path are gated behind an `elle-config`
constant (divergence threshold, or a dedicated enable flag mirroring
`AHRS_TURN_COMP`'s `Off` default). Set it to a disabled/inert value to turn the
disarm path off without a code change. The second IMU's driver init and logging can
stay running even with the disarm path off — it's additive and doesn't touch
control. Full rollback otherwise reverts the commit; nothing is written to flash.

## Doc updates

- [ ] `docs/OPERATIONS.md` (new event rows)
- [ ] `STATE_DIAGRAMS.md` (new disarm trigger, if the gated rollout reaches that stage)
- [ ] `TEST_PLAN.md` (new bench row)
- [ ] `CLAUDE.md` (Core 1 task list, IMU pipeline section)
- [ ] `crates/elle-ulog/README.md` (new message)
- [ ] `CHANGELOG.md`
