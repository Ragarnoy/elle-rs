# NNNN: Title

| | |
|-|-|
| Status | Draft |
| Airframes | eagle / dart / both |
| Date | YYYY-MM-DD |
| Version impact | breaking / minor / patch ([rules](../VERSIONING.md#bump-rules)) |
| Surfaces | operator behaviour / RPC ICD / flash profile / ULog / event codes / none |

## Motivation

What problem this solves, and the evidence for it (flight log, bench run, TODO item).

## Behaviour delta

What the pilot or operator sees, before → after. Write it against
[`OPERATIONS.md`](../OPERATIONS.md) and [`STATE_DIAGRAMS.md`](../../STATE_DIAGRAMS.md):
which sections and transitions change.

| Situation | Before | After |
|-----------|--------|-------|
| | | |

## Design

- Crates and files touched (behaviour in `elle-app` or below, never in a binary)
- New pure logic in `elle-control` / `elle-nav`, with host tests
- New constants in `elle-config`, per platform where needed, with their
  `const _: () = assert!(…)` invariants
- Timing: anything running per tick, its cost, and what it means for the 5 ms loop or
  Core 1's 1 ms budget

## Interfaces

- **Events:** new `EVT_*` codes, host label (`tools/elle-rpc-host/src/events.rs`),
  OPERATIONS.md row
- **RPC:** endpoints or types added or changed
- **Flash:** `ProfileEntry` keys or layouts (and `MAP_KEY_SLOTS`)
- **ULog:** messages or fields added or changed
- **CRSF telemetry / LED:** anything the pilot sees in flight

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| | | |

Include what happens on: sensor loss, RC loss, a Core 1 stall, a flash write, a
reboot in the air.

## Verification

- **Host tests:** which, new or existing
- **Replay / simulation:** `elle-replay` runs, `--simulate` scenarios
- **Bench:** TEST_PLAN rows, new or existing (numbers)
- **Flight:** TEST_PLAN rows, and what to look for in the ULog

## Rollback

How to turn it off without a code change if possible (feature flag, constant), and
otherwise which commit to revert. Note anything a rollback leaves behind, such as
flash entries.

## Doc updates

- [ ] `docs/OPERATIONS.md`
- [ ] `STATE_DIAGRAMS.md`
- [ ] `TEST_PLAN.md`
- [ ] `CLAUDE.md`
- [ ] `crates/elle-ulog/README.md`
- [ ] `CHANGELOG.md`
