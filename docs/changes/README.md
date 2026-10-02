# Change proposals

A short written proposal comes before a change that could hurt the aircraft or
surprise the pilot. Its purpose: settle what changes and how it gets proven *before*
writing code, and leave a record of why. Once the change is verified, the proposal is
folded into the docs that describe current behaviour and archived.

## When one is required

- Arming, disarming, failsafe or the kill switch
- Flight modes: adding one, or changing what one does
- Anything that newly moves a control surface or an engine: navigation leaving
  observation mode, turning on turn compensation (`AHRS_TURN_COMP`), a new
  autotune behaviour
- Any **breaking** change under [`VERSIONING.md`](../VERSIONING.md#compatibility-surfaces)

Anything else may have one when it helps; most changes don't need it.

## Lifecycle

| Status | Meaning |
|--------|---------|
| `Draft` | Being written; open questions remain |
| `Accepted` | Design and verification agreed; implementation can start |
| `Implemented` | Merged; host tests pass; hardware verification still due |
| `Verified` | The listed TEST_PLAN rows passed on hardware |

On **Verified**:

1. Fold the behaviour delta into [`OPERATIONS.md`](../OPERATIONS.md) and
   [`STATE_DIAGRAMS.md`](../../STATE_DIAGRAMS.md) (and `CLAUDE.md` if the
   architecture changed). Those docs, not the proposal, describe current
   behaviour.
2. Make sure the `CHANGELOG.md` entry exists.
3. Move the file to `docs/archive/changes/`.

A proposal that is dropped gets `Status: Withdrawn` and a line saying why, and is
archived the same way.

## Writing one

Copy [`TEMPLATE.md`](TEMPLATE.md) to `NNNN-short-name.md`. Number in sequence across
this folder and the archive; a number is never reused. Keep it short: a section that
doesn't apply gets "None", not padding.

## Open proposals

| # | Title | Status |
|---|-------|--------|
| [0001](0001-eagle-yaw-damper.md) | Yaw damper on the eagle | Accepted: code inert (gain 0); enabling waits for an eagle flight log (gate 1) |
| [0002](0002-esc-config-over-rpc.md) | AM32 ESC configuration over RPC | Draft: bootloader entry to confirm on the bench |
| [0003](0003-board-orientation.md) | Per-airframe board orientation | Implemented: bench 0.1.2 / 0.1.4 / Part 1.3 on the eagle due |
