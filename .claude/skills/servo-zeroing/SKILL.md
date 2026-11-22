---
name: servo-zeroing
description: Zero the Elle elevon servos on the bench through the `elle mcp` tools, either as a re-zero (linkage installed, neutral drifted) or as a brand-new zero (new servo, horn, pushrod or airframe, set up from a true 1500 µs). Finds each elevon's flush pulse with the operator, checks throws and binding, writes ELEVON_LEFT/RIGHT_TRIM_US, reflashes and verifies. Use after a linkage, servo, horn or crash repair, when the elevons are not flush at neutral, or when installing servos.
---

# Zeroing the elevons through `elle mcp`

The trims are build constants (`ELEVON_LEFT_TRIM_US` / `ELEVON_RIGHT_TRIM_US`, the
airframe's `#[cfg]` pair in `crates/elle-config/src/lib.rs`, with a history comment
above it). This skill finds new values with the operator, writes them, reflashes and
verifies. You can't see the surfaces: every judgement is the operator's. Ask one short
question per step and wait for the answer.

## How the numbers work

- `set_elevons(left, right)` takes −100..=100, **1 unit ≈ 5 µs**. In Manual mode on a
  pure RPC build (`rpc-control,gnss`; `rpc-rc` ignores it), left = L and right = R
  through the mixer. Then the firmware adds the trim: left pulse = 1500 + 5·L + trim_L,
  right pulse = 1500 − 5·R + trim_R (the right servo is inverted).
- **Never do that sign arithmetic by hand for the result.** `read controller` returns
  `elevon_left_us` / `elevon_right_us`, the pulses actually written. **New trim = flush
  pulse − 1500**, per side, read back.
- **Hold:** disarmed, the firmware fails safe to center ~0.3 s after the host goes
  quiet. Always pass `hold_s` (≈60, max 120). Then `read controller` right after the
  set, while it is held. Re-issue the position before every question. Tell the
  operator once: "if a surface snaps back to center, the hold ran out; say so and I'll
  put it back."
- The mixer clamps each side at the **current** trim's stops (`ELEVON_*_MIX_MIN/MAX`).
  Before the reflash, a side only reaches `1500 + old_trim ± 500`, clipped to
  1000–2000. That's why the real throw check happens after the reflash.
- The trim is capped at ±`MAX_TRIM_US` (150 µs, with compile asserts). A flush pulse
  further out can be commanded but not stored: that means a linkage change.

## Never

- Arm, or use any engine tool (`arm`, `set_throttle`, `extend_armed`).
- Edit the other airframe's trims, or raise `MAX_TRIM_US` (it costs throw; that's the
  operator's call and a separate change).
- Flash without asking first (it replaces the firmware).
- Commit or leave trim edits in place whose post-flash throw check failed.

## 1. Setup (both modes)

1. `connect`, `link_status`, `read build`. The build must be the airframe's **pure
   RPC** profile (`rpc-control,gnss`, no `rpc-rc`). If not, ask, then
   `build_and_flash` profile `rpc`.
2. Ask: which airframe (must match `read build`'s platform)? Re-zero or brand-new
   zero? Props off or ESC power unplugged? Nothing here arms, but say so. Is a
   straightedge laid along the wing at each elevon root?
3. `set_mode manual`. Read the current trims from `elle-config` (the airframe's pair).
4. **Readback gate:** `set_elevons(0, 0, hold_s=60)`, `read controller`. Both pulses
   must be `1500 + current trim`. If not, **stop**: the flashed build doesn't match
   the source, and every later number depends on it. Offer to rebuild.

Then go to **Brand-new zero** or **Re-zero**.

## Brand-new zero (new servo, horn, pushrod or airframe)

The old trim means nothing here. Set the linkage up around the servo's true center so
the trim ends near 0 and no throw is lost.

- **N1. Unhook the pushrod** (clevis off the horn) on the side being set up. A new
  servo off center can drive the surface into a stop at full throw.
- **N2. True 1500 µs.** Command the offset that cancels the built-in trim:
  `L = −trim_L / 5`, `R = +trim_R / 5` (eagle 150 / −100 → `L = −30`, `R = −20`;
  dart 100 / −95 → `L = −20`, `R = −19`; recompute from the source). Hold it,
  `read controller`: both must read **1500**. If not, move by ±1 unit until they do.
  Quote it. This offset command, not `set_elevons(0, 0)`, is "center" for the rest of
  this mode.
- **N3. Horn and linkage.** At true 1500, the operator fits the horn as near
  perpendicular to the pushrod as the spline allows. They set the clevis or pushrod
  length so the surface is flush, then hook it up. Aim for less than ~25 µs of trim
  left over. If the spline step leaves the surface badly off, suggest the horn's other
  arm or flipping it 180° (a different spline offset) before adjusting the clevis.
- **N4. Direction and ends.** Run step R2 with the N2 offsets added. Then go to the
  reachable ends on each side, held, and ask about servo buzz, a bending pushrod, the
  surface touching the wing or the horn touching the frame. Fix any binding before
  going on.
- Then do **R3 to R8**, with the search starting from the N2 offsets instead of 0.
  The result (flush pulse − 1500) is relative to true center, whatever the old trim
  was. The history comment says "new install: <servo / horn / linkage>".

## Re-zero (linkage installed, neutral drifted)

- **R2. Direction.** `set_elevons(+20, 0, hold_s=60)`: "Did the left elevon go up or
  down?" Then `set_elevons(0, +20, hold_s=60)` for the right. Record it. Turn the
  operator's "higher / lower" into a step sign from this alone, never from the
  comments in `elle-config` (the dart's right comment, "Raises right elevon" on −95,
  has misled before).
- **R3. Search, one side at a time**, the other side at its center (0, or its N2
  offset).
  - Start at center. Ask: "Left elevon against the straightedge: higher, lower, or
    flush?"
  - Steps of 10 units (50 µs) until the answer flips, then halve (5, 2, 1). Every
    step is one `set_elevons(..., hold_s=60)`, one `read controller` (quote the pulse)
    and one question.
  - Done when the operator says flush, or "higher" and "lower" are 1 unit apart: then
    ask which is closer. 5 µs is the floor (servo deadband ~2–4 µs).
  - **Backlash check:** go 30 units the other way, come back to the found value,
    ask again. If the answer changes, approach from the same side both times, and
    note the slop (in units) for the linkage.
  - Repeat for the other side.
- **R4. Compute.** `trim = flush_pulse_us − 1500` per side, from the readback.
  - |trim| > 150: **stop this side.** Say how many µs over it is and which way it sat
    (surface high or low at the cap). Suggest the linkage fix: clevis turns, the next
    horn hole, the horn's other arm, or a spline step. Redo R3 after the change.
  - Report the mixer stops the new trims give: left max = min(1, 1 − trim_L/500),
    left min = max(−1, −1 − trim_L/500), right max = min(1, 1 + trim_R/500), right
    min = max(−1, −1 + trim_R/500). A large trim's lost throw is then visible.
- **R5. Provisional binding sweep** (old trim still flashed). Each side to ±100,
  held: "Any buzz, a bending pushrod, or the surface touching something?" This only
  catches gross binding before a reflash; the throw check is R7.
- **R6. Write (leave uncommitted).**
  - Edit the airframe's `ELEVON_LEFT_TRIM_US` / `ELEVON_RIGHT_TRIM_US`. Keep the
    inline `// + raises…` comments matching what R2 found.
  - Add a line to the history comment above: date, old → new, "flush on the bench
    (servo-zeroing)", re-zero or new install, backlash noted.
  - If the eagle's trims changed, update the quoted mixer stops ("left 0.7 / right
    0.8") in the `ELEVON_*_MIX_MAX` comment in `elle-config/src/lib.rs` and in
    `CLAUDE.md` (Controller section). Grep `docs/` for them too.
  - Add a `CHANGELOG.md` `Unreleased` / `Changed` line: "<Airframe> elevon trims left
    a → b µs, right c → d µs (tuning, <airframe> only): flush on the bench
    (re-zero / new install). Upward/downward travel now stops at mixer …".
  - `cargo test -p elle-control --target x86_64-unknown-linux-gnu`, plus
    `--features platform-dart` for the dart. The anti-windup and equivalence tests
    read the trim.
- **R7. Reflash, verify, throw check.** Ask, then `build_and_flash` the RPC profile.
  `set_mode manual`, `set_elevons(0, 0, hold_s=60)`. The operator confirms both
  flush; `read controller` shows `1500 + new trim` on both.
  - **Throw check:** each side at −50, +50, −100, +100, held. "Reaches without buzz
    or bending? Up and down about the same distance by eye?" Then both together at
    full pitch (`±100, ±100`) and full roll (`+100, −100` and `−100, +100`).
  - A buzz at an end means binding. The advice is less mechanical throw (an inner
    horn hole or an outer control-horn hole), not more trim.
  - Release the hold (`set_elevons(0, 0)` without `hold_s`) and wait a second. Both
    must still be flush: failsafe drives the trimmed center.
  - **Anything failed:** revert the R6 edits (`git checkout` those files after
    showing the diff), say what failed with the numbers, and give the linkage advice.
- **R8. Finish.** Summarise per side: old → new trim, flush pulse, backlash, throw
  answers. Remind the operator to flash the **flight** build before flying, and that
  a big neutral change can move the flown trim (see the `elle-config` history). Offer
  a check in level Stabilized flight on the next sortie (`flight-logs` skill:
  `controller` elevon pulses while holding level). Don't commit unasked; commit text
  carries no AI attribution.
