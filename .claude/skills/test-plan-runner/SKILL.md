---
name: test-plan-runner
description: Run a Part (or rows) of TEST_PLAN.md on the Elle flight controller through the `elle mcp` server tools: flash the right build, guide the operator through physical steps, verify with the firmware's own readings and events, check SD card logs, and record every result. Use when asked to run, continue or report on a test plan session.
---

# Running TEST_PLAN.md through `elle mcp`

The server's tools (`mcp__elle__*`) hold the debug probe. Which sections are auto,
assisted, log or manual, and which tools they use, is in TEST_PLAN.md's "Running it
through `elle mcp`" table: read it and the Part before starting.

## Before anything else

1. **Part 8 first** if it has no pass recorded (`test_report`, or ask): the server has
   not been validated on hardware, and every other result depends on it.
2. `link_status`. If `motors_allowed` is false, rows marked **motors** are skipped
   (record `skip`, note "server started without motors"). Never suggest adding the flag
   yourself; the operator decides, and only with props off.
3. Ask the operator which airframe, whether props are off, and whether the SD card is in
   the aircraft. Before any engine row, ask again that props are off, then pass
   `props_off_confirmed`.
4. `read build`: is the flashed build the one the Part asks for (profile, features,
   git)? If not, ask before `build_and_flash`: it replaces the firmware.

## Running a row

- **Say what the operator must do, then wait for it** with `wait_for` or `wait_event`
  (not by polling `read` in a loop): "Tilt the nose up about 20° and hold it."
  Timeouts of 30–60 s for a hand action; tell the operator the timeout.
- **Use `since_seq`**: note `events`' last `seq` before the action, so an old event
  cannot pass the row.
- **Check against the row's expected values**, not against what looks plausible.
  Quote the numbers (`pitch_deg` 19.4, event 10 at seq 812).
- **Manual rows**: ask the operator what they saw or heard; record their words.
- **Record each row as you go**, never in a batch at the end:
  `test_record(part, row, outcome, note, logs)`. `note` holds what was measured.
  `inconclusive` when the reading is ambiguous or the operator is unsure.
- **Stop at the first fail** in Parts 0, 1 and 6 and in anything listed under Abort
  Criteria. Say which row failed, what was expected and what was measured, and do not
  go on to the next row unless the operator asks.
- **Disarm** (`disarm`) after every engine row, even when it passed.

## Log rows

After the session (or when the operator pulls the card): `card_logs`, then
`copy_logs` for the session's files (pick them with `analyse_log list`: power
cycles and resets each start a new file). Then `analyse_log` and `replay` as the row
says. How to read the numbers: the `flight-logs` skill. Put the file names in `logs`.

## Flight builds

No RPC: after `build_and_flash` profile `flight`, verify with `log` / `wait_log` on the
defmt output (the event codes are in the text), or from the SD log afterwards.

## Ending

`test_report` for the day, then a short summary to the operator: passes, fails with
their measured values, skips and why, and the rows still to run. Ticking the `[ ]`
boxes in TEST_PLAN.md is the operator's call: offer, don't do it unasked.
