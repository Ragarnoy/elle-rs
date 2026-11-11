# 0002: AM32 ESC configuration from the TUI and MCP

| | |
|-|-|
| Status | Accepted |
| Airframes | both (eagle: two ESCs; dart: one) |
| Date | 2026-10-02, rewritten 2026-11-11 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules), 0.x): additive only (new endpoints, new event codes, a new RPC-build mode); no existing type or code changes; flight builds unchanged |
| Surfaces | operator behaviour (ESC-config mode in RPC builds), RPC ICD (endpoints added), event codes (170–177 added) |

The first draft (2026-10-02) used a separate `esc-config` build. This rewrite uses the
normal `rpc-control` build, rebooted into an ESC-config mode, and builds on the
esc-config-rs crates as they now stand (`esc-session`, `esc-onewire-rp`,
`esc-config-icd`).

## Motivation

Reading or changing an AM32 ESC's settings today means unplugging its signal lead from
the FC and connecting the separate esc-config-rs relay (an RP2350 that bridges USB to
the ESC's bootloader). On the eagle that is two ESCs inside a closed airframe. As a
result ESC settings go unchecked, although several parts of the firmware depend on
them:

- `GOVERNOR_FF_TABLE` was swept against one ESC configuration (timing, PWM frequency,
  motor KV, direction). A changed ESC setting invalidates the table without any
  visible sign. The dart's re-sweep after the prop swap is the precedent.
- Spin direction is asserted by DShot command every boot (`ENGINE_SPIN_REVERSED`, no
  `SettingsSave`) because the EEPROM setting is not known. That stays, but it could
  then confirm a stored setting instead of being the only place the direction is set.
- The ESC link work (`EscLink`, `EdtWatch`, events 160–165) would be easier to
  diagnose with the ESC's EEPROM and firmware name at hand.

The bench session is already in the TUI (or `elle mcp`) with the probe attached.
Configuring the ESCs from there, with no reflash and no re-cabling, is what makes it
routine. An agent running TEST_PLAN through `elle mcp` should be able to read and
snapshot ESC settings too, for example next to a governor sweep.

## Behaviour delta

| Situation | Before | After |
|-----------|--------|-------|
| Flight build | — | Unchanged. None of this code is compiled in. |
| `rpc-control` build, normal boot | — | Unchanged, except that a new `EnterEscConfig` RPC exists. |
| Read or change ESC settings | Unplug the ESC lead, connect the relay, run `esc-configurator` | TUI ESC panel (replaces the main panel in ESC-config mode), `elle direct esc …`, or `elle mcp` `esc_*` tools |
| `EnterEscConfig` while disarmed, both eRPM targets 0 | — | Event 170, MCU reboots into ESC-config mode; the host reattaches |
| `EnterEscConfig` while armed, or a target ≠ 0 | — | Refused (event 171), nothing happens |
| ESC-config mode, boot | — | No DShot program is loaded and no DShot is ever sent. The engine pins idle high (one-wire UART). The LED shows a distinct pattern. About 2 s later each ESC, seeing no signal, resets itself into its bootloader and stays there because the line is high (see [Bootloader entry](#bootloader-entry)). No power cycle: the battery stays connected (it also powers the FC through the ESCs' 5 V). |
| ESC-config mode, flash-writing RPCs (PID save, calibrations, ULog erase) | — | Refused (event 63, as when armed), so no flash operation can pause the ESC link |
| ESC-config mode, `Arm` / RC arming gesture / throttle | — | Arming refused (event 172); throttle has no output path |
| ESC-config mode, elevons | — | As in any RPC build: `SetElevons` and Stabilized still move surfaces disarmed. Not changed. |
| ESC-config mode, `ExitEscConfig` | — | Each connected ESC is sent Run (app starts), then the MCU reboots normally |
| ESC-config mode, any other reset or power cycle | — | Normal boot (the flag is gone). The ESCs should start their app on their own (see Open questions 2). |

OPERATIONS.md: new section "ESC configuration (bench)". STATE_DIAGRAMS.md: a boot-mode
note (normal ↔ ESC-config, via reboot only) and, on the arming diagram, that
ESC-config mode never leaves Disarmed.

## Design

**Phase 1 (this proposal): settings** (connect, read, write, file name, run).
**Phase 2: ESC firmware flashing**, with the `esc-config-icd` flash operations
(`begin` / 256-byte `chunk` / `finish` / `read`) over the same mode and task. Phase 2
gets its own TEST_PLAN rows but no new proposal unless the design changes.

### Bootloader entry

From the AM32 sources (firmware `am32-firmware/AM32` @ `077b6a4`, bootloader
`am32-firmware/AM32-bootloader` @ `578ff29`). Not yet seen on the bench (Open
questions 1):

- **The app resets itself on signal loss.** `Src/main.c`: `signaltimeout` is cleared
  only by a valid DShot frame. Disarmed, after `LOOP_FREQUENCY_HZ << 1` loop counts
  (2 s at 20 kHz) the app calls `NVIC_SystemReset()` (0.5 s when armed).
- **The bootloader keeps a driven-high line.** `bootloader/main.c` `checkForSignal()`
  samples the pin three ways. With a pull-down, a mostly-low line jumps to the app,
  but not after a software reset (`CHECK_SOFTWARE_RESET`). With a pull-up, a line
  that never reads low makes it **stay** in the bootloader. With no pull, a line that
  reads low once jumps.
- So once the FC stops DShot and holds the line high, each ESC drops into its
  bootloader within ~2 s, with no power cycle. If a reset catches the line still low
  (FC still booting), the ESC jumps back to its app, times out 2 s later and tries
  again, so entry corrects itself once the UART holds the line.
- The FC is powered from the ESCs' 5 V, so a power cycle would reset the FC anyway.
  The design never needs one.

### Why a reboot, not a runtime handover

- **PIO budget.** The bidirectional DShot program is ~30 instructions, the one-wire
  UART 13; a PIO block holds 32, so they cannot share PIO1 or PIO2. PIO0 (LED on SM2)
  could host one UART at a time, but only after the engine pin has been taken off its
  DShot PIO, which `BidirDshotPio` does not give back (it would need `steal()`).
- **A reboot costs the operator nothing extra.** The ESCs need ~2 s without DShot to
  reset into their bootloader in either design, and the TUI reattaches on its own.
- **Clean ownership.** In each boot exactly one driver owns each engine pin, from
  `main()` on. A normal boot cannot reach the ESC-config code path, and an ESC-config
  boot never constructs DShot. Both are visible in `main()`.

### Mode selection

- **Decision, pure:** `elle_control::boot_mode::decide(scratch0, scratch1,
  reset_reason) -> BootMode` returns `EscConfig` only if the pair is
  `ESC_CONFIG_MAGIC` and its complement **and** the reset was forced (`Forced`, not a
  watchdog timeout or power-on). Host-tested.
- **Registers:** `elle_hardware::boot_mode`. `request_esc_config(&mut Watchdog)` writes
  the pair to watchdog scratch 0 and 1 and calls `trigger_reset()`. `take(&mut
  Watchdog) -> BootMode` reads (`Watchdog::scratch`, `reset_reason`), **clears** both
  registers and calls `decide`. Any later reset, power cycle, probe reset or watchdog
  timeout is therefore a normal boot. Scratch 4–7 are left alone: the RP2350 bootrom
  uses some of them for reboot parameters (to confirm against the datasheet).
- The flag lives in the watchdog block, which a power cycle clears. Nothing in a
  session needs one (see [Bootloader entry](#bootloader-entry)). Unplugging the
  battery ends the session, and the next boot is normal.
- Only `rpc-control` builds compile the `EscConfig` branch. A flight build never reads
  the flag.
- **Who resets.** The `EnterEscConfig` handler only sends `RpcCommand::EnterEscConfig`
  to the RPC loop, as every handler does. The loop owns the watchdog. It checks
  disarmed and both eRPM targets 0, emits 170, flushes the event, then requests the
  reboot. Not armed means no flash write is in progress (flash writes only happen
  disarmed); the loop also waits for any pending deferred save to finish first.

### Firmware

- **Binaries** (engine setup is binary code): `main()` calls `boot_mode::take()`
  before the engine setup. In `EscConfig` (rpc-control only) PIO1 gets
  `esc_onewire_rp::OneWireUart` on `PIN_14` and, on the eagle, PIO2 one on `PIN_11`,
  instead of `BidirDshotPio`. The DShot task is not spawned. The new
  `esc_config_task` is spawned on `EXECUTOR_DSHOT` instead, which is otherwise idle in
  this mode.
- **Why the interrupt executor.** The one-wire UART pushes bytes into a 4-word PIO
  FIFO (~2 ms at 19200). A gap over ~230 µs inside a frame splits it, and a split
  payload can leave the bootloader in payload mode until a power cycle. Any thread-mode
  task that holds Core 0 for a few ms (sdspi poll gaps, CRSF/GNSS parsing, the RPC
  server itself) would starve it; the interrupt executor preempts them all.
- **Flash stays out of the way.** A flash operation pauses even the interrupt
  executor (interrupts off). In ESC-config mode the loop refuses every
  `writes_flash()` RPC (event 63, the armed refusal), and the boot skips the
  `ulog-flash-count` legacy count (~26 ms, ~5 s after boot). No other flash operation
  runs in this mode.
- **`elle-hardware/src/esc_config.rs`**: `esc_config_task(uarts)` owns the UARTs and,
  per engine, the `(ConnectResponse, Layout)` of a connected ESC, exactly like the
  relay's `Context` (`esc-relay-fw/src/app.rs`): each request resumes an
  `esc_session::Esc` over the borrowed UART (`Esc::resume`) and runs **one whole
  operation**. Requests arrive over a `Channel<EscRequest, 1>`; replies go out on a
  `Signal`. No bootloader sequence crosses the RPC boundary.
- **RPC handlers** are `async` kind in `define_dispatch!` (the first in Elle; the
  relay does the same with no spawner): send the request, await the reply with a
  timeout (`ESC_OP_TIMEOUT_MS`, 1.5 s). The server is busy for one operation:
  typically ~0.35 s (a 192-byte write with its read-back), at worst 1.5 s, inside the
  host's 2 s `CMD_TIMEOUT`. Other requests queue behind it. `RttRx` stamps
  `HOST_LAST_RX_MS` only when the server takes the next frame, so the host link ages
  by up to 1.5 s during an operation. In this boot that is harmless (arming is
  impossible), as long as the RPC failsafe does nothing while disarmed beyond raising
  its flag. Implementation checks that, and the host suppresses its "link lost"
  warning while an ESC operation is pending.
- **Arming lock.** In ESC-config mode the `FlightController` gets
  `set_engines_unavailable(true)`; every arm path (RPC `Arm`, the `rpc-rc` gesture)
  then refuses with event 172. The loop still publishes eRPM targets; nothing reads
  them.
- **Exit.** `ExitEscConfig`, then the loop sends Run to each connected ESC through the
  task and resets normally. On any other reset the RP2350 pads fall back to their
  pull-down default, and the boot holds them there well past 20 ms before PIO setup.
  By REVAMP P1, a line held low for 20 ms makes a waiting bootloader start its app, so
  a plain FC reset should release the ESCs too. To confirm on the bench (Open
  questions 2).
- **Dependencies** (by git rev from esc-config-rs, like `icm426xx`; switch to a tag
  once one exists): `esc-session`, `esc-onewire-rp` (`rp235xa`, `defmt`),
  `esc-config-icd` (`session`), `esc-protocol` (no default features). Versions
  already match Elle: embassy `340bf1de`, heapless 0.9, postcard-rpc 0.12,
  postcard-schema 0.2.
- **Constants** in `elle-config`: `ESC_CONFIG_MAGIC`, `ESC_OP_TIMEOUT_MS` (1 500).
  Protocol timing stays in `esc_session::Timing::default()` (direct-wire defaults);
  Elle sets none of it.

### Host (`tools/elle-rpc-host`)

The firmware never interprets AM32 fields; it moves bytes. Settings parsing and
editing (`esc_protocol::am32_config::Am32Config`, with `std,serde`), melody/RTTTL, and
the host policy all stay on the host:

- **Host policy is reused, not re-implemented.** Edits touch settings bytes 5..48
  only; restore merges 5..192 and refuses another EEPROM layout version; byte 0 stays
  `0x01`. Today this lives in the bin-only `esc-configurator-cli` (`device.rs`:
  `EscDevice` trait + `Session`). **Prerequisite:** move it into a library crate in
  esc-config-rs (e.g. `esc-host`) with an async `EscDevice`. Elle implements
  `EscDevice` for its `HostClient` with an `engine` argument, and the relay CLI keeps
  its own.
- **Reattach after the mode reboot.** The TUI and `direct` gain a reattach path:
  after `EnterEscConfig` / `ExitEscConfig` they drop the link and reconnect with
  `Link::connect` (which already retries until RTT appears). `elle mcp` reuses
  `connect_retrying`. The new `elle/esc/mode` endpoint reports the boot mode, so the
  host shows `ESC CONFIG` and knows which commands apply. Existing responses
  (`GetStatus`, `GetBuildInfo`) are not changed. Older firmware answers the endpoint
  as unknown, which the host reads as "no ESC config".
- **TUI: the main panel becomes the ESC panel.** After reattaching, the TUI reads
  `elle/esc/mode`. In `EscConfig` it replaces the flight dashboard with an ESC view:
  per engine, the connect state, `EscInfo` and file name, and the decoded settings
  table, editable in place. The log panel and command line stay. Commands: `esc
  enter`, `esc connect <left|right>`, `esc set <field> <value>`, `esc backup [file]`,
  `esc restore <file>`, `esc exit`. The flight panels' polls (RC 20 Hz, attitude
  10 Hz, engine 5 Hz, …) are not sent in this mode, so nothing queues behind an ESC
  operation. The view refreshes after each operation instead.
- **`direct`:** `elle direct esc enter|connect|show|set|backup|restore|melody|run|exit
  --engine left|right`.
- **`backup`** writes a TOML snapshot (the CLI's format, so relay and Elle backups
  compare). The `governor-calibration` skill gains a step: save the ESC snapshot next
  to the new table, and compare it before trusting an old table.

### `elle mcp` tools

Reading is always allowed. **Writing ESC EEPROM is not**, mirroring the motors flag: it
needs `--dangerously-allow-esc-write` in `.mcp.json` (a human decision), and a backup
of that ESC taken in the same session.

| Tool | Gate | Does |
|------|------|------|
| `esc_mode` `{ enter \| exit }` | disarmed (firmware enforces) | Reboot into or out of ESC-config mode, reattach, report the mode. `enter` returns once the ESCs have had their ~2 s to reset into the bootloader. |
| `esc_connect` `{ engine }` | ESC-config mode | Bootloader handshake; returns `EscInfo` + file name |
| `esc_read` `{ engine }` | ESC-config mode | Decoded settings as JSON (`Am32Config`), plus the raw 192 bytes as hex |
| `esc_backup` `{ engine, path? }` | ESC-config mode | TOML snapshot (default under `logs/esc/`); returns the path. Unlocks writes for that ESC. |
| `esc_compare` `{ engine, path }` | ESC-config mode | Field-by-field diff of the ESC against a snapshot (for the governor-table check) |
| `esc_set` `{ engine, fields: {name: value} }` | `--dangerously-allow-esc-write` + same-session backup | Read-modify-write of settings bytes 5..48, read back, return the diff |
| `esc_restore` `{ engine, path }` | `--dangerously-allow-esc-write` | Restore a snapshot (host policy), read back |
| `esc_run` `{ engine }` | ESC-config mode | Leave the bootloader, start the app |

Phase 2 adds `esc_flash` behind its own `--dangerously-allow-esc-flash`.

### Timing

Nothing runs per control tick. In normal boots nothing changes. In ESC-config mode the
control loop, IMU and RPC server run as usual; `esc_config_task` sleeps on its channel
between operations, and on the interrupt executor while one runs. A 192-byte write
with its read-back is ~0.35 s on the wire.

## Interfaces

- **Events** (new block 170–179, "ESC configuration"; host labels and OPERATIONS.md rows):
  - 170 `EVT_ESC_CONFIG_ENTER`: rebooting into ESC-config mode
  - 171 `EVT_ESC_CONFIG_REFUSED`: `EnterEscConfig` refused (armed or engine target ≠ 0)
  - 172 `EVT_ARM_REFUSED_ESC_CONFIG`: arming refused, this is an ESC-config boot
  - 173 / 174 `EVT_ESC_LEFT_CONNECTED` / `EVT_ESC_RIGHT_CONNECTED`: bootloader handshake answered
  - 175 / 176 `EVT_ESC_LEFT_WRITE_FAILED` / `EVT_ESC_RIGHT_WRITE_FAILED`: NACK, CRC, no reply or verify mismatch on a write
  - 177 `EVT_ESC_WRITE_OK`: a write was committed and read back identical
- **RPC** (all added, `rpc-control` only; no existing type changes). Elle's endpoints
  carry an `engine` and reuse the `esc-config-icd` request and response types and
  `EscError`:
  - `elle/esc/mode` `() → BootMode` (`Normal` / `EscConfig`)
  - `elle/esc/mode/enter` `() → Ack`, `elle/esc/mode/exit` `() → Ack`
  - `elle/esc/connect` `Engine → ConnectResp`
  - `elle/esc/eeprom/read` `Engine → EepromResp` (192 bytes)
  - `elle/esc/eeprom/write` `{ engine, Eeprom } → AckResp` (written, then read back)
  - `elle/esc/name` `Engine → BlockResp`
  - `elle/esc/read` `{ engine, ReadReq } → BlockResp`
  - `elle/esc/run` `Engine → AckResp`
  - In a normal boot the ESC endpoints answer `EscError::NotConnected`. `Engine::Right`
    on the dart answers `EscError::Link`.
  - The largest message (a 256-byte block) fits the 1 024-byte RPC buffers.
- **Flash:** none. The FC's own flash is never written by this feature.
- **ULog:** none. Events still reach the SD card.
- **LED / CRSF:** a distinct LED pattern in ESC-config mode; CRSF flight-mode text
  `ESCCFG`.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| Aircraft flown in ESC-config mode | No engine output at all | The mode is unreachable in flight builds (cfg'd out). An `rpc-rc` build can fly, but entry needs a host on the probe sending `EnterEscConfig`, and a watchdog timeout is not a forced reset, so it never enters the mode. Once in it: arming refused (172), LED pattern, `ESCCFG` text. Any power cycle returns to a normal boot. |
| Mode entered with engines running | Engines cut by the reboot | `EnterEscConfig` refused unless disarmed with both eRPM targets 0 (171); the check runs in the loop, which owns the state |
| Stale flag reboots a later session into the mode | Unexpected silent engines on the bench | Flag cleared on read, valid only with the magic pair **and** a forced watchdog reset |
| Engine spins during configuration | Injury | No DShot program is loaded in this boot; an AM32 bootloader does not drive the motor. Props off for every session regardless. |
| Bad settings written (direction, timing, KV) | Engines misbehave, or the governor table no longer fits | Writes only through the host policy (bytes 5..48); read-back after every write; MCP writes need the flag and a same-session backup; boot-time direction assert still overrides direction; the governor snapshot compare |
| ESC left in its bootloader at the next normal boot | Silent ESC | `ExitEscConfig` sends Run first. A plain reset should release it too (pads pulled low > 20 ms, REVAMP P1; bench step 8). Otherwise events 160/161 report the silent ESC. |
| Frame split by a scheduling stall | Bootloader stuck in payload mode | Session on the interrupt executor; flash operations refused in this mode; one `Link::send` per frame (esc-session F10); `esc-session` resends the payload (F8); last resort a battery cycle, which also reboots the FC into a normal boot |
| Battery unplugged during a session | FC and ESCs both reset; normal boot | Harmless for the FC. An ESC cut mid-write is the row below. |
| Power or link lost mid-write | ESC EEPROM corrupt | With byte 0 not `0x01` the bootloader never starts the app, so the ESC stays in its bootloader. Re-enter ESC-config mode and `restore` from the backup. The bootloader is never written in Phase 1. |
| Phase 2 flash interrupted | ESC app erased | The AM32 bootloader survives (the boot byte stays cleared until `finish`); reflash. |

Sensor loss, RC loss, Core 1 stall, reboot in the air: no change in normal boots;
ESC-config mode does not fly. FC flash writes: the mode reboot waits for any pending
deferred save to finish.

## Verification

- **Host tests:** esc-config-rs's (fake AM32 bootloader, ICD round-trips, host
  policy). New in Elle: `elle_control::boot_mode::decide` (magic pair, complement,
  only `Forced` resets); `elle-rpc-host` tests of the
  Elle `EscDevice` against the fake-device `HostClient` path already used by the MCP
  tests; MCP gating (`esc_set` refused without the flag, and without a backup).
- **Replay / simulation:** none.
- **Bench** (new TEST_PLAN section 6.11, "ESC configuration from the TUI", props off):
  1. Armed (or throttle up) → `esc enter` refused, event 171. Disarmed → 170, reboot,
     TUI reattaches and its main panel becomes the ESC panel; LED pattern; `Arm` → 172.
  2. **Entry without a power cycle:** wait ≥ 2 s, then `esc connect` each engine →
     173/174 and `EscInfo` matching the relay's reading of the same ESC. With a
     logic analyser once: time from the last DShot frame to the ESC answering.
  3. `esc backup` → byte-identical to a relay backup of the same ESC.
  4. Change one harmless field (beep volume), read back. `esc exit`, `esc enter`,
     reconnect → change kept; melody and CAN bytes unchanged.
  5. `esc restore` step 3 → identical to the original.
  6. Same steps 2–5 through `elle mcp`, including `esc_set` refused without the flag.
  7. Pull the battery mid-write once (on an ESC with a backup) → after repower, the
     normal boot reports the ESC silent (160/161); `esc enter`, `restore` recovers it.
  8. `esc exit` → normal boot, events 160–165 absent, governor as before (TEST_PLAN
     6.8). Once more with a plain FC reset instead of `esc exit`, the ESC connected:
     does it start its app on its own (expected, from the pull-down)?
  9. Probe reset in ESC-config mode → normal boot (flag cleared).
  10. In ESC-config mode: `mag cal` / PID save → refused, event 63.
- **Flight:** none for Phase 1; the next flight after a settings change runs 6.8 first.

## Prerequisites (esc-config-rs)

- The bench regression and entry checklist in `esc-config-rs/docs/REVAMP.md`
  (Phase 1 bench, Phase 3 regression) pass on a real ESC. Nothing has run on
  hardware yet. **This gates Accepted.** Its entry check gains the signal-timeout
  case: ESC app running, line driven high with no DShot → bootloader within ~2 s.
- The host policy moves from the `esc-configurator-cli` binary into a library crate
  with an async `EscDevice`.
- `revamp` merged and tagged (REVAMP Phase 4); Elle pins the tag.

## Open questions

1. **Bootloader entry by signal timeout** ([Bootloader entry](#bootloader-entry)) is
   read from the AM32 source, not seen on hardware; bench step 2 settles it. Older
   AM32 versions without the disarmed 2 s reset are **not supported**: on those,
   `esc connect` fails with `NoReply`, and the host says the ESC firmware is too old.
2. **Exit by FC reset**, also source-derived and bench-tested (step 8): after a plain
   FC reset the pads pull low, and a bootloader waiting for a byte jumps to its app
   after 20 ms of low line. `esc exit` does not depend on it (it sends Run first).

## Rollback

Don't send `EnterEscConfig`. Normal boots behave as before. To remove the code, revert
the commits; flight builds contain none of it. No FC flash entries are created. An ESC
changed during a session is restored from its backup.

## Doc updates

- [ ] `docs/OPERATIONS.md`: "ESC configuration (bench)" section, events 170–177
- [ ] `STATE_DIAGRAMS.md`: boot-mode note; arming note for ESC-config mode
- [ ] `TEST_PLAN.md`: section 6.11
- [ ] `CLAUDE.md`: boot modes, `esc_config` task on `EXECUTOR_DSHOT`, ESC RPC endpoints, dependency table
- [ ] `crates/elle-ulog/README.md`: none
- [ ] `CHANGELOG.md`: `Unreleased`, operator behaviour, RPC ICD, event codes
- [ ] `tools/elle-rpc-host/README.md`: TUI `esc` commands, `direct esc …`, MCP `esc_*` tools and `--dangerously-allow-esc-write`
- [ ] `.claude/skills/governor-calibration/SKILL.md`: ESC snapshot step
- [ ] `.claude/skills/test-plan-runner/SKILL.md`: the `esc_*` tools for 6.11
