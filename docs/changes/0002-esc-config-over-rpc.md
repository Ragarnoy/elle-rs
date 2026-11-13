# 0002: AM32 ESC configuration from the TUI and MCP

| | |
|-|-|
| Status | Accepted |
| Airframes | both (eagle: two ESCs; dart: one) |
| Date | 2026-10-02, rewritten 2026-11-11 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules), 0.x): additive only (new endpoints, new event codes, a new RPC-build mode); no existing type or code changes; flight builds unchanged |
| Surfaces | operator behaviour (ESC-config mode in RPC builds), RPC ICD (endpoints added), event codes (170–178 added) |

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
| `rpc-control` build, normal boot | — | Unchanged, except that the `elle/esc/*` endpoints exist (ESC operations answer `NotConnected`). |
| Read or change ESC settings | Unplug the ESC lead, connect the relay, run `esc-configurator` | TUI ESC panel (replaces the main panel in ESC-config mode), `elle direct esc …`, or `elle mcp` `esc_*` tools |
| `elle/esc/mode/enter` while disarmed, both eRPM targets 0 | — | Event 170, ULog file closed, MCU reboots ~200 ms later into ESC-config mode; the host reattaches |
| `elle/esc/mode/enter` while armed, or a target ≠ 0 | — | Refused (event 171), nothing happens |
| ESC-config mode, boot | — | No DShot program is loaded and no DShot is ever sent. The engine pins idle high (one-wire UART). The LED double-blinks orange; CRSF flight-mode text `ESCCFG`. About 2 s later each ESC, seeing no signal, resets itself into its bootloader and stays there because the line is high (see [Bootloader entry](#bootloader-entry)). No power cycle: the battery stays connected (it also powers the FC through the ESCs' 5 V). |
| ESC-config mode, flash-writing RPCs (PID save, calibration clears, ULog erase) and level cal | — | Refused (event 178), so no flash operation can pause the ESC link |
| ESC-config mode, `Arm` / RC arming gesture / throttle | — | Arming refused (event 172); throttle has no output path |
| ESC-config mode, elevons | — | As in any RPC build: `SetElevons` and Stabilized still move surfaces disarmed. Not changed. |
| ESC-config mode, `elle/esc/mode/exit` | — | Each connected ESC is sent Run (app starts), then the MCU reboots normally |
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
- **Registers:** `elle_hardware::watchdog::reboot_into(mode)` writes the pair to
  watchdog scratch 0 and 1 (`EscConfig`) or clears them (`Normal`) and calls
  `trigger_reset()`. `elle_hardware::boot_mode::init(&mut Watchdog) -> BootMode`, first
  thing in `main()`, reads them (`Watchdog::scratch`, `reset_reason`), **clears** both
  and calls `decide`; `boot_mode::current()` / `is_esc_config()` answer for the rest
  of the boot. Any later reset, power cycle, probe reset or watchdog
  timeout is therefore a normal boot. Scratch 4–7 are left alone: the RP2350 bootrom
  uses some of them for reboot parameters (to confirm against the datasheet).
- The flag lives in the watchdog block, which a power cycle clears. Nothing in a
  session needs one (see [Bootloader entry](#bootloader-entry)). Unplugging the
  battery ends the session, and the next boot is normal.
- Only `rpc-control` builds compile the `EscConfig` branch. A flight build never reads
  the flag.
- **Who resets.** The `elle/esc/mode/enter` handler refuses while armed (171,
  `ESC_MODE_REFUSED_ARMED`) and otherwise only sends `RpcCommand::SwitchBootMode` to
  the RPC loop, as every handler does (`ESC_MODE_BUSY` if the queue is full). The loop
  owns the watchdog and decides: armed or an eRPM target ≠ 0 → 171 and nothing else
  (the host then sees the old mode after reattaching and says the switch did not
  take); otherwise 170, ULog stopped and the SD file closed, and the reboot
  `ESC_REBOOT_DELAY_MS` (200 ms) later so the event and the RPC answer get out. A
  calibration save still in flight would be cut, as by any reset;
  `sequential-storage` is built to survive that, and in ESC-config mode no flash
  write can start.

### Firmware

- **Binaries** (engine setup is binary code): `main()` calls `boot_mode::init()`
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
  `writes_flash()` RPC, level cal and ULog peeks (a peek can erase while repairing
  a corrupt queue; event 178), and the boot skips the
  `ulog-flash-count` legacy count (~26 ms, ~5 s after boot). No other flash operation
  runs in this mode.
- **`elle-hardware/src/esc_config.rs`**: `esc_config_task(uarts)` owns the UARTs and,
  per engine, the `(ConnectResponse, Layout)` of a connected ESC, exactly like the
  relay's `Context` (`esc-relay-fw/src/app.rs`): each request resumes an
  `esc_session::Esc` over the borrowed UART (`Esc::resume`) and runs **one whole
  operation**. Requests arrive over a `Channel<EscRequest, 1>`; replies go out on a
  `Signal`. No bootloader sequence crosses the RPC boundary.
- **RPC handlers** (`elle-app/src/rpc_esc.rs`) are `async` kind in `define_dispatch!`
  (the first in Elle; the relay does the same with no spawner): send the request,
  await the reply with a timeout (`ESC_OP_TIMEOUT_MS`, 4 s: a write with its
  read-back at `esc_session`'s worst-case retries is ~2.1 s). The host waits
  `ESC_HOST_TIMEOUT_MS` (5 s), so the firmware's own `NoReply` arrives first. The
  server is busy for one operation, typically ~0.35 s; other requests queue behind
  it. `RttRx` stamps `HOST_LAST_RX_MS` only when the server takes the next frame, so
  while it waits the handler stamps it itself every `ESC_HOST_STAMP_MS` (100 ms):
  the host is waiting for this answer, and the RPC failsafe must not age it out.
- **Late replies.** Each request carries a sequence number; a reply to a request
  that timed out is dropped, never taken for the next one's answer.
- **Write verification.** `elle/esc/eeprom/write` writes, reads back and compares
  every byte but byte 2 (the bootloader writes its own version there;
  `elle_control::boot_mode::esc_write_mismatch`, host-tested). A mismatch answers
  `EscError::Verify { offset }` and event 175/176.
- **Arming lock.** In ESC-config mode `boot::run` calls `FlightController::lock_arming()`
  before anything can arm; every arm path (RPC `Arm`, the `rpc-rc` gesture) then
  refuses with event 172 (`take_arm_refused`). The loop still publishes eRPM targets;
  nothing reads them.
- **Exit.** `elle/esc/mode/exit`: the handler connects both ESCs (one this boot never
  connected may still wait in its bootloader), then sends Run to both back to back,
  then `SwitchBootMode(Normal)`, and the loop resets normally. Running one before a
  slow connect of the other would let it drop back into its bootloader first. Best
  effort: a failed Run is not reported; check events 160–165 after the normal boot.
  At most four ESC operations: the host waits up to ~31 s.
- **Not during boot.** ESC operations and the mode switch wait until the RPC loop
  runs (`RPC_LOOP_RUNNING`): before that, boot's flash profile loads (which can
  repair, so erase) could overlap an ESC frame, and an accepted switch would sit
  queued for seconds. `enter` answers `ESC_MODE_BUSY` until then.
- **ULog closed before the reboot.** The loop reboots once the SD writer has
  written the session's queued chunks and closed the file (`SD_FILE_OPEN`). A file
  still open after `ESC_REBOOT_SD_WAIT_MS` (2 s) cancels the switch (171) rather
  than reset in the middle of a FAT update; the host then reports that the switch
  did not take.
- **Host link in this mode.** The TUI pings every 100 ms. `direct` and `elle mcp`
  send nothing between calls, so the pure-RPC host failsafe raises its events
  (13/14/15) between them. Harmless here (disarmed, arming locked); the orange double
  blink takes precedence over the failsafe flash. On any other reset the RP2350 pads fall back to their
  pull-down default, and the boot holds them there well past 20 ms before PIO setup.
  By REVAMP P1, a line held low for 20 ms makes a waiting bootloader start its app, so
  a plain FC reset should release the ESCs too. To confirm on the bench (Open
  questions 2).
- **Dependencies** (by git rev from esc-config-rs, like `icm426xx`: branch
  `elle-integration` @ `1dcff3f`; switch to a tag once one exists): `esc-session`,
  `esc-onewire-rp` (`rp235xa`), `esc-config-icd` (`session` in firmware, `use-std` on
  the host), `esc-protocol` (no default features in firmware; `std,serde` on the host).
  All behind the `elle-hardware/esc-config` feature, which `rpc-control` enables. Versions
  already match Elle: embassy `340bf1de`, heapless 0.9, postcard-rpc 0.12,
  postcard-schema 0.2.
- **Constants** in `elle-config`: `ESC_CONFIG_MAGIC`, `ESC_OP_TIMEOUT_MS` (4 000),
  `ESC_HOST_TIMEOUT_MS` (5 000), `ESC_HOST_STAMP_MS` (100), `ESC_REBOOT_DELAY_MS` (200),
  with asserts tying the stamp under `RC_WARNING_MS` and the firmware timeout under
  the host's.
  Protocol timing stays in `esc_session::Timing::default()` (direct-wire defaults);
  Elle sets none of it.

### Host (`tools/elle-rpc-host`)

The firmware never interprets AM32 fields; it moves bytes. Settings parsing and
editing (`esc_protocol::am32_config::Am32Config`, with `std,serde`), melody/RTTTL, and
the host policy all stay on the host:

- **Host policy is reused, not re-implemented.** Edits touch settings bytes 5..48
  only; restore merges 5..192 and refuses another EEPROM layout version; byte 0 stays
  `0x01`. These are pure functions in `esc_protocol::policy` (`merge_settings`,
  `merge_restore`, `merge_melody`), which both the relay CLI and Elle's
  `elle_rpc_host::esc` call around their own transport.
- **Reattach after the mode reboot.** After `enter` / `exit` the TUI, `direct` and
  `elle mcp` drop the link, wait ~1 s for the reboot, attach again
  (`Link::connect_within`, 30 s), wait for the first ping reply and read
  `elle/esc/mode`. If the mode is not the one asked for, they report that the switch
  did not take. The new `elle/esc/mode` endpoint reports the boot mode, so the
  host shows `ESC CONFIG` and knows which commands apply. Existing responses
  (`GetStatus`, `GetBuildInfo`) are not changed. Older firmware answers the endpoint
  as unknown, which the host reads as "no ESC config".
- **TUI: the main panel becomes the ESC panel.** After reattaching, the TUI reads
  `elle/esc/mode`. In `EscConfig` it replaces the flight dashboard with an ESC view:
  per engine, `EscInfo` and file name and the decoded settings. The log panel and
  command line stay. Commands: `esc` (mode), `esc enter`, `esc exit`, `esc
  connect|show [left|right]`, `esc set [left|right] name=value …`, `esc backup
  [left|right] [file]`, `esc restore [left|right] <file>`, `esc run [left|right]`. The
  flight panels' polls (RC 20 Hz, attitude 10 Hz, engine 5 Hz, …) are not sent in
  this mode, so nothing queues behind an ESC operation; a 100 ms ping keeps the host
  link fresh. The view refreshes after each operation.
- **`direct`:** `elle direct esc mode|enter|exit|connect|show|set|backup|restore|run
  [-e left|right]`.
- **`backup`** writes the raw 256 bytes at the EEPROM base, the esc-config-rs CLI's
  backup format, so relay and Elle backups compare byte for byte (default
  `logs/esc/<engine>-<device id>-<time>.bin`), never over an existing file. The CLI's TOML export/import stays a
  separate, human-readable format; `esc_read` shows the same field names. The
  `governor-calibration` skill gains a step: save an ESC backup next to the new
  table, and `esc_compare` against it before trusting an old table.

### `elle mcp` tools

Reading is always allowed. **Writing ESC EEPROM is not**, mirroring the motors flag: it
needs `--dangerously-allow-esc-write` in `.mcp.json` (a human decision); `esc_set` also
needs a backup of that ESC (engine and device id) taken on the current link: a
device id names an ESC model, so the server forgets its backups whenever the link
ends (disconnect, reset, mode switch). `esc_set` and `esc_restore` are serialized.
`link_status` reports `esc_write_allowed`.

| Tool | Gate | Does |
|------|------|------|
| `esc_mode` `{ get \| enter \| exit }` | disarmed (firmware enforces) | Report the mode, or reboot into or out of ESC-config mode, reattach and report it. After `enter` the ESCs need ~2 s without DShot before they answer. |
| `esc_connect` `{ engine }` | ESC-config mode | Bootloader handshake; returns `EscInfo` + file name |
| `esc_read` `{ engine }` | ESC-config mode | Decoded settings as JSON (`Am32ConfigData` names), plus the raw 192 bytes as hex |
| `esc_backup` `{ engine, path? }` | ESC-config mode | Raw 256-byte backup (default under `logs/esc/`); returns the path. Unlocks `esc_set` for that ESC. |
| `esc_compare` `{ engine, path }` | ESC-config mode | Field-by-field diff of the ESC against a backup (for the governor-table check) |
| `esc_set` `{ engine, fields: {name: value} }` | `--dangerously-allow-esc-write` + a backup on the current link | Read-modify-write of settings bytes 5..48, read back, return the diff |
| `esc_restore` `{ engine, path }` | `--dangerously-allow-esc-write` | Restore a backup (host policy), read back, return the diff |
| `esc_run` `{ engine }` | ESC-config mode | Connect, then leave the bootloader and start the app. Transient: with no DShot the app resets back into the bootloader ~2 s later. `esc_mode exit` is how to leave. |

Phase 2 adds `esc_flash` behind its own `--dangerously-allow-esc-flash`.

### Timing

Nothing runs per control tick. In normal boots nothing changes. In ESC-config mode the
control loop, IMU and RPC server run as usual; `esc_config_task` sleeps on its channel
between operations, and on the interrupt executor while one runs. A 192-byte write
with its read-back is ~0.35 s on the wire.

## Interfaces

- **Events** (new block 170–179, "ESC configuration"; host labels and OPERATIONS.md rows):
  - 170 `EVT_ESC_CONFIG_ENTER`: rebooting into ESC-config mode
  - 171 `EVT_ESC_CONFIG_REFUSED`: mode switch refused (armed or engine target ≠ 0) or cancelled (ULog file still open after the wait)
  - 172 `EVT_ARM_REFUSED_ESC_CONFIG`: arming refused, this is an ESC-config boot
  - 173 / 174 `EVT_ESC_LEFT_CONNECTED` / `EVT_ESC_RIGHT_CONNECTED`: bootloader handshake answered
  - 175 / 176 `EVT_ESC_LEFT_WRITE_FAILED` / `EVT_ESC_RIGHT_WRITE_FAILED`: NACK, CRC, no reply or verify mismatch on a write
  - 177 `EVT_ESC_WRITE_OK`: a write was committed and read back identical
  - 178 `EVT_FLASH_REFUSED_ESC_CONFIG`: a flash-touching RPC or level cal refused in ESC-config mode
- **RPC** (all added, `rpc-control` only; no existing type changes). Elle's endpoints
  carry an `engine` and reuse the `esc-config-icd` request and response types and
  `EscError`:
  - `elle/esc/mode` `() → BootMode` (`Normal` / `EscConfig`)
  - `elle/esc/mode/enter` `() → Ack`, `elle/esc/mode/exit` `() → Ack`; error codes
    `ESC_MODE_REFUSED_ARMED` (1), `ESC_MODE_BUSY` (2)
  - `elle/esc/connect` `Engine → ConnectResp`
  - `elle/esc/eeprom/read` `Engine → EepromResp` (192 bytes)
  - `elle/esc/eeprom/write` `{ engine, Eeprom } → AckResp` (written, then read back)
  - `elle/esc/name` `Engine → BlockResp`
  - `elle/esc/read` `{ engine, ReadReq } → BlockResp`
  - `elle/esc/run` `Engine → AckResp`
  - In a normal boot the ESC endpoints answer `EscError::NotConnected`; in ESC-config
    mode, no answer within `ESC_OP_TIMEOUT_MS` is `EscError::NoReply`. `Engine::Right`
    on the dart answers `EscError::Link`.
  - The largest message (a 256-byte block) fits the 1 024-byte RPC buffers.
- **Flash:** none. The FC's own flash is never written by this feature.
- **ULog:** none. Events still reach the SD card.
- **LED / CRSF:** orange double blink in ESC-config mode; CRSF flight-mode text
  `ESCCFG`.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| Aircraft flown in ESC-config mode | No engine output at all | The mode is unreachable in flight builds (cfg'd out). An `rpc-rc` build can fly, but entry needs a host on the probe sending `elle/esc/mode/enter`, and a watchdog timeout is not a forced reset, so it never enters the mode. Once in it: arming refused (172), orange double blink, `ESCCFG` text. Any power cycle returns to a normal boot. |
| Mode entered with engines running | Engines cut by the reboot | The switch is refused unless disarmed with both eRPM targets 0 (171); the check runs in the loop, which owns the state |
| Stale flag reboots a later session into the mode | Unexpected silent engines on the bench | Flag cleared on read, valid only with the magic pair **and** a forced watchdog reset |
| Engine spins during configuration | Injury | No DShot program is loaded in this boot; an AM32 bootloader does not drive the motor. Props off for every session regardless. |
| Bad settings written (direction, timing, KV) | Engines misbehave, or the governor table no longer fits | Writes only through the host policy: an edit changes bytes 5..48 only; a restore writes 5..192 (melody and CAN settings included) from a backup, and only one of the same EEPROM layout version; read-back after every write; MCP writes need the flag, `esc_set` also a backup on the current link; boot-time direction assert still overrides direction; the governor-table backup compare |
| ESC left in its bootloader at the next normal boot | Silent ESC | `exit` sends Run first. A plain reset should release it too (pads pulled low > 20 ms, REVAMP P1; bench step 8). Otherwise events 160/161 report the silent ESC. |
| Frame split by a scheduling stall | Bootloader stuck in payload mode | Session on the interrupt executor; flash operations refused in this mode; one `Link::send` per frame (esc-session F10); `esc-session` resends the payload (F8); last resort a battery cycle, which also reboots the FC into a normal boot |
| Battery unplugged during a session | FC and ESCs both reset; normal boot | Harmless for the FC. An ESC cut mid-write is the row below. |
| Power or link lost mid-write | ESC EEPROM corrupt | With byte 0 not `0x01` the bootloader never starts the app, so the ESC stays in its bootloader. Re-enter ESC-config mode and `restore` from the backup. The bootloader is never written in Phase 1. |
| Phase 2 flash interrupted | ESC app erased | The AM32 bootloader survives (the boot byte stays cleared until `finish`); reflash. |

Sensor loss, RC loss, Core 1 stall, reboot in the air: no change in normal boots;
ESC-config mode does not fly. FC flash writes: none can start in ESC-config mode; a
calibration save in flight when the switch reboots is cut as by any reset, which
`sequential-storage` tolerates.

## Verification

- **Host tests:** esc-config-rs's (fake AM32 bootloader, ICD round-trips, host
  policy). New in Elle: `elle_control::boot_mode::decide` (magic pair, complement,
  only `Forced` resets), `esc_write_mismatch` (byte 2 ignored), the arming lock;
  `elle_rpc_host::esc` (edits keep bytes 0..5 and 48..192, every decoded name is
  settable, diffs show melody bytes); the MCP tools against a fake ESC on the
  fake-device `HostClient` path already used by the MCP tests (reads always allowed,
  `esc_set` / `esc_restore` refused without the flag, `esc_set` refused without a
  backup, backup → set → compare → restore round trip).
- **Replay / simulation:** none.
- **Bench** (new TEST_PLAN section 6.11, "ESC configuration from the TUI", props off):
  1. Armed (or throttle up) → `esc enter` refused, event 171. Disarmed → 170, reboot,
     TUI reattaches and its main panel becomes the ESC panel; LED pattern; `Arm` → 172.
  2. **Entry without a power cycle:** wait ≥ 2 s (a `NoReply` sooner is the ESC app
     still timing out: retry), then `esc connect` each engine →
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
  10. In ESC-config mode: `mag cal clear` / PID save / `level cal` → refused, event 178.
  11. A large write over RTT: `esc set` and `esc restore` answer (the 192-byte
      request is the largest host-to-FC frame Elle sends).
- **Flight:** none for Phase 1; the next flight after a settings change runs 6.8 first.

## Prerequisites (esc-config-rs)

- The bench regression and entry checklist in `esc-config-rs/docs/REVAMP.md`
  (Phase 1 bench, Phase 3 regression) pass on a real ESC. Nothing has run on
  hardware yet. **This gates Verified** (with TEST_PLAN 6.11). Its entry check gains
  the signal-timeout case: ESC app running, line driven high with no DShot →
  bootloader within ~2 s.
- Done: the host policy is pure functions in `esc_protocol::policy`, on
  esc-config-rs branch `elle-integration` (`1dcff3f`), which Elle pins by rev. That
  branch must not be rebased or deleted until it is merged or tagged; then Elle pins
  the tag (REVAMP Phase 4).

## Open questions

Both are answered on the eagle (F4A_4IN1_F421, AM32 2.20), 2026-11-11, TEST_PLAN 6.11.
The dart is still to run.

1. **Bootloader entry by signal timeout** ([Bootloader entry](#bootloader-entry)):
   **yes.** After `esc enter`, with no ESC power cycle, both ESCs answered the
   bootloader handshake (173/174, row 2). Not timed with a logic analyser. Older
   AM32 versions without the disarmed 2 s reset are still **not supported**: on
   those, `esc connect` keeps failing with `NoReply`.
2. **Exit by FC reset:** **yes.** With both ESCs connected in their bootloaders, a
   plain probe reset (no `esc exit`) gave a normal boot with both ESCs sending
   their app's telemetry and no events 160–165 within 15 s (row 8, which also passes
   `esc exit` with one or both ESCs connected). `esc exit` still sends Run to each
   ESC first, connecting one this boot never connected (an ESC still in its app
   just fails the connect).

The bench also found a fault that predates this proposal: the AON timer keeps
running through a watchdog reset, and `set_counter` panicked on a running timer, so
every watchdog reset (the mode switch included) ended in HardFault until a power
cycle. Fixed in `d5e9af8` (stop the timer before reseeding it); TEST_PLAN 6.11 row 13.

## Rollback

Don't send `elle/esc/mode/enter`. Normal boots behave as before. To remove the code, revert
the commits; flight builds contain none of it. No FC flash entries are created. An ESC
changed during a session is restored from its backup.

## Doc updates

- [x] `docs/OPERATIONS.md`: "ESC configuration (bench)" section, events 170–178
- [x] `STATE_DIAGRAMS.md`: boot-mode note; arming note for ESC-config mode
- [x] `TEST_PLAN.md`: section 6.11
- [x] `CLAUDE.md`: boot modes, `esc_config` task on `EXECUTOR_DSHOT`, ESC RPC endpoints, dependency table
- [x] `crates/elle-ulog/README.md`: none
- [x] `CHANGELOG.md`: `Unreleased`, operator behaviour, RPC ICD, event codes
- [x] `tools/elle-rpc-host/README.md`: TUI `esc` commands, `direct esc …`, MCP `esc_*` tools and `--dangerously-allow-esc-write`
- [x] `.claude/skills/governor-calibration/SKILL.md`: ESC backup and compare step
- [x] `.claude/skills/test-plan-runner/SKILL.md`: the `esc_*` tools for 6.11
