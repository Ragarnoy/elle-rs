# 0002: AM32 ESC configuration over RPC

| | |
|-|-|
| Status | Draft |
| Airframes | both (eagle: two ESCs; dart: one) |
| Date | 2026-10-02 |
| Version impact | patch ([rules](../VERSIONING.md#bump-rules)): a new bench build feature and additive RPC endpoints; flight builds unchanged |
| Surfaces | operator behaviour (new `esc-config` bench build), RPC ICD (endpoints added), event codes (170–175 added) |

## Motivation

Reading or changing an AM32 ESC's settings today means unplugging its signal lead from
the FC and connecting the separate `esc-config-rs` relay (an RP2350 that bridges USB to
the ESC's bootloader). On the eagle that is two ESCs inside a closed airframe. As a
result, ESC settings go unchecked, although several parts of the firmware depend on
them:

- `GOVERNOR_FF_TABLE` was swept against one ESC configuration (timing, PWM frequency,
  KV, direction). A changed ESC setting invalidates the table without any visible
  sign. The dart's re-sweep after the prop swap is the precedent.
- Spin direction is asserted by DShot command every boot (`ENGINE_SPIN_REVERSED`, no
  `SettingsSave`), because the EEPROM setting is not known. That stays, but it could
  confirm a stored setting rather than be the only place the direction is set.
- The ESC link work (`EscLink`, `EdtWatch`, events 160–165) would be easier to diagnose
  if we could read the ESC's EEPROM and firmware version.

The relay also has a structural flaw. The PC drives the bootloader protocol over USB,
and AM32's write sequence (Set Buffer gets no ACK; the data must follow at once) does
not tolerate USB latency. The relay therefore inspects the byte stream to recognise a
write (`tx_buf[0] == 0xFF && tx_buf[6] == 0xFE …`) and waits 2 ms to reassemble USB
fragments. Running the protocol on the FC, one typed RPC per operation, removes both.

## Behaviour delta

| Situation | Before | After |
|-----------|--------|-------|
| Flight build, `rpc-control` builds | — | Unchanged. The feature is off by default and never in a flight build. |
| Read or change ESC settings | Unplug the ESC lead, connect the relay, use `esc-configurator` | Flash the `esc-config` build, `elle direct esc show/set/backup/restore --engine left\|right` |
| `esc-config` build, boot | — | No DShot is ever sent. Engine pins hold the line high (one-wire UART idle). `Arm` is refused (event 170). ESCs drop into the AM32 bootloader (see Open questions). |
| `esc-config` build, RC or RPC throttle | — | Ignored; no engine output path exists |
| `esc-config` build, elevons | — | As in `rpc-control`: `SetElevons` and Stabilized still move surfaces disarmed. Not changed by this proposal. |

OPERATIONS.md: new section "ESC configuration (bench)". STATE_DIAGRAMS.md: no new state;
the arming diagram notes that `esc-config` builds never leave Disarmed.

## Design

**Phase 1 (this proposal): settings.** **Phase 2 (follow-up): ESC firmware flashing**,
using the same endpoints with more 256-byte writes. Phase 2 gets its own TEST_PLAN rows
but no new proposal unless the design changes.

### Build shape

A new feature `esc-config` on both binaries, forwarding to `elle-app`. It **requires
`rpc-control`** (`compile_error!` in `elle-app`, like `rpc-rc`) and is **mutually
exclusive with `rpc-rc`** (RC arming makes no sense with no engines).

A runtime switch (RPC command → watchdog reboot into a config mode, or handing the pin
from PIO1/PIO2 to PIO0 with `steal()`) is deliberately left out:

- The bidirectional DShot program is ~30 instructions; the one-wire UART needs 13. A PIO
  block holds 32, so they cannot share PIO1 or PIO2.
- A build-time switch means a flight build cannot reach this code at all, and an
  `esc-config` build cannot produce DShot at all. Both properties are easier to check
  than a runtime handover.

Revisit once Phase 1 has been used on the bench.

### Firmware

- **Binaries** (engine setup, which is where the binaries' code belongs): under
  `esc-config`, PIO1 (and PIO2 on the eagle) get a `OneWireUart` on `PIN_14` (and
  `PIN_11`) instead of `BidirDshotPio`. `dshot_task` / `dshot_single_task` is not
  spawned; `esc_config_task` is spawned on the thread executor instead.
- **`elle-hardware/src/esc_config/`**
  - `onewire.rs`: the PIO half-duplex UART from `esc-relay-fw/src/onewire_uart.rs`,
    ported with one fix: the clock divider comes from `clk_sys_freq()`, not a
    hardcoded 150 MHz (Elle runs at 200 MHz; copied as-is it would run at ~25.6 kbaud).
  - `task.rs`: `esc_config_task` owns the UARTs, receives `EscRequest`s from the RPC
    handlers over a channel and answers on a `Signal`. **It runs each operation
    end to end**: no partial sequences cross the RPC boundary.
- **Protocol logic** comes from `esc-protocol` (`no_std` parts only: `crc`, `packet`,
  `bootloader::Session`, `device`), pinned by git rev like `icm426xx`. Before that,
  esc-config-rs's uncommitted work must be committed and its `no_std` build checked for
  `thumbv8m.main-none-eabihf`. If pinning proves awkward, vendor those modules under
  `drivers/esc-protocol`.
- **Constants** in `elle-config`: `ESC_BOOT_BAUD` (19 200), `ESC_CONNECT_TIMEOUT_MS`
  (2 000), `ESC_ACK_TIMEOUT_MS` (100), `ESC_WRITE_TIMEOUT_MS` (200),
  `ESC_KEEPALIVE_MS` (taken from the bootloader timeout, see Open questions).
- **Keepalive.** While an ESC is connected and idle, the task sends a keepalive
  (`0xFD`) every `ESC_KEEPALIVE_MS` so the host can take its time between operations.

### Host

`elle-rpc-host` depends on `esc-protocol` with `std,serde`: `Am32Config` parsing and
editing, melody/RTTTL, Intel HEX (Phase 2). Firmware never interprets AM32 fields; it
moves bytes.

- `elle direct esc connect|show|set|backup|restore|melody --engine left|right`
- **Writes are read-modify-write of the whole 176-byte block** (config 48 B + melody
  128 B at the EEPROM base), as esc-config-rs already does: an AM32 flash write erases
  the page, and a 256-byte write is NACKed.
- `backup` writes a TOML snapshot. The `governor-calibration` skill gains a step:
  save the ESC snapshot next to the new table, and compare it before trusting an old
  table.
- `elle mcp` tools: `esc_read_config`, `esc_write_config` (the latter refused unless the
  server runs with `--dangerously-allow-esc-write`, mirroring the motors flag).

### Timing

Nothing runs per control tick. The control loop, IMU and RPC server are unchanged.
`esc_config_task` sleeps on its channel between operations. A 176-byte write is ~10 ms
on the wire plus the ESC's flash commit (≤ 200 ms); the task `await`s throughout, so
the thread executor and the watchdog feed are not blocked.

## Interfaces

- **Events** (new block 170–179, "ESC configuration"; host labels and OPERATIONS.md rows):
  - 170 `EVT_ARM_REFUSED_ESC_CONFIG`: arming refused, this is an `esc-config` build
  - 171 / 172 `EVT_ESC_LEFT_BOOTLOADER` / `EVT_ESC_RIGHT_BOOTLOADER`: connect handshake answered
  - 173 / 174 `EVT_ESC_LEFT_WRITE_FAILED` / `EVT_ESC_RIGHT_WRITE_FAILED`: NACK, CRC or timeout on a write
  - 175 `EVT_ESC_WRITE_OK`: a write was committed and read back identical
- **RPC** (all added, `elle/esc/…`, compiled under `esc-config`; `GetBuildInfo` lists the feature):
  - `EscConnect { engine } → EscDeviceInfo { signature, mcu, eeprom_addr, boot_version }`
  - `EscRead { engine, addr, len ≤ 256 } → EscData`
  - `EscWrite { engine, addr, data ≤ 256 } → EscAck` (Set Address, Set Buffer, data, CRC,
    commit, then read back and compare)
  - `EscDisconnect { engine } → EscAck`
  - Errors are a typed `EscError` (NotConnected, Timeout, Nack(u8), Crc, VerifyMismatch,
    BadLength). 256 B of data fits the 1 024-byte RPC buffers.
- **Flash:** none. The FC's own flash is never written by this feature.
- **ULog:** none. (The bench session's events still reach the SD card.)
- **CRSF telemetry / LED:** an `esc-config` build shows a distinct LED pattern so it
  cannot be mistaken for a flight build on the bench.

## Flight-safety risks

| Risk | Effect | Mitigation |
|------|--------|------------|
| `esc-config` build flown by mistake | No engine output at all | `Arm` refused (event 170); distinct LED pattern; CRSF flight-mode text `ESCCFG`; feature excluded from every CI flight build |
| Feature leaks into a flight build | Engines driven by a UART instead of DShot | `compile_error!` unless `rpc-control`; CI flight builds use default features; `cargo hack` powerset includes it |
| Engine spins during configuration | Injury | No DShot program is loaded in this build; an AM32 bootloader does not drive the motor. Props off for every session regardless. |
| Bad settings written (wrong direction, timing, KV) | Engines misbehave or the governor table no longer fits | Read-back compare after every write; `backup` before `set` (host enforces it); boot-time direction assert still overrides direction; governor snapshot compare |
| ESC left in the bootloader | Silent ESC at the next flight boot | `EscDisconnect` resets the ESC; in the flight build the existing events 160/161 report an ESC that never answers |
| Power or link lost mid-write | ESC EEPROM corrupt | `restore` from the backup through the bootloader, which a bad EEPROM does not affect (how AM32 itself treats a corrupt block is to be checked in bench step 6). The bootloader itself is never written in Phase 1. |
| Phase 2 flash interrupted | ESC application erased | The AM32 bootloader survives; reflash from the same build. Phase 2 refuses images that overlap the bootloader region. |

Sensor loss, RC loss, Core 1 stall, FC flash write, reboot in the air: not applicable;
this build does not fly and has no engine output.

## Verification

- **Host tests:** `esc-protocol`'s existing tests (CRC vectors, packets, session);
  new tests in `elle-rpc-host` for read-modify-write of the 176-byte block
  (config-only and melody-only edits leave the other half intact), and `EscError`
  mapping.
- **Replay / simulation:** none.
- **Bench** (new TEST_PLAN section 6.11, "ESC configuration over RPC", props off,
  battery connected):
  1. Flash the `esc-config` build; check the LED pattern, `GetBuildInfo` lists
     `esc-config`, `Arm` → event 170 and stays disarmed.
  2. Power the ESCs; `esc connect` on each engine → events 171/172 and device info
     matching the relay's reading of the same ESC.
  3. `esc backup`, then compare byte for byte with a backup taken through the relay.
  4. Change one harmless field (e.g. beep volume), write, read back, power-cycle, read
     again → change persists, rest of the block identical.
  5. `esc restore` from step 3 → identical to the original.
  6. Pull the battery mid-write once → the ESC still enters the bootloader, `restore`
     recovers it.
  7. Flash the flight build → events 160–165 absent; governor behaves as before
     (TEST_PLAN 6.8).
- **Flight:** none required for Phase 1; the next flight after a settings change runs
  6.8 first.

## Open questions

1. **Bootloader entry.** The plan assumes that holding the signal line high with no
   DShot makes the AM32 application reset on signal timeout, and that the bootloader
   then sees the driven-high line and waits for commands. esc-config-rs's docs say the
   opposite ("power-on with signal held low"). **Settle this on the bench with the
   relay before any implementation**: it decides whether the operator must power-cycle
   the ESCs and whether `EscDisconnect` can rely on a reset.
2. Bootloader idle timeout (sets `ESC_KEEPALIVE_MS`).
3. Pin `esc-protocol` by git rev or vendor it under `drivers/`. **Prerequisite:** the
   esc-config-rs revamp (`esc-config-rs/docs/REVAMP.md`) extracts the bootloader
   sequencing into a `no_std` async `esc-session` crate and the PIO UART into
   `esc-onewire-rp`; Elle should depend on those rather than port the relay code.
   The firmware design above shrinks accordingly once they exist.

## Rollback

Don't build with `esc-config`. Flight builds contain none of this code. No FC flash
entries are created; an ESC changed during a session is restored from its `backup`.

## Doc updates

- [ ] `docs/OPERATIONS.md`: "ESC configuration (bench)" section, events 170–175
- [ ] `STATE_DIAGRAMS.md`: arming note for `esc-config` builds
- [ ] `TEST_PLAN.md`: section 6.11
- [ ] `CLAUDE.md`: feature flag, `esc_config` module, ESC config RPC endpoints, dependency table
- [ ] `crates/elle-ulog/README.md`: none
- [ ] `CHANGELOG.md`: `Unreleased`, RPC ICD and event codes
- [ ] `tools/elle-rpc-host/README.md`: `direct esc …` commands, MCP tools
- [ ] `.claude/skills/governor-calibration/SKILL.md`: ESC snapshot step
