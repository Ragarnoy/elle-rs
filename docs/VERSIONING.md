# Versioning

One [semantic version](https://semver.org) covers the whole workspace:
`[workspace.package] version` in the root `Cargo.toml`. Both firmware binaries and the
host tools carry the same number. They have to agree anyway: postcard-rpc endpoint
keys are hashes of the request and response schemas, so a host built from a different
ICD simply gets no answer.
The vendored `mmc5616wa` and `sam-m10q` drivers inherit it too; `bmp390` keeps its
upstream version so the crates.io patch still matches.

The version reaches the outside in three places, with no code to keep in sync:

| Where | What | Source |
|-------|------|--------|
| `GetVersion` RPC | major/minor/patch | `CARGO_PKG_VERSION_*`, parsed at compile time (`elle-app/src/rpc_app.rs`) |
| `GetBuildInfo` RPC | `git describe --always --dirty` | `elle-app/build.rs`: `v0.2.0` on a tag, `v0.2.0-14-gabc1234567` after it |
| ULog header | `ver_sw` | `CARGO_PKG_VERSION` (`elle-hardware/src/ulog_logger.rs`) |

## Compatibility surfaces

A change is **breaking** when it changes one of these in a way an existing user,
transmitter setup, host build, calibration or log analysis would notice:

1. **Operator behaviour.** The arming gesture, failsafe timing and action, kill
   switch, RC channel map, mode semantics, LED meaning: anything in
   [`OPERATIONS.md`](OPERATIONS.md) a pilot relies on.
2. **RPC ICD** (`crates/elle-rpc-icd`). Changing or removing an endpoint or topic,
   or changing the layout of any type it carries. Adding an endpoint is not
   breaking.
3. **Flash profile** (`elle_config::profile::ProfileEntry`). Renumbering a key or
   changing a value's layout. A break here silently drops or misreads a stored
   calibration or gain set.
4. **ULog** ([`crates/elle-ulog/README.md`](../crates/elle-ulog/README.md)).
   Removing or renaming a message or field, or changing its meaning or units. This
   breaks `elle_log.py`, `elle-replay` and comparisons with older logs. Adding a
   message or a field is not breaking.
5. **Event codes** (`elle-hardware/src/event.rs`). Giving an existing code a
   different meaning. Retired codes stay reserved; adding a code is not breaking.

## Bump rules

**While on 0.x** (now):

| Bump | When |
|------|------|
| `0.MINOR.0` | Any breaking change |
| `0.x.PATCH` | Everything else: features, tuning, fixes |

**From 1.0.0:**

| Bump | When |
|------|------|
| MAJOR | Any breaking change |
| MINOR | New features, and **tuning**: PID gains, governor table, filter constants, setpoint limits. Tuning is minor rather than a patch because it changes how the aircraft flies. |
| PATCH | Fixes, docs, refactors with no behaviour change |

**1.0.0** comes when navigation leaves observation mode and actually commands the
aircraft, and the TEST_PLAN rows for active navigation have passed.

## Changelog

[`CHANGELOG.md`](../CHANGELOG.md) follows [Keep a Changelog](https://keepachangelog.com).
Every PR that changes a compatibility surface, or tunes the aircraft, adds a line
under `## [Unreleased]`. Breaking lines name the surface they break, e.g.
`**ULog:** …`. Other PRs may add a line; pure refactors and doc fixes don't need to.

## Releasing

1. In `CHANGELOG.md`, rename `## [Unreleased]` to `## [X.Y.Z] - YYYY-MM-DD` and add
   a fresh empty `## [Unreleased]` above it.
2. Set `version` in the root `Cargo.toml`, then `cargo update -w` so `Cargo.lock`
   follows.
3. Commit as `Release vX.Y.Z`.
4. `git tag -a vX.Y.Z -m "vX.Y.Z"` on that commit, on `master`.
5. `git push origin vX.Y.Z`.

## Flown builds

Fly a tagged build, or at least a clean one. A `-dirty` describe can't be traced back
to source afterwards. `GetBuildInfo` (TUI, `elle mcp`) and the ULog header then
identify exactly what flew.
