# elle-rpc-host

Host-side CLI for the Elle flight controller. Talks postcard-RPC over RTT through a
debug probe, so the firmware must be built with the `rpc-control` feature. List `gnss`
too — `--no-default-features` drops it, and without it there is no GNSS at all:

```sh
# firmware (ground test mode); elle-eagle works the same
cargo run --release -p elle-dart --no-default-features --features rpc-control,gnss

# host tool — the workspace defaults to thumbv8m, so name the host target
cargo build -p elle-rpc-host --release --target x86_64-unknown-linux-gnu
```

Two modes:

- **`elle`** (no subcommand) — the TUI monitoring dashboard.
- **`elle direct <cmd>`** — one RPC call and exit, for scripting.
  `ping`, `version`, `status`, `attitude`, `throttle`, `elevon`, `arm`, `disarm`,
  `stop`, `perf`, `mag`, `gnss`, `engine`, `mag-cal {start|clear|status}`,
  `level-cal {start|clear|status}`.

Arming, calibration, autotune and the event codes shown in the Logs panel are explained
in [`docs/OPERATIONS.md`](../../docs/OPERATIONS.md).

## MCP server (`elle mcp`)

An MCP server on stdio that holds the probe, so an agent (Claude Code) can
interrogate and command the flight controller and run most of `TEST_PLAN.md`:
read any query endpoint, sample a source for statistics, wait for a condition
while the operator acts (tilt, switch, TX off), wait for firmware events, set
surfaces, modes, calibrations and ULog. It needs an RPC build
(`rpc-control,gnss` or `rpc-control,rpc-rc,gnss`); the probe has one owner, so
the TUI cannot run at the same time (`disconnect` hands it over).

To use it, copy `mcp.example.json` to `.mcp.json` at the repo root (Claude
Code asks before starting it), then ask for `connect`.

**Engines are off limits by default.** `arm` and `set_throttle` refuse unless
the server is started with `--dangerously-allow-motors` (add it to `args` in
`.mcp.json` yourself, for a props-off session). Even then:

- `arm` needs `props_off_confirmed: true`;
- throttle is capped (`--max-throttle`, 30 %);
- an armed timer (`--max-armed-s`, 60 s) commands throttle 0 and disarms unless
  `extend_armed` renews it;
- the server pings every 100 ms while armed and disarms before disconnecting or
  exiting (Ctrl-C or the client closing stdin);
- `disarm` and `emergency_stop` always work.

`read` covers every query endpoint, including `build` (platform, features, turn
compensation, git describe: which build is flashed), `nav` (home, validity,
bank demand) and `core1` (IMU task load). Tools: `connect`, `disconnect`, `link_status`, `read`, `sample`, `wait_for`,
`events`, `wait_event`, `arm`, `extend_armed`, `set_throttle`, `disarm`,
`emergency_stop`, `set_elevons`, `set_mode`, `set_heading_hold`, `mag_cal`,
`level_cal`, `ulog`, `autotune`. Tests (`tests/mcp.rs`) run every tool against
a fake flight controller on the real RPC client path.

## Link keepalive

In pure RPC mode the firmware treats the host like an RC transmitter: if nothing arrives
for 300 ms it fails safe — disarm, throttle and surfaces zeroed (events 13/14). The TUI
polls continuously, so it holds the link by itself. `direct throttle` (non-zero),
`direct elevon` and `direct arm` keep running after the command, pinging every 100 ms,
until Ctrl-C; they then send throttle 0 and disarm before releasing the probe. Other
`direct` commands exit immediately. `arm` is refused while the commanded throttle is
above zero (event 18).

## Dashboard layout

```
┌ header: version · armed · mode · failsafe · autotune · REC · link · device clock ┐
├ Telemetry ─────────────┬ RC Channels ──────┬ Logs ────────────────────────────────┤
│ attitude, mag, baro,   │ per-channel bars  │ timestamped events, repeats          │
│ GNSS, loop, IMU, RC age│                   │ collapsed as ×N                      │
│ PID / elevon / engine  │                   │                                      │
├ Horizon ───────────────┤                   │                                      │
│ artificial horizon     │                   │                                      │
├ Command ───────────────┴───────────────────┴──────────────────────────────────────┤
│ > input                                                                           │
│ transient status message                                                          │
│ permanent key + command help bar                                                  │
└───────────────────────────────────────────────────────────────────────────────────┘
```

Values are health-coloured where a threshold is meaningful (satellites, HDOP, ESC
temperature, control-loop time against the loop period (5 ms at 200 Hz, read from `elle-config`), RC age against the
firmware's warning/timeout staging). Missing data is dimmed rather than shown at full
brightness.

Keys: `Enter` run · `Tab` complete · `↑↓` history · `Esc` clear · `^C`/`^D` quit.
Commands come from the `COMMANDS` table in `src/tui/commands.rs` — adding an entry
there updates tab completion, `help`, and the help bar together.

Poll rates: attitude and controller 10 Hz, RC 20 Hz, mag and engine 5 Hz, baro and
GNSS 1 Hz, status 0.5 Hz; UI redraw 10 Hz.

## Artificial horizon

`src/tui/horizon.rs` is a plain `Widget` rather than a `Canvas`: canvas cells can only
be set, not filled, and a horizon without a sky/ground fill reads as a stray line on
black. It writes cell backgrounds directly and uses `▀` half-blocks to get half-row
vertical resolution on the fill edge.

**Unverified:** `ROLL_SIGN` assumes the aviation convention — positive roll is
right-wing-down, so the ground swings to the right. The previous canvas version drew
the opposite. Bank the airframe right and check; if the display is mirrored, flip that
one constant. `positive_roll_puts_ground_to_the_right` documents the assumption.

`cargo test -p elle-rpc-host --target x86_64-unknown-linux-gnu --bin elle` covers the
geometry. For a visual check:

```sh
cargo test -p elle-rpc-host --target x86_64-unknown-linux-gnu --bin elle dump \
  -- --nocapture --ignored
```

## Not done yet

Ordered by effort. Everything below is deliberately outstanding, not forgotten.

### Small — an afternoon each

- **Roll scale**: tick arc at 0/±10/±20/±30/±45/±60 above the horizon with a moving
  pointer, so bank angle is readable without reading the digits.
- **Split the Telemetry panel** into titled sub-blocks (Attitude / Sensors / GNSS /
  Control / Engine). It is currently 14 undifferentiated lines.
- **RC channel semantics**: CH1/2/4 are bipolar and want centre-zero bars; CH5-8 are
  switches (heading hold, flight mode, autotune, kill) and want LOW/MID/HIGH pills,
  not 0→2047 gauges.
- **Engine block** with a target-vs-actual RPM bar — the number that matters most now
  that the governor is closed-loop.

### Medium — a day or two

- **PFD chrome** inside the horizon: heading tape across the top, altitude and vario
  tape right, throttle/RPM left.
- **Sparklines.** `AppState::attitude_history` and `pid_history` are already populated
  on every poll and read by nothing. Attitude error, PID corrections, RPM vs target,
  loop time and pack voltage are all worth a trace.
- **Theme module**: replace the scattered `Color::*` literals with semantic roles
  (`ok`/`warn`/`crit`/`muted`/`accent`). `src/tui/ui.rs` has a partial set at the top;
  finishing it makes a palette change one file and light terminals possible.
- **Responsive and tabbed layout** (F1 Flight / F2 Sensors / F3 Tuning / F4 Logs). The
  40/25/35 split and fixed row heights mean a small terminal loses the horizon.
- **Smooth attitude**: interpolate between the 10 Hz samples and redraw at ~30 Hz. The
  horizon is visibly stepped today.

### Large — project-scale

- **Live tuning view**: command a step and plot the RPM/attitude response inline.
- **Mag-cal coverage visualiser**: three projections of sample coverage, replacing
  "rotate for 30 s and hope".
- **ULog replay** through the same dashboard for post-flight review.
- **GPS track plot**.
