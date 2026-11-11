//! Proposal 0007: moving `FlightController::update()` into
//! `elle_control::flight_law` changes no behaviour.
//!
//! `legacy` is `crates/elle-system/src/system.rs` from the base commit, byte for
//! byte (checked by hash below). `current` is today's `system.rs`, the same
//! file the firmware builds. Both compile here against the same host shims
//! (`shims.rs`: PWM, events, Core 1 heartbeat, watchdog) on embassy-time's mock
//! clock. Every operation of each recorded stream (`*.stream`) runs on both, and
//! after each one the writes, events (with their messages), every query and the
//! whole output snapshot must match, floats bit for bit.
//!
//! Remove `legacy`, its copy and this oracle once 0007 is Implemented.

#[path = "flight_law_equivalence/scenario.rs"]
mod scenario;
#[path = "flight_law_equivalence/shims.rs"]
mod shims;
#[path = "flight_law_equivalence/stream.rs"]
mod stream;

#[allow(
    dead_code,
    unused_imports,
    unused_variables,
    unexpected_cfgs,
    clippy::all,
    clippy::pedantic
)]
mod legacy {
    use crate::shims::{elle_hardware, embassy_rp};
    include!("flight_law_equivalence/legacy_system.rs");
}

#[allow(
    dead_code,
    unused_imports,
    unexpected_cfgs,
    clippy::all,
    clippy::pedantic
)]
mod current {
    use crate::shims::{elle_hardware, embassy_rp};
    include!("../../elle-system/src/system.rs");
}

use elle_control::commands::{AttitudeMode, NormalizedCommands, PilotCommands, RawCommands};
use elle_control::pid::PidConfig;
use embassy_time::{Duration, Instant, MockDriver};
use shims::elle_hardware::imu::{AttitudeData, CORE1_HEARTBEAT};
use shims::elle_hardware::pwm::PwmOutputs;
use shims::{Effect, take_effects};
use stream::{Cmd, Op};

/// The commit whose `system.rs` is `legacy_system.rs`.
const BASE_COMMIT: &str = "3438ced";
/// SHA-256 of `git show 3438ced:crates/elle-system/src/system.rs`.
const BASE_SYSTEM_RS_SHA256: &str =
    "992eef7f5f758edd03a78b329143460d08ee5d522840757f2d0105ce5de2783d";

#[test]
fn legacy_copy_is_the_base_commit_file() {
    use sha2::{Digest, Sha256};
    let text = include_bytes!("flight_law_equivalence/legacy_system.rs");
    let hash: String = Sha256::digest(text)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    assert_eq!(
        hash, BASE_SYSTEM_RS_SHA256,
        "legacy_system.rs is no longer {BASE_COMMIT}'s system.rs"
    );
}

/// Every field of a snapshot, as bits. Destructured without `..`, so a field
/// added to either type and not compared here fails to compile.
macro_rules! snapshot_bits {
    ($s:expr, $ty:path) => {{
        let $ty {
            pitch_correction,
            roll_correction,
            pitch_setpoint_deg,
            roll_setpoint_deg,
            elevon_left_us,
            elevon_right_us,
            engine_left_dshot,
            engine_right_dshot,
            heading_hold_active,
            heading_target_deg,
            heading_error_deg,
            dt_us,
            att_age_us,
            pitch_sp_used_deg,
            roll_sp_used_deg,
            pitch_terms,
            roll_terms,
            saturation,
            elevon_left_pulse_us,
            elevon_right_pulse_us,
            yaw_damp,
            pwm_counter_us,
        } = *$s;
        vec![
            u64::from(pitch_correction.to_bits()),
            u64::from(roll_correction.to_bits()),
            u64::from(pitch_setpoint_deg.to_bits()),
            u64::from(roll_setpoint_deg.to_bits()),
            u64::from(elevon_left_us),
            u64::from(elevon_right_us),
            u64::from(engine_left_dshot),
            u64::from(engine_right_dshot),
            u64::from(heading_hold_active),
            u64::from(heading_target_deg.to_bits()),
            u64::from(heading_error_deg.to_bits()),
            u64::from(dt_us),
            u64::from(att_age_us),
            u64::from(pitch_sp_used_deg.to_bits()),
            u64::from(roll_sp_used_deg.to_bits()),
            u64::from(pitch_terms.p.to_bits()),
            u64::from(pitch_terms.i.to_bits()),
            u64::from(pitch_terms.d.to_bits()),
            u64::from(roll_terms.p.to_bits()),
            u64::from(roll_terms.i.to_bits()),
            u64::from(roll_terms.d.to_bits()),
            u64::from(saturation.bits()),
            u64::from(elevon_left_pulse_us),
            u64::from(elevon_right_pulse_us),
            u64::from(yaw_damp.to_bits()),
            u64::from(pwm_counter_us),
        ]
    }};
}

/// Everything observable about a controller after an operation.
#[derive(Debug, PartialEq)]
struct Observed {
    returned: Option<bool>,
    effects: Vec<Effect>,
    armed: bool,
    failsafe: bool,
    attitude_enabled: bool,
    mode: String,
    heading_hold: bool,
    rc_link: String,
    rc_age_ms: u16,
    gains_version: u32,
    gains: [u32; 8],
    engines: (u16, u16),
    snapshot: Vec<u64>,
}

fn gains_bits(c: &PidConfig) -> [u32; 8] {
    [
        c.kp_pitch, c.ki_pitch, c.kd_pitch, c.kp_roll, c.ki_roll, c.kd_roll, c.i_limit, c.scale,
    ]
    .map(f32::to_bits)
}

/// Same calls on either controller type (their APIs are identical).
macro_rules! apply {
    ($fc:expr, $snap_ty:path, $op:expr, $base:expr) => {{
        let fc = &mut $fc;
        let at = |us: u64| $base + Duration::from_micros(us);
        let mut returned = None;
        match *$op {
            Op::Tick {
                rx_us, cmd, att, ..
            } => {
                let commands = match cmd {
                    Cmd::Raw(channels) => PilotCommands::Raw(RawCommands {
                        channels,
                        timestamp: at(rx_us),
                    }),
                    Cmd::Norm {
                        throttle,
                        pitch,
                        roll,
                        yaw,
                        mode,
                    } => PilotCommands::Normalized(NormalizedCommands {
                        throttle,
                        pitch,
                        roll,
                        yaw,
                        attitude_mode: match mode {
                            'M' => AttitudeMode::Manual,
                            'S' => AttitudeMode::Stabilized,
                            'A' => AttitudeMode::AltitudeHold,
                            _ => panic!("mode {mode}"),
                        },
                        timestamp: at(rx_us),
                    }),
                };
                let attitude = att.map(|a| AttitudeData {
                    pitch: a.pitch,
                    roll: a.roll,
                    yaw: a.yaw,
                    pitch_rate: a.pitch_rate,
                    roll_rate: a.roll_rate,
                    yaw_rate: a.yaw_rate,
                    timestamp: at(a.t_us),
                });
                fc.update(&commands, attitude.as_ref());
            }
            Op::Wait(_) => {}
            Op::Rc(t) => fc.note_rc_packet(at(t)),
            Op::Failsafe => fc.check_failsafe(),
            Op::Supervise(heartbeat) => {
                if heartbeat {
                    CORE1_HEARTBEAT.signal(());
                }
                returned = Some(fc.supervisor_check());
                CORE1_HEARTBEAT.reset();
            }
            Op::EnableSupervisor => fc.enable_supervisor_monitoring(),
            Op::Arm => fc.arm(),
            Op::Disarm => fc.disarm(),
            Op::Safe => fc.set_safe_positions(),
            Op::ClearOverride => fc.clear_setpoint_override(),
            Op::Unheading => fc.disengage_heading_hold(),
            Op::Explicit(on) => fc.set_explicit_arming(on),
            Op::Override(p, r) => fc.set_setpoint_override(p, r),
            Op::Heading(rad) => fc.engage_heading_hold(rad),
            Op::Gains(g) => fc.set_pid_gains(PidConfig {
                kp_pitch: g[0],
                ki_pitch: g[1],
                kd_pitch: g[2],
                kp_roll: g[3],
                ki_roll: g[4],
                kd_roll: g[5],
                i_limit: g[6],
                scale: g[7],
            }),
        }
        Observed {
            returned,
            effects: take_effects(),
            armed: fc.is_armed(),
            failsafe: fc.is_failsafe(),
            attitude_enabled: fc.is_attitude_enabled(),
            mode: format!("{:?}", fc.current_control_mode()),
            heading_hold: fc.is_heading_hold_active(),
            rc_link: format!("{:?}", fc.rc_link_state()),
            rc_age_ms: fc.rc_signal_age_ms(),
            gains_version: fc.gains_version(),
            gains: gains_bits(fc.pid_config()),
            engines: fc.engine_output(),
            snapshot: snapshot_bits!(fc.last_output(), $snap_ty),
        }
    }};
}

/// Counts of what a stream exercised, for its coverage checks.
#[derive(Debug, Default)]
struct Coverage {
    ticks: usize,
    events: std::collections::BTreeMap<u16, usize>,
    safe_writes: usize,
    armed_ticks: usize,
    attitude_enabled_ticks: usize,
    heading_ticks: usize,
    saturated_ticks: usize,
    pid_i_nonzero: usize,
}

/// Run one stream through both controllers; panics at the first difference.
fn run(name: &str, ops: &[Op]) -> Coverage {
    take_effects();
    CORE1_HEARTBEAT.reset();
    let base = Instant::now() + Duration::from_secs(1);
    MockDriver::get().advance(Duration::from_secs(1));
    let mut old = legacy::FlightController::new(PwmOutputs::new());
    let mut new = current::FlightController::new(PwmOutputs::new());
    let mut clock_us = 0u64;
    let mut cov = Coverage::default();

    for (i, op) in ops.iter().enumerate() {
        if let Op::Tick { t_us, .. } | Op::Wait(t_us) = *op {
            assert!(t_us >= clock_us, "{name}: time goes backwards at op {i}");
            MockDriver::get().advance(Duration::from_micros(t_us - clock_us));
            clock_us = t_us;
        }
        let a = apply!(old, legacy::ControllerOutputSnapshot, op, base);
        let b = apply!(new, current::ControllerOutputSnapshot, op, base);
        assert_eq!(
            a, b,
            "{name}: op {i} ({op:?}) differs: legacy left, current right"
        );

        for e in &b.effects {
            match e {
                Effect::Event { code, .. } => *cov.events.entry(*code).or_default() += 1,
                Effect::Safe => cov.safe_writes += 1,
                Effect::Trimmed { .. } => {}
            }
        }
        if matches!(op, Op::Tick { .. }) {
            cov.ticks += 1;
            cov.armed_ticks += usize::from(b.armed);
            cov.attitude_enabled_ticks += usize::from(b.attitude_enabled);
            cov.heading_ticks += usize::from(b.heading_hold);
            // saturation bits, PID I terms (see snapshot_bits! order)
            cov.saturated_ticks += usize::from(b.snapshot[21] != 0);
            cov.pid_i_nonzero += usize::from(
                f32::from_bits(b.snapshot[16] as u32) != 0.0
                    || f32::from_bits(b.snapshot[19] as u32) != 0.0,
            );
        }
    }
    cov
}

fn load(file: &str) -> Vec<Op> {
    let path = format!(
        "{}/tests/flight_law_equivalence/{file}",
        env!("CARGO_MANIFEST_DIR")
    );
    stream::parse(&std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}")))
}

/// The mock clock is global, so every stream runs in this one test.
#[test]
fn legacy_and_current_controllers_agree() {
    let scripted = load("scripted.stream");
    let cov = run("scripted.stream", &scripted);
    println!("scripted.stream: {cov:?}");
    // The scripted stream must reach what it was written to reach.
    use shims::elle_hardware::event::*;
    for code in [
        EVT_RC_WARNING,
        EVT_RC_SIGNAL_LOST,
        EVT_RC_RESTORED,
        EVT_CORE1_UNHEALTHY,
        EVT_CORE1_RESTORED,
        EVT_ATTITUDE_HELD,
        EVT_ATTITUDE_STALE,
    ] {
        assert!(
            cov.events.contains_key(&code),
            "scripted stream never emits event {code}"
        );
    }
    assert!(cov.safe_writes > 0, "no failsafe hold");
    assert!(
        cov.attitude_enabled_ticks > 100,
        "too little attitude control"
    );
    assert!(cov.heading_ticks > 20, "too little heading hold");
    assert!(cov.saturated_ticks > 5, "no mixer saturation");
    assert!(cov.pid_i_nonzero > 50, "integrators never ran");

    let flight = load("flight.stream");
    let cov = run("flight.stream", &flight);
    println!("flight.stream: {cov:?}");
    assert!(cov.ticks > 1000, "flight stream too short");
    assert!(
        cov.pid_i_nonzero > 1000,
        "flight stream never runs the integrators"
    );
}

/// The scripted stream is written by `scenario.rs`; this keeps the committed
/// file in step with it.
#[test]
fn scripted_stream_matches_its_scenario() {
    let file = load("scripted.stream");
    assert!(
        file == scenario::scripted(),
        "scripted.stream is out of date: run `cargo test -p elle-control --target \
         x86_64-unknown-linux-gnu --test flight_law_equivalence -- --ignored write_scripted_stream`"
    );
}

#[test]
#[ignore = "writes tests/flight_law_equivalence/scripted.stream"]
fn write_scripted_stream() {
    let path = format!(
        "{}/tests/flight_law_equivalence/scripted.stream",
        env!("CARGO_MANIFEST_DIR")
    );
    let mut text = String::from(
        "# Proposal 0007 scripted stream, written by scenario.rs (write_scripted_stream).\n",
    );
    text.push_str(&stream::write(&scenario::scripted()));
    std::fs::write(&path, text).unwrap();
}
