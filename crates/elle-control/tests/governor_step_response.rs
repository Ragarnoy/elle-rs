//! Governor step-response regression tests.
//!
//! These exist because of a latch-up found on the bench: dropping the TUI throttle
//! from 40% to 10% left the engine commanded off permanently (`cmd:0`, `0 RPM`,
//! against a live 1336 RPM target), while 20% -> 10% behaved fine. Three pieces
//! closed a loop:
//!
//!   1. `dshot.rs` sent `MotorStop` whenever the *governor output* was 0 and then
//!      fabricated `erpm = 0, valid = true` for the cache;
//!   2. the spike filter here rejected that fabricated 0 (jump > GOVERNOR_ERPM_MAX_JUMP)
//!      and kept the stale pre-chop `last_measured` forever, since no telemetry is
//!      read while MotorStop is going out;
//!   3. the frozen huge negative error clamped the output back to 0. Repeat forever.
//!
//! Run on the host: `cargo test -p elle-control --target x86_64-unknown-linux-gnu
//! --features elle-config/platform-dart`.

#![cfg(feature = "platform-dart")]

use elle_control::governor::RpmGovernor;

/// Full-stick target, matching `MAX_ERPM` for the dart.
const MAX_ERPM: u32 = 93_968;
const POLE_PAIRS: u32 = 7;
/// Motor + prop spin-up/down time constant, seconds.
const TAU: f32 = 0.15;
const DT: f32 = 0.001;

/// Steady-state eRPM the dart reaches at a given DShot value — the inverse of the
/// measured `GOVERNOR_FF_TABLE`, used here as a plant model.
fn erpm_steady_state(dshot: u16) -> f32 {
    const T: [(f32, f32); 18] = [
        (2534., 48.),
        (9569., 148.),
        (17997., 248.),
        (25816., 348.),
        (33096., 448.),
        (39690., 548.),
        (46270., 648.),
        (53039., 748.),
        (57988., 848.),
        (63126., 948.),
        (68810., 1048.),
        (73325., 1148.),
        (77287., 1248.),
        (80808., 1348.),
        (84462., 1448.),
        (87955., 1548.),
        (91084., 1648.),
        (93968., 1748.),
    ];
    let d = f32::from(dshot);
    if d <= T[0].1 {
        return T[0].0 * d / T[0].1;
    }
    for i in 1..T.len() {
        if d <= T[i].1 {
            let (e0, d0) = T[i - 1];
            let (e1, d1) = T[i];
            return e0 + (d - d0) * (e1 - e0) / (d1 - d0);
        }
    }
    T[T.len() - 1].0
}

/// Stick percentage -> governor target eRPM, mirroring `FlightController::engine_output`
/// scaling in `system.rs` and the `MAX_ERPM` rescale in the platform mains.
fn target_for(stick_pct: f32) -> u32 {
    (stick_pct * 1999.0) as u32 * MAX_ERPM / 1999
}

/// One engine plus the parts of the DShot task that close the loop around the governor.
struct Rig {
    gov: RpmGovernor,
    erpm: f32,
}

impl Rig {
    fn new() -> Self {
        Self {
            gov: RpmGovernor::new(),
            erpm: 0.0,
        }
    }

    /// Hold a target for `secs`, returning the final commanded DShot value.
    ///
    /// Post-fix `update_engine_unit` behaviour: a zero reading is only reported when a
    /// stop was actually commanded (target == 0). Otherwise the engine keeps spinning
    /// at minimum throttle and real telemetry keeps flowing.
    fn hold(&mut self, target: u32, secs: f32) -> u16 {
        self.run(target, secs, false)
    }

    /// As `hold`, but with `report_zero_on_zero_cmd` reproducing the *old* DShot task:
    /// a governor output of 0 meant MotorStop, no telemetry was read, and `erpm = 0`
    /// was fabricated for the cache while the prop was still spinning. The governor
    /// must survive that on its own — a caller reporting a bogus measurement should
    /// cost a few ticks, not the engine.
    fn run(&mut self, target: u32, secs: f32, report_zero_on_zero_cmd: bool) -> u16 {
        let mut cmd = 0;
        for _ in 0..(secs / DT) as usize {
            let stopped = target == 0 || (report_zero_on_zero_cmd && cmd == 0);
            let reported = if stopped { 0 } else { self.erpm as u32 };
            cmd = self.gov.update(target, reported, true);
            self.erpm += (erpm_steady_state(cmd) - self.erpm) * DT / TAU;
        }
        cmd
    }

    fn rpm(&self) -> u32 {
        self.erpm as u32 / POLE_PAIRS
    }
}

/// A large downward step must settle at the new target, not latch the engine off.
/// This is the exact 40% -> 10% case that failed on the bench.
#[test]
fn large_step_down_settles_and_does_not_latch_off() {
    let hi = target_for(0.40);
    let lo = target_for(0.10);
    let mut rig = Rig::new();

    rig.hold(hi, 3.0);
    assert!(
        rig.rpm().abs_diff(hi / POLE_PAIRS) < 100,
        "did not reach the 40% target: {} RPM",
        rig.rpm()
    );

    let cmd = rig.hold(lo, 5.0);
    assert!(cmd > 0, "engine latched off (cmd 0) with a live target");
    assert!(
        rig.rpm().abs_diff(lo / POLE_PAIRS) < 100,
        "did not settle at the 10% target: {} RPM (cmd {cmd})",
        rig.rpm()
    );
}

/// The small step that worked on the bench — kept so a fix to the large-step case
/// can't regress the case that was already fine.
#[test]
fn small_step_down_settles() {
    let lo = target_for(0.10);
    let mut rig = Rig::new();

    rig.hold(target_for(0.20), 3.0);
    let cmd = rig.hold(lo, 5.0);

    assert!(cmd > 0, "engine latched off (cmd 0) with a live target");
    assert!(
        rig.rpm().abs_diff(lo / POLE_PAIRS) < 100,
        "did not settle at the 10% target: {} RPM (cmd {cmd})",
        rig.rpm()
    );
}

/// A commanded stop must still stop the engine.
#[test]
fn zero_target_commands_stop() {
    let mut rig = Rig::new();
    rig.hold(target_for(0.40), 3.0);
    assert_eq!(rig.hold(0, 1.0), 0, "zero target must command 0");
}

/// The spike filter must not be able to latch shut on a stale reading: once a
/// plausible-but-distant value keeps arriving, it has to be adopted. Guards the
/// `MAX_CONSECUTIVE_SPIKE_REJECTS` bound directly, independent of the plant model.
#[test]
fn spike_filter_cannot_latch_on_stale_reading() {
    let target = target_for(0.10);
    let mut gov = RpmGovernor::new();

    // Establish a high `last_measured`, then jump far away from it — more than
    // GOVERNOR_ERPM_MAX_JUMP (20_000) so every reading looks like a spike.
    gov.update(target_for(0.40), 37_000, true);

    let mut cmd = 0;
    for _ in 0..200 {
        cmd = gov.update(target, 0, true);
    }
    assert!(
        cmd > 0,
        "spike filter latched on the stale reading: still commanding {cmd}"
    );
}

/// Defense in depth for the original bench failure: even with a caller that
/// fabricates `erpm = 0` whenever it sees a zero command — the pre-fix DShot task —
/// the governor must claw its way back rather than sitting at 0 forever.
#[test]
fn large_step_down_recovers_even_if_caller_reports_zero() {
    let lo = target_for(0.10);
    let mut rig = Rig::new();

    rig.run(target_for(0.40), 3.0, true);
    let cmd = rig.run(lo, 5.0, true);

    assert!(
        cmd > 0,
        "governor latched off against a caller that reports zero on zero command"
    );
    assert!(
        rig.rpm().abs_diff(lo / POLE_PAIRS) < 200,
        "did not settle at the 10% target: {} RPM (cmd {cmd})",
        rig.rpm()
    );
}
