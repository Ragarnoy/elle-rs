//! Host tests for the autotune run: safety aborts, cycle capacity, run conditions.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test autotune_run

use elle_config::CONTROL_LOOP_FREQUENCY_HZ;
use elle_control::SavedGains;
use elle_control::autotune::{
    AUTOTUNE_MAX_CYCLES, AutotuneAction, AutotuneAxis, AutotuneStop, Autotuner, TuningRule,
    check_run_conditions,
};

const SETTLE_TICKS: u32 = 2 * CONTROL_LOOP_FREQUENCY_HZ;

/// Flown gains, within `AUTOTUNE_MAX_GAIN_RATIO` of what `sine` identifies
/// (Kp ~1.2, Kd ~0.15) on either axis.
fn gains() -> SavedGains {
    SavedGains {
        pitch_kp: 0.6,
        pitch_ki: 0.02,
        pitch_kd: 0.16,
        roll_kp: 0.6,
        roll_ki: 0.012,
        roll_kd: 0.16,
        scale: 5.0,
        i_limit: 0.5,
    }
}

fn started(axis: AutotuneAxis, cycles: usize) -> Autotuner {
    let mut t = Autotuner::new();
    t.start(axis, gains(), 5.0, cycles, TuningRule::TyreusLuyben, 0);
    t
}

/// A clean 4 deg, 0.8 s oscillation on the tuned axis, the other axis level.
fn sine(tick: u32) -> f32 {
    let period_ticks = 0.8 * CONTROL_LOOP_FREQUENCY_HZ as f32;
    4.0 * (core::f32::consts::TAU * tick as f32 / period_ticks).sin()
}

/// Feed `f(tick)` as (pitch, roll) until the run ends; return the final action.
fn run(t: &mut Autotuner, f: impl Fn(u32) -> (f32, f32)) -> AutotuneAction {
    for tick in 0..60 * CONTROL_LOOP_FREQUENCY_HZ + 10 {
        let (p, r) = f(tick);
        let action = t.update(p, r, tick);
        if !t.is_active() {
            return action;
        }
    }
    panic!("run never ended");
}

#[test]
fn clean_oscillation_completes_with_valid_gains() {
    let mut t = started(AutotuneAxis::Pitch, 6);
    match run(&mut t, |k| (sine(k), 0.0)) {
        AutotuneAction::Completed(r) => {
            assert_eq!(r.axis, AutotuneAxis::Pitch);
            assert_eq!(r.cycles, 6);
            assert!((r.tu_s - 0.8).abs() < 0.05, "tu {}", r.tu_s);
            assert!(t.computed_gains().is_some_and(|g| g.is_valid()));
        }
        other => panic!("expected Completed, got {other:?}"),
    }
}

#[test]
fn measures_its_own_axis() {
    // Roll run: the oscillation is on roll, pitch stays level.
    let mut t = started(AutotuneAxis::Roll, 6);
    assert!(matches!(
        run(&mut t, |k| (0.0, sine(k))),
        AutotuneAction::Completed(_)
    ));
    // Roll run fed a pitch oscillation sees nothing and times out.
    let mut t = started(AutotuneAxis::Roll, 6);
    assert!(matches!(
        run(&mut t, |k| (sine(k), 0.0)),
        AutotuneAction::RestoreGains(_)
    ));
}

/// +/-`amp` alternating every tick.
fn chatter(tick: u32, amp: f32) -> f32 {
    if tick.is_multiple_of(2) { amp } else { -amp }
}

#[test]
fn noise_alone_never_counts_as_oscillation() {
    // The review's case: +/-0.01 deg alternating every tick. Inside the
    // hysteresis band it makes no crossings, so the run times out.
    let mut t = started(AutotuneAxis::Pitch, 6);
    assert!(matches!(
        run(&mut t, |k| (chatter(k, 0.01), 0.0)),
        AutotuneAction::RestoreGains(_)
    ));
    assert_eq!(t.cycles_completed(), 0);
    assert!(t.computed_gains().is_none());
}

#[test]
fn noise_on_a_real_oscillation_does_not_fake_crossings() {
    // 0.4 deg of chatter would add crossings around every zero without
    // hysteresis and fail the period-consistency check.
    let mut t = started(AutotuneAxis::Pitch, 6);
    match run(&mut t, |k| (sine(k) + chatter(k, 0.4), 0.0)) {
        AutotuneAction::Completed(r) => {
            assert_eq!(r.cycles, 6);
            assert!((r.tu_s - 0.8).abs() < 0.05, "tu {}", r.tu_s);
        }
        other => panic!("expected Completed, got {other:?}"),
    }
}

#[test]
fn relay_holds_inside_the_hysteresis_band() {
    let mut t = started(AutotuneAxis::Pitch, 6);
    // Settle, then start positive: the relay pushes negative.
    let mut tick = 0;
    while t.phase_u8() == 1 {
        let _ = t.update(0.0, 0.0, tick);
        tick += 1;
    }
    let _ = t.update(1.0, 0.0, tick);
    assert!(!t.relay_positive());
    // Dipping just below zero, inside the band, does not flip it.
    let _ = t.update(-0.3, 0.0, tick + 1);
    assert!(!t.relay_positive());
    // Past the band it flips.
    let _ = t.update(-0.6, 0.0, tick + 2);
    assert!(t.relay_positive());
}

#[test]
fn envelope_is_enforced_while_settling() {
    let mut t = started(AutotuneAxis::Pitch, 6);
    assert!(matches!(
        t.update(30.0, 0.0, 1),
        AutotuneAction::RestoreGains(g) if g.pitch_kp == gains().pitch_kp
    ));
    assert!(!t.is_active());
}

#[test]
fn envelope_covers_the_other_axis() {
    // Pitch run, roll departs past 20 deg during the relay.
    let mut t = started(AutotuneAxis::Pitch, 6);
    let action = run(&mut t, |k| {
        let roll = if k > SETTLE_TICKS + 50 { 25.0 } else { 0.0 };
        (sine(k), roll)
    });
    assert!(matches!(action, AutotuneAction::RestoreGains(_)));
}

#[test]
fn non_finite_attitude_aborts() {
    for (p, r) in [(f32::NAN, 0.0), (0.0, f32::INFINITY)] {
        let mut t = started(AutotuneAxis::Roll, 6);
        assert!(matches!(t.update(p, r, 1), AutotuneAction::RestoreGains(_)));
        assert!(!t.is_active());
    }
}

#[test]
fn cycle_request_is_clamped_to_capacity() {
    assert_eq!(AUTOTUNE_MAX_CYCLES, 14);
    assert_eq!(started(AutotuneAxis::Pitch, 20).num_cycles(), 14);
    assert_eq!(started(AutotuneAxis::Pitch, 0).num_cycles(), 1);

    // A max-length run reports every cycle it was asked for.
    let mut t = started(AutotuneAxis::Pitch, AUTOTUNE_MAX_CYCLES);
    match run(&mut t, |k| (sine(k), 0.0)) {
        AutotuneAction::Completed(r) => assert_eq!(r.cycles, AUTOTUNE_MAX_CYCLES),
        other => panic!("expected Completed, got {other:?}"),
    }
}

#[test]
fn run_conditions() {
    assert_eq!(check_run_conditions(true, true, false, true), Ok(()));
    assert_eq!(
        check_run_conditions(false, false, true, true),
        Err(AutotuneStop::Killed)
    );
    assert_eq!(
        check_run_conditions(false, false, false, true),
        Err(AutotuneStop::Disarmed)
    );
    assert_eq!(
        check_run_conditions(true, false, false, true),
        Err(AutotuneStop::NotStabilized)
    );
    assert_eq!(
        check_run_conditions(true, true, false, false),
        Err(AutotuneStop::AttitudeLost)
    );
}
