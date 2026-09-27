//! Host tests for the autotune result sanity checks.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test autotune_bounds

use elle_control::SavedGains;
use elle_control::autotune::{AutotuneReject, validate_result};

fn gains(kp: f32) -> SavedGains {
    SavedGains {
        pitch_kp: kp,
        pitch_ki: 0.02,
        pitch_kd: 0.16,
        roll_kp: 0.25,
        roll_ki: 0.012,
        roll_kd: 0.07,
        scale: 5.0,
        i_limit: 0.5,
    }
}

#[test]
fn realistic_result_passes() {
    // 5 deg relay, 4 deg swing, 0.8 s period, ordinary gains.
    assert_eq!(validate_result(4.0, 5.0, 0.8, &gains(0.6)), Ok(()));
}

#[test]
fn noise_level_amplitude_is_rejected() {
    // The finding's example: 0.2 deg against a 5 deg relay.
    assert_eq!(
        validate_result(0.2, 5.0, 0.8, &gains(0.6)),
        Err(AutotuneReject::AmplitudeTooSmall)
    );
    // Below 20 % of the relay, even if above the absolute floor.
    assert_eq!(
        validate_result(0.9, 5.0, 0.8, &gains(0.6)),
        Err(AutotuneReject::AmplitudeTooSmall)
    );
    // The absolute floor applies to small relays.
    assert_eq!(
        validate_result(0.4, 1.0, 0.8, &gains(0.6)),
        Err(AutotuneReject::AmplitudeTooSmall)
    );
    assert_eq!(
        validate_result(f32::NAN, 5.0, 0.8, &gains(0.6)),
        Err(AutotuneReject::AmplitudeTooSmall)
    );
}

#[test]
fn implausible_period_is_rejected() {
    for tu in [0.0, 0.05, 6.0, f32::INFINITY, f32::NAN] {
        assert_eq!(
            validate_result(4.0, 5.0, tu, &gains(0.6)),
            Err(AutotuneReject::PeriodOutOfRange),
            "tu {tu}"
        );
    }
}

/// Gains the flash loader would refuse must never be applied or saved.
#[test]
fn gains_the_loader_rejects_are_rejected() {
    for kp in [148.0, -1.0, f32::INFINITY] {
        let g = gains(kp);
        assert!(SavedGains::from_bytes(&g.to_bytes()).is_none());
        assert_eq!(
            validate_result(4.0, 5.0, 0.8, &g),
            Err(AutotuneReject::GainsOutOfRange),
            "kp {kp}"
        );
    }
}

#[test]
fn accepted_gains_round_trip_through_the_loader() {
    let g = gains(0.6);
    assert_eq!(validate_result(4.0, 5.0, 0.8, &g), Ok(()));
    assert!(SavedGains::from_bytes(&g.to_bytes()).is_some());
}
