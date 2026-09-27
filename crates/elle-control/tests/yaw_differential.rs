//! Host tests for yaw → differential thrust.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test yaw_differential
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --features platform-dart --test yaw_differential

use elle_config::lut::apply_differential_thrust_lut;

const BASE: u16 = 1000;
const CENTER: u16 = 1024;
const FULL_RIGHT: u16 = 2047;
const FULL_LEFT: u16 = 0;

#[test]
fn centred_yaw_leaves_both_engines_at_base() {
    assert_eq!(apply_differential_thrust_lut(BASE, CENTER), (BASE, BASE));
}

/// Single engine: it is driven by the left target only, so yaw must never
/// change either target — any reduction would be lost thrust, one way only.
#[cfg(feature = "platform-dart")]
#[test]
fn dart_yaw_never_changes_thrust() {
    for yaw in (0..=2047u16).step_by(7).chain([FULL_LEFT, FULL_RIGHT]) {
        assert_eq!(
            apply_differential_thrust_lut(BASE, yaw),
            (BASE, BASE),
            "yaw {yaw}"
        );
    }
}

/// Twin engine: right yaw slows the left engine, left yaw the right one, by at
/// most 20 % (the 0.8 floor is 819/1024 in fixed point, so full stick lands on
/// 799 or 800 of 1000 after truncation).
#[cfg(not(feature = "platform-dart"))]
#[test]
fn eagle_yaw_slows_the_inside_engine_up_to_20_percent() {
    let (l, r) = apply_differential_thrust_lut(BASE, FULL_RIGHT);
    assert!(l < BASE && r == BASE, "{l} {r}");
    assert!(l >= 799, "{l}");
    let (l, r) = apply_differential_thrust_lut(BASE, FULL_LEFT);
    assert!(r < BASE && l == BASE, "{l} {r}");
    assert!(r >= 799, "{r}");
}
