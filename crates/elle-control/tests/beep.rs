//! Arm/disarm beeps never take a frame slot from throttle or a stop.

use elle_config::{BEEP_AT_REST_ERPM, BEEP_GAP_MS, BEEP_MAX_WAIT_MS, BEEP_SPINDOWN_FALLBACK_MS};
use elle_control::beep::{BURST_FRAMES, BeepSequencer, Pattern, Tone, engine_at_rest};

/// Run `ms` 1 ms slots from `t0`, collecting what each slot sends.
fn run(s: &mut BeepSequencer, t0: u64, ms: u64, spin: bool, at_rest: bool) -> Vec<Option<Tone>> {
    (t0..t0 + ms).map(|t| s.next(t, spin, at_rest)).collect()
}

#[test]
fn disarm_beep_waits_for_rest_and_never_takes_a_slot_before() {
    let mut s = BeepSequencer::new();
    s.request(Pattern::Disarm, 0);
    // Spinning down: every slot is a normal (stop) frame.
    assert!(
        run(&mut s, 0, 500, false, false)
            .iter()
            .all(Option::is_none)
    );
    // At rest: two bursts of Beep2 with the gap between them.
    let out = run(&mut s, 500, 400, false, true);
    let tones: Vec<usize> = out
        .iter()
        .enumerate()
        .filter_map(|(i, t)| t.map(|_| i))
        .collect();
    assert_eq!(tones.len(), 2 * BURST_FRAMES as usize);
    assert!(out.iter().flatten().all(|t| *t == Tone::Beep2));
    let first_end = tones[BURST_FRAMES as usize - 1];
    let second_start = tones[BURST_FRAMES as usize];
    assert_eq!((second_start - first_end) as u64, BEEP_GAP_MS);
    assert!(!s.busy());
}

#[test]
fn arm_beep_is_one_burst_of_beep1() {
    let mut s = BeepSequencer::new();
    s.request(Pattern::Arm, 0);
    let out = run(&mut s, 0, 100, false, true);
    assert_eq!(out.iter().flatten().count(), BURST_FRAMES as usize);
    assert!(out.iter().flatten().all(|t| *t == Tone::Beep1));
    assert!(out[..BURST_FRAMES as usize].iter().all(Option::is_some));
}

#[test]
fn a_nonzero_target_cancels_a_beep_mid_burst() {
    let mut s = BeepSequencer::new();
    s.request(Pattern::Arm, 0);
    assert_eq!(s.next(0, false, true), Some(Tone::Beep1));
    assert_eq!(s.next(1, false, true), Some(Tone::Beep1));
    // Throttle up right after arming: the very next slot is a throttle frame.
    assert_eq!(s.next(2, true, false), None);
    assert!(!s.busy());
    // And the beep does not come back once the target is zero again.
    assert!(run(&mut s, 3, 100, false, true).iter().all(Option::is_none));
}

#[test]
fn a_nonzero_target_drops_a_pending_beep() {
    let mut s = BeepSequencer::new();
    s.request(Pattern::Disarm, 0);
    assert_eq!(s.next(0, false, false), None);
    assert_eq!(s.next(1, true, false), None);
    assert!(!s.busy());
}

#[test]
fn a_beep_that_cannot_start_in_time_is_dropped() {
    let mut s = BeepSequencer::new();
    s.request(Pattern::Disarm, 0);
    assert!(
        run(&mut s, 0, BEEP_MAX_WAIT_MS + 1, false, false)
            .iter()
            .all(Option::is_none)
    );
    // At rest only after the deadline: nothing plays.
    assert_eq!(s.next(BEEP_MAX_WAIT_MS + 1, false, true), None);
    assert!(!s.busy());
}

#[test]
fn a_new_request_replaces_the_one_playing() {
    let mut s = BeepSequencer::new();
    s.request(Pattern::Disarm, 0);
    assert_eq!(s.next(0, false, true), Some(Tone::Beep2));
    s.request(Pattern::Arm, 1);
    let out = run(&mut s, 1, 50, false, true);
    assert!(out.iter().flatten().all(|t| *t == Tone::Beep1));
    assert_eq!(out.iter().flatten().count(), BURST_FRAMES as usize);
}

#[test]
fn rest_from_telemetry_or_from_time() {
    // Target non-zero: never at rest.
    assert!(!engine_at_rest(Some(0), None));
    // Telemetry decides when it is valid.
    assert!(engine_at_rest(Some(0), Some(0)));
    assert!(!engine_at_rest(Some(BEEP_AT_REST_ERPM), Some(5_000)));
    // No telemetry: the spin-down allowance.
    assert!(!engine_at_rest(None, Some(BEEP_SPINDOWN_FALLBACK_MS - 1)));
    assert!(engine_at_rest(None, Some(BEEP_SPINDOWN_FALLBACK_MS)));
}
