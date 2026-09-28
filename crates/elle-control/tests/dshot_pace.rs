//! Host tests for the DShot loop pacer.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test dshot_pace

use elle_control::dshot_pace::next_deadline_us;

const PERIOD: u64 = 1_000;
const GAP: u64 = 200;

#[test]
fn on_schedule_keeps_the_period() {
    // Loop body took 300 us: next frame on the 1 ms grid.
    assert_eq!(next_deadline_us(10_000, 10_300, PERIOD, GAP), 11_000);
}

#[test]
fn stall_is_not_replayed() {
    // 30 ms stall: one frame a gap after "now", then the 1 ms grid from there.
    let mut deadline = next_deadline_us(10_000, 40_000, PERIOD, GAP);
    assert_eq!(deadline, 40_200);
    for k in 1..=5 {
        // Each iteration finishes shortly after its deadline.
        deadline = next_deadline_us(deadline, deadline + 50, PERIOD, GAP);
        assert_eq!(deadline, 40_200 + k * 1_000);
    }
}

#[test]
fn deadlines_are_never_closer_than_the_gap() {
    // Loop bodies of every length from 0 to 3 ms, in 50 us steps.
    let mut deadline = 0;
    for body in (0..=3_000).step_by(50) {
        let now = deadline + body;
        let next = next_deadline_us(deadline, now, PERIOD, GAP);
        assert!(next >= now + GAP, "body {body}");
        assert!(next - deadline >= GAP, "body {body}");
        deadline = next;
    }
}
