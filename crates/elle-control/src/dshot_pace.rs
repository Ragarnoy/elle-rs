//! Pacing for the 1 kHz DShot send loop.
//!
//! `embassy_time::Ticker` replays missed ticks back to back after a stall. For
//! DShot that is worse than useless: every replayed frame carries the same
//! latest target, and a frame pushed while the previous one is still on the wire
//! truncates it (embassy-dshot issue #8). This pacer skips missed ticks instead
//! and keeps frames at least `min_gap_us` apart.
//!
//! Times are plain microsecond counts (`Instant::as_micros`) so this stays
//! host-testable.

/// Deadline, in µs, for the next DShot frame.
///
/// On schedule this is `prev_us + period_us`. After a stall, when that deadline
/// is already too close or past, the next frame goes out `min_gap_us` from
/// `now_us` and the schedule restarts from there — missed ticks are dropped, not
/// replayed.
#[must_use]
pub const fn next_deadline_us(prev_us: u64, now_us: u64, period_us: u64, min_gap_us: u64) -> u64 {
    let scheduled = prev_us.saturating_add(period_us);
    let earliest = now_us.saturating_add(min_gap_us);
    if scheduled >= earliest {
        scheduled
    } else {
        earliest
    }
}
