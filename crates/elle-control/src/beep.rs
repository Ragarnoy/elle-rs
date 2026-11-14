//! Arm and disarm beeps through the ESCs, never ahead of a stop.
//!
//! A beep is a burst of DShot beep commands. Each one takes the frame slot that
//! would otherwise carry the throttle or `MotorStop`, so the old DShot task, which
//! played the whole pattern (with its sleeps) before reading the new target,
//! kept the engines at their last throttle for ~300 ms after every disarm,
//! failsafe and kill switch included (bench LOG_0094).
//!
//! [`BeepSequencer`] runs one frame slot at a time inside the 1 kHz send loop,
//! after the loop has taken the new target:
//! - a beep only starts with both targets at zero and both engines at rest;
//! - it is cancelled, and any pending request dropped, as soon as a target is
//!   non-zero, so a beep never delays throttle either;
//! - a request that cannot start within [`BEEP_MAX_WAIT_MS`] is dropped.
//!
//! Between the bursts of a pattern the loop sends its normal (stop) frames.

use elle_config::{BEEP_GAP_MS, BEEP_MAX_WAIT_MS};

/// Which confirmation to play.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pattern {
    /// One burst of `Beep1`.
    Arm,
    /// Two bursts of `Beep2`, [`BEEP_GAP_MS`] apart.
    Disarm,
}

/// The DShot beep command to send in this frame slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tone {
    Beep1,
    Beep2,
}

/// Command frames per burst (the DShot convention the old task used: 10 × ~1 ms).
pub const BURST_FRAMES: u8 = 10;

impl Pattern {
    const fn tone(self) -> Tone {
        match self {
            Self::Arm => Tone::Beep1,
            Self::Disarm => Tone::Beep2,
        }
    }

    const fn bursts(self) -> u8 {
        match self {
            Self::Arm => 1,
            Self::Disarm => 2,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Playing {
    pattern: Pattern,
    burst: u8,
    sent: u8,
    /// No command before this time (the gap between bursts), ms.
    resume_ms: u64,
}

/// Plays beep patterns one frame slot at a time. See the module docs.
#[derive(Clone, Copy, Debug, Default)]
pub struct BeepSequencer {
    pending: Option<(Pattern, u64)>,
    playing: Option<Playing>,
}

impl BeepSequencer {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            pending: None,
            playing: None,
        }
    }

    /// Queue `pattern` (requested at `now_ms`). It replaces any pending or
    /// playing beep.
    pub fn request(&mut self, pattern: Pattern, now_ms: u64) {
        self.pending = Some((pattern, now_ms));
        self.playing = None;
    }

    /// Whether a beep is queued or playing.
    #[must_use]
    pub const fn busy(&self) -> bool {
        self.pending.is_some() || self.playing.is_some()
    }

    /// One frame slot. `spin_commanded`: either engine's target is non-zero
    /// (already taken this iteration); `at_rest`: both engines are stopped.
    /// `Some(tone)`: send that command instead of the throttle frame. `None`:
    /// send the normal frame.
    pub fn next(&mut self, now_ms: u64, spin_commanded: bool, at_rest: bool) -> Option<Tone> {
        if spin_commanded {
            self.pending = None;
            self.playing = None;
            return None;
        }
        if self.playing.is_none() {
            let (pattern, at) = self.pending?;
            if now_ms.saturating_sub(at) > BEEP_MAX_WAIT_MS {
                self.pending = None;
                return None;
            }
            if !at_rest {
                return None;
            }
            self.pending = None;
            self.playing = Some(Playing {
                pattern,
                burst: 0,
                sent: 0,
                resume_ms: now_ms,
            });
        }
        let p = self.playing.as_mut()?;
        if now_ms < p.resume_ms {
            return None;
        }
        let tone = p.pattern.tone();
        p.sent += 1;
        if p.sent == BURST_FRAMES {
            p.burst += 1;
            p.sent = 0;
            if p.burst == p.pattern.bursts() {
                self.playing = None;
            } else {
                p.resume_ms = now_ms + BEEP_GAP_MS;
            }
        }
        Some(tone)
    }
}

/// Whether one engine is at rest for beeping: below [`elle_config::BEEP_AT_REST_ERPM`]
/// on valid telemetry, else (no telemetry) once its target has been zero for
/// [`elle_config::BEEP_SPINDOWN_FALLBACK_MS`]. `zero_for_ms`: how long the
/// target has been zero, `None` while it is non-zero.
#[must_use]
pub const fn engine_at_rest(telemetry: Option<u32>, zero_for_ms: Option<u64>) -> bool {
    match (zero_for_ms, telemetry) {
        (None, _) => false,
        (Some(_), Some(erpm)) => erpm < elle_config::BEEP_AT_REST_ERPM,
        (Some(ms), None) => ms >= elle_config::BEEP_SPINDOWN_FALLBACK_MS,
    }
}
