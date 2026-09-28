//! ESC reply tracking over bidirectional DShot.
//!
//! A bidirectional ESC answers every frame that requests telemetry. Counting
//! consecutive frames without an answer tells the DShot task when an ESC went
//! silent (lost power, restarted) and when it came back — at which point it has
//! forgotten the settings sent at boot (spin direction, extended telemetry) and
//! needs them again. Counting frames rather than time keeps an executor stall
//! (no frames sent) from looking like a silent ESC.

/// A change in an ESC's link state, reported once per transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum EscLinkEvent {
    /// Answered after being silent, or for the first time since boot without
    /// having answered the boot configuration: it needs configuring.
    Appeared,
    /// Stopped answering for `silent_frames` consecutive requests.
    Lost,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// No answer since boot.
    NeverSeen,
    Up,
    Lost,
}

/// Per-ESC link tracker. Feed it once per frame that requested telemetry.
#[derive(Debug, Clone, Copy)]
pub struct EscLink {
    state: State,
    missed: u32,
    silent_frames: u32,
}

impl EscLink {
    /// `silent_frames`: consecutive unanswered requests before `Lost`.
    #[must_use]
    pub const fn new(silent_frames: u32) -> Self {
        Self {
            state: State::NeverSeen,
            missed: 0,
            silent_frames,
        }
    }

    /// Mark the ESC as configured and answering, without an event: used when
    /// it answered right after the boot configuration was sent.
    pub fn mark_up(&mut self) {
        self.state = State::Up;
        self.missed = 0;
    }

    /// Whether the ESC is currently answering.
    #[must_use]
    pub const fn is_up(&self) -> bool {
        matches!(self.state, State::Up)
    }

    /// Record one telemetry request: `answered` if any reply came back,
    /// corrupt or not (a corrupt reply still proves the ESC is alive).
    pub fn update(&mut self, answered: bool) -> Option<EscLinkEvent> {
        if answered {
            self.missed = 0;
            return match self.state {
                State::Up => None,
                State::NeverSeen | State::Lost => {
                    self.state = State::Up;
                    Some(EscLinkEvent::Appeared)
                }
            };
        }
        self.missed = self.missed.saturating_add(1);
        if matches!(self.state, State::Up) && self.missed >= self.silent_frames {
            self.state = State::Lost;
            return Some(EscLinkEvent::Lost);
        }
        None
    }
}
