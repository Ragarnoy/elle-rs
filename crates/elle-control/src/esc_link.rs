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

/// What [`EdtWatch`] asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum EdtAction {
    /// The ESC answers but has sent no extended telemetry since it was last
    /// configured: the configuration (spin direction, EDT enable) evidently did
    /// not take. Send it again.
    Reconfigure,
    /// Still no extended telemetry after every retry: stop trying and report it.
    GiveUp,
}

/// Checks that a configured ESC actually delivers extended telemetry (EDT).
///
/// An ESC answers every telemetry request with eRPM whether or not the
/// `ExtendedTelemetryEnable` command took, so a reply proves only that the ESC
/// is alive. EDT frames (temperature, voltage, ...) prove the configuration
/// sent with it arrived. Feed it once per frame that requested telemetry.
#[derive(Debug, Clone, Copy)]
pub struct EdtWatch {
    seen: bool,
    frames: u32,
    retries: u8,
    gave_up: bool,
    confirm_frames: u32,
    max_retries: u8,
}

impl EdtWatch {
    /// `confirm_frames`: answered frames without EDT before the configuration
    /// counts as lost. `max_retries`: re-sends before giving up.
    #[must_use]
    pub const fn new(confirm_frames: u32, max_retries: u8) -> Self {
        Self {
            seen: false,
            frames: 0,
            retries: 0,
            gave_up: false,
            confirm_frames,
            max_retries,
        }
    }

    /// The configuration was just (re)sent: watch for EDT afresh. Keeps the
    /// retry count, so repeated failures still end in [`EdtAction::GiveUp`].
    pub fn configured(&mut self) {
        self.seen = false;
        self.frames = 0;
    }

    /// The ESC restarted: forget everything, including retries.
    pub fn reset(&mut self) {
        *self = Self::new(self.confirm_frames, self.max_retries);
    }

    /// Whether EDT has arrived since the last configuration.
    #[must_use]
    pub const fn edt_seen(&self) -> bool {
        self.seen
    }

    /// Record one telemetry request. `edt_frame`: the reply was an EDT frame
    /// (not eRPM). Actions only fire while `stopped`, since settings commands
    /// need a stopped motor; frames still count while running.
    pub fn update(&mut self, answered: bool, edt_frame: bool, stopped: bool) -> Option<EdtAction> {
        if edt_frame {
            self.seen = true;
        }
        if self.seen || self.gave_up || !answered {
            return None;
        }
        self.frames = self.frames.saturating_add(1);
        if self.frames < self.confirm_frames || !stopped {
            return None;
        }
        self.frames = 0;
        if self.retries < self.max_retries {
            self.retries += 1;
            Some(EdtAction::Reconfigure)
        } else {
            self.gave_up = true;
            Some(EdtAction::GiveUp)
        }
    }
}
