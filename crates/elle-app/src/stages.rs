//! Per-stage timing of the flight loop tick, logged to ULog as `loop_stages`.
//!
//! `loop_time_us` only gives the tick total; this splits it into the stages
//! below so a change in the total can be traced to one of them. Each stage is
//! wall time, so it includes whatever preempted it (interrupts, the DShot
//! executor); DShot executor time is reported next to it for that reason.

use embassy_time::Instant;

/// The stages of one flight-loop tick, in order.
#[derive(Clone, Copy)]
pub(crate) enum Stage {
    /// Supervisor check, attitude and RC intake, kill switch, calibration polls.
    Intake,
    /// Attitude validation and `FlightController::update`.
    Update,
    /// Arm/disarm beeps, engine output, CRSF flight-mode handoff.
    Outputs,
    /// Autotune and heading-hold switch handling.
    Switches,
    /// Autotune guard and step, deferred PID save.
    Autotune,
    /// ULog start and per-tick recording.
    Log,
    /// Failsafe check, perf summary, LED.
    Tail,
}

pub(crate) const STAGES: usize = 7;

/// Lap timer across one tick.
pub(crate) struct StageClock {
    last: Instant,
}

impl StageClock {
    pub(crate) const fn start(now: Instant) -> Self {
        Self { last: now }
    }

    /// µs since the previous lap (or the tick start).
    pub(crate) fn lap(&mut self) -> u32 {
        let now = Instant::now();
        let us = now.duration_since(self.last).as_micros() as u32;
        self.last = now;
        us
    }
}

/// Sum and max per stage over a window of ticks.
pub(crate) struct StageWindow {
    ticks: u32,
    sum: [u32; STAGES],
    max: [u32; STAGES],
}

/// One window of [`StageWindow`]: mean and max µs per stage.
pub(crate) struct StageStats {
    pub ticks: u32,
    pub avg: [u32; STAGES],
    pub max: [u32; STAGES],
}

impl StageWindow {
    pub(crate) const fn new() -> Self {
        Self {
            ticks: 0,
            sum: [0; STAGES],
            max: [0; STAGES],
        }
    }

    /// A stage that didn't run this tick simply records nothing (counts as 0).
    pub(crate) fn record(&mut self, stage: Stage, us: u32) {
        let i = stage as usize;
        self.sum[i] = self.sum[i].saturating_add(us);
        self.max[i] = self.max[i].max(us);
    }

    pub(crate) fn end_tick(&mut self) {
        self.ticks += 1;
    }

    pub(crate) const fn ticks(&self) -> u32 {
        self.ticks
    }

    /// Read the window and start a new one.
    pub(crate) fn take(&mut self) -> StageStats {
        let ticks = self.ticks.max(1);
        let stats = StageStats {
            ticks: self.ticks,
            avg: self.sum.map(|s| s / ticks),
            max: self.max,
        };
        *self = Self::new();
        stats
    }
}
