//! Firmware-side PID auto-tuner via relay feedback (Astrom-Hagglund method).
//!
//! Runs in the flight control loop, triggered by an RC aux channel.
//! Applies alternating setpoint bias to excite a limit cycle, measures the oscillation
//! period and amplitude, then computes PID gains using classical tuning rules.
//!
//! **no_std, no heap.** Uses fixed-size arrays and tick counts.

use elle_config::{CONTROL_LOOP_DT, CONTROL_LOOP_FREQUENCY_HZ};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Settling phase duration in ticks (2 s)
const SETTLE_TICKS: u32 = 2 * CONTROL_LOOP_FREQUENCY_HZ;
/// Total timeout in ticks (60 s)
const TOTAL_TIMEOUT_TICKS: u32 = 60 * CONTROL_LOOP_FREQUENCY_HZ;
/// No-oscillation timeout in ticks (10 s)
const NO_OSC_TIMEOUT_TICKS: u32 = 10 * CONTROL_LOOP_FREQUENCY_HZ;
/// Attitude envelope in degrees, on both axes, for the whole run (settling
/// included): the tuned axis swings about zero and the other axis is held at
/// zero, so either one beyond this means the test has lost control.
const MAX_AMPLITUDE_DEG: f32 = 20.0;
/// Number of initial cycles to discard (transient)
const DISCARD_CYCLES: usize = 2;
/// Proportional-only gain used during relay test
const TEST_KP: f32 = 1.0;
/// Maximum number of half-cycles we can record
const MAX_HALF_CYCLES: usize = 32;
/// Half-cycles recorded during the discarded transient
const SKIP_HALF_CYCLES: usize = 2 * DISCARD_CYCLES;
/// Fewest half-cycles that leave one full measurable cycle after the transient
const MIN_HALF_CYCLES: usize = 2 * (DISCARD_CYCLES + 1);

/// Most measurable cycles a run can use: the half-cycle buffer holds this many
/// full cycles after the discarded transient. `start` clamps requests to it.
pub const AUTOTUNE_MAX_CYCLES: usize = (MAX_HALF_CYCLES - SKIP_HALF_CYCLES) / 2;

// A run must be able to record enough half-cycles to produce a result.
const _: () = assert!(MIN_HALF_CYCLES <= MAX_HALF_CYCLES);

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Which axis to auto-tune.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AutotuneAxis {
    Pitch,
    Roll,
}

/// Tuning rule for computing PID gains from ultimate gain/period.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TuningRule {
    /// Tyreus-Luyben: conservative, minimal overshoot (default)
    TyreusLuyben,
    /// Ziegler-Nichols: aggressive, ~25% overshoot
    ZieglerNichols,
    /// Some-overshoot: moderate
    SomeOvershoot,
}

/// Saved PID configuration for all axes.
#[derive(Debug, Clone, Copy, defmt::Format, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct SavedGains {
    pub pitch_kp: f32,
    pub pitch_ki: f32,
    pub pitch_kd: f32,
    pub roll_kp: f32,
    pub roll_ki: f32,
    pub roll_kd: f32,
    pub scale: f32,
    pub i_limit: f32,
}

impl From<crate::pid::PidConfig> for SavedGains {
    fn from(c: crate::pid::PidConfig) -> Self {
        Self {
            pitch_kp: c.kp_pitch,
            pitch_ki: c.ki_pitch,
            pitch_kd: c.kd_pitch,
            roll_kp: c.kp_roll,
            roll_ki: c.ki_roll,
            roll_kd: c.kd_roll,
            scale: c.scale,
            i_limit: c.i_limit,
        }
    }
}

impl SavedGains {
    pub fn to_bytes(self) -> [u8; 32] {
        bytemuck::cast(self)
    }

    /// Returns `None` if any value is non-finite or out of its valid range:
    /// - kp/ki/kd/scale: 0.0..=100.0
    /// - i_limit: 0.0..=1000.0
    pub fn from_bytes(b: &[u8; 32]) -> Option<Self> {
        let g: Self = bytemuck::pod_read_unaligned(b);
        g.is_valid().then_some(g)
    }

    /// Every gain finite and within the range the flash loader accepts. The
    /// autotuner checks its result against this too, so it can never apply or
    /// save gains that the next boot would silently throw away.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        let gain_ok = |v: f32| v.is_finite() && (0.0..=100.0).contains(&v);
        gain_ok(self.pitch_kp)
            && gain_ok(self.pitch_ki)
            && gain_ok(self.pitch_kd)
            && gain_ok(self.roll_kp)
            && gain_ok(self.roll_ki)
            && gain_ok(self.roll_kd)
            && gain_ok(self.scale)
            && self.i_limit.is_finite()
            && (0.0..=1000.0).contains(&self.i_limit)
    }
}

/// Result of a successful autotune run.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct AutotuneResult {
    pub axis: AutotuneAxis,
    pub ku: f32,
    pub tu_s: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub amplitude_deg: f32,
    pub cycles: usize,
}

/// Autotune error conditions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum AutotuneError {
    InsufficientData,
    UnstablePeriod,
    InvalidParams,
}

/// Actions the autotuner requests from the flight loop.
#[derive(Debug)]
pub enum AutotuneAction {
    /// No action needed this tick.
    None,
    /// Override the attitude setpoint.
    SetpointOverride { pitch_deg: f32, roll_deg: f32 },
    /// Apply new PID gains (test gains at start, or computed gains on success).
    ApplyGains(SavedGains),
    /// Restore original gains (on abort or safety trip).
    RestoreGains(SavedGains),
    /// Autotune completed successfully.
    Completed(AutotuneResult),
    /// The measurement finished but the result failed [`validate_result`]:
    /// restore these (original) gains.
    Rejected {
        reason: AutotuneReject,
        gains: SavedGains,
    },
}

/// Why a finished autotune measurement was not applied.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AutotuneReject {
    /// Oscillation too small to measure: a noise-level amplitude divides into a
    /// huge ultimate gain.
    AmplitudeTooSmall,
    /// Oscillation period outside the plausible airframe range.
    PeriodOutOfRange,
    /// Computed gains are non-finite or outside what the flash loader accepts.
    GainsOutOfRange,
}

/// Smallest usable oscillation amplitude, in degrees, whatever the relay size.
pub const AUTOTUNE_MIN_AMPLITUDE_DEG: f32 = 0.5;
/// Smallest usable amplitude as a fraction of the relay amplitude. With the
/// setpoint relay, the airframe oscillates at roughly the relay's size; a much
/// smaller swing is noise, and Ku ~ relay / amplitude blows up.
pub const AUTOTUNE_MIN_AMPLITUDE_RELAY_FRACTION: f32 = 0.2;
/// Plausible oscillation period range for these airframes, in seconds.
pub const AUTOTUNE_PERIOD_RANGE_S: core::ops::RangeInclusive<f32> = 0.1..=5.0;

/// Check a finished relay measurement and the gains it produced before they
/// are applied in flight or saved.
///
/// # Errors
///
/// The first check that fails, in the order amplitude, period, gains.
pub fn validate_result(
    amplitude_deg: f32,
    relay_deg: f32,
    tu_s: f32,
    gains: &SavedGains,
) -> Result<(), AutotuneReject> {
    let min_amplitude =
        AUTOTUNE_MIN_AMPLITUDE_DEG.max(AUTOTUNE_MIN_AMPLITUDE_RELAY_FRACTION * relay_deg);
    if !amplitude_deg.is_finite() || amplitude_deg < min_amplitude {
        return Err(AutotuneReject::AmplitudeTooSmall);
    }
    if !AUTOTUNE_PERIOD_RANGE_S.contains(&tu_s) {
        return Err(AutotuneReject::PeriodOutOfRange);
    }
    if !gains.is_valid() {
        return Err(AutotuneReject::GainsOutOfRange);
    }
    Ok(())
}

/// Why an active run had to stop before its next update.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AutotuneStop {
    /// Disarmed: RPC/gesture disarm, kill switch or failsafe.
    Disarmed,
    /// Kill switch engaged.
    Killed,
    /// The attitude controller is off: Manual mode, failsafe, or Core 1 unhealthy.
    NotStabilized,
    /// No valid attitude this tick.
    AttitudeLost,
}

/// Whether an active run still owns the aircraft. The relay test only means
/// anything while the stabilized controller is flying the test gains; any other
/// motion would be measured as the test response.
///
/// # Errors
///
/// The first condition that fails, in the order killed, disarmed, not
/// stabilized, attitude lost.
pub const fn check_run_conditions(
    armed: bool,
    attitude_enabled: bool,
    killed: bool,
    has_attitude: bool,
) -> Result<(), AutotuneStop> {
    if killed {
        Err(AutotuneStop::Killed)
    } else if !armed {
        Err(AutotuneStop::Disarmed)
    } else if !attitude_enabled {
        Err(AutotuneStop::NotStabilized)
    } else if !has_attitude {
        Err(AutotuneStop::AttitudeLost)
    } else {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// OscillationDetector
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct HalfCycle {
    peak: f32,
    duration_ticks: u32,
}

/// Detects and measures oscillation parameters from a relay feedback signal.
///
/// Tracks zero crossings, half-cycle peaks and durations, then computes
/// the average period and amplitude after discarding initial transient cycles.
struct OscillationDetector {
    half_cycles: [HalfCycle; MAX_HALF_CYCLES],
    half_cycle_count: usize,
    last_sign: Option<bool>,
    last_crossing_tick: u32,
    current_peak: f32,
    half_cycle_parity: usize,
    full_cycle_count: usize,
}

/// Result from oscillation analysis.
#[derive(Debug, Clone, Copy)]
struct OscillationResult {
    period_s: f32,
    amplitude_deg: f32,
    cycles_completed: usize,
}

impl OscillationDetector {
    const fn new() -> Self {
        Self {
            half_cycles: [HalfCycle {
                peak: 0.0,
                duration_ticks: 0,
            }; MAX_HALF_CYCLES],
            half_cycle_count: 0,
            last_sign: None,
            last_crossing_tick: 0,
            current_peak: 0.0,
            half_cycle_parity: 0,
            full_cycle_count: 0,
        }
    }

    /// Feed a measurement sample. Returns the number of full cycles completed so far.
    fn feed(&mut self, value: f32, tick: u32) -> usize {
        let sign = value >= 0.0;
        let abs_val = if value < 0.0 { -value } else { value };

        if abs_val > self.current_peak {
            self.current_peak = abs_val;
        }

        match self.last_sign {
            None => {
                self.last_sign = Some(sign);
                self.last_crossing_tick = tick;
            }
            Some(prev_sign) if prev_sign != sign => {
                // Zero crossing detected
                let duration_ticks = tick.wrapping_sub(self.last_crossing_tick);
                if self.half_cycle_count < MAX_HALF_CYCLES {
                    self.half_cycles[self.half_cycle_count] = HalfCycle {
                        peak: self.current_peak,
                        duration_ticks,
                    };
                    self.half_cycle_count += 1;
                }
                self.half_cycle_parity += 1;
                if self.half_cycle_parity & 1 == 0 {
                    self.full_cycle_count += 1;
                }
                self.last_crossing_tick = tick;
                self.current_peak = abs_val;
                self.last_sign = Some(sign);
            }
            _ => {}
        }

        self.full_cycle_count
    }

    /// Number of measurable full cycles (after discarding transient).
    const fn measurable_cycles(&self) -> usize {
        self.full_cycle_count.saturating_sub(DISCARD_CYCLES)
    }

    /// Whether at least one zero crossing has been detected.
    const fn has_crossings(&self) -> bool {
        self.half_cycle_count > 0
    }

    /// Compute oscillation result from collected half-cycles.
    fn compute_result(&self) -> Result<OscillationResult, AutotuneError> {
        if self.half_cycle_count < MIN_HALF_CYCLES {
            return Err(AutotuneError::InsufficientData);
        }

        let usable_count = self.half_cycle_count - SKIP_HALF_CYCLES;
        if usable_count < 2 {
            return Err(AutotuneError::InsufficientData);
        }

        // Pair consecutive half-cycles into full cycles
        let num_pairs = usable_count / 2;
        if num_pairs == 0 {
            return Err(AutotuneError::InsufficientData);
        }

        let mut sum_period_ticks: u32 = 0;
        let mut sum_amplitude: f32 = 0.0;
        let mut sum_period_sq: f64 = 0.0; // Use f64 for variance to avoid overflow

        // First pass: compute means
        for i in 0..num_pairs {
            let idx = SKIP_HALF_CYCLES + i * 2;
            let h0 = &self.half_cycles[idx];
            let h1 = &self.half_cycles[idx + 1];
            let full_period = h0.duration_ticks + h1.duration_ticks;
            let avg_amp = (h0.peak + h1.peak) / 2.0;
            sum_period_ticks += full_period;
            sum_amplitude += avg_amp;
            sum_period_sq += (full_period as f64) * (full_period as f64);
        }

        let n = num_pairs as f32;
        let mean_period_ticks = sum_period_ticks as f32 / n;
        let mean_amplitude = sum_amplitude / n;

        // CV check on period: variance > 0.09 * mean² means CV > 0.3 (30%)
        if num_pairs >= 2 {
            let mean_sq = (mean_period_ticks as f64) * (mean_period_ticks as f64);
            let variance = sum_period_sq / (num_pairs as f64) - mean_sq;
            // variance > 0.09 * mean² → CV > 30%
            if variance > 0.09 * mean_sq {
                return Err(AutotuneError::UnstablePeriod);
            }
        }

        let mean_period_s = mean_period_ticks * CONTROL_LOOP_DT;

        if mean_period_s <= 0.0 || mean_amplitude <= 0.0 {
            return Err(AutotuneError::InvalidParams);
        }

        Ok(OscillationResult {
            period_s: mean_period_s,
            amplitude_deg: mean_amplitude,
            cycles_completed: num_pairs,
        })
    }
}

// ---------------------------------------------------------------------------
// Tuning Rules
// ---------------------------------------------------------------------------

impl TuningRule {
    /// Compute PID gains from ultimate gain and period.
    /// Returns (kp, ki, kd) — these are effective gains (multiply by scale for actuator output).
    const fn compute(self, ku: f32, tu: f32) -> (f32, f32, f32) {
        match self {
            Self::TyreusLuyben => {
                let kp = 0.45 * ku;
                let ki = 0.45 * ku / (2.2 * tu);
                let kd = 0.45 * ku * tu / 6.3;
                (kp, ki, kd)
            }
            Self::ZieglerNichols => {
                let kp = 0.6 * ku;
                let ki = 1.2 * ku / tu;
                let kd = 0.075 * ku * tu;
                (kp, ki, kd)
            }
            Self::SomeOvershoot => {
                let kp = 0.33 * ku;
                let ki = 0.66 * ku / tu;
                let kd = 0.11 * ku * tu;
                (kp, ki, kd)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Autotuner State Machine
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Phase {
    Idle,
    Settling,
    Relay,
    Complete,
    Aborted,
}

/// Firmware-side autotuner state machine.
///
/// Call `start()` to begin, then `update()` every control loop tick.
/// The returned `AutotuneAction` tells the flight loop what to do.
pub struct Autotuner {
    phase: Phase,
    axis: AutotuneAxis,
    rule: TuningRule,
    relay_deg: f32,
    num_cycles: usize,
    saved_gains: SavedGains,
    start_tick: u32,
    detector: OscillationDetector,
    relay_positive: bool,
    first_crossing: bool,
    result: Option<AutotuneResult>,
}

impl Default for Autotuner {
    fn default() -> Self {
        Self::new()
    }
}

impl Autotuner {
    pub const fn new() -> Self {
        Self {
            phase: Phase::Idle,
            axis: AutotuneAxis::Pitch,
            rule: TuningRule::TyreusLuyben,
            relay_deg: 5.0,
            num_cycles: 6,
            saved_gains: SavedGains {
                pitch_kp: 0.0,
                pitch_ki: 0.0,
                pitch_kd: 0.0,
                roll_kp: 0.0,
                roll_ki: 0.0,
                roll_kd: 0.0,
                scale: 0.0,
                i_limit: 0.0,
            },
            start_tick: 0,
            detector: OscillationDetector::new(),
            relay_positive: true,
            first_crossing: false,
            result: None,
        }
    }

    /// Start an autotune run.
    ///
    /// `current_gains` — snapshot of current PID configuration (will be restored on abort).
    /// `relay_deg` — relay amplitude in degrees (typically 5.0).
    /// `num_cycles` — number of measurable cycles to collect (typically 6),
    ///   clamped to `1..=AUTOTUNE_MAX_CYCLES`.
    /// `rule` — tuning rule for gain computation.
    /// `tick` — current monotonic tick counter.
    ///
    /// Returns the initial test gains to apply (P-only on tuned axis).
    pub fn start(
        &mut self,
        axis: AutotuneAxis,
        current_gains: SavedGains,
        relay_deg: f32,
        num_cycles: usize,
        rule: TuningRule,
        tick: u32,
    ) -> SavedGains {
        self.phase = Phase::Settling;
        self.axis = axis;
        self.rule = rule;
        self.relay_deg = relay_deg;
        self.num_cycles = num_cycles.clamp(1, AUTOTUNE_MAX_CYCLES);
        self.saved_gains = current_gains;
        self.start_tick = tick;
        self.detector = OscillationDetector::new();
        self.relay_positive = true;
        self.first_crossing = false;
        self.result = None;

        self.initial_test_gains()
    }

    /// Whether the autotuner is currently running (settling or relay phase).
    pub const fn is_active(&self) -> bool {
        matches!(self.phase, Phase::Settling | Phase::Relay)
    }

    /// Which axis is being tuned (only meaningful while active).
    pub const fn axis(&self) -> AutotuneAxis {
        self.axis
    }

    /// Measurable cycles this run collects (the request after clamping).
    pub const fn num_cycles(&self) -> usize {
        self.num_cycles
    }

    /// The tuned axis's attitude out of a pitch/roll pair, in degrees.
    pub const fn measurement(&self, pitch_deg: f32, roll_deg: f32) -> f32 {
        match self.axis {
            AutotuneAxis::Pitch => pitch_deg,
            AutotuneAxis::Roll => roll_deg,
        }
    }

    /// Current phase as u8 for ULog logging.
    pub const fn phase_u8(&self) -> u8 {
        match self.phase {
            Phase::Idle => 0,
            Phase::Settling => 1,
            Phase::Relay => 2,
            Phase::Complete => 3,
            Phase::Aborted => 4,
        }
    }

    /// Current relay direction.
    pub const fn relay_positive(&self) -> bool {
        self.relay_positive
    }

    /// Current relay setpoint in degrees (±relay_deg during relay, 0.0 during settling/idle).
    pub const fn current_setpoint_deg(&self) -> f32 {
        match self.phase {
            Phase::Relay => {
                if self.relay_positive {
                    self.relay_deg
                } else {
                    -self.relay_deg
                }
            }
            _ => 0.0,
        }
    }

    /// Current half-cycle peak amplitude (degrees), or 0.0 if no crossings yet.
    pub const fn current_amplitude_deg(&self) -> f32 {
        self.detector.current_peak
    }

    /// Number of full oscillation cycles completed.
    pub const fn cycles_completed(&self) -> u8 {
        self.detector.full_cycle_count as u8
    }

    /// Abort the current autotune run.
    /// Returns the saved gains to restore.
    pub fn abort(&mut self) -> Option<SavedGains> {
        if self.is_active() {
            self.phase = Phase::Aborted;
            Some(self.saved_gains)
        } else {
            None
        }
    }

    /// Tick the autotuner state machine.
    ///
    /// `pitch_deg`, `roll_deg` — current raw attitude in degrees. The tuner
    /// measures its own axis and holds both inside the attitude envelope.
    /// `tick` — monotonic tick counter.
    pub fn update(&mut self, pitch_deg: f32, roll_deg: f32, tick: u32) -> AutotuneAction {
        if !self.is_active() {
            return AutotuneAction::None;
        }

        // Safety: finite attitude inside the envelope on both axes, in every
        // active phase. Settling already flies the test gains.
        let in_envelope =
            |deg: f32| deg.is_finite() && (-MAX_AMPLITUDE_DEG..=MAX_AMPLITUDE_DEG).contains(&deg);
        if !in_envelope(pitch_deg) || !in_envelope(roll_deg) {
            self.phase = Phase::Aborted;
            return AutotuneAction::RestoreGains(self.saved_gains);
        }
        let measurement_deg = self.measurement(pitch_deg, roll_deg);

        match self.phase {
            Phase::Idle | Phase::Complete | Phase::Aborted => AutotuneAction::None,

            Phase::Settling => {
                let elapsed = tick.wrapping_sub(self.start_tick);
                if elapsed >= SETTLE_TICKS {
                    self.phase = Phase::Relay;
                }
                // During settling, hold zero setpoint
                AutotuneAction::SetpointOverride {
                    pitch_deg: 0.0,
                    roll_deg: 0.0,
                }
            }

            Phase::Relay => {
                let elapsed = tick.wrapping_sub(self.start_tick);

                // Total timeout check
                if elapsed >= TOTAL_TIMEOUT_TICKS {
                    self.phase = Phase::Aborted;
                    return AutotuneAction::RestoreGains(self.saved_gains);
                }

                // Feed measurement to oscillation detector
                self.detector.feed(measurement_deg, tick);

                // Track first crossing for no-oscillation timeout
                if !self.first_crossing && self.detector.has_crossings() {
                    self.first_crossing = true;
                }

                // No oscillation timeout
                if !self.first_crossing && elapsed >= SETTLE_TICKS + NO_OSC_TIMEOUT_TICKS {
                    self.phase = Phase::Aborted;
                    return AutotuneAction::RestoreGains(self.saved_gains);
                }

                // Relay logic: flip setpoint when measurement crosses zero
                let should_be_positive = measurement_deg < 0.0;
                if should_be_positive != self.relay_positive {
                    self.relay_positive = should_be_positive;
                }

                let sp = if self.relay_positive {
                    self.relay_deg
                } else {
                    -self.relay_deg
                };

                let (pitch_sp, roll_sp) = match self.axis {
                    AutotuneAxis::Pitch => (sp, 0.0),
                    AutotuneAxis::Roll => (0.0, sp),
                };

                // Check completion
                if self.detector.measurable_cycles() >= self.num_cycles {
                    return self.finish_relay(pitch_sp, roll_sp);
                }

                AutotuneAction::SetpointOverride {
                    pitch_deg: pitch_sp,
                    roll_deg: roll_sp,
                }
            }
        }
    }

    /// Analyze oscillation data and compute gains.
    fn finish_relay(&mut self, _pitch_sp: f32, _roll_sp: f32) -> AutotuneAction {
        let osc = match self.detector.compute_result() {
            Ok(r) => r,
            Err(_) => {
                self.phase = Phase::Aborted;
                return AutotuneAction::RestoreGains(self.saved_gains);
            }
        };

        let tu = osc.period_s;
        let a_deg = osc.amplitude_deg;
        let a_rad = a_deg.to_radians();
        let relay_rad = self.relay_deg.to_radians();

        // Setpoint-relay method: plant input is u = K·(r − y) with K = scale·TEST_KP,
        // i.e. a relay of amplitude h = K·d_rad in parallel with proportional feedback K.
        // Oscillation condition: (4h/(πa) + K)·G(jω) = −1, so Ku includes the K term —
        // omitting it underestimates Ku by K (≈40% when a ≈ d).
        let k_eff = self.saved_gains.scale * TEST_KP;
        let h = k_eff * relay_rad;
        let ku = 4.0 * h / (core::f32::consts::PI * a_rad) + k_eff;

        // Compute effective gains
        let (kp_eff, ki_eff, kd_eff) = self.rule.compute(ku, tu);

        // Convert from effective gains back to per-axis gains (divide by scale)
        let kp = kp_eff / self.saved_gains.scale;
        let ki = ki_eff / self.saved_gains.scale;
        let kd = kd_eff / self.saved_gains.scale;

        let result = AutotuneResult {
            axis: self.axis,
            ku,
            tu_s: tu,
            kp,
            ki,
            kd,
            amplitude_deg: a_deg,
            cycles: osc.cycles_completed,
        };

        self.result = Some(result);
        let gains = self.computed_gains();
        let check = match gains {
            Some(g) => validate_result(a_deg, self.relay_deg, tu, &g),
            None => Err(AutotuneReject::GainsOutOfRange),
        };
        if let Err(reason) = check {
            self.result = None;
            self.phase = Phase::Aborted;
            return AutotuneAction::Rejected {
                reason,
                gains: self.saved_gains,
            };
        }
        self.phase = Phase::Complete;

        AutotuneAction::Completed(result)
    }

    /// Build P-only test gains: TEST_KP on tuned axis, originals on other axis.
    const fn initial_test_gains(&self) -> SavedGains {
        match self.axis {
            AutotuneAxis::Pitch => SavedGains {
                pitch_kp: TEST_KP,
                pitch_ki: 0.0,
                pitch_kd: 0.0,
                roll_kp: self.saved_gains.roll_kp,
                roll_ki: self.saved_gains.roll_ki,
                roll_kd: self.saved_gains.roll_kd,
                scale: self.saved_gains.scale,
                i_limit: self.saved_gains.i_limit,
            },
            AutotuneAxis::Roll => SavedGains {
                pitch_kp: self.saved_gains.pitch_kp,
                pitch_ki: self.saved_gains.pitch_ki,
                pitch_kd: self.saved_gains.pitch_kd,
                roll_kp: TEST_KP,
                roll_ki: 0.0,
                roll_kd: 0.0,
                scale: self.saved_gains.scale,
                i_limit: self.saved_gains.i_limit,
            },
        }
    }

    /// Build computed gains: tuned axis gets new gains, other axis keeps originals.
    pub fn computed_gains(&self) -> Option<SavedGains> {
        let r = self.result?;
        Some(match self.axis {
            AutotuneAxis::Pitch => SavedGains {
                pitch_kp: r.kp,
                pitch_ki: r.ki,
                pitch_kd: r.kd,
                roll_kp: self.saved_gains.roll_kp,
                roll_ki: self.saved_gains.roll_ki,
                roll_kd: self.saved_gains.roll_kd,
                scale: self.saved_gains.scale,
                i_limit: self.saved_gains.i_limit,
            },
            AutotuneAxis::Roll => SavedGains {
                pitch_kp: self.saved_gains.pitch_kp,
                pitch_ki: self.saved_gains.pitch_ki,
                pitch_kd: self.saved_gains.pitch_kd,
                roll_kp: r.kp,
                roll_ki: r.ki,
                roll_kd: r.kd,
                scale: self.saved_gains.scale,
                i_limit: self.saved_gains.i_limit,
            },
        })
    }
}
