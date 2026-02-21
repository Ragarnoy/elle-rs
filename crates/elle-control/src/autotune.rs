//! Firmware-side PID auto-tuner via relay feedback (Astrom-Hagglund method).
//!
//! Runs in the flight control loop, triggered by an RC aux channel.
//! Applies alternating setpoint bias to excite a limit cycle, measures the oscillation
//! period and amplitude, then computes PID gains using classical tuning rules.
//!
//! **no_std, no heap.** Uses fixed-size arrays and tick counts.

use elle_config::CONTROL_LOOP_DT;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Settling phase duration in ticks (2s at 77Hz)
const SETTLE_TICKS: u32 = 154;
/// Total timeout in ticks (60s at 77Hz)
const TOTAL_TIMEOUT_TICKS: u32 = 4620;
/// No-oscillation timeout in ticks (10s at 77Hz)
const NO_OSC_TIMEOUT_TICKS: u32 = 770;
/// Maximum safe amplitude in degrees
const MAX_AMPLITUDE_DEG: f32 = 20.0;
/// Number of initial cycles to discard (transient)
const DISCARD_CYCLES: usize = 2;
/// Proportional-only gain used during relay test
const TEST_KP: f32 = 1.0;
/// Maximum number of half-cycles we can record
const MAX_HALF_CYCLES: usize = 32;

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
#[derive(Debug, Clone, Copy, defmt::Format)]
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

impl SavedGains {
    /// Serialize to 32 bytes (8 × f32 little-endian).
    pub fn to_bytes(&self) -> [u8; 32] {
        let mut buf = [0u8; 32];
        buf[0..4].copy_from_slice(&self.pitch_kp.to_le_bytes());
        buf[4..8].copy_from_slice(&self.pitch_ki.to_le_bytes());
        buf[8..12].copy_from_slice(&self.pitch_kd.to_le_bytes());
        buf[12..16].copy_from_slice(&self.roll_kp.to_le_bytes());
        buf[16..20].copy_from_slice(&self.roll_ki.to_le_bytes());
        buf[20..24].copy_from_slice(&self.roll_kd.to_le_bytes());
        buf[24..28].copy_from_slice(&self.scale.to_le_bytes());
        buf[28..32].copy_from_slice(&self.i_limit.to_le_bytes());
        buf
    }

    /// Deserialize from 32 bytes (8 × f32 little-endian).
    ///
    /// Returns `None` if any value is non-finite or out of its valid range:
    /// - kp/ki/kd: 0.0..=100.0
    /// - scale: 0.0..=1.0
    /// - i_limit: 0.0..=1000.0
    pub fn from_bytes(b: &[u8; 32]) -> Option<Self> {
        let pitch_kp = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        let pitch_ki = f32::from_le_bytes([b[4], b[5], b[6], b[7]]);
        let pitch_kd = f32::from_le_bytes([b[8], b[9], b[10], b[11]]);
        let roll_kp = f32::from_le_bytes([b[12], b[13], b[14], b[15]]);
        let roll_ki = f32::from_le_bytes([b[16], b[17], b[18], b[19]]);
        let roll_kd = f32::from_le_bytes([b[20], b[21], b[22], b[23]]);
        let scale = f32::from_le_bytes([b[24], b[25], b[26], b[27]]);
        let i_limit = f32::from_le_bytes([b[28], b[29], b[30], b[31]]);

        // Validate all gains are finite and within reasonable ranges
        let gain_ok = |v: f32| v.is_finite() && (0.0..=100.0).contains(&v);
        if !gain_ok(pitch_kp)
            || !gain_ok(pitch_ki)
            || !gain_ok(pitch_kd)
            || !gain_ok(roll_kp)
            || !gain_ok(roll_ki)
            || !gain_ok(roll_kd)
        {
            return None;
        }
        if !scale.is_finite() || !(0.0..=1.0).contains(&scale) {
            return None;
        }
        if !i_limit.is_finite() || !(0.0..=1000.0).contains(&i_limit) {
            return None;
        }

        Some(Self {
            pitch_kp,
            pitch_ki,
            pitch_kd,
            roll_kp,
            roll_ki,
            roll_kd,
            scale,
            i_limit,
        })
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
pub enum AutotuneError {
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
pub struct OscillationDetector {
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
pub struct OscillationResult {
    pub period_s: f32,
    pub amplitude_deg: f32,
    pub cycles_completed: usize,
}

impl Default for OscillationDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl OscillationDetector {
    pub const fn new() -> Self {
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
    pub fn feed(&mut self, value: f32, tick: u32) -> usize {
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
    pub const fn measurable_cycles(&self) -> usize {
        self.full_cycle_count.saturating_sub(DISCARD_CYCLES)
    }

    /// Whether at least one zero crossing has been detected.
    pub const fn has_crossings(&self) -> bool {
        self.half_cycle_count > 0
    }

    /// Compute oscillation result from collected half-cycles.
    pub fn compute_result(&self) -> Result<OscillationResult, AutotuneError> {
        let min_half_cycles = 2 * (DISCARD_CYCLES + 1);
        if self.half_cycle_count < min_half_cycles {
            return Err(AutotuneError::InsufficientData);
        }

        let skip = 2 * DISCARD_CYCLES;
        let usable_count = self.half_cycle_count - skip;
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
            let idx = skip + i * 2;
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
    pub fn compute(self, ku: f32, tu: f32) -> (f32, f32, f32) {
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
    pub fn new() -> Self {
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
    /// `num_cycles` — number of measurable cycles to collect (typically 6).
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
        self.num_cycles = num_cycles;
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
    /// `measurement_deg` — current attitude on the tuned axis in degrees.
    /// `tick` — monotonic tick counter.
    pub fn update(&mut self, measurement_deg: f32, tick: u32) -> AutotuneAction {
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

                // Safety: amplitude check
                let abs_meas = if measurement_deg < 0.0 {
                    -measurement_deg
                } else {
                    measurement_deg
                };
                if abs_meas > MAX_AMPLITUDE_DEG {
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
                if !self.first_crossing
                    && elapsed >= SETTLE_TICKS + NO_OSC_TIMEOUT_TICKS
                {
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

        // h = scale * kp_test * d_rad (effective relay amplitude seen by plant)
        let h = self.saved_gains.scale * TEST_KP * relay_rad;

        // Ku = 4h / (pi * a_rad)
        let ku = 4.0 * h / (core::f32::consts::PI * a_rad);

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
