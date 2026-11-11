//! The flight control law: pilot commands and attitude in, elevon and engine
//! commands out. Proposal 0007.
//!
//! This is the body of what used to be `FlightController::update()`, moved out
//! of `elle-system` so it builds and runs on the host. It reads no clock and
//! touches no hardware: time arrives as [`LawInputs::now`], the elevon write is
//! returned as an [`ElevonCommand`] for the caller to make through an
//! [`ElevonOutput`], and events come back as [`LawEvents`] for the caller to
//! emit. `FlightController` keeps the PWM, the watchdog, the supervisor, the RC
//! link state machine and the events.

use crate::commands::{AttitudeMode, NormalizedCommands, PilotCommands};
use crate::filter::smooth_setpoint;
use crate::mixing::elevons::{ControlInputs, MixSaturation, mix_elevons, mix_elevons_direct_lut};
use crate::mixing::yaw::{normalized_yaw_to_rc, throttle_with_differential_lut};
use crate::{
    arming::ArmingState,
    heading::HeadingController,
    pid::{AttitudeController, AxisTerms, PidConfig},
    yaw_damper::YawDamper,
};
use elle_config::lut::apply_differential_thrust_lut;
use elle_config::*;
use embassy_time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ControlMode {
    Manual,       // Full manual control (~306)
    Stabilized,   // Stick = attitude setpoint, 100% PID (~1000)
    AltitudeHold, // Level hold, manual throttle (~1694)
}

impl From<AttitudeMode> for ControlMode {
    fn from(mode: AttitudeMode) -> Self {
        match mode {
            AttitudeMode::Manual => Self::Manual,
            AttitudeMode::Stabilized => Self::Stabilized,
            AttitudeMode::AltitudeHold => Self::AltitudeHold,
        }
    }
}

/// AHRS attitude and filtered rates, as published by Core 1
/// (`elle_hardware::imu::AttitudeData` is this type).
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Attitude {
    pub pitch: f32,      // radians
    pub roll: f32,       // radians
    pub yaw: f32,        // radians
    pub pitch_rate: f32, // rad/s
    pub roll_rate: f32,  // rad/s
    pub yaw_rate: f32,   // rad/s
    pub timestamp: Instant,
}

impl Attitude {
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            pitch_rate: 0.0,
            roll_rate: 0.0,
            yaw_rate: 0.0,
            timestamp: Instant::from_ticks(0),
        }
    }
}

/// A real sample (not the zero placeholder) younger than `max_age` at `now`.
/// `elle_hardware::imu::is_attitude_valid` with the clock passed in.
#[must_use]
pub fn is_attitude_valid_at(attitude: &Attitude, max_age: Duration, now: Instant) -> bool {
    attitude.timestamp != Instant::from_ticks(0) && now - attitude.timestamp < max_age
}

/// Snapshot of controller outputs for observability
#[derive(Debug, Clone, Copy, Default)]
pub struct ControllerOutputSnapshot {
    pub pitch_correction: f32,
    pub roll_correction: f32,
    pub pitch_setpoint_deg: f32,
    pub roll_setpoint_deg: f32,
    pub elevon_left_us: u32,
    pub elevon_right_us: u32,
    pub engine_left_dshot: u16,
    pub engine_right_dshot: u16,
    pub heading_hold_active: bool,
    pub heading_target_deg: f32,
    pub heading_error_deg: f32,
    /// Measured time since the previous `update`, in µs (0 on the first).
    pub dt_us: u32,
    /// Age of the attitude sample the PID used, in µs; `u32::MAX` when none.
    pub att_age_us: u32,
    /// Setpoints the PID actually used (after smoothing and rate limit).
    pub pitch_sp_used_deg: f32,
    pub roll_sp_used_deg: f32,
    /// Scaled P/I/D contributions; each axis's terms sum to its correction.
    pub pitch_terms: AxisTerms,
    pub roll_terms: AxisTerms,
    /// Mixer saturation from this tick.
    pub saturation: MixSaturation,
    /// Pulses the PWM actually output (after trim and right-servo inversion).
    pub elevon_left_pulse_us: u32,
    pub elevon_right_pulse_us: u32,
    /// Yaw damper command added to the differential thrust (normalized yaw,
    /// positive slows the left engine); 0 when the damper is not running.
    pub yaw_damp: f32,
    /// Elevon PWM counter when this tick's pulses were written, µs into the
    /// frame; they latch at the next wrap (`REFRESH_INTERVAL_US - counter` later).
    pub pwm_counter_us: u16,
}

/// One tick's inputs.
pub struct LawInputs<'a> {
    /// The tick's time: attitude age and hold expiry are measured against it.
    pub now: Instant,
    pub commands: &'a PilotCommands,
    /// A new attitude sample this tick, if one arrived.
    pub attitude: Option<&'a Attitude>,
    /// The supervisor's Core 1 verdict: the attitude controller stays off while false.
    pub core1_healthy: bool,
}

/// The elevon write the tick asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElevonCommand {
    /// Mixed pulses, before trim and the right servo's inversion.
    Trimmed { left_us: u32, right_us: u32 },
    /// Both elevons centred (failsafe).
    Safe,
}

/// What the caller reports, in order, after the tick's other work.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LawEvents {
    /// The mode changed this tick (from, to).
    pub mode_change: Option<(ControlMode, ControlMode)>,
    /// `EVT_ATTITUDE_HELD`: second tick on a held sample, its age in ms.
    pub attitude_held_ms: Option<u64>,
    /// `EVT_ATTITUDE_STALE`: the held sample expired, its age in ms.
    pub attitude_stale_ms: Option<u64>,
}

/// One tick's outputs.
#[derive(Debug, Clone, Copy)]
pub struct LawOutputs {
    pub elevons: ElevonCommand,
    /// Everything but the PWM write's results (`elevon_*_pulse_us`,
    /// `pwm_counter_us`) and the measured `dt_us`; [`LawOutputs::write`] fills those.
    pub snapshot: ControllerOutputSnapshot,
    pub events: LawEvents,
}

/// The elevon PWM, as the law's caller drives it.
pub trait ElevonOutput {
    /// Write trimmed pulses; returns the pulses output (after trim and the
    /// right servo's inversion) and the PWM counter at the write.
    fn set_elevons_with_trim(&mut self, left_us: u32, right_us: u32) -> (u32, u32, u16);
    /// Centre both elevons; returns the PWM counter at the write.
    fn set_safe_positions(&mut self) -> u16;
}

impl LawOutputs {
    /// Make the tick's elevon write and complete the snapshot with its results
    /// and the measured `dt_us`.
    pub fn write<O: ElevonOutput>(&self, out: &mut O, dt_us: u32) -> ControllerOutputSnapshot {
        let (left_pulse, right_pulse, pwm_counter_us) = match self.elevons {
            ElevonCommand::Trimmed { left_us, right_us } => {
                out.set_elevons_with_trim(left_us, right_us)
            }
            ElevonCommand::Safe => (
                ELEVON_LEFT_CENTER_US,
                ELEVON_RIGHT_CENTER_US,
                out.set_safe_positions(),
            ),
        };
        ControllerOutputSnapshot {
            elevon_left_pulse_us: left_pulse,
            elevon_right_pulse_us: right_pulse,
            pwm_counter_us,
            dt_us,
            ..self.snapshot
        }
    }
}

/// Control law state: everything a tick reads and writes.
pub struct FlightLaw {
    arming: ArmingState,
    attitude_controller: AttitudeController,
    /// Yaw rate → differential thrust, alongside the attitude PID.
    yaw_damper: YawDamper,
    last_attitude: Option<Attitude>,
    /// Consecutive ticks flown on `last_attitude` because no new sample came.
    held_ticks: u32,
    /// The held sample expired (EVT_ATTITUDE_STALE sent); cleared by a new sample.
    attitude_dropped: bool,
    // Smoothed setpoints for attitude hold (in radians for consistency)
    filtered_pitch_setpoint_rad: f32,
    filtered_roll_setpoint_rad: f32,
    /// Previous tick's mixer saturation, for the PID's anti-windup.
    last_saturation: MixSaturation,
    /// Bumped on every gain change, so the logger can record new gains.
    gains_version: u32,
    current_control_mode: ControlMode,
    /// The previous tick's snapshot: fields a path doesn't set carry over.
    last_output: ControllerOutputSnapshot,
    // Setpoint override for autotune relay feedback
    setpoint_override: Option<(f32, f32)>,
    // Heading-hold outer loop: modifier active only while AttitudeMode::Stabilized
    heading_controller: HeadingController,
    heading_hold_active: bool,
    locked_heading_rad: Option<f32>,
    // When true, arming is only via explicit arm()/disarm() — skips throttle-low auto-arm
    explicit_arming_only: bool,
}

impl Default for FlightLaw {
    fn default() -> Self {
        Self::new()
    }
}

impl FlightLaw {
    #[must_use]
    pub fn new() -> Self {
        let config = PidConfig {
            kp_pitch: PITCH_KP,
            ki_pitch: PITCH_KI,
            kd_pitch: PITCH_KD,
            kp_roll: ROLL_KP,
            ki_roll: ROLL_KI,
            kd_roll: ROLL_KD,
            i_limit: PID_I_LIMIT,
            scale: PID_SCALE,
        };

        let mut attitude_controller = AttitudeController::with_config(config);
        attitude_controller.pitch_hold_enabled = true;
        attitude_controller.roll_hold_enabled = true;

        Self {
            arming: ArmingState::default(),
            attitude_controller,
            yaw_damper: YawDamper::from_config(),
            last_attitude: None,
            held_ticks: 0,
            attitude_dropped: false,
            filtered_pitch_setpoint_rad: 0.0,
            filtered_roll_setpoint_rad: 0.0,
            last_saturation: MixSaturation::default(),
            gains_version: 0,
            current_control_mode: ControlMode::Manual,
            last_output: ControllerOutputSnapshot::default(),
            setpoint_override: None,
            heading_controller: HeadingController::new(
                HEADING_HOLD_KP,
                HEADING_HOLD_KI,
                HEADING_HOLD_I_LIMIT_DEG,
                HEADING_HOLD_MAX_ROLL_DEG,
                HEADING_HOLD_MAX_ROLL_RATE_DEG_S,
            ),
            heading_hold_active: false,
            locked_heading_rad: None,
            explicit_arming_only: false,
        }
    }

    /// One control tick. `dt` is the period the law integrates over (PID and
    /// heading-hold integrals, the setpoint and heading slew limits); the
    /// firmware passes `CONTROL_LOOP_DT`. The setpoint EMA weight
    /// (`SETPOINT_FILTER_ALPHA`) and the yaw damper's washout are fixed at the
    /// configured tick.
    pub fn step(&mut self, inputs: &LawInputs, dt: f32) -> LawOutputs {
        let commands = inputs.commands;
        let mode = commands.attitude_mode();
        let mut events = LawEvents::default();

        // Track mode changes
        let current_mode = ControlMode::from(mode);

        if current_mode != self.current_control_mode {
            events.mode_change = Some((self.current_control_mode, current_mode));
            // Leaving Manual: the PID never ran there (the RC fast path skips it),
            // so its integrators still hold whatever they had when Manual was
            // entered. Clear them here, on the real transition — this is the only
            // place that still knows the previous mode.
            if self.current_control_mode == ControlMode::Manual {
                self.attitude_controller.reset();
                self.yaw_damper.reset();
                self.last_saturation = MixSaturation::default();
            }
            self.current_control_mode = current_mode;
        }

        // Link lost: hold the failsafe outputs on *every* tick. The caller keeps
        // passing the last RC frame it received, so processing it would drive the
        // surfaces from stale sticks and let a stale low throttle re-arm.
        if self.arming.failsafe_active {
            return self.hold_failsafe(events);
        }

        // Dispatch on command variant and mode
        let elevons = match (commands, mode) {
            // ULTRA-FAST PATH: Raw commands in manual mode
            (PilotCommands::Raw(raw), AttitudeMode::Manual) => {
                self.step_fast_path_raw(&raw.channels)
            }

            // NORMALIZED PATH: Everything else
            (PilotCommands::Raw(raw), _) => {
                self.step_normalized(&raw.to_normalized(), inputs, dt, &mut events)
            }

            (PilotCommands::Normalized(norm), _) => {
                self.step_normalized(norm, inputs, dt, &mut events)
            }
        };
        LawOutputs {
            elevons,
            snapshot: self.last_output,
            events,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    fn step_fast_path_raw(&mut self, channels: &[u16; 16]) -> ElevonCommand {
        self.arming.update(channels[THROTTLE_CH]);
        self.attitude_controller.enabled = false;
        self.yaw_damper.reset();

        let elevon_outputs = mix_elevons_direct_lut(channels);
        self.last_saturation = elevon_outputs.saturation;

        let (left_thrust, right_thrust) = if self.arming.armed {
            // Invert yaw in raw RC space if YAW_INVERT is -1.0 (mirror around center=1024)
            let yaw_raw = if YAW_INVERT < 0.0 {
                2047 - channels[YAW_CH]
            } else {
                channels[YAW_CH]
            };
            throttle_with_differential_lut(channels[THROTTLE_CH], yaw_raw)
        } else {
            (0, 0)
        };

        // Store engine values — the async main loop sends them via DShot
        self.last_output = ControllerOutputSnapshot {
            elevon_left_us: elevon_outputs.left_us,
            elevon_right_us: elevon_outputs.right_us,
            engine_left_dshot: left_thrust,
            engine_right_dshot: right_thrust,
            att_age_us: u32::MAX,
            pitch_terms: AxisTerms::default(),
            roll_terms: AxisTerms::default(),
            saturation: elevon_outputs.saturation,
            yaw_damp: 0.0,
            ..self.last_output
        };
        ElevonCommand::Trimmed {
            left_us: elevon_outputs.left_us,
            right_us: elevon_outputs.right_us,
        }
    }

    /// Report flying on a held attitude sample: once when it lasts a second
    /// tick (EVT_ATTITUDE_HELD), once when it expires and the attitude
    /// controller falls back to the pilot's inputs (EVT_ATTITUDE_STALE). Only
    /// while the controller is on: in Manual or disarmed nothing uses it.
    fn track_attitude_hold(
        &mut self,
        fresh: bool,
        cached: Option<&Attitude>,
        held: bool,
        now: Instant,
        events: &mut LawEvents,
    ) {
        if fresh {
            self.held_ticks = 0;
            self.attitude_dropped = false;
            return;
        }
        let age_ms = cached.map_or(0, |a| (now - a.timestamp).as_millis());
        if held {
            self.held_ticks = self.held_ticks.saturating_add(1);
            if self.held_ticks == 2 && self.attitude_controller.enabled {
                events.attitude_held_ms = Some(age_ms);
            }
        } else if cached.is_some() && !self.attitude_dropped {
            self.attitude_dropped = true;
            if self.attitude_controller.enabled {
                events.attitude_stale_ms = Some(age_ms);
            }
        }
    }

    fn step_normalized(
        &mut self,
        norm: &NormalizedCommands,
        inputs: &LawInputs,
        dt: f32,
        events: &mut LawEvents,
    ) -> ElevonCommand {
        let attitude = inputs.attitude;
        let now = inputs.now;

        // Throttle-low auto-arm for RC input; skipped in RPC mode (explicit arm/disarm)
        if !self.explicit_arming_only {
            let throttle_rc_equiv = (norm.throttle * 2047.0) as u16;
            self.arming.update(throttle_rc_equiv);
        }

        // Enable attitude controller based on mode. Recomputed every tick, so it
        // must keep the supervisor's verdict too: `check_core1_health` clears
        // `enabled` when Core 1 (IMU) stops, and this line would otherwise turn it
        // straight back on. `core1_healthy` stays true while the supervisor is off.
        self.attitude_controller.enabled = (norm.attitude_mode == AttitudeMode::Stabilized
            || norm.attitude_mode == AttitudeMode::AltitudeHold)
            && self.arming.armed
            && inputs.core1_healthy;

        // The cached sample covers ticks where no new attitude arrived, but only
        // for `ATTITUDE_HOLD_MAX_MS`. Without the age check a dead IMU would
        // leave the PID flying on a frozen attitude (the IMU's own failure
        // marker is rejected upstream, and this fallback would quietly bring
        // back the last good sample instead).
        let cached = self.last_attitude;
        let fresh_cached = cached
            .as_ref()
            .filter(|a| is_attitude_valid_at(a, Duration::from_millis(ATTITUDE_HOLD_MAX_MS), now));
        self.track_attitude_hold(
            attitude.is_some(),
            cached.as_ref(),
            fresh_cached.is_some(),
            now,
            events,
        );

        // Compute setpoint from mode, with autotune override taking precedence
        let mut heading_target_deg = 0.0f32;
        let mut heading_error_deg = 0.0f32;
        let (pitch_sp_deg, roll_sp_deg) = if let Some(ovr) = self.setpoint_override {
            ovr
        } else {
            match norm.attitude_mode {
                AttitudeMode::Stabilized => {
                    // Stick IS the setpoint: center=level, full deflection=max angle
                    let stick_roll_deg = norm.roll * STABILIZED_MAX_ROLL_DEG;
                    let roll_sp_deg = if self.heading_hold_active {
                        match (self.locked_heading_rad, attitude.or(fresh_cached)) {
                            (Some(target_rad), Some(att)) => {
                                heading_target_deg = target_rad.to_degrees();
                                heading_error_deg = self
                                    .heading_controller
                                    .heading_error_deg(target_rad, att.yaw);
                                self.heading_controller.update_dt(target_rad, att.yaw, dt)
                            }
                            // No target locked yet or no attitude data — fall back to stick.
                            _ => stick_roll_deg,
                        }
                    } else {
                        stick_roll_deg
                    };
                    (norm.pitch * STABILIZED_MAX_PITCH_DEG, roll_sp_deg)
                }
                AttitudeMode::AltitudeHold => {
                    // Level hold: fixed 0°/0° (future: altitude controller adjusts)
                    (0.0, 0.0)
                }
                AttitudeMode::Manual => (0.0, 0.0), // unused, PID disabled
            }
        };

        // Convert setpoints to radians and apply smoothing
        let pitch_setpoint_rad = pitch_sp_deg.to_radians();
        let roll_setpoint_rad = roll_sp_deg.to_radians();

        if norm.attitude_mode != AttitudeMode::Manual {
            if self.setpoint_override.is_some() {
                // Autotune relay is an intentional square wave — smoothing it would
                // attenuate the excitation the Ku/Tu computation assumes.
                self.filtered_pitch_setpoint_rad = pitch_setpoint_rad;
                self.filtered_roll_setpoint_rad = roll_setpoint_rad;
            } else {
                // EMA smoothing, with each step capped so the target never moves
                // faster than MAX_SETPOINT_RATE_DEG_S.
                let max_step = MAX_SETPOINT_RATE_DEG_S.to_radians() * dt;
                self.filtered_pitch_setpoint_rad = smooth_setpoint(
                    self.filtered_pitch_setpoint_rad,
                    pitch_setpoint_rad,
                    SETPOINT_FILTER_ALPHA,
                    max_step,
                );
                self.filtered_roll_setpoint_rad = smooth_setpoint(
                    self.filtered_roll_setpoint_rad,
                    roll_setpoint_rad,
                    SETPOINT_FILTER_ALPHA,
                    max_step,
                );
            }
        }

        // Build control inputs from normalized commands
        let pilot_inputs = ControlInputs {
            pitch: norm.pitch,
            roll: norm.roll,
            yaw: norm.yaw,
            throttle: norm.throttle,
        };

        // Store attitude for fallback
        if let Some(att) = attitude {
            self.last_attitude = Some(*att);
        }

        // Track PID corrections for output snapshot
        let mut pitch_correction = 0.0f32;
        let mut roll_correction = 0.0f32;
        let mut pitch_terms = AxisTerms::default();
        let mut roll_terms = AxisTerms::default();
        let mut att_age_us = u32::MAX;
        let mut yaw_damp = 0.0f32;

        // Apply control mode logic
        let final_inputs = match norm.attitude_mode {
            AttitudeMode::Manual => {
                if self.attitude_controller.is_active() {
                    self.attitude_controller.reset();
                    self.last_saturation = MixSaturation::default();
                }
                self.yaw_damper.reset();
                if self.heading_hold_active {
                    self.disengage_heading_hold();
                }
                pilot_inputs
            }

            AttitudeMode::Stabilized | AttitudeMode::AltitudeHold => {
                // (PID reset on leaving Manual happens in `step()`, which is the
                // only place that still knows the previous mode.)

                // Try to get attitude data (current, or a cached sample that is
                // still fresh)
                match attitude.or(fresh_cached) {
                    Some(att) => {
                        // Compute attitude corrections
                        // Reset integrator when disarmed or throttle near zero
                        let low_throttle = !self.arming.armed || norm.throttle < 0.05;
                        // Measurements stay in raw AHRS frame (nose-up = positive pitch).
                        // PITCH_INVERT is already baked into the setpoint via norm.pitch
                        // from to_normalized(), so applying it again here would double-invert
                        // and produce the wrong correction direction.
                        // D-term rates are negated because rate = d(error)/dt = -d(measurement)/dt.
                        att_age_us =
                            now.saturating_duration_since(att.timestamp)
                                .as_micros()
                                .min(u64::from(u32::MAX - 1)) as u32;
                        let (pt, rt) = self.attitude_controller.update_dt(
                            self.filtered_pitch_setpoint_rad,
                            self.filtered_roll_setpoint_rad,
                            att.pitch,
                            att.roll,
                            Some((-att.roll_rate, -att.pitch_rate, att.yaw_rate)),
                            low_throttle,
                            self.last_saturation,
                            dt,
                        );
                        pitch_terms = pt;
                        roll_terms = rt;
                        pitch_correction = pt.total();
                        roll_correction = rt.total();

                        // Same gate as the PID (mode, armed, Core 1 healthy),
                        // plus the throttle: differential thrust scales with it.
                        yaw_damp = if self.attitude_controller.enabled && !low_throttle {
                            self.yaw_damper.update(att.yaw_rate)
                        } else {
                            self.yaw_damper.reset();
                            0.0
                        };

                        // 100% PID output for pitch/roll, throttle/yaw remain manual
                        ControlInputs {
                            pitch: pitch_correction,
                            roll: roll_correction,
                            yaw: pilot_inputs.yaw,
                            throttle: pilot_inputs.throttle,
                        }
                    }
                    None => {
                        // No attitude - fallback to manual, damper included
                        self.yaw_damper.reset();
                        pilot_inputs
                    }
                }
            }
        };

        // Apply final outputs
        let elevon_outputs = mix_elevons(&final_inputs);
        self.last_saturation = elevon_outputs.saturation;

        // Linear DShot mapping for normalized commands — the RC throttle curve
        // has a deadzone/ramp designed for stick input, not a 0-100% command.
        let base_thrust = (final_inputs.throttle * DSHOT_THROTTLE_MAX as f32) as u16;
        // The damper drives the engines only: through `mix_elevons` its yaw
        // would become a roll command that the roll PID then fights.
        let yaw_rc = normalized_yaw_to_rc((final_inputs.yaw + yaw_damp).clamp(-1.0, 1.0));

        let (left_thrust, right_thrust) = if self.arming.armed {
            apply_differential_thrust_lut(base_thrust, yaw_rc)
        } else {
            (0, 0)
        };

        // Engine values stored in last_output — the async main loop sends them via DShot

        // Capture controller output snapshot
        self.last_output = ControllerOutputSnapshot {
            pitch_correction,
            roll_correction,
            pitch_setpoint_deg: pitch_sp_deg,
            roll_setpoint_deg: roll_sp_deg,
            elevon_left_us: elevon_outputs.left_us,
            elevon_right_us: elevon_outputs.right_us,
            engine_left_dshot: left_thrust,
            engine_right_dshot: right_thrust,
            heading_hold_active: self.heading_hold_active,
            heading_target_deg,
            heading_error_deg,
            att_age_us,
            pitch_sp_used_deg: self.filtered_pitch_setpoint_rad.to_degrees(),
            roll_sp_used_deg: self.filtered_roll_setpoint_rad.to_degrees(),
            pitch_terms,
            roll_terms,
            saturation: elevon_outputs.saturation,
            yaw_damp,
            ..self.last_output
        };
        ElevonCommand::Trimmed {
            left_us: elevon_outputs.left_us,
            right_us: elevon_outputs.right_us,
        }
    }

    /// Failsafe outputs for one tick: surfaces centred, engines off, PID off, and
    /// the snapshot says so (the main loop sends the engine output to the ESCs).
    fn hold_failsafe(&mut self, events: LawEvents) -> LawOutputs {
        self.attitude_controller.enabled = false;
        self.yaw_damper.reset();
        self.last_output = ControllerOutputSnapshot {
            pitch_correction: 0.0,
            roll_correction: 0.0,
            elevon_left_us: ELEVON_LEFT_CENTER_US,
            elevon_right_us: ELEVON_RIGHT_CENTER_US,
            engine_left_dshot: 0,
            engine_right_dshot: 0,
            att_age_us: u32::MAX,
            pitch_terms: AxisTerms::default(),
            roll_terms: AxisTerms::default(),
            saturation: MixSaturation::default(),
            yaw_damp: 0.0,
            ..self.last_output
        };
        LawOutputs {
            elevons: ElevonCommand::Safe,
            snapshot: self.last_output,
            events,
        }
    }

    /// RC link lost: disarm, latch failsafe, clear the PID and its anti-windup.
    pub fn signal_loss(&mut self) {
        self.arming.signal_loss();
        self.attitude_controller.reset();
        self.last_saturation = MixSaturation::default();
    }

    /// RC link back: clear failsafe, but do not re-arm (the pilot repeats the gesture).
    pub fn signal_restored(&mut self) {
        self.arming.signal_restored();
    }

    /// The supervisor found Core 1 unhealthy: attitude control off until the next tick
    /// recomputes it (with `core1_healthy` false, it stays off).
    pub fn disable_attitude(&mut self) {
        self.attitude_controller.enabled = false;
    }

    #[must_use]
    pub const fn is_armed(&self) -> bool {
        self.arming.armed
    }

    #[must_use]
    pub const fn is_failsafe(&self) -> bool {
        self.arming.failsafe_active
    }

    #[must_use]
    pub const fn is_attitude_enabled(&self) -> bool {
        self.attitude_controller.enabled
    }

    #[must_use]
    pub const fn current_control_mode(&self) -> ControlMode {
        self.current_control_mode
    }

    /// Disable throttle-low auto-arming (RPC mode uses explicit arm/disarm only)
    pub fn set_explicit_arming(&mut self, enabled: bool) {
        self.explicit_arming_only = enabled;
    }

    /// Manual arm (for RTT/debug control)
    pub fn arm(&mut self) {
        self.arming.arm();
    }

    /// Manual disarm (for RTT/debug control)
    pub fn disarm(&mut self) {
        self.arming.disarm();
        self.disengage_heading_hold();
    }

    /// Update PID gains at runtime (resets integral state).
    pub fn set_pid_gains(&mut self, config: PidConfig) {
        self.attitude_controller.update_config(config);
        self.gains_version = self.gains_version.wrapping_add(1);
    }

    /// Active PID gains.
    #[must_use]
    pub const fn pid_config(&self) -> &PidConfig {
        &self.attitude_controller.config
    }

    /// Changes on every gain update; compare to detect new gains.
    #[must_use]
    pub const fn gains_version(&self) -> u32 {
        self.gains_version
    }

    /// Set attitude setpoint override (degrees). Used by autotuner.
    pub fn set_setpoint_override(&mut self, pitch_deg: f32, roll_deg: f32) {
        self.setpoint_override = Some((pitch_deg, roll_deg));
    }

    /// Clear setpoint override, returning to normal RC/RPC control.
    pub fn clear_setpoint_override(&mut self) {
        self.setpoint_override = None;
    }

    /// Engage heading-hold with the given target heading (AHRS yaw, radians).
    /// Only takes effect while `AttitudeMode::Stabilized` is selected.
    pub fn engage_heading_hold(&mut self, target_rad: f32) {
        self.heading_hold_active = true;
        self.locked_heading_rad = Some(target_rad);
        self.heading_controller.reset();
    }

    /// Disengage heading-hold, returning roll to stick-derived control.
    pub fn disengage_heading_hold(&mut self) {
        self.heading_hold_active = false;
        self.locked_heading_rad = None;
        self.heading_controller.reset();
    }

    #[must_use]
    pub const fn is_heading_hold_active(&self) -> bool {
        self.heading_hold_active
    }
}
