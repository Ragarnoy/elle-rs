use defmt::{info, warn};
use elle_config::lut::apply_differential_thrust_lut;
use elle_config::*;
use elle_control::SavedGains;
use elle_control::commands::{AttitudeMode, NormalizedCommands, PilotCommands};
use elle_control::filter::smooth_setpoint;
use elle_control::mixing::{
    elevons::{ControlInputs, MixSaturation, mix_elevons, mix_elevons_direct_lut},
    yaw::{normalized_yaw_to_rc, throttle_with_differential_lut},
};
use elle_control::{
    arming::ArmingState,
    heading::HeadingController,
    pid::{AttitudeController, AxisTerms, PidConfig},
    yaw_damper::YawDamper,
};
use elle_hardware::event::{EVT_RC_RESTORED, EVT_RC_SIGNAL_LOST, EVT_RC_WARNING};
use elle_hardware::imu::{AttitudeData, CORE1_HEARTBEAT, is_attitude_valid};
use elle_hardware::pwm::PwmOutputs;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum RcLinkState {
    Ok,
    Warning,
    Lost,
}

/// Core health monitoring structure
#[derive(Debug, Clone, Copy, defmt::Format)]
struct CoreHealth {
    last_heartbeat: Instant,
    is_healthy: bool,
    last_logged_healthy: bool,
}

impl Default for CoreHealth {
    fn default() -> Self {
        Self {
            last_heartbeat: Instant::now(),
            is_healthy: true,
            last_logged_healthy: true,
        }
    }
}

impl CoreHealth {
    fn update_heartbeat(&mut self) {
        self.last_heartbeat = Instant::now();
        self.is_healthy = true;
    }

    fn check_health(&mut self, timeout: Duration) -> bool {
        if self.last_heartbeat.elapsed() > timeout {
            self.is_healthy = false;
            false
        } else {
            true
        }
    }
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
}

pub struct FlightController<'a> {
    pwm: PwmOutputs<'a>,
    arming: ArmingState,
    attitude_controller: AttitudeController,
    /// Yaw rate → differential thrust, alongside the attitude PID.
    yaw_damper: YawDamper,
    last_packet_time: Instant,
    last_attitude: Option<AttitudeData>,
    // Smoothed setpoints for attitude hold (in radians for consistency)
    filtered_pitch_setpoint_rad: f32,
    filtered_roll_setpoint_rad: f32,
    /// Previous tick's mixer saturation, for the PID's anti-windup.
    last_saturation: MixSaturation,
    /// When `update` last ran, for the measured dt.
    last_update_at: Option<Instant>,
    /// Bumped on every gain change, so the logger can record new gains.
    gains_version: u32,
    // Control mode tracking
    current_control_mode: ControlMode,
    // Controller output snapshot for observability
    last_output: ControllerOutputSnapshot,
    // Supervisor components
    core1_health: CoreHealth,
    last_watchdog_kick: Instant,
    supervisor_enabled: bool,
    // Setpoint override for autotune relay feedback
    setpoint_override: Option<(f32, f32)>,
    // Heading-hold outer loop: modifier active only while AttitudeMode::Stabilized
    heading_controller: HeadingController,
    heading_hold_active: bool,
    locked_heading_rad: Option<f32>,
    // RC link state machine
    rc_link_state: RcLinkState,
    // When true, arming is only via explicit arm()/disarm() — skips throttle-low auto-arm
    explicit_arming_only: bool,
}

impl<'a> FlightController<'a> {
    #[must_use]
    pub fn new(pwm: PwmOutputs<'a>) -> Self {
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
            pwm,
            arming: ArmingState::default(),
            attitude_controller,
            yaw_damper: YawDamper::from_config(),
            last_packet_time: Instant::now(),
            last_attitude: None,
            filtered_pitch_setpoint_rad: 0.0,
            filtered_roll_setpoint_rad: 0.0,
            last_saturation: MixSaturation::default(),
            last_update_at: None,
            gains_version: 0,
            current_control_mode: ControlMode::Manual,
            last_output: ControllerOutputSnapshot::default(),
            core1_health: CoreHealth::default(),
            last_watchdog_kick: Instant::now(),
            supervisor_enabled: false,
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
            rc_link_state: RcLinkState::Ok,
            explicit_arming_only: false,
        }
    }

    /// Get the last computed engine output values (DShot 0-1999).
    /// The caller is responsible for sending these to the engines via DShot.
    #[must_use]
    pub const fn engine_output(&self) -> (u16, u16) {
        (
            self.last_output.engine_left_dshot,
            self.last_output.engine_right_dshot,
        )
    }

    /// Initialize supervisor components (watchdog and health monitoring)
    pub fn initialize_supervisor(&mut self, watchdog: Watchdog<'static>) {
        // Configure watchdog for critical flight safety timeout. It lives in
        // elle_hardware::watchdog so the flash manager can extend it too.
        elle_hardware::watchdog::install(watchdog, Duration::from_millis(WATCHDOG_TIMEOUT_MS));
        // Keep supervisor health monitoring disabled until explicitly enabled
        self.last_watchdog_kick = Instant::now();
        info!(
            "Supervisor: Watchdog initialized with {}ms timeout",
            WATCHDOG_TIMEOUT_MS
        );
    }

    /// Enable supervisor health monitoring after all tasks are ready
    pub fn enable_supervisor_monitoring(&mut self) {
        self.supervisor_enabled = true;
        info!("Supervisor: Health monitoring enabled");
    }

    /// Feed the watchdog timer to prevent system reset
    fn kick_watchdog(&mut self) {
        elle_hardware::watchdog::feed(Duration::from_millis(WATCHDOG_TIMEOUT_MS));
        self.last_watchdog_kick = Instant::now();
    }

    /// Check and update Core 1 health based on heartbeat signal
    fn check_core1_health(&mut self) -> bool {
        if !self.supervisor_enabled {
            return true;
        }

        // Check for heartbeat signal from Core 1
        if CORE1_HEARTBEAT.try_take().is_some() {
            self.core1_health.update_heartbeat();
        }

        // Check if Core 1 is healthy using configured timeout
        let is_healthy = self
            .core1_health
            .check_health(Duration::from_millis(CORE1_HEALTH_TIMEOUT_MS));

        // Only log on state transitions to prevent spam
        if is_healthy != self.core1_health.last_logged_healthy {
            if !is_healthy {
                elle_hardware::elle_event!(
                    warn,
                    elle_hardware::event::EVT_CORE1_UNHEALTHY,
                    "Supervisor: Core 1 (IMU) unhealthy - last heartbeat {}ms ago",
                    self.core1_health.last_heartbeat.elapsed().as_millis()
                );
                // Disable attitude control if Core 1 is unhealthy
                self.attitude_controller.enabled = false;
            } else {
                elle_hardware::elle_event!(
                    info,
                    elle_hardware::event::EVT_CORE1_RESTORED,
                    "Supervisor: Core 1 (IMU) healthy - heartbeat restored"
                );
            }
            self.core1_health.last_logged_healthy = is_healthy;
        } else if !is_healthy {
            // Still disable attitude control even if we don't log
            self.attitude_controller.enabled = false;
        }

        is_healthy
    }

    /// Main supervisor check - should be called in the main control loop
    pub fn supervisor_check(&mut self) -> bool {
        if !self.supervisor_enabled {
            return true;
        }

        let core1_healthy = self.check_core1_health();

        // Kick watchdog if both cores are healthy
        if core1_healthy {
            self.kick_watchdog();
        } else {
            warn!("Supervisor: Skipping watchdog kick due to Core 1 health issues");
        }

        core1_healthy
    }

    /// Record RC packet arrival for link-age tracking / failsafe.
    /// Call on packet reception, not per control tick — `update()` runs every
    /// tick with retained commands, so stamping there would mask link loss.
    pub fn note_rc_packet(&mut self, timestamp: Instant) {
        self.last_packet_time = timestamp;
    }

    /// Main update method - accepts PilotCommands from any source
    pub fn update(&mut self, commands: &PilotCommands, attitude: Option<&AttitudeData>) {
        self.last_packet_time = commands.timestamp();

        let now = Instant::now();
        let dt_us = self
            .last_update_at
            .map_or(0, |t| now.saturating_duration_since(t).as_micros() as u32);
        self.last_update_at = Some(now);

        let mode = commands.attitude_mode();

        // Track mode changes
        let current_mode = ControlMode::from(mode);

        if current_mode != self.current_control_mode {
            info!(
                "Control mode changed: {:?} -> {:?}",
                self.current_control_mode, current_mode
            );
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
            self.hold_failsafe();
            self.last_output.dt_us = dt_us;
            return;
        }

        // Dispatch on command variant and mode
        match (commands, mode) {
            // ULTRA-FAST PATH: Raw commands in manual mode
            (PilotCommands::Raw(raw), AttitudeMode::Manual) => {
                self.update_fast_path_raw(&raw.channels);
            }

            // NORMALIZED PATH: Everything else
            (PilotCommands::Raw(raw), _) => {
                self.update_normalized(&raw.to_normalized(), attitude);
            }

            (PilotCommands::Normalized(norm), _) => {
                self.update_normalized(norm, attitude);
            }
        }
        self.last_output.dt_us = dt_us;
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    fn update_fast_path_raw(&mut self, channels: &[u16; 16]) {
        self.arming.update(channels[THROTTLE_CH]);
        self.attitude_controller.enabled = false;
        self.yaw_damper.reset();

        let elevon_outputs = mix_elevons_direct_lut(channels);
        self.last_saturation = elevon_outputs.saturation;
        let (left_pulse, right_pulse) = self
            .pwm
            .set_elevons_with_trim(elevon_outputs.left_us, elevon_outputs.right_us);

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
            elevon_left_pulse_us: left_pulse,
            elevon_right_pulse_us: right_pulse,
            yaw_damp: 0.0,
            ..self.last_output
        };
    }

    fn update_normalized(&mut self, norm: &NormalizedCommands, attitude: Option<&AttitudeData>) {
        // Throttle-low auto-arm for RC input; skipped in RPC mode (explicit arm/disarm)
        if !self.explicit_arming_only {
            let throttle_rc_equiv = (norm.throttle * 2047.0) as u16;
            self.arming.update(throttle_rc_equiv);
        }

        // Enable attitude controller based on mode. Recomputed every tick, so it
        // must keep the supervisor's verdict too: `check_core1_health` clears
        // `enabled` when Core 1 (IMU) stops, and this line would otherwise turn it
        // straight back on. `is_healthy` stays true while the supervisor is off.
        self.attitude_controller.enabled = (norm.attitude_mode == AttitudeMode::Stabilized
            || norm.attitude_mode == AttitudeMode::AltitudeHold)
            && self.arming.armed
            && self.core1_health.is_healthy;

        // The cached sample covers ticks where no new attitude arrived, but only
        // within the same freshness limit the caller applies. Without the age
        // check a dead IMU would leave the PID flying on a frozen attitude (the
        // IMU's own failure marker is rejected upstream, and this fallback would
        // quietly bring back the last good sample instead).
        let cached = self.last_attitude;
        let fresh_cached = cached
            .as_ref()
            .filter(|a| is_attitude_valid(a, Duration::from_millis(IMU_MAX_AGE_MS)));

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
                                self.heading_controller.update(target_rad, att.yaw)
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
                let max_step = MAX_SETPOINT_RATE_DEG_S.to_radians() * CONTROL_LOOP_DT;
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
                // (PID reset on leaving Manual happens in `update()`, which is the
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
                        att_age_us = Instant::now()
                            .saturating_duration_since(att.timestamp)
                            .as_micros()
                            .min(u64::from(u32::MAX - 1))
                            as u32;
                        let (pt, rt) = self.attitude_controller.update(
                            self.filtered_pitch_setpoint_rad,
                            self.filtered_roll_setpoint_rad,
                            att.pitch,
                            att.roll,
                            Some((-att.roll_rate, -att.pitch_rate, att.yaw_rate)),
                            low_throttle,
                            self.last_saturation,
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
        let (left_pulse, right_pulse) = self
            .pwm
            .set_elevons_with_trim(elevon_outputs.left_us, elevon_outputs.right_us);

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
            dt_us: self.last_output.dt_us,
            att_age_us,
            pitch_sp_used_deg: self.filtered_pitch_setpoint_rad.to_degrees(),
            roll_sp_used_deg: self.filtered_roll_setpoint_rad.to_degrees(),
            pitch_terms,
            roll_terms,
            saturation: elevon_outputs.saturation,
            elevon_left_pulse_us: left_pulse,
            elevon_right_pulse_us: right_pulse,
            yaw_damp,
        };
    }

    pub fn check_failsafe(&mut self) {
        let age = self.last_packet_time.elapsed();
        let new_state = if age > Duration::from_millis(RC_TIMEOUT_MS) {
            RcLinkState::Lost
        } else if age > Duration::from_millis(RC_WARNING_MS) {
            RcLinkState::Warning
        } else {
            RcLinkState::Ok
        };

        if new_state == self.rc_link_state {
            return;
        }

        let old_state = self.rc_link_state;
        self.rc_link_state = new_state;

        match (old_state, new_state) {
            (_, RcLinkState::Ok) => {
                // Restored — clear failsafe but do NOT re-arm (pilot must throttle-low arm)
                self.arming.signal_restored();
                elle_hardware::elle_event!(info, EVT_RC_RESTORED, "RC signal restored");
            }
            (RcLinkState::Ok, RcLinkState::Warning) => {
                elle_hardware::elle_event!(
                    warn,
                    EVT_RC_WARNING,
                    "RC signal warning ({}ms)",
                    age.as_millis()
                );
            }
            (_, RcLinkState::Lost) => {
                self.arming.signal_loss();
                self.attitude_controller.reset();
                self.last_saturation = MixSaturation::default();
                self.apply_failsafe();
                elle_hardware::elle_event!(
                    error,
                    EVT_RC_SIGNAL_LOST,
                    "RC SIGNAL LOST ({}ms)",
                    age.as_millis()
                );
            }
            // Warning→Warning already handled by early return
            _ => {}
        }
    }

    pub fn apply_failsafe(&mut self) {
        self.pwm.set_safe_positions();
    }

    /// Failsafe outputs for one tick: surfaces centred, engines off, PID off, and
    /// the snapshot says so (the main loop sends `engine_output()` to the ESCs).
    fn hold_failsafe(&mut self) {
        self.attitude_controller.enabled = false;
        self.yaw_damper.reset();
        self.pwm.set_safe_positions();
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
            elevon_left_pulse_us: ELEVON_LEFT_CENTER_US,
            elevon_right_pulse_us: ELEVON_RIGHT_CENTER_US,
            yaw_damp: 0.0,
            ..self.last_output
        };
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
    pub const fn rc_link_state(&self) -> RcLinkState {
        self.rc_link_state
    }

    #[must_use]
    pub fn rc_signal_age_ms(&self) -> u16 {
        let age = self.last_packet_time.elapsed().as_millis();
        age.min(u16::MAX as u64) as u16
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

    /// Force elevons to center (safe position). Used by kill switch.
    pub fn set_safe_positions(&mut self) {
        self.pwm.set_safe_positions();
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

    /// Apply PID gains from a `SavedGains` snapshot (used by autotuner).
    pub fn apply_saved_gains(&mut self, gains: &SavedGains) {
        self.set_pid_gains((*gains).into());
    }

    /// Get current PID gains as a `SavedGains` snapshot.
    #[must_use]
    pub fn get_pid_gains(&self) -> SavedGains {
        self.attitude_controller.config.into()
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

    /// Get the last controller output snapshot
    #[must_use]
    pub const fn last_output(&self) -> &ControllerOutputSnapshot {
        &self.last_output
    }
}

/// Performance monitoring utilities for tracking task execution times
#[cfg(feature = "performance-monitoring")]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct TaskTiming {
    min_us: u32,
    pub max_us: u32,
    pub avg_us: u32,
    samples: u32,
}

#[cfg(feature = "performance-monitoring")]
impl Default for TaskTiming {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "performance-monitoring")]
impl TaskTiming {
    const fn new() -> Self {
        Self {
            min_us: u32::MAX,
            max_us: 0,
            avg_us: 0,
            samples: 0,
        }
    }

    fn update(&mut self, execution_time_us: u32) {
        self.min_us = self.min_us.min(execution_time_us);
        self.max_us = self.max_us.max(execution_time_us);

        // Running average calculation to avoid overflow
        if self.samples == 0 {
            self.avg_us = execution_time_us;
        } else {
            // Weighted average with more recent samples having slightly more weight
            let weight = (self.samples + 1).min(100);
            self.avg_us = (self.avg_us * (weight - 1) + execution_time_us) / weight;
        }

        self.samples = self.samples.saturating_add(1);
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    /// Get CPU utilization as percentage for a given target frequency
    const fn cpu_utilization_percent(&self, target_frequency_hz: u32) -> f32 {
        let target_period_us = 1_000_000 / target_frequency_hz;
        (self.avg_us as f32 / target_period_us as f32) * 100.0
    }
}

/// Performance monitor for tracking multiple tasks
#[cfg(feature = "performance-monitoring")]
#[derive(Debug, defmt::Format)]
pub struct PerformanceMonitor {
    pub control_loop: TaskTiming,
    led_update: TaskTiming,
    ulog_logging: TaskTiming,
    // IMU (Core 1) and flash manager timings live in elle-hardware's `timing`
    // module: those tasks are in that crate and cannot call back into this one.
}

#[cfg(feature = "performance-monitoring")]
impl Default for PerformanceMonitor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "performance-monitoring")]
impl PerformanceMonitor {
    const fn new() -> Self {
        Self {
            control_loop: TaskTiming::new(),
            led_update: TaskTiming::new(),
            ulog_logging: TaskTiming::new(),
        }
    }

    fn log_performance_summary(&self) {
        info!("=== DUAL-CORE PERFORMANCE SUMMARY ===");

        // Core 0 tasks (Control, LED, Flash)
        let core0_control_cpu = self
            .control_loop
            .cpu_utilization_percent(CONTROL_LOOP_FREQUENCY_HZ);
        let core0_led_cpu = self.led_update.cpu_utilization_percent(100);
        let core0_total_cpu = core0_control_cpu + core0_led_cpu;

        info!(
            "CORE 0 (Control + LED + Flash): {}% total load",
            core0_total_cpu as u8
        );
        info!(
            "  Control: min={} avg={} max={}μs ({}% @ {}Hz)",
            self.control_loop.min_us,
            self.control_loop.avg_us,
            self.control_loop.max_us,
            core0_control_cpu as u8,
            CONTROL_LOOP_FREQUENCY_HZ
        );
        info!(
            "  LED: min={} avg={} max={}μs ({}% @ 100Hz)",
            self.led_update.min_us,
            self.led_update.avg_us,
            self.led_update.max_us,
            core0_led_cpu as u8
        );

        // Core 1 tasks (IMU only): one wake-up per 1 kHz DATA_RDY sample
        let imu = elle_hardware::timing::IMU_TIMING.snapshot();
        let core1_imu_cpu =
            (imu.avg_us as f32 * elle_config::IMU_UPDATE_FREQUENCY_HZ as f32) / 10_000.0;
        info!("CORE 1 (IMU only): {}% total load", core1_imu_cpu as u8);
        if imu.samples > 0 {
            info!(
                "  IMU: min={} avg={} max={}μs ({}% @ {}Hz)",
                imu.min_us,
                imu.avg_us,
                imu.max_us,
                core1_imu_cpu as u8,
                elle_config::IMU_UPDATE_FREQUENCY_HZ
            );
        }

        // Flash operations (on-demand, Core 0)
        let flash = elle_hardware::timing::FLASH_TIMING.snapshot();
        if flash.samples > 0 {
            info!(
                "Flash Operations: min={} avg={} max={}μs | {} operations completed",
                flash.min_us, flash.avg_us, flash.max_us, flash.samples
            );
        }

        // ULog logging (control-loop rate, Core 0)
        if self.ulog_logging.samples > 0 {
            let ulog_cpu = self
                .ulog_logging
                .cpu_utilization_percent(CONTROL_LOOP_FREQUENCY_HZ);
            info!(
                "ULog Logging: min={} avg={} max={}μs ({}% @ {}Hz) | {} samples",
                self.ulog_logging.min_us,
                self.ulog_logging.avg_us,
                self.ulog_logging.max_us,
                ulog_cpu as u8,
                CONTROL_LOOP_FREQUENCY_HZ,
                self.ulog_logging.samples
            );
        }
    }

    pub fn reset_all(&mut self) {
        self.control_loop.reset();
        self.led_update.reset();
        self.ulog_logging.reset();
        elle_hardware::timing::IMU_TIMING.reset();
        elle_hardware::timing::FLASH_TIMING.reset();
    }
}

/// Simple timing helper for measuring execution time
#[cfg(feature = "performance-monitoring")]
pub struct TimingMeasurement {
    start: Instant,
}

#[cfg(feature = "performance-monitoring")]
impl TimingMeasurement {
    pub fn start() -> Self {
        Self {
            start: Instant::now(),
        }
    }

    pub fn elapsed_us(&self) -> u32 {
        let elapsed = self.start.elapsed();
        // Use embassy-time methods - get as microseconds directly
        elapsed.as_micros() as u32
    }
}

/// Global performance monitor instance
#[cfg(feature = "performance-monitoring")]
pub static mut PERFORMANCE_MONITOR: PerformanceMonitor = PerformanceMonitor::new();

/// Helper function to safely update performance monitor
#[cfg(feature = "performance-monitoring")]
pub fn update_control_loop_timing(elapsed_us: u32) {
    unsafe {
        (*core::ptr::addr_of_mut!(PERFORMANCE_MONITOR))
            .control_loop
            .update(elapsed_us);
    }
}

#[cfg(feature = "performance-monitoring")]
pub fn update_led_timing(elapsed_us: u32) {
    unsafe {
        (*core::ptr::addr_of_mut!(PERFORMANCE_MONITOR))
            .led_update
            .update(elapsed_us);
    }
}

#[cfg(feature = "performance-monitoring")]
pub fn update_ulog_timing(elapsed_us: u32) {
    unsafe {
        (*core::ptr::addr_of_mut!(PERFORMANCE_MONITOR))
            .ulog_logging
            .update(elapsed_us);
    }
}

#[cfg(feature = "performance-monitoring")]
pub fn log_performance_summary() {
    unsafe {
        (*core::ptr::addr_of!(PERFORMANCE_MONITOR)).log_performance_summary();
    }
}

// No-op stubs when performance monitoring is disabled
#[cfg(not(feature = "performance-monitoring"))]
#[inline(always)]
pub const fn update_control_loop_timing(_elapsed_us: u32) {}

#[cfg(not(feature = "performance-monitoring"))]
#[inline(always)]
pub const fn update_led_timing(_elapsed_us: u32) {}

#[cfg(not(feature = "performance-monitoring"))]
#[inline(always)]
pub const fn update_ulog_timing(_elapsed_us: u32) {}

#[cfg(not(feature = "performance-monitoring"))]
#[inline(always)]
pub const fn log_performance_summary() {}

// Dummy timing measurement when feature is disabled
#[cfg(not(feature = "performance-monitoring"))]
pub struct TimingMeasurement;

#[cfg(not(feature = "performance-monitoring"))]
impl TimingMeasurement {
    #[must_use]
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub const fn start() -> Self {
        Self
    }

    #[must_use]
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub const fn elapsed_us(&self) -> u32 {
        0
    }
}

pub static SUP_LED_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static SUP_IMU_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static SUP_FC_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// Start signals to release tasks from the barrier
pub static SUP_START_IMU: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static SUP_START_FC: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Supervisor task that waits for all participants to be ready, then releases them simultaneously
#[embassy_executor::task]
pub async fn supervisor_task() {
    info!("Supervisor: waiting for tasks to initialize");

    // Prepare futures for all participants and wait for them concurrently
    let led_ready = SUP_LED_READY.wait();
    let imu_ready = SUP_IMU_READY.wait();
    let fc_ready = SUP_FC_READY.wait();

    let _ = embassy_futures::join::join3(led_ready, imu_ready, fc_ready).await;

    info!("Supervisor: all tasks initialized, releasing start barrier");

    // Release participants
    SUP_START_IMU.signal(());
    SUP_START_FC.signal(());
}
