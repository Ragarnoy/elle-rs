use crate::config::*;
use crate::control::{arming::ArmingState, pid::AttitudeController};
use crate::hardware::imu::AttitudeData;
use crate::hardware::pwm::PwmOutputs;
use defmt::{debug, info, warn};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use free_flight_stabilization::FlightStabilizerConfig;
use sbus_rs::SbusPacket;

#[cfg(not(feature = "legacy-ctrl"))]
use crate::control::mixing::{
    elevons::{ControlInputs, mix_elevons, mix_elevons_direct_lut},
    yaw::{apply_differential_thrust_direct, throttle_with_differential_lut},
};

#[cfg(feature = "legacy-ctrl")]
use crate::control::mixing::apply_differential_complete;

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum ControlMode {
    Manual,    // Full manual control (~306)
    Mixed,     // Pilot + Autopilot blend (~1000)
    Autopilot, // Full autopilot control (~1694)
}

/// Core health monitoring structure
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct CoreHealth {
    pub last_heartbeat: Instant,
    pub heartbeat_count: u32,
    pub is_healthy: bool,
    pub last_logged_healthy: bool,
}

impl Default for CoreHealth {
    fn default() -> Self {
        Self {
            last_heartbeat: Instant::now(),
            heartbeat_count: 0,
            is_healthy: true,
            last_logged_healthy: true,
        }
    }
}

impl CoreHealth {
    pub fn update_heartbeat(&mut self) {
        self.last_heartbeat = Instant::now();
        self.heartbeat_count = self.heartbeat_count.wrapping_add(1);
        self.is_healthy = true;
    }

    pub fn check_health(&mut self, timeout: Duration) -> bool {
        if self.last_heartbeat.elapsed() > timeout {
            self.is_healthy = false;
            false
        } else {
            true
        }
    }
}

/// Global signal for Core 1 (IMU) heartbeat
pub static CORE1_HEARTBEAT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub struct FlightController<'a> {
    pwm: PwmOutputs<'a>,
    arming: ArmingState,
    attitude_controller: AttitudeController,
    last_packet_time: Instant,
    last_attitude: Option<AttitudeData>,
    // Smoothed setpoints for attitude hold (in radians for consistency)
    filtered_pitch_setpoint_rad: f32,
    filtered_roll_setpoint_rad: f32,
    // Control mode tracking
    current_control_mode: ControlMode,
    // Supervisor components
    watchdog: Option<Watchdog>,
    core1_health: CoreHealth,
    last_watchdog_kick: Instant,
    supervisor_enabled: bool,
}

impl<'a> FlightController<'a> {
    pub fn new(pwm: PwmOutputs<'a>) -> Self {
        // Use conservative gains similar to free-flight-stabilization example
        let mut config = FlightStabilizerConfig::<f32>::new();

        // Set the PID gains for roll, pitch, and yaw (from the example you provided)
        config.kp_roll = ROLL_KP;
        config.ki_roll = ROLL_KI;
        config.kd_roll = ROLL_KD;
        config.kp_pitch = PITCH_KP;
        config.ki_pitch = PITCH_KI;
        config.kd_pitch = PITCH_KD;
        config.kp_yaw = 0.3;
        config.ki_yaw = 0.05;
        config.kd_yaw = 0.00015;

        // Set the upper limit for the integral term to prevent windup
        config.i_limit = 25.0;

        // Set the scale to adjust the PID outputs to the actuator range
        config.scale = 0.015; // Increased for more responsive but still smooth control

        let mut attitude_controller = AttitudeController::with_config(config);
        attitude_controller.roll_hold_enabled = true; // Enable roll hold for CH8 control

        Self {
            pwm,
            arming: ArmingState::default(),
            attitude_controller,
            last_packet_time: Instant::now(),
            last_attitude: None,
            filtered_pitch_setpoint_rad: 0.0,
            filtered_roll_setpoint_rad: 0.0,
            current_control_mode: ControlMode::Manual,
            watchdog: None,
            core1_health: CoreHealth::default(),
            last_watchdog_kick: Instant::now(),
            supervisor_enabled: false,
        }
    }

    pub async fn initialize_escs(&mut self) {
        info!("ESC init: Starting");

        // Hold at minimum
        for _ in 0..200 {
            self.pwm
                .set_engines(ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US);
            Timer::after(Duration::from_millis(10)).await;
        }

        // Brief idle pulse
        info!("ESC init: Idle pulse");
        for _ in 0..50 {
            self.pwm
                .set_engines(ENGINE_IDLE_PULSE_US, ENGINE_IDLE_PULSE_US);
            Timer::after(Duration::from_millis(10)).await;
        }

        // Return to minimum
        for _ in 0..100 {
            self.pwm
                .set_engines(ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US);
            Timer::after(Duration::from_millis(10)).await;
        }

        info!("ESC init: Complete");
    }

    /// Initialize supervisor components (watchdog and health monitoring)
    pub fn initialize_supervisor(&mut self, mut watchdog: Watchdog) {
        // Configure watchdog for critical flight safety timeout
        watchdog.start(Duration::from_millis(WATCHDOG_TIMEOUT_MS));
        self.watchdog = Some(watchdog);
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
    pub fn kick_watchdog(&mut self) {
        if let Some(ref mut wd) = self.watchdog {
            wd.feed();
            self.last_watchdog_kick = Instant::now();
        }
    }

    /// Check and update Core 1 health based on heartbeat signal
    pub fn check_core1_health(&mut self) -> bool {
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
                warn!(
                    "Supervisor: Core 1 (IMU) unhealthy - last heartbeat {}ms ago",
                    self.core1_health.last_heartbeat.elapsed().as_millis()
                );
                // Disable attitude control if Core 1 is unhealthy
                self.attitude_controller.enabled = false;
            } else {
                info!("Supervisor: Core 1 (IMU) healthy - heartbeat restored");
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

    /// Get supervisor status for monitoring
    pub fn supervisor_status(&self) -> (bool, bool, u32) {
        (
            self.supervisor_enabled,
            self.core1_health.is_healthy,
            self.core1_health.heartbeat_count,
        )
    }

    /// Determine control mode from CH5 3-state switch
    fn determine_control_mode(&self, ch5_value: u16) -> ControlMode {
        if ch5_value < MANUAL_MODE_THRESHOLD {
            ControlMode::Manual
        } else if ch5_value < MIXED_MODE_THRESHOLD {
            ControlMode::Mixed
        } else {
            ControlMode::Autopilot
        }
    }

    /// Update method with feature flag support
    pub fn update(&mut self, packet: &SbusPacket) {
        self.last_packet_time = Instant::now();

        #[cfg(not(feature = "legacy-ctrl"))]
        {
            // Update arming state using throttle channel
            self.arming
                .update(packet.channels[THROTTLE_CH], packet.flags.failsafe);
            self.update_with_mixing(packet, None);
        }

        #[cfg(feature = "legacy-ctrl")]
        {
            // Update arming state using legacy engine channel
            self.arming
                .update(packet.channels[ENGINE_CH], packet.flags.failsafe);
            self.update_direct_elevons(packet);
        }
    }

    /// Ultra-fast mixing update method using LUTs
    #[cfg(not(feature = "legacy-ctrl"))]
    pub fn update_with_mixing(&mut self, packet: &SbusPacket, attitude: Option<&AttitudeData>) {
        self.last_packet_time = Instant::now();

        // Update arming state
        self.arming
            .update(packet.channels[THROTTLE_CH], packet.flags.failsafe);

        // Determine control mode from CH5 3-state switch
        let new_control_mode = self.determine_control_mode(packet.channels[ATTITUDE_ENABLE_CH]);
        if new_control_mode != self.current_control_mode {
            info!(
                "Control mode changed: {:?} -> {:?}",
                self.current_control_mode, new_control_mode
            );
            self.current_control_mode = new_control_mode;
        }

        // Enable attitude controller for Mixed and Autopilot modes
        self.attitude_controller.enabled = (self.current_control_mode == ControlMode::Mixed
            || self.current_control_mode == ControlMode::Autopilot)
            && self.arming.armed;

        // Get raw desired angles from SBUS channels using LUTs and convert to radians immediately
        let ch6_normalized = sbus_to_normalized_lut(packet.channels[ATTITUDE_PITCH_SETPOINT_CH]);
        let raw_pitch_deg = ATTITUDE_PITCH_MIN_DEG
            + (ch6_normalized + 1.0) * 0.5 * (ATTITUDE_PITCH_MAX_DEG - ATTITUDE_PITCH_MIN_DEG);
        let raw_pitch_setpoint_rad = raw_pitch_deg * core::f32::consts::PI / 180.0;

        let ch8_normalized = sbus_to_normalized_lut(packet.channels[ATTITUDE_ROLL_SETPOINT_CH]);
        let raw_roll_deg = ATTITUDE_ROLL_MIN_DEG
            + (ch8_normalized + 1.0) * 0.5 * (ATTITUDE_ROLL_MAX_DEG - ATTITUDE_ROLL_MIN_DEG);
        let raw_roll_setpoint_rad = raw_roll_deg * core::f32::consts::PI / 180.0;

        // Apply low-pass filter directly to raw setpoint (in radians)
        // This provides smooth transitions without double-damping
        self.filtered_pitch_setpoint_rad +=
            SETPOINT_FILTER_ALPHA * (raw_pitch_setpoint_rad - self.filtered_pitch_setpoint_rad);
        self.filtered_roll_setpoint_rad +=
            SETPOINT_FILTER_ALPHA * (raw_roll_setpoint_rad - self.filtered_roll_setpoint_rad);

        // Get pilot control inputs using ultra-fast LUT
        let pilot_inputs = ControlInputs::from_sbus_channels_fast(&packet.channels);

        // Store new attitude if provided
        if let Some(att) = attitude {
            self.last_attitude = Some(*att);
        }

        // Apply control mode logic
        let final_inputs = match self.current_control_mode {
            ControlMode::Manual => {
                // Full manual control - use pilot inputs directly
                debug!("Manual Mode - Direct pilot control");
                // Reset PID when in manual mode
                if self.attitude_controller.is_active() {
                    self.attitude_controller.reset();
                }
                pilot_inputs
            }

            ControlMode::Mixed => {
                // Mixed mode - blend pilot and autopilot
                if let Some(effective_attitude) = attitude.or(self.last_attitude.as_ref()) {
                    let gyro_rates = Some((
                        effective_attitude.roll_rate,
                        effective_attitude.pitch_rate,
                        effective_attitude.yaw_rate,
                    ));
                    let (pitch_correction, roll_correction) = self.attitude_controller.update(
                        self.filtered_pitch_setpoint_rad,
                        self.filtered_roll_setpoint_rad,
                        effective_attitude.pitch,
                        effective_attitude.roll,
                        gyro_rates,
                        Instant::now(),
                    );

                    // Blend pilot inputs with autopilot corrections
                    let mut mixed_inputs = pilot_inputs;
                    mixed_inputs.pitch = (1.0 - MIXED_MODE_AUTOPILOT_WEIGHT) * pilot_inputs.pitch
                        + MIXED_MODE_AUTOPILOT_WEIGHT * pitch_correction;
                    mixed_inputs.roll = (1.0 - MIXED_MODE_AUTOPILOT_WEIGHT) * pilot_inputs.roll
                        + MIXED_MODE_AUTOPILOT_WEIGHT * roll_correction;

                    debug!(
                        "Mixed Mode ({}% Auto) - P: {}°/{}° R: {}°/{}°",
                        (MIXED_MODE_AUTOPILOT_WEIGHT * 100.0) as i16,
                        (self.filtered_pitch_setpoint_rad * 180.0 / core::f32::consts::PI) as i16,
                        (effective_attitude.pitch * 180.0 / core::f32::consts::PI) as i16,
                        (self.filtered_roll_setpoint_rad * 180.0 / core::f32::consts::PI) as i16,
                        (effective_attitude.roll * 180.0 / core::f32::consts::PI) as i16
                    );

                    mixed_inputs
                } else {
                    // No attitude data - use pilot inputs only
                    pilot_inputs
                }
            }

            ControlMode::Autopilot => {
                // Full autopilot control - use attitude corrections only
                if let Some(effective_attitude) = attitude.or(self.last_attitude.as_ref()) {
                    let gyro_rates = Some((
                        effective_attitude.roll_rate,
                        effective_attitude.pitch_rate,
                        effective_attitude.yaw_rate,
                    ));
                    let (pitch_correction, roll_correction) = self.attitude_controller.update(
                        self.filtered_pitch_setpoint_rad,
                        self.filtered_roll_setpoint_rad,
                        effective_attitude.pitch,
                        effective_attitude.roll,
                        gyro_rates,
                        Instant::now(),
                    );

                    // Use autopilot corrections only, keep pilot throttle and yaw
                    let mut auto_inputs = pilot_inputs;
                    auto_inputs.pitch = pitch_correction;
                    auto_inputs.roll = roll_correction;

                    debug!(
                        "Autopilot Mode - P: {}°/{}° R: {}°/{}° Corrections: P:{} R:{}",
                        (self.filtered_pitch_setpoint_rad * 180.0 / core::f32::consts::PI) as i16,
                        (effective_attitude.pitch * 180.0 / core::f32::consts::PI) as i16,
                        (self.filtered_roll_setpoint_rad * 180.0 / core::f32::consts::PI) as i16,
                        (effective_attitude.roll * 180.0 / core::f32::consts::PI) as i16,
                        (pitch_correction * 100.0) as i16,
                        (roll_correction * 100.0) as i16
                    );

                    auto_inputs
                } else {
                    // No attitude data - fallback to manual
                    debug!("Autopilot Mode - No attitude data, using manual control");
                    pilot_inputs
                }
            }
        };

        // ULTRA-FAST PATH: Use direct LUT mixing for maximum performance
        if self.current_control_mode == ControlMode::Manual {
            // For manual mode, use direct SBUS to PWM conversion - fastest possible path
            let elevon_outputs = mix_elevons_direct_lut(&packet.channels);
            self.pwm
                .set_elevons_with_trim(elevon_outputs.left_us, elevon_outputs.right_us);

            // Combined throttle + differential calculation in single LUT call
            let (left_thrust, right_thrust) = if self.arming.armed {
                throttle_with_differential_lut(
                    packet.channels[THROTTLE_CH],
                    packet.channels[YAW_CH],
                )
            } else {
                (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
            };

            self.pwm.set_engines(left_thrust, right_thrust);
        } else {
            // For mixed/autopilot modes, use the corrected inputs
            let elevon_outputs = mix_elevons(&final_inputs);
            self.pwm
                .set_elevons_with_trim(elevon_outputs.left_us, elevon_outputs.right_us);

            // Calculate engine thrust with differential using LUT
            let base_thrust = throttle_curve_lut((final_inputs.throttle * 2047.0) as u16);
            let yaw_sbus = ((final_inputs.yaw * 1023.5) + 1023.5).clamp(0.0, 2047.0) as u16;

            let (left_thrust, right_thrust) = if self.arming.armed {
                apply_differential_thrust_direct(base_thrust, yaw_sbus)
            } else {
                (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
            };

            self.pwm.set_engines(left_thrust, right_thrust);
        }
    }

    /// Ultra-fast direct elevon control method using LUTs
    #[cfg(feature = "legacy-ctrl")]
    fn update_direct_elevons(&mut self, packet: &SbusPacket) {
        // Ultra-fast LUT lookups
        let elevon_left_us = sbus_to_pulse_lut(packet.channels[ELEVON_LEFT_CH]);
        let elevon_right_us = sbus_to_pulse_lut(packet.channels[ELEVON_RIGHT_CH]);

        // Set elevon positions
        self.pwm.set_elevons(elevon_left_us, elevon_right_us);

        // Combined throttle + differential in single LUT operation
        let (left_thrust, right_thrust) = if self.arming.armed {
            apply_differential_complete(
                throttle_curve_lut(packet.channels[ENGINE_CH]),
                packet.channels[DIFFERENTIAL_CH],
            )
        } else {
            (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
        };

        self.pwm.set_engines(left_thrust, right_thrust);

        info!(
            "Direct - LEFT: {}μs RIGHT: {}μs",
            elevon_left_us, elevon_right_us
        );
    }

    /// Updated method that uses mixing by default (legacy-ctrl disables it)
    pub fn update_with_attitude(&mut self, packet: &SbusPacket, attitude: Option<&AttitudeData>) {
        #[cfg(not(feature = "legacy-ctrl"))]
        {
            self.update_with_mixing(packet, attitude);
        }

        #[cfg(feature = "legacy-ctrl")]
        {
            // Legacy behavior - attitude is ignored in direct mode
            self.update(packet);
        }
    }

    pub fn check_failsafe(&mut self) {
        if self.last_packet_time.elapsed() > Duration::from_millis(SBUS_TIMEOUT_MS) {
            self.arming.signal_loss();
            self.attitude_controller.reset(); // Reset PID on signal loss
            self.apply_failsafe();
        }
    }

    pub fn apply_failsafe(&mut self) {
        self.pwm.set_safe_positions();
    }

    pub fn is_armed(&self) -> bool {
        self.arming.armed
    }

    pub fn is_failsafe(&self) -> bool {
        self.arming.failsafe_active
    }

    pub fn is_attitude_enabled(&self) -> bool {
        self.attitude_controller.enabled
    }

    pub fn current_control_mode(&self) -> ControlMode {
        self.current_control_mode
    }
}

// Helper function for LUT conversion (used in setpoint calculations)
#[inline(always)]
fn sbus_to_normalized_lut(sbus_value: u16) -> f32 {
    // Use roll center as default - can be optimized further with specific LUTs
    sbus_to_normalized_roll_lut(sbus_value)
}

/// Performance monitoring utilities for tracking task execution times
#[cfg(feature = "performance-monitoring")]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct TaskTiming {
    pub min_us: u32,
    pub max_us: u32,
    pub avg_us: u32,
    pub samples: u32,
}

#[cfg(feature = "performance-monitoring")]
impl Default for TaskTiming {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "performance-monitoring")]
impl TaskTiming {
    pub const fn new() -> Self {
        Self {
            min_us: u32::MAX,
            max_us: 0,
            avg_us: 0,
            samples: 0,
        }
    }

    pub fn update(&mut self, execution_time_us: u32) {
        self.min_us = self.min_us.min(execution_time_us);
        self.max_us = self.max_us.max(execution_time_us);

        // Running average calculation to avoid overflow
        if self.samples == 0 {
            self.avg_us = execution_time_us;
        } else {
            // Weighted average with more recent samples having slightly more weight
            let weight = if self.samples < 100 {
                self.samples + 1
            } else {
                100
            };
            self.avg_us = (self.avg_us * (weight - 1) + execution_time_us) / weight;
        }

        self.samples = self.samples.saturating_add(1);
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Get CPU utilization as percentage for a given target frequency
    pub fn cpu_utilization_percent(&self, target_frequency_hz: u32) -> f32 {
        let target_period_us = 1_000_000 / target_frequency_hz;
        (self.avg_us as f32 / target_period_us as f32) * 100.0
    }
}

/// Performance monitor for tracking multiple tasks
#[cfg(feature = "performance-monitoring")]
#[derive(Debug, defmt::Format)]
pub struct PerformanceMonitor {
    pub control_loop: TaskTiming,
    pub imu_update: TaskTiming,
    pub led_update: TaskTiming,
    pub flash_operation: TaskTiming,
}

#[cfg(feature = "performance-monitoring")]
impl Default for PerformanceMonitor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "performance-monitoring")]
impl PerformanceMonitor {
    pub const fn new() -> Self {
        Self {
            control_loop: TaskTiming::new(),
            imu_update: TaskTiming::new(),
            led_update: TaskTiming::new(),
            flash_operation: TaskTiming::new(),
        }
    }

    pub fn log_performance_summary(&self) {
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

        // Core 1 tasks (IMU only)
        let core1_imu_cpu = self.imu_update.cpu_utilization_percent(4000); // 4kHz actual rate
        info!("CORE 1 (IMU only): {}% total load", core1_imu_cpu as u8);
        info!(
            "  IMU: min={} avg={} max={}μs ({}% @ 4000Hz)",
            self.imu_update.min_us,
            self.imu_update.avg_us,
            self.imu_update.max_us,
            core1_imu_cpu as u8
        );

        // Flash operations (on-demand, Core 0)
        if self.flash_operation.samples > 0 {
            info!(
                "Flash Operations: min={} avg={} max={}μs | {} operations completed",
                self.flash_operation.min_us,
                self.flash_operation.avg_us,
                self.flash_operation.max_us,
                self.flash_operation.samples
            );
        }
    }

    pub fn reset_all(&mut self) {
        self.control_loop.reset();
        self.imu_update.reset();
        self.led_update.reset();
        self.flash_operation.reset();
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
pub fn update_imu_timing(elapsed_us: u32) {
    unsafe {
        (*core::ptr::addr_of_mut!(PERFORMANCE_MONITOR))
            .imu_update
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
pub fn update_flash_timing(elapsed_us: u32) {
    unsafe {
        (*core::ptr::addr_of_mut!(PERFORMANCE_MONITOR))
            .flash_operation
            .update(elapsed_us);
    }
}

#[cfg(feature = "performance-monitoring")]
pub fn log_performance_summary() {
    unsafe {
        (*core::ptr::addr_of!(PERFORMANCE_MONITOR)).log_performance_summary();
    }
}

#[cfg(feature = "performance-monitoring")]
pub fn debug_timing_test() {
    let timer = TimingMeasurement::start();
    // Do a tiny bit of work to test timing precision
    let mut x = 0u32;
    for _ in 0..100 {
        x = x.wrapping_add(1);
    }
    let elapsed = timer.elapsed_us();
    info!("DEBUG: Timing test took {}μs (x={})", elapsed, x);
}

// No-op stubs when performance monitoring is disabled
#[cfg(not(feature = "performance-monitoring"))]
#[inline(always)]
pub fn update_control_loop_timing(_elapsed_us: u32) {}

#[cfg(not(feature = "performance-monitoring"))]
#[inline(always)]
pub fn update_imu_timing(_elapsed_us: u32) {}

#[cfg(not(feature = "performance-monitoring"))]
#[inline(always)]
pub fn update_led_timing(_elapsed_us: u32) {}

#[cfg(not(feature = "performance-monitoring"))]
#[inline(always)]
pub fn update_flash_timing(_elapsed_us: u32) {}

#[cfg(not(feature = "performance-monitoring"))]
#[inline(always)]
pub fn log_performance_summary() {}

// Dummy timing measurement when feature is disabled
#[cfg(not(feature = "performance-monitoring"))]
pub struct TimingMeasurement;

#[cfg(not(feature = "performance-monitoring"))]
impl TimingMeasurement {
    #[inline(always)]
    pub fn start() -> Self {
        Self
    }

    #[inline(always)]
    pub fn elapsed_us(&self) -> u32 {
        0
    }
}

pub static SUP_LED_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static SUP_IMU_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static SUP_FC_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
#[cfg(feature = "rtt-control")]
pub static SUP_RTT_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// Start signals to release tasks from the barrier
pub static SUP_START_IMU: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static SUP_START_FC: Signal<CriticalSectionRawMutex, ()> = Signal::new();
#[cfg(feature = "rtt-control")]
pub static SUP_START_RTT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Supervisor task that waits for all participants to be ready, then releases them simultaneously
#[embassy_executor::task]
pub async fn supervisor_task() {
    info!("Supervisor: waiting for tasks to initialize");

    // Prepare futures for all participants and wait for them concurrently
    let led_ready = SUP_LED_READY.wait();
    let imu_ready = SUP_IMU_READY.wait();
    let fc_ready = SUP_FC_READY.wait();

    #[cfg(not(feature = "rtt-control"))]
    {
        let _ = embassy_futures::join::join3(led_ready, imu_ready, fc_ready).await;
    }

    #[cfg(feature = "rtt-control")]
    {
        let rtt_ready = SUP_RTT_READY.wait();
        let _ = embassy_futures::join::join4(led_ready, imu_ready, fc_ready, rtt_ready).await;
    }

    info!("Supervisor: all tasks initialized, releasing start barrier");

    // Release participants. Use per-task start signals instead of broadcast.
    SUP_START_IMU.signal(());
    SUP_START_FC.signal(());
    #[cfg(feature = "rtt-control")]
    {
        SUP_START_RTT.signal(());
    }
}
