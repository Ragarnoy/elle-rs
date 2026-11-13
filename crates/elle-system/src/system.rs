use defmt::{info, warn};
use elle_config::*;
use elle_control::SavedGains;
use elle_control::commands::PilotCommands;
use elle_control::flight_law::{FlightLaw, LawInputs};
use elle_control::pid::PidConfig;
use elle_hardware::event::{EVT_RC_RESTORED, EVT_RC_SIGNAL_LOST, EVT_RC_WARNING};
use elle_hardware::imu::{AttitudeData, CORE1_HEARTBEAT};
use elle_hardware::pwm::PwmOutputs;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant};

pub use elle_control::flight_law::{ControlMode, ControllerOutputSnapshot};

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

/// The flight law (`elle_control::flight_law`) plus what it can't do on the
/// host: the elevon PWM, the clock, events, the watchdog and supervisor, and
/// the RC link state machine.
pub struct FlightController<'a> {
    pwm: PwmOutputs<'a>,
    law: FlightLaw,
    last_packet_time: Instant,
    /// When `update` last ran, for the measured dt.
    last_update_at: Option<Instant>,
    // Controller output snapshot for observability
    last_output: ControllerOutputSnapshot,
    // Supervisor components
    core1_health: CoreHealth,
    last_watchdog_kick: Instant,
    supervisor_enabled: bool,
    // RC link state machine
    rc_link_state: RcLinkState,
}

impl<'a> FlightController<'a> {
    #[must_use]
    pub fn new(pwm: PwmOutputs<'a>) -> Self {
        Self {
            pwm,
            law: FlightLaw::new(),
            last_packet_time: Instant::now(),
            last_update_at: None,
            last_output: ControllerOutputSnapshot::default(),
            core1_health: CoreHealth::default(),
            last_watchdog_kick: Instant::now(),
            supervisor_enabled: false,
            rc_link_state: RcLinkState::Ok,
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
                self.law.disable_attitude();
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
            self.law.disable_attitude();
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

    /// Main update method - accepts PilotCommands from any source. One tick of
    /// the flight law, then its elevon write and events.
    pub fn update(&mut self, commands: &PilotCommands, attitude: Option<&AttitudeData>) {
        self.last_packet_time = commands.timestamp();

        let now = Instant::now();
        let dt_us = self
            .last_update_at
            .map_or(0, |t| now.saturating_duration_since(t).as_micros() as u32);
        self.last_update_at = Some(now);

        let out = self.law.step(
            &LawInputs {
                now,
                commands,
                attitude,
                core1_healthy: self.core1_health.is_healthy,
            },
            CONTROL_LOOP_DT,
        );

        if let Some((from, to)) = out.events.mode_change {
            info!("Control mode changed: {:?} -> {:?}", from, to);
        }
        if let Some(age_ms) = out.events.attitude_held_ms {
            elle_hardware::elle_event!(
                warn,
                elle_hardware::event::EVT_ATTITUDE_HELD,
                "No new attitude for 2 ticks; holding a {} ms old sample",
                age_ms
            );
        }
        if let Some(age_ms) = out.events.attitude_stale_ms {
            elle_hardware::elle_event!(
                warn,
                elle_hardware::event::EVT_ATTITUDE_STALE,
                "Held attitude expired ({} ms); attitude control off until IMU returns",
                age_ms
            );
        }

        self.last_output = out.write(&mut self.pwm, dt_us);
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
                self.law.signal_restored();
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
                self.law.signal_loss();
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

    #[must_use]
    pub const fn is_armed(&self) -> bool {
        self.law.is_armed()
    }

    #[must_use]
    pub const fn is_failsafe(&self) -> bool {
        self.law.is_failsafe()
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
        self.law.is_attitude_enabled()
    }

    #[must_use]
    pub const fn current_control_mode(&self) -> ControlMode {
        self.law.current_control_mode()
    }

    /// Disable throttle-low auto-arming (RPC mode uses explicit arm/disarm only)
    pub fn set_explicit_arming(&mut self, enabled: bool) {
        self.law.set_explicit_arming(enabled);
    }

    /// Manual arm (for RTT/debug control). Returns false when arming is locked
    /// (ESC-config boot, or a mode switch pending); the caller reports it.
    #[must_use]
    pub fn arm(&mut self) -> bool {
        self.law.arm()
    }

    /// Refuse every arm: for the rest of an ESC-config boot (no engine output
    /// path), or while a boot-mode switch is pending. Covers the explicit arm
    /// and the throttle gesture.
    pub fn lock_arming(&mut self) {
        self.law.lock_arming();
    }

    /// Undo [`lock_arming`](Self::lock_arming) after a cancelled boot-mode
    /// switch (normal boots only).
    pub fn unlock_arming(&mut self) {
        self.law.unlock_arming();
    }

    /// True once after the throttle gesture was refused by [`lock_arming`](Self::lock_arming).
    pub fn take_arm_refused(&mut self) -> bool {
        self.law.take_arm_refused()
    }

    /// Manual disarm (for RTT/debug control)
    pub fn disarm(&mut self) {
        self.law.disarm();
    }

    /// Force elevons to center (safe position). Used by kill switch.
    pub fn set_safe_positions(&mut self) {
        self.pwm.set_safe_positions();
    }

    /// Update PID gains at runtime (resets integral state).
    pub fn set_pid_gains(&mut self, config: PidConfig) {
        self.law.set_pid_gains(config);
    }

    /// Active PID gains.
    #[must_use]
    pub const fn pid_config(&self) -> &PidConfig {
        self.law.pid_config()
    }

    /// Changes on every gain update; compare to detect new gains.
    #[must_use]
    pub const fn gains_version(&self) -> u32 {
        self.law.gains_version()
    }

    /// Apply PID gains from a `SavedGains` snapshot (used by autotuner).
    pub fn apply_saved_gains(&mut self, gains: &SavedGains) {
        self.set_pid_gains((*gains).into());
    }

    /// Get current PID gains as a `SavedGains` snapshot.
    #[must_use]
    pub fn get_pid_gains(&self) -> SavedGains {
        (*self.law.pid_config()).into()
    }

    /// Set attitude setpoint override (degrees). Used by autotuner.
    pub fn set_setpoint_override(&mut self, pitch_deg: f32, roll_deg: f32) {
        self.law.set_setpoint_override(pitch_deg, roll_deg);
    }

    /// Clear setpoint override, returning to normal RC/RPC control.
    pub fn clear_setpoint_override(&mut self) {
        self.law.clear_setpoint_override();
    }

    /// Engage heading-hold with the given target heading (AHRS yaw, radians).
    /// Only takes effect while `AttitudeMode::Stabilized` is selected.
    pub fn engage_heading_hold(&mut self, target_rad: f32) {
        self.law.engage_heading_hold(target_rad);
    }

    /// Disengage heading-hold, returning roll to stick-derived control.
    pub fn disengage_heading_hold(&mut self) {
        self.law.disengage_heading_hold();
    }

    #[must_use]
    pub const fn is_heading_hold_active(&self) -> bool {
        self.law.is_heading_hold_active()
    }

    /// Get the last controller output snapshot
    /// When the elevon PWM next latches its pulses (see `PwmOutputs::next_wrap`).
    #[must_use]
    pub fn next_pwm_wrap(&self) -> Instant {
        self.pwm.next_wrap()
    }

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
