#![no_std]
#![no_main]

//! Firmware for the XFly Eagle Testbed with WS2812B LED

#[cfg(feature = "defmt-logging")]
use defmt_rtt as _;

use defmt::{info, warn};
use elle_config::profile::FLASH_SIZE;
use elle_config::{
    CONTROL_LOOP_FREQUENCY_HZ, CONTROL_LOOP_PERIOD_MS, IMU_CALIBRATION_TIMEOUT_S, IMU_I2C_FREQ, IMU_MAX_AGE_MS,
};

#[cfg(not(feature = "rtt-control"))]
use defmt::debug;
#[cfg(not(feature = "rtt-control"))]
use elle_config::{
    ATTITUDE_ENABLE_CH, ATTITUDE_PITCH_SETPOINT_CH, ATTITUDE_ROLL_SETPOINT_CH, PITCH_CH, ROLL_CH,
    THROTTLE_CH, YAW_CH,
};
#[cfg(not(feature = "rtt-control"))]
use elle_control::commands::PilotCommands;
use elle_hardware::imu::{
    ATTITUDE_SIGNAL, AttitudeData, BnoImu, IMU_STATUS, LED_COMMAND_CHANNEL, is_attitude_valid,
};
use elle_hardware::led::{LedPattern, StatusLed, colors};
use elle_hardware::{
    pwm::{PwmOutputs, PwmPins},
    sequential_flash_manager::SequentialFlashManager,
};

#[cfg(not(feature = "rtt-control"))]
use elle_hardware::sbus::{SBUS_COMMANDS, SbusReceiver, sbus_receiver_task};
#[cfg(feature = "rtt-control")]
use elle_system::rtt_control::{COMMAND_CHANNEL, DebugCommand, RttCommander, RttControl};
#[cfg(feature = "rtt-control")]
use elle_system::{SUP_RTT_READY, SUP_START_RTT};
#[cfg(feature = "performance-monitoring")]
use elle_system::{
    TimingMeasurement, log_performance_summary, update_control_loop_timing, update_led_timing,
};

use elle_system::{
    FlightController, SUP_FC_READY, SUP_IMU_READY, SUP_LED_READY, SUP_START_FC, SUP_START_IMU,
    supervisor_task,
};

// Dummy timing when performance monitoring is disabled
#[cfg(not(feature = "performance-monitoring"))]
struct TimingMeasurement;

#[cfg(not(feature = "performance-monitoring"))]
impl TimingMeasurement {
    fn start() -> Self {
        Self
    }
    fn elapsed_us(&self) -> u32 {
        0
    }
}

#[cfg(not(feature = "performance-monitoring"))]
fn update_control_loop_timing(_elapsed_us: u32) {}

#[cfg(not(feature = "performance-monitoring"))]
fn update_led_timing(_elapsed_us: u32) {}

#[cfg(not(feature = "performance-monitoring"))]
fn log_performance_summary() {}
use embassy_executor::{Executor, Spawner};
use embassy_rp::clocks::{ClockConfig, CoreVoltage};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::i2c::{Config, I2c};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH2, FLASH, I2C0, PIN_8, PIN_9, PIN_10, PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler as PioIrqHandler, Pio};
#[cfg(not(feature = "rtt-control"))]
use embassy_rp::uart::InterruptHandler as UartIrqHandler;
use embassy_rp::watchdog::Watchdog;
use embassy_rp::{Peri, bind_interrupts};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_time::{Duration, Ticker, Timer};
use panic_probe as _;
use static_cell::StaticCell;

/// Helper to validate attitude data and return only if fresh
#[inline]
fn validate_attitude(attitude: Option<AttitudeData>) -> Option<AttitudeData> {
    attitude.filter(|att| is_attitude_valid(att, Duration::from_millis(IMU_MAX_AGE_MS)))
}

bind_interrupts!(
    struct Irqs {
        PIO0_IRQ_0 => PioIrqHandler<PIO0>;
        PIO1_IRQ_0 => PioIrqHandler<PIO1>;
        #[cfg(not(feature = "rtt-control"))]
        UART0_IRQ => UartIrqHandler<UART0>;
    }
);

static mut CORE1_STACK: Stack<8192> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config =
        embassy_rp::config::Config::new(ClockConfig::system_freq(200_000_000).unwrap());
    config.clocks.core_voltage = CoreVoltage::V1_15;

    let p = embassy_rp::init(config);

    info!("Core0: Starting flash manager");
    // Create flash manager on Core 0 before spawning Core 1
    let flash = embassy_rp::flash::Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH1);
    // Small delay to let debug probe settle
    Timer::after_millis(10).await;
    spawner.spawn(flash_manager_task(flash).unwrap());

    // Setup WS2812B LED on PIO1 (separate from PWM on PIO0)
    info!("Core0: Setting up status LED");
    let Pio {
        common: led_common,
        sm0: led_sm0,
        ..
    } = Pio::new(p.PIO1, Irqs);

    spawner.spawn(led_task(led_common, led_sm0, p.DMA_CH2, p.PIN_10).unwrap());

    #[cfg(feature = "rtt-control")]
    {
        info!("Core0: Starting RTT control interface");
        spawner.spawn(rtt_control_task().unwrap());
    }

    // Start supervisor to coordinate task startup
    info!("Core0: Spawning Supervisor");
    spawner.spawn(supervisor_task().unwrap());

    info!("Core0: Spawning Core1 for IMU");
    // Spawn IMU task on core1
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.spawn(imu_task(spawner, p.I2C0, p.PIN_8, p.PIN_9).unwrap());
            });
        },
    );

    info!("Core0: Setting up flight control hardware");
    // Core0: Setup flight control hardware (PWM on PIO0)
    let mut pwm_pins = PwmPins {
        elevon_left: p.PIN_12,
        elevon_right: p.PIN_14,
        engine_left: p.PIN_11,
        engine_right: p.PIN_15,
    };

    let Pio {
        mut common,
        sm0,
        sm1,
        sm2,
        sm3,
        ..
    } = Pio::new(p.PIO0, Irqs);
    let mut pwm = PwmOutputs::new(&mut common, sm0, sm1, sm2, sm3, &mut pwm_pins);
    pwm.set_safe_positions();

    #[cfg(not(feature = "rtt-control"))]
    {
        info!("Core0: Starting SBUS receiver task");
        let sbus = SbusReceiver::new(p.UART0, p.PIN_13, Irqs, p.DMA_CH0);
        spawner.spawn(sbus_receiver_task(sbus).unwrap());
    }
    let mut fc = FlightController::new(pwm);

    // Wait for IMU to be ready
    info!("Core0: Waiting for IMU initialization...");
    let mut wait_counter = 0u32;
    loop {
        let status = IMU_STATUS.read().await;
        if status.initialized {
            info!("Core0: IMU ready! Calibrated: {}", status.calibrated);
            // Signal LED to show system ready
            let _ = LED_COMMAND_CHANNEL.try_send(if status.calibrated {
                LedPattern::Solid(colors::GREEN)
            } else {
                LedPattern::Pulse(colors::CYAN)
            });
            break;
        }
        drop(status);
        wait_counter += 1;
        if wait_counter.is_multiple_of(50) {
            // Log every 5 seconds
            warn!("Core0: Still waiting for IMU... ({}s)", wait_counter / 10);
        }
        Timer::after(Duration::from_millis(100)).await;
    }

    // Initialize ESCs before entering synchronized start
    info!("Core0: Initializing ESCs");
    fc.initialize_escs().await;

    // Initialize supervisor components (but keep health monitoring disabled)
    info!("Core0: Initializing supervisor (watchdog only)");
    let watchdog = Watchdog::new(p.WATCHDOG);
    fc.initialize_supervisor(watchdog);

    // Signal supervisor that Flight Controller is ready and wait for start
    SUP_FC_READY.signal(());
    info!("Core0: Waiting for Supervisor start barrier");
    SUP_START_FC.wait().await;

    // Now enable health monitoring after all tasks are ready
    fc.enable_supervisor_monitoring();

    // Update LED based on IMU calibration status at start
    let status = IMU_STATUS.read().await;
    let _ = LED_COMMAND_CHANNEL.try_send(if status.calibrated {
        LedPattern::Solid(colors::GREEN)
    } else {
        LedPattern::Pulse(colors::CYAN)
    });
    drop(status);

    info!("Core0: Starting main control loop");

    // Test timing precision (only when performance monitoring is enabled)
    #[cfg(feature = "performance-monitoring")]
    {
        let test_timer = TimingMeasurement::start();
        let mut test_var = 0u32;
        for _ in 0..1000 {
            test_var = test_var.wrapping_add(1);
        }
        info!(
            "Timing test: {}μs for 1000 additions (result: {})",
            test_timer.elapsed_us(),
            test_var
        );
    }

    // State for control loop
    let mut loop_counter = 0u32;

    #[cfg(not(feature = "rtt-control"))]
    {
        info!("FLIGHT MODE - SBUS Control");

        // Create ticker for precise 13ms periods (77Hz)
        let mut ticker = Ticker::every(Duration::from_millis(CONTROL_LOOP_PERIOD_MS));

        // Track last commands for consistent update rate
        let mut last_commands: Option<PilotCommands> = None;

        loop {
            ticker.next().await; // Wait for next tick BEFORE processing
            let loop_timer = TimingMeasurement::start();

            // Supervisor check - monitor core health and kick watchdog
            let _supervisor_healthy = fc.supervisor_check();

            // Get latest attitude data (non-blocking)
            let attitude = ATTITUDE_SIGNAL.try_take();

            // Check for latest SBUS commands from dedicated receiver task (non-blocking)
            // The SBUS task runs independently and updates this signal when packets arrive
            if let Some(commands) = SBUS_COMMANDS.try_take() {
                // Debug logging (~8Hz)
                if loop_counter.is_multiple_of(CONTROL_LOOP_FREQUENCY_HZ / 10)
                    && let PilotCommands::Raw(raw) = &commands
                {
                    debug!(
                        "SBUS: CH1:{} CH2:{} CH3:{} CH4:{} CH5:{} CH6:{} CH8:{}",
                        raw.channels[ROLL_CH],
                        raw.channels[PITCH_CH],
                        raw.channels[THROTTLE_CH],
                        raw.channels[YAW_CH],
                        raw.channels[ATTITUDE_ENABLE_CH],
                        raw.channels[ATTITUDE_PITCH_SETPOINT_CH],
                        raw.channels[ATTITUDE_ROLL_SETPOINT_CH]
                    );
                }

                last_commands = Some(commands);
            }

            // Always update flight controller at 13ms intervals for consistent PID timing
            // Use last known commands if no new packet arrived this iteration
            if let Some(commands) = &last_commands {
                // Update with validated attitude (warns if stale)
                let valid_attitude = validate_attitude(attitude);
                if valid_attitude.is_none() && attitude.is_some() {
                    warn!("Stale attitude data, using manual control only");
                }
                fc.update(commands, valid_attitude.as_ref());
            }

            // Check for failsafe (triggers after 300ms of no valid packets)
            fc.check_failsafe();

            update_control_loop_timing(loop_timer.elapsed_us());
            loop_counter = loop_counter.saturating_add(1);

            // Periodic status updates
            if loop_counter.is_multiple_of(CONTROL_LOOP_FREQUENCY_HZ * 10) {
                log_performance_summary();
            }

            if loop_counter.is_multiple_of(20000) {
                loop_counter = 0;

                let imu_status = IMU_STATUS.read().await;

                let led_pattern = if fc.is_armed() {
                    if fc.is_attitude_enabled() {
                        LedPattern::Pulse(colors::CYAN)
                    } else {
                        LedPattern::DoubleBlink(colors::GREEN)
                    }
                } else if fc.is_failsafe() {
                    LedPattern::RapidFlash(colors::ORANGE)
                } else if imu_status.calibrated {
                    LedPattern::Solid(colors::GREEN)
                } else {
                    LedPattern::Pulse(colors::CYAN)
                };

                let _ = LED_COMMAND_CHANNEL.try_send(led_pattern);
                drop(imu_status);
            }
        }
    }

    #[cfg(feature = "rtt-control")]
    {
        info!("GROUND TEST MODE - RTT Control");
        info!("WARNING: This mode requires programmer connection");

        let mut rtt_commander = RttCommander::new(COMMAND_CHANNEL.receiver());
        let mut last_had_commands = false;

        loop {
            let loop_timer = TimingMeasurement::start();

            // Supervisor check
            let _supervisor_healthy = fc.supervisor_check();

            // Get latest attitude data
            let attitude = ATTITUDE_SIGNAL.try_take();

            // Collect debug commands (up to 16 per loop iteration)
            let mut debug_commands: [Option<DebugCommand>; 16] = [None; 16];
            let mut debug_count = 0;

            // Read commands from RTT (processes both flight and debug commands)
            let pilot_commands = rtt_commander
                .read_commands(|cmd| {
                    if debug_count < debug_commands.len() {
                        debug_commands[debug_count] = Some(cmd);
                        debug_count += 1;
                    }
                })
                .await;

            // Process collected debug commands
            for cmd in debug_commands[..debug_count].iter().flatten() {
                process_debug_command(&mut fc, *cmd).await;
            }

            // Apply pilot commands or failsafe
            let has_commands = pilot_commands.is_some();
            if let Some(commands) = pilot_commands {
                if !last_had_commands {
                    info!("RTT commands active");
                }
                fc.update(&commands, validate_attitude(attitude).as_ref());
            } else {
                // RTT timed out - apply failsafe
                if last_had_commands {
                    warn!("RTT command timeout - applying failsafe");
                }
                fc.apply_failsafe();
            }
            last_had_commands = has_commands;

            // Don't check SBUS failsafe in RTT mode - RTT has its own timeout logic above

            update_control_loop_timing(loop_timer.elapsed_us());
            loop_counter = loop_counter.saturating_add(1);

            if loop_counter.is_multiple_of(CONTROL_LOOP_FREQUENCY_HZ * 10) {
                log_performance_summary();
            }

            if loop_counter.is_multiple_of(2000) {
                loop_counter = 0;

                let imu_status = IMU_STATUS.read().await;
                let led_pattern = if fc.is_armed() {
                    LedPattern::DoubleBlink(colors::CYAN)
                } else if fc.is_failsafe() {
                    LedPattern::RapidFlash(colors::ORANGE)
                } else if imu_status.calibrated {
                    LedPattern::Solid(colors::BLUE)
                } else {
                    LedPattern::Pulse(colors::BLUE)
                };

                let _ = LED_COMMAND_CHANNEL.try_send(led_pattern);
                drop(imu_status);
            }

            // Small delay for RTT command processing
            Timer::after(Duration::from_millis(10)).await;
        }
    }
}

#[embassy_executor::task]
async fn imu_task(
    _spawner: Spawner,
    i2c: Peri<'static, I2C0>,
    sda: Peri<'static, PIN_8>,
    scl: Peri<'static, PIN_9>,
) {
    info!("Core1: IMU task starting with flash calibration support");

    let mut i2c_config = Config::default();
    i2c_config.frequency = IMU_I2C_FREQ;

    let i2c_bus = I2c::new_blocking(i2c, scl, sda, i2c_config);
    let led_sender = LED_COMMAND_CHANNEL.sender();
    let mut imu = BnoImu::new(i2c_bus, led_sender);

    // Initialize with flash calibration support
    match imu.initialize().await {
        Ok(_) => info!("Core1: IMU initialized"),
        Err(e) => {
            defmt::panic!("Core1: IMU init failed: {}", e);
        }
    }

    // Notify supervisor that IMU is initialized
    SUP_IMU_READY.signal(());

    // Calibration wait (may be shorter if loaded from flash)
    if let Err(e) = imu.wait_for_calibration(IMU_CALIBRATION_TIMEOUT_S).await {
        warn!("Core1: IMU calibration incomplete: {}", e);
    }

    // Wait for supervisor start before entering main IMU run loop
    info!("Core1: Waiting for Supervisor start barrier");
    SUP_START_IMU.wait().await;

    // Run continuous IMU reading
    imu.run().await;
}

#[embassy_executor::task]
async fn flash_manager_task(flash: Flash<'static, FLASH, Async, { FLASH_SIZE }>) {
    info!("Core0: Flash manager task starting");
    let mut manager = SequentialFlashManager::new(flash);
    manager.run().await;
}

#[embassy_executor::task]
async fn led_task(
    mut common: embassy_rp::pio::Common<'static, PIO1>,
    sm0: embassy_rp::pio::StateMachine<'static, PIO1, 0>,
    dma: Peri<'static, DMA_CH2>,
    pin: Peri<'static, PIN_10>,
) {
    info!("Core0: LED task starting");

    let mut led = StatusLed::new(&mut common, sm0, pin, dma);
    let receiver: Receiver<'static, CriticalSectionRawMutex, LedPattern, 8> =
        LED_COMMAND_CHANNEL.receiver();

    // Set initial pattern
    led.set_pattern(LedPattern::SlowBlink(colors::BLUE)).await;

    // Notify supervisor that LED task is initialized
    SUP_LED_READY.signal(());

    // Main LED update loop
    loop {
        let led_timer = TimingMeasurement::start();

        // Check for new patterns
        if let Ok(pattern) = receiver.try_receive() {
            led.set_pattern(pattern).await;
        }

        // Update LED animation
        led.update().await;

        // Update performance metrics
        update_led_timing(led_timer.elapsed_us());

        // Small delay for animation timing
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[cfg(feature = "rtt-control")]
#[embassy_executor::task]
async fn rtt_control_task() {
    info!("Core0: RTT control task starting");
    let mut rtt_control = RttControl::init();

    // Notify supervisor that RTT control is ready, then wait for start
    SUP_RTT_READY.signal(());
    info!("Core0: RTT control waiting for Supervisor start barrier");
    SUP_START_RTT.wait().await;

    rtt_control.run().await;
}

/// Process debug commands from RTT (non-flight commands only)
/// Flight commands (throttle, elevons, mode) are handled by RttCommander
#[cfg(feature = "rtt-control")]
async fn process_debug_command(fc: &mut FlightController<'_>, command: DebugCommand) {
    use elle_hardware::imu::IMU_STATUS;

    match command {
        // Flight control commands handled by RttCommander
        DebugCommand::SetThrottle(_)
        | DebugCommand::SetElevons { .. }
        | DebugCommand::SetControlMode(_) => {
            // No-op - handled by RttCommander
        }

        DebugCommand::Arm => {
            fc.arm();
            info!("Motors ARMED via RTT");
        }

        DebugCommand::Disarm => {
            fc.disarm();
            info!("Motors DISARMED via RTT");
        }

        DebugCommand::EmergencyStop => {
            info!("RTT: EMERGENCY STOP");
            fc.apply_failsafe();
        }

        DebugCommand::AdjustTrim { left, right } => {
            info!(
                "RTT: Trim adjustment requested L={} R={} (not implemented)",
                left, right
            );
        }

        DebugCommand::SaveCalibration => {
            info!("RTT: Save calibration requested");
        }

        DebugCommand::ClearCalibration => {
            info!("RTT: Clear calibration requested");
        }

        DebugCommand::GetStatus => {
            let status = IMU_STATUS.read().await;
            info!(
                "RTT Status - Armed: {}, Failsafe: {}, IMU Cal: {}, IMU Errors: {}",
                fc.is_armed(),
                fc.is_failsafe(),
                status.calibrated,
                status.error_count
            );
        }

        DebugCommand::GetAttitude => {
            if let Some(attitude) = ATTITUDE_SIGNAL.try_take() {
                info!(
                    "RTT Attitude - P:{}° R:{}° Y:{}°",
                    (attitude.pitch * 180.0 / core::f32::consts::PI) as i16,
                    (attitude.roll * 180.0 / core::f32::consts::PI) as i16,
                    (attitude.yaw * 180.0 / core::f32::consts::PI) as i16
                );
                // Put it back for normal processing
                ATTITUDE_SIGNAL.signal(attitude);
            } else {
                info!("RTT: No attitude data available");
            }
        }

        #[cfg(feature = "performance-monitoring")]
        DebugCommand::GetPerformance => {
            use elle_system::log_performance_summary;
            info!("RTT: Performance summary requested");
            log_performance_summary();
        }

        #[cfg(feature = "performance-monitoring")]
        DebugCommand::ResetPerformance => {
            use elle_system::PERFORMANCE_MONITOR;
            info!("RTT: Resetting performance counters");
            unsafe {
                (*core::ptr::addr_of_mut!(PERFORMANCE_MONITOR)).reset_all();
            }
        }
    }
}
