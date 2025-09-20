#![no_std]
#![no_main]

//! Firmware for the XFly Eagle Testbed with WS2812B LED

#[cfg(feature = "defmt-logging")]
use defmt_rtt as _;

use defmt::{debug, info, warn};
use elle::config::profile::FLASH_SIZE;
use elle::config::{
    ATTITUDE_ENABLE_CH, ATTITUDE_PITCH_SETPOINT_CH, ATTITUDE_ROLL_SETPOINT_CH,
    CONTROL_LOOP_FREQUENCY_HZ, IMU_CALIBRATION_TIMEOUT_S, IMU_I2C_FREQ, IMU_MAX_AGE_MS, PITCH_CH,
    ROLL_CH, THROTTLE_CH, YAW_CH,
};
use elle::hardware::imu::{
    ATTITUDE_SIGNAL, BnoImu, IMU_STATUS, LED_COMMAND_CHANNEL, is_attitude_valid,
};
use elle::hardware::led::{LedPattern, StatusLed, colors};
use elle::hardware::{
    flash_manager::FlashManager,
    pwm::{PwmOutputs, PwmPins},
    sbus::SbusReceiver,
};
#[cfg(feature = "rtt-control")]
use elle::rtt_control::{COMMAND_CHANNEL, DebugCommand, RttControl};
#[cfg(feature = "performance-monitoring")]
use elle::system::{
    TimingMeasurement, log_performance_summary, update_control_loop_timing, update_led_timing,
};

use elle::system::FlightController;

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
use embassy_rp::peripherals::{DMA_CH2, FLASH, I2C0, PIN_8, PIN_9, PIN_10, PIO0, PIO1, UART0};
use embassy_rp::pio::{InterruptHandler as PioIrqHandler, Pio};
use embassy_rp::uart::InterruptHandler as UartIrqHandler;
use embassy_rp::{Peri, bind_interrupts};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_time::{Duration, Timer};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(
    struct Irqs {
        PIO0_IRQ_0 => PioIrqHandler<PIO0>;
        PIO1_IRQ_0 => PioIrqHandler<PIO1>;
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
    spawner.spawn(flash_manager_task(flash)).unwrap();

    // Setup WS2812B LED on PIO1 (separate from PWM on PIO0)
    info!("Core0: Setting up status LED");
    let Pio {
        common: led_common,
        sm0: led_sm0,
        ..
    } = Pio::new(p.PIO1, Irqs);

    spawner
        .spawn(led_task(led_common, led_sm0, p.DMA_CH2, p.PIN_10))
        .unwrap();

    #[cfg(feature = "rtt-control")]
    {
        info!("Core0: Starting RTT control interface");
        spawner.spawn(rtt_control_task()).unwrap();
    }

    info!("Core0: Spawning Core1 for IMU");
    // Spawn IMU task on core1
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner
                    .spawn(imu_task(spawner, p.I2C0, p.PIN_8, p.PIN_9))
                    .unwrap();
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

    let mut sbus = SbusReceiver::new(p.UART0, p.PIN_13, Irqs, p.DMA_CH0);
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

    info!("Core0: Initializing ESCs");
    fc.initialize_escs().await;

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

    loop {
        let loop_timer = TimingMeasurement::start();

        // Process RTT debug commands (non-blocking)
        #[cfg(feature = "rtt-control")]
        if let Ok(command) = COMMAND_CHANNEL.try_receive() {
            process_debug_command(&mut fc, command).await;
        }

        // Get latest attitude data (non-blocking)
        let attitude = ATTITUDE_SIGNAL.try_take();

        // Read SBUS packet with timeout appropriate for ~77Hz rate
        if let Some(packet) = sbus.read_packet().await {
            // Print debug info less frequently
            if loop_counter.is_multiple_of(CONTROL_LOOP_FREQUENCY_HZ / 10) {
                // ~8Hz logging
                debug!(
                    "SBUS: CH1:{} CH2:{} CH3:{} CH4:{} CH5:{} CH6:{} CH8:{}",
                    packet.channels[ROLL_CH],
                    packet.channels[PITCH_CH],
                    packet.channels[THROTTLE_CH],
                    packet.channels[YAW_CH],
                    packet.channels[ATTITUDE_ENABLE_CH],
                    packet.channels[ATTITUDE_PITCH_SETPOINT_CH],
                    packet.channels[ATTITUDE_ROLL_SETPOINT_CH]
                );
            }

            // Pass attitude data to flight controller
            if let Some(att) = attitude {
                if is_attitude_valid(&att, Duration::from_millis(IMU_MAX_AGE_MS)) {
                    fc.update_with_attitude(&packet, Some(&att));
                } else {
                    defmt::warn!("Stale attitude data, using manual control only");
                    fc.update(&packet);
                }
            } else {
                fc.update(&packet);
            }
            // Always check failsafe
            fc.check_failsafe();
        } else {
            // No SBUS packet - still do failsafe check
            fc.check_failsafe();
        }

        // Update performance metrics for this loop iteration
        update_control_loop_timing(loop_timer.elapsed_us());

        // Status output and LED updates
        loop_counter = loop_counter.saturating_add(1);

        // Periodic status updates
        if loop_counter.is_multiple_of(CONTROL_LOOP_FREQUENCY_HZ * 10) {
            // Every 10 seconds - show performance summary
            log_performance_summary();
        }

        if loop_counter.is_multiple_of(20000) {
            // Every ~100 seconds at 200Hz
            loop_counter = 0;

            // Get IMU status
            let imu_status = IMU_STATUS.read().await;

            // Update LED based on system state
            let led_pattern = if fc.is_armed() {
                if fc.is_attitude_enabled() {
                    LedPattern::Pulse(colors::CYAN) // Attitude hold active
                } else {
                    LedPattern::DoubleBlink(colors::GREEN) // Armed, manual
                }
            } else if fc.is_failsafe() {
                LedPattern::RapidFlash(colors::ORANGE) // Failsafe
            } else if imu_status.calibrated {
                LedPattern::Solid(colors::GREEN) // Ready to arm
            } else {
                LedPattern::SlowBlink(colors::YELLOW) // Not calibrated
            };

            let _ = LED_COMMAND_CHANNEL.try_send(led_pattern);

            if fc.is_armed() {
                info!(
                    "ARMED | IMU Cal: S{} G{} A{} M{} | Att:{}",
                    imu_status.calibration_status.sys,
                    imu_status.calibration_status.gyro,
                    imu_status.calibration_status.accel,
                    imu_status.calibration_status.mag,
                    if fc.is_attitude_enabled() {
                        "ON"
                    } else {
                        "OFF"
                    }
                );
            } else if fc.is_failsafe() {
                #[cfg(not(feature = "rtt-control"))]
                info!("FAILSAFE | IMU Errors: {}", imu_status.error_count);
            } else {
                #[cfg(not(feature = "rtt-control"))]
                info!(
                    "DISARMED | IMU: {}",
                    if imu_status.calibrated {
                        "Ready"
                    } else {
                        "Not calibrated"
                    }
                );
            }
        }

        // Yield control frequently for good responsiveness
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

    // Calibration wait (may be shorter if loaded from flash)
    if let Err(e) = imu.wait_for_calibration(IMU_CALIBRATION_TIMEOUT_S).await {
        defmt::warn!("Core1: IMU calibration incomplete: {}", e);
    }

    // Run continuous IMU reading
    imu.run().await;
}

#[embassy_executor::task]
async fn flash_manager_task(flash: Flash<'static, FLASH, Async, { FLASH_SIZE }>) {
    info!("Core0: Flash manager task starting");
    let mut manager = FlashManager::new(flash);
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
    rtt_control.run().await;
}

/// Process debug commands from RTT
#[cfg(feature = "rtt-control")]
async fn process_debug_command(fc: &mut FlightController<'_>, command: DebugCommand) {
    use elle::hardware::imu::IMU_STATUS;

    match command {
        DebugCommand::SetThrottle(value) => {
            info!("RTT: Set throttle to {}", value);
            // Note: This would require extending FlightController to accept direct throttle commands
            // For now, just log the command
        }

        DebugCommand::SetElevons { left, right } => {
            info!("RTT: Set elevons L={} R={}", left, right);
            // Note: This would require extending FlightController for direct elevon control
        }

        DebugCommand::SetControlMode(mode) => {
            info!("RTT: Set control mode to {:?}", mode);
            // Note: This would require extending FlightController to accept mode changes
        }

        DebugCommand::Arm => {
            info!("RTT: Arm command received");
            // Note: This would require extending FlightController for direct arming
        }

        DebugCommand::Disarm => {
            info!("RTT: Disarm command received");
            // Note: This would require extending FlightController for direct disarming
        }

        DebugCommand::EmergencyStop => {
            info!("RTT: EMERGENCY STOP");
            fc.apply_failsafe();
        }

        DebugCommand::AdjustTrim { left, right } => {
            info!("RTT: Adjust trim L={} R={}", left, right);
            // Note: This would require extending FlightController for trim adjustment
        }

        DebugCommand::SaveCalibration => {
            info!("RTT: Save calibration requested");
            // Note: This would require triggering calibration save
        }

        DebugCommand::ClearCalibration => {
            info!("RTT: Clear calibration requested");
            // Note: This would require implementing calibration clearing
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
            use elle::system::log_performance_summary;
            info!("RTT: Performance summary requested");
            log_performance_summary();
        }

        #[cfg(feature = "performance-monitoring")]
        DebugCommand::ResetPerformance => {
            use elle::system::PERFORMANCE_MONITOR;
            info!("RTT: Resetting performance counters");
            unsafe {
                (*core::ptr::addr_of_mut!(PERFORMANCE_MONITOR)).reset_all();
            }
        }
    }
}
