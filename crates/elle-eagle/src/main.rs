#![no_std]
#![no_main]

//! Firmware for the XFly Eagle Testbed with WS2812B LED

#[cfg(feature = "defmt-logging")]
use defmt_rtt as _;

use defmt::{info, warn};
use elle_config::profile::FLASH_SIZE;
use elle_config::{
    CONTROL_LOOP_FREQUENCY_HZ, CONTROL_LOOP_PERIOD_MS, IMU_CALIBRATION_TIMEOUT_S, IMU_I2C_FREQ,
    IMU_MAX_AGE_MS,
};

#[cfg(not(feature = "rpc-control"))]
use defmt::debug;
#[cfg(not(feature = "rpc-control"))]
use elle_config::{
    ATTITUDE_ENABLE_CH, ATTITUDE_PITCH_SETPOINT_CH, ATTITUDE_ROLL_SETPOINT_CH, PITCH_CH, ROLL_CH,
    THROTTLE_CH, YAW_CH,
};
#[cfg(not(feature = "rpc-control"))]
use elle_control::commands::PilotCommands;
use elle_hardware::imu::{
    ATTITUDE_SIGNAL, AttitudeData, IMU_STATUS, Imu, LED_COMMAND_CHANNEL, is_attitude_valid,
};
#[cfg(feature = "ulog-logging")]
use elle_hardware::imu::{BARO_SIGNAL, MAG_SIGNAL};
use elle_hardware::led::{LedPattern, StatusLed, colors};
use elle_hardware::{
    pwm::{PwmOutputs, PwmPins},
    sequential_flash_manager::SequentialFlashManager,
};

#[cfg(feature = "ulog-logging")]
use elle_hardware::ULogLogger;

use elle_hardware::crsf::{CrsfReceiver, RC_COMMANDS, crsf_receiver_task, crsf_uart_config};

#[cfg(feature = "rpc-control")]
use elle_rpc_icd::ControlMode;
#[cfg(feature = "rpc-control")]
use elle_system::rpc::init_rtt_rpc;
#[cfg(feature = "rpc-control")]
pub mod flight_state;
#[cfg(feature = "rpc-control")]
mod rpc_app;

#[cfg(feature = "ulog-logging")]
use elle_system::update_ulog_timing;
use elle_system::{
    FlightController, SUP_FC_READY, SUP_IMU_READY, SUP_LED_READY, SUP_START_FC, SUP_START_IMU,
    TimingMeasurement, log_performance_summary, supervisor_task, update_control_loop_timing,
    update_led_timing,
};
use embassy_executor::{Executor, Spawner};
use embassy_rp::clocks::{ClockConfig, CoreVoltage};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::i2c::{Config, I2c};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{
    DMA_CH2, FLASH, I2C0, PIN_0, PIN_1, PIN_2, PIN_3, PIN_8, PIN_9, PIN_10, PIO0, PIO1, SPI0,
    UART0, UART1,
};
use embassy_rp::pio::{InterruptHandler as PioIrqHandler, Pio};
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

/// Log flight data to ULog flash storage
///
/// Logs attitude, commands, and periodic status updates at appropriate rates:
/// - Attitude: 77Hz (every call)
/// - Commands: 77Hz (every call)
/// - Status: 7.7Hz (every 10th call)
#[cfg(feature = "ulog-logging")]
async fn log_flight_data(
    logger: &mut ULogLogger,
    attitude: Option<&AttitudeData>,
    commands: &elle_control::commands::PilotCommands,
    loop_counter: u32,
    loop_timer_us: u32,
    fc: &FlightController<'_>,
) {
    use elle_control::commands::PilotCommands;

    // Measure ULog logging performance
    let ulog_timer = TimingMeasurement::start();

    // Log attitude data at 77Hz
    if let Some(att) = attitude {
        let _ = logger
            .log_attitude(
                att.pitch,
                att.roll,
                att.yaw,
                att.pitch_rate,
                att.roll_rate,
                att.yaw_rate,
            )
            .await;
    }

    // Log commands at 77Hz
    // Convert to normalized for consistent logging
    match commands {
        PilotCommands::Normalized(norm) => {
            let _ = logger
                .log_commands(
                    norm.throttle,
                    norm.pitch,
                    norm.roll,
                    norm.yaw,
                    norm.attitude_mode as u8,
                    norm.pitch_setpoint_deg,
                    norm.roll_setpoint_deg,
                )
                .await;
        }
        PilotCommands::Raw(raw) => {
            // Convert raw to normalized for logging
            let norm = raw.to_normalized();
            let _ = logger
                .log_commands(
                    norm.throttle,
                    norm.pitch,
                    norm.roll,
                    norm.yaw,
                    norm.attitude_mode as u8,
                    norm.pitch_setpoint_deg,
                    norm.roll_setpoint_deg,
                )
                .await;
        }
    }

    // Log status at reduced rate (7.7Hz - every 10th iteration)
    if loop_counter.is_multiple_of(10) {
        let imu_status = IMU_STATUS.try_read();
        let _ = logger
            .log_status(
                loop_timer_us,
                imu_status.as_ref().map(|s| s.error_count).unwrap_or(0),
                imu_status.as_ref().map(|s| s.calibrated).unwrap_or(false),
                fc.is_armed(),
                0.0, // CPU load - could calculate from timing data
            )
            .await;
    }

    // Log barometer at ~2Hz (every 38 iterations)
    if loop_counter.is_multiple_of(38) {
        if let Some(baro) = BARO_SIGNAL.try_take() {
            BARO_SIGNAL.signal(baro); // put back for other readers
            let _ = logger
                .log_barometer(baro.pressure_hpa, baro.temperature_c, baro.altitude_m)
                .await;
        }
    }

    // Log magnetometer at ~10Hz (every 8 iterations)
    if loop_counter.is_multiple_of(8) {
        if let Some(mag) = MAG_SIGNAL.try_take() {
            MAG_SIGNAL.signal(mag); // put back for other readers
            let _ = logger
                .log_magnetometer(mag.x as f32, mag.y as f32, mag.z as f32)
                .await;
        }
    }

    // Log GNSS at ~1Hz (every 77 iterations) — feature-gated
    #[cfg(all(feature = "gnss", feature = "rpc-control"))]
    if loop_counter.is_multiple_of(77) {
        if let Some(gnss) = gnss_signal::GNSS_SIGNAL.try_take() {
            gnss_signal::GNSS_SIGNAL.signal(gnss); // put back for other readers
            let _ = logger
                .log_gnss(
                    gnss.latitude,
                    gnss.longitude,
                    gnss.altitude_m,
                    gnss.fix_quality,
                    gnss.num_satellites,
                    gnss.hdop,
                )
                .await;
        }
    }

    // Drain event channel into ULog
    while let Ok((level, code)) = elle_hardware::event::ULOG_EVENT_CHANNEL.try_receive() {
        let _ = logger.log_event(level, code).await;
    }

    // Update performance monitoring
    update_ulog_timing(ulog_timer.elapsed_us());
}

bind_interrupts!(
    struct Irqs {
        PIO0_IRQ_0 => PioIrqHandler<PIO0>;
        PIO1_IRQ_0 => PioIrqHandler<PIO1>;
        UART0_IRQ => UartIrqHandler<UART0>;
        UART1_IRQ => UartIrqHandler<UART1>;
    }
);

static mut CORE1_STACK: Stack<16384> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// RC channel signal for RPC handler
#[cfg(feature = "rpc-control")]
pub mod rc_signal {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;

    pub static RC_SIGNAL: Signal<CriticalSectionRawMutex, [u16; 16]> = Signal::new();
}

// GNSS signal for sharing position data with RPC handler
#[cfg(all(feature = "gnss", feature = "rpc-control"))]
pub mod gnss_signal {
    use elle_rpc_icd::GnssResp;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;

    pub static GNSS_SIGNAL: Signal<CriticalSectionRawMutex, GnssResp> = Signal::new();
}

#[cfg(feature = "rpc-control")]
mod rpc_handlers;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config =
        embassy_rp::config::Config::new(ClockConfig::system_freq(200_000_000).unwrap());
    config.clocks.core_voltage = CoreVoltage::V1_15;

    let p = embassy_rp::init(config);

    info!("Core0: Starting flash manager");
    // Create flash manager on Core 0 before spawning Core 1
    let flash = embassy_rp::flash::Flash::<_, Async, { FLASH_SIZE }>::new(p.FLASH, p.DMA_CH1);
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

    #[cfg(feature = "rpc-control")]
    {
        info!("Core0: Starting RPC server");
        spawner.spawn(rpc_server_task(spawner).unwrap());
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
                spawner.spawn(
                    imu_task(
                        spawner, p.I2C0, p.PIN_8, p.PIN_9, p.SPI0, p.PIN_0, p.PIN_1, p.PIN_2,
                        p.PIN_3,
                    )
                    .unwrap(),
                );
            })
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

    // Always spawn CRSF receiver — use DMA_CH3 when rpc-control is enabled
    // (DMA_CH0 is reserved for GNSS in that configuration)
    {
        info!("Core0: Starting CRSF receiver task (UART1, GPIO21)");
        let config = crsf_uart_config();

        #[cfg(feature = "crsf-telemetry")]
        {
            // Split UART1: RX for CRSF receiver, TX for telemetry
            #[cfg(not(feature = "rpc-control"))]
            let uart = embassy_rp::uart::Uart::new(
                p.UART1, p.PIN_20, p.PIN_21, Irqs, p.DMA_CH4, p.DMA_CH0, config,
            );
            #[cfg(feature = "rpc-control")]
            let uart = embassy_rp::uart::Uart::new(
                p.UART1, p.PIN_20, p.PIN_21, Irqs, p.DMA_CH4, p.DMA_CH3, config,
            );
            let (tx, rx) = uart.split();
            let crsf = CrsfReceiver::new(rx);
            spawner.spawn(crsf_receiver_task(crsf).unwrap());

            info!("Core0: Starting CRSF telemetry TX task (PIN_20, DMA_CH4)");
            spawner.spawn(elle_hardware::crsf_telemetry::crsf_telemetry_task(tx).unwrap());
        }

        #[cfg(not(feature = "crsf-telemetry"))]
        {
            #[cfg(not(feature = "rpc-control"))]
            let rx = embassy_rp::uart::UartRx::new(p.UART1, p.PIN_21, Irqs, p.DMA_CH0, config);
            #[cfg(feature = "rpc-control")]
            let rx = embassy_rp::uart::UartRx::new(p.UART1, p.PIN_21, Irqs, p.DMA_CH3, config);
            let crsf = CrsfReceiver::new(rx);
            spawner.spawn(crsf_receiver_task(crsf).unwrap());
        }
    }

    #[cfg(all(feature = "gnss", feature = "rpc-control"))]
    {
        info!("Core0: Starting GNSS task (UART0, GPIO28/29)");
        spawner.spawn(gnss_task(p.UART0, p.PIN_29, p.DMA_CH0).unwrap());
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

    // ULog logger created but NOT initialized — user starts recording via `ulog start`
    #[cfg(feature = "ulog-logging")]
    let mut ulog_logger = ULogLogger::new();

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

    #[cfg(not(feature = "rpc-control"))]
    {
        info!("FLIGHT MODE - CRSF/ELRS Control");

        #[cfg(feature = "ulog-logging")]
        let mut ulog_recording = false;

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

            // Check for latest RC commands from dedicated CRSF receiver task (non-blocking)
            if let Some(commands) = RC_COMMANDS.try_take() {
                // Debug logging (~8Hz)
                if loop_counter.is_multiple_of(CONTROL_LOOP_FREQUENCY_HZ / 10)
                    && let PilotCommands::Raw(raw) = &commands
                {
                    debug!(
                        "RC: CH1:{} CH2:{} CH3:{} CH4:{} CH5:{} CH6:{} CH8:{}",
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
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_ATTITUDE_STALE,
                        "Stale attitude data, using manual control only"
                    );
                }
                fc.update(commands, valid_attitude.as_ref());

                #[cfg(feature = "crsf-telemetry")]
                elle_hardware::crsf_telemetry::CRSF_FLIGHT_MODE.signal(
                    elle_hardware::crsf_telemetry::CrsfFlightMode {
                        armed: fc.is_armed(),
                        failsafe: fc.is_failsafe(),
                        attitude_mode: fc.is_attitude_enabled(),
                    },
                );

                // ULog recording controlled by RC switch
                #[cfg(feature = "ulog-logging")]
                {
                    let switch_on = if let PilotCommands::Raw(raw) = commands {
                        raw.channels[elle_config::ULOG_ENABLE_CH]
                            > elle_config::ULOG_ENABLE_THRESHOLD
                    } else {
                        false
                    };

                    if switch_on && !ulog_recording {
                        // Switch just turned on — initialize and start
                        if !ulog_logger.is_initialized() {
                            let _ = ulog_logger.initialize().await;
                        }
                        if ulog_logger.is_initialized() {
                            ulog_recording = true;
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_ULOG_RC_ON,
                                "ULog recording ON (RC switch)"
                            );
                        }
                    } else if !switch_on && ulog_recording {
                        // Switch just turned off — flush and stop
                        let _ = ulog_logger.flush().await;
                        ulog_recording = false;
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_ULOG_RC_OFF,
                            "ULog recording OFF (RC switch)"
                        );
                    }

                    if ulog_recording {
                        log_flight_data(
                            &mut ulog_logger,
                            valid_attitude.as_ref(),
                            commands,
                            loop_counter,
                            loop_timer.elapsed_us(),
                            &fc,
                        )
                        .await;
                    } else {
                        // Drain stale events when not recording
                        while elle_hardware::event::ULOG_EVENT_CHANNEL
                            .try_receive()
                            .is_ok()
                        {}
                    }
                }
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

    #[cfg(feature = "rpc-control")]
    {
        use core::sync::atomic::Ordering;
        use elle_config::profile::{FlashRequest, FlashResponse};
        use elle_control::commands::{AttitudeMode, NormalizedCommands, PilotCommands};
        use elle_hardware::sequential_flash_manager::{
            FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL,
        };
        use embassy_time::Instant;
        use rpc_app::{ULOG_ENABLED, ULOG_ITEM_LEN, ULOG_ITEM_SIGNAL, ULOG_OFFSET, ULOG_STATE};
        use rpc_handlers::{RPC_CMD_CHANNEL, RpcCommand};

        info!("GROUND TEST MODE - RPC Control (postcard-RPC over RTT)");
        info!("WARNING: This mode requires programmer connection");

        // Ticker for consistent control loop timing
        let mut ticker = Ticker::every(Duration::from_millis(CONTROL_LOOP_PERIOD_MS));

        // RPC commands accumulator (updated by RPC handlers)
        let mut rpc_throttle: f32 = 0.0;
        let mut rpc_elevon_left: f32 = 0.0;
        let mut rpc_elevon_right: f32 = 0.0;

        loop {
            ticker.next().await;
            let loop_timer = TimingMeasurement::start();

            // Supervisor check
            let _supervisor_healthy = fc.supervisor_check();

            // Get latest attitude data
            let attitude = ATTITUDE_SIGNAL.try_take();

            // Process all pending RPC commands
            while let Ok(cmd) = RPC_CMD_CHANNEL.try_receive() {
                match cmd {
                    RpcCommand::SetThrottle(percent) => {
                        rpc_throttle = (percent as f32 / 100.0).clamp(0.0, 1.0);
                    }
                    RpcCommand::SetElevons { left, right } => {
                        rpc_elevon_left = (left as f32 / 100.0).clamp(-1.0, 1.0);
                        rpc_elevon_right = (right as f32 / 100.0).clamp(-1.0, 1.0);
                    }
                    RpcCommand::SetMode(_mode) => {
                        // TODO: implement mode switching
                    }
                    RpcCommand::Arm => {
                        fc.arm();
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_MOTORS_ARMED,
                            "Motors ARMED via RPC"
                        );
                    }
                    RpcCommand::Disarm => {
                        fc.disarm();
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_MOTORS_DISARMED,
                            "Motors DISARMED via RPC"
                        );
                    }
                    RpcCommand::EmergencyStop => {
                        elle_hardware::elle_event!(
                            warn,
                            elle_hardware::event::EVT_EMERGENCY_STOP,
                            "RPC: EMERGENCY STOP"
                        );
                        rpc_throttle = 0.0;
                        rpc_elevon_left = 0.0;
                        rpc_elevon_right = 0.0;
                        fc.disarm();
                        fc.apply_failsafe();
                    }
                    RpcCommand::AdjustTrim { left, right } => {
                        info!("RPC: Trim L={} R={} (not implemented)", left, right);
                    }
                    RpcCommand::SaveCalibration => {
                        info!("RPC: Save calibration requested");
                    }
                    RpcCommand::ClearCalibration => {
                        info!("RPC: Clear calibration requested");
                    }
                    RpcCommand::StartULog => {
                        #[cfg(feature = "ulog-logging")]
                        {
                            let mut ok = ulog_logger.is_initialized();
                            if !ok {
                                ok = ulog_logger.initialize().await.is_ok();
                            }
                            if ok {
                                ULOG_ENABLED.store(true, Ordering::Release);
                                elle_hardware::elle_event!(
                                    info,
                                    elle_hardware::event::EVT_ULOG_STARTED,
                                    "ULog recording started"
                                );
                            } else {
                                elle_hardware::elle_event!(
                                    error,
                                    elle_hardware::event::EVT_ULOG_INIT_FAILED,
                                    "ULog init failed"
                                );
                            }
                        }
                        #[cfg(not(feature = "ulog-logging"))]
                        {
                            elle_hardware::elle_event!(
                                warn,
                                elle_hardware::event::EVT_ULOG_NOT_COMPILED,
                                "ULog not compiled in"
                            );
                        }
                    }
                    RpcCommand::StopULog => {
                        ULOG_ENABLED.store(false, Ordering::Release);
                        #[cfg(feature = "ulog-logging")]
                        {
                            let _ = ulog_logger.flush().await;
                        }
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_ULOG_STOPPED,
                            "ULog recording stopped"
                        );
                    }
                    RpcCommand::ReadULogChunk => {
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::PeekULog);
                        match FLASH_RESPONSE_SIGNAL.wait().await {
                            FlashResponse::ULogData { data, len } => {
                                ULOG_ITEM_LEN.store(len as u16, Ordering::Release);
                                ULOG_ITEM_SIGNAL.signal((data, len));
                                ULOG_STATE.store(2, Ordering::Release); // ULOG_READY
                            }
                            FlashResponse::ULogEmpty => {
                                ULOG_STATE.store(3, Ordering::Release); // ULOG_EMPTY
                            }
                            _ => {
                                ULOG_STATE.store(3, Ordering::Release); // ULOG_EMPTY on error
                            }
                        }
                    }
                    RpcCommand::PopAndPeekULog => {
                        // Pop the item we just finished sending
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::PopULog);
                        let _ = FLASH_RESPONSE_SIGNAL.wait().await;
                        // Peek the next item
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::PeekULog);
                        match FLASH_RESPONSE_SIGNAL.wait().await {
                            FlashResponse::ULogData { data, len } => {
                                ULOG_ITEM_LEN.store(len as u16, Ordering::Release);
                                ULOG_ITEM_SIGNAL.signal((data, len));
                                ULOG_STATE.store(2, Ordering::Release); // ULOG_READY
                            }
                            FlashResponse::ULogEmpty => {
                                ULOG_STATE.store(3, Ordering::Release); // ULOG_EMPTY
                            }
                            _ => {
                                ULOG_STATE.store(3, Ordering::Release); // ULOG_EMPTY on error
                            }
                        }
                    }
                    RpcCommand::EraseULog => {
                        // Stop recording first
                        ULOG_ENABLED.store(false, Ordering::Release);
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::EraseULog);
                        let _ = FLASH_RESPONSE_SIGNAL.wait().await;
                        // Reset ULog transfer state
                        ULOG_STATE.store(0, Ordering::Release); // ULOG_IDLE
                        ULOG_OFFSET.store(0, Ordering::Release);
                        ULOG_ITEM_LEN.store(0, Ordering::Release);
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_ULOG_ERASED,
                            "ULog: flash erased"
                        );
                    }
                }
            }

            // Poll CRSF receiver and update RC signal for RPC handler
            if let Some(commands) = RC_COMMANDS.try_take() {
                if let PilotCommands::Raw(raw) = &commands {
                    rc_signal::RC_SIGNAL.signal(raw.channels);
                }
            }

            // Build pilot commands from RPC state
            let commands = PilotCommands::Normalized(NormalizedCommands {
                throttle: rpc_throttle,
                pitch: (rpc_elevon_left + rpc_elevon_right) / 2.0, // Mixed
                roll: (rpc_elevon_right - rpc_elevon_left) / 2.0,  // Mixed
                yaw: 0.0,
                attitude_mode: AttitudeMode::Manual,
                pitch_setpoint_deg: 0.0,
                roll_setpoint_deg: 0.0,
                timestamp: Instant::now(),
            });

            // Update flight controller
            let valid_attitude = validate_attitude(attitude);
            fc.update(&commands, valid_attitude.as_ref());

            #[cfg(feature = "crsf-telemetry")]
            elle_hardware::crsf_telemetry::CRSF_FLIGHT_MODE.signal(
                elle_hardware::crsf_telemetry::CrsfFlightMode {
                    armed: fc.is_armed(),
                    failsafe: fc.is_failsafe(),
                    attitude_mode: fc.is_attitude_enabled(),
                },
            );

            // Publish flight state for RPC handlers
            flight_state::FLIGHT_STATE.signal(flight_state::FlightState {
                armed: fc.is_armed(),
                failsafe: fc.is_failsafe(),
                mode: match fc.current_control_mode() {
                    elle_system::ControlMode::Manual => ControlMode::Manual,
                    elle_system::ControlMode::Mixed => ControlMode::Mixed,
                    elle_system::ControlMode::Autopilot => ControlMode::Autopilot,
                },
            });

            // Log flight data to ULog flash storage (only when recording is active)
            #[cfg(feature = "ulog-logging")]
            if ULOG_ENABLED.load(Ordering::Acquire) {
                log_flight_data(
                    &mut ulog_logger,
                    valid_attitude.as_ref(),
                    &commands,
                    loop_counter,
                    loop_timer.elapsed_us(),
                    &fc,
                )
                .await;
            } else {
                // Drain stale events when not recording
                while elle_hardware::event::ULOG_EVENT_CHANNEL
                    .try_receive()
                    .is_ok()
                {}
            }

            update_control_loop_timing(loop_timer.elapsed_us());
            loop_counter = loop_counter.saturating_add(1);

            if loop_counter.is_multiple_of(CONTROL_LOOP_FREQUENCY_HZ * 10) {
                log_performance_summary();
            }

            if loop_counter.is_multiple_of(2000) {
                loop_counter = 0;

                let imu_status = IMU_STATUS.read().await;
                let led_pattern = if fc.is_armed() {
                    LedPattern::DoubleBlink(colors::PURPLE)
                } else if fc.is_failsafe() {
                    LedPattern::RapidFlash(colors::ORANGE)
                } else if imu_status.calibrated {
                    LedPattern::Solid(colors::PURPLE)
                } else {
                    LedPattern::Pulse(colors::PURPLE)
                };

                let _ = LED_COMMAND_CHANNEL.try_send(led_pattern);
                drop(imu_status);
            }
        }
    }
}

#[embassy_executor::task]
async fn imu_task(
    _spawner: Spawner,
    // I2C (mag + baro)
    i2c: Peri<'static, I2C0>,
    sda: Peri<'static, PIN_8>,
    scl: Peri<'static, PIN_9>,
    // SPI (ICM-42686) — unused in disable-imu stub
    #[allow(unused_variables)] spi: Peri<'static, SPI0>,
    #[allow(unused_variables)] spi_miso: Peri<'static, PIN_0>,
    #[allow(unused_variables)] spi_cs: Peri<'static, PIN_1>,
    #[allow(unused_variables)] spi_sck: Peri<'static, PIN_2>,
    #[allow(unused_variables)] spi_mosi: Peri<'static, PIN_3>,
) {
    info!("Core1: IMU task starting");

    // I2C bus — always wrapped in RefCell (both cfg paths use shared I2C)
    let mut i2c_config = Config::default();
    i2c_config.frequency = IMU_I2C_FREQ;
    let i2c_bus = I2c::new_blocking(i2c, scl, sda, i2c_config);

    use core::cell::RefCell;
    static I2C_BUS: StaticCell<RefCell<I2c<'static, I2C0, embassy_rp::i2c::Blocking>>> =
        StaticCell::new();
    let i2c_ref = I2C_BUS.init(RefCell::new(i2c_bus));

    let led_sender = LED_COMMAND_CHANNEL.sender();

    #[cfg(feature = "disable-imu")]
    let mut imu = Imu::new(i2c_ref, led_sender);

    #[cfg(not(feature = "disable-imu"))]
    let mut imu = {
        use embassy_rp::gpio::{Level, Output};
        use embassy_rp::spi as rp_spi;
        use embedded_hal_bus::spi::ExclusiveDevice;

        let mut spi_config = rp_spi::Config::default();
        spi_config.frequency = elle_config::IMU_SPI_FREQ;
        // SPI Mode 0 (CPOL=0, CPHA=0) — ICM-42686 default
        spi_config.polarity = rp_spi::Polarity::IdleLow;
        spi_config.phase = rp_spi::Phase::CaptureOnFirstTransition;

        let spi_bus = rp_spi::Spi::new_blocking(spi, spi_sck, spi_mosi, spi_miso, spi_config);
        let cs = Output::new(spi_cs, Level::High);
        let spi_dev = ExclusiveDevice::new(spi_bus, cs, embassy_time::Delay).unwrap();

        Imu::new(spi_dev, i2c_ref, led_sender)
    };

    // Initialize sensors
    match imu.initialize().await {
        Ok(_) => info!("Core1: IMU initialized"),
        Err(e) => {
            defmt::panic!("Core1: IMU init failed: {}", e);
        }
    }

    // Notify supervisor that IMU is initialized
    SUP_IMU_READY.signal(());

    // Calibration wait (ICM-42686 returns immediately — factory calibrated)
    if let Err(e) = imu.wait_for_calibration(IMU_CALIBRATION_TIMEOUT_S).await {
        warn!("Core1: IMU calibration incomplete: {}", e);
    }

    // Wait for supervisor start before entering main IMU run loop
    info!("Core1: Waiting for Supervisor start barrier");
    SUP_START_IMU.wait().await;

    // Run continuous IMU reading
    imu.run().await;
}

#[cfg(all(feature = "gnss", feature = "rpc-control"))]
#[embassy_executor::task]
async fn gnss_task(
    uart: Peri<'static, UART0>,
    rx_pin: Peri<'static, embassy_rp::peripherals::PIN_29>,
    rx_dma: Peri<'static, embassy_rp::peripherals::DMA_CH0>,
) {
    use embassy_rp::uart::{self, UartRx};
    use sam_m10q::decoder::{Decoder, FeedResult};
    use sam_m10q::nmea::ParseResult;
    use sam_m10q::types::Frame;

    info!("GNSS task starting (9600 baud, UART0 RX on GPIO29)");

    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 9600;

    let mut rx = UartRx::new(uart, rx_pin, Irqs, rx_dma, uart_config);
    let mut decoder = Decoder::new();
    let mut gga_count: u32 = 0;
    let mut uart_error_count: u32 = 0;
    let mut last_lat: f32 = 0.0;
    let mut last_lon: f32 = 0.0;
    let mut last_alt: f32 = 0.0;

    loop {
        let mut byte = [0u8; 1];
        match rx.read(&mut byte).await {
            Ok(()) => {
                uart_error_count = 0;
            }
            Err(e) => {
                uart_error_count += 1;
                if uart_error_count <= 3 || uart_error_count % 1000 == 0 {
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_GNSS_UART_ERROR,
                        "GNSS UART read error: {} (total={})",
                        e,
                        uart_error_count
                    );
                }
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }
        }

        match decoder.feed(byte[0]) {
            FeedResult::Pending => {}
            FeedResult::FrameReady => {
                if let Frame::Nmea(nmea_frame) = decoder.take_frame() {
                    if let Some(ParseResult::GGA(gga)) = nmea_frame.parsed {
                        gga_count = gga_count.wrapping_add(1);
                        let sats = gga.fix_satellites.unwrap_or(0) as u8;
                        let fix = match gga.fix_type {
                            Some(sam_m10q::nmea::sentences::FixType::Invalid) | None => 0,
                            Some(sam_m10q::nmea::sentences::FixType::Gps) => 1,
                            Some(sam_m10q::nmea::sentences::FixType::DGps) => 2,
                            Some(_) => 3,
                        };

                        if gga_count == 1 {
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_GNSS_FIRST_FIX,
                                "GNSS: first GGA received (fix={}, sats={})",
                                fix,
                                sats
                            );
                        } else if gga_count.is_multiple_of(60) {
                            elle_hardware::elle_event!(
                                debug,
                                elle_hardware::event::EVT_GNSS_PERIODIC,
                                "GNSS: {} GGA sentences (fix={}, sats={})",
                                gga_count,
                                fix,
                                sats
                            );
                        }

                        if fix > 0 {
                            if let Some(lat) = gga.latitude {
                                last_lat = lat as f32;
                            }
                            if let Some(lon) = gga.longitude {
                                last_lon = lon as f32;
                            }
                            if let Some(alt) = gga.altitude {
                                last_alt = alt;
                            }
                        }

                        let resp = elle_rpc_icd::GnssResp {
                            latitude: last_lat,
                            longitude: last_lon,
                            altitude_m: last_alt,
                            fix_quality: fix,
                            num_satellites: sats,
                            hdop: gga.hdop.unwrap_or(99.9),
                        };
                        gnss_signal::GNSS_SIGNAL.signal(resp);

                        #[cfg(feature = "crsf-telemetry")]
                        elle_hardware::crsf_telemetry::TELEMETRY_GNSS.signal(
                            elle_hardware::crsf_telemetry::TelemetryGpsData {
                                latitude: last_lat,
                                longitude: last_lon,
                                altitude_m: last_alt,
                                num_satellites: sats,
                            },
                        );
                    }
                }
            }
            FeedResult::Error(_) => {
                // Non-fatal decode error, continue
            }
        }
    }
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

#[cfg(feature = "rpc-control")]
#[embassy_executor::task]
async fn log_publisher_task(sender: postcard_rpc::server::Sender<elle_system::rpc::RttTx>) {
    use postcard_rpc::header::VarSeq;

    let mut seq: u16 = 0;
    loop {
        let (level, code) = elle_hardware::event::EVENT_CHANNEL.receive().await;
        let msg = elle_rpc_icd::LogMsg { level, code };
        let _ = sender
            .publish::<elle_rpc_icd::LogTopic>(VarSeq::Seq2(seq), &msg)
            .await;
        seq = seq.wrapping_add(1);
    }
}

/// RPC server task using postcard-RPC over RTT with define_dispatch!
#[cfg(feature = "rpc-control")]
#[embassy_executor::task]
async fn rpc_server_task(spawner: Spawner) {
    use elle_system::rpc::ElleWireSpawn;
    use postcard_rpc::server::{Dispatch, Server};
    use rpc_handlers::RPC_CMD_CHANNEL;

    info!("Core0: RPC server task starting");

    // Initialize RTT channels for RPC
    let channels = init_rtt_rpc();

    // RX buffer for incoming messages
    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let rx_buf: &'static mut [u8] = RX_BUF.init([0u8; 1024]);

    // Create dispatch context with command channel sender
    let context = rpc_app::RpcContext {
        cmd_sender: RPC_CMD_CHANNEL.sender(),
    };

    // Create dispatcher via define_dispatch!-generated type
    let dispatcher = rpc_app::ElleApp::new(context, ElleWireSpawn);
    let vkk = dispatcher.min_key_len();

    // Create and run the server
    let mut server = Server::new(channels.tx, channels.rx, rx_buf, dispatcher, vkk);

    let sender = server.sender();
    spawner.spawn(log_publisher_task(sender).unwrap());

    info!("Core0: RPC server ready, waiting for commands");

    loop {
        // run() returns on fatal error; just restart
        let _err = server.run().await;
    }
}
