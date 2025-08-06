#![no_std]
#![no_main]

//! Firmware for the XFly Eagle Testbed with WS2812B LED

use defmt::{debug, info};
use defmt_rtt as _;
use elle::config::profile::FLASH_SIZE;
use elle::config::{IMU_CALIBRATION_TIMEOUT_S, IMU_I2C_FREQ, IMU_MAX_AGE_MS};
use elle::hardware::imu::{
    ATTITUDE_SIGNAL, BnoImu, IMU_STATUS, LED_COMMAND_CHANNEL, is_attitude_valid,
};
use elle::hardware::led::{LedPattern, StatusLed, colors};
use elle::hardware::{
    flash_manager::FlashManager,
    pwm::{PwmOutputs, PwmPins},
    sbus::SbusReceiver,
};
use elle::system::FlightController;
use embassy_executor::{Executor, Spawner};
use embassy_rp::clocks::{ClockConfig, CoreVoltage};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::i2c::Config;
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
        elevon_left: p.PIN_15,
        elevon_right: p.PIN_14,
        engine_left: p.PIN_12,
        engine_right: p.PIN_11,
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
        if wait_counter % 50 == 0 {
            // Log every 5 seconds
            info!("Core0: Still waiting for IMU... ({}s)", wait_counter / 10);
        }
        Timer::after(Duration::from_millis(100)).await;
    }

    info!("Core0: Initializing ESCs");
    fc.initialize_escs().await;

    info!("Core0: Starting main control loop");
    // Main control loop with IMU integration
    let mut loop_counter = 0u32;

    loop {
        // Get latest attitude data (non-blocking)
        let attitude = ATTITUDE_SIGNAL.try_take();

        if let Some(packet) = sbus.read_packet().await {
            // Print all SBUS channels (less frequently)
            if loop_counter % 5000 == 0 {
                // Every ~25 seconds at 200Hz
                debug!(
                    "SBUS: CH1:{} CH2:{} CH3:{} CH4:{} CH5:{} CH6:{}",
                    packet.channels[0],
                    packet.channels[1],
                    packet.channels[2],
                    packet.channels[3],
                    packet.channels[4],
                    packet.channels[5]
                );
            }

            // Pass attitude data to flight controller if available and valid
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
        }

        fc.check_failsafe();

        // Status output and LED updates
        loop_counter = loop_counter.saturating_add(1);
        if loop_counter % 20000 == 0 {
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
            } else {
                if imu_status.calibrated {
                    LedPattern::Solid(colors::GREEN) // Ready to arm
                } else {
                    LedPattern::SlowBlink(colors::YELLOW) // Not calibrated
                }
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
                info!("FAILSAFE | IMU Errors: {}", imu_status.error_count);
            } else {
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
        Timer::after(Duration::from_millis(5)).await;
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

    let i2c_bus = embassy_rp::i2c::I2c::new_blocking(i2c, scl, sda, i2c_config);
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
        // Check for new patterns
        if let Ok(pattern) = receiver.try_receive() {
            led.set_pattern(pattern).await;
        }

        // Update LED animation
        led.update().await;

        // Small delay for animation timing
        Timer::after(Duration::from_millis(10)).await;
    }
}
