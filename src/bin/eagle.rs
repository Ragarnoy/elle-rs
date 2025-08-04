#![no_std]
#![no_main]

//! Firmware for the XFly Eagle Testbed

use defmt::{debug, info};
use defmt_rtt as _;
use elle::config::{IMU_CALIBRATION_TIMEOUT_S, IMU_I2C_FREQ, IMU_MAX_AGE_MS};
use elle::hardware::imu::{ATTITUDE_SIGNAL, BnoImu, IMU_STATUS, is_attitude_valid};
use elle::hardware::{
    pwm::{PwmOutputs, PwmPins},
    sbus::SbusReceiver,
};
use elle::system::FlightController;
use embassy_executor::{Executor, Spawner};
use embassy_rp::i2c::Config;
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{I2C0, PIN_8, PIN_9, PIN_25, PIO0, UART0};
use embassy_rp::pio::{InterruptHandler as PioIrqHandler, Pio};
use embassy_rp::uart::InterruptHandler as UartIrqHandler;
use embassy_rp::{Peri, bind_interrupts};
use embassy_time::{Duration, Timer};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(
    struct Irqs {
        PIO0_IRQ_0 => PioIrqHandler<PIO0>;
        UART0_IRQ => UartIrqHandler<UART0>;
    }
);

static mut CORE1_STACK: Stack<8192> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Spawn IMU task on core1
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner
                    .spawn(imu_task(spawner, p.I2C0, p.PIN_8, p.PIN_9, p.PIN_25))
                    .unwrap();
            });
        },
    );

    // Core0: Setup flight control hardware
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
    info!("Waiting for IMU initialization...");
    loop {
        let status = IMU_STATUS.read().await;
        if status.initialized {
            info!("IMU ready! Calibrated: {}", status.calibrated);
            break;
        }
        drop(status);
        Timer::after(Duration::from_millis(100)).await;
    }

    fc.initialize_escs().await;

    // Main control loop with IMU integration
    let mut loop_counter = 0u32;

    loop {
        // Get latest attitude data (non-blocking)
        let attitude = ATTITUDE_SIGNAL.try_take();

        if let Some(packet) = sbus.read_packet().await {
            // Print all SBUS channels
            if loop_counter % 1000 == 0 {
                debug!(
                    "CH1: {} CH2: {} CH3: {} CH4: {} CH5: {} CH6: {} CH7: {} CH8: {} CH9: {} CH10: {}",
                    packet.channels[0],
                    packet.channels[1],
                    packet.channels[2],
                    packet.channels[3],
                    packet.channels[4],
                    packet.channels[5],
                    packet.channels[6],
                    packet.channels[7],
                    packet.channels[8],
                    packet.channels[9],
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

        // Status output
        loop_counter += 1;
        if loop_counter >= 10000 {
            loop_counter = 0;

            // Get IMU status
            let imu_status = IMU_STATUS.read().await;

            if fc.is_armed() {
                info!(
                    "ARMED | IMU Cal: S{} G{} A{} M{}",
                    imu_status.calibration_status.sys,
                    imu_status.calibration_status.gyro,
                    imu_status.calibration_status.accel,
                    imu_status.calibration_status.mag
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
    }
}

#[embassy_executor::task]
async fn imu_task(
    _spawner: Spawner,
    i2c: Peri<'static, I2C0>,
    sda: Peri<'static, PIN_8>,
    scl: Peri<'static, PIN_9>,
    led: Peri<'static, PIN_25>,
) {
    info!("Core1: IMU task starting");

    // Configure I2C for IMU
    let mut i2c_config = Config::default();
    i2c_config.frequency = IMU_I2C_FREQ;

    let i2c_bus = embassy_rp::i2c::I2c::new_blocking(i2c, scl, sda, i2c_config);
    let mut imu = BnoImu::new(i2c_bus, led);

    // Initialize IMU
    match imu.initialize().await {
        Ok(_) => info!("Core1: IMU initialized"),
        Err(e) => {
            defmt::panic!("Core1: IMU init failed: {}", e);
        }
    }

    // Wait for calibration (with timeout)
    if let Err(e) = imu.wait_for_calibration(IMU_CALIBRATION_TIMEOUT_S).await {
        defmt::warn!("Core1: IMU calibration incomplete: {}", e);
    }

    // Run continuous IMU reading
    imu.run().await;
}
