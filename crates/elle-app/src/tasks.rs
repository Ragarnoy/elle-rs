//! Tasks every airframe spawns: the Core1 IMU/AHRS task, the flash manager, and
//! (RPC builds) the postcard-RPC server and its log publisher.

use defmt::{info, warn};
use elle_config::profile::FLASH_SIZE;
use elle_config::{IMU_CALIBRATION_TIMEOUT_S, IMU_I2C_FREQ};
use elle_hardware::flash::SequentialFlashManager;
use elle_hardware::imu::{Imu, LED_COMMAND_CHANNEL};
#[cfg(feature = "rpc-control")]
use elle_system::rpc::init_rtt_rpc;
use elle_system::{SUP_IMU_READY, SUP_START_IMU};
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::Flash;
use embassy_rp::i2c::{Config, I2c, InterruptHandler as I2cIrqHandler};
use embassy_rp::mode::Async;
use embassy_rp::peripherals::{I2C0, PIN_0, PIN_1, PIN_2, PIN_3, PIN_5, PIN_8, PIN_9, SPI0};
#[cfg(feature = "rpc-control")]
use static_cell::StaticCell;

// I2C0 (mag + baro) is interrupt-driven. Bound here rather than in the binaries,
// which don't use I2C0: the bus is created on Core 1 below, so its interrupt is
// enabled on Core 1's NVIC and wakes the Core 1 executor directly.
bind_interrupts!(struct I2cIrqs {
    I2C0_IRQ => I2cIrqHandler<I2C0>;
});

#[allow(clippy::too_many_arguments)]
#[embassy_executor::task]
pub async fn imu_task(
    spawner: Spawner,
    // I2C (mag + baro)
    i2c: Peri<'static, I2C0>,
    sda: Peri<'static, PIN_8>,
    scl: Peri<'static, PIN_9>,
    // SPI (ICM-42686)
    spi: Peri<'static, SPI0>,
    spi_miso: Peri<'static, PIN_0>,
    spi_cs: Peri<'static, PIN_1>,
    spi_sck: Peri<'static, PIN_2>,
    spi_mosi: Peri<'static, PIN_3>,
    // INT1 (DATA_RDY interrupt)
    int1_pin: Peri<'static, PIN_5>,
) {
    info!("Core1: IMU task starting");

    // I2C0 for mag + baro, interrupt-driven, run by its own task (see below).
    let mut i2c_config = Config::default();
    i2c_config.frequency = embassy_rp::time::Hertz(IMU_I2C_FREQ);
    let i2c_bus = I2c::new(i2c, scl, sda, I2cIrqs, i2c_config);

    let led_sender = LED_COMMAND_CHANNEL.sender();

    let mut imu = {
        use embassy_rp::gpio::{Input, Level, Output, Pull};
        use embassy_rp::spi as rp_spi;
        use embedded_hal_bus::spi::ExclusiveDevice;

        let mut spi_config = rp_spi::Config::default();
        spi_config.frequency = embassy_rp::time::Hertz(elle_config::IMU_SPI_FREQ);
        // SPI Mode 0 (CPOL=0, CPHA=0) — ICM-42686 default
        spi_config.polarity = rp_spi::Polarity::IdleLow;
        spi_config.phase = rp_spi::Phase::CaptureOnFirstTransition;

        let spi_bus =
            rp_spi::Spi::new_blocking(spi, spi_sck, spi_mosi, spi_miso, spi_config).unwrap();
        let cs = Output::new(spi_cs, Level::High);
        let spi_dev = ExclusiveDevice::new(spi_bus, cs, embassy_time::Delay).unwrap();
        let int1 = Input::new(int1_pin, Pull::None);

        Imu::new(spi_dev, led_sender, int1)
    };

    // Initialize sensors
    match imu.initialize().await {
        Ok(_) => info!("Core1: IMU initialized"),
        Err(e) => {
            defmt::panic!("Core1: IMU init failed: {}", e);
        }
    }

    // Mag and baro run beside the IMU on this core, so their I2C transfers never
    // hold up a 1 kHz sample.
    spawner.spawn(i2c_sensors_task(i2c_bus).unwrap());

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

/// Core 1: magnetometer and barometer on I2C0 (`elle_hardware::imu::i2c_sensors`).
#[embassy_executor::task]
async fn i2c_sensors_task(i2c: I2c<'static, Async>) {
    elle_hardware::imu::i2c_sensors::run(i2c).await
}

#[embassy_executor::task]
pub async fn flash_manager_task(flash: Flash<'static, Async, { FLASH_SIZE }>) {
    info!("Core0: Flash manager task starting");
    let mut manager = SequentialFlashManager::new(flash);
    manager.run().await;
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
pub async fn rpc_server_task(
    spawner: Spawner,
    aon_timer: &'static mut embassy_rp::aon_timer::AonTimer<'static>,
) {
    use crate::rpc_handlers::RPC_CMD_CHANNEL;
    use elle_system::rpc::ElleWireSpawn;
    use postcard_rpc::server::{Dispatch, Server};

    info!("Core0: RPC server task starting");

    // Initialize RTT channels for RPC
    let channels = init_rtt_rpc();

    // RX buffer for incoming messages
    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let rx_buf: &'static mut [u8] = RX_BUF.init([0u8; 1024]);

    // Create dispatch context with command channel sender
    let context = crate::rpc_app::RpcContext {
        cmd_sender: RPC_CMD_CHANNEL.sender(),
        aon_timer,
    };

    // Create dispatcher via define_dispatch!-generated type
    let dispatcher = crate::rpc_app::ElleApp::new(context, ElleWireSpawn);
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
