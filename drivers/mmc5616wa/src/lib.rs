#![no_std]

#[cfg(feature = "async")]
pub mod asynch;
pub mod error;
pub mod registers;
pub mod types;

mod interface;

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::error::Error;
use crate::interface::RegisterCache;
use crate::registers::*;
use crate::types::*;

/// Maximum number of status polls before declaring a timeout.
const MAX_POLL_ATTEMPTS: u32 = 500;

/// Driver for the MEMSIC MMC5616WA 3-axis magnetometer.
pub struct Mmc5616wa<I> {
    i2c: I,
    addr: u8,
    cache: RegisterCache,
}

impl<I: I2c> Mmc5616wa<I> {
    /// Create a new driver instance with a custom I2C address.
    pub fn new(i2c: I, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            cache: RegisterCache::default(),
        }
    }

    /// Create a new driver instance using the default address (0x30).
    pub fn new_default(i2c: I) -> Self {
        Self::new(i2c, DEFAULT_ADDRESS)
    }

    /// Release the I2C bus, consuming the driver.
    pub fn destroy(self) -> I {
        self.i2c
    }

    /// Wait for the power-on time (tOp = 5 ms) and reset the shadow cache.
    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I::Error>> {
        delay.delay_ms(5);
        self.cache = RegisterCache::default();
        Ok(())
    }

    /// Read the Chip ID register and verify it matches the expected value (0xD2).
    pub fn validate(&mut self) -> Result<(), Error<I::Error>> {
        let id = self.chip_id()?;
        if id != CHIP_ID_VALUE {
            return Err(Error::InvalidChipId(id));
        }
        Ok(())
    }

    // --- Identity ---

    /// Read the Chip ID register (0x21).
    pub fn chip_id(&mut self) -> Result<u8, Error<I::Error>> {
        interface::read_reg(&mut self.i2c, self.addr, CHIP_ID)
    }

    /// Read the Product ID register (0x39).
    pub fn product_id(&mut self) -> Result<u8, Error<I::Error>> {
        interface::read_reg(&mut self.i2c, self.addr, PRODUCT_ID)
    }

    // --- Measurement ---

    /// Read the latest magnetic output registers and return signed counts.
    ///
    /// Use this after starting continuous mode with [`start_continuous`].
    /// Does **not** trigger a new measurement.
    pub fn read_magnetic(&mut self) -> Result<MagData, Error<I::Error>> {
        let raw = self.read_raw_mag()?;
        Ok(MagData {
            x: to_signed(raw.x),
            y: to_signed(raw.y),
            z: to_signed(raw.z),
        })
    }

    /// Check whether a magnetic measurement is ready by reading Status1.
    pub fn data_ready(&mut self) -> Result<bool, Error<I::Error>> {
        let status = interface::read_reg(&mut self.i2c, self.addr, STATUS1)?;
        Ok(status & MEAS_M_DONE != 0)
    }

    /// Perform a one-shot magnetic measurement with automatic SET/RESET.
    ///
    /// Triggers TM_M with AUTO_SR_EN, polls Status1 for completion,
    /// burst-reads all 9 output bytes, and returns signed counts.
    pub fn measure_magnetic(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<MagData, Error<I::Error>> {
        // Trigger measurement: TM_M + AUTO_SR_EN
        interface::modify_ctrl0(
            &mut self.i2c,
            self.addr,
            &mut self.cache,
            TM_M | AUTO_SR_EN,
            0,
        )?;

        // Poll for measurement done
        self.poll_status(MEAS_M_DONE, delay)?;

        // Burst-read output registers
        let raw = self.read_raw_mag()?;

        Ok(MagData {
            x: to_signed(raw.x),
            y: to_signed(raw.y),
            z: to_signed(raw.z),
        })
    }

    /// Perform a SET/RESET compensated magnetic measurement.
    ///
    /// Executes SET → measure → RESET → measure, then computes H = (SET - RESET) / 2
    /// to cancel the sensor offset. Respects tSR = 1 ms between SET/RESET operations.
    pub fn measure_compensated(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<MagData, Error<I::Error>> {
        // SET pulse
        interface::modify_ctrl0(&mut self.i2c, self.addr, &mut self.cache, DO_SET, 0)?;
        delay.delay_ms(1); // tSR

        // Measure after SET
        interface::modify_ctrl0(&mut self.i2c, self.addr, &mut self.cache, TM_M, 0)?;
        self.poll_status(MEAS_M_DONE, delay)?;
        let set_raw = self.read_raw_mag()?;

        // RESET pulse
        interface::modify_ctrl0(&mut self.i2c, self.addr, &mut self.cache, DO_RESET, 0)?;
        delay.delay_ms(1); // tSR

        // Measure after RESET (MEAS_M_DONE was cleared by set_raw read above)
        interface::modify_ctrl0(&mut self.i2c, self.addr, &mut self.cache, TM_M, 0)?;
        self.poll_status(MEAS_M_DONE, delay)?;
        let reset_raw = self.read_raw_mag()?;

        // H = (SET - RESET) / 2
        Ok(MagData {
            x: (to_signed(set_raw.x) - to_signed(reset_raw.x)) / 2,
            y: (to_signed(set_raw.y) - to_signed(reset_raw.y)) / 2,
            z: (to_signed(set_raw.z) - to_signed(reset_raw.z)) / 2,
        })
    }

    /// Read the raw temperature register value.
    pub fn measure_temperature_raw(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<u8, Error<I::Error>> {
        // Trigger temperature measurement
        interface::modify_ctrl0(&mut self.i2c, self.addr, &mut self.cache, TM_T, 0)?;

        // Poll for temperature done
        self.poll_status(MEAS_T_DONE, delay)?;

        interface::read_reg(&mut self.i2c, self.addr, TOUT)
    }

    /// Measure temperature and convert to degrees Celsius.
    pub fn measure_temperature(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<f32, Error<I::Error>> {
        let raw = self.measure_temperature_raw(delay)?;
        Ok(tout_to_celsius(raw))
    }

    // --- Configuration ---

    /// Set the measurement bandwidth (BW1:BW0 in Ctrl1).
    pub fn set_bandwidth(&mut self, bw: Bandwidth) -> Result<(), Error<I::Error>> {
        interface::modify_ctrl1(
            &mut self.i2c,
            self.addr,
            &mut self.cache,
            bw.bits(),
            BW_MASK,
        )
    }

    /// Set the output data rate register.
    pub fn set_odr(&mut self, odr: u8) -> Result<(), Error<I::Error>> {
        interface::write_odr(&mut self.i2c, self.addr, &mut self.cache, odr)
    }

    /// Start continuous measurement mode.
    ///
    /// Writes the ODR value, enables CMM_FREQ_EN in Ctrl0, and sets CMM_EN in Ctrl2.
    /// Returns `BadParam` for an ODR of 0 or above what the current bandwidth
    /// sustains with automatic SET/RESET ([`Bandwidth::max_odr_auto_sr`]).
    pub fn start_continuous(&mut self, odr: u8) -> Result<(), Error<I::Error>> {
        if odr == 0 || odr > Bandwidth::from_bits(self.cache.ctrl1).max_odr_auto_sr() {
            return Err(Error::BadParam);
        }
        self.set_odr(odr)?;
        interface::modify_ctrl0(
            &mut self.i2c,
            self.addr,
            &mut self.cache,
            CMM_FREQ_EN | AUTO_SR_EN,
            0,
        )?;
        interface::modify_ctrl2(&mut self.i2c, self.addr, &mut self.cache, CMM_EN, 0)?;
        Ok(())
    }

    /// Stop continuous measurement mode by clearing CMM_EN in Ctrl2.
    pub fn stop_continuous(&mut self) -> Result<(), Error<I::Error>> {
        interface::modify_ctrl2(&mut self.i2c, self.addr, &mut self.cache, 0, CMM_EN)
    }

    /// Perform a software reset.
    ///
    /// Sets SW_RESET in Ctrl1, resets the shadow cache, and waits 20 ms
    /// for the device to complete its power-on sequence.
    pub fn soft_reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I::Error>> {
        interface::modify_ctrl1(&mut self.i2c, self.addr, &mut self.cache, SW_RESET, 0)?;
        self.cache = RegisterCache::default();
        delay.delay_ms(20);
        Ok(())
    }

    // --- Internal helpers ---

    /// Poll Status1 for the given flag, delaying 100 µs between attempts.
    fn poll_status(&mut self, flag: u8, delay: &mut impl DelayNs) -> Result<(), Error<I::Error>> {
        for _ in 0..MAX_POLL_ATTEMPTS {
            let status = interface::read_reg(&mut self.i2c, self.addr, STATUS1)?;
            if status & flag != 0 {
                return Ok(());
            }
            delay.delay_us(100);
        }
        Err(Error::Timeout)
    }

    /// Burst-read the 9 output registers and reconstruct 20-bit XYZ.
    fn read_raw_mag(&mut self) -> Result<RawMagData, Error<I::Error>> {
        let mut buf = [0u8; MAG_DATA_LEN];
        interface::read_regs(&mut self.i2c, self.addr, XOUT0, &mut buf)?;

        Ok(RawMagData {
            x: reconstruct_20bit(buf[0], buf[1], buf[6]),
            y: reconstruct_20bit(buf[2], buf[3], buf[7]),
            z: reconstruct_20bit(buf[4], buf[5], buf[8]),
        })
    }
}

#[cfg(test)]
extern crate alloc;

#[cfg(test)]
mod tests {
    use super::registers::*;
    use super::types::*;
    use alloc::vec;

    // --- Pure logic tests ---

    #[test]
    fn reconstruct_20bit_zero() {
        assert_eq!(reconstruct_20bit(0x00, 0x00, 0x00), 0);
    }

    #[test]
    fn reconstruct_20bit_max() {
        // All bits set: 0xFF<<12 | 0xFF<<4 | 0xF0>>4 = 0xFFFFF
        assert_eq!(reconstruct_20bit(0xFF, 0xFF, 0xF0), 0xF_FFFF);
    }

    #[test]
    fn reconstruct_20bit_null_field() {
        // Null field = 524288 = 0x80000
        // out0 = 0x80, out1 = 0x00, out2 = 0x00
        assert_eq!(reconstruct_20bit(0x80, 0x00, 0x00), 0x80000);
        assert_eq!(
            reconstruct_20bit(0x80, 0x00, 0x00),
            NULL_FIELD_OUTPUT as u32
        );
    }

    #[test]
    fn reconstruct_20bit_low_nibble() {
        // Only low nibble set: out2 = 0xF0 => lower 4 bits = 0x0F
        assert_eq!(reconstruct_20bit(0x00, 0x00, 0xF0), 0x0F);
    }

    #[test]
    fn to_signed_at_null() {
        assert_eq!(to_signed(NULL_FIELD_OUTPUT as u32), 0);
    }

    #[test]
    fn to_signed_above_null() {
        assert_eq!(to_signed(NULL_FIELD_OUTPUT as u32 + 100), 100);
    }

    #[test]
    fn to_signed_below_null() {
        assert_eq!(to_signed(NULL_FIELD_OUTPUT as u32 - 100), -100);
    }

    #[test]
    fn to_signed_zero_raw() {
        assert_eq!(to_signed(0), -NULL_FIELD_OUTPUT);
    }

    #[test]
    fn to_gauss_one_gauss() {
        let signed = COUNTS_PER_GAUSS as i32;
        let gauss = to_gauss(signed);
        assert!((gauss - 1.0).abs() < 1e-6);
    }

    #[test]
    fn to_gauss_zero() {
        assert!((to_gauss(0) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn to_gauss_negative() {
        let gauss = to_gauss(-(COUNTS_PER_GAUSS as i32));
        assert!((gauss - (-1.0)).abs() < 1e-6);
    }

    #[test]
    fn tout_to_celsius_zero() {
        assert!((tout_to_celsius(0) - (-75.0)).abs() < 1e-6);
    }

    #[test]
    fn tout_to_celsius_room_temp() {
        // 25°C: Tout = (25 + 75) / 0.8 = 125
        let temp = tout_to_celsius(125);
        assert!((temp - 25.0).abs() < 1e-6);
    }

    #[test]
    fn bandwidth_bits() {
        assert_eq!(Bandwidth::Bw00.bits(), 0b00);
        assert_eq!(Bandwidth::Bw01.bits(), 0b01);
        assert_eq!(Bandwidth::Bw10.bits(), 0b10);
        assert_eq!(Bandwidth::Bw11.bits(), 0b11);
    }

    #[test]
    fn bandwidth_measurement_time() {
        assert_eq!(Bandwidth::Bw00.measurement_time_us(), 6600);
        assert_eq!(Bandwidth::Bw01.measurement_time_us(), 3500);
        assert_eq!(Bandwidth::Bw10.measurement_time_us(), 2000);
        assert_eq!(Bandwidth::Bw11.measurement_time_us(), 1200);
    }

    // --- Mock I2C tests ---

    use super::Mmc5616wa;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};

    #[test]
    fn validate_success() {
        let expectations = [
            // read_reg(CHIP_ID) => write_read([0x21], buf) => 0xD2
            I2cTrans::write_read(DEFAULT_ADDRESS, vec![CHIP_ID], vec![CHIP_ID_VALUE]),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Mmc5616wa::new_default(i2c);

        dev.validate().unwrap();
        dev.destroy().done();
    }

    #[test]
    fn validate_wrong_id() {
        let expectations = [I2cTrans::write_read(
            DEFAULT_ADDRESS,
            vec![CHIP_ID],
            vec![0xAB],
        )];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Mmc5616wa::new_default(i2c);

        match dev.validate() {
            Err(crate::error::Error::InvalidChipId(0xAB)) => {}
            other => panic!("expected InvalidChipId(0xAB), got {:?}", other),
        }
        dev.destroy().done();
    }

    #[test]
    fn measure_magnetic_flow() {
        // Null field output bytes: X=0x80000, Y=0x80000, Z=0x80000 (all at null)
        // Xout0=0x80, Xout1=0x00, Yout0=0x80, Yout1=0x00, Zout0=0x80, Zout1=0x00
        // Xout2=0x00, Yout2=0x00, Zout2=0x00
        let expectations = [
            // Trigger: write ctrl0 = TM_M | AUTO_SR_EN = 0x21
            I2cTrans::write(DEFAULT_ADDRESS, vec![CTRL0, TM_M | AUTO_SR_EN]),
            // Poll status: first poll returns not done
            I2cTrans::write_read(DEFAULT_ADDRESS, vec![STATUS1], vec![0x00]),
            // Second poll returns done
            I2cTrans::write_read(DEFAULT_ADDRESS, vec![STATUS1], vec![MEAS_M_DONE]),
            // Burst read 9 bytes from XOUT0
            I2cTrans::write_read(
                DEFAULT_ADDRESS,
                vec![XOUT0],
                vec![0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00],
            ),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Mmc5616wa::new_default(i2c);
        let mut delay = embedded_hal_mock::eh1::delay::NoopDelay::new();

        let data = dev.measure_magnetic(&mut delay).unwrap();
        assert_eq!(data.x, 0);
        assert_eq!(data.y, 0);
        assert_eq!(data.z, 0);

        dev.destroy().done();
    }

    #[test]
    fn measure_magnetic_nonzero() {
        // X raw = 0x80100 => signed = 0x100 = 256
        // Xout0 = 0x80, Xout1 = 0x10, Xout2 = 0x00
        // Y and Z at null
        let expectations = [
            I2cTrans::write(DEFAULT_ADDRESS, vec![CTRL0, TM_M | AUTO_SR_EN]),
            I2cTrans::write_read(DEFAULT_ADDRESS, vec![STATUS1], vec![MEAS_M_DONE]),
            I2cTrans::write_read(
                DEFAULT_ADDRESS,
                vec![XOUT0],
                vec![0x80, 0x10, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00],
            ),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Mmc5616wa::new_default(i2c);
        let mut delay = embedded_hal_mock::eh1::delay::NoopDelay::new();

        let data = dev.measure_magnetic(&mut delay).unwrap();
        assert_eq!(data.x, 256);
        assert_eq!(data.y, 0);
        assert_eq!(data.z, 0);

        dev.destroy().done();
    }

    #[test]
    fn measure_temperature_flow() {
        let expectations = [
            // Trigger: write ctrl0 = TM_T = 0x02
            I2cTrans::write(DEFAULT_ADDRESS, vec![CTRL0, TM_T]),
            // Poll status done
            I2cTrans::write_read(DEFAULT_ADDRESS, vec![STATUS1], vec![MEAS_T_DONE]),
            // Read Tout
            I2cTrans::write_read(DEFAULT_ADDRESS, vec![TOUT], vec![125]),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut dev = Mmc5616wa::new_default(i2c);
        let mut delay = embedded_hal_mock::eh1::delay::NoopDelay::new();

        let temp = dev.measure_temperature(&mut delay).unwrap();
        assert!((temp - 25.0).abs() < 1e-6);

        dev.destroy().done();
    }

    #[test]
    fn timeout_on_poll() {
        // Status never becomes ready
        let mut expectations = vec![I2cTrans::write(
            DEFAULT_ADDRESS,
            vec![CTRL0, TM_M | AUTO_SR_EN],
        )];
        for _ in 0..500 {
            expectations.push(I2cTrans::write_read(
                DEFAULT_ADDRESS,
                vec![STATUS1],
                vec![0x00],
            ));
        }
        let i2c = I2cMock::new(&expectations);
        let mut dev = Mmc5616wa::new_default(i2c);
        let mut delay = embedded_hal_mock::eh1::delay::NoopDelay::new();

        match dev.measure_magnetic(&mut delay) {
            Err(crate::error::Error::Timeout) => {}
            other => panic!("expected Timeout, got {:?}", other),
        }

        dev.destroy().done();
    }
}
