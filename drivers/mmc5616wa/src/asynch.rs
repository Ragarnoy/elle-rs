//! Async driver (`async` feature) for continuous-mode use: reset, identify,
//! start continuous measurement, read. Same register sequences and shadow-cache
//! rules as the blocking [`crate::Mmc5616wa`].

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use crate::error::Error;
use crate::interface::RegisterCache;
use crate::registers::*;
use crate::types::*;

/// Async driver for the MEMSIC MMC5616WA 3-axis magnetometer.
pub struct Mmc5616waAsync<I> {
    i2c: I,
    addr: u8,
    cache: RegisterCache,
}

impl<I: I2c> Mmc5616waAsync<I> {
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

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<I::Error>> {
        self.i2c.write(self.addr, &[reg, val]).await?;
        Ok(())
    }

    async fn read_regs(&mut self, start: u8, buf: &mut [u8]) -> Result<(), Error<I::Error>> {
        self.i2c.write_read(self.addr, &[start], buf).await?;
        Ok(())
    }

    /// Wait for the power-on time (tOp = 5 ms) and reset the shadow cache.
    pub async fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I::Error>> {
        delay.delay_ms(5).await;
        self.cache = RegisterCache::default();
        Ok(())
    }

    /// Read the Chip ID register and verify it matches the expected value (0xD2).
    pub async fn validate(&mut self) -> Result<(), Error<I::Error>> {
        let mut id = [0u8];
        self.read_regs(CHIP_ID, &mut id).await?;
        if id[0] != CHIP_ID_VALUE {
            return Err(Error::InvalidChipId(id[0]));
        }
        Ok(())
    }

    /// Software reset: set SW_RESET in Ctrl1, reset the shadow cache, and wait
    /// 20 ms for the power-on sequence.
    pub async fn soft_reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I::Error>> {
        self.cache.ctrl1 |= SW_RESET;
        self.write_reg(CTRL1, self.cache.ctrl1).await?;
        self.cache = RegisterCache::default();
        delay.delay_ms(20).await;
        Ok(())
    }

    /// Read Product ID 1 (0x11 on this part).
    pub async fn product_id(&mut self) -> Result<u8, Error<I::Error>> {
        let mut id = [0u8];
        self.read_regs(PRODUCT_ID, &mut id).await?;
        Ok(id[0])
    }

    /// Read Status1.
    pub async fn status(&mut self) -> Result<u8, Error<I::Error>> {
        let mut s = [0u8];
        self.read_regs(STATUS1, &mut s).await?;
        Ok(s[0])
    }

    /// Saturation self-test (datasheet v1.6 p. 19): thresholds at 80 % of the
    /// factory values, one measurement with Auto_st_en, then `Sat_sensor` low
    /// means pass. Run it before continuous mode. Waits up to ~20 ms for the
    /// measurement; [`SelfTest::completed`] says whether it reported done.
    pub async fn self_test(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<SelfTest, Error<I::Error>> {
        let mut factory = [0u8; 3];
        self.read_regs(ST_X, &mut factory).await?;
        for (i, v) in factory.iter().enumerate() {
            let threshold = (u16::from(*v) * 4 / 5) as u8;
            self.write_reg(ST_X_TH + i as u8, threshold).await?;
        }
        // TM_M and Auto_st_en both self-clear; the shadow keeps neither.
        self.write_reg(CTRL0, self.cache.ctrl0 | TM_M | AUTO_ST_EN)
            .await?;
        // One measurement is 6.6 ms at BW00; reading Status1 clears the
        // done flag, so the read that sees it also gives Sat_sensor.
        let mut status = 0;
        for _ in 0..10 {
            delay.delay_ms(2).await;
            status = self.status().await?;
            if status & MEAS_M_DONE_INT != 0 {
                return Ok(SelfTest {
                    factory,
                    status,
                    completed: true,
                });
            }
        }
        Ok(SelfTest {
            factory,
            status,
            completed: false,
        })
    }

    /// Start continuous measurement mode: write the ODR, enable CMM_FREQ_EN and
    /// AUTO_SR_EN in Ctrl0, then CMM_EN in Ctrl2. Returns `BadParam` for an ODR
    /// of 0 or above what the current bandwidth sustains with automatic
    /// SET/RESET ([`Bandwidth::max_odr_auto_sr`]).
    pub async fn start_continuous(&mut self, odr: u8) -> Result<(), Error<I::Error>> {
        if odr == 0 || odr > Bandwidth::from_bits(self.cache.ctrl1).max_odr_auto_sr() {
            return Err(Error::BadParam);
        }
        self.cache.odr = odr;
        self.write_reg(ODR, odr).await?;
        self.cache.ctrl0 |= CMM_FREQ_EN | AUTO_SR_EN;
        self.write_reg(CTRL0, self.cache.ctrl0).await?;
        // Self-clearing bits must never be re-written from the shadow.
        self.cache.ctrl0 &= !CTRL0_SELF_CLEARING;
        self.cache.ctrl2 |= CMM_EN;
        self.write_reg(CTRL2, self.cache.ctrl2).await?;
        Ok(())
    }

    /// Read back the ODR register.
    pub async fn odr(&mut self) -> Result<u8, Error<I::Error>> {
        let mut v = [0u8];
        self.read_regs(ODR, &mut v).await?;
        Ok(v[0])
    }

    /// Read the latest magnetic output registers (continuous mode) as signed counts.
    pub async fn read_magnetic(&mut self) -> Result<MagData, Error<I::Error>> {
        let mut buf = [0u8; MAG_DATA_LEN];
        self.read_regs(XOUT0, &mut buf).await?;
        Ok(MagData {
            x: to_signed(reconstruct_20bit(buf[0], buf[1], buf[6])),
            y: to_signed(reconstruct_20bit(buf[2], buf[3], buf[7])),
            z: to_signed(reconstruct_20bit(buf[4], buf[5], buf[8])),
        })
    }
}
