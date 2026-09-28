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
