use embedded_hal::i2c::I2c;

use crate::error::Error;
use crate::registers::{CTRL0, CTRL0_SELF_CLEARING, CTRL1, CTRL2, ODR};

/// Shadow cache for writable control registers.
///
/// Maintains local copies so that read-modify-write operations don't
/// require a bus read and don't accidentally clobber unrelated bits.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct RegisterCache {
    pub odr: u8,
    pub ctrl0: u8,
    pub ctrl1: u8,
    pub ctrl2: u8,
}

/// Write a single register.
pub(crate) fn write_reg<I: I2c>(
    i2c: &mut I,
    addr: u8,
    reg: u8,
    val: u8,
) -> Result<(), Error<I::Error>> {
    i2c.write(addr, &[reg, val])?;
    Ok(())
}

/// Read a single register.
pub(crate) fn read_reg<I: I2c>(i2c: &mut I, addr: u8, reg: u8) -> Result<u8, Error<I::Error>> {
    let mut buf = [0u8];
    i2c.write_read(addr, &[reg], &mut buf)?;
    Ok(buf[0])
}

/// Read multiple consecutive registers into `buf`.
pub(crate) fn read_regs<I: I2c>(
    i2c: &mut I,
    addr: u8,
    start: u8,
    buf: &mut [u8],
) -> Result<(), Error<I::Error>> {
    i2c.write_read(addr, &[start], buf)?;
    Ok(())
}

/// Modify Ctrl0 using the shadow cache, then clear self-clearing bits from the shadow.
pub(crate) fn modify_ctrl0<I: I2c>(
    i2c: &mut I,
    addr: u8,
    cache: &mut RegisterCache,
    set: u8,
    clear: u8,
) -> Result<(), Error<I::Error>> {
    cache.ctrl0 = (cache.ctrl0 & !clear) | set;
    write_reg(i2c, addr, CTRL0, cache.ctrl0)?;
    // Self-clearing bits (TM_M, TM_T, DO_SET, DO_RESET) are cleared by hardware
    // after the operation completes. Clear them from the shadow so subsequent
    // RMW operations don't accidentally re-trigger them.
    cache.ctrl0 &= !CTRL0_SELF_CLEARING;
    Ok(())
}

/// Modify Ctrl1 using the shadow cache.
pub(crate) fn modify_ctrl1<I: I2c>(
    i2c: &mut I,
    addr: u8,
    cache: &mut RegisterCache,
    set: u8,
    clear: u8,
) -> Result<(), Error<I::Error>> {
    cache.ctrl1 = (cache.ctrl1 & !clear) | set;
    write_reg(i2c, addr, CTRL1, cache.ctrl1)?;
    Ok(())
}

/// Modify Ctrl2 using the shadow cache.
pub(crate) fn modify_ctrl2<I: I2c>(
    i2c: &mut I,
    addr: u8,
    cache: &mut RegisterCache,
    set: u8,
    clear: u8,
) -> Result<(), Error<I::Error>> {
    cache.ctrl2 = (cache.ctrl2 & !clear) | set;
    write_reg(i2c, addr, CTRL2, cache.ctrl2)?;
    Ok(())
}

/// Write the ODR register and update the shadow cache.
pub(crate) fn write_odr<I: I2c>(
    i2c: &mut I,
    addr: u8,
    cache: &mut RegisterCache,
    val: u8,
) -> Result<(), Error<I::Error>> {
    cache.odr = val;
    write_reg(i2c, addr, ODR, val)
}
