/// Driver error type, generic over the I2C bus error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C bus error.
    I2c(E),
    /// A polling operation timed out waiting for a status flag.
    Timeout,
    /// An invalid parameter was provided.
    BadParam,
    /// The Chip ID register returned an unexpected value.
    InvalidChipId(u8),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::I2c(e)
    }
}
