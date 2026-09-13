/// Driver error type, generic over the UART bus error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// UART bus error.
    Uart(E),
    /// UBX checksum verification failed.
    Checksum,
    /// Received frame exceeds the internal buffer size.
    FrameTooLarge,
    /// A blocking read timed out (no data available).
    Timeout,
    /// A GPIO operation on the RESET_N pin failed.
    ///
    /// The underlying pin error is not carried: `OutputPin::Error` is a separate
    /// type parameter from the UART error, and pin failures here are not
    /// actionable beyond "the reset did not happen".
    Pin,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::Uart(e)
    }
}
