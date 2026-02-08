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
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::Uart(e)
    }
}
