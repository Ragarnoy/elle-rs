//! Async driver over `embedded-io-async`.
//!
//! This is the API the firmware uses. The blocking [`crate::SamM10q`] is kept
//! for host-side tooling and tests.
//!
//! # Choosing a UART
//!
//! On embassy-rp, the DMA-backed `UartRx<'d, Async>` does **not** implement
//! `embedded_io_async::Read` — its inherent `read()` fills the entire buffer and
//! returns `Result<(), Error>`, so a short read blocks until enough bytes
//! arrive. Use `BufferedUart` / `BufferedUartRx` instead: they implement the
//! async traits with correct partial-read semantics, which is what a bursty
//! GNSS stream needs.

use embedded_io_async::{Read, Write};

use crate::decoder::{Decoder, FeedResult};
use crate::error::Error;
use crate::types::{Frame, MAX_FRAME_SIZE};
use crate::ubx;

/// Size of the driver's UART read buffer.
///
/// One NAV-PVT frame is 100 bytes on the wire; this holds a frame plus change,
/// so a 5 Hz PVT + GGA stream is typically drained in one or two reads.
const RX_CHUNK: usize = 128;

/// Outcome of waiting for a configuration acknowledgement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AckStatus {
    /// The module accepted the message (UBX-ACK-ACK).
    Ack,
    /// The module rejected the message (UBX-ACK-NAK) — usually an unknown
    /// configuration key, or a layers bitfield naming no valid destination.
    Nak,
}

/// Async driver for the u-blox SAM-M10Q over UART.
///
/// Generic over separate RX and TX halves so a split UART can be used. Both
/// halves must report the same error type, which is the case for a UART split
/// into its two directions.
pub struct SamM10q<RX, TX> {
    rx: RX,
    tx: TX,
    decoder: Decoder,
    buf: [u8; RX_CHUNK],
    /// Number of valid bytes in `buf`.
    len: usize,
    /// Read cursor into `buf`.
    pos: usize,
}

impl<RX, TX> SamM10q<RX, TX> {
    /// Wrap the two halves of a UART.
    pub fn new(rx: RX, tx: TX) -> Self {
        Self {
            rx,
            tx,
            decoder: Decoder::new(),
            buf: [0; RX_CHUNK],
            len: 0,
            pos: 0,
        }
    }

    /// Consume the driver and return the UART halves.
    pub fn destroy(self) -> (RX, TX) {
        (self.rx, self.tx)
    }

    /// Discard any partially decoded frame and buffered input.
    ///
    /// Call this after changing baud rate: bytes captured at the old rate are
    /// garbage at the new one.
    pub fn reset_decoder(&mut self) {
        self.decoder.reset();
        self.len = 0;
        self.pos = 0;
    }
}

impl<RX, TX, E> SamM10q<RX, TX>
where
    RX: Read<Error = E>,
    TX: Write<Error = E>,
    E: embedded_io_async::Error,
{
    /// Wait for the next complete frame.
    ///
    /// Decode errors (a bad checksum, an oversized frame) are skipped rather
    /// than surfaced — they mean one corrupt frame, not a broken link, and the
    /// decoder resynchronises on its own.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Uart`] on a read failure, or [`Error::Timeout`] if the
    /// port reports end-of-stream.
    pub async fn next_frame(&mut self) -> Result<Frame<'_>, Error<E>> {
        loop {
            if self.pos < self.len {
                let byte = self.buf[self.pos];
                self.pos += 1;
                if self.decoder.feed(byte) == FeedResult::FrameReady {
                    break;
                }
                continue;
            }

            self.len = self.rx.read(&mut self.buf).await.map_err(Error::Uart)?;
            self.pos = 0;
            if self.len == 0 {
                return Err(Error::Timeout);
            }
        }

        // `FrameReady` guarantees the decoder holds a frame; the `None` arm is
        // unreachable and is mapped to an error rather than a panic.
        self.decoder.take_frame().ok_or(Error::Timeout)
    }

    /// Send a UBX message, framing it with sync bytes, header and checksum.
    ///
    /// # Errors
    ///
    /// Returns [`Error::FrameTooLarge`] if the payload exceeds the internal
    /// frame buffer, or [`Error::Uart`] on a write failure.
    pub async fn send_ubx(&mut self, class: u8, id: u8, payload: &[u8]) -> Result<(), Error<E>> {
        let mut frame = [0u8; MAX_FRAME_SIZE + ubx::HEADER_SIZE + ubx::CHECKSUM_SIZE];
        let len = ubx::build_frame(&mut frame, class, id, payload).ok_or(Error::FrameTooLarge)?;
        self.tx.write_all(&frame[..len]).await.map_err(Error::Uart)?;
        self.tx.flush().await.map_err(Error::Uart)
    }

    /// Send a UBX-CFG-VALSET applying `items` to the given storage `layers`.
    ///
    /// Does not wait for the acknowledgement — follow with [`wait_for_ack`] to
    /// confirm the module accepted it.
    ///
    /// [`wait_for_ack`]: Self::wait_for_ack
    ///
    /// # Errors
    ///
    /// Returns [`Error::FrameTooLarge`] if the items exceed one VALSET message,
    /// or [`Error::Uart`] on a write failure.
    pub async fn send_valset(
        &mut self,
        layers: u8,
        items: &[ubx::cfg::CfgVal],
    ) -> Result<(), Error<E>> {
        let mut frame = [0u8; ubx::cfg::MAX_VALSET_FRAME];
        let len = ubx::cfg::build_valset(&mut frame, layers, items).ok_or(Error::FrameTooLarge)?;
        self.tx.write_all(&frame[..len]).await.map_err(Error::Uart)?;
        self.tx.flush().await.map_err(Error::Uart)
    }

    /// Consume frames until the module acknowledges the message `class`/`id`.
    ///
    /// Other frames seen along the way are discarded. This never gives up on its
    /// own: wrap the call in the caller's timeout (for example
    /// `embassy_time::with_timeout`) so a module that answers nothing at all
    /// cannot stall the task.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Uart`] on a read failure, or [`Error::Timeout`] if the
    /// port reports end-of-stream.
    pub async fn wait_for_ack(&mut self, class: u8, id: u8) -> Result<AckStatus, Error<E>> {
        loop {
            let status = {
                let Frame::Ubx(frame) = self.next_frame().await? else {
                    continue;
                };
                if frame.class != ubx::class::ACK || frame.payload.len() < 2 {
                    continue;
                }
                // ACK payload is the class and id of the message being answered.
                if frame.payload[0] != class || frame.payload[1] != id {
                    continue;
                }
                match frame.id {
                    ubx::ack::ACK => AckStatus::Ack,
                    ubx::ack::NAK => AckStatus::Nak,
                    _ => continue,
                }
            };
            return Ok(status);
        }
    }
}

#[cfg(test)]
extern crate alloc;

#[cfg(test)]
mod tests {
    use alloc::vec;
    use alloc::vec::Vec;
    use futures::executor::block_on;

    use super::*;
    use embedded_io_async::ErrorType;

    #[derive(Debug)]
    struct MockError;

    impl core::fmt::Display for MockError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            write!(f, "MockError")
        }
    }

    impl core::error::Error for MockError {}

    impl embedded_io_async::Error for MockError {
        fn kind(&self) -> embedded_io_async::ErrorKind {
            embedded_io_async::ErrorKind::Other
        }
    }

    /// Reader that hands out the canned stream in small chunks, so the driver's
    /// refill path is exercised rather than swallowing everything in one read.
    struct MockRx {
        data: Vec<u8>,
        pos: usize,
        chunk: usize,
    }

    impl MockRx {
        fn new(data: &[u8], chunk: usize) -> Self {
            Self {
                data: data.to_vec(),
                pos: 0,
                chunk,
            }
        }
    }

    impl ErrorType for MockRx {
        type Error = MockError;
    }

    impl Read for MockRx {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let remaining = self.data.len() - self.pos;
            let n = remaining.min(buf.len()).min(self.chunk);
            buf[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
            self.pos += n;
            Ok(n)
        }
    }

    #[derive(Default)]
    struct MockTx {
        written: Vec<u8>,
    }

    impl ErrorType for MockTx {
        type Error = MockError;
    }

    impl Write for MockTx {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.written.extend_from_slice(buf);
            Ok(buf.len())
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    fn build_ubx(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = vec![0u8; ubx::HEADER_SIZE + payload.len() + ubx::CHECKSUM_SIZE];
        let len = ubx::build_frame(&mut buf, class, id, payload).unwrap();
        buf.truncate(len);
        buf
    }

    const GGA: &[u8] = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";

    #[test]
    fn reads_frames_across_chunk_boundaries() {
        let mut stream = Vec::new();
        stream.extend_from_slice(GGA);
        stream.extend_from_slice(&build_ubx(ubx::class::NAV, ubx::nav::PVT, &[1, 2, 3, 4]));
        // A chunk size of 7 guarantees both frames straddle several refills.
        let mut gnss = SamM10q::new(MockRx::new(&stream, 7), MockTx::default());

        block_on(async {
            match gnss.next_frame().await.unwrap() {
                Frame::Nmea(f) => assert!(f.raw.starts_with(b"$GPGGA")),
                other => panic!("expected NMEA, got {other:?}"),
            }
            match gnss.next_frame().await.unwrap() {
                Frame::Ubx(f) => {
                    assert_eq!(f.class, ubx::class::NAV);
                    assert_eq!(f.id, ubx::nav::PVT);
                    assert_eq!(f.payload, &[1, 2, 3, 4]);
                }
                other => panic!("expected UBX, got {other:?}"),
            }
        });
    }

    #[test]
    fn skips_corrupt_frames_without_dropping_the_next_one() {
        let mut stream = Vec::new();
        // Corrupt NMEA checksum, then a good sentence.
        stream.extend_from_slice(b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4E\r\n");
        stream.extend_from_slice(GGA);
        let mut gnss = SamM10q::new(MockRx::new(&stream, 16), MockTx::default());

        block_on(async {
            match gnss.next_frame().await.unwrap() {
                Frame::Nmea(f) => {
                    assert!(f.raw.ends_with(b"*4F"), "should be the valid sentence");
                }
                other => panic!("expected NMEA, got {other:?}"),
            }
        });
    }

    #[test]
    fn end_of_stream_reports_timeout() {
        let mut gnss = SamM10q::new(MockRx::new(&[], 8), MockTx::default());
        block_on(async {
            assert!(matches!(gnss.next_frame().await, Err(Error::Timeout)));
        });
    }

    #[test]
    fn send_ubx_writes_a_valid_frame() {
        let mut gnss = SamM10q::new(MockRx::new(&[], 8), MockTx::default());
        block_on(async {
            gnss.send_ubx(ubx::class::CFG, ubx::cfg::RST, &[0x00, 0x00, 0x02, 0x00])
                .await
                .unwrap();
        });

        let (_, tx) = gnss.destroy();
        let w = &tx.written;
        assert_eq!(w[0], ubx::SYNC1);
        assert_eq!(w[1], ubx::SYNC2);
        assert_eq!(w[2], ubx::class::CFG);
        assert_eq!(w[3], ubx::cfg::RST);
        let (ck_a, ck_b) = ubx::checksum(&w[2..w.len() - 2]);
        assert_eq!(w[w.len() - 2], ck_a);
        assert_eq!(w[w.len() - 1], ck_b);
    }

    #[test]
    fn send_valset_writes_the_configured_keys() {
        let mut gnss = SamM10q::new(MockRx::new(&[], 8), MockTx::default());
        block_on(async {
            gnss.send_valset(
                ubx::cfg::LAYER_RAM,
                &[ubx::cfg::CfgVal::Uart1Baudrate(115_200)],
            )
            .await
            .unwrap();
        });

        let (_, tx) = gnss.destroy();
        let w = &tx.written;
        assert_eq!(w[3], ubx::cfg::VALSET);
        assert_eq!(w[7], ubx::cfg::LAYER_RAM);
        assert_eq!(
            u32::from_le_bytes([w[10], w[11], w[12], w[13]]),
            0x4052_0001
        );
        assert_eq!(u32::from_le_bytes([w[14], w[15], w[16], w[17]]), 115_200);
    }

    #[test]
    fn wait_for_ack_matches_the_right_message() {
        let mut stream = Vec::new();
        // Noise the ACK scan must skip: an unrelated sentence, and an ACK for a
        // different message than the one we asked about.
        stream.extend_from_slice(GGA);
        stream.extend_from_slice(&build_ubx(
            ubx::class::ACK,
            ubx::ack::ACK,
            &[ubx::class::CFG, ubx::cfg::RST],
        ));
        stream.extend_from_slice(&build_ubx(
            ubx::class::ACK,
            ubx::ack::ACK,
            &[ubx::class::CFG, ubx::cfg::VALSET],
        ));
        let mut gnss = SamM10q::new(MockRx::new(&stream, 13), MockTx::default());

        block_on(async {
            let status = gnss
                .wait_for_ack(ubx::class::CFG, ubx::cfg::VALSET)
                .await
                .unwrap();
            assert_eq!(status, AckStatus::Ack);
        });
    }

    #[test]
    fn wait_for_ack_reports_a_nak() {
        let stream = build_ubx(
            ubx::class::ACK,
            ubx::ack::NAK,
            &[ubx::class::CFG, ubx::cfg::VALSET],
        );
        let mut gnss = SamM10q::new(MockRx::new(&stream, 32), MockTx::default());

        block_on(async {
            let status = gnss
                .wait_for_ack(ubx::class::CFG, ubx::cfg::VALSET)
                .await
                .unwrap();
            assert_eq!(status, AckStatus::Nak);
        });
    }
}
