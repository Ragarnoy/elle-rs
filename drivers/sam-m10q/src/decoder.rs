use crate::types::{Frame, MAX_FRAME_SIZE, NmeaFrame, UbxFrame};
use crate::ubx;

/// Internal decoder state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// Waiting for a frame start byte ('$' or 0xB5).
    Idle,
    /// Accumulating NMEA sentence bytes after '$'.
    NmeaBody,
    /// Received UBX SYNC1 (0xB5), expecting SYNC2 (0x62).
    UbxSync2,
    /// Receiving UBX header bytes (class, id, len_lo, len_hi).
    UbxHeader { header_idx: u8 },
    /// Receiving UBX payload bytes.
    UbxPayload,
    /// Receiving UBX checksum bytes (ck_a, ck_b).
    UbxChecksum { ck_idx: u8 },
}

/// Tracks which frame type was last decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameType {
    None,
    Nmea,
    Ubx,
}

/// Result of feeding a single byte to the decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeedResult {
    /// More data needed.
    Pending,
    /// A complete frame is ready — call `take_frame()`.
    FrameReady,
    /// A decode error occurred. The decoder has been reset to Idle.
    Error(DecodeError),
}

/// Decode errors that can occur within the state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DecodeError {
    /// UBX checksum mismatch.
    UbxChecksum,
    /// Frame exceeded the internal buffer.
    FrameTooLarge,
    /// Invalid UBX sync byte (0xB5 not followed by 0x62).
    InvalidSync,
}

/// Streaming decoder for interleaved UBX and NMEA frames.
///
/// Feed bytes one at a time via `feed()`. When `FeedResult::FrameReady` is
/// returned, call `take_frame()` to borrow the decoded frame data.
pub struct Decoder {
    state: State,
    buf: [u8; MAX_FRAME_SIZE],
    pos: usize,
    last_frame: FrameType,
    // UBX header fields (populated during UbxHeader state)
    ubx_class: u8,
    ubx_id: u8,
    ubx_payload_len: u16,
    // UBX checksum accumulators
    ubx_ck_a: u8,
    ubx_ck_b: u8,
    // Received checksum bytes
    ubx_rx_ck_a: u8,
}

impl Decoder {
    /// Create a new decoder in the Idle state.
    pub fn new() -> Self {
        Self {
            state: State::Idle,
            buf: [0; MAX_FRAME_SIZE],
            pos: 0,
            last_frame: FrameType::None,
            ubx_class: 0,
            ubx_id: 0,
            ubx_payload_len: 0,
            ubx_ck_a: 0,
            ubx_ck_b: 0,
            ubx_rx_ck_a: 0,
        }
    }

    /// Reset the decoder to Idle, discarding any partial frame.
    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.pos = 0;
        self.last_frame = FrameType::None;
    }

    /// Feed a single byte into the decoder.
    pub fn feed(&mut self, byte: u8) -> FeedResult {
        match self.state {
            State::Idle => self.feed_idle(byte),
            State::NmeaBody => self.feed_nmea(byte),
            State::UbxSync2 => self.feed_ubx_sync2(byte),
            State::UbxHeader { header_idx } => self.feed_ubx_header(byte, header_idx),
            State::UbxPayload => self.feed_ubx_payload(byte),
            State::UbxChecksum { ck_idx } => self.feed_ubx_checksum(byte, ck_idx),
        }
    }

    /// Retrieve the last decoded frame.
    ///
    /// Must only be called after `feed()` returns `FeedResult::FrameReady`.
    /// The returned `Frame` borrows from the decoder's internal buffer —
    /// it must be consumed before the next call to `feed()`.
    pub fn take_frame(&self) -> Frame<'_> {
        match self.last_frame {
            FrameType::Ubx => Frame::Ubx(UbxFrame {
                class: self.ubx_class,
                id: self.ubx_id,
                payload: &self.buf[..self.ubx_payload_len as usize],
            }),
            FrameType::Nmea => {
                let raw = &self.buf[..self.pos];
                let parsed = core::str::from_utf8(raw)
                    .ok()
                    .and_then(|s| nmea::parse_bytes(s.as_bytes()).ok());
                Frame::Nmea(NmeaFrame { raw, parsed })
            }
            FrameType::None => panic!("take_frame() called without a ready frame"),
        }
    }

    // --- State handlers ---

    fn feed_idle(&mut self, byte: u8) -> FeedResult {
        match byte {
            b'$' => {
                self.pos = 0;
                self.buf[0] = byte;
                self.pos = 1;
                self.state = State::NmeaBody;
                FeedResult::Pending
            }
            ubx::SYNC1 => {
                self.state = State::UbxSync2;
                FeedResult::Pending
            }
            _ => {
                // Garbage byte, stay in Idle
                FeedResult::Pending
            }
        }
    }

    fn feed_nmea(&mut self, byte: u8) -> FeedResult {
        match byte {
            b'\n' => {
                // Strip trailing \r if present
                if self.pos > 0 && self.buf[self.pos - 1] == b'\r' {
                    self.pos -= 1;
                }
                self.state = State::Idle;
                self.last_frame = FrameType::Nmea;
                FeedResult::FrameReady
            }
            _ => {
                if self.pos >= MAX_FRAME_SIZE {
                    self.state = State::Idle;
                    self.pos = 0;
                    return FeedResult::Error(DecodeError::FrameTooLarge);
                }
                self.buf[self.pos] = byte;
                self.pos += 1;
                FeedResult::Pending
            }
        }
    }

    fn feed_ubx_sync2(&mut self, byte: u8) -> FeedResult {
        if byte == ubx::SYNC2 {
            // Valid sync sequence, start header
            self.ubx_class = 0;
            self.ubx_id = 0;
            self.ubx_payload_len = 0;
            self.ubx_ck_a = 0;
            self.ubx_ck_b = 0;
            self.ubx_rx_ck_a = 0;
            self.pos = 0;
            self.state = State::UbxHeader { header_idx: 0 };
            FeedResult::Pending
        } else {
            // False sync — re-feed byte through Idle
            self.state = State::Idle;
            self.feed_idle(byte)
        }
    }

    fn feed_ubx_header(&mut self, byte: u8, header_idx: u8) -> FeedResult {
        // Accumulate checksum over all header bytes
        self.ubx_ck_a = self.ubx_ck_a.wrapping_add(byte);
        self.ubx_ck_b = self.ubx_ck_b.wrapping_add(self.ubx_ck_a);

        match header_idx {
            0 => {
                self.ubx_class = byte;
                self.state = State::UbxHeader { header_idx: 1 };
            }
            1 => {
                self.ubx_id = byte;
                self.state = State::UbxHeader { header_idx: 2 };
            }
            2 => {
                self.ubx_payload_len = byte as u16;
                self.state = State::UbxHeader { header_idx: 3 };
            }
            3 => {
                self.ubx_payload_len |= (byte as u16) << 8;
                if self.ubx_payload_len as usize > MAX_FRAME_SIZE {
                    self.state = State::Idle;
                    self.pos = 0;
                    return FeedResult::Error(DecodeError::FrameTooLarge);
                }
                if self.ubx_payload_len == 0 {
                    self.state = State::UbxChecksum { ck_idx: 0 };
                } else {
                    self.pos = 0;
                    self.state = State::UbxPayload;
                }
            }
            _ => unreachable!(),
        }
        FeedResult::Pending
    }

    fn feed_ubx_payload(&mut self, byte: u8) -> FeedResult {
        self.ubx_ck_a = self.ubx_ck_a.wrapping_add(byte);
        self.ubx_ck_b = self.ubx_ck_b.wrapping_add(self.ubx_ck_a);

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos >= self.ubx_payload_len as usize {
            self.state = State::UbxChecksum { ck_idx: 0 };
        }
        FeedResult::Pending
    }

    fn feed_ubx_checksum(&mut self, byte: u8, ck_idx: u8) -> FeedResult {
        match ck_idx {
            0 => {
                self.ubx_rx_ck_a = byte;
                self.state = State::UbxChecksum { ck_idx: 1 };
                FeedResult::Pending
            }
            1 => {
                self.state = State::Idle;
                if self.ubx_rx_ck_a == self.ubx_ck_a && byte == self.ubx_ck_b {
                    self.last_frame = FrameType::Ubx;
                    FeedResult::FrameReady
                } else {
                    FeedResult::Error(DecodeError::UbxChecksum)
                }
            }
            _ => unreachable!(),
        }
    }
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
extern crate alloc;

#[cfg(test)]
mod tests {
    use alloc::vec;
    use alloc::vec::Vec;

    use super::*;
    use crate::ubx;

    /// Helper: feed all bytes and collect FeedResults.
    fn feed_all(decoder: &mut Decoder, data: &[u8]) -> Vec<FeedResult> {
        data.iter().map(|&b| decoder.feed(b)).collect()
    }

    /// Helper: feed all bytes, returning the index of the first FrameReady.
    fn feed_until_frame(decoder: &mut Decoder, data: &[u8]) -> Option<usize> {
        for (i, &b) in data.iter().enumerate() {
            if decoder.feed(b) == FeedResult::FrameReady {
                return Some(i);
            }
        }
        None
    }

    /// Build a valid UBX frame for testing.
    fn build_ubx(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = vec![0u8; ubx::HEADER_SIZE + payload.len() + ubx::CHECKSUM_SIZE];
        ubx::build_frame(&mut buf, class, id, payload).unwrap();
        buf
    }

    #[test]
    fn simple_nmea_sentence() {
        let mut dec = Decoder::new();
        let sentence = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47\r\n";
        feed_until_frame(&mut dec, sentence).expect("should produce a frame");

        match dec.take_frame() {
            Frame::Nmea(f) => {
                // raw should not contain \r\n
                assert!(f.raw.starts_with(b"$GPGGA"));
                assert!(f.raw.ends_with(b"*47"));
                assert!(!f.raw.contains(&b'\r'));
                assert!(!f.raw.contains(&b'\n'));
            }
            _ => panic!("expected NMEA frame"),
        }
    }

    #[test]
    fn ubx_ack_frame() {
        let mut dec = Decoder::new();
        let frame = build_ubx(ubx::class::ACK, ubx::ack::ACK, &[0x06, 0x01]);
        feed_until_frame(&mut dec, &frame).expect("should produce a frame");

        match dec.take_frame() {
            Frame::Ubx(f) => {
                assert_eq!(f.class, ubx::class::ACK);
                assert_eq!(f.id, ubx::ack::ACK);
                assert_eq!(f.payload, &[0x06, 0x01]);
            }
            _ => panic!("expected UBX frame"),
        }
    }

    #[test]
    fn ubx_bad_checksum() {
        let mut dec = Decoder::new();
        let mut frame = build_ubx(ubx::class::ACK, ubx::ack::ACK, &[0x06, 0x01]);
        // Corrupt the checksum
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;

        let results = feed_all(&mut dec, &frame);
        assert!(results.contains(&FeedResult::Error(DecodeError::UbxChecksum)));
    }

    #[test]
    fn garbage_before_frame() {
        let mut dec = Decoder::new();
        let mut data = vec![0xFF, 0x00, 0x42, 0x13]; // garbage
        data.extend_from_slice(b"$GPRMC,test*00\r\n");

        feed_until_frame(&mut dec, &data).expect("should produce a frame");
        match dec.take_frame() {
            Frame::Nmea(f) => {
                assert!(f.raw.starts_with(b"$GPRMC"));
            }
            _ => panic!("expected NMEA frame"),
        }
    }

    #[test]
    fn mixed_nmea_and_ubx() {
        let mut dec = Decoder::new();

        // First: an NMEA sentence
        let nmea = b"$GPGGA,test*00\r\n";
        feed_until_frame(&mut dec, nmea).expect("NMEA frame");
        match dec.take_frame() {
            Frame::Nmea(_) => {}
            _ => panic!("expected NMEA"),
        }

        // Second: a UBX frame
        let ubx_data = build_ubx(ubx::class::NAV, ubx::nav::PVT, &[1, 2, 3, 4]);
        feed_until_frame(&mut dec, &ubx_data).expect("UBX frame");
        match dec.take_frame() {
            Frame::Ubx(f) => {
                assert_eq!(f.class, ubx::class::NAV);
                assert_eq!(f.id, ubx::nav::PVT);
                assert_eq!(f.payload, &[1, 2, 3, 4]);
            }
            _ => panic!("expected UBX"),
        }
    }

    #[test]
    fn split_across_reads() {
        let mut dec = Decoder::new();
        let frame = build_ubx(ubx::class::ACK, ubx::ack::ACK, &[0x06, 0x01]);

        // Feed first half
        let mid = frame.len() / 2;
        for &b in &frame[..mid] {
            assert_eq!(dec.feed(b), FeedResult::Pending);
        }

        // Feed second half
        feed_until_frame(&mut dec, &frame[mid..]).expect("should complete");
        match dec.take_frame() {
            Frame::Ubx(f) => {
                assert_eq!(f.payload, &[0x06, 0x01]);
            }
            _ => panic!("expected UBX"),
        }
    }

    #[test]
    fn false_ubx_sync() {
        let mut dec = Decoder::new();
        // 0xB5 followed by something other than 0x62
        let data = [ubx::SYNC1, b'$'];

        // Feed 0xB5 -> goes to UbxSync2
        assert_eq!(dec.feed(data[0]), FeedResult::Pending);
        // Feed '$' -> false sync, re-feeds through Idle, starts NMEA
        assert_eq!(dec.feed(data[1]), FeedResult::Pending);

        // Now we should be in NMEA mode — feed rest of sentence
        let rest = b"GPGGA,test*00\r\n";
        feed_until_frame(&mut dec, rest).expect("NMEA frame");
        match dec.take_frame() {
            Frame::Nmea(f) => {
                assert!(f.raw.starts_with(b"$GPGGA"));
            }
            _ => panic!("expected NMEA"),
        }
    }

    #[test]
    fn frame_too_large_nmea() {
        let mut dec = Decoder::new();
        // Start NMEA
        assert_eq!(dec.feed(b'$'), FeedResult::Pending);
        // Feed MAX_FRAME_SIZE - 1 bytes (the '$' already took one slot)
        for _ in 1..MAX_FRAME_SIZE {
            assert_eq!(dec.feed(b'A'), FeedResult::Pending);
        }
        // Next byte should trigger FrameTooLarge
        assert_eq!(dec.feed(b'A'), FeedResult::Error(DecodeError::FrameTooLarge));
    }

    #[test]
    fn frame_too_large_ubx() {
        let mut dec = Decoder::new();
        // Craft a UBX header claiming a payload larger than MAX_FRAME_SIZE
        assert_eq!(dec.feed(ubx::SYNC1), FeedResult::Pending);
        assert_eq!(dec.feed(ubx::SYNC2), FeedResult::Pending);
        assert_eq!(dec.feed(0x01), FeedResult::Pending); // class
        assert_eq!(dec.feed(0x07), FeedResult::Pending); // id
        // Length = 0x0200 = 512 > 256
        assert_eq!(dec.feed(0x00), FeedResult::Pending); // len_lo
        assert_eq!(
            dec.feed(0x02),
            FeedResult::Error(DecodeError::FrameTooLarge)
        ); // len_hi
    }

    #[test]
    fn zero_length_ubx_payload() {
        let mut dec = Decoder::new();
        let frame = build_ubx(ubx::class::CFG, ubx::cfg::CFG, &[]);
        feed_until_frame(&mut dec, &frame).expect("should produce frame");
        match dec.take_frame() {
            Frame::Ubx(f) => {
                assert_eq!(f.class, ubx::class::CFG);
                assert_eq!(f.id, ubx::cfg::CFG);
                assert!(f.payload.is_empty());
            }
            _ => panic!("expected UBX"),
        }
    }

    #[test]
    fn reset_discards_partial() {
        let mut dec = Decoder::new();
        // Start an NMEA sentence
        dec.feed(b'$');
        dec.feed(b'G');
        dec.feed(b'P');

        // Reset mid-frame
        dec.reset();

        // Now feed a complete UBX frame
        let frame = build_ubx(ubx::class::ACK, ubx::ack::ACK, &[0x05, 0x01]);
        feed_until_frame(&mut dec, &frame).expect("should produce frame");
        match dec.take_frame() {
            Frame::Ubx(f) => {
                assert_eq!(f.class, ubx::class::ACK);
            }
            _ => panic!("expected UBX"),
        }
    }

    #[test]
    fn nmea_without_cr() {
        let mut dec = Decoder::new();
        // Some receivers send just \n without \r
        let sentence = b"$GPGGA,test*00\n";
        feed_until_frame(&mut dec, sentence).expect("should produce frame");
        match dec.take_frame() {
            Frame::Nmea(f) => {
                assert_eq!(f.raw, b"$GPGGA,test*00");
            }
            _ => panic!("expected NMEA"),
        }
    }
}
