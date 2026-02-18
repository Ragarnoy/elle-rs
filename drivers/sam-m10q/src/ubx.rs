/// UBX protocol constants and helpers.

/// UBX sync byte 1.
pub const SYNC1: u8 = 0xB5;
/// UBX sync byte 2.
pub const SYNC2: u8 = 0x62;
/// UBX header size (sync1 + sync2 + class + id + length_lo + length_hi).
pub const HEADER_SIZE: usize = 6;
/// UBX checksum size (ck_a + ck_b).
pub const CHECKSUM_SIZE: usize = 2;

/// Well-known UBX message classes.
pub mod class {
    pub const NAV: u8 = 0x01;
    pub const RXM: u8 = 0x02;
    pub const INF: u8 = 0x04;
    pub const ACK: u8 = 0x05;
    pub const CFG: u8 = 0x06;
    pub const UPD: u8 = 0x09;
    pub const MON: u8 = 0x0A;
    pub const TIM: u8 = 0x0D;
    pub const SEC: u8 = 0x27;
    pub const NMEA: u8 = 0xF0;
    pub const PUBX: u8 = 0xF1;
}

/// Well-known ACK message IDs.
pub mod ack {
    pub const NAK: u8 = 0x00;
    pub const ACK: u8 = 0x01;
}

/// Well-known CFG message IDs.
pub mod cfg {
    pub const PRT: u8 = 0x00;
    pub const MSG: u8 = 0x01;
    pub const RST: u8 = 0x04;
    pub const RATE: u8 = 0x08;
    pub const CFG: u8 = 0x09;
    pub const VALSET: u8 = 0x8A;
    pub const VALGET: u8 = 0x8B;
    pub const VALDEL: u8 = 0x8C;
}

/// Well-known NAV message IDs.
pub mod nav {
    pub const POSLLH: u8 = 0x02;
    pub const STATUS: u8 = 0x03;
    pub const DOP: u8 = 0x04;
    pub const SOL: u8 = 0x06;
    pub const PVT: u8 = 0x07;
    pub const VELNED: u8 = 0x12;
    pub const TIMEUTC: u8 = 0x21;
    pub const SAT: u8 = 0x35;
    pub const SIG: u8 = 0x43;
}

/// Compute the UBX Fletcher-8 checksum over the given data.
///
/// The checksum is computed over class + id + length + payload (everything
/// between sync bytes and the checksum itself).
#[must_use]
pub fn checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &byte in data {
        ck_a = ck_a.wrapping_add(byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

/// Verify a UBX checksum against expected values.
#[must_use]
pub fn verify_checksum(data: &[u8], expected: (u8, u8)) -> bool {
    checksum(data) == expected
}

/// Build a complete UBX frame into `buf`.
///
/// Returns the total frame length, or `None` if `buf` is too small.
/// Layout: `[SYNC1, SYNC2, class, id, len_lo, len_hi, payload..., ck_a, ck_b]`
#[must_use]
pub fn build_frame(buf: &mut [u8], class: u8, id: u8, payload: &[u8]) -> Option<usize> {
    let total = HEADER_SIZE + payload.len() + CHECKSUM_SIZE;
    if buf.len() < total {
        return None;
    }

    let len = payload.len() as u16;
    buf[0] = SYNC1;
    buf[1] = SYNC2;
    buf[2] = class;
    buf[3] = id;
    buf[4] = (len & 0xFF) as u8;
    buf[5] = (len >> 8) as u8;
    buf[HEADER_SIZE..HEADER_SIZE + payload.len()].copy_from_slice(payload);

    // Checksum covers class + id + length + payload
    let (ck_a, ck_b) = checksum(&buf[2..HEADER_SIZE + payload.len()]);
    buf[HEADER_SIZE + payload.len()] = ck_a;
    buf[HEADER_SIZE + payload.len() + 1] = ck_b;

    Some(total)
}

/// Compute the NMEA XOR checksum over data between '$' and '*'.
///
/// `data` should be the bytes between (but not including) '$' and '*'.
#[must_use]
pub fn nmea_checksum(data: &[u8]) -> u8 {
    let mut ck: u8 = 0;
    for &byte in data {
        ck ^= byte;
    }
    ck
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksum_empty() {
        assert_eq!(checksum(&[]), (0, 0));
    }

    #[test]
    fn checksum_ack_ack() {
        // ACK-ACK for CFG-MSG: class=0x05, id=0x01, len=2, payload=[0x06, 0x01]
        let data = [0x05, 0x01, 0x02, 0x00, 0x06, 0x01];
        let (ck_a, ck_b) = checksum(&data);
        // Manually compute:
        // ck_a: 5, 6, 8, 8, 14, 15
        // ck_b: 5, 11, 19, 27, 41, 56
        assert_eq!(ck_a, 0x0F);
        assert_eq!(ck_b, 0x38);
    }

    #[test]
    fn verify_checksum_pass() {
        let data = [0x05, 0x01, 0x02, 0x00, 0x06, 0x01];
        assert!(verify_checksum(&data, (0x0F, 0x38)));
    }

    #[test]
    fn verify_checksum_fail() {
        let data = [0x05, 0x01, 0x02, 0x00, 0x06, 0x01];
        assert!(!verify_checksum(&data, (0x00, 0x00)));
    }

    #[test]
    fn build_frame_empty_payload() {
        let mut buf = [0u8; 16];
        let len = build_frame(&mut buf, class::ACK, ack::ACK, &[]).unwrap();
        assert_eq!(len, 8);
        assert_eq!(buf[0], SYNC1);
        assert_eq!(buf[1], SYNC2);
        assert_eq!(buf[2], class::ACK);
        assert_eq!(buf[3], ack::ACK);
        assert_eq!(buf[4], 0); // len_lo
        assert_eq!(buf[5], 0); // len_hi
        let (ck_a, ck_b) = checksum(&buf[2..6]);
        assert_eq!(buf[6], ck_a);
        assert_eq!(buf[7], ck_b);
    }

    #[test]
    fn build_frame_with_payload() {
        let payload = [0x06, 0x01];
        let mut buf = [0u8; 16];
        let len = build_frame(&mut buf, class::ACK, ack::ACK, &payload).unwrap();
        assert_eq!(len, 10);
        assert_eq!(&buf[6..8], &payload);
        let (ck_a, ck_b) = checksum(&buf[2..8]);
        assert_eq!(buf[8], ck_a);
        assert_eq!(buf[9], ck_b);
    }

    #[test]
    fn build_frame_buffer_too_small() {
        let mut buf = [0u8; 4];
        assert!(build_frame(&mut buf, 0, 0, &[]).is_none());
    }

    #[test]
    fn nmea_checksum_gpgga() {
        // "$GPGGA,..." — the checksum covers everything between '$' and '*'
        let sentence = b"GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,";
        let ck = nmea_checksum(sentence);
        assert_eq!(ck, 0x4F);
    }
}
