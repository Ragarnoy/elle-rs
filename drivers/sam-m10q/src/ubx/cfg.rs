//! UBX CFG class: message IDs and the CFG-VALSET builder.
//!
//! Configuration keys and their value types come from the [`ublox`] crate's
//! typed [`CfgVal`] enum rather than a hand-transcribed key table — a wrong key
//! ID is otherwise silently ignored by the module or answered with a NAK.

use super::{CHECKSUM_SIZE, HEADER_SIZE, build_frame, class};

pub use ublox::cfg_val::{CfgKey, CfgVal};

/// UBX CFG message IDs.
pub const PRT: u8 = 0x00;
pub const MSG: u8 = 0x01;
pub const RST: u8 = 0x04;
pub const RATE: u8 = 0x08;
pub const CFG: u8 = 0x09;
pub const VALSET: u8 = 0x8A;
pub const VALGET: u8 = 0x8B;
pub const VALDEL: u8 = 0x8C;

/// CFG-VALSET payload header: `version`, `layers`, then two reserved bytes.
pub const VALSET_HEADER_SIZE: usize = 4;

/// CFG-VALSET message version.
///
/// Sources disagree on whether this should be 0x00 or 0x01 — 0x01 is documented
/// as adding transaction support. We send 0x00 (no transaction) and rely on the
/// ACK/NAK check to catch it if a module disagrees.
pub const VALSET_VERSION: u8 = 0x00;

/// Apply to the RAM layer only — lost on reset, so every boot reconfigures from
/// the module's known power-on defaults instead of inheriting unknown state.
pub const LAYER_RAM: u8 = 0x01;
/// Apply to battery-backed RAM (survives a warm start).
pub const LAYER_BBR: u8 = 0x02;
/// Apply to flash. Avoid: it wears the part and hides misconfiguration.
pub const LAYER_FLASH: u8 = 0x04;

/// Largest CFG-VALSET payload this builder will assemble.
///
/// The protocol allows up to 64 key/value pairs; we send a handful, so a small
/// stack buffer is plenty and keeps `build_valset` allocation-free.
pub const MAX_VALSET_PAYLOAD: usize = 128;

/// Total buffer size needed to hold the largest frame `build_valset` can emit.
pub const MAX_VALSET_FRAME: usize = HEADER_SIZE + MAX_VALSET_PAYLOAD + CHECKSUM_SIZE;

/// Build a complete UBX-CFG-VALSET frame into `buf`.
///
/// `layers` is a bitmask of `LAYER_RAM` / `LAYER_BBR` / `LAYER_FLASH`; the module
/// answers with ACK-NAK if it is zero. Returns the frame length, or `None` if
/// the items do not fit in `MAX_VALSET_PAYLOAD` or `buf` is too small.
#[must_use]
pub fn build_valset(buf: &mut [u8], layers: u8, items: &[CfgVal]) -> Option<usize> {
    let mut payload = [0u8; MAX_VALSET_PAYLOAD];
    payload[0] = VALSET_VERSION;
    payload[1] = layers;
    // payload[2..4] are reserved and stay zero.

    let mut len = VALSET_HEADER_SIZE;
    for item in items {
        // `CfgVal::write_to` panics on a short buffer, so check before calling.
        if len + item.len() > MAX_VALSET_PAYLOAD {
            return None;
        }
        len += item.write_to(&mut payload[len..]);
    }

    build_frame(buf, class::CFG, VALSET, &payload[..len])
}

#[cfg(test)]
extern crate alloc;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ubx;

    #[test]
    fn valset_single_item_frame_layout() {
        let mut buf = [0u8; MAX_VALSET_FRAME];
        // CFG-RATE-MEAS is a u16 key: 4 key bytes + 2 value bytes.
        let len = build_valset(&mut buf, LAYER_RAM, &[CfgVal::RateMeas(200)]).unwrap();

        // 6 header + (4 valset header + 6 kv) + 2 checksum
        assert_eq!(len, ubx::HEADER_SIZE + VALSET_HEADER_SIZE + 6 + ubx::CHECKSUM_SIZE);
        assert_eq!(buf[0], ubx::SYNC1);
        assert_eq!(buf[1], ubx::SYNC2);
        assert_eq!(buf[2], ubx::class::CFG);
        assert_eq!(buf[3], VALSET);
        assert_eq!(u16::from_le_bytes([buf[4], buf[5]]) as usize, VALSET_HEADER_SIZE + 6);

        // VALSET payload header
        assert_eq!(buf[6], VALSET_VERSION);
        assert_eq!(buf[7], LAYER_RAM);
        assert_eq!(&buf[8..10], &[0, 0], "reserved bytes must be zero");

        // Key ID little-endian, then the value.
        assert_eq!(u32::from_le_bytes([buf[10], buf[11], buf[12], buf[13]]), 0x3021_0001);
        assert_eq!(u16::from_le_bytes([buf[14], buf[15]]), 200);

        // Checksum covers class..payload end.
        let (ck_a, ck_b) = ubx::checksum(&buf[2..len - 2]);
        assert_eq!(buf[len - 2], ck_a);
        assert_eq!(buf[len - 1], ck_b);
    }

    #[test]
    fn valset_packs_multiple_keys_in_order() {
        let mut buf = [0u8; MAX_VALSET_FRAME];
        let items = [
            CfgVal::RateMeas(200),
            CfgVal::RateNav(1),
            CfgVal::MsgOutUbxNavPvtUart1(1),
        ];
        let len = build_valset(&mut buf, LAYER_RAM, &items).unwrap();
        // 6 + 6 + 4 (two u16 keys) + 5 (one u8 key) + 2
        assert_eq!(len, ubx::HEADER_SIZE + VALSET_HEADER_SIZE + 6 + 6 + 5 + ubx::CHECKSUM_SIZE);

        let payload = &buf[ubx::HEADER_SIZE + VALSET_HEADER_SIZE..len - 2];
        assert_eq!(u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]), 0x3021_0001);
        assert_eq!(u32::from_le_bytes([payload[6], payload[7], payload[8], payload[9]]), 0x3021_0002);
        assert_eq!(u32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]), 0x2091_0007);
        assert_eq!(payload[16], 1);
    }

    #[test]
    fn valset_rejects_undersized_buffer() {
        let mut buf = [0u8; 8];
        assert!(build_valset(&mut buf, LAYER_RAM, &[CfgVal::RateMeas(200)]).is_none());
    }

    #[test]
    fn valset_rejects_too_many_items() {
        let mut buf = [0u8; MAX_VALSET_FRAME];
        // u32 keys are 8 bytes each; 16 of them overflow the 128-byte payload.
        let items = [CfgVal::Uart1Baudrate(115_200); 16];
        assert!(build_valset(&mut buf, LAYER_RAM, &items).is_none());
    }
}
