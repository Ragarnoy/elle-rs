//! UBX CFG class: message IDs and the CFG-VALSET builder.
//!
//! Configuration keys and their value types come from the [`ublox`] crate's
//! typed [`CfgVal`] enum rather than a hand-transcribed key table — a wrong key
//! ID is otherwise silently ignored by the module or answered with a NAK.

use super::{CHECKSUM_SIZE, HEADER_SIZE, build_frame, class};

pub use ublox::cfg_nav5::{NavDynamicModel, NavFixMode};
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
/// 0x00 is the non-transactional form, confirmed against the u-blox M10
/// SPG 5.10 interface description (UBX-21035062 R03, §3.10.5.1: "Message
/// version (0x00 for this version)"). Version 1 exists only to support
/// transactions, which we do not use.
pub const VALSET_VERSION: u8 = 0x00;

/// Apply to the RAM layer only — lost on reset, so every boot reconfigures from
/// the module's known power-on defaults instead of inheriting unknown state.
///
/// Note that RAM is also the only layer whose *validity* the receiver checks:
/// per UBX-21035062 §3.10.5.1, a VALSET aimed at RAM is NAKed if "the requested
/// configuration is not valid", not merely if a key is unknown. Keys that
/// constrain one another must therefore be sent in the same message.
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

/// CFG-VALGET poll and response header: `version`, `layer`, then a `u16`
/// position.
pub const VALGET_HEADER_SIZE: usize = 4;

/// CFG-VALGET `layer` for the RAM layer, the configuration in use.
///
/// Unlike VALSET's bitmask this is an index (UBX-21035062 §3.10.4: 0 RAM,
/// 1 BBR, 2 flash, 7 default), so [`LAYER_RAM`] would ask for BBR.
pub const VALGET_LAYER_RAM: u8 = 0;

/// Bytes of key ID in a key/value pair.
const KEY_SIZE: usize = 4;

/// Build a UBX-CFG-VALGET poll for the keys of `items` into `buf`.
///
/// Only the keys are sent; the values in `items` are what
/// [`valget_matches`] later compares the answer against. Returns the frame
/// length, or `None` if it does not fit in `MAX_VALSET_PAYLOAD` or `buf`.
#[must_use]
pub fn build_valget(buf: &mut [u8], layer: u8, items: &[CfgVal]) -> Option<usize> {
    let mut payload = [0u8; MAX_VALSET_PAYLOAD];
    // version 0 (poll request), layer, position 0.
    payload[1] = layer;

    let mut len = VALGET_HEADER_SIZE;
    let mut kv = [0u8; 16];
    for item in items {
        if len + KEY_SIZE > MAX_VALSET_PAYLOAD || item.len() > kv.len() {
            return None;
        }
        item.write_to(&mut kv);
        payload[len..len + KEY_SIZE].copy_from_slice(&kv[..KEY_SIZE]);
        len += KEY_SIZE;
    }

    build_frame(buf, class::CFG, VALGET, &payload[..len])
}

/// Size of the value stored under `key`, from the size field in bits 28–30.
const fn value_size(key: u32) -> Option<usize> {
    match (key >> 28) & 0b111 {
        1 | 2 => Some(1),
        3 => Some(2),
        4 => Some(4),
        5 => Some(8),
        _ => None,
    }
}

/// Whether a CFG-VALGET response `payload` holds exactly the values in
/// `items`.
///
/// Compares the raw key/value bytes against what [`CfgVal::write_to`]
/// produces, so no per-type decoding is needed. A key missing from the
/// response, or a response that cannot be walked to its end, does not match.
#[must_use]
pub fn valget_matches(payload: &[u8], items: &[CfgVal]) -> bool {
    let mut kv = [0u8; 16];
    items.iter().all(|item| {
        if item.len() > kv.len() {
            return false;
        }
        let n = item.write_to(&mut kv);
        let want = &kv[..n];
        let mut pos = VALGET_HEADER_SIZE;
        while pos + KEY_SIZE <= payload.len() {
            let key = u32::from_le_bytes([
                payload[pos],
                payload[pos + 1],
                payload[pos + 2],
                payload[pos + 3],
            ]);
            let Some(size) = value_size(key) else {
                return false;
            };
            let end = pos + KEY_SIZE + size;
            if end > payload.len() {
                return false;
            }
            if payload[pos..pos + KEY_SIZE] == want[..KEY_SIZE] {
                return payload[pos..end] == *want;
            }
            pos = end;
        }
        false
    })
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
        assert_eq!(
            len,
            ubx::HEADER_SIZE + VALSET_HEADER_SIZE + 6 + ubx::CHECKSUM_SIZE
        );
        assert_eq!(buf[0], ubx::SYNC1);
        assert_eq!(buf[1], ubx::SYNC2);
        assert_eq!(buf[2], ubx::class::CFG);
        assert_eq!(buf[3], VALSET);
        assert_eq!(
            u16::from_le_bytes([buf[4], buf[5]]) as usize,
            VALSET_HEADER_SIZE + 6
        );

        // VALSET payload header
        assert_eq!(buf[6], VALSET_VERSION);
        assert_eq!(buf[7], LAYER_RAM);
        assert_eq!(&buf[8..10], &[0, 0], "reserved bytes must be zero");

        // Key ID little-endian, then the value.
        assert_eq!(
            u32::from_le_bytes([buf[10], buf[11], buf[12], buf[13]]),
            0x3021_0001
        );
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
        assert_eq!(
            len,
            ubx::HEADER_SIZE + VALSET_HEADER_SIZE + 6 + 6 + 5 + ubx::CHECKSUM_SIZE
        );

        let payload = &buf[ubx::HEADER_SIZE + VALSET_HEADER_SIZE..len - 2];
        assert_eq!(
            u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            0x3021_0001
        );
        assert_eq!(
            u32::from_le_bytes([payload[6], payload[7], payload[8], payload[9]]),
            0x3021_0002
        );
        assert_eq!(
            u32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]),
            0x2091_0007
        );
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

    const GROUP0: [CfgVal; 2] = [
        CfgVal::NavSpgDynModel(NavDynamicModel::AirborneWithLess4gAcceleration),
        CfgVal::NavSpgFixMode(NavFixMode::Only3D),
    ];

    /// A VALGET response payload: version 1, the layer, position 0, then the
    /// key/value pairs as the module would send them.
    fn response(pairs: &[&[u8]]) -> alloc::vec::Vec<u8> {
        let mut p = alloc::vec![0x01, VALGET_LAYER_RAM, 0, 0];
        for kv in pairs {
            p.extend_from_slice(kv);
        }
        p
    }

    #[test]
    fn valget_poll_carries_only_the_keys() {
        let mut buf = [0u8; MAX_VALSET_FRAME];
        let len = build_valget(&mut buf, VALGET_LAYER_RAM, &GROUP0).unwrap();
        assert_eq!(
            len,
            ubx::HEADER_SIZE + VALGET_HEADER_SIZE + 2 * 4 + ubx::CHECKSUM_SIZE
        );
        assert_eq!(buf[3], VALGET);
        let payload = &buf[ubx::HEADER_SIZE..len - ubx::CHECKSUM_SIZE];
        assert_eq!(&payload[..4], &[0, VALGET_LAYER_RAM, 0, 0]);
        assert_eq!(
            u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            0x2011_0021
        );
        assert_eq!(
            u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            0x2011_0011
        );
    }

    #[test]
    fn valget_matches_values_in_any_order() {
        // FIXMODE 2 = 3D only, DYNMODEL 8 = AIR4, answered in the other order.
        let p = response(&[&[0x11, 0x00, 0x11, 0x20, 2], &[0x21, 0x00, 0x11, 0x20, 8]]);
        assert!(valget_matches(&p, &GROUP0));
    }

    #[test]
    fn valget_rejects_a_different_value() {
        // DYNMODEL 0 = portable, the module's default.
        let p = response(&[&[0x21, 0x00, 0x11, 0x20, 0], &[0x11, 0x00, 0x11, 0x20, 2]]);
        assert!(!valget_matches(&p, &GROUP0));
    }

    #[test]
    fn valget_rejects_a_missing_key() {
        let p = response(&[&[0x21, 0x00, 0x11, 0x20, 8]]);
        assert!(!valget_matches(&p, &GROUP0));
    }

    #[test]
    fn valget_walks_multi_byte_values() {
        // RATE-MEAS (u16) before the key we look for.
        let p = response(&[
            &[0x01, 0x00, 0x21, 0x30, 0xC8, 0x00],
            &[0x02, 0x00, 0x21, 0x30, 1, 0],
        ]);
        assert!(valget_matches(&p, &[CfgVal::RateNav(1)]));
        assert!(!valget_matches(&p, &[CfgVal::RateMeas(1000)]));
    }

    #[test]
    fn valget_rejects_a_truncated_response() {
        let p = response(&[&[0x01, 0x00, 0x21, 0x30, 0xC8]]);
        assert!(!valget_matches(&p, &[CfgVal::RateMeas(200)]));
    }
}
