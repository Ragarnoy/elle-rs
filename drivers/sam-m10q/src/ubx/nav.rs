//! UBX NAV class: message IDs and NAV-PVT access.

pub use ublox::nav_pvt::proto33::NavPvtRef;

/// UBX NAV message IDs.
pub const POSLLH: u8 = 0x02;
pub const STATUS: u8 = 0x03;
pub const DOP: u8 = 0x04;
pub const SOL: u8 = 0x06;
pub const PVT: u8 = 0x07;
pub const VELNED: u8 = 0x12;
pub const TIMEUTC: u8 = 0x21;
pub const SAT: u8 = 0x35;
pub const SIG: u8 = 0x43;

/// Fix quality from a NAV-PVT, in the GGA convention: 0 none, 1 GNSS, 2 DGNSS.
///
/// `fixType` alone is **not** the answer. UBX specifies that `fixType` is only
/// meaningful when `flags` bit 0 (`gnssFixOK`) is set, and an M10 acquiring
/// under a partial sky routinely reports `fixType = 3` with `gnssFixOK = 0` —
/// a solution that has not passed the receiver's own DOP and accuracy masks.
/// Reading `fixType` on its own turns that into a confident 3-D fix, which is
/// how a bad position reaches the CRSF GPS frame and becomes a home point.
///
/// Kept next to the parsing so the rule lives with the field it guards.
#[must_use]
pub fn fix_quality(pvt: &NavPvtRef<'_>) -> u8 {
    use ublox::nav_pvt::common::NavPvtFlags;

    if !pvt.flags().contains(NavPvtFlags::GPS_FIX_OK) {
        return 0;
    }
    // fixType: 0 none, 1 dead reckoning, 2 2-D, 3 3-D, 4 GNSS+DR, 5 time only.
    match pvt.fix_type() as u8 {
        2 | 3 => 1,
        4 => 2,
        _ => 0,
    }
}

/// Interpret a decoded UBX frame as NAV-PVT, if that is what it is.
///
/// Returns `None` for any other message and for a payload too short to be a
/// valid NAV-PVT. Parsing is delegated to the [`ublox`] crate so the field
/// offsets and scaling factors are not transcribed by hand.
///
/// Protocol note: the crate tops out at `ubx_proto33` while the SAM-M10Q
/// (SPG 5.10) speaks 34.10. NAV-PVT is unchanged between those versions, so
/// proto33 is the correct choice here rather than an oversight.
#[must_use]
pub fn parse_pvt(class_id: u8, msg_id: u8, payload: &[u8]) -> Option<NavPvtRef<'_>> {
    use ublox::{UbxProtocol, proto33};

    if class_id != super::class::NAV || msg_id != PVT {
        return None;
    }
    match proto33::Proto33::match_packet(class_id, msg_id, payload) {
        Ok(proto33::PacketRef::NavPvt(pvt)) => Some(pvt),
        _ => None,
    }
}

#[cfg(test)]
extern crate alloc;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::{Decoder, FeedResult};
    use crate::types::Frame;
    use crate::ubx;
    use alloc::vec;

    /// Build a NAV-PVT payload with known values at the documented offsets.
    fn sample_pvt_payload() -> [u8; 92] {
        let mut p = [0u8; 92];
        p[0..4].copy_from_slice(&123_456_u32.to_le_bytes()); // iTOW
        p[4..6].copy_from_slice(&2026u16.to_le_bytes());
        p[6] = 9; // month
        p[7] = 9; // day
        p[8] = 12; // hour
        p[9] = 34; // min
        p[10] = 56; // sec
        p[20] = 3; // fixType = 3D
        p[21] = 0x01; // flags: gnssFixOK
        p[23] = 8; // numSV
        p[24..28].copy_from_slice(&115_167_000_i32.to_le_bytes()); // lon 11.5167 deg
        p[28..32].copy_from_slice(&481_173_000_i32.to_le_bytes()); // lat 48.1173 deg
        p[36..40].copy_from_slice(&545_400_i32.to_le_bytes()); // hMSL 545.4 m
        p[40..44].copy_from_slice(&1_500_u32.to_le_bytes()); // hAcc 1.5 m
        p[44..48].copy_from_slice(&2_500_u32.to_le_bytes()); // vAcc 2.5 m
        p[48..52].copy_from_slice(&3_000_i32.to_le_bytes()); // velN 3.0 m/s
        p[52..56].copy_from_slice(&(-4_000_i32).to_le_bytes()); // velE -4.0 m/s
        p[56..60].copy_from_slice(&500_i32.to_le_bytes()); // velD 0.5 m/s
        p[60..64].copy_from_slice(&5_000_i32.to_le_bytes()); // gSpeed 5.0 m/s
        p[64..68].copy_from_slice(&9_000_000_i32.to_le_bytes()); // headMot 90 deg
        p[68..72].copy_from_slice(&250_u32.to_le_bytes()); // sAcc 0.25 m/s
        p
    }

    #[test]
    fn parses_nav_pvt_fields() {
        let payload = sample_pvt_payload();
        let pvt = parse_pvt(ubx::class::NAV, PVT, &payload).expect("should parse");

        assert!((pvt.latitude() - 48.1173).abs() < 1e-6);
        assert!((pvt.longitude() - 11.5167).abs() < 1e-6);
        assert!((pvt.height_msl() - 545.4).abs() < 1e-6);
        assert!((pvt.horizontal_accuracy() - 1.5).abs() < 1e-6);
        assert!((pvt.vertical_accuracy() - 2.5).abs() < 1e-6);
        assert!((pvt.vel_north() - 3.0).abs() < 1e-6);
        assert!((pvt.vel_east() + 4.0).abs() < 1e-6);
        assert!((pvt.vel_down() - 0.5).abs() < 1e-6);
        assert!((pvt.ground_speed_2d() - 5.0).abs() < 1e-6);
        assert!((pvt.heading_motion() - 90.0).abs() < 1e-6);
        assert!((pvt.speed_accuracy() - 0.25).abs() < 1e-6);
        assert_eq!(pvt.num_satellites(), 8);
    }

    #[test]
    fn fix_quality_requires_gnss_fix_ok() {
        let mut payload = sample_pvt_payload();

        // fixType 3 with gnssFixOK set: a real 3-D fix.
        let pvt = parse_pvt(ubx::class::NAV, PVT, &payload).unwrap();
        assert_eq!(fix_quality(&pvt), 1);

        // Same fixType, gnssFixOK clear — acquiring, outside the DOP/accuracy
        // masks. Must not read as a fix.
        payload[21] = 0x00;
        let pvt = parse_pvt(ubx::class::NAV, PVT, &payload).unwrap();
        assert_eq!(fix_quality(&pvt), 0);

        // DGNSS is reported separately, and still needs the flag.
        payload[20] = 4;
        payload[21] = 0x01;
        let pvt = parse_pvt(ubx::class::NAV, PVT, &payload).unwrap();
        assert_eq!(fix_quality(&pvt), 2);

        // Dead reckoning and time-only are not positions we will fly on.
        for ft in [0u8, 1, 5] {
            payload[20] = ft;
            let pvt = parse_pvt(ubx::class::NAV, PVT, &payload).unwrap();
            assert_eq!(fix_quality(&pvt), 0, "fixType {ft} should not be a fix");
        }
    }

    #[test]
    fn rejects_other_messages_and_short_payloads() {
        let payload = sample_pvt_payload();
        // Right payload, wrong class/id.
        assert!(parse_pvt(ubx::class::ACK, PVT, &payload).is_none());
        assert!(parse_pvt(ubx::class::NAV, SAT, &payload).is_none());
        // Right class/id, truncated payload — must not panic.
        assert!(parse_pvt(ubx::class::NAV, PVT, &payload[..40]).is_none());
        assert!(parse_pvt(ubx::class::NAV, PVT, &[]).is_none());
    }

    /// The whole path a real byte stream takes: decoder frames it, `parse_pvt`
    /// interprets it.
    #[test]
    fn decodes_and_parses_from_a_byte_stream() {
        let payload = sample_pvt_payload();
        let mut frame = vec![0u8; ubx::HEADER_SIZE + payload.len() + ubx::CHECKSUM_SIZE];
        let len = ubx::build_frame(&mut frame, ubx::class::NAV, PVT, &payload).unwrap();

        let mut dec = Decoder::new();
        let mut got = false;
        for &b in &frame[..len] {
            if dec.feed(b) == FeedResult::FrameReady {
                let Some(Frame::Ubx(u)) = dec.take_frame() else {
                    panic!("expected a UBX frame")
                };
                let pvt = parse_pvt(u.class, u.id, u.payload).expect("should parse");
                assert!((pvt.ground_speed_2d() - 5.0).abs() < 1e-6);
                assert_eq!(pvt.num_satellites(), 8);
                got = true;
            }
        }
        assert!(got, "stream should have produced a NAV-PVT frame");
    }
}
