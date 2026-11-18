//! Encoding PID gains for `SetPidGainsReq`.
//!
//! Every field is an `i16` fixed-point value. A plain `as i16` cast saturates
//! silently: the scale field (×10 000) holds at most 3.2767, so `pid … 5 0.5`
//! sent scale 3.2767, and `savepid` kept it, every term ~35 % weak across
//! reboots. This encoding refuses, with nothing sent, gains the flash loader
//! would throw away (`SavedGains::is_valid` in `elle-control/src/autotune.rs`),
//! a value that doesn't fit its field, and a value its field's step would change
//! by more than 1 % (a non-zero gain rounding to 0 included). The firmware's own
//! `PID_SCALE` (5.0) cannot go over RPC until the ICD widens the field: compile
//! such gains in.

use std::time::Duration;

use elle_rpc_icd::{
    AckResp, PID_ERR_INVALID, SetPidGainsEndpoint, SetPidGainsF32Endpoint, SetPidGainsF32Req,
    SetPidGainsReq,
};
use postcard_rpc::host_client::{HostClient, HostErr};
use postcard_rpc::standard_icd::WireError;
use tokio::time::timeout;

/// Gains as typed by the operator, in firmware units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidGains {
    pub pitch_kp: f32,
    pub pitch_ki: f32,
    pub pitch_kd: f32,
    pub roll_kp: f32,
    pub roll_ki: f32,
    pub roll_kd: f32,
    pub scale: f32,
    pub i_limit: f32,
}

/// The flash loader's bounds (`SavedGains::is_valid`, `elle-control`
/// `autotune.rs`): kp/ki/kd/scale 0..=100, i_limit 0..=1000, all finite.
/// Mirrored here (the host doesn't build `elle-control`); keep the two in step.
const GAIN_MAX: f32 = 100.0;
const I_LIMIT_MAX: f32 = 1000.0;

/// Largest relative change rounding to a field's step may make.
const MAX_ROUNDING: f32 = 0.01;

fn in_bounds(name: &str, value: f32, max: f32) -> Result<(), String> {
    if !value.is_finite() || !(0.0..=max).contains(&value) {
        return Err(format!(
            "{name} {value} is outside 0..={max} (the flash loader would refuse it); nothing sent"
        ));
    }
    Ok(())
}

/// One field: `value × factor`, rounded, or why it can't be sent as typed.
fn fixed(name: &str, value: f32, factor: f32) -> Result<i16, String> {
    let scaled = (value * factor).round();
    if !value.is_finite() || scaled > f32::from(i16::MAX) || scaled < f32::from(i16::MIN) {
        let (min, max) = (f32::from(i16::MIN) / factor, f32::from(i16::MAX) / factor);
        return Err(format!(
            "{name} {value} doesn't fit the RPC field ({min}..={max}); nothing sent"
        ));
    }
    let sent = scaled / factor;
    if value != 0.0 && sent == 0.0 {
        return Err(format!(
            "{name} {value} would go out as 0 (the field's step is {}); nothing sent",
            1.0 / factor
        ));
    }
    if value != 0.0 && ((sent - value) / value).abs() > MAX_ROUNDING {
        return Err(format!(
            "{name} {value} would go out as {sent} (the field's step is {}); nothing sent",
            1.0 / factor
        ));
    }
    Ok(scaled as i16)
}

/// The flash loader's range for every field, or the first one outside it.
/// Both endpoints are checked against it before anything is sent; the firmware
/// applies the same check (`PID_ERR_INVALID`, event 104).
pub fn check_bounds(g: &PidGains) -> Result<(), String> {
    for (name, v) in [
        ("pitch kp", g.pitch_kp),
        ("pitch ki", g.pitch_ki),
        ("pitch kd", g.pitch_kd),
        ("roll kp", g.roll_kp),
        ("roll ki", g.roll_ki),
        ("roll kd", g.roll_kd),
        ("scale", g.scale),
    ] {
        in_bounds(name, v, GAIN_MAX)?;
    }
    in_bounds("i_limit", g.i_limit, I_LIMIT_MAX)
}

/// The request for `gains`, or the first field that can't be sent as typed.
pub fn set_pid_gains_req(g: &PidGains) -> Result<SetPidGainsReq, String> {
    check_bounds(g)?;
    Ok(SetPidGainsReq {
        pitch_kp_x1000: fixed("pitch kp", g.pitch_kp, 1000.0)?,
        pitch_ki_x1000: fixed("pitch ki", g.pitch_ki, 1000.0)?,
        pitch_kd_x1000: fixed("pitch kd", g.pitch_kd, 1000.0)?,
        roll_kp_x1000: fixed("roll kp", g.roll_kp, 1000.0)?,
        roll_ki_x1000: fixed("roll ki", g.roll_ki, 1000.0)?,
        roll_kd_x1000: fixed("roll kd", g.roll_kd, 1000.0)?,
        scale_x10000: fixed("scale", g.scale, 10_000.0)?,
        i_limit_x10: fixed("i_limit", g.i_limit, 10.0)?,
    })
}

/// The request for the f32 endpoint: every value as typed.
#[must_use]
pub const fn set_pid_gains_f32_req(g: &PidGains) -> SetPidGainsF32Req {
    SetPidGainsF32Req {
        pitch_kp: g.pitch_kp,
        pitch_ki: g.pitch_ki,
        pitch_kd: g.pitch_kd,
        roll_kp: g.roll_kp,
        roll_ki: g.roll_ki,
        roll_kd: g.roll_kd,
        scale: g.scale,
        i_limit: g.i_limit,
    }
}

/// Which endpoint carried the gains.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sent {
    /// `elle/ctrl/pid_f32`: the values as typed.
    F32,
    /// `elle/ctrl/pid`: firmware without the f32 endpoint, fixed-point (and
    /// refused when a value doesn't fit, e.g. scale 5.0).
    Fixed,
}

/// Send `gains` through the f32 endpoint. Falls back to the fixed-point one
/// only when the firmware answers that it doesn't know the f32 endpoint
/// (`WireError::UnknownKey`, older firmware), never on a timeout or any other
/// error: a slow link must not resend through a lossier path.
pub async fn send_gains(
    client: &HostClient<WireError>,
    gains: &PidGains,
    limit: Duration,
) -> Result<Sent, String> {
    check_bounds(gains)?;
    let req = set_pid_gains_f32_req(gains);
    match timeout(limit, client.send_resp::<SetPidGainsF32Endpoint>(&req)).await {
        Ok(Ok(ack)) => ack_result(ack).map(|()| Sent::F32),
        Ok(Err(HostErr::Wire(WireError::UnknownKey))) => {
            let req = set_pid_gains_req(gains)?;
            match timeout(limit, client.send_resp::<SetPidGainsEndpoint>(&req)).await {
                Ok(Ok(ack)) => ack_result(ack).map(|()| Sent::Fixed),
                Ok(Err(e)) => Err(format!("RPC error: {e:?}")),
                Err(_) => Err("timeout".into()),
            }
        }
        Ok(Err(e)) => Err(format!("RPC error: {e:?}")),
        Err(_) => Err("timeout".into()),
    }
}

fn ack_result(ack: AckResp) -> Result<(), String> {
    match (ack.success, ack.error_code) {
        (true, _) => Ok(()),
        (false, PID_ERR_INVALID) => Err(
            "the flight controller refused them: out of range (kp/ki/kd/scale 0..=100, \
             i_limit 0..=1000), event 104"
                .into(),
        ),
        (false, code) => Err(format!("error {code}")),
    }
}

/// A heading for `SetHeadingHoldReq`: wrapped to ±180° (an `i16` of
/// centidegrees holds ±327.67°, so 0..360 doesn't fit) and rounded to 0.01°.
/// `None` for a non-finite angle.
#[must_use]
pub fn heading_cdeg(deg: f32) -> Option<i16> {
    if !deg.is_finite() {
        return None;
    }
    let wrapped = (deg + 180.0).rem_euclid(360.0) - 180.0;
    Some((wrapped * 100.0).round() as i16)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gains(scale: f32) -> PidGains {
        PidGains {
            pitch_kp: 0.35,
            pitch_ki: 0.02,
            pitch_kd: 0.02,
            roll_kp: 0.25,
            roll_ki: 0.012,
            roll_kd: 0.07,
            scale,
            i_limit: 0.5,
        }
    }

    #[test]
    fn operating_scale_is_refused_not_saturated() {
        let err = set_pid_gains_req(&gains(5.0)).unwrap_err();
        assert!(err.starts_with("scale 5"), "{err}");
    }

    #[test]
    fn largest_scale_that_fits_is_sent() {
        assert_eq!(
            set_pid_gains_req(&gains(3.2767)).unwrap().scale_x10000,
            32767
        );
        assert!(set_pid_gains_req(&gains(3.2768)).is_err());
    }

    #[test]
    fn values_round_instead_of_truncating() {
        // 0.35_f32 × 1000 is 349.99997; a cast gave 349.
        let req = set_pid_gains_req(&gains(1.0)).unwrap();
        assert_eq!(req.pitch_kp_x1000, 350);
        assert_eq!(req.roll_ki_x1000, 12);
        assert_eq!(req.roll_kd_x1000, 70);
        assert_eq!(req.i_limit_x10, 5);
        assert_eq!(req.scale_x10000, 10_000);
    }

    #[test]
    fn every_field_is_checked() {
        let mut g = gains(1.0);
        g.roll_kd = 40.0;
        assert!(set_pid_gains_req(&g).unwrap_err().starts_with("roll kd"));
        let mut g = gains(1.0);
        g.pitch_kp = f32::NAN;
        assert!(set_pid_gains_req(&g).is_err());
        let mut g = gains(1.0);
        g.i_limit = -4000.0;
        assert!(set_pid_gains_req(&g).unwrap_err().starts_with("i_limit"));
    }

    /// What the flash loader would refuse is refused before encoding.
    #[test]
    fn out_of_loader_bounds_is_refused() {
        let err = set_pid_gains_req(&gains(-1.0)).unwrap_err();
        assert!(
            err.starts_with("scale -1") && err.contains("0..=100"),
            "{err}"
        );
        let mut g = gains(1.0);
        g.pitch_ki = -0.02;
        assert!(set_pid_gains_req(&g).unwrap_err().starts_with("pitch ki"));
        let mut g = gains(1.0);
        g.i_limit = -0.5;
        assert!(set_pid_gains_req(&g).unwrap_err().starts_with("i_limit"));
        let mut g = gains(1.0);
        g.i_limit = 1500.0;
        assert!(set_pid_gains_req(&g).unwrap_err().contains("0..=1000"));
    }

    /// A non-zero gain the step would zero, or move by > 1 %, is refused.
    #[test]
    fn lossy_rounding_is_refused() {
        let mut g = gains(1.0);
        g.pitch_ki = 0.0004;
        assert!(set_pid_gains_req(&g).unwrap_err().contains("out as 0"));
        let mut g = gains(1.0);
        g.i_limit = 0.04;
        assert!(set_pid_gains_req(&g).unwrap_err().contains("out as 0"));
        let mut g = gains(1.0);
        g.i_limit = 0.55;
        assert!(
            set_pid_gains_req(&g)
                .unwrap_err()
                .starts_with("i_limit 0.55")
        );
        let mut g = gains(1.0);
        g.pitch_ki = 0.0125;
        assert!(
            set_pid_gains_req(&g)
                .unwrap_err()
                .starts_with("pitch ki 0.0125")
        );
        // Zero stays allowed: it switches a term off on purpose.
        let mut g = gains(1.0);
        g.roll_ki = 0.0;
        assert_eq!(set_pid_gains_req(&g).unwrap().roll_ki_x1000, 0);
    }

    #[test]
    fn heading_wraps_and_rounds() {
        assert_eq!(heading_cdeg(400.0), Some(4000));
        assert_eq!(heading_cdeg(-350.0), Some(1000));
        assert_eq!(heading_cdeg(179.996), Some(18000));
        assert_eq!(heading_cdeg(12.345), Some(1235));
        assert_eq!(heading_cdeg(f32::NAN), None);
    }
}
