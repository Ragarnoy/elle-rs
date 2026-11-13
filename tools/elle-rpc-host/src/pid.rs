//! Encoding PID gains for `SetPidGainsReq`.
//!
//! Every field is an `i16` fixed-point value. A plain `as i16` cast saturates
//! silently: the scale field (×10 000) holds at most 3.2767, so `pid … 5 0.5`
//! sent scale 3.2767, and `savepid` kept it, every term ~35 % weak across
//! reboots. This encoding rounds to the nearest step and refuses anything
//! that does not fit, so nothing is sent at all. The firmware's own
//! `PID_SCALE` (5.0) cannot go over RPC until the ICD widens the field: compile
//! such gains in.

use elle_rpc_icd::SetPidGainsReq;

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

/// One field: `value × factor`, rounded, or why it doesn't fit.
fn fixed(name: &str, value: f32, factor: f32) -> Result<i16, String> {
    let max = f32::from(i16::MAX) / factor;
    let min = f32::from(i16::MIN) / factor;
    let scaled = (value * factor).round();
    if !value.is_finite() || scaled > f32::from(i16::MAX) || scaled < f32::from(i16::MIN) {
        return Err(format!(
            "{name} {value} doesn't fit the RPC field ({min}..={max}); nothing sent"
        ));
    }
    Ok(scaled as i16)
}

/// The request for `gains`, or the first field that doesn't fit.
pub fn set_pid_gains_req(g: &PidGains) -> Result<SetPidGainsReq, String> {
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
}
