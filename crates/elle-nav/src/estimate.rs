//! Navigation state from timestamped GNSS and baro samples.
//!
//! Every input carries the time it was measured and every output says whether
//! it is usable: a cached number is not a measurement. Position comes from the
//! newest acceptable fix, extrapolated along the measured ground velocity for a
//! short, bounded time; after [`EstimatorConfig::fix_timeout_us`] it is gone.
//! Altitude is baro height above home; GNSS height above home rides along for
//! comparison. There is no inertial fusion: GNSS loss means no position.

use crate::geo::{GeoPoint, LocalFrame, Ne};

/// Ground velocity from a GNSS solution (NAV-PVT only; the GGA fallback has none).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssVelocity {
    /// North/east ground velocity, m/s.
    pub ne: Ne,
    /// Speed accuracy estimate, m/s.
    pub s_acc_ms: f32,
}

/// One GNSS solution, stamped with the time it was received.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssSample {
    /// Receive time, µs since boot.
    pub t_us: u64,
    /// The receiver reports a valid fix (`gnssFixOK` on NAV-PVT).
    pub fix_ok: bool,
    pub num_satellites: u8,
    pub pos: GeoPoint,
    /// Height above mean sea level, m.
    pub alt_msl_m: f32,
    /// Horizontal accuracy estimate, m (`None` on the GGA fallback).
    pub h_acc_m: Option<f32>,
    /// Vertical accuracy estimate, m (`None` on the GGA fallback).
    pub v_acc_m: Option<f32>,
    pub vel: Option<GnssVelocity>,
}

/// One barometer reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroSample {
    /// Measurement time, µs since boot.
    pub t_us: u64,
    /// Pressure altitude (standard atmosphere), m.
    pub alt_m: f32,
    /// Filtered climb rate, m/s, positive up.
    pub climb_ms: f32,
}

/// Quality gates and time limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatorConfig {
    pub max_h_acc_m: f32,
    pub max_s_acc_ms: f32,
    pub max_v_acc_m: f32,
    pub home_max_h_acc_m: f32,
    pub home_min_sats: u8,
    pub fix_timeout_us: u64,
    pub extrapolate_max_us: u64,
    /// Fix age from which the state reports `coasting`.
    pub coast_after_us: u64,
    pub baro_timeout_us: u64,
}

impl EstimatorConfig {
    /// The firmware configuration (`NAV_*` in elle-config).
    #[must_use]
    pub const fn from_config() -> Self {
        use elle_config as c;
        Self {
            max_h_acc_m: c::NAV_MAX_H_ACC_M,
            max_s_acc_ms: c::NAV_MAX_S_ACC_MS,
            max_v_acc_m: c::NAV_MAX_V_ACC_M,
            home_max_h_acc_m: c::NAV_HOME_MAX_H_ACC_M,
            home_min_sats: c::NAV_HOME_MIN_SATS,
            fix_timeout_us: c::NAV_FIX_TIMEOUT_MS as u64 * 1000,
            extrapolate_max_us: c::NAV_EXTRAPOLATE_MAX_MS as u64 * 1000,
            coast_after_us: c::NAV_COAST_AFTER_MS as u64 * 1000,
            baro_timeout_us: c::NAV_BARO_TIMEOUT_MS as u64 * 1000,
        }
    }
}

/// Where the aircraft started: the origin of the local frame and the altitude
/// references.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Home {
    pub pos: GeoPoint,
    pub frame: LocalFrame,
    /// GNSS height above MSL at home, m (`None` if the fix had no usable vertical accuracy).
    pub gnss_alt_msl_m: Option<f32>,
    /// Baro altitude at home, m (`None` if no fresh reading when it was taken).
    pub baro_alt_m: Option<f32>,
    /// Taken while armed (locked), rather than still following fixes on the ground.
    pub locked: bool,
}

/// Position, velocity and altitude at one instant, each with its own validity.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NavState {
    /// Metres north/east of home. Meaningful only when `pos_valid`.
    pub pos: Ne,
    pub pos_valid: bool,
    /// No usable fix for longer than expected (`coast_after_us`): `pos` is the
    /// last fix carried along the ground velocity, until `fix_timeout_us` drops
    /// it. Clear on a live stream, where `pos` is also carried forward, but only
    /// across the gap since the last fix.
    pub coasting: bool,
    /// Ground velocity, m/s. Meaningful only when `vel_valid`.
    pub vel: Ne,
    pub vel_valid: bool,
    /// Age of the fix `pos` comes from, µs (`u64::MAX` if none).
    pub fix_age_us: u64,
    /// Baro height above home, m. Meaningful only when `alt_valid`.
    pub alt_rel_m: f32,
    pub alt_valid: bool,
    /// Baro climb rate, m/s, positive up. Meaningful only when `alt_valid`.
    pub climb_ms: f32,
    /// GNSS height above home, m, when the fix is fresh and accurate enough.
    pub gnss_alt_rel_m: Option<f32>,
}

/// Keeps the newest samples and home, and turns them into a [`NavState`].
#[derive(Clone, Copy, Debug)]
pub struct Estimator {
    cfg: EstimatorConfig,
    fix: Option<GnssSample>,
    /// `fix` in the home frame, converted once when the fix (or home) arrives:
    /// the conversion is `f64`, software floating point on the RP2350.
    fix_ne: Option<Ne>,
    baro: Option<BaroSample>,
    home: Option<Home>,
}

impl Estimator {
    #[must_use]
    pub const fn new(cfg: EstimatorConfig) -> Self {
        Self {
            cfg,
            fix: None,
            fix_ne: None,
            baro: None,
            home: None,
        }
    }

    #[must_use]
    pub const fn home(&self) -> Option<&Home> {
        self.home.as_ref()
    }

    /// Feed a GNSS solution. Unusable ones (no fix, poor accuracy, GGA without
    /// an accuracy estimate) are dropped, so the previous good fix ages out.
    ///
    /// While disarmed, home follows every fix good enough to be home; while
    /// armed it stays put ([`Self::lock_home`]): home is where the flight
    /// started, never a position taken in the air.
    pub fn on_gnss(&mut self, s: GnssSample, armed: bool) {
        if !self.position_usable(&s) {
            return;
        }
        self.fix = Some(s);
        if !armed && self.home_usable(&s) {
            let baro_alt_m = self
                .baro
                .filter(|b| s.t_us.abs_diff(b.t_us) <= self.cfg.baro_timeout_us)
                .map(|b| b.alt_m);
            self.home = Some(Home {
                pos: s.pos,
                frame: LocalFrame::new(s.pos),
                gnss_alt_msl_m: self.vertical_usable(&s).then_some(s.alt_msl_m),
                baro_alt_m,
                locked: false,
            });
        }
        self.fix_ne = self.home.as_ref().map(|h| h.frame.to_ne(s.pos));
    }

    pub fn on_baro(&mut self, b: BaroSample, armed: bool) {
        self.baro = Some(b);
        // Keep the home altitude reference on the ground with the aircraft.
        if !armed && let Some(h) = &mut self.home {
            h.baro_alt_m = Some(b.alt_m);
        }
    }

    /// Lock home now (called on the arming edge, so a flight without a new
    /// fix still has a fixed origin). Does nothing when there is no home.
    pub fn lock_home(&mut self) {
        if let Some(h) = &mut self.home {
            h.locked = true;
        }
    }

    /// Drop a locked home after landing: disarmed, it follows fixes again.
    pub fn unlock_home(&mut self) {
        if let Some(h) = &mut self.home {
            h.locked = false;
        }
    }

    fn position_usable(&self, s: &GnssSample) -> bool {
        s.fix_ok && s.h_acc_m.is_some_and(|a| a <= self.cfg.max_h_acc_m)
    }

    fn home_usable(&self, s: &GnssSample) -> bool {
        s.num_satellites >= self.cfg.home_min_sats
            && s.h_acc_m.is_some_and(|a| a <= self.cfg.home_max_h_acc_m)
    }

    fn vertical_usable(&self, s: &GnssSample) -> bool {
        s.v_acc_m.is_some_and(|a| a <= self.cfg.max_v_acc_m)
    }

    fn velocity(&self, s: &GnssSample) -> Option<Ne> {
        s.vel
            .filter(|v| v.s_acc_ms <= self.cfg.max_s_acc_ms)
            .map(|v| v.ne)
    }

    /// The state at `now_us`. Everything is invalid until there is a home.
    #[must_use]
    pub fn state(&self, now_us: u64) -> NavState {
        let mut st = NavState {
            fix_age_us: u64::MAX,
            ..NavState::default()
        };
        let Some(home) = &self.home else {
            return st;
        };

        if let Some(b) = &self.baro
            && now_us.saturating_sub(b.t_us) <= self.cfg.baro_timeout_us
            && let Some(ref_alt) = home.baro_alt_m
        {
            st.alt_rel_m = b.alt_m - ref_alt;
            st.climb_ms = b.climb_ms;
            st.alt_valid = true;
        }

        let (Some(fix), Some(fix_ne)) = (&self.fix, self.fix_ne) else {
            return st;
        };
        let age = now_us.saturating_sub(fix.t_us);
        st.fix_age_us = age;
        if age > self.cfg.fix_timeout_us {
            return st;
        }

        st.pos = fix_ne;
        st.pos_valid = true;
        if let Some(v) = self.velocity(fix) {
            st.vel = v;
            st.vel_valid = true;
            let dt = age.min(self.cfg.extrapolate_max_us);
            st.pos += v * (dt as f32 * 1e-6);
        }
        st.coasting = age > self.cfg.coast_after_us;
        if let Some(ref_alt) = home.gnss_alt_msl_m
            && self.vertical_usable(fix)
        {
            st.gnss_alt_rel_m = Some(fix.alt_msl_m - ref_alt);
        }
        st
    }
}
