//! Navigation in observation mode, shared by the flight and RPC loops.
//!
//! Feeds new GNSS and baro samples to `elle_nav::Navigator` every tick, and
//! every `NAV_UPDATE_DIVISOR` ticks (25 Hz) asks it for the state and for the
//! lateral demand on the reference path (a loiter around home). The result is
//! only logged (ULog `nav`): nothing here reaches the attitude controller.

use elle_config::NAV_UPDATE_DIVISOR;
use elle_hardware::gnss::{GNSS, GnssData};
use elle_hardware::imu::{AttitudeData, BARO, BaroReading};
use elle_nav::{BaroSample, GeoPoint, GnssSample, GnssVelocity, Navigator, Ne};
use elle_ulog::NavMessage;
use embassy_time::Instant;

/// What one tick produced for the log.
pub(crate) struct NavTick {
    /// A GNSS solution not seen before (logged as `gnss_data`, one per solution).
    pub gnss: Option<GnssData>,
    /// A navigator update, every `NAV_UPDATE_DIVISOR` ticks once GNSS has spoken.
    pub nav: Option<NavMessage>,
}

pub(crate) struct NavObserver {
    nav: Navigator,
    last_gnss_us: u64,
    last_baro_us: u64,
}

impl NavObserver {
    pub(crate) const fn new() -> Self {
        Self {
            nav: Navigator::from_config(),
            last_gnss_us: 0,
            last_baro_us: 0,
        }
    }

    /// Whether a home exists (for the radio's flight-mode text while disarmed).
    pub(crate) fn home_set(&self) -> bool {
        self.nav.home().is_some()
    }

    pub(crate) fn tick(
        &mut self,
        armed: bool,
        attitude: Option<&AttitudeData>,
        loop_counter: u32,
    ) -> NavTick {
        // Arming first, so a fix arriving on the arming tick cannot move home.
        self.nav.set_armed(armed);

        let baro = BARO.read_cached();
        if baro.sample_us != self.last_baro_us {
            self.last_baro_us = baro.sample_us;
            self.nav.on_baro(baro_sample(&baro));
        }

        let g = GNSS.read_cached();
        let gnss = (g.sample_us != self.last_gnss_us).then(|| {
            self.last_gnss_us = g.sample_us;
            self.nav.on_gnss(gnss_sample(&g));
            g
        });

        let nav = (self.last_gnss_us != 0 && loop_counter.is_multiple_of(NAV_UPDATE_DIVISOR))
            .then(|| self.message(attitude));
        NavTick { gnss, nav }
    }

    fn message(&self, attitude: Option<&AttitudeData>) -> NavMessage {
        use elle_nav::status;
        let out = self
            .nav
            .update(Instant::now().as_micros(), &Navigator::reference_path());
        let st = &out.state;
        let when = |valid: bool, v: f32| if valid { v } else { f32::NAN };
        let pos_valid = out.status & status::POS_VALID != 0;
        let vel_valid = out.status & status::VEL_VALID != 0;
        let alt_valid = out.status & status::ALT_VALID != 0;
        let g = out.guidance;
        NavMessage {
            timestamp: 0,
            status: out.status,
            fix_age_ms: (st.fix_age_us / 1000).min(u64::from(u16::MAX)) as u16,
            pos_n_m: when(pos_valid, st.pos.n),
            pos_e_m: when(pos_valid, st.pos.e),
            vel_n_ms: when(vel_valid, st.vel.n),
            vel_e_ms: when(vel_valid, st.vel.e),
            alt_rel_m: when(alt_valid, st.alt_rel_m),
            climb_ms: when(alt_valid, st.climb_ms),
            gnss_alt_rel_m: st.gnss_alt_rel_m.unwrap_or(f32::NAN),
            home_dist_m: when(pos_valid, out.home_dist_m),
            home_bearing_deg: when(pos_valid, out.home_bearing_deg),
            track_error_m: g.map_or(f32::NAN, |g| g.track_error_m),
            lat_accel_ms2: g.map_or(f32::NAN, |g| g.lat_accel_ms2),
            bank_demand_deg: g.map_or(f32::NAN, |g| g.bank_deg),
            roll_deg: attitude.map_or(f32::NAN, |a| a.roll.to_degrees()),
        }
    }
}

fn baro_sample(b: &BaroReading) -> BaroSample {
    BaroSample {
        t_us: b.sample_us,
        alt_m: b.altitude_m,
        climb_ms: b.vario_ms,
    }
}

/// A finite value, or `None` (the GNSS task reports what a source lacks as NaN).
fn finite(v: f32) -> Option<f32> {
    v.is_finite().then_some(v)
}

fn gnss_sample(g: &GnssData) -> GnssSample {
    // Accuracy and velocity only exist on NAV-PVT; the GGA fallback yields a
    // position without them, which the estimator does not use.
    let pvt = |v: f32| if g.pvt_active { finite(v) } else { None };
    let vel = match (pvt(g.vel_n_ms), pvt(g.vel_e_ms), pvt(g.s_acc_ms)) {
        (Some(n), Some(e), Some(s_acc_ms)) => Some(GnssVelocity {
            ne: Ne::new(n, e),
            s_acc_ms,
        }),
        _ => None,
    };
    GnssSample {
        t_us: g.sample_us,
        fix_ok: g.fix_quality > 0,
        num_satellites: g.num_satellites,
        pos: GeoPoint::new(g.lat_e7, g.lon_e7),
        alt_msl_m: g.altitude_m,
        h_acc_m: pvt(g.h_acc_m),
        v_acc_m: pvt(g.v_acc_m),
        vel,
    }
}
