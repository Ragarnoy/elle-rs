//! L1 lateral guidance: position and ground velocity → lateral acceleration → bank.
//!
//! After ArduPilot's `AP_L1_Control` (Park, Deyst and How, "A New Nonlinear
//! Guidance Logic for Trajectory Tracking", 2004). The aircraft steers towards
//! a point on the path a distance L1 ahead, where L1 grows with ground speed so
//! the convergence *time* (the period) stays constant. It works on the ground
//! track, not the heading, so a crosswind is absorbed as crab angle.
//!
//! Sign convention: lateral acceleration and bank are positive to the right
//! (right wing down, turning clockwise seen from above), the same as the
//! heading-hold roll setpoint.

use core::f32::consts::FRAC_PI_2;

use crate::geo::Ne;

/// Standard gravity, m/s².
pub const G: f32 = 9.806_65;

/// Guidance tuning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct L1Config {
    /// Convergence period, s.
    pub period_s: f32,
    /// Damping ratio.
    pub damping: f32,
    /// Bank limit, degrees.
    pub max_bank_deg: f32,
    /// No guidance below this ground speed, m/s.
    pub min_ground_speed_ms: f32,
}

impl L1Config {
    /// The firmware configuration (`NAV_*` in elle-config).
    #[must_use]
    pub const fn from_config() -> Self {
        use elle_config as c;
        Self {
            period_s: c::NAV_L1_PERIOD_S,
            damping: c::NAV_L1_DAMPING,
            max_bank_deg: c::NAV_MAX_BANK_DEG,
            min_ground_speed_ms: c::NAV_MIN_GROUND_SPEED_MS,
        }
    }

    /// L1 distance at a ground speed, m.
    #[must_use]
    pub fn l1_distance(&self, ground_speed_ms: f32) -> f32 {
        core::f32::consts::FRAC_1_PI * self.damping * self.period_s * ground_speed_ms
    }

    /// The L1 gain, 4ζ².
    fn k_l1(&self) -> f32 {
        4.0 * self.damping * self.damping
    }
}

/// A path to follow.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Path {
    /// The straight line from `from` through `to` (flown beyond `to`).
    Line { from: Ne, to: Ne },
    /// A circle; `clockwise` seen from above means right turns.
    Loiter {
        center: Ne,
        radius_m: f32,
        clockwise: bool,
    },
}

/// Which law produced the demand.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Tracking a line.
    Line,
    /// Heading for the start of a line from behind it.
    ToLineStart,
    /// Heading for a loiter circle from outside it.
    LoiterCapture,
    /// On the loiter circle.
    LoiterCircle,
}

/// One guidance output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Guidance {
    /// Lateral acceleration demand, m/s², positive right, before the bank limit.
    pub lat_accel_ms2: f32,
    /// Bank demand, degrees, positive right, within the bank limit.
    pub bank_deg: f32,
    /// The demand hit the bank limit.
    pub bank_limited: bool,
    /// Distance off the path, m: positive right of a line, positive outside a circle.
    pub track_error_m: f32,
    pub phase: Phase,
}

/// Lateral guidance towards `path` from `pos` at ground velocity `vel`.
///
/// `None` below the minimum ground speed, where the velocity direction is noise.
#[must_use]
pub fn guide(cfg: &L1Config, path: &Path, pos: Ne, vel: Ne) -> Option<Guidance> {
    let speed = vel.norm();
    if speed < cfg.min_ground_speed_ms {
        return None;
    }
    let l1 = cfg.l1_distance(speed);
    let (lat_accel_ms2, track_error_m, phase) = match *path {
        Path::Line { from, to } => line(cfg, from, to, pos, vel, speed, l1),
        Path::Loiter {
            center,
            radius_m,
            clockwise,
        } => loiter(cfg, center, radius_m, clockwise, pos, vel, speed, l1),
    };
    let max = cfg.max_bank_deg.to_radians();
    let bank = libm::atanf(lat_accel_ms2 / G);
    Some(Guidance {
        lat_accel_ms2,
        bank_deg: bank.clamp(-max, max).to_degrees(),
        bank_limited: bank.abs() > max,
        track_error_m,
        phase,
    })
}

/// Angle from `vel` to `dir`, radians, positive when `dir` is to the right.
fn angle_to(vel: Ne, dir: Ne) -> f32 {
    libm::atan2f(dir.dot(vel.right()), dir.dot(vel))
}

/// L1 acceleration for a look-ahead angle η: K v² / L1 · sin η, with η capped
/// at ±90° so a target behind gives a full-rate turn, not a reversal.
fn l1_accel(cfg: &L1Config, speed: f32, l1: f32, eta: f32) -> f32 {
    cfg.k_l1() * speed * speed / l1 * libm::sinf(eta.clamp(-FRAC_PI_2, FRAC_PI_2))
}

fn line(
    cfg: &L1Config,
    from: Ne,
    to: Ne,
    pos: Ne,
    vel: Ne,
    speed: f32,
    l1: f32,
) -> (f32, f32, Phase) {
    let rel = pos - from;
    // A degenerate segment has no direction: fly to its point.
    let Some(dir) = (to - from).unit() else {
        return (
            l1_accel(cfg, speed, l1, angle_to(vel, -rel)),
            rel.norm(),
            Phase::ToLineStart,
        );
    };
    let track_error = rel.dot(dir.right());
    let along = rel.dot(dir);

    // Well behind the start (more than L1 away and more than 45° off the
    // back of it): head for the start point itself first.
    let dist = rel.norm();
    if dist > l1 && along < -core::f32::consts::FRAC_1_SQRT_2 * dist {
        return (
            l1_accel(cfg, speed, l1, angle_to(vel, -rel)),
            track_error,
            Phase::ToLineStart,
        );
    }

    // The L1 point is on the line ahead; seen from the track direction it
    // lies at -asin(e / L1), capped at 45° so a far-off aircraft intercepts
    // at 45° instead of flying straight at the line.
    let intercept = -libm::asinf((track_error / l1).clamp(
        -core::f32::consts::FRAC_1_SQRT_2,
        core::f32::consts::FRAC_1_SQRT_2,
    ));
    let eta = intercept - angle_to(dir, vel);
    (l1_accel(cfg, speed, l1, eta), track_error, Phase::Line)
}

#[allow(clippy::too_many_arguments)]
fn loiter(
    cfg: &L1Config,
    center: Ne,
    radius_m: f32,
    clockwise: bool,
    pos: Ne,
    vel: Ne,
    speed: f32,
    l1: f32,
) -> (f32, f32, Phase) {
    let dir = if clockwise { 1.0 } else { -1.0 };
    let rel = pos - center;
    let dist = rel.norm();
    let radial_error = dist - radius_m;
    // At the centre any direction is outward; pick one so the maths is defined.
    let out = rel.unit().unwrap_or(Ne::new(1.0, 0.0));

    // Capture: head for the centre through L1.
    let capture = l1_accel(cfg, speed, l1, angle_to(vel, -out));

    // On the circle: centripetal acceleration for the tangential speed, plus
    // a PD on the radial error at the L1 natural frequency.
    let omega = 2.0 * core::f32::consts::PI / cfg.period_s;
    let kx = omega * omega;
    let kv = 2.0 * cfg.damping * omega;
    let tangent = out.right() * dir; // direction of travel around the circle
    let v_t = vel.dot(tangent);
    let v_out = vel.dot(out);
    let centripetal = v_t * v_t / (radius_m + radial_error).max(0.5 * radius_m);
    let circle = dir * (centripetal + kx * radial_error + kv * v_out);

    // Outside the circle, use the capture demand while it turns less hard in
    // the loiter direction: on the way in it points at the centre, and the
    // circle law takes over as the aircraft reaches the edge.
    if radial_error > 0.0 && dir * capture < dir * circle {
        (capture, radial_error, Phase::LoiterCapture)
    } else {
        (circle, radial_error, Phase::LoiterCircle)
    }
}
