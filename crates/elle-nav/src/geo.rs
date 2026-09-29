//! Geographic points and a local north/east frame around an origin (home).
//!
//! Coordinates stay integer degrees × 10⁷ (the receiver's own resolution,
//! ~1.1 cm) until they are converted into the home frame, in `f64`, by sguaba
//! (exact WGS84 → ECEF → NED). Guidance then works in `f32` [`Ne`] metres, which
//! keep centimetres out to tens of kilometres from home.

use core::ops::{Add, AddAssign, Mul, Neg, Sub};

use sguaba::Coordinate;
use sguaba::math::RigidBodyTransform;
use sguaba::systems::{Ecef, Wgs84};
use uom::si::angle::degree;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

/// A latitude/longitude, degrees × 10⁷ (the UBX NAV-PVT encoding).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GeoPoint {
    pub lat_e7: i32,
    pub lon_e7: i32,
}

impl GeoPoint {
    #[must_use]
    pub const fn new(lat_e7: i32, lon_e7: i32) -> Self {
        Self { lat_e7, lon_e7 }
    }

    /// From degrees, rounded to the nearest 10⁻⁷ degree.
    #[must_use]
    pub fn from_deg(lat_deg: f64, lon_deg: f64) -> Self {
        Self {
            lat_e7: libm::round(lat_deg * 1e7) as i32,
            lon_e7: libm::round(lon_deg * 1e7) as i32,
        }
    }

    #[must_use]
    pub fn lat_deg(self) -> f64 {
        f64::from(self.lat_e7) * 1e-7
    }

    #[must_use]
    pub fn lon_deg(self) -> f64 {
        f64::from(self.lon_e7) * 1e-7
    }
}

/// A horizontal vector in a local north/east frame, metres (or m/s).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Ne {
    pub n: f32,
    pub e: f32,
}

impl Ne {
    pub const ZERO: Self = Self { n: 0.0, e: 0.0 };

    #[must_use]
    pub const fn new(n: f32, e: f32) -> Self {
        Self { n, e }
    }

    #[must_use]
    pub fn dot(self, o: Self) -> f32 {
        self.n * o.n + self.e * o.e
    }

    #[must_use]
    pub fn norm(self) -> f32 {
        libm::sqrtf(self.dot(self))
    }

    /// Unit vector, or `None` for a (near) zero vector.
    #[must_use]
    pub fn unit(self) -> Option<Self> {
        let len = self.norm();
        (len > 1e-6).then(|| self * (1.0 / len))
    }

    /// The vector turned 90° to the right (clockwise seen from above).
    #[must_use]
    pub const fn right(self) -> Self {
        Self {
            n: -self.e,
            e: self.n,
        }
    }

    /// Compass bearing of the vector, radians clockwise from north, in (-π, π].
    #[must_use]
    pub fn bearing_rad(self) -> f32 {
        libm::atan2f(self.e, self.n)
    }

    /// Unit vector along a compass bearing (radians clockwise from north).
    #[must_use]
    pub fn from_bearing(bearing_rad: f32) -> Self {
        Self {
            n: libm::cosf(bearing_rad),
            e: libm::sinf(bearing_rad),
        }
    }
}

impl Add for Ne {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.n + o.n, self.e + o.e)
    }
}

impl AddAssign for Ne {
    fn add_assign(&mut self, o: Self) {
        *self = *self + o;
    }
}

impl Sub for Ne {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.n - o.n, self.e - o.e)
    }
}

impl Mul<f32> for Ne {
    type Output = Self;
    fn mul(self, k: f32) -> Self {
        Self::new(self.n * k, self.e * k)
    }
}

impl Neg for Ne {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.n, -self.e)
    }
}

sguaba::system!(
    /// The local north/east/down frame with its origin at home.
    ///
    /// A sguaba coordinate system, so a position in it cannot be mixed up with
    /// one in another frame (the body FRD frame, later) without an explicit
    /// transform. Guidance works on [`Ne`] once positions are in this frame.
    pub struct HomeNed using NED
);

/// Local frame at a geographic origin: WGS84 → ECEF → NED at the origin,
/// through sguaba (`f64`, software floating point on the RP2350, so it runs
/// once per fix rather than every tick).
#[derive(Clone, Copy, Debug)]
pub struct LocalFrame {
    origin: GeoPoint,
    ecef_to_ned: RigidBodyTransform<Ecef, HomeNed>,
}

impl PartialEq for LocalFrame {
    fn eq(&self, o: &Self) -> bool {
        self.origin == o.origin
    }
}

/// A point on the ellipsoid (altitude 0) for a lat/lon. Both points of a
/// conversion sit on the ellipsoid, so north/east are the horizontal offsets
/// and the down component is only the Earth's curvature, which is dropped.
fn wgs84(p: GeoPoint) -> Wgs84 {
    // Any i32 longitude is a valid angle; latitude is clamped so a corrupt
    // value cannot fail the range check (a receiver never reports one).
    let lat = p.lat_deg().clamp(-90.0, 90.0);
    Wgs84::builder()
        .latitude(Angle::new::<degree>(lat))
        .expect("latitude clamped to [-90, 90]")
        .longitude(Angle::new::<degree>(p.lon_deg()))
        .altitude(Length::new::<meter>(0.0))
        .build()
}

impl LocalFrame {
    #[must_use]
    pub fn new(origin: GeoPoint) -> Self {
        // SAFETY (sguaba's frame-correctness contract, not memory safety):
        // `HomeNed` is by definition the NED frame at `origin`, and this is
        // the only place a transform into it is built.
        let ecef_to_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&wgs84(origin)) };
        Self {
            origin,
            ecef_to_ned,
        }
    }

    #[must_use]
    pub const fn origin(&self) -> GeoPoint {
        self.origin
    }

    /// `p` as a typed position in the home frame.
    #[must_use]
    pub fn to_ned(&self, p: GeoPoint) -> Coordinate<HomeNed> {
        self.ecef_to_ned
            .transform(Coordinate::<Ecef>::from_wgs84(&wgs84(p)))
    }

    /// Position of `p` relative to the origin, metres north/east.
    #[must_use]
    pub fn to_ne(&self, p: GeoPoint) -> Ne {
        let c = self.to_ned(p);
        Ne::new(
            c.ned_north().get::<meter>() as f32,
            c.ned_east().get::<meter>() as f32,
        )
    }
}
