//! Level calibration: the fixed rotation from the IMU's frame to the airframe's.
//!
//! The board is rarely mounted exactly level with the airframe. With the aircraft
//! held at its reference attitude, the averaged accelerometer reading is gravity in
//! the board's frame; the rotation taking it onto +Z is the mounting offset
//! ("mount"). Applying `mount` to every sensor vector before the AHRS makes the
//! filter run in the airframe's frame, so attitude and rates both come out
//! corrected, at every attitude — unlike subtracting angles from its output.
//!
//! Pure math, no hardware: the Core1 driver collects samples and applies the result.

use elle_config::LEVEL_CAL_MAX_TILT_DEG;
use nalgebra::{ComplexField, Quaternion, UnitQuaternion, Vector3};

/// `LEVEL_CAL_MAX_TILT_DEG` in radians.
const MAX_TILT_RAD: f32 = LEVEL_CAL_MAX_TILT_DEG.to_radians();

/// Why a level calibration was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum LevelCalFail {
    /// The gyro saw rotation during collection: the aircraft was being handled.
    Moving,
    /// The offset exceeds `LEVEL_CAL_MAX_TILT_DEG` (or gravity was unusable): the
    /// aircraft was not at its reference attitude.
    Tilted,
}

/// Compute the mount rotation from the mean raw accelerometer vector and the
/// largest gyro magnitude seen while collecting it.
///
/// # Errors
///
/// `Moving` if `max_gyro_rad_s` exceeds `max_gyro`; `Tilted` if the offset is
/// larger than `LEVEL_CAL_MAX_TILT_DEG`, or the vector is zero or points down.
pub fn compute_mount(
    mean_accel: Vector3<f32>,
    max_gyro_rad_s: f32,
    max_gyro: f32,
) -> Result<UnitQuaternion<f32>, LevelCalFail> {
    if max_gyro_rad_s.is_nan() || max_gyro_rad_s > max_gyro {
        return Err(LevelCalFail::Moving);
    }
    // Check the tilt on the vector itself, before building the rotation: for exactly
    // opposite vectors `rotation_between` returns the *identity*, and `angle()`
    // returns 0 for a zero vector, so an upside-down board or a dead accelerometer
    // would otherwise pass as perfectly level.
    let up = mean_accel.try_normalize(1e-6).ok_or(LevelCalFail::Tilted)?;
    // Compare cosines (acos is not in core): tilt <= max  <=>  cos(tilt) >= cos(max).
    if up.z.is_nan() || up.z < ComplexField::cos(MAX_TILT_RAD) {
        return Err(LevelCalFail::Tilted);
    }
    UnitQuaternion::rotation_between(&up, &Vector3::z()).ok_or(LevelCalFail::Tilted)
}

/// The mounting offset as (roll, pitch) in degrees, in the same sign convention as
/// the attitude telemetry: what the *uncorrected* attitude reads with the airframe
/// level. "pitch −2.1" means the board sits 2.1° nose-down.
#[must_use]
pub fn mount_to_display_deg(mount: &UnitQuaternion<f32>) -> (f32, f32) {
    let (roll, pitch, _yaw) = mount.euler_angles();
    // Same roll flip as the IMU driver applies to published attitude.
    (-roll.to_degrees(), pitch.to_degrees())
}

/// Serialize for flash: quaternion (w, i, j, k) as four little-endian f32.
#[must_use]
pub fn mount_to_bytes(mount: &UnitQuaternion<f32>) -> [u8; 16] {
    let q = mount.quaternion();
    bytemuck::cast([q.w, q.i, q.j, q.k])
}

/// Parse a stored mount. `None` for anything that must not be applied: non-finite
/// or non-unit values (corrupt flash), an offset beyond `LEVEL_CAL_MAX_TILT_DEG`,
/// or the identity (what "clear" stored before it removed the entry instead).
#[must_use]
pub fn mount_from_bytes(data: &[u8; 16]) -> Option<UnitQuaternion<f32>> {
    let [w, i, j, k]: [f32; 4] = bytemuck::pod_read_unaligned(data);
    if ![w, i, j, k].iter().all(|v| v.is_finite()) {
        return None;
    }
    let q = Quaternion::new(w, i, j, k);
    if (q.norm() - 1.0).abs() > 0.01 {
        return None;
    }
    let mount = UnitQuaternion::from_quaternion(q);
    let angle = mount.angle();
    if angle < 1e-6 || angle > MAX_TILT_RAD {
        return None;
    }
    Some(mount)
}
