//! Attitude estimation from one IMU sample at a time, shared by the firmware
//! (Core 1 IMU task) and host replay, so a replay runs exactly the code that flew.
//!
//! Per sample: subtract the gyro bias, rotate gyro and accel from the sensor
//! frame into the airframe frame (level-cal mount), step the Madgwick AHRS on
//! the unfiltered gyro, and low-pass the rates handed to the PID. The level
//! calibration itself (collection, signals) stays in the driver, which runs it
//! between [`AttitudePipeline::debias`] and [`AttitudePipeline::fuse`].

use core::time::Duration;

use nalgebra::{UnitQuaternion, Vector3};
use uf_ahrs::{Ahrs, Madgwick, MadgwickParams};

use crate::filter::GyroFilter;

/// Attitude and rates after one sample, in the airframe frame.
///
/// Board flat: pitch ≈ 0, roll ≈ 0; nose up: pitch > 0; right wing down: roll > 0.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Attitude {
    /// Radians.
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    /// Rad/s, low-passed (`GYRO_RATE_LPF_HZ`).
    pub pitch_rate: f32,
    pub roll_rate: f32,
    pub yaw_rate: f32,
}

pub struct AttitudePipeline {
    ahrs: Madgwick,
    rate_filter: GyroFilter,
    /// Level calibration: rotation from the IMU frame to the airframe frame,
    /// applied to every sensor vector before the AHRS. Identity = uncorrected.
    pub mount: UnitQuaternion<f32>,
    /// Gyro zero-rate offset (sensor frame), subtracted before everything else.
    /// Zero until the boot-time estimate completes.
    pub gyro_bias: Vector3<f32>,
}

impl Default for AttitudePipeline {
    fn default() -> Self {
        Self::new()
    }
}

impl AttitudePipeline {
    /// The firmware configuration: `AHRS_BETA` at `AHRS_SAMPLE_PERIOD_US`, rates
    /// filtered at `GYRO_RATE_LPF_HZ`, no mount, no bias.
    #[must_use]
    pub fn new() -> Self {
        Self {
            ahrs: Madgwick::new(
                Duration::from_micros(elle_config::AHRS_SAMPLE_PERIOD_US),
                MadgwickParams {
                    beta: elle_config::AHRS_BETA,
                },
            ),
            rate_filter: GyroFilter::new(
                elle_config::GYRO_RATE_LPF_HZ,
                elle_config::IMU_UPDATE_FREQUENCY_HZ as f32,
            ),
            mount: UnitQuaternion::identity(),
            gyro_bias: Vector3::zeros(),
        }
    }

    /// The AHRS state (its whole state: seeding it reproduces the filter exactly).
    #[must_use]
    pub fn quat(&self) -> UnitQuaternion<f32> {
        self.ahrs.orientation()
    }

    /// Replace the AHRS state, e.g. to start a replay where a log starts.
    pub fn set_quat(&mut self, q: UnitQuaternion<f32>) {
        self.ahrs.set_orientation(q);
    }

    /// Gyro with the bias removed, still in the sensor frame.
    #[must_use]
    pub fn debias(&self, gyro: Vector3<f32>) -> Vector3<f32> {
        gyro - self.gyro_bias
    }

    /// A mag vector (offset-corrected, sensor frame) in the airframe frame.
    #[must_use]
    pub fn mag_to_airframe(&self, mag: Vector3<f32>) -> Vector3<f32> {
        self.mount * mag
    }

    /// Step the filters with one sample: `gyro` from [`Self::debias`] and the
    /// raw accel, both in the sensor frame; `mag` already in the airframe frame
    /// (9-DOF when given, else 6-DOF). A zero accel or mag vector falls back to
    /// fewer sensors (gyro only at worst), so every sample is integrated.
    pub fn fuse(
        &mut self,
        gyro: Vector3<f32>,
        accel: Vector3<f32>,
        mag: Option<&Vector3<f32>>,
    ) -> Attitude {
        // Into the airframe frame; everything downstream (AHRS, rates)
        // then sees a level-mounted IMU.
        let gyro = self.mount * gyro;
        let accel = self.mount * accel;
        let rates = self.rate_filter.apply(&gyro);

        let q = match mag {
            Some(mag) => self.ahrs.update(gyro, accel, *mag),
            None => self.ahrs.update_imu(gyro, accel),
        };
        let (roll, pitch, yaw) = q.euler_angles();

        // Roll sign is inverted relative to the ICM-42686's raw frame on this PCB
        // orientation — field-confirmed rolling the wrong way with identity mapping.
        Attitude {
            pitch,
            roll: -roll,
            yaw,
            pitch_rate: rates.y,
            roll_rate: -rates.x,
            yaw_rate: rates.z,
        }
    }
}

/// Standard gravity, m/s².
const G: f32 = 9.806_65;

/// The attitude filter's forward axis, in its body frame (after the mount).
///
/// Derived from the firmware's hardware-verified conventions: pitch positive
/// nose up and roll positive right wing down (after `fuse` negates it), with
/// +1 g on z when level, make the filter's body axes forward −x, right +y,
/// up +z. `elle-replay`'s simulator checks the derivation against those
/// conventions; a wrong sign would show in a replay as more error, not less.
#[must_use]
pub fn forward() -> Vector3<f32> {
    Vector3::new(-1.0, 0.0, 0.0)
}

/// A GNSS ground speed usable for turn compensation: fresh, from NAV-PVT,
/// and fast enough that the aircraft is flying.
#[must_use]
pub fn usable_ground_speed(speed_ms: f32, age_ms: u32, pvt: bool) -> Option<f32> {
    (pvt && speed_ms.is_finite()
        && age_ms <= elle_config::AHRS_TURN_COMP_MAX_AGE_MS
        && speed_ms >= elle_config::AHRS_TURN_COMP_MIN_SPEED_MS)
        .then_some(speed_ms)
}

/// Turn compensation: in a coordinated turn the accelerometer reads gravity
/// plus the centripetal acceleration ω × v, with v the velocity along the
/// nose. Removing it leaves gravity, so the AHRS stops levelling the turn.
///
/// Without an airspeed sensor, v is the GNSS ground speed: in wind that is off
/// by the wind speed, and the correction by ω × wind (≈ 1 m/s² at 0.2 rad/s in
/// 5 m/s of wind). It fades in and out over `AHRS_TURN_COMP_RAMP_S`.
#[derive(Clone, Copy, Debug)]
pub struct TurnComp {
    weight: f32,
    speed: f32,
    step: f32,
}

impl Default for TurnComp {
    fn default() -> Self {
        Self::new()
    }
}

impl TurnComp {
    #[must_use]
    pub fn new() -> Self {
        Self {
            weight: 0.0,
            speed: 0.0,
            step: elle_config::AHRS_SAMPLE_PERIOD_US as f32
                / 1_000_000.0
                / elle_config::AHRS_TURN_COMP_RAMP_S,
        }
    }

    /// Once per sample: the usable ground speed ([`usable_ground_speed`]), or
    /// `None`, which fades the correction out on the last speed.
    pub fn update(&mut self, speed: Option<f32>) {
        let target = match speed {
            Some(v) => {
                self.speed = v;
                1.0
            }
            None => 0.0,
        };
        self.weight += (target - self.weight).clamp(-self.step, self.step);
    }

    /// How much of the correction applies, 0..=1.
    #[must_use]
    pub const fn weight(&self) -> f32 {
        self.weight
    }

    /// The accel with the centripetal term removed (both in the filter frame,
    /// gyro debiased). Unchanged while the weight is zero.
    #[must_use]
    pub fn correct(&self, gyro: &Vector3<f32>, accel: &Vector3<f32>) -> Vector3<f32> {
        if self.weight == 0.0 {
            return *accel;
        }
        accel - gyro.cross(&(forward() * self.speed)) * self.weight
    }
}

/// Accel gate: says when the accel is too far from 1 g to trust for tilt.
#[derive(Clone, Copy, Debug)]
pub struct AccelGate {
    threshold_ms2: Option<f32>,
    alpha: f32,
    norm_lp: Option<f32>,
}

impl AccelGate {
    /// `threshold_g`: skip while the low-passed magnitude is further than this
    /// from 1 g; `None` never skips.
    #[must_use]
    pub fn new(threshold_g: Option<f32>) -> Self {
        let dt = elle_config::AHRS_SAMPLE_PERIOD_US as f32 / 1_000_000.0;
        let rc = 1.0 / (2.0 * core::f32::consts::PI * elle_config::AHRS_ACCEL_GATE_LPF_HZ);
        Self {
            threshold_ms2: threshold_g.map(|g| g * G),
            alpha: dt / (rc + dt),
            norm_lp: None,
        }
    }

    /// Once per sample: `true` when the accel should be skipped.
    pub fn skip(&mut self, accel: &Vector3<f32>) -> bool {
        let n = accel.norm();
        let lp = self.norm_lp.map_or(n, |p| p + self.alpha * (n - p));
        self.norm_lp = Some(lp);
        self.threshold_ms2.is_some_and(|t| (lp - G).abs() > t)
    }
}
