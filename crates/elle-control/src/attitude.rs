//! Attitude estimation from one IMU sample at a time, shared by the firmware
//! (Core 1 IMU task) and host replay, so a replay runs exactly the code that flew.
//!
//! Per sample: subtract the gyro bias, rotate gyro and accel from the sensor
//! frame into the airframe frame (level-cal mount), step the Madgwick AHRS on
//! the unfiltered gyro, and low-pass the rates handed to the PID. The level
//! calibration itself (collection, signals) stays in the driver, which runs it
//! between [`AttitudePipeline::debias`] and [`AttitudePipeline::fuse`].

use ahrs::Ahrs;
use nalgebra::{UnitQuaternion, Vector3};

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
    ahrs: ahrs::Madgwick<f32>,
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
            ahrs: ahrs::Madgwick::new(
                elle_config::AHRS_SAMPLE_PERIOD_US as f32 / 1_000_000.0,
                elle_config::AHRS_BETA,
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
        self.ahrs.quat
    }

    /// Replace the AHRS state, e.g. to start a replay where a log starts.
    pub fn set_quat(&mut self, q: UnitQuaternion<f32>) {
        self.ahrs.quat = q;
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
    /// (9-DOF when given, else 6-DOF). The rate filter sees every sample even
    /// when the AHRS rejects one (normalisation failure), which gives `None`.
    pub fn fuse(
        &mut self,
        gyro: Vector3<f32>,
        accel: Vector3<f32>,
        mag: Option<&Vector3<f32>>,
    ) -> Option<Attitude> {
        // Into the airframe frame; everything downstream (AHRS, rates)
        // then sees a level-mounted IMU.
        let gyro = self.mount * gyro;
        let accel = self.mount * accel;
        let rates = self.rate_filter.apply(&gyro);

        let q = match mag {
            Some(mag) => self.ahrs.update(&gyro, &accel, mag),
            None => self.ahrs.update_imu(&gyro, &accel),
        }
        .ok()?;
        let (roll, pitch, yaw) = q.euler_angles();

        // Roll sign is inverted relative to the ICM-42686's raw frame on this PCB
        // orientation — field-confirmed rolling the wrong way with identity mapping.
        Some(Attitude {
            pitch,
            roll: -roll,
            yaw,
            pitch_rate: rates.y,
            roll_rate: -rates.x,
            yaw_rate: rates.z,
        })
    }
}
