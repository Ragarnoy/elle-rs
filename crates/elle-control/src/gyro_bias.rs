//! Gyro zero-rate offset ("bias"), measured at boot while the airframe sits still.
//!
//! The attitude PID's damping term uses the gyro rate directly, so a bias shows up
//! as a constant elevon offset the (deliberately small) integral has to fight, and
//! the Madgwick filter at a low beta removes little of it. Averaging a still second
//! at boot is what most flight stacks do; the window restarts whenever the spread
//! says the board moved.
//!
//! Pure math, no hardware: the Core1 driver feeds raw samples and applies the result.

use elle_config::{
    GYRO_BIAS_MAX_RAD_S, GYRO_BIAS_MAX_SPREAD_RAD_S, GYRO_BIAS_SAMPLES, GYRO_BIAS_TIMEOUT_SAMPLES,
};
use nalgebra::Vector3;

/// Why no bias was produced.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum GyroBiasFail {
    /// No still window within the timeout: the board kept moving.
    Timeout,
    /// A still window averaged to more than the plausible offset.
    TooLarge,
}

/// Collects raw gyro samples until one still window has been averaged.
pub struct GyroBiasEstimator {
    window: u32,
    max_spread: f32,
    max_bias: f32,
    timeout: u32,
    seen: u32,
    n: u32,
    sum: Vector3<f32>,
    min: Vector3<f32>,
    max: Vector3<f32>,
}

impl GyroBiasEstimator {
    /// Estimator with the `elle-config` limits.
    #[must_use]
    pub fn new() -> Self {
        Self::with_limits(
            GYRO_BIAS_SAMPLES,
            GYRO_BIAS_MAX_SPREAD_RAD_S,
            GYRO_BIAS_MAX_RAD_S,
            GYRO_BIAS_TIMEOUT_SAMPLES,
        )
    }

    /// Estimator with explicit limits: `window` samples averaged, restart when any
    /// axis spreads more than `max_spread`, reject a mean above `max_bias`, and
    /// give up after `timeout` samples in total.
    #[must_use]
    pub fn with_limits(window: u32, max_spread: f32, max_bias: f32, timeout: u32) -> Self {
        Self {
            window: window.max(1),
            max_spread,
            max_bias,
            timeout,
            seen: 0,
            n: 0,
            sum: Vector3::zeros(),
            min: Vector3::repeat(f32::MAX),
            max: Vector3::repeat(f32::MIN),
        }
    }

    fn restart(&mut self) {
        self.n = 0;
        self.sum = Vector3::zeros();
        self.min = Vector3::repeat(f32::MAX);
        self.max = Vector3::repeat(f32::MIN);
    }

    /// Feed one raw gyro sample (rad/s). `None` while still collecting; once
    /// finished, the bias to subtract or why there is none. Call no further after
    /// a `Some`.
    ///
    /// # Errors
    ///
    /// `Timeout` if no still window completed within the timeout; `TooLarge` if a
    /// still window's mean exceeds the plausible offset on any axis.
    pub fn push(&mut self, raw: Vector3<f32>) -> Option<Result<Vector3<f32>, GyroBiasFail>> {
        self.seen += 1;
        if !raw.iter().all(|v| v.is_finite()) {
            self.restart();
        } else {
            self.sum += raw;
            self.min = self.min.inf(&raw);
            self.max = self.max.sup(&raw);
            self.n += 1;
            if (self.max - self.min).max() > self.max_spread {
                // Moved: start over from this sample.
                self.restart();
                self.sum = raw;
                self.min = raw;
                self.max = raw;
                self.n = 1;
            }
        }

        if self.n >= self.window {
            let mean = self.sum / self.n as f32;
            return Some(if mean.amax() > self.max_bias {
                Err(GyroBiasFail::TooLarge)
            } else {
                Ok(mean)
            });
        }
        if self.seen >= self.timeout {
            return Some(Err(GyroBiasFail::Timeout));
        }
        None
    }
}

impl Default for GyroBiasEstimator {
    fn default() -> Self {
        Self::new()
    }
}
