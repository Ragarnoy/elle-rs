//! Low-pass filtering for the gyro rates the attitude PID damps with.
//!
//! The PID samples the latest gyro rate at the control-loop rate (~83 Hz). Engine
//! vibration sits far above that, so unfiltered it folds down into the loop band
//! and drives the elevons (seen on the eagle: 20-30 deg/s of roll-rate noise with
//! the EDFs running, against 0.1 deg/s with them stopped). Filtering at the IMU's
//! 1 kHz, before the controller downsamples, removes it instead of aliasing it.

use nalgebra::{ComplexField, Vector3};

/// Second-order Butterworth low-pass (bilinear transform, prewarped), direct
/// form II transposed.
#[derive(Clone, Copy, Debug)]
pub struct LowPass2 {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
    primed: bool,
}

impl LowPass2 {
    /// Filter with corner `cutoff_hz` for samples arriving at `sample_hz`.
    /// `cutoff_hz` is clamped below Nyquist.
    #[must_use]
    pub fn new(cutoff_hz: f32, sample_hz: f32) -> Self {
        let fc = cutoff_hz.clamp(1e-3, 0.45 * sample_hz);
        let k = ComplexField::tan(core::f32::consts::PI * fc / sample_hz);
        let q = core::f32::consts::FRAC_1_SQRT_2;
        let norm = 1.0 / (1.0 + k / q + k * k);
        let b0 = k * k * norm;
        Self {
            b0,
            b1: 2.0 * b0,
            b2: b0,
            a1: 2.0 * (k * k - 1.0) * norm,
            a2: (1.0 - k / q + k * k) * norm,
            z1: 0.0,
            z2: 0.0,
            primed: false,
        }
    }

    /// Feed one sample, return the filtered value. The first sample primes the
    /// state to steady state so a non-zero start doesn't ring.
    pub fn apply(&mut self, x: f32) -> f32 {
        if !self.primed {
            // Steady state for constant input x: y = x.
            self.z1 = x - self.b0 * x;
            self.z2 = self.b2 * x - self.a2 * x;
            self.primed = true;
        }
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}

/// One `LowPass2` per gyro axis.
#[derive(Clone, Copy, Debug)]
pub struct GyroFilter([LowPass2; 3]);

impl GyroFilter {
    #[must_use]
    pub fn new(cutoff_hz: f32, sample_hz: f32) -> Self {
        Self([LowPass2::new(cutoff_hz, sample_hz); 3])
    }

    pub fn apply(&mut self, v: &Vector3<f32>) -> Vector3<f32> {
        Vector3::new(
            self.0[0].apply(v.x),
            self.0[1].apply(v.y),
            self.0[2].apply(v.z),
        )
    }
}
