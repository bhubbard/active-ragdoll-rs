use glam::Vec3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Proportional-Derivative (PD) controller for 3D vector signals.
///
/// Implements the exact algorithm from Unity Humanoid ActiveRagdoll:
/// ```text
/// derivative = (error - last_error) / dt
/// signal = P * (error + D * derivative)
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PdController3d {
    /// Proportional gain coefficient (P)
    pub p_gain: f32,
    /// Derivative gain coefficient (D)
    pub d_gain: f32,
    /// Previous error recorded for finite-difference differentiation
    pub last_error: Vec3,
}

impl PdController3d {
    /// Create a new 3D PD controller with the given gains.
    #[inline]
    pub const fn new(p_gain: f32, d_gain: f32) -> Self {
        Self {
            p_gain,
            d_gain,
            last_error: Vec3::ZERO,
        }
    }

    /// Reset internal state / accumulated error to zero.
    #[inline]
    pub fn reset(&mut self) {
        self.last_error = Vec3::ZERO;
    }

    /// Calculate the output signal given the current error and time step.
    ///
    /// If `dt <= 0.0`, returns proportional term only to avoid division by zero or NaN.
    pub fn calculate(&mut self, error: Vec3, dt: f32) -> Vec3 {
        if dt <= 0.0 || !dt.is_finite() {
            self.last_error = error;
            return self.p_gain * error;
        }

        let derivative = (error - self.last_error) / dt;
        let signal = self.p_gain * (error + self.d_gain * derivative);
        self.last_error = error;
        signal
    }

    /// Clamps vector magnitude to `max_magnitude`.
    #[inline]
    pub fn clamp_magnitude(vector: Vec3, max_magnitude: f32) -> Vec3 {
        let sq_mag = vector.length_squared();
        if sq_mag > max_magnitude * max_magnitude && sq_mag > 0.0 {
            vector * (max_magnitude / sq_mag.sqrt())
        } else {
            vector
        }
    }
}

/// Proportional-Derivative (PD) controller for 1D scalar signals.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PdController1d {
    pub p_gain: f32,
    pub d_gain: f32,
    pub last_error: f32,
}

impl PdController1d {
    #[inline]
    pub const fn new(p_gain: f32, d_gain: f32) -> Self {
        Self {
            p_gain,
            d_gain,
            last_error: 0.0,
        }
    }

    #[inline]
    pub fn reset(&mut self) {
        self.last_error = 0.0;
    }

    pub fn calculate(&mut self, error: f32, dt: f32) -> f32 {
        if dt <= 0.0 || !dt.is_finite() {
            self.last_error = error;
            return self.p_gain * error;
        }

        let derivative = (error - self.last_error) / dt;
        let signal = self.p_gain * (error + self.d_gain * derivative);
        self.last_error = error;
        signal
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pd_controller_proportional_step() {
        let mut pd = PdController3d::new(8.0, 0.01);
        let error = Vec3::new(1.0, 0.0, 0.0);
        let dt = 0.02; // 50 Hz

        let signal = pd.calculate(error, dt);
        // derivative = (1.0 - 0.0) / 0.02 = 50.0
        // signal = 8.0 * (1.0 + 0.01 * 50.0) = 8.0 * (1.0 + 0.5) = 12.0
        assert!((signal.x - 12.0).abs() < 1e-4);
        assert_eq!(signal.y, 0.0);
        assert_eq!(signal.z, 0.0);
    }

    #[test]
    fn test_pd_controller_steady_state() {
        let mut pd = PdController3d::new(8.0, 0.01);
        let error = Vec3::new(2.0, 3.0, -1.0);
        let dt = 0.02;

        let _ = pd.calculate(error, dt);
        // Second step with same error -> derivative is 0
        let signal2 = pd.calculate(error, dt);
        let expected = 8.0 * error;
        assert!((signal2 - expected).length() < 1e-5);
    }

    #[test]
    fn test_clamp_magnitude() {
        let v = Vec3::new(3.0, 4.0, 0.0); // magnitude = 5.0
        let clamped = PdController3d::clamp_magnitude(v, 2.5);
        assert!((clamped.length() - 2.5).abs() < 1e-5);
        assert!((clamped.x - 1.5).abs() < 1e-5);
        assert!((clamped.y - 2.0).abs() < 1e-5);

        let small = Vec3::new(1.0, 1.0, 0.0);
        let not_clamped = PdController3d::clamp_magnitude(small, 10.0);
        assert_eq!(not_clamped, small);
    }
}
