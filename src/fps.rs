#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Exponential moving average frame rate and delta time tracker.
///
/// Direct port of Unity's `FPSDisplay.cs`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FpsDisplay {
    /// Smoothed delta time in seconds.
    pub delta_time: f32,
    /// Exponential smoothing weight factor (default 0.1).
    pub smoothing: f32,
}

impl Default for FpsDisplay {
    fn default() -> Self {
        Self {
            delta_time: 0.0,
            smoothing: 0.1,
        }
    }
}

impl FpsDisplay {
    /// Create a new FPS tracker with custom smoothing weight.
    pub const fn new(smoothing: f32) -> Self {
        Self {
            delta_time: 0.0,
            smoothing,
        }
    }

    /// Update with the latest frame's unscaled delta time (seconds).
    ///
    /// Matches `FPSDisplay.cs`:
    /// `deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;`
    pub fn update(&mut self, unscaled_delta_time: f32) {
        if unscaled_delta_time > 0.0 && unscaled_delta_time.is_finite() {
            if self.delta_time <= 0.0 {
                self.delta_time = unscaled_delta_time;
            } else {
                self.delta_time += (unscaled_delta_time - self.delta_time) * self.smoothing;
            }
        }
    }

    /// Calculated frames per second ($1 / \Delta t$).
    #[inline]
    pub fn fps(&self) -> f32 {
        if self.delta_time > 0.0 {
            1.0 / self.delta_time
        } else {
            0.0
        }
    }

    /// Frame duration in milliseconds ($\Delta t \cdot 1000$).
    #[inline]
    pub fn frame_time_ms(&self) -> f32 {
        self.delta_time * 1000.0
    }

    /// Format string matching Unity: `"{0:0.0} ms ({1:0.} fps)"`.
    pub fn format_text(&self) -> String {
        format!("{:.1} ms ({:.0} fps)", self.frame_time_ms(), self.fps())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fps_display() {
        let mut fps = FpsDisplay::default();
        let dt = 1.0 / 60.0; // ~0.01666s -> 60 fps

        for _ in 0..100 {
            fps.update(dt);
        }

        assert!((fps.fps() - 60.0).abs() < 0.5);
        assert!((fps.frame_time_ms() - 16.666).abs() < 0.5);
        assert!(fps.format_text().contains("60 fps"));
    }
}
