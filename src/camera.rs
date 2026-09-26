use glam::{Quat, Vec2, Vec3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// 3D Fly / Orbit camera controller for debugging and observing active ragdoll physics.
///
/// Port of Unity's `FlyCamera.cs`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FlyCamera {
    /// Camera world position.
    pub position: Vec3,
    /// Yaw angle around world Y axis (degrees).
    pub yaw_deg: f32,
    /// Pitch angle around local X axis (degrees).
    pub pitch_deg: f32,
    /// Mouse sensitivity multiplier.
    pub sensitivity: f32,
    /// Normal move speed in m/s.
    pub normal_move_speed: f32,
    /// Fast move multiplier (holding Shift).
    pub fast_multiplier: f32,
    /// Slow move multiplier (holding Ctrl).
    pub slow_multiplier: f32,
    /// Vertical climb/descend speed in m/s.
    pub climb_speed: f32,
    /// Pitch rotation limits [min_pitch, max_pitch] in degrees.
    pub pitch_limits: Vec2,
}

impl Default for FlyCamera {
    fn default() -> Self {
        Self {
            position: Vec3::new(0.0, 1.8, -4.0),
            yaw_deg: 0.0,
            pitch_deg: 10.0,
            sensitivity: 90.0,
            normal_move_speed: 10.0,
            fast_multiplier: 3.0,
            slow_multiplier: 0.25,
            climb_speed: 4.0,
            pitch_limits: Vec2::new(-85.0, 85.0),
        }
    }
}

impl FlyCamera {
    /// Add mouse look rotation (delta_x = yaw change, delta_y = pitch change).
    pub fn rotate(&mut self, delta_x: f32, delta_y: f32) {
        self.yaw_deg += delta_x * self.sensitivity;
        self.pitch_deg = (self.pitch_deg - delta_y * self.sensitivity)
            .clamp(self.pitch_limits.x, self.pitch_limits.y);
    }

    /// Rotation quaternion for the camera.
    #[inline]
    pub fn rotation(&self) -> Quat {
        let yaw = Quat::from_rotation_y(self.yaw_deg.to_radians());
        let pitch = Quat::from_rotation_x(self.pitch_deg.to_radians());
        yaw * pitch
    }

    /// Forward direction vector.
    #[inline]
    pub fn forward(&self) -> Vec3 {
        self.rotation() * Vec3::Z
    }

    /// Right direction vector.
    #[inline]
    pub fn right(&self) -> Vec3 {
        self.rotation() * Vec3::X
    }

    /// Up direction vector.
    #[inline]
    pub fn up(&self) -> Vec3 {
        self.rotation() * Vec3::Y
    }

    /// Translate camera given input directions and modifiers.
    pub fn translate(&mut self, move_input: Vec3, fast: bool, slow: bool, dt: f32) {
        let mut speed = self.normal_move_speed;
        if fast {
            speed *= self.fast_multiplier;
        } else if slow {
            speed *= self.slow_multiplier;
        }

        let delta = (self.right() * move_input.x
            + Vec3::Y * move_input.y
            + self.forward() * move_input.z)
            * speed
            * dt;

        self.position += delta;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_rotation() {
        let mut cam = FlyCamera {
            yaw_deg: 0.0,
            pitch_deg: 0.0,
            ..Default::default()
        };

        let fwd = cam.forward();
        assert!((fwd - Vec3::Z).length() < 1e-4);

        cam.yaw_deg = 90.0;
        let fwd90 = cam.forward();
        assert!((fwd90 - Vec3::X).length() < 1e-4);
    }
}
