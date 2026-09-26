use glam::Vec3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// High-velocity projectile for knock-down testing of active ragdolls.
///
/// Port of Unity's `ShootObject.cs`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Projectile {
    /// World position of projectile.
    pub position: Vec3,
    /// Velocity vector in m/s.
    pub velocity: Vec3,
    /// Radius of the spherical projectile in meters.
    pub radius: f32,
    /// Mass of the projectile in kg (e.g. 5.0 kg cannonball or 0.5 kg ball).
    pub mass: f32,
    /// Whether this projectile is currently active.
    pub active: bool,
    /// Total flight time in seconds.
    pub lifetime: f32,
}

impl Default for Projectile {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            radius: 0.15,
            mass: 4.0,
            active: true,
            lifetime: 0.0,
        }
    }
}

impl Projectile {
    /// Spawns a projectile from a shooter position looking in `forward` direction.
    ///
    /// Matches `ShootObject.cs`:
    /// `position = origin + forward * 1.5`
    /// `velocity = forward * speed (default 21.0 m/s)`
    pub fn shoot(origin: Vec3, forward: Vec3, speed: f32, radius: f32, mass: f32) -> Self {
        let forward = forward.normalize_or_zero();
        Self {
            position: origin + forward * 1.5,
            velocity: forward * speed,
            radius,
            mass,
            active: true,
            lifetime: 0.0,
        }
    }

    /// Advance projectile physics by `dt` with gravity.
    pub fn step(&mut self, gravity: f32, dt: f32) {
        if !self.active || dt <= 0.0 {
            return;
        }

        self.lifetime += dt;
        self.velocity.y -= gravity * dt;
        self.position += self.velocity * dt;

        // Automatically deactivate if underground or too old
        if self.position.y < -5.0 || self.lifetime > 10.0 {
            self.active = false;
        }
    }

    /// Check for collision against a sphere at `target_center` with `target_radius`.
    ///
    /// Returns impulse vector if a collision occurs on this step.
    pub fn check_sphere_collision(
        &mut self,
        target_center: Vec3,
        target_radius: f32,
        target_mass: f32,
    ) -> Option<Vec3> {
        if !self.active {
            return None;
        }

        let dist_sq = (self.position - target_center).length_squared();
        let min_dist = self.radius + target_radius;

        if dist_sq <= min_dist * min_dist {
            self.active = false;
            // Elastic collision impulse approximation
            let normal = (target_center - self.position).normalize_or_zero();
            let relative_vel = self.velocity.dot(normal);

            if relative_vel > 0.0 {
                // Reduced mass impulse: J = (1 + e) * m_eff * v_rel
                let m_eff = (self.mass * target_mass) / (self.mass + target_mass);
                let impulse_mag = 1.6 * m_eff * relative_vel;
                return Some(normal * impulse_mag);
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shoot_projectile() {
        let proj = Projectile::shoot(Vec3::ZERO, Vec3::Z, 21.0, 0.15, 4.0);
        assert_eq!(proj.position, Vec3::new(0.0, 0.0, 1.5));
        assert_eq!(proj.velocity, Vec3::new(0.0, 0.0, 21.0));
        assert!(proj.active);
    }

    #[test]
    fn test_projectile_hit() {
        let mut proj = Projectile::shoot(Vec3::ZERO, Vec3::Z, 20.0, 0.2, 5.0);
        let target_pos = Vec3::new(0.0, 0.0, 1.6);
        let impulse = proj.check_sphere_collision(target_pos, 0.2, 10.0);

        assert!(impulse.is_some());
        assert!(!proj.active);
    }
}
