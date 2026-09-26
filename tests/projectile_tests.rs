use active_ragdoll_rs::prelude::*;
use glam::Vec3;

#[test]
fn test_projectile_trajectory_and_gravity() {
    let mut proj = Projectile::shoot(Vec3::ZERO, Vec3::Z, 20.0, 0.15, 5.0);
    assert_eq!(proj.velocity, Vec3::new(0.0, 0.0, 20.0));

    // Step 0.5s with 10.0 m/s^2 gravity
    proj.step(10.0, 0.5);

    assert_eq!(proj.velocity.z, 20.0);
    assert!((proj.velocity.y - (-5.0)).abs() < 1e-4);
    assert!((proj.position.z - (1.5 + 10.0)).abs() < 1e-4);
    assert!(proj.active);
}

#[test]
fn test_projectile_collision_impulse() {
    let mut proj = Projectile::shoot(Vec3::ZERO, Vec3::Z, 25.0, 0.2, 4.0);
    let target_pos = Vec3::new(0.0, 0.0, 1.6);
    let target_radius = 0.2;
    let target_mass = 20.0;

    let impulse = proj.check_sphere_collision(target_pos, target_radius, target_mass);
    assert!(impulse.is_some());
    let imp = impulse.unwrap();
    assert!(imp.z > 0.0);
    assert!(!proj.active);
}

#[test]
fn test_projectile_ground_and_lifetime_deactivation() {
    let mut proj = Projectile::shoot(Vec3::new(0.0, -4.9, 0.0), Vec3::new(0.0, -10.0, 0.0), 10.0, 0.1, 1.0);
    proj.step(9.81, 0.1);
    // Y < -5.0 -> should deactivate
    assert!(!proj.active);

    let mut old_proj = Projectile::shoot(Vec3::ZERO, Vec3::X, 1.0, 0.1, 1.0);
    old_proj.lifetime = 10.1;
    old_proj.step(0.0, 0.1);
    assert!(!old_proj.active);
}
