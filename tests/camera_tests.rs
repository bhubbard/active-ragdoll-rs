use active_ragdoll_rs::prelude::*;
use glam::Vec3;

#[test]
fn test_fly_camera_look_and_pitch_clamping() {
    let mut cam = FlyCamera::default();
    cam.yaw_deg = 0.0;
    cam.pitch_deg = 0.0;
    cam.sensitivity = 1.0;

    // Rotate mouse
    cam.rotate(10.0, 5.0);
    assert_eq!(cam.yaw_deg, 10.0);
    assert_eq!(cam.pitch_deg, -5.0);

    // Extreme pitch rotation -> clamped to limits [-85, 85]
    cam.rotate(0.0, -200.0);
    assert_eq!(cam.pitch_deg, 85.0);

    cam.rotate(0.0, 300.0);
    assert_eq!(cam.pitch_deg, -85.0);
}

#[test]
fn test_fly_camera_basis_vectors() {
    let mut cam = FlyCamera::default();
    cam.yaw_deg = 0.0;
    cam.pitch_deg = 0.0;

    let fwd = cam.forward();
    let right = cam.right();
    let up = cam.up();

    // Verify orthonormal frame
    assert!((fwd.dot(right)).abs() < 1e-5);
    assert!((fwd.dot(up)).abs() < 1e-5);
    assert!((right.dot(up)).abs() < 1e-5);
}

#[test]
fn test_fly_camera_translation_speed_modifiers() {
    let mut cam = FlyCamera {
        normal_move_speed: 10.0,
        fast_multiplier: 3.0,
        slow_multiplier: 0.25,
        ..Default::default()
    };
    cam.yaw_deg = 0.0;
    cam.pitch_deg = 0.0;
    cam.position = Vec3::ZERO;

    // Normal translation forward (1.0 on Z) for 1.0s
    cam.translate(Vec3::new(0.0, 0.0, 1.0), false, false, 1.0);
    assert!((cam.position.z - 10.0).abs() < 1e-4);

    // Fast translation forward with Shift
    cam.position = Vec3::ZERO;
    cam.translate(Vec3::new(0.0, 0.0, 1.0), true, false, 1.0);
    assert!((cam.position.z - 30.0).abs() < 1e-4);

    // Slow translation forward with Ctrl
    cam.position = Vec3::ZERO;
    cam.translate(Vec3::new(0.0, 0.0, 1.0), false, true, 1.0);
    assert!((cam.position.z - 2.5).abs() < 1e-4);
}
