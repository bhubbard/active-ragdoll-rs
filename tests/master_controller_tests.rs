use active_ragdoll_rs::prelude::*;
use glam::Vec3;

#[test]
fn test_master_state_machine_and_anim_cond() {
    let mut master = MasterController::default();

    // 1. Idle
    let idle_input = LocomotionInput { horizontal: 0.0, vertical: 0.0, run: false };
    master.step(&idle_input, 0.0, master.position, true, 0.02);
    assert_eq!(master.state, CharacterState::Idle);
    assert_eq!(master.anim_cond(), 0);

    // 2. Walking
    let walk_input = LocomotionInput { horizontal: 0.0, vertical: 1.0, run: false };
    master.step(&walk_input, 0.0, master.position, true, 0.02);
    assert_eq!(master.state, CharacterState::Walking);
    assert_eq!(master.anim_cond(), 1);

    // 3. Running
    let run_input = LocomotionInput { horizontal: 0.0, vertical: 1.0, run: true };
    master.step(&run_input, 0.0, master.position, true, 0.02);
    assert_eq!(master.state, CharacterState::Running);
    assert_eq!(master.anim_cond(), 2);

    // 4. Falling
    master.step(&walk_input, 0.0, master.position, false, 0.02);
    assert_eq!(master.state, CharacterState::Falling);
    assert_eq!(master.anim_cond(), 3);
}

#[test]
fn test_master_falling_gravity() {
    let mut master = MasterController::new(
        MasterControllerConfig { gravity: 10.0, ..Default::default() },
        Vec3::new(0.0, 5.0, 0.0),
        0.0,
    );

    let input = LocomotionInput::default();
    master.step(&input, 0.0, master.position, false, 0.1);

    // After 0.1s: fall velocity = -1.0, pos.y = 5.0 - 0.1 = 4.9
    assert_eq!(master.state, CharacterState::Falling);
    assert!((master.current_fall_velocity - (-1.0)).abs() < 1e-4);
    assert!((master.position.y - 4.9).abs() < 1e-4);
}

#[test]
fn test_push_back_movement_clamping() {
    let master = MasterController::new(
        MasterControllerConfig {
            max_possible_distance_from_ragdoll: 2.0,
            ..Default::default()
        },
        Vec3::ZERO,
        0.0,
    );

    // Ragdoll behind master (-3.0m on Z) -> distance exceeds 2.0m -> ratio = 1.0 -> magnitude = 1.0
    let pb = master.calculate_push_back_movement(Vec3::new(0.0, 0.0, -3.0));
    assert_eq!(pb.y, 0.0);
    assert!(pb.z < 0.0);
    assert!((pb.length() - 1.0).abs() < 1e-4);
}

#[test]
fn test_smooth_turning_with_camera() {
    let mut master = MasterController::new(
        MasterControllerConfig { turn_smooth_time: 0.1, ..Default::default() },
        Vec3::ZERO,
        0.0,
    );

    // Walking right (+X) with camera facing north (yaw = 0) -> target angle = 90 deg
    let input = LocomotionInput { horizontal: 1.0, vertical: 0.0, run: false };

    for _ in 0..60 {
        master.step(&input, 0.0, master.position, true, 0.02);
    }

    // Should have smoothly turned to ~90 degrees
    assert!((master.rotation_y_deg - 90.0).abs() < 2.0);
}

#[test]
fn test_ground_check_sphere() {
    let master = MasterController::new(
        MasterControllerConfig { ground_distance: 0.1, ..Default::default() },
        Vec3::new(0.0, 0.05, 0.0),
        0.0,
    );

    assert!(master.check_sphere_grounded(0.0));
    assert!(!master.check_sphere_grounded(-1.0));
}

#[test]
fn test_forward_kinematics_poses() {
    let master = MasterController::default();
    let poses = master.generate_master_poses();

    assert_eq!(poses.world_transforms.len(), 11);
    assert_eq!(poses.local_rotations.len(), 11);

    for (i, xf) in poses.world_transforms.iter().enumerate() {
        assert!(xf.position.is_finite(), "Bone {} position must be finite", i);
        assert!(xf.rotation.is_finite(), "Bone {} rotation must be finite", i);
    }
}
