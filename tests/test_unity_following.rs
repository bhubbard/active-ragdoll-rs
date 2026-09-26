use active_ragdoll_rs::prelude::*;
use glam::{Quat, Vec3};

/// Verifies that AnimationFollowing calculates PD forces and joint drives
/// matching the exact Unity equations.
#[test]
fn test_unity_pd_linear_force_equation() {
    let p_force = 8.0;
    let d_force = 0.01;
    let max_force = 10.0;
    let dt = 0.02; // 50 Hz

    let mut follower = AnimationFollower::new(
        AnimationFollowerConfig {
            p_force,
            d_force,
            max_force,
            ..Default::default()
        },
        vec![ActiveLimb::new(
            "Hips",
            Some(HumanoidBone::Hips),
            Vec3::ZERO,
            None,
            p_force,
            d_force,
        )],
    );

    // Initial step: master at (0, 1, 0), slave at (0, 0.8, 0) -> error = (0, 0.2, 0)
    let master_world = [Transform3d::from_translation(Vec3::new(0.0, 1.0, 0.0))];
    let master_local = [Quat::IDENTITY];
    let slave_states = [SlaveLimbPhysicsState {
        world_center_of_mass: Vec3::new(0.0, 0.8, 0.0),
        world_rotation: Quat::IDENTITY,
        local_rotation: Quat::IDENTITY,
        angular_velocity: Vec3::ZERO,
    }];

    // Step 1:
    // error = (0, 0.2, 0)
    // derivative = (0.2 - 0) / 0.02 = 10.0
    // signal = 8.0 * (0.2 + 0.01 * 10.0) = 8.0 * (0.2 + 0.1) = 2.4
    let outputs1 = follower
        .step(&master_world, &master_local, &slave_states, 1.0, 1.0, true, dt)
        .unwrap();

    assert!((outputs1[0].linear_force.y - 2.4).abs() < 1e-4);
    assert_eq!(outputs1[0].linear_force.x, 0.0);
    assert_eq!(outputs1[0].linear_force.z, 0.0);

    // Step 2: same error -> derivative = 0
    // signal = 8.0 * (0.2 + 0.0) = 1.6
    let outputs2 = follower
        .step(&master_world, &master_local, &slave_states, 1.0, 1.0, true, dt)
        .unwrap();

    assert!((outputs2[0].linear_force.y - 1.6).abs() < 1e-4);
}

#[test]
fn test_unity_joint_space_rotation_math() {
    // In Unity:
    // forward = cross(axis, secondaryAxis)
    // up = secondaryAxis
    // localToJointSpace = LookRotation(forward, up)
    // startLocalRotation = localRotation * localToJointSpace
    // targetRotation = Inv(localToJointSpace) * Inv(masterLocalRotation) * startLocalRotation
    let axis = Vec3::X;
    let secondary = Vec3::Y;
    let initial_local_rot = Quat::IDENTITY;

    let joint_space = JointSpace::from_axes(axis, secondary, initial_local_rot);

    // When masterLocalRotation is identity, targetRotation must equal identity (initial orientation)
    let target_identity = joint_space.compute_target_rotation(Quat::IDENTITY);
    assert!((target_identity - Quat::IDENTITY).length() < 1e-5);

    // When master rotates 30 degrees around X:
    let rot_x_30 = Quat::from_rotation_x(30.0_f32.to_radians());
    let target_rot_x = joint_space.compute_target_rotation(rot_x_30);

    // Target rotation in joint drive space should drive toward matching rotation
    assert!(!target_rot_x.is_nan());
    assert!((target_rot_x.length() - 1.0).abs() < 1e-5);
}

#[test]
fn test_unity_force_clamping_and_profiles() {
    let p_force = 100.0;
    let d_force = 0.0;
    let max_force = 5.0; // clamp at 5.0

    let mut follower = AnimationFollower::new(
        AnimationFollowerConfig {
            p_force,
            d_force,
            max_force,
            ..Default::default()
        },
        vec![
            ActiveLimb::new("LimbA", None, Vec3::ZERO, None, p_force, d_force),
            ActiveLimb::new("LimbB", None, Vec3::ZERO, None, p_force, d_force),
        ],
    );

    // Limb B has half max force profile
    follower.limbs[1].profile.max_force_scale = 0.5;

    let master_world = [
        Transform3d::from_translation(Vec3::new(10.0, 0.0, 0.0)),
        Transform3d::from_translation(Vec3::new(10.0, 0.0, 0.0)),
    ];
    let master_local = [Quat::IDENTITY, Quat::IDENTITY];
    let slave_states = [
        SlaveLimbPhysicsState {
            world_center_of_mass: Vec3::ZERO,
            world_rotation: Quat::IDENTITY,
            local_rotation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
        },
        SlaveLimbPhysicsState {
            world_center_of_mass: Vec3::ZERO,
            world_rotation: Quat::IDENTITY,
            local_rotation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
        },
    ];

    let outputs = follower
        .step(&master_world, &master_local, &slave_states, 1.0, 1.0, true, 0.02)
        .unwrap();

    // LimbA clamped at 5.0
    assert!((outputs[0].linear_force.length() - 5.0).abs() < 1e-5);
    // LimbB clamped at 2.5 (5.0 * 0.5)
    assert!((outputs[1].linear_force.length() - 2.5).abs() < 1e-5);
}

#[test]
fn test_joint_limits_toggle() {
    let mut follower = AnimationFollower::new(
        AnimationFollowerConfig::default(),
        vec![ActiveLimb::new("Hips", None, Vec3::ZERO, None, 8.0, 0.01)],
    );

    assert!(follower.config.joint_limits_enabled);
    follower.enable_joint_limits(false);
    assert!(!follower.config.joint_limits_enabled);
    follower.enable_joint_limits(true);
    assert!(follower.config.joint_limits_enabled);
}
