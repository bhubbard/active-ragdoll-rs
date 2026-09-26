use active_ragdoll_rs::prelude::*;
use glam::{Quat, Vec3};

#[test]
fn test_follower_linear_and_angular_drive() {
    let rig = StandardHumanoidRig::new();
    let mut follower = rig.build_animation_follower(AnimationFollowerConfig::default());
    let count = follower.limbs.len();

    let master_xfs = vec![Transform3d::IDENTITY; count];
    let master_local_rots = vec![Quat::IDENTITY; count];

    // Slave limbs offset downward by 0.2m on Y
    let slave_states = vec![
        SlaveLimbPhysicsState {
            world_center_of_mass: Vec3::new(0.0, -0.2, 0.0),
            world_rotation: Quat::IDENTITY,
            local_rotation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
        };
        count
    ];

    let outputs = follower
        .step(&master_xfs, &master_local_rots, &slave_states, 1.0, 1.0, true, 0.02)
        .expect("Follower step should succeed");

    assert_eq!(outputs.len(), count);

    // Root (Hips): no joint target, but positive upward linear force
    assert_eq!(outputs[0].target_rotation, None);
    assert!(outputs[0].linear_force.y > 0.0);
    assert_eq!(outputs[0].joint_spring, 0.0);

    // Child limb (Spine): has joint target, joint spring > 0
    assert!(outputs[5].target_rotation.is_some());
    assert!(outputs[5].joint_spring > 0.0);
    assert!(outputs[5].joint_damper > 0.0);
    assert!(outputs[5].joint_limits_enabled);
}

#[test]
fn test_follower_limb_profile_scaling_and_disabling() {
    let rig = StandardHumanoidRig::new();
    let mut follower = rig.build_animation_follower(AnimationFollowerConfig::default());

    // Disable Spine (index 5)
    follower.limbs[5].profile.enabled = false;
    // Scale LeftUpperLeg max force by 0.5
    follower.limbs[1].profile.max_force_scale = 0.5;

    let count = follower.limbs.len();
    let master_xfs = vec![Transform3d::from_translation(Vec3::new(0.0, 1.0, 0.0)); count];
    let master_local = vec![Quat::IDENTITY; count];
    let slave_states = vec![
        SlaveLimbPhysicsState {
            world_center_of_mass: Vec3::ZERO,
            world_rotation: Quat::IDENTITY,
            local_rotation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
        };
        count
    ];

    let outputs = follower
        .step(&master_xfs, &master_local, &slave_states, 1.0, 1.0, true, 0.02)
        .unwrap();

    // Disabled spine should output zero force and zero torque
    assert_eq!(outputs[5].linear_force, Vec3::ZERO);
    assert_eq!(outputs[5].joint_spring, 0.0);
    assert_eq!(outputs[5].target_rotation, None);

    // LeftUpperLeg should have force clamped to 0.5 * max_force (5.0)
    assert!(outputs[1].linear_force.length() <= 5.0 + 1e-4);
}

#[test]
fn test_follower_toggle_joint_limits() {
    let rig = StandardHumanoidRig::new();
    let mut follower = rig.build_animation_follower(AnimationFollowerConfig::default());
    assert!(follower.config.joint_limits_enabled);

    follower.enable_joint_limits(false);
    assert!(!follower.config.joint_limits_enabled);

    let count = follower.limbs.len();
    let outputs = follower
        .step(
            &vec![Transform3d::IDENTITY; count],
            &vec![Quat::IDENTITY; count],
            &vec![
                SlaveLimbPhysicsState {
                    world_center_of_mass: Vec3::ZERO,
                    world_rotation: Quat::IDENTITY,
                    local_rotation: Quat::IDENTITY,
                    angular_velocity: Vec3::ZERO,
                };
                count
            ],
            1.0,
            1.0,
            true,
            0.02,
        )
        .unwrap();

    assert!(!outputs[1].joint_limits_enabled);
}

#[test]
fn test_follower_hierarchy_mismatch_error() {
    let rig = StandardHumanoidRig::new();
    let mut follower = rig.build_animation_follower(AnimationFollowerConfig::default());

    let wrong_xfs = vec![Transform3d::IDENTITY; 5]; // only 5 instead of 11
    let local = vec![Quat::IDENTITY; 11];
    let slave = vec![
        SlaveLimbPhysicsState {
            world_center_of_mass: Vec3::ZERO,
            world_rotation: Quat::IDENTITY,
            local_rotation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
        };
        11
    ];

    let res = follower.step(&wrong_xfs, &local, &slave, 1.0, 1.0, true, 0.02);
    assert!(matches!(res, Err(ActiveRagdollError::HierarchyMismatch { .. })));
}
