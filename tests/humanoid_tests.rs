use active_ragdoll_rs::prelude::*;
use glam::{Quat, Vec3};

#[test]
fn test_standard_humanoid_rig_anatomy() {
    let rig = StandardHumanoidRig::new();
    assert_eq!(rig.bones.len(), 11);

    // Verify parent relationships
    assert_eq!(rig.bones[0].name, "Hips");
    assert!(rig.bones[0].parent_index.is_none());

    // Left leg: Hips -> LeftUpperLeg -> LeftLowerLeg
    assert_eq!(rig.bones[1].parent_index, Some(0));
    assert_eq!(rig.bones[2].parent_index, Some(1));

    // Right leg: Hips -> RightUpperLeg -> RightLowerLeg
    assert_eq!(rig.bones[3].parent_index, Some(0));
    assert_eq!(rig.bones[4].parent_index, Some(3));

    // Spine & Arms: Hips -> Spine -> LeftUpperArm -> LeftLowerArm
    assert_eq!(rig.bones[5].parent_index, Some(0));
    assert_eq!(rig.bones[6].parent_index, Some(5));
    assert_eq!(rig.bones[7].parent_index, Some(6));

    // Head: Spine -> Head
    assert_eq!(rig.bones[8].parent_index, Some(5));

    // Right arm: Spine -> RightUpperArm -> RightLowerArm
    assert_eq!(rig.bones[9].parent_index, Some(5));
    assert_eq!(rig.bones[10].parent_index, Some(9));

    // Total mass should be realistic humanoid (~75kg)
    let total_mass = rig.total_mass();
    assert!((total_mass - 75.0).abs() < 5.0);
}

#[test]
fn test_humanoid_builder_and_step_character() {
    let mut setup = HumanoidActiveRagdollBuilder::standard_humanoid_11()
        .with_master_initial_position(Vec3::new(0.0, 0.95, 0.0))
        .build()
        .expect("Builder should succeed");

    assert_eq!(setup.limb_count(), 11);
    assert_eq!(setup.state(), RagdollState::FollowingAnimation);
    assert!(setup.is_alive());

    let slave_states = vec![
        SlaveLimbPhysicsState {
            world_center_of_mass: Vec3::new(0.0, 0.95, 0.0),
            world_rotation: Quat::IDENTITY,
            local_rotation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
        };
        11
    ];

    let input = LocomotionInput { horizontal: 0.0, vertical: 1.0, run: false };
    let outputs = setup
        .step_character(&input, true, &slave_states, 0.02)
        .expect("step_character should succeed");

    assert_eq!(outputs.len(), 11);
    assert_eq!(setup.master.state, CharacterState::Walking);
}

#[test]
fn test_humanoid_die_and_come_alive() {
    let mut setup = HumanoidActiveRagdollBuilder::standard_humanoid_11()
        .build()
        .unwrap();

    setup.die();
    assert!(!setup.is_alive());
    assert_eq!(setup.state(), RagdollState::Dead);
    assert_eq!(setup.current_strength(), 0.0);

    setup.come_alive();
    assert!(setup.is_alive());
}

#[test]
fn test_humanoid_set_limb_profile_and_bounds() {
    let mut setup = HumanoidActiveRagdollBuilder::standard_humanoid_11()
        .build()
        .unwrap();

    let custom_profile = LimbProfile {
        max_force_scale: 0.8,
        max_joint_torque_scale: 1.2,
        joint_damping_scale: 1.5,
        enabled: true,
    };

    setup.set_limb_profile(1, custom_profile).unwrap();
    assert_eq!(setup.follower.limbs[1].profile, custom_profile);

    // Out of bounds limb index
    let err = setup.set_limb_profile(99, custom_profile);
    assert!(matches!(err, Err(ActiveRagdollError::IndexOutOfBounds(99, 11))));
}
