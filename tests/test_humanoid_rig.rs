use active_ragdoll_rs::humanoid::StandardHumanoidRig;
use active_ragdoll_rs::prelude::*;

/// Verifies that the standard 11-bone humanoid rig matches
/// Unity ActiveRagdoll.prefab specifications.
#[test]
fn test_standard_humanoid_11_rig_integrity() {
    let rig = StandardHumanoidRig::new();
    assert_eq!(rig.bones.len(), 11);

    // Verify root bone is Hips and has no parent
    let hips = &rig.bones[0];
    assert_eq!(hips.name, "Hips");
    assert_eq!(hips.bone, HumanoidBone::Hips);
    assert!(hips.parent_index.is_none());

    // Verify limbs 1..10 all have valid parents earlier in the array
    for (i, bone) in rig.bones.iter().enumerate().skip(1) {
        let parent = bone.parent_index.unwrap_or_else(|| panic!("Bone {} must have a parent", bone.name));
        assert!(parent < i, "Parent must precede child in hierarchy");
    }

    // Verify total character mass is realistic humanoid mass (~70-80kg)
    let total_mass = rig.total_mass();
    assert!((65.0..=85.0).contains(&total_mass), "Total mass {}kg outside expected humanoid range", total_mass);

    // Verify builder builds clean 11-limb setup
    let setup = HumanoidActiveRagdollBuilder::standard_humanoid_11()
        .build()
        .expect("Builder must succeed");
    assert_eq!(setup.limb_count(), 11);
    assert_eq!(setup.state(), RagdollState::FollowingAnimation);
}
