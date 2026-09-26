#[cfg(feature = "rapier3d")]
#[test]
fn test_rapier_adapter_physical_simulation() {
    use active_ragdoll_rs::prelude::*;
    use glam::{Quat, Vec3};
    use rapier3d::prelude::*;

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();

    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let integration_parameters = IntegrationParameters {
        dt: 0.02,
        ..Default::default()
    };
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();

    // Create ground
    let ground_collider = ColliderBuilder::cuboid(10.0, 0.1, 10.0)
        .translation(Vec3::new(0.0, -0.1, 0.0))
        .build();
    collider_set.insert(ground_collider);

    // Create hips body
    let hips_rb = RigidBodyBuilder::dynamic()
        .translation(Vec3::new(0.0, 1.0, 0.0))
        .build();
    let hips_handle = rigid_body_set.insert(hips_rb);
    let hips_collider = ColliderBuilder::cuboid(0.2, 0.15, 0.15).build();
    collider_set.insert_with_parent(hips_collider, hips_handle, &mut rigid_body_set);

    // Create spine body
    let spine_rb = RigidBodyBuilder::dynamic()
        .translation(Vec3::new(0.0, 1.35, 0.0))
        .build();
    let spine_handle = rigid_body_set.insert(spine_rb);
    let spine_collider = ColliderBuilder::cuboid(0.18, 0.2, 0.12).build();
    collider_set.insert_with_parent(spine_collider, spine_handle, &mut rigid_body_set);

    // Connect hips and spine with spherical joint
    let joint_data = SphericalJointBuilder::new()
        .local_anchor1(Vec3::new(0.0, 0.15, 0.0))
        .local_anchor2(Vec3::new(0.0, -0.2, 0.0))
        .build();
    let joint_handle = impulse_joint_set.insert(hips_handle, spine_handle, joint_data, true);

    // Build ragdoll
    let ragdoll = HumanoidActiveRagdollBuilder::new()
        .add_limb(ActiveLimb::new(
            "Hips",
            Some(HumanoidBone::Hips),
            Vec3::ZERO,
            None,
            15.0,
            0.02,
        ))
        .add_limb(ActiveLimb::new(
            "Spine",
            Some(HumanoidBone::Spine),
            Vec3::new(0.0, 0.1, 0.0),
            Some(JointSpace::from_axes(Vec3::X, Vec3::Y, Quat::IDENTITY)),
            15.0,
            0.02,
        ))
        .build()
        .unwrap();

    let bindings = vec![
        RapierLimbBinding::new(hips_handle, None, None, 9),
        RapierLimbBinding::new(spine_handle, Some(hips_handle), Some(joint_handle), 9),
    ];
    let mut adapter = RapierRagdollAdapter::new(ragdoll, bindings).unwrap();

    let dt = 0.02;
    let master_world = [
        Transform3d::from_translation(Vec3::new(0.0, 1.0, 0.0)),
        Transform3d::from_translation(Vec3::new(0.0, 1.35, 0.0)),
    ];
    let master_local = [Quat::IDENTITY, Quat::IDENTITY];

    // Step simulation 10 times
    for _ in 0..10 {
        adapter
            .step(
                &master_world,
                &master_local,
                &mut rigid_body_set,
                Some(&mut impulse_joint_set),
                dt,
            )
            .unwrap();

        physics_pipeline.step(
            gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut Default::default(),
            &mut ccd_solver,
            &physics_hooks,
            &event_handler,
        );
    }

    // Verify bodies did not fall through floor and remain upright
    let hips_body = rigid_body_set.get(hips_handle).unwrap();
    let spine_body = rigid_body_set.get(spine_handle).unwrap();

    assert!(hips_body.translation().y > 0.5, "Hips must not collapse below 0.5m");
    assert!(spine_body.translation().y > hips_body.translation().y, "Spine must remain above hips");
}
