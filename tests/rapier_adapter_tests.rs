#[cfg(feature = "rapier3d")]
mod rapier_tests {
    use active_ragdoll_rs::prelude::*;
    use glam::{Quat, Vec3};
    use rapier3d::prelude::*;

    #[test]
    fn test_rapier_adapter_full_cycle() {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = ImpulseJointSet::new();

        let rig = StandardHumanoidRig::new();
        let setup = HumanoidActiveRagdollBuilder::standard_humanoid_11()
            .build()
            .unwrap();

        let mut body_handles = Vec::new();
        let mut bindings = Vec::new();

        for (i, def) in rig.bones.iter().enumerate() {
            let rb = RigidBodyBuilder::dynamic()
                .translation(Vec3::new(0.0, 1.0 + (i as f32) * 0.1, 0.0))
                .additional_mass(def.mass)
                .build();
            let handle = bodies.insert(rb);
            body_handles.push(handle);

            // Add collider so body has world center of mass matching its position
            let col = ColliderBuilder::ball(0.1).build();
            colliders.insert_with_parent(col, handle, &mut bodies);

            let (parent_h, joint_h) = if let Some(p_idx) = def.parent_index {
                let p = body_handles[p_idx];
                let j = SphericalJointBuilder::new().build();
                let jh = joints.insert(p, handle, j, true);
                (Some(p), Some(jh))
            } else {
                (None, None)
            };

            bindings.push(RapierLimbBinding::new(
                handle,
                parent_h,
                joint_h,
                LayerMask::RAGDOLL.bits().trailing_zeros() as u8,
            ));
        }

        let mut adapter = RapierRagdollAdapter::new(setup, bindings).unwrap();

        // 1. Verify physics state extraction
        let states = adapter.extract_physics_states(&bodies).unwrap();
        assert_eq!(states.len(), 11);
        // Body 0 translation was Y=1.0, so COM is ~1.0
        assert!((states[0].world_center_of_mass.y - 1.0).abs() < 1e-4);

        // 2. Step adapter with master at Y=0.0
        let master_xfs = vec![Transform3d::IDENTITY; 11];
        let master_rots = vec![Quat::IDENTITY; 11];

        adapter
            .step(&master_xfs, &master_rots, &mut bodies, Some(&mut joints), 0.02)
            .expect("Adapter step should succeed");

        // Hips (root) should have received a downward velocity change to correct Y=1.0 -> Y=0.0
        let hips_body = bodies.get(adapter.bindings[0].body).unwrap();
        assert!(hips_body.linvel().y < 0.0);
    }
}
