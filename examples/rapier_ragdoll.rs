use active_ragdoll_rs::prelude::*;
use glam::Vec3;
use rapier3d::prelude::*;

fn main() -> Result<()> {
    println!("============================================================");
    println!("     active-ragdoll-rs: Rapier3D Physics Simulation Demo    ");
    println!("============================================================");

    // 1. Initialize Rapier physics engine structures
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut soft_body_set = SoftBodySet::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd_solver = CCDSolver::new();
    let mut physics_pipeline = PhysicsPipeline::new();
    let integration_parameters = IntegrationParameters {
        dt: 0.02, // 50 Hz physics
        ..Default::default()
    };

    let gravity = Vec3::new(0.0, -9.81, 0.0);

    // 2. Add ground plane collider
    let ground_collider = ColliderBuilder::cuboid(50.0, 0.5, 50.0)
        .translation(Vec3::new(0.0, -0.5, 0.0))
        .friction(0.7)
        .build();
    collider_set.insert(ground_collider);
    println!("Spawned ground collider (50x50m).");

    // 3. Setup standard 11-bone humanoid ragdoll and bindings
    let rig = StandardHumanoidRig::new();
    let setup = HumanoidActiveRagdollBuilder::standard_humanoid_11()
        .build()?;

    let mut master = MasterController::new(
        MasterControllerConfig::default(),
        Vec3::new(0.0, 0.95, 0.0),
        0.0,
    );

    let initial_poses = master.generate_master_poses();
    let mut body_handles = Vec::with_capacity(11);
    let mut bindings = Vec::with_capacity(11);

    for (i, def) in rig.bones.iter().enumerate() {
        let xf = &initial_poses.world_transforms[i];
        let (axis, angle) = xf.rotation.to_axis_angle();

        // Create Rapier dynamic rigid body
        let rb = RigidBodyBuilder::dynamic()
            .translation(xf.position)
            .rotation(axis * angle)
            .linear_damping(0.1)
            .angular_damping(0.0)
            .additional_mass(def.mass)
            .build();
        let rb_handle = rigid_body_set.insert(rb);
        body_handles.push(rb_handle);

        // Create collider
        let col = match def.collider {
            LimbCollider::Capsule { radius, half_height, .. } => {
                ColliderBuilder::capsule_y(half_height, radius)
            }
            LimbCollider::Sphere { radius } => ColliderBuilder::ball(radius),
            LimbCollider::Box { half_extents } => ColliderBuilder::cuboid(
                half_extents.x,
                half_extents.y,
                half_extents.z,
            ),
        }
        .friction(0.5)
        .restitution(0.1)
        .build();
        collider_set.insert_with_parent(col, rb_handle, &mut rigid_body_set);

        // Create impulse joint if not root
        let (parent_handle, joint_handle) = if let Some(parent_idx) = def.parent_index {
            let parent_h = body_handles[parent_idx];

            let joint = SphericalJointBuilder::new()
                .local_anchor1(def.parent_anchor)
                .local_anchor2(def.child_anchor)
                .build();
            let j_handle = impulse_joint_set.insert(parent_h, rb_handle, joint, true);
            (Some(parent_h), Some(j_handle))
        } else {
            (None, None)
        };

        bindings.push(RapierLimbBinding::new(
            rb_handle,
            parent_handle,
            joint_handle,
            LayerMask::RAGDOLL.bits().trailing_zeros() as u8,
        ));
    }

    println!("Spawned 11 Rapier rigid bodies, colliders, and spherical joints.");

    // 4. Wrap with RapierRagdollAdapter
    let mut adapter = RapierRagdollAdapter::new(setup, bindings)?;

    let input = LocomotionInput {
        horizontal: 0.0,
        vertical: 1.0,
        run: false,
    };

    let dt = 0.02;
    let mut time = 0.0;
    let total_time = 4.0;

    println!("\nStepping Rapier3D Active Ragdoll physics for {}s...", total_time);
    println!("----------------------------------------------------------------------");
    println!("Time (s) | Master Z | Ragdoll Hips Z | Ragdoll State        | Strength");
    println!("----------------------------------------------------------------------");

    while time < total_time {
        // Step master animation controller
        let hips_body = rigid_body_set.get(adapter.bindings[0].body).unwrap();
        let hips_pos = hips_body.translation();
        master.step(&input, 0.0, hips_pos, true, dt);
        let master_poses = master.generate_master_poses();

        // Step active ragdoll adapter (applies PD forces & joint torque impulses)
        adapter.step(
            &master_poses.world_transforms,
            &master_poses.local_rotations,
            &mut rigid_body_set,
            Some(&mut impulse_joint_set),
            dt,
        )?;

        // Step Rapier physics world
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
            &mut soft_body_set,
            &mut ccd_solver,
            &(),
            &(),
        );

        let step_int = (time / dt).round() as i32;
        if step_int % 25 == 0 {
            let hips_curr = rigid_body_set.get(adapter.bindings[0].body).unwrap();
            println!(
                "{:7.2} | {:8.3} | {:14.3} | {:<20} | {:8.2}",
                time,
                master.position.z,
                hips_curr.translation().z,
                format!("{:?}", adapter.ragdoll.state()),
                adapter.ragdoll.current_strength()
            );
        }

        time += dt;
    }

    println!("----------------------------------------------------------------------");
    println!("Rapier3D physics simulation completed successfully!");
    println!("============================================================");

    Ok(())
}
