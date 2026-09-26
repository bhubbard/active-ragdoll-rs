use active_ragdoll_rs::prelude::*;
use glam::Vec3;

fn main() -> Result<()> {
    println!("============================================================");
    println!("     active-ragdoll-rs: Headless Simulation Demo            ");
    println!("============================================================");

    // 1. Build standard 11-bone humanoid active ragdoll setup
    let mut ragdoll = HumanoidActiveRagdollBuilder::standard_humanoid_11()
        .with_controller_config(SlaveControllerConfig {
            loose_strength_lerp: 2.0, // lose strength faster on impact
            gain_strength_lerp: 0.2,  // recover gradually
            dead_time: 2.0,           // 2 seconds knockout duration
            ..Default::default()
        })
        .build()?;

    let mut master = MasterController::new(
        MasterControllerConfig::default(),
        Vec3::new(0.0, 0.95, 0.0), // ~0.95m hip height
        0.0,
    );

    let limb_count = ragdoll.limb_count();
    println!("Initialized humanoid active ragdoll with {} limbs.", limb_count);

    // 2. Initialize slave physics state (standing at rest)
    let initial_poses = master.generate_master_poses();
    let mut slave_states: Vec<SlaveLimbPhysicsState> = initial_poses
        .world_transforms
        .iter()
        .zip(initial_poses.local_rotations.iter())
        .map(|(xf, local_rot)| SlaveLimbPhysicsState {
            world_center_of_mass: xf.position,
            world_rotation: xf.rotation,
            local_rotation: *local_rot,
            angular_velocity: Vec3::ZERO,
        })
        .collect();

    // Slave velocities for simple numerical integration
    let mut slave_velocities = vec![Vec3::ZERO; limb_count];

    // Locomotion input: Walk forward
    let input = LocomotionInput {
        horizontal: 0.0,
        vertical: 1.0,
        run: false,
    };

    let dt = 0.02; // 50 Hz physics step
    let mut time = 0.0;
    let total_time = 7.0; // 7 seconds simulation

    let mut projectile: Option<Projectile> = None;
    let mut projectile_fired = false;
    let mut knockout_triggered = false;

    println!("\nStarting simulation loop (dt = {}s, total = {}s)...", dt, total_time);
    println!("--------------------------------------------------------------------------------------");
    println!("Time (s) | Master Z | Slave Z  | State                | Strength | Force Coeff | Event");
    println!("--------------------------------------------------------------------------------------");

    while time < total_time {
        let mut event_str = "";

        // Event 1: Fire projectile at t = 2.0s
        if time >= 2.0 && !projectile_fired {
            projectile_fired = true;
            // Shoot projectile directly at ragdoll's chest (Spine)
            let chest_pos = slave_states[HumanoidBone::Spine.index()].world_center_of_mass;
            let spawn_origin = chest_pos + Vec3::new(0.0, 0.1, 4.0);
            let dir = (chest_pos - spawn_origin).normalize();

            projectile = Some(Projectile::shoot(spawn_origin, dir, 18.0, 0.15, 6.0));
            event_str = "BALL FIRED!";
        }

        // Event 2: Knockout trigger at t = 4.5s
        if time >= 4.5 && !knockout_triggered {
            knockout_triggered = true;
            ragdoll.die();
            event_str = "KNOCKOUT (DIE)!";
        }

        // 1. Step master locomotion controller
        let slave_hips_pos = slave_states[0].world_center_of_mass;
        master.step(&input, 0.0, slave_hips_pos, true, dt);
        let master_poses = master.generate_master_poses();

        // 2. Check projectile collision with ragdoll chest
        if let Some(proj) = &mut projectile {
            proj.step(9.81, dt);
            if proj.active {
                let chest_idx = HumanoidBone::Spine.index();
                let chest_pos = slave_states[chest_idx].world_center_of_mass;

                if let Some(impulse) = proj.check_sphere_collision(chest_pos, 0.25, 22.0) {
                    // Impact transfer to slave chest
                    slave_velocities[chest_idx] += impulse / 22.0;
                    ragdoll.on_collision_enter(LayerMask::ENVIRONMENT.bits().trailing_zeros() as u8);
                    event_str = "IMPACT! LOOSING STRENGTH";
                }
            } else if projectile_fired && time < 2.8 && ragdoll.controller.current_number_of_collisions > 0 {
                // Clear collision after impact
                ragdoll.on_collision_exit(LayerMask::ENVIRONMENT.bits().trailing_zeros() as u8);
                event_str = "CONTACT ENDED (RECOVERING)";
            }
        }

        // 3. Step active ragdoll PD follower and state machine
        let drive_outputs = ragdoll.update(
            &master_poses.world_transforms,
            &master_poses.local_rotations,
            &slave_states,
            dt,
        )?;

        // 4. Numerical integration for slave bodies
        for i in 0..limb_count {
            // Add PD linear force (velocity change impulse)
            slave_velocities[i] += drive_outputs[i].linear_force * (dt * 5.0);
            // Linear drag
            slave_velocities[i] *= 1.0 - (0.1 * dt);
            // Position update
            slave_states[i].world_center_of_mass += slave_velocities[i] * dt;

            // Angular tracking
            if let Some(target_q) = drive_outputs[i].target_rotation {
                let s = (drive_outputs[i].joint_spring * dt * 0.005).clamp(0.0, 1.0);
                slave_states[i].local_rotation = slave_states[i].local_rotation.slerp(target_q, s);
            }
        }

        // Print telemetry every 0.5s or on major events
        let step_int = (time / dt).round() as i32;
        if step_int % 25 == 0 || !event_str.is_empty() {
            println!(
                "{:7.2} | {:8.3} | {:8.3} | {:<20} | {:8.2} | {:11.2} | {}",
                time,
                master.position.z,
                slave_states[0].world_center_of_mass.z,
                format!("{:?}", ragdoll.state()),
                ragdoll.current_strength(),
                ragdoll.controller.force_coefficient,
                event_str
            );
        }

        time += dt;
    }

    println!("--------------------------------------------------------------------------------------");
    println!("Simulation finished successfully!");
    println!("Final Ragdoll State: {:?}", ragdoll.state());
    println!("Final Muscle Strength: {:.2}", ragdoll.current_strength());
    println!("Final Distance between Master and Slave: {:.3}m", (master.position - slave_states[0].world_center_of_mass).length());
    println!("============================================================");

    Ok(())
}
