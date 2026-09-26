use active_ragdoll_rs::prelude::*;
use glam::Vec3;
use std::env;
use std::time::Instant;

fn print_usage() {
    println!("active-ragdoll-rs: High-performance physical animation and active ragdoll controller");
    println!("Version: {}\n", env!("CARGO_PKG_VERSION"));
    println!("USAGE:");
    println!("    active-ragdoll [COMMAND]\n");
    println!("COMMANDS:");
    println!("    info         Display standard humanoid 11-bone hierarchy and physical parameters");
    println!("    simulate     Run headless active ragdoll simulation loop with projectile impact");
    println!("    benchmark    Benchmark PD controller and joint target calculation performance");
    println!("    help         Print this help message");
}

fn cmd_info() -> Result<()> {
    let ragdoll = HumanoidActiveRagdollBuilder::standard_humanoid_11().build()?;
    println!("=== Humanoid Active Ragdoll Configuration ===");
    println!("Limb Count: {}", ragdoll.limb_count());
    println!("Initial State: {:?}", ragdoll.state());
    println!("Initial Strength: {:.2}", ragdoll.current_strength());
    println!("\nBones Hierarchy:");
    for bone in &HumanoidBone::ALL_11 {
        println!("  - Bone [{:2}] {:<14} (Root: {})", bone.index(), bone.name(), bone.is_root());
    }
    println!("\nFollower Parameters:");
    println!("  - Max Joint Torque:  {:.1}", ragdoll.follower.config.max_joint_torque);
    println!("  - Joint Damping:     {:.2}", ragdoll.follower.config.joint_damping);
    println!("  - Linear Drag:       {:.2}", ragdoll.follower.config.linear_drag);
    println!("  - Angular Drag:      {:.2}", ragdoll.follower.config.angular_drag);
    println!("  - Max Ang. Velocity: {:.1}", ragdoll.follower.config.max_angular_velocity);
    println!("  - Use Gravity:       {}", ragdoll.follower.config.use_gravity);
    Ok(())
}

fn cmd_simulate() -> Result<()> {
    println!("Running headless humanoid active ragdoll simulation...");
    let mut ragdoll = HumanoidActiveRagdollBuilder::standard_humanoid_11()
        .with_controller_config(SlaveControllerConfig {
            loose_strength_lerp: 2.0,
            gain_strength_lerp: 0.2,
            dead_time: 2.0,
            ..Default::default()
        })
        .build()?;

    let mut master = MasterController::new(
        MasterControllerConfig::default(),
        Vec3::new(0.0, 0.95, 0.0),
        0.0,
    );

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

    let dt = 0.02;
    let total_time = 3.0;
    let mut time = 0.0;
    let input = LocomotionInput {
        horizontal: 0.0,
        vertical: 1.0,
        run: false,
    };

    println!("-------------------------------------------------------------------------");
    println!(" Time(s) | State                | Strength | Master Z  | Slave Z   | Status");
    println!("-------------------------------------------------------------------------");

    while time < total_time {
        let slave_hips = slave_states[0].world_center_of_mass;
        master.step(&input, 0.0, slave_hips, true, dt);
        let master_poses = master.generate_master_poses();

        let _outputs = ragdoll.update(
            &master_poses.world_transforms,
            &master_poses.local_rotations,
            &slave_states,
            dt,
        )?;

        // Integrate slave simple kinematic position for demo
        for s in &mut slave_states {
            s.world_center_of_mass += Vec3::new(0.0, 0.0, 1.2 * dt);
        }

        let step_int = (time / dt).round() as i32;
        if step_int % 25 == 0 {
            println!(
                "{:7.2} | {:<20} | {:8.2} | {:9.3} | {:9.3} | OK",
                time,
                format!("{:?}", ragdoll.state()),
                ragdoll.current_strength(),
                master.position.z,
                slave_states[0].world_center_of_mass.z
            );
        }
        time += dt;
    }
    println!("-------------------------------------------------------------------------");
    println!("Simulation finished successfully!");
    Ok(())
}

fn cmd_benchmark() -> Result<()> {
    println!("Benchmarking active ragdoll controller PD drive calculation...");
    let mut ragdoll = HumanoidActiveRagdollBuilder::standard_humanoid_11().build()?;
    let master = MasterController::new(MasterControllerConfig::default(), Vec3::new(0.0, 0.95, 0.0), 0.0);
    let poses = master.generate_master_poses();
    let slave_states: Vec<SlaveLimbPhysicsState> = poses
        .world_transforms
        .iter()
        .zip(poses.local_rotations.iter())
        .map(|(xf, local_rot)| SlaveLimbPhysicsState {
            world_center_of_mass: xf.position,
            world_rotation: xf.rotation,
            local_rotation: *local_rot,
            angular_velocity: Vec3::ZERO,
        })
        .collect();

    let iterations = 100_000;
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = ragdoll.update(
            &poses.world_transforms,
            &poses.local_rotations,
            &slave_states,
            0.02,
        )?;
    }
    let elapsed = start.elapsed();
    let ns_per_step = elapsed.as_nanos() as f64 / iterations as f64;
    let steps_per_sec = iterations as f64 / elapsed.as_secs_f64();

    println!("Completed {} steps in {:.3}ms", iterations, elapsed.as_secs_f64() * 1000.0);
    println!("Average time per step: {:.1} ns ({:.2} µs)", ns_per_step, ns_per_step / 1000.0);
    println!("Throughput: {:.0} steps/second", steps_per_sec);
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        print_usage();
        return Ok(());
    }

    match args[1].as_str() {
        "info" => cmd_info(),
        "simulate" => cmd_simulate(),
        "benchmark" => cmd_benchmark(),
        "help" | "--help" | "-h" => {
            print_usage();
            Ok(())
        }
        "version" | "--version" | "-V" => {
            println!("active-ragdoll {}", env!("CARGO_PKG_VERSION"));
            Ok(())
        }
        unknown => {
            eprintln!("Unknown command: {}", unknown);
            print_usage();
            std::process::exit(1);
        }
    }
}
