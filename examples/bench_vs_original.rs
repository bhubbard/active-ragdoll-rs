use std::time::Instant;
use active_ragdoll_rs::prelude::*;
use glam::Vec3;

fn main() -> Result<()> {
    println!("============================================================");
    println!("  active-ragdoll-rs (Rust) vs Unity ActiveRagdoll (C#) Bench");
    println!("============================================================");

    // 1. Single 11-Bone Humanoid Active Ragdoll Step Latency
    println!("\n--- 1. Single Humanoid Active Ragdoll (11 Bones, PD Follower, State Machine) ---");
    {
        let mut ragdoll = HumanoidActiveRagdollBuilder::standard_humanoid_11()
            .with_controller_config(SlaveControllerConfig::default())
            .build()?;

        let mut master = MasterController::new(
            MasterControllerConfig::default(),
            Vec3::new(0.0, 0.95, 0.0),
            0.0,
        );

        let initial_poses = master.generate_master_poses();
        let slave_states: Vec<SlaveLimbPhysicsState> = initial_poses
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

        let input = LocomotionInput {
            horizontal: 0.0,
            vertical: 1.0,
            run: false,
        };

        let dt = 0.02; // 50 Hz

        // Warm up
        for _ in 0..10_000 {
            master.step(&input, 0.0, slave_states[0].world_center_of_mass, true, dt);
            let poses = master.generate_master_poses();
            let _ = ragdoll.update(&poses.world_transforms, &poses.local_rotations, &slave_states, dt)?;
        }

        let iterations = 500_000;
        let start = Instant::now();
        for _ in 0..iterations {
            master.step(&input, 0.0, slave_states[0].world_center_of_mass, true, dt);
            let poses = master.generate_master_poses();
            let _ = ragdoll.update(&poses.world_transforms, &poses.local_rotations, &slave_states, dt)?;
        }
        let elapsed = start.elapsed();
        let us_per_ragdoll = elapsed.as_micros() as f64 / iterations as f64;
        let ragdolls_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "Iterations: {} | Total Time: {:.2?} | Latency: {:.2} µs/ragdoll | {:>10.0} ragdoll-ticks/s",
            iterations, elapsed, us_per_ragdoll, ragdolls_per_sec
        );
    }

    // 2. Multi-Ragdoll Crowd Scaling (50, 100, 250 Concurrent Humanoids)
    println!("\n--- 2. Multi-Ragdoll Crowd Scaling (Concurrent Humanoids at 50/60 Hz) ---");
    for &crowd_size in &[50, 100, 250] {
        let mut crowd: Vec<(HumanoidSetUp, MasterController, Vec<SlaveLimbPhysicsState>)> = (0..crowd_size)
            .map(|i| {
                let ragdoll = HumanoidActiveRagdollBuilder::standard_humanoid_11().build().unwrap();
                let master = MasterController::new(
                    MasterControllerConfig::default(),
                    Vec3::new(i as f32 * 1.5, 0.95, 0.0),
                    0.0,
                );
                let initial = master.generate_master_poses();
                let slave_states = initial
                    .world_transforms
                    .iter()
                    .zip(initial.local_rotations.iter())
                    .map(|(xf, local_rot)| SlaveLimbPhysicsState {
                        world_center_of_mass: xf.position,
                        world_rotation: xf.rotation,
                        local_rotation: *local_rot,
                        angular_velocity: Vec3::ZERO,
                    })
                    .collect();
                (ragdoll, master, slave_states)
            })
            .collect();

        let input = LocomotionInput { horizontal: 0.0, vertical: 1.0, run: false };
        let dt = 0.02;
        let frames = 500;
        let start = Instant::now();

        for _ in 0..frames {
            for (ragdoll, master, slave_states) in &mut crowd {
                master.step(&input, 0.0, slave_states[0].world_center_of_mass, true, dt);
                let poses = master.generate_master_poses();
                let _ = ragdoll.update(&poses.world_transforms, &poses.local_rotations, slave_states, dt);
            }
        }

        let elapsed = start.elapsed();
        let frame_latency = elapsed / frames as u32;
        let fps_capacity = frames as f64 / elapsed.as_secs_f64();
        let percent_60fps = (frame_latency.as_secs_f64() / 0.016666) * 100.0;

        println!(
            "Crowd Size: {:>4} ragdolls ({:>5} bones) | Frame: {:>8.2?} | {:>8.0} FPS max | {:>5.2}% of 16.6ms budget",
            crowd_size, crowd_size * 11, frame_latency, fps_capacity, percent_60fps
        );
    }

    // 3. PD Follower Joint Torque Micro-Benchmark
    println!("\n--- 3. PD Angular & Linear Follower Joint Solver Micro-Benchmark ---");
    {
        use active_ragdoll_rs::pd::PdController3d;

        let mut pd = PdController3d::new(1500.0, 0.05);

        let iterations = 10_000_000;
        let start = Instant::now();
        let mut force_sum = 0.0;

        for i in 0..iterations {
            let error = Vec3::new(0.1 + (i % 10) as f32 * 0.01, 0.05, -0.02);
            let f = pd.calculate(error, 0.02);
            force_sum += f.x;
        }

        let elapsed = start.elapsed();
        let ns_per_calc = elapsed.as_nanos() as f64 / iterations as f64;
        let calcs_per_sec = iterations as f64 / elapsed.as_secs_f64();

        println!(
            "PD Calculations: {} | Time: {:.2?} | Latency: {:.2} ns/calc ({:>10.0} calcs/s) | Sum: {:.1}",
            iterations, elapsed, ns_per_calc, calcs_per_sec, force_sum
        );
    }

    println!("\n============================================================");
    println!("                      Benchmark Complete                    ");
    println!("============================================================");

    Ok(())
}
