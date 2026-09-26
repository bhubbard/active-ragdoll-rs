# active-ragdoll-rs

[![Rust](https://img.shields.io/badge/rust-2024%20edition-orange.svg)](https://www.rust-lang.org)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE)
[![Physics](https://img.shields.io/badge/physics-Rapier3D-red.svg)](https://rapier.rs)

> A high-performance, idiomatically engineered pure Rust port of [ashleve/ActiveRagdoll](https://github.com/ashleve/ActiveRagdoll) (Unity Humanoid Active Ragdoll).

Active ragdolls combine kinematic animations with real-time physical forces. Rather than playing canned ragdoll animations upon death, active ragdolls continuously use **Proportional-Derivative (PD) controllers and joint torque drives** to track animated target poses. When characters collide with obstacles or get struck by projectiles, they stumble, lose muscle strength, stagger, recover balance, or completely collapse when knocked out.

---

## Features

- **Master-Slave Architecture**:
  - **Kinematic Master**: 3rd-person character controller driving animated poses with smooth turning, walking/running/falling gaits, and procedural forward kinematics.
  - **Push-Back Constraint**: Prevents the kinematic master from desynchronizing if the physics ragdoll is caught on geometry, pinned, or falling behind.
  - **Physics Slave**: Ragdoll skeleton with configurable joint drives, spring stiffness, velocity damping, and angular limits.
- **Biomechanical Humanoid Definition**:
  - Full **11-bone standard humanoid rig** (`Hips`, `LeftUpperLeg`, `LeftLowerLeg`, `RightUpperLeg`, `RightLowerLeg`, `Spine`, `LeftUpperArm`, `LeftLowerArm`, `Head`, `RightUpperArm`, `RightLowerArm`).
  - Anatomically realistic mass distributions (~75 kg total), local Center-of-Mass (COM) offsets, joint attachment anchors, and swing/twist limits.
- **Physical Animation Following**:
  - **Linear COM Tracking**: PD controller calculates force impulses directed at target bone Centers of Mass.
  - **Angular Joint Drive**: Computes target orientations in joint coordinate space and applies quaternion-based PD rotational torques.
- **Real-Time Muscle Strength Degradation & Recovery**:
  - Tracks active collisions against environment colliders (with 32-bit layer mask filtering).
  - Muscle strength smoothly attenuates during continuous contact (`loose_strength_lerp`), creating realistic wobbling and stumbling.
  - Strength recovers upon freeing from contact (`gain_strength_lerp`).
  - **Unconsciousness / Knockout**: Complete loss of joint torque (`die()`), followed by limp physics ragdolling and timed recovery (`come_alive()`).
- **Interactive Sandbox & Tooling**:
  - **Ballistic Projectiles** (`Projectile`): High-speed physics balls for testing active balance and knockdown impacts (port of `ShootObject.cs`).
  - **Observation Fly Camera** (`FlyCamera`): 6DOF free-look orbit inspection camera (port of `FlyCamera.cs`).
- **Engine Agnostic + Rapier3D Integration**:
  - Core crate has **zero heavy engine prerequisites** (runs headless, in CI, or with any custom engine using `glam`).
  - Optional `rapier3d` feature provides seamless 1:1 synchronization via `RapierRagdollAdapter`.

---

## Architecture Overview

```
                      +-----------------------------+
                      |      User Input (WASD)      |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      |      MasterController       |
                      |  - Locomotion State Machine |
                      |  - SmoothDamp Turning       |
                      |  - Procedural Kinematics    |
                      +--------------+--------------+
                                     |  Master Bone Poses
                                     v
+-----------------------+     +-----------------------------+
|    SlaveController    |     |      AnimationFollower      |
|  - Strength Atten.    +---->+  - Linear COM PD Forces     |
|  - Collision Filtering|     |  - Angular Joint Drive      |
|  - Knockout / Revival |     |  - Limb Profile Overrides   |
+-----------+-----------+     +--------------+--------------+
            ^                                |  LimbDriveOutput
            | Collisions                     v
+-----------+-----------+     +-----------------------------+
|   CollisionDetector   |     |    Physical Ragdoll Limb    |
|   & LayerMask Filter  |     | (Rapier3D or Custom Physics)|
+-----------------------+     +-----------------------------+
```

---

## Quickstart

Add `active-ragdoll-rs` to your `Cargo.toml`:

```toml
[dependencies]
active-ragdoll-rs = "0.1.0"
glam = "0.33"
```

### Basic Headless Simulation

```rust
use active_ragdoll_rs::prelude::*;
use glam::{Quat, Vec3};

fn main() -> Result<()> {
    // 1. Build a standard 11-bone humanoid active ragdoll
    let mut ragdoll = HumanoidActiveRagdollBuilder::standard_humanoid_11()
        .build()?;

    // 2. Setup kinematic master controller
    let mut master = MasterController::new(
        MasterControllerConfig::default(),
        Vec3::new(0.0, 0.95, 0.0), // hip height
        0.0,
    );

    // 3. Step physics at 50 Hz (dt = 0.02s)
    let input = LocomotionInput { horizontal: 0.0, vertical: 1.0, run: false };
    let dt = 0.02;

    let slave_root_pos = Vec3::new(0.0, 0.95, 0.0);
    master.step(&input, 0.0, slave_root_pos, true, dt);
    let master_poses = master.generate_master_poses();

    let slave_states = vec![
        SlaveLimbPhysicsState {
            world_center_of_mass: Vec3::new(0.0, 0.95, 0.0),
            world_rotation: Quat::IDENTITY,
            local_rotation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
        };
        ragdoll.limb_count()
    ];

    let drive_outputs = ragdoll.update(
        &master_poses.world_transforms,
        &master_poses.local_rotations,
        &slave_states,
        dt,
    )?;

    println!("Computed {} limb drive outputs", drive_outputs.len());
    Ok(())
}
```

---

## Running the Demos

### 1. Headless Walk & Knockdown Simulation

Demonstrates the active ragdoll walking forward, getting struck in the chest by a high-speed ball, stumbling, losing muscle strength, recovering balance, experiencing a knockout, and automatically reviving:

```bash
cargo run --example headless_simulation
```

### 2. Rapier3D Full Physics Simulation

Demonstrates full rigid-body dynamics, spherical joints with limits, ground collision, and active locomotion tracking within the Rapier3D physics engine:

```bash
cargo run --example rapier_ragdoll --features rapier3d
```

---

## Port Mapping (Unity C# vs Rust)

| Unity C# Reference File | Rust Port Implementation | Description |
|:---|:---|:---|
| `AnimationFollowing.cs` | [`follower.rs`](src/follower.rs) | PD linear force calculation and joint drive target computation |
| `MasterController.cs` | [`master.rs`](src/master.rs) | 3rd-person locomotion, smooth turning, push-back movement |
| `SlaveController.cs` | [`controller.rs`](src/controller.rs) | Muscle strength lerp, collision reactions, knockout & recovery |
| `CollisionDetector.cs` | [`collision.rs`](src/collision.rs) | Collision count tracking and 32-bit layer mask filtering |
| `HumanoidSetUp.cs` | [`humanoid.rs`](src/humanoid.rs) | Unified coordinator & builder for the 11-bone humanoid rig |
| `ShootObject.cs` | [`projectile.rs`](src/projectile.rs) | Ballistic projectile shooting & momentum transfer |
| `FlyCamera.cs` | [`camera.rs`](src/camera.rs) | Free-look orbit and fly camera inspection controller |
| *(Unity Physics / ConfigurableJoint)* | [`rapier/`](src/rapier/) | Direct Rapier3D rigid-body and impulse joint adapter |

---

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
