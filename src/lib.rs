//! # active-ragdoll-rs
//!
//! A high-performance, idiomatically engineered Rust port of
//! [`Unity.Humanoid.ActiveRagdoll`](https://github.com/ashleve/ActiveRagdoll).
//!
//! Active ragdolls blend kinematic animations with real-time physical forces, allowing
//! characters to naturally respond to collisions, stumble, recover balance, and dynamically
//! collapse when knocked out.
//!
//! ## Architecture Overview
//!
//! - **[`pd`]**: Proportional-Derivative (PD) controllers for 3D linear COM tracking and angular orientations.
//! - **[`controller`]**: State machine managing muscle strength degradation upon obstacle contact,
//!   gradual recovery, death/knockout, and revival ([`SlaveController`], [`RagdollState`]).
//! - **[`follower`]**: Physics-based animation following that calculates Center-of-Mass (COM) PD forces
//!   and joint target drives ([`AnimationFollower`], [`ActiveLimb`]).
//! - **[`collision`]**: Layer-mask filtered contact detection ([`CollisionDetector`], [`LayerMask`]).
//! - **[`humanoid`]**: High-level humanoid ragdoll coordinator and builder presets ([`HumanoidSetUp`], [`HumanoidActiveRagdollBuilder`]).
//! - **[`master`]**: 3rd-person kinematic locomotion controller with push-back constraint and procedural animation poses ([`MasterController`]).
//! - **[`projectile`]**: Ballistic projectile launcher for testing active balance and knockdowns ([`Projectile`]).
//! - **[`camera`]**: 3D fly / orbit observation camera ([`FlyCamera`]).
//! - **[`rapier`]** *(optional `rapier3d` feature)*: Seamless synchronization with Rapier3D rigid bodies and joints.
//!
//! ## Quick Start
//!
//! ```rust
//! use active_ragdoll_rs::prelude::*;
//! use glam::{Quat, Vec3};
//!
//! // 1. Build a standard 11-bone humanoid active ragdoll
//! let mut setup = HumanoidActiveRagdollBuilder::standard_humanoid_11()
//!     .build()
//!     .expect("Failed to build humanoid ragdoll");
//!
//! // 2. Create master (animated) transforms and slave (physics) states
//! let limb_count = setup.limb_count();
//! let master_world = vec![Transform3d::IDENTITY; limb_count];
//! let master_local = vec![Quat::IDENTITY; limb_count];
//! let slave_states = vec![
//!     SlaveLimbPhysicsState {
//!         world_center_of_mass: Vec3::ZERO,
//!         world_rotation: Quat::IDENTITY,
//!         local_rotation: Quat::IDENTITY,
//!         angular_velocity: Vec3::ZERO,
//!     };
//!     limb_count
//! ];
//!
//! // 3. Advance active ragdoll at fixed 50 Hz physics step (dt = 0.02s)
//! let dt = 0.02;
//! let drive_outputs = setup.update(&master_world, &master_local, &slave_states, dt)
//!     .expect("Active ragdoll update failed");
//!
//! assert_eq!(drive_outputs.len(), limb_count);
//! ```

pub mod camera;
pub mod collision;
pub mod controller;
pub mod error;
pub mod follower;
pub mod fps;
pub mod humanoid;
pub mod joint;
pub mod master;
pub mod math;
pub mod pd;
pub mod projectile;
pub mod types;

#[cfg(feature = "rapier3d")]
pub mod rapier;

/// Convenient re-exports of commonly used active ragdoll types.
pub mod prelude {
    pub use crate::camera::FlyCamera;
    pub use crate::collision::{CollisionDetector, LayerMask};
    pub use crate::controller::{RagdollState, SlaveController, SlaveControllerConfig};
    pub use crate::error::{ActiveRagdollError, Result};
    pub use crate::follower::{
        ActiveLimb, AnimationFollower, AnimationFollowerConfig, LimbDriveOutput,
        SlaveLimbPhysicsState,
    };
    pub use crate::fps::FpsDisplay;
    pub use crate::humanoid::{
        HumanoidActiveRagdollBuilder, HumanoidBoneDef, HumanoidSetUp, JointLimits, LimbCollider,
        StandardHumanoidRig,
    };
    pub use crate::joint::{
        ConfigurableJointDescriptor, JointFollowAnimRot, JointMotionMode, RotationDriveMode,
    };
    pub use crate::master::{
        CharacterState, LocomotionInput, MasterController, MasterControllerConfig,
        MasterSkeletonPose,
    };
    pub use crate::math::{clamp_magnitude, delta_angle_deg, smooth_damp, smooth_damp_angle, smooth_step};
    pub use crate::pd::{PdController1d, PdController3d};
    pub use crate::projectile::Projectile;
    pub use crate::types::{HumanoidBone, JointSpace, LimbProfile, Transform3d};

    #[cfg(feature = "rapier3d")]
    pub use crate::rapier::{RapierLimbBinding, RapierRagdollAdapter};
}
