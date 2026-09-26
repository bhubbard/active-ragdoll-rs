//! Rapier3D physics engine integration for active ragdolls.
//!
//! Provides bidirectional synchronization between [`HumanoidSetUp`] and Rapier's
//! rigid bodies, impulse joints, and collision pipelines.

pub mod adapter;

pub use adapter::{RapierLimbBinding, RapierRagdollAdapter};
