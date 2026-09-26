use thiserror::Error;

/// Errors that can occur during active ragdoll initialization or update.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum ActiveRagdollError {
    #[error("Master transform count ({master_count}) does not equal slave transform count ({slave_count})")]
    HierarchyMismatch {
        master_count: usize,
        slave_count: usize,
    },

    #[error("Limb at index {index} ('{name}') is missing required joint configuration")]
    MissingJoint { index: usize, name: String },

    #[error("Ragdoll root (index 0) must not have a joint attached")]
    RootHasJoint,

    #[error("Invalid delta time: {0} (must be strictly positive and finite)")]
    InvalidDeltaTime(f32),

    #[error("Limb '{0}' not found in active ragdoll hierarchy")]
    LimbNotFound(String),

    #[error("Limb index {0} is out of bounds (ragdoll has {1} limbs)")]
    IndexOutOfBounds(usize, usize),

    #[cfg(feature = "rapier3d")]
    #[error("Rapier rigid body with handle {0:?} not found")]
    RapierBodyNotFound(rapier3d::prelude::RigidBodyHandle),

    #[cfg(feature = "rapier3d")]
    #[error("Rapier joint with handle {0:?} not found")]
    RapierJointNotFound(rapier3d::prelude::ImpulseJointHandle),
}

pub type Result<T> = std::result::Result<T, ActiveRagdollError>;
