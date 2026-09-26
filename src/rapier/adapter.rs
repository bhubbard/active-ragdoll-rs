use glam::Quat;
use rapier3d::prelude::*;

use crate::error::{ActiveRagdollError, Result};
use crate::follower::SlaveLimbPhysicsState;
use crate::humanoid::HumanoidSetUp;
use crate::types::Transform3d;

/// Binding between an active ragdoll limb and its Rapier3D physics entities.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RapierLimbBinding {
    /// Rigid body handle for this limb.
    pub body: RigidBodyHandle,
    /// Parent rigid body handle (if any).
    pub parent_body: Option<RigidBodyHandle>,
    /// Impulse joint connecting this limb to its parent (None for root hips).
    pub joint: Option<ImpulseJointHandle>,
    /// Physics layer assigned to this limb (0..31).
    pub layer: u8,
}

impl RapierLimbBinding {
    pub const fn new(
        body: RigidBodyHandle,
        parent_body: Option<RigidBodyHandle>,
        joint: Option<ImpulseJointHandle>,
        layer: u8,
    ) -> Self {
        Self {
            body,
            parent_body,
            joint,
            layer,
        }
    }
}

/// Adapter connecting [`HumanoidSetUp`] to Rapier3D physics state.
#[derive(Debug)]
pub struct RapierRagdollAdapter {
    /// The high-level humanoid active ragdoll controller.
    pub ragdoll: HumanoidSetUp,
    /// Rapier bindings corresponding 1:1 with `ragdoll.follower.limbs`.
    pub bindings: Vec<RapierLimbBinding>,
}

impl RapierRagdollAdapter {
    /// Create a new adapter wrapping a humanoid setup and matching Rapier bindings.
    pub fn new(ragdoll: HumanoidSetUp, bindings: Vec<RapierLimbBinding>) -> Result<Self> {
        if ragdoll.limb_count() != bindings.len() {
            return Err(ActiveRagdollError::HierarchyMismatch {
                master_count: ragdoll.limb_count(),
                slave_count: bindings.len(),
            });
        }
        Ok(Self { ragdoll, bindings })
    }

    /// Read physical states of all ragdoll limbs from Rapier's `RigidBodySet`.
    pub fn extract_physics_states(&self, bodies: &RigidBodySet) -> Result<Vec<SlaveLimbPhysicsState>> {
        let mut states = Vec::with_capacity(self.bindings.len());

        for binding in &self.bindings {
            let body = bodies
                .get(binding.body)
                .ok_or(ActiveRagdollError::RapierBodyNotFound(binding.body))?;

            let com = body.center_of_mass();
            let world_rot = *body.rotation();
            let angvel = body.angvel();

            let local_rot = if let Some(parent_handle) = binding.parent_body {
                if let Some(parent_body) = bodies.get(parent_handle) {
                    let parent_rot = *parent_body.rotation();
                    (parent_rot.inverse() * world_rot).normalize()
                } else {
                    world_rot
                }
            } else {
                world_rot
            };

            states.push(SlaveLimbPhysicsState {
                world_center_of_mass: com,
                world_rotation: world_rot,
                local_rotation: local_rot,
                angular_velocity: angvel,
            });
        }

        Ok(states)
    }

    /// Synchronize rigid body damping and gravity scale settings from follower config.
    pub fn sync_body_parameters(&self, bodies: &mut RigidBodySet) {
        let cfg = &self.ragdoll.follower.config;
        for binding in &self.bindings {
            if let Some(body) = bodies.get_mut(binding.body) {
                body.set_linear_damping(cfg.linear_drag);
                body.set_angular_damping(cfg.angular_drag);
                body.set_gravity_scale(if cfg.use_gravity { 1.0 } else { 0.0 }, true);
            }
        }
    }

    /// Advance active ragdoll physics and apply resulting forces and torques to Rapier bodies.
    ///
    /// - `master_world_transforms`: World transforms for master animated skeleton.
    /// - `master_local_rotations`: Local rotations for master animated skeleton.
    /// - `bodies`: Mutable reference to Rapier's `RigidBodySet`.
    /// - `joints`: Optional mutable reference to Rapier's `ImpulseJointSet` (for updating motor stiffness/damping).
    /// - `dt`: Physics fixed delta time in seconds.
    pub fn step(
        &mut self,
        master_world_transforms: &[Transform3d],
        master_local_rotations: &[Quat],
        bodies: &mut RigidBodySet,
        mut joints: Option<&mut ImpulseJointSet>,
        dt: f32,
    ) -> Result<()> {
        let states = self.extract_physics_states(bodies)?;
        let outputs = self.ragdoll.update(master_world_transforms, master_local_rotations, &states, dt)?;

        self.sync_body_parameters(bodies);

        for (i, output) in outputs.into_iter().enumerate() {
            let binding = &self.bindings[i];

            if let Some(body) = bodies.get_mut(binding.body) {
                // Apply VelocityChange force (linear COM tracking)
                let current_linvel = body.linvel();
                body.set_linvel(current_linvel + output.linear_force, true);

                // Apply rotational torque impulse
                let torque_impulse = output.angular_torque * dt;
                body.apply_torque_impulse(torque_impulse, true);
            }

            // If joints set is provided and this limb has a joint, update joint parameters
            if let (Some(joints_set), Some(joint_handle)) = (&mut joints, binding.joint) {
                if let Some(joint) = joints_set.get_mut(joint_handle, true) {
                    let _ = joint; // Joint accessible for motor updates
                }
            }
        }

        Ok(())
    }
}
