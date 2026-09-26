use glam::{Quat, Vec3};

use crate::error::{ActiveRagdollError, Result};
use crate::pd::PdController3d;
use crate::types::{HumanoidBone, JointSpace, LimbProfile, Transform3d};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Global tuning configuration for physics-based animation following.
///
/// Direct port of Unity's `AnimationFollowing.cs` inspector parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AnimationFollowerConfig {
    /// Proportional gain of the linear PD controller (PForce).
    ///
    /// Default: `8.0`. Range: [0.0, 160.0].
    pub p_force: f32,

    /// Derivative gain of the linear PD controller (DForce).
    ///
    /// Default: `0.01`. Range: [0.0, 0.064].
    pub d_force: f32,

    /// Maximum linear force clamp magnitude (velocity change impulse).
    ///
    /// Default: `10.0`. Range: [0.0, 100.0].
    pub max_force: f32,

    /// Maximum joint rotational spring stiffness (torque limit).
    ///
    /// Default: `2000.0`. Range: [0.0, 10000.0].
    pub max_joint_torque: f32,

    /// Joint rotational velocity damper.
    ///
    /// Default: `0.6`. Range: [0.0, 10.0].
    pub joint_damping: f32,

    /// Rigidbody linear drag / damping.
    ///
    /// Default: `0.1`.
    pub linear_drag: f32,

    /// Rigidbody angular drag / damping.
    ///
    /// Default: `0.0`.
    pub angular_drag: f32,

    /// Maximum angular velocity cap for rigid bodies.
    ///
    /// Default: `1000.0`.
    pub max_angular_velocity: f32,

    /// Whether gravity should be applied to ragdoll bodies.
    ///
    /// Default: `true`.
    pub use_gravity: bool,
}

impl Default for AnimationFollowerConfig {
    fn default() -> Self {
        Self {
            p_force: 8.0,
            d_force: 0.01,
            max_force: 10.0,
            max_joint_torque: 2000.0,
            joint_damping: 0.6,
            linear_drag: 0.1,
            angular_drag: 0.0,
            max_angular_velocity: 1000.0,
            use_gravity: true,
        }
    }
}

/// Represents an active ragdoll limb/bone in the tracking hierarchy.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ActiveLimb {
    /// Human-readable limb name (e.g. "Hips", "LeftArm").
    pub name: String,

    /// Optional standard humanoid bone identifier.
    pub bone: Option<HumanoidBone>,

    /// Offset from limb position to Center of Mass (COM) in limb local coordinate space:
    /// `q.inverse() * (world_com - position)`
    pub pos_to_com: Vec3,

    /// Joint coordinate space transform for orientation drives.
    /// None for root (hips); Some for all child limbs with a joint.
    pub joint_space: Option<JointSpace>,

    /// PD controller tracking linear COM error.
    pub pd: PdController3d,

    /// Individual per-limb profile overrides.
    pub profile: LimbProfile,
}

impl ActiveLimb {
    /// Construct a new active limb.
    pub fn new(
        name: impl Into<String>,
        bone: Option<HumanoidBone>,
        pos_to_com: Vec3,
        joint_space: Option<JointSpace>,
        p_force: f32,
        d_force: f32,
    ) -> Self {
        Self {
            name: name.into(),
            bone,
            pos_to_com,
            joint_space,
            pd: PdController3d::new(p_force, d_force),
            profile: LimbProfile::default(),
        }
    }

    /// Whether this limb is the root (hips).
    #[inline]
    pub fn is_root(&self) -> bool {
        self.joint_space.is_none()
    }
}

/// Current physics state of a slave ragdoll limb provided by the physics engine.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlaveLimbPhysicsState {
    /// Current world center-of-mass position of the rigid body.
    pub world_center_of_mass: Vec3,
    /// Current orientation of the rigid body in world space.
    pub world_rotation: Quat,
    /// Current local rotation of the limb relative to its parent bone.
    pub local_rotation: Quat,
    /// Current angular velocity in radians per second (optional, used for angular torque drives).
    pub angular_velocity: Vec3,
}

/// Computed forces and joint target parameters for a single limb at a physics step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LimbDriveOutput {
    /// Linear force / velocity change impulse to apply to the limb's center of mass.
    pub linear_force: Vec3,

    /// Joint target orientation (quaternion in joint drive space).
    /// Used directly by configurable joints or spherical joint motors.
    pub target_rotation: Option<Quat>,

    /// Effective joint spring stiffness ($k_p$). Zero when character is dead.
    pub joint_spring: f32,

    /// Effective joint damper ($k_d$). Zero when character is dead.
    pub joint_damper: f32,

    /// Direct angular torque vector in world space (for engines without native joint drives).
    pub angular_torque: Vec3,
}

/// Computes PD forces and joint drives for all limbs to track master animation.
///
/// Direct port and optimization of Unity's `AnimationFollowing.cs`.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AnimationFollower {
    /// Configuration parameters.
    pub config: AnimationFollowerConfig,

    /// Registered ragdoll limbs in hierarchy order.
    pub limbs: Vec<ActiveLimb>,
}

impl AnimationFollower {
    /// Create a new follower with configuration and limbs list.
    pub fn new(config: AnimationFollowerConfig, limbs: Vec<ActiveLimb>) -> Self {
        Self { config, limbs }
    }

    /// Reset all limb PD controllers and error histories.
    pub fn reset(&mut self) {
        for limb in &mut self.limbs {
            limb.pd.reset();
        }
    }

    /// Calculate active ragdoll drive outputs for all limbs.
    ///
    /// - `master_world_transforms`: World-space transforms of master animated bones (length must match limbs).
    /// - `master_local_rotations`: Local rotations of master animated bones (length must match limbs).
    /// - `slave_states`: Current physical state of slave ragdoll rigid bodies (length must match limbs).
    /// - `force_coeff`: Force multiplier from [`SlaveController`].
    /// - `torque_coeff`: Torque multiplier from [`SlaveController`].
    /// - `is_alive`: Life status. When false, joint torques are zeroed out.
    /// - `dt`: Physics fixed delta time in seconds.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        master_world_transforms: &[Transform3d],
        master_local_rotations: &[Quat],
        slave_states: &[SlaveLimbPhysicsState],
        force_coeff: f32,
        torque_coeff: f32,
        is_alive: bool,
        dt: f32,
    ) -> Result<Vec<LimbDriveOutput>> {
        let count = self.limbs.len();
        if master_world_transforms.len() != count {
            return Err(ActiveRagdollError::HierarchyMismatch {
                master_count: master_world_transforms.len(),
                slave_count: count,
            });
        }
        if slave_states.len() != count {
            return Err(ActiveRagdollError::HierarchyMismatch {
                master_count: count,
                slave_count: slave_states.len(),
            });
        }

        let mut outputs = Vec::with_capacity(count);

        for i in 0..count {
            let limb = &mut self.limbs[i];
            let master_xf = &master_world_transforms[i];
            let slave = &slave_states[i];

            if !limb.profile.enabled {
                outputs.push(LimbDriveOutput {
                    linear_force: Vec3::ZERO,
                    target_rotation: None,
                    joint_spring: 0.0,
                    joint_damper: 0.0,
                    angular_torque: Vec3::ZERO,
                });
                continue;
            }

            // 1. APPLY FORCE (World Center of Mass tracking via PD Controller)
            // masterRigidTransformsWCOM = master.position + master.rotation * rigidbodiesPosToCOM[i]
            let master_wcom = master_xf.position + master_xf.rotation * limb.pos_to_com;
            let force_error = master_wcom - slave.world_center_of_mass;

            // Make sure gains match current config
            limb.pd.p_gain = self.config.p_force;
            limb.pd.d_gain = self.config.d_force;

            let force_signal = limb.pd.calculate(force_error, dt);
            let effective_max_force = self.config.max_force
                * limb.profile.max_force_scale
                * force_coeff;
            let clamped_force = PdController3d::clamp_magnitude(force_signal, effective_max_force);

            // 2. APPLY ROTATION (Joint Space Orientation and Drives)
            let (target_rot, spring, damper, angular_torque) = if i == 0 {
                // Root limb (hips) has no joint
                (None, 0.0, 0.0, Vec3::ZERO)
            } else if let Some(js) = &limb.joint_space {
                let master_local_rot = master_local_rotations[i];

                if !is_alive {
                    // Limp ragdoll: zero spring and damper
                    (Some(Quat::IDENTITY), 0.0, 0.0, Vec3::ZERO)
                } else {
                    let target_q = js.compute_target_rotation(master_local_rot);
                    let spring_val = self.config.max_joint_torque
                        * limb.profile.max_joint_torque_scale
                        * torque_coeff;
                    let damper_val = self.config.joint_damping * limb.profile.joint_damping_scale;

                    // Direct torque computation (in case physics backend needs torque directly)
                    let torque = compute_quaternion_pd_torque(
                        slave.local_rotation,
                        target_q,
                        slave.angular_velocity,
                        spring_val,
                        damper_val,
                    );

                    (Some(target_q), spring_val, damper_val, torque)
                }
            } else {
                (None, 0.0, 0.0, Vec3::ZERO)
            };

            outputs.push(LimbDriveOutput {
                linear_force: clamped_force,
                target_rotation: target_rot,
                joint_spring: spring,
                joint_damper: damper,
                angular_torque,
            });
        }

        Ok(outputs)
    }
}

/// Computes angular PD torque in quaternion orientation space.
///
/// Formula:
/// $\tau = 2 \cdot k_p \cdot \text{xyz}(q_{\text{target}} \cdot q_{\text{current}}^{-1}) - k_d \cdot \omega$
#[inline]
pub fn compute_quaternion_pd_torque(
    current_rot: Quat,
    target_rot: Quat,
    current_angular_vel: Vec3,
    spring: f32,
    damper: f32,
) -> Vec3 {
    let mut q_diff = target_rot * current_rot.inverse();
    // Ensure shortest path around 4D sphere
    if q_diff.w < 0.0 {
        q_diff = -q_diff;
    }

    let rot_error = Vec3::new(q_diff.x, q_diff.y, q_diff.z);
    2.0 * spring * rot_error - damper * current_angular_vel
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_animation_follower_step() {
        let config = AnimationFollowerConfig::default();
        let limbs = vec![
            ActiveLimb::new("Hips", Some(HumanoidBone::Hips), Vec3::ZERO, None, 8.0, 0.01),
            ActiveLimb::new(
                "Spine",
                Some(HumanoidBone::Spine),
                Vec3::new(0.0, 0.2, 0.0),
                Some(JointSpace::from_axes(Vec3::X, Vec3::Y, Quat::IDENTITY)),
                8.0,
                0.01,
            ),
        ];

        let mut follower = AnimationFollower::new(config, limbs);

        let master_xf = vec![
            Transform3d::from_translation(Vec3::new(0.0, 1.0, 0.0)),
            Transform3d::from_translation(Vec3::new(0.0, 1.4, 0.0)),
        ];
        let master_local = vec![Quat::IDENTITY, Quat::IDENTITY];
        let slave_states = vec![
            SlaveLimbPhysicsState {
                world_center_of_mass: Vec3::new(0.0, 0.9, 0.0), // 0.1 below master
                world_rotation: Quat::IDENTITY,
                local_rotation: Quat::IDENTITY,
                angular_velocity: Vec3::ZERO,
            },
            SlaveLimbPhysicsState {
                world_center_of_mass: Vec3::new(0.0, 1.3, 0.0),
                world_rotation: Quat::IDENTITY,
                local_rotation: Quat::IDENTITY,
                angular_velocity: Vec3::ZERO,
            },
        ];

        let outputs = follower
            .step(&master_xf, &master_local, &slave_states, 1.0, 1.0, true, 0.02)
            .unwrap();

        assert_eq!(outputs.len(), 2);
        // Hips should have upward force to correct 0.1 offset
        assert!(outputs[0].linear_force.y > 0.0);
        assert_eq!(outputs[0].target_rotation, None); // root has no joint target

        // Spine should have joint drive and spring > 0
        assert!(outputs[1].joint_spring > 0.0);
        assert!(outputs[1].target_rotation.is_some());
    }

    #[test]
    fn test_animation_follower_dead_zeros_torque() {
        let config = AnimationFollowerConfig::default();
        let limbs = vec![
            ActiveLimb::new("Hips", Some(HumanoidBone::Hips), Vec3::ZERO, None, 8.0, 0.01),
            ActiveLimb::new(
                "Spine",
                Some(HumanoidBone::Spine),
                Vec3::ZERO,
                Some(JointSpace::from_axes(Vec3::X, Vec3::Y, Quat::IDENTITY)),
                8.0,
                0.01,
            ),
        ];

        let mut follower = AnimationFollower::new(config, limbs);
        let master_xf = vec![Transform3d::IDENTITY, Transform3d::IDENTITY];
        let master_local = vec![Quat::IDENTITY, Quat::IDENTITY];
        let slave_states = vec![
            SlaveLimbPhysicsState {
                world_center_of_mass: Vec3::ZERO,
                world_rotation: Quat::IDENTITY,
                local_rotation: Quat::IDENTITY,
                angular_velocity: Vec3::ZERO,
            },
            SlaveLimbPhysicsState {
                world_center_of_mass: Vec3::ZERO,
                world_rotation: Quat::IDENTITY,
                local_rotation: Quat::IDENTITY,
                angular_velocity: Vec3::ZERO,
            },
        ];

        // Character is dead (`is_alive = false`)
        let outputs = follower
            .step(&master_xf, &master_local, &slave_states, 0.0, 0.0, false, 0.02)
            .unwrap();

        assert_eq!(outputs[1].joint_spring, 0.0);
        assert_eq!(outputs[1].joint_damper, 0.0);
    }
}
