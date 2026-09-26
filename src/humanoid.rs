use glam::{Quat, Vec3};

use crate::collision::{CollisionDetector, LayerMask};
use crate::controller::{RagdollState, SlaveController, SlaveControllerConfig};
use crate::error::{ActiveRagdollError, Result};
use crate::follower::{
    ActiveLimb, AnimationFollower, AnimationFollowerConfig, LimbDriveOutput, SlaveLimbPhysicsState,
};
use crate::types::{HumanoidBone, JointSpace, LimbProfile, Transform3d};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Rotational angular limit configuration for a humanoid joint.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointLimits {
    /// Minimum twist angle around joint primary axis in degrees.
    pub low_twist_limit_deg: f32,
    /// Maximum twist angle around joint primary axis in degrees.
    pub high_twist_limit_deg: f32,
    /// Swing limit cone half-angle 1 in degrees.
    pub swing1_limit_deg: f32,
    /// Swing limit cone half-angle 2 in degrees.
    pub swing2_limit_deg: f32,
}

impl Default for JointLimits {
    fn default() -> Self {
        Self {
            low_twist_limit_deg: -30.0,
            high_twist_limit_deg: 30.0,
            swing1_limit_deg: 45.0,
            swing2_limit_deg: 45.0,
        }
    }
}

/// Physical collision geometry descriptor for a humanoid ragdoll limb.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum LimbCollider {
    Capsule {
        radius: f32,
        half_height: f32,
        axis: Vec3,
    },
    Sphere {
        radius: f32,
    },
    Box {
        half_extents: Vec3,
    },
}

/// Complete structural definition of a humanoid limb including hierarchy, physics, and limits.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct HumanoidBoneDef {
    /// Bone identifier.
    pub bone: HumanoidBone,
    /// Name string.
    pub name: &'static str,
    /// Index of parent bone (None for Hips root).
    pub parent_index: Option<usize>,
    /// Local attachment offset in parent space.
    pub parent_anchor: Vec3,
    /// Local attachment offset in this bone's space.
    pub child_anchor: Vec3,
    /// Joint primary drive/twist axis.
    pub joint_axis: Vec3,
    /// Joint secondary swing axis.
    pub secondary_axis: Vec3,
    /// Rotational angle limits.
    pub limits: JointLimits,
    /// Limb mass in kg.
    pub mass: f32,
    /// Local offset from origin to Center of Mass.
    pub pos_to_com: Vec3,
    /// Collision shape.
    pub collider: LimbCollider,
    /// Per-limb profile.
    pub profile: LimbProfile,
}

/// Standard 11-bone humanoid ragdoll template.
///
/// Direct specification matching the Unity `ActiveRagdoll.prefab`.
#[derive(Debug, Clone, PartialEq)]
pub struct StandardHumanoidRig {
    pub bones: [HumanoidBoneDef; 11],
}

impl Default for StandardHumanoidRig {
    fn default() -> Self {
        Self::new()
    }
}

impl StandardHumanoidRig {
    /// Builds the standard 11-bone rig with anatomically realistic masses and joint limits.
    pub fn new() -> Self {
        let bones = [
            // 0: Hips (Root)
            HumanoidBoneDef {
                bone: HumanoidBone::Hips,
                name: "Hips",
                parent_index: None,
                parent_anchor: Vec3::ZERO,
                child_anchor: Vec3::ZERO,
                joint_axis: Vec3::X,
                secondary_axis: Vec3::Y,
                limits: JointLimits::default(),
                mass: 14.0,
                pos_to_com: Vec3::ZERO,
                collider: LimbCollider::Capsule {
                    radius: 0.14,
                    half_height: 0.12,
                    axis: Vec3::X,
                },
                profile: LimbProfile::default(),
            },
            // 1: LeftUpperLeg
            HumanoidBoneDef {
                bone: HumanoidBone::LeftUpperLeg,
                name: "LeftUpperLeg",
                parent_index: Some(0),
                parent_anchor: Vec3::new(-0.12, -0.05, 0.0),
                child_anchor: Vec3::new(0.0, 0.21, 0.0),
                joint_axis: Vec3::X,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -20.0,
                    high_twist_limit_deg: 20.0,
                    swing1_limit_deg: 70.0,
                    swing2_limit_deg: 30.0,
                },
                mass: 8.5,
                pos_to_com: Vec3::new(0.0, -0.1, 0.0),
                collider: LimbCollider::Capsule {
                    radius: 0.08,
                    half_height: 0.2,
                    axis: Vec3::Y,
                },
                profile: LimbProfile::default(),
            },
            // 2: LeftLowerLeg
            HumanoidBoneDef {
                bone: HumanoidBone::LeftLowerLeg,
                name: "LeftLowerLeg",
                parent_index: Some(1),
                parent_anchor: Vec3::new(0.0, -0.21, 0.0),
                child_anchor: Vec3::new(0.0, 0.2, 0.0),
                joint_axis: Vec3::X,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -5.0,
                    high_twist_limit_deg: 5.0,
                    swing1_limit_deg: 130.0,
                    swing2_limit_deg: 5.0,
                },
                mass: 4.5,
                pos_to_com: Vec3::new(0.0, -0.1, 0.0),
                collider: LimbCollider::Capsule {
                    radius: 0.065,
                    half_height: 0.19,
                    axis: Vec3::Y,
                },
                profile: LimbProfile::default(),
            },
            // 3: RightUpperLeg
            HumanoidBoneDef {
                bone: HumanoidBone::RightUpperLeg,
                name: "RightUpperLeg",
                parent_index: Some(0),
                parent_anchor: Vec3::new(0.12, -0.05, 0.0),
                child_anchor: Vec3::new(0.0, 0.21, 0.0),
                joint_axis: Vec3::X,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -20.0,
                    high_twist_limit_deg: 20.0,
                    swing1_limit_deg: 70.0,
                    swing2_limit_deg: 30.0,
                },
                mass: 8.5,
                pos_to_com: Vec3::new(0.0, -0.1, 0.0),
                collider: LimbCollider::Capsule {
                    radius: 0.08,
                    half_height: 0.2,
                    axis: Vec3::Y,
                },
                profile: LimbProfile::default(),
            },
            // 4: RightLowerLeg
            HumanoidBoneDef {
                bone: HumanoidBone::RightLowerLeg,
                name: "RightLowerLeg",
                parent_index: Some(3),
                parent_anchor: Vec3::new(0.0, -0.21, 0.0),
                child_anchor: Vec3::new(0.0, 0.2, 0.0),
                joint_axis: Vec3::X,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -5.0,
                    high_twist_limit_deg: 5.0,
                    swing1_limit_deg: 130.0,
                    swing2_limit_deg: 5.0,
                },
                mass: 4.5,
                pos_to_com: Vec3::new(0.0, -0.1, 0.0),
                collider: LimbCollider::Capsule {
                    radius: 0.065,
                    half_height: 0.19,
                    axis: Vec3::Y,
                },
                profile: LimbProfile::default(),
            },
            // 5: Spine
            HumanoidBoneDef {
                bone: HumanoidBone::Spine,
                name: "Spine1",
                parent_index: Some(0),
                parent_anchor: Vec3::new(0.0, 0.1, 0.0),
                child_anchor: Vec3::new(0.0, -0.12, 0.0),
                joint_axis: Vec3::X,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -25.0,
                    high_twist_limit_deg: 25.0,
                    swing1_limit_deg: 40.0,
                    swing2_limit_deg: 25.0,
                },
                mass: 18.0,
                pos_to_com: Vec3::new(0.0, 0.05, 0.0),
                collider: LimbCollider::Box {
                    half_extents: Vec3::new(0.16, 0.14, 0.11),
                },
                profile: LimbProfile::default(),
            },
            // 6: LeftUpperArm
            HumanoidBoneDef {
                bone: HumanoidBone::LeftUpperArm,
                name: "LeftArm",
                parent_index: Some(5),
                parent_anchor: Vec3::new(-0.22, 0.14, 0.0),
                child_anchor: Vec3::new(0.0, 0.15, 0.0),
                joint_axis: Vec3::Z,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -60.0,
                    high_twist_limit_deg: 60.0,
                    swing1_limit_deg: 85.0,
                    swing2_limit_deg: 85.0,
                },
                mass: 3.5,
                pos_to_com: Vec3::new(0.0, -0.07, 0.0),
                collider: LimbCollider::Capsule {
                    radius: 0.055,
                    half_height: 0.15,
                    axis: Vec3::Y,
                },
                profile: LimbProfile::default(),
            },
            // 7: LeftLowerArm
            HumanoidBoneDef {
                bone: HumanoidBone::LeftLowerArm,
                name: "LeftForeArm",
                parent_index: Some(6),
                parent_anchor: Vec3::new(0.0, -0.15, 0.0),
                child_anchor: Vec3::new(0.0, 0.14, 0.0),
                joint_axis: Vec3::X,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -5.0,
                    high_twist_limit_deg: 5.0,
                    swing1_limit_deg: 135.0,
                    swing2_limit_deg: 5.0,
                },
                mass: 2.0,
                pos_to_com: Vec3::new(0.0, -0.06, 0.0),
                collider: LimbCollider::Capsule {
                    radius: 0.045,
                    half_height: 0.14,
                    axis: Vec3::Y,
                },
                profile: LimbProfile::default(),
            },
            // 8: Head
            HumanoidBoneDef {
                bone: HumanoidBone::Head,
                name: "Head",
                parent_index: Some(5),
                parent_anchor: Vec3::new(0.0, 0.22, 0.0),
                child_anchor: Vec3::new(0.0, -0.1, 0.0),
                joint_axis: Vec3::X,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -40.0,
                    high_twist_limit_deg: 40.0,
                    swing1_limit_deg: 40.0,
                    swing2_limit_deg: 30.0,
                },
                mass: 4.5,
                pos_to_com: Vec3::ZERO,
                collider: LimbCollider::Sphere { radius: 0.12 },
                profile: LimbProfile::default(),
            },
            // 9: RightUpperArm
            HumanoidBoneDef {
                bone: HumanoidBone::RightUpperArm,
                name: "RightArm",
                parent_index: Some(5),
                parent_anchor: Vec3::new(0.22, 0.14, 0.0),
                child_anchor: Vec3::new(0.0, 0.15, 0.0),
                joint_axis: Vec3::Z,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -60.0,
                    high_twist_limit_deg: 60.0,
                    swing1_limit_deg: 85.0,
                    swing2_limit_deg: 85.0,
                },
                mass: 3.5,
                pos_to_com: Vec3::new(0.0, -0.07, 0.0),
                collider: LimbCollider::Capsule {
                    radius: 0.055,
                    half_height: 0.15,
                    axis: Vec3::Y,
                },
                profile: LimbProfile::default(),
            },
            // 10: RightLowerArm
            HumanoidBoneDef {
                bone: HumanoidBone::RightLowerArm,
                name: "RightForeArm",
                parent_index: Some(9),
                parent_anchor: Vec3::new(0.0, -0.15, 0.0),
                child_anchor: Vec3::new(0.0, 0.14, 0.0),
                joint_axis: Vec3::X,
                secondary_axis: Vec3::Y,
                limits: JointLimits {
                    low_twist_limit_deg: -5.0,
                    high_twist_limit_deg: 5.0,
                    swing1_limit_deg: 135.0,
                    swing2_limit_deg: 5.0,
                },
                mass: 2.0,
                pos_to_com: Vec3::new(0.0, -0.06, 0.0),
                collider: LimbCollider::Capsule {
                    radius: 0.045,
                    half_height: 0.14,
                    axis: Vec3::Y,
                },
                profile: LimbProfile::default(),
            },
        ];

        Self { bones }
    }

    /// Total mass of the humanoid character in kilograms.
    pub fn total_mass(&self) -> f32 {
        self.bones.iter().map(|b| b.mass).sum()
    }

    /// Instantiate an [`AnimationFollower`] initialized with the standard 11 bones.
    pub fn build_animation_follower(&self, config: AnimationFollowerConfig) -> AnimationFollower {
        let mut limbs = Vec::with_capacity(11);

        for (i, def) in self.bones.iter().enumerate() {
            let joint_space = if i == 0 {
                None
            } else {
                Some(JointSpace::from_axes(
                    def.joint_axis,
                    def.secondary_axis,
                    Quat::IDENTITY,
                ))
            };

            let mut limb = ActiveLimb::new(
                def.name,
                Some(def.bone),
                def.pos_to_com,
                joint_space,
                config.p_force,
                config.d_force,
            );
            limb.profile = def.profile;
            limbs.push(limb);
        }

        AnimationFollower::new(config, limbs)
    }
}

/// High-level active ragdoll coordinator combining [`SlaveController`], [`AnimationFollower`], [`MasterController`], and [`CollisionDetector`].
///
/// Port and modern Rust equivalent of Unity's `HumanoidSetUp.cs`.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct HumanoidSetUp {
    /// Animation following physics system.
    pub follower: AnimationFollower,

    /// State machine controlling muscle strength and life status.
    pub controller: SlaveController,

    /// Collision detector and layer filter.
    pub collision_detector: CollisionDetector,

    /// Kinematic 3rd-person master controller.
    pub master: crate::master::MasterController,

    /// Optional 3rd-person observation camera.
    pub camera: Option<crate::camera::FlyCamera>,
}

impl HumanoidSetUp {
    /// Create a new humanoid setup from follower, controller, master, and layer mask.
    pub fn new(
        follower: AnimationFollower,
        controller: SlaveController,
        master: crate::master::MasterController,
        dont_loose_strength_mask: LayerMask,
    ) -> Self {
        Self {
            follower,
            controller,
            collision_detector: CollisionDetector::new(dont_loose_strength_mask),
            master,
            camera: None,
        }
    }

    /// Attach an observation camera.
    pub fn with_camera(mut self, camera: crate::camera::FlyCamera) -> Self {
        self.camera = Some(camera);
        self
    }

    /// Number of limbs registered in this humanoid ragdoll.
    #[inline]
    pub fn limb_count(&self) -> usize {
        self.follower.limbs.len()
    }

    /// Current operational state of the ragdoll.
    #[inline]
    pub fn state(&self) -> RagdollState {
        self.controller.state
    }

    /// Current muscle strength ratio [0.0, 1.0].
    #[inline]
    pub fn current_strength(&self) -> f32 {
        self.controller.current_strength
    }

    /// Whether the character is currently alive.
    #[inline]
    pub fn is_alive(&self) -> bool {
        self.controller.is_alive
    }

    /// Advance active ragdoll physics and strength state by fixed delta time `dt` seconds.
    pub fn update(
        &mut self,
        master_world_transforms: &[Transform3d],
        master_local_rotations: &[Quat],
        slave_states: &[SlaveLimbPhysicsState],
        dt: f32,
    ) -> Result<Vec<LimbDriveOutput>> {
        if dt <= 0.0 || !dt.is_finite() {
            return Err(ActiveRagdollError::InvalidDeltaTime(dt));
        }

        // 1. Advance state machine and strength interpolation
        self.controller.update(dt);

        // 2. Compute PD forces and joint targets
        self.follower.step(
            master_world_transforms,
            master_local_rotations,
            slave_states,
            self.controller.force_coefficient,
            self.controller.torque_coefficient,
            self.controller.is_alive,
            dt,
        )
    }

    /// Notify that a limb collided with an object on the given physics layer.
    pub fn on_collision_enter(&mut self, layer: u8) {
        if self.collision_detector.on_collision_enter(layer) {
            self.controller.on_collision_enter();
        }
    }

    /// Notify that a limb stopped colliding with an object on the given physics layer.
    pub fn on_collision_exit(&mut self, layer: u8) {
        if self.collision_detector.on_collision_exit(layer) {
            self.controller.on_collision_exit();
        }
    }

    /// Make the character go limp (zero muscle strength) and start revival countdown.
    #[inline]
    pub fn die(&mut self) {
        self.controller.die();
    }

    /// Revive the character and restore muscle drive.
    #[inline]
    pub fn come_alive(&mut self) {
        self.controller.come_alive();
    }

    /// Zero out all forces and strength instantly.
    #[inline]
    pub fn reset_forces(&mut self) {
        self.controller.reset_forces();
    }

    /// Full coordinated physics step: advances master locomotion, generates animated skeleton
    /// transforms, evaluates push-back constraint, steps muscle strength state machine, and
    /// calculates PD forces and joint drives for all limbs.
    pub fn step_character(
        &mut self,
        input: &crate::master::LocomotionInput,
        is_grounded: bool,
        slave_states: &[SlaveLimbPhysicsState],
        dt: f32,
    ) -> Result<Vec<LimbDriveOutput>> {
        if slave_states.len() != self.limb_count() {
            return Err(ActiveRagdollError::HierarchyMismatch {
                master_count: self.limb_count(),
                slave_count: slave_states.len(),
            });
        }

        let slave_hips_pos = slave_states[0].world_center_of_mass;
        let cam_yaw = self.camera.as_ref().map(|c| c.yaw_deg).unwrap_or(0.0);
        self.master.step(input, cam_yaw, slave_hips_pos, is_grounded, dt);
        let master_pose = self.master.generate_master_poses();

        self.update(
            &master_pose.world_transforms,
            &master_pose.local_rotations,
            slave_states,
            dt,
        )
    }

    /// Set individual limb profile by index.
    pub fn set_limb_profile(&mut self, index: usize, profile: LimbProfile) -> Result<()> {
        if index >= self.follower.limbs.len() {
            return Err(ActiveRagdollError::IndexOutOfBounds(index, self.follower.limbs.len()));
        }
        self.follower.limbs[index].profile = profile;
        Ok(())
    }
}

/// Builder for constructing standard or custom humanoid active ragdoll setups.
#[derive(Debug, Clone, Default)]
pub struct HumanoidActiveRagdollBuilder {
    pub follower_config: AnimationFollowerConfig,
    pub controller_config: SlaveControllerConfig,
    pub master_config: crate::master::MasterControllerConfig,
    pub master_initial_pos: Vec3,
    pub master_initial_yaw: f32,
    pub dont_loose_strength_mask: LayerMask,
    pub limbs: Vec<ActiveLimb>,
}

impl HumanoidActiveRagdollBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_follower_config(mut self, config: AnimationFollowerConfig) -> Self {
        self.follower_config = config;
        self
    }

    pub fn with_controller_config(mut self, config: SlaveControllerConfig) -> Self {
        self.controller_config = config;
        self
    }

    pub fn with_master_config(mut self, config: crate::master::MasterControllerConfig) -> Self {
        self.master_config = config;
        self
    }

    pub fn with_master_initial_position(mut self, pos: Vec3) -> Self {
        self.master_initial_pos = pos;
        self
    }

    pub fn with_master_initial_yaw(mut self, yaw_deg: f32) -> Self {
        self.master_initial_yaw = yaw_deg;
        self
    }

    pub fn with_ignore_mask(mut self, mask: LayerMask) -> Self {
        self.dont_loose_strength_mask = mask;
        self
    }

    /// Add an active limb. The first added limb must be root (no joint).
    pub fn add_limb(mut self, limb: ActiveLimb) -> Self {
        self.limbs.push(limb);
        self
    }

    /// Creates a standard 11-bone humanoid ragdoll template matching the reference implementation.
    pub fn standard_humanoid_11() -> Self {
        let rig = StandardHumanoidRig::new();
        let follower = rig.build_animation_follower(AnimationFollowerConfig::default());

        Self {
            follower_config: follower.config,
            controller_config: SlaveControllerConfig::default(),
            master_config: crate::master::MasterControllerConfig::default(),
            master_initial_pos: Vec3::new(0.0, 0.95, 0.0),
            master_initial_yaw: 0.0,
            dont_loose_strength_mask: LayerMask::default(),
            limbs: follower.limbs,
        }
    }

    /// Build the configured [`HumanoidSetUp`].
    pub fn build(self) -> Result<HumanoidSetUp> {
        if self.limbs.is_empty() {
            return Err(ActiveRagdollError::HierarchyMismatch {
                master_count: 0,
                slave_count: 0,
            });
        }

        // Validate root limb has no joint
        if self.limbs[0].joint_space.is_some() {
            return Err(ActiveRagdollError::RootHasJoint);
        }

        // Validate child limbs have joints
        for (i, limb) in self.limbs.iter().enumerate().skip(1) {
            if limb.joint_space.is_none() {
                return Err(ActiveRagdollError::MissingJoint {
                    index: i,
                    name: limb.name.clone(),
                });
            }
        }

        let follower = AnimationFollower::new(self.follower_config, self.limbs);
        let controller = SlaveController::new(self.controller_config);
        let master = crate::master::MasterController::new(
            self.master_config,
            self.master_initial_pos,
            self.master_initial_yaw,
        );

        Ok(HumanoidSetUp::new(
            follower,
            controller,
            master,
            self.dont_loose_strength_mask,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_humanoid_rig_construction() {
        let rig = StandardHumanoidRig::new();
        assert_eq!(rig.bones.len(), 11);

        // Hips must be root
        assert!(rig.bones[0].parent_index.is_none());
        assert_eq!(rig.bones[0].name, "Hips");

        // Total humanoid mass ~ 70 - 80kg
        let total = rig.total_mass();
        assert!((65.0..=85.0).contains(&total));

        // All child bones must reference valid parents
        for i in 1..11 {
            let parent = rig.bones[i].parent_index.expect("child bone must have parent");
            assert!(parent < i);
        }
    }

    #[test]
    fn test_standard_humanoid_11_builder() {
        let setup = HumanoidActiveRagdollBuilder::standard_humanoid_11()
            .build()
            .expect("Standard 11 humanoid should build successfully");

        assert_eq!(setup.limb_count(), 11);
        assert_eq!(setup.follower.limbs[0].name, "Hips");
        assert!(setup.follower.limbs[0].is_root());
        assert!(!setup.follower.limbs[1].is_root());
    }

    #[test]
    fn test_root_has_joint_error() {
        let invalid = HumanoidActiveRagdollBuilder::new()
            .add_limb(ActiveLimb::new(
                "InvalidRoot",
                Some(HumanoidBone::Hips),
                Vec3::ZERO,
                Some(JointSpace::from_axes(Vec3::X, Vec3::Y, Quat::IDENTITY)), // ERROR: root cannot have joint
                8.0,
                0.01,
            ))
            .build();

        assert_eq!(invalid.unwrap_err(), ActiveRagdollError::RootHasJoint);
    }
}
