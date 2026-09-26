use glam::{Mat3, Quat, Vec3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// 3D rigid transform consisting of translation and rotation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Transform3d {
    pub position: Vec3,
    pub rotation: Quat,
}

impl Default for Transform3d {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform3d {
    pub const IDENTITY: Self = Self {
        position: Vec3::ZERO,
        rotation: Quat::IDENTITY,
    };

    #[inline]
    pub const fn new(position: Vec3, rotation: Quat) -> Self {
        Self { position, rotation }
    }

    #[inline]
    pub const fn from_translation(position: Vec3) -> Self {
        Self {
            position,
            rotation: Quat::IDENTITY,
        }
    }

    #[inline]
    pub const fn from_rotation(rotation: Quat) -> Self {
        Self {
            position: Vec3::ZERO,
            rotation,
        }
    }

    /// Transforms a point from local space to parent/world space:
    /// `p' = position + rotation * p`
    #[inline]
    pub fn transform_point(&self, point: Vec3) -> Vec3 {
        self.position + self.rotation * point
    }

    /// Transforms a vector (direction only) from local space to parent/world space:
    /// `v' = rotation * v`
    #[inline]
    pub fn transform_vector(&self, vector: Vec3) -> Vec3 {
        self.rotation * vector
    }

    /// Transforms a point from parent/world space to local space:
    /// `p = rotation.inverse() * (p' - position)`
    #[inline]
    pub fn inverse_transform_point(&self, world_point: Vec3) -> Vec3 {
        self.rotation.inverse() * (world_point - self.position)
    }

    /// Transforms a vector from parent/world space to local space:
    /// `v = rotation.inverse() * v'`
    #[inline]
    pub fn inverse_transform_vector(&self, world_vector: Vec3) -> Vec3 {
        self.rotation.inverse() * world_vector
    }

    /// Returns the inverse transform.
    #[inline]
    pub fn inverse(&self) -> Self {
        let inv_rot = self.rotation.inverse();
        Self {
            position: inv_rot * (-self.position),
            rotation: inv_rot,
        }
    }

    /// Multiplies two transforms (combines them: `self * other`).
    #[inline]
    pub fn mul_transform(&self, other: &Self) -> Self {
        Self {
            position: self.transform_point(other.position),
            rotation: (self.rotation * other.rotation).normalize(),
        }
    }
}

/// Orthonormal joint coordinate space representation.
///
/// In Unity ConfigurableJoints, the joint local space is formed by:
/// `axis` (primary axis) and `secondaryAxis` (perpendicular).
/// `forward = axis.cross(secondaryAxis)`
/// `up = secondaryAxis`
/// Joint space orientation converts rotations between limb space and joint drive space.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointSpace {
    /// Rotation transforming from limb local space into joint space
    pub local_to_joint_space: Quat,
    /// Reference initial local rotation combined with joint space
    pub start_local_rotation: Quat,
}

impl JointSpace {
    /// Construct joint space from joint axes and limb initial local rotation.
    pub fn from_axes(
        axis: Vec3,
        secondary_axis: Vec3,
        initial_local_rotation: Quat,
    ) -> Self {
        let primary = axis.normalize();
        let secondary = secondary_axis.normalize();
        let forward = primary.cross(secondary).normalize();
        let right = forward.cross(secondary).normalize();

        // Construct 3x3 orthonormal basis where:
        // column 0 = right, column 1 = secondary (up), column 2 = forward
        let rot_mat = Mat3::from_cols(right, secondary, forward);
        let local_to_joint_space = Quat::from_mat3(&rot_mat).normalize();
        let start_local_rotation = (initial_local_rotation * local_to_joint_space).normalize();

        Self {
            local_to_joint_space,
            start_local_rotation,
        }
    }

    /// Computes joint target rotation given the master animated bone's current local rotation.
    ///
    /// Matches the exact Unity calculation:
    /// `targetRotation = Inv(localToJointSpace) * Inv(masterLocalRotation) * startLocalRotation`
    #[inline]
    pub fn compute_target_rotation(&self, master_local_rotation: Quat) -> Quat {
        let inv_joint_space = self.local_to_joint_space.inverse();
        let inv_master = master_local_rotation.inverse();
        (inv_joint_space * inv_master * self.start_local_rotation).normalize()
    }
}

/// Per-limb tuning profile allowing individual overrides for strength and torque limits.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct LimbProfile {
    /// Multiplier for max linear force applied to this limb (default 1.0)
    pub max_force_scale: f32,
    /// Multiplier for max angular joint torque applied to this limb (default 1.0)
    pub max_joint_torque_scale: f32,
    /// Multiplier for joint damping on this limb (default 1.0)
    pub joint_damping_scale: f32,
    /// Whether forces and torques are active for this limb
    pub enabled: bool,
}

impl Default for LimbProfile {
    #[inline]
    fn default() -> Self {
        Self {
            max_force_scale: 1.0,
            max_joint_torque_scale: 1.0,
            joint_damping_scale: 1.0,
            enabled: true,
        }
    }
}

/// Standard humanoid bone indices matching standard 11-bone humanoid ragdoll setups.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum HumanoidBone {
    Hips = 0,
    LeftUpperLeg = 1,
    LeftLowerLeg = 2,
    RightUpperLeg = 3,
    RightLowerLeg = 4,
    Spine = 5,
    LeftUpperArm = 6,
    LeftLowerArm = 7,
    Head = 8,
    RightUpperArm = 9,
    RightLowerArm = 10,
}

impl HumanoidBone {
    pub const ALL_11: [HumanoidBone; 11] = [
        HumanoidBone::Hips,
        HumanoidBone::LeftUpperLeg,
        HumanoidBone::LeftLowerLeg,
        HumanoidBone::RightUpperLeg,
        HumanoidBone::RightLowerLeg,
        HumanoidBone::Spine,
        HumanoidBone::LeftUpperArm,
        HumanoidBone::LeftLowerArm,
        HumanoidBone::Head,
        HumanoidBone::RightUpperArm,
        HumanoidBone::RightLowerArm,
    ];

    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    pub const fn name(self) -> &'static str {
        match self {
            HumanoidBone::Hips => "Hips",
            HumanoidBone::LeftUpperLeg => "LeftUpperLeg",
            HumanoidBone::LeftLowerLeg => "LeftLowerLeg",
            HumanoidBone::RightUpperLeg => "RightUpperLeg",
            HumanoidBone::RightLowerLeg => "RightLowerLeg",
            HumanoidBone::Spine => "Spine",
            HumanoidBone::LeftUpperArm => "LeftUpperArm",
            HumanoidBone::LeftLowerArm => "LeftLowerArm",
            HumanoidBone::Head => "Head",
            HumanoidBone::RightUpperArm => "RightUpperArm",
            HumanoidBone::RightLowerArm => "RightLowerArm",
        }
    }

    /// Whether this bone is the ragdoll root (hips). Root does not have a joint.
    #[inline]
    pub const fn is_root(self) -> bool {
        matches!(self, HumanoidBone::Hips)
    }
}
