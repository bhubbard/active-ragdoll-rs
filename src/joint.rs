use glam::{Quat, Vec3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Direct joint animation following controller.
///
/// Port of Unity's `JointFollowAnimRot.cs`.
/// Allows a specific joint to directly mirror a target transform's local position
/// and rotation with optional inversion.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointFollowAnimRot {
    /// Invert rotational direction.
    pub invert: bool,
    /// Joint rotational torque spring stiffness (default 5_000_000.0).
    pub torque_force: f32,
    /// Joint angular velocity damping (default 0.0).
    pub angular_damping: f32,
    /// Maximum allowable joint torque force.
    pub max_force: f32,
    /// Linear position spring force.
    pub spring_force: f32,
    /// Linear position damping.
    pub spring_damping: f32,
    /// Initial inverse local rotation captured at start.
    pub starting_rotation: Quat,
    /// Initial local position captured at start.
    pub starting_position: Vec3,
}

impl Default for JointFollowAnimRot {
    fn default() -> Self {
        Self {
            invert: false,
            torque_force: 5_000_000.0,
            angular_damping: 0.0,
            max_force: 5_000_000.0,
            spring_force: 0.0,
            spring_damping: 0.0,
            starting_rotation: Quat::IDENTITY,
            starting_position: Vec3::ZERO,
        }
    }
}

impl JointFollowAnimRot {
    /// Initialize with target starting local transform.
    pub fn new(target_local_rot: Quat, target_local_pos: Vec3, invert: bool) -> Self {
        Self {
            invert,
            starting_rotation: target_local_rot.inverse(),
            starting_position: target_local_pos,
            ..Default::default()
        }
    }

    /// Calculate target rotation and position given current target local transform.
    ///
    /// Matches `JointFollowAnimRot.cs`:
    /// ```text
    /// if (invert)
    ///     joint.targetRotation = Quaternion.Inverse(target.localRotation * startingRotation);
    /// else
    ///     joint.targetRotation = target.localRotation * startingRotation;
    /// joint.targetPosition = target.localPosition;
    /// ```
    #[inline]
    pub fn compute_target(&self, target_local_rot: Quat, target_local_pos: Vec3) -> (Quat, Vec3) {
        let combined = target_local_rot * self.starting_rotation;
        let target_rot = if self.invert {
            combined.inverse()
        } else {
            combined
        };
        (target_rot.normalize(), target_local_pos)
    }
}

/// Motion freedom mode for joint axes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum JointMotionMode {
    #[default]
    Locked,
    Limited,
    Free,
}

/// Drive mode for rotational joint tracking.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum RotationDriveMode {
    #[default]
    Slerp,
    XyAndZ,
}

/// High-level 6DOF Configurable Joint descriptor.
///
/// Port of Unity's `ConfigurableJoint` and `ReplaceJoints.cs`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConfigurableJointDescriptor {
    pub axis: Vec3,
    pub secondary_axis: Vec3,
    pub x_motion: JointMotionMode,
    pub y_motion: JointMotionMode,
    pub z_motion: JointMotionMode,
    pub angular_x_motion: JointMotionMode,
    pub angular_y_motion: JointMotionMode,
    pub angular_z_motion: JointMotionMode,
    pub low_angular_x_limit_deg: f32,
    pub high_angular_x_limit_deg: f32,
    pub angular_y_limit_deg: f32,
    pub angular_z_limit_deg: f32,
    pub rotation_drive_mode: RotationDriveMode,
    pub position_spring: f32,
    pub position_damper: f32,
    pub maximum_force: f32,
}

impl Default for ConfigurableJointDescriptor {
    fn default() -> Self {
        Self {
            axis: Vec3::X,
            secondary_axis: Vec3::Y,
            x_motion: JointMotionMode::Locked,
            y_motion: JointMotionMode::Locked,
            z_motion: JointMotionMode::Locked,
            angular_x_motion: JointMotionMode::Limited,
            angular_y_motion: JointMotionMode::Limited,
            angular_z_motion: JointMotionMode::Limited,
            low_angular_x_limit_deg: -20.0,
            high_angular_x_limit_deg: 20.0,
            angular_y_limit_deg: 45.0,
            angular_z_limit_deg: 45.0,
            rotation_drive_mode: RotationDriveMode::Slerp,
            position_spring: 2000.0,
            position_damper: 0.6,
            maximum_force: f32::MAX,
        }
    }
}

impl ConfigurableJointDescriptor {
    /// Port of `ReplaceJoints.cs`: Converts CharacterJoint limits into ConfigurableJoint limits.
    pub fn from_character_joint_limits(
        axis: Vec3,
        swing_axis: Vec3,
        low_twist_limit_deg: f32,
        high_twist_limit_deg: f32,
        swing1_limit_deg: f32,
        swing2_limit_deg: f32,
    ) -> Self {
        Self {
            axis,
            secondary_axis: swing_axis,
            x_motion: JointMotionMode::Locked,
            y_motion: JointMotionMode::Locked,
            z_motion: JointMotionMode::Locked,
            angular_x_motion: JointMotionMode::Limited,
            angular_y_motion: JointMotionMode::Limited,
            angular_z_motion: JointMotionMode::Limited,
            low_angular_x_limit_deg: low_twist_limit_deg,
            high_angular_x_limit_deg: high_twist_limit_deg,
            angular_y_limit_deg: swing1_limit_deg,
            angular_z_limit_deg: swing2_limit_deg,
            rotation_drive_mode: RotationDriveMode::Slerp,
            ..Default::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_joint_follow_anim_rot() {
        let init_rot = Quat::IDENTITY;
        let init_pos = Vec3::new(1.0, 2.0, 3.0);
        let follower = JointFollowAnimRot::new(init_rot, init_pos, false);

        let target_rot = Quat::from_rotation_y(0.5);
        let target_pos = Vec3::new(1.5, 2.5, 3.5);

        let (calc_rot, calc_pos) = follower.compute_target(target_rot, target_pos);
        assert!((calc_rot - target_rot).length() < 1e-5);
        assert_eq!(calc_pos, target_pos);
    }

    #[test]
    fn test_configurable_joint_from_character_joint() {
        let desc = ConfigurableJointDescriptor::from_character_joint_limits(
            Vec3::X,
            Vec3::Y,
            -25.0,
            25.0,
            40.0,
            30.0,
        );
        assert_eq!(desc.axis, Vec3::X);
        assert_eq!(desc.secondary_axis, Vec3::Y);
        assert_eq!(desc.low_angular_x_limit_deg, -25.0);
        assert_eq!(desc.high_angular_x_limit_deg, 25.0);
        assert_eq!(desc.angular_y_limit_deg, 40.0);
        assert_eq!(desc.angular_z_limit_deg, 30.0);
        assert_eq!(desc.rotation_drive_mode, RotationDriveMode::Slerp);
    }
}
