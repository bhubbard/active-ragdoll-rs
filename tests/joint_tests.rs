use active_ragdoll_rs::prelude::*;
use glam::{Quat, Vec3};

#[test]
fn test_joint_follow_anim_rot_inverted() {
    let init_rot = Quat::IDENTITY;
    let init_pos = Vec3::ZERO;
    let follower = JointFollowAnimRot::new(init_rot, init_pos, true); // inverted = true

    let target_rot = Quat::from_rotation_z(0.3);
    let target_pos = Vec3::new(0.0, 1.0, 0.0);

    let (calc_rot, calc_pos) = follower.compute_target(target_rot, target_pos);
    assert!((calc_rot - target_rot.inverse()).length() < 1e-4);
    assert_eq!(calc_pos, target_pos);
}

#[test]
fn test_configurable_joint_limits() {
    let desc = ConfigurableJointDescriptor::from_character_joint_limits(
        Vec3::X,
        Vec3::Y,
        -15.0,
        25.0,
        60.0,
        45.0,
    );

    assert_eq!(desc.axis, Vec3::X);
    assert_eq!(desc.secondary_axis, Vec3::Y);
    assert_eq!(desc.angular_x_motion, JointMotionMode::Limited);
    assert_eq!(desc.angular_y_motion, JointMotionMode::Limited);
    assert_eq!(desc.angular_z_motion, JointMotionMode::Limited);
    assert_eq!(desc.x_motion, JointMotionMode::Locked);
    assert_eq!(desc.y_motion, JointMotionMode::Locked);
    assert_eq!(desc.z_motion, JointMotionMode::Locked);
    assert_eq!(desc.low_angular_x_limit_deg, -15.0);
    assert_eq!(desc.high_angular_x_limit_deg, 25.0);
    assert_eq!(desc.angular_y_limit_deg, 60.0);
    assert_eq!(desc.angular_z_limit_deg, 45.0);
}
