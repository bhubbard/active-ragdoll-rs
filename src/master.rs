use glam::{Quat, Vec3};

use crate::math::{clamp_magnitude, smooth_damp_angle, smooth_step};
use crate::types::{HumanoidBone, Transform3d};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// High-level locomotion state of the animated master controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum CharacterState {
    #[default]
    Idle,
    Walking,
    Running,
    Falling,
}

// Aliases matching Unity uppercase enum naming
#[allow(non_upper_case_globals)]
impl CharacterState {
    pub const IDLE: Self = Self::Idle;
    pub const WALKING: Self = Self::Walking;
    pub const RUNNING: Self = Self::Running;
    pub const FALLING: Self = Self::Falling;
}

/// Configuration settings for [`MasterController`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MasterControllerConfig {
    /// Walking locomotion speed in m/s (default 3.0).
    pub walk_speed: f32,
    /// Running locomotion speed in m/s (default 7.0).
    pub run_speed: f32,
    /// Downward gravitational acceleration in m/s^2 (default 9.81).
    pub gravity: f32,
    /// Smooth damping time for character turning in seconds (default 0.35).
    pub turn_smooth_time: f32,
    /// Speed at which master is pushed back towards slave when idle (default 7.0).
    pub push_back_speed: f32,
    /// Maximum allowed distance between master and slave roots before strong pushback (default 2.0).
    pub max_possible_distance_from_ragdoll: f32,
    /// Determines how high will character float above ground (default 0.1).
    pub ground_distance: f32,
    /// Layer mask containing layers considered ground (default ENVIRONMENT).
    pub ground_mask: crate::collision::LayerMask,
}

impl Default for MasterControllerConfig {
    fn default() -> Self {
        Self {
            walk_speed: 3.0,
            run_speed: 7.0,
            gravity: 9.81,
            turn_smooth_time: 0.35,
            push_back_speed: 7.0,
            max_possible_distance_from_ragdoll: 2.0,
            ground_distance: 0.1,
            ground_mask: crate::collision::LayerMask::ENVIRONMENT,
        }
    }
}

/// User input structure for locomotion.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LocomotionInput {
    /// Horizontal axis (-1.0 left to 1.0 right).
    pub horizontal: f32,
    /// Vertical axis (-1.0 backward to 1.0 forward).
    pub vertical: f32,
    /// Sprint / run toggle (LeftShift).
    pub run: bool,
}

impl LocomotionInput {
    #[inline]
    pub fn is_moving(&self) -> bool {
        self.horizontal.abs() > 0.01 || self.vertical.abs() > 0.01
    }

    #[inline]
    pub fn direction(&self) -> Vec3 {
        let dir = Vec3::new(self.horizontal, 0.0, self.vertical);
        if dir.length_squared() > 1.0 {
            dir.normalize()
        } else {
            dir
        }
    }
}

/// Third-person kinematic master controller.
///
/// Direct port of Unity's `MasterController.cs`.
/// Drives the kinematic skeleton and prevents it from desynchronizing
/// from the physics ragdoll through push-back constraints.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MasterController {
    /// Configuration parameters.
    pub config: MasterControllerConfig,
    /// Current locomotion state.
    pub state: CharacterState,
    /// World position of master root.
    pub position: Vec3,
    /// Current yaw rotation in degrees around Y axis.
    pub rotation_y_deg: f32,
    /// Vertical falling velocity in m/s.
    pub current_fall_velocity: f32,
    /// Smooth damping velocity for yaw turning.
    pub turn_smooth_velocity: f32,
    /// Animation gait phase in radians [0, 2π).
    pub gait_phase: f32,
}

impl Default for MasterController {
    fn default() -> Self {
        Self::new(MasterControllerConfig::default(), Vec3::ZERO, 0.0)
    }
}

impl MasterController {
    /// Construct a new master controller at given position and yaw angle.
    pub fn new(config: MasterControllerConfig, position: Vec3, rotation_y_deg: f32) -> Self {
        Self {
            config,
            state: CharacterState::Idle,
            position,
            rotation_y_deg,
            current_fall_velocity: 0.0,
            turn_smooth_velocity: 0.0,
            gait_phase: 0.0,
        }
    }

    /// Calculates push-back movement vector towards slave ragdoll root.
    ///
    /// Matches Unity:
    /// ```text
    /// slaveToMasterVector = slaveRoot.position - masterRoot.position;
    /// distance = slaveToMasterVector.magnitude;
    /// ratio = Clamp(distance / maxPossibleDistanceFromRagdoll, 0, 1);
    /// magnitude = SmoothStep(0, 1, ratio);
    /// moveVector = ClampMagnitude(slaveToMasterVector, magnitude);
    /// moveVector.y = 0;
    /// ```
    pub fn calculate_push_back_movement(&self, slave_root_position: Vec3) -> Vec3 {
        let slave_to_master = slave_root_position - self.position;
        let distance = slave_to_master.length();

        let ratio = (distance / self.config.max_possible_distance_from_ragdoll).clamp(0.0, 1.0);
        let magnitude = smooth_step(0.0, 1.0, ratio);
        let mut move_vector = clamp_magnitude(slave_to_master, magnitude);
        move_vector.y = 0.0;
        move_vector
    }

    /// Evaluates current character locomotion state based on input and grounding.
    pub fn determine_character_state(&self, input: &LocomotionInput, is_grounded: bool) -> CharacterState {
        if !is_grounded {
            CharacterState::Falling
        } else if input.is_moving() && input.run {
            CharacterState::Running
        } else if input.is_moving() {
            CharacterState::Walking
        } else {
            CharacterState::Idle
        }
    }

    /// Step the master controller by time step `dt` (seconds).
    ///
    /// - `input`: WASD / direction input and run flag.
    /// - `camera_angle_y_deg`: Camera yaw in degrees.
    /// - `slave_root_position`: Current world position of slave ragdoll hips.
    /// - `is_grounded`: Whether character has ground support.
    /// - `dt`: Physics fixed delta time in seconds.
    pub fn step(
        &mut self,
        input: &LocomotionInput,
        camera_angle_y_deg: f32,
        slave_root_position: Vec3,
        is_grounded: bool,
        dt: f32,
    ) {
        if dt <= 0.0 || !dt.is_finite() {
            return;
        }

        self.state = self.determine_character_state(input, is_grounded);

        if self.state != CharacterState::Falling {
            self.current_fall_velocity = 0.0;
        }

        let direction = input.direction();

        // Calculate smooth character angle when there is movement input
        if direction.length_squared() > 0.001 {
            let target_angle = direction.x.atan2(direction.z).to_degrees() + camera_angle_y_deg;
            self.rotation_y_deg = smooth_damp_angle(
                self.rotation_y_deg,
                target_angle,
                &mut self.turn_smooth_velocity,
                self.config.turn_smooth_time,
                f32::INFINITY,
                dt,
            );
        }

        let move_rot = Quat::from_rotation_y(self.rotation_y_deg.to_radians());
        let move_direction = if direction.length_squared() > 0.001 {
            (move_rot * Vec3::Z).normalize()
        } else {
            Vec3::ZERO
        };

        let push_back_movement = self.calculate_push_back_movement(slave_root_position);
        let movement = move_direction + push_back_movement;

        match self.state {
            CharacterState::Falling => {
                self.current_fall_velocity -= self.config.gravity * dt;
                self.position.y += self.current_fall_velocity * dt;
            }
            CharacterState::Running => {
                self.position += movement * self.config.run_speed * dt;
                self.gait_phase += 14.0 * dt; // fast cadence
            }
            CharacterState::Walking => {
                self.position += movement * self.config.walk_speed * dt;
                self.gait_phase += 8.0 * dt; // normal walk cadence
            }
            CharacterState::Idle => {
                self.position += push_back_movement * self.config.push_back_speed * dt;
                self.gait_phase += 2.0 * dt; // breathing cadence
            }
        }

        // Keep gait phase within [0, 2pi)
        self.gait_phase %= std::f32::consts::TAU;
    }

    /// World transform of the master root.
    #[inline]
    pub fn root_transform(&self) -> Transform3d {
        Transform3d {
            position: self.position,
            rotation: Quat::from_rotation_y(self.rotation_y_deg.to_radians()),
        }
    }

    /// Returns the integer animation condition code matching Unity `anim.SetInteger("Cond", ...)`.
    ///
    /// - `0`: Idle
    /// - `1`: Walking
    /// - `2`: Running
    /// - `3`: Falling
    #[inline]
    pub fn anim_cond(&self) -> i32 {
        match self.state {
            CharacterState::Idle => 0,
            CharacterState::Walking => 1,
            CharacterState::Running => 2,
            CharacterState::Falling => 3,
        }
    }

    /// Checks if master is grounded via sphere check at feet level (Unity `Physics.CheckSphere`).
    #[inline]
    pub fn check_sphere_grounded(&self, ground_surface_y: f32) -> bool {
        self.position.y <= ground_surface_y + self.config.ground_distance
    }

    /// Procedurally synthesizes master animation poses for the standard 11 humanoid bones.
    ///
    /// Generates world transforms and local rotations for:
    /// `[Hips, LeftUpperLeg, LeftLowerLeg, RightUpperLeg, RightLowerLeg, Spine, LeftUpperArm, LeftLowerArm, Head, RightUpperArm, RightLowerArm]`
    pub fn generate_master_poses(&self) -> MasterSkeletonPose {
        let phase = self.gait_phase;
        let root_rot = Quat::from_rotation_y(self.rotation_y_deg.to_radians());

        let mut local_rots = [Quat::IDENTITY; 11];

        match self.state {
            CharacterState::Idle => {
                // Breathing and subtle idle weight shifting
                let breath = (phase * 1.5).sin() * 0.03;
                let sway = (phase * 0.8).cos() * 0.02;

                local_rots[HumanoidBone::Hips.index()] =
                    root_rot * Quat::from_rotation_z(sway);
                local_rots[HumanoidBone::Spine.index()] =
                    Quat::from_rotation_x(breath) * Quat::from_rotation_z(-sway * 0.5);
                local_rots[HumanoidBone::Head.index()] =
                    Quat::from_rotation_x(-breath * 0.5);

                // Relaxed arms
                local_rots[HumanoidBone::LeftUpperArm.index()] =
                    Quat::from_rotation_z(0.15) * Quat::from_rotation_x(0.05);
                local_rots[HumanoidBone::LeftLowerArm.index()] = Quat::from_rotation_x(-0.2);
                local_rots[HumanoidBone::RightUpperArm.index()] =
                    Quat::from_rotation_z(-0.15) * Quat::from_rotation_x(0.05);
                local_rots[HumanoidBone::RightLowerArm.index()] = Quat::from_rotation_x(-0.2);

                // Slight leg stance
                local_rots[HumanoidBone::LeftUpperLeg.index()] = Quat::from_rotation_z(0.05);
                local_rots[HumanoidBone::RightUpperLeg.index()] = Quat::from_rotation_z(-0.05);
            }
            CharacterState::Walking => {
                let leg_swing = phase.sin() * 0.45;
                let knee_flex_l = (phase.sin().max(0.0)) * 0.6;
                let knee_flex_r = ((-phase).sin().max(0.0)) * 0.6;
                let arm_swing = phase.sin() * 0.35;
                let _hip_bob = (phase * 2.0).cos() * 0.04;

                local_rots[HumanoidBone::Hips.index()] = root_rot * Quat::from_rotation_y((phase).sin() * 0.08);
                local_rots[HumanoidBone::Spine.index()] =
                    Quat::from_rotation_x(0.08) * Quat::from_rotation_y((-phase).sin() * 0.08);
                local_rots[HumanoidBone::Head.index()] = Quat::from_rotation_x(-0.04);

                // Legs antiphase
                local_rots[HumanoidBone::LeftUpperLeg.index()] = Quat::from_rotation_x(leg_swing);
                local_rots[HumanoidBone::LeftLowerLeg.index()] = Quat::from_rotation_x(-knee_flex_l);
                local_rots[HumanoidBone::RightUpperLeg.index()] = Quat::from_rotation_x(-leg_swing);
                local_rots[HumanoidBone::RightLowerLeg.index()] = Quat::from_rotation_x(-knee_flex_r);

                // Arms swing antiphase to legs (left arm forward with right leg)
                local_rots[HumanoidBone::LeftUpperArm.index()] =
                    Quat::from_rotation_x(-arm_swing) * Quat::from_rotation_z(0.12);
                local_rots[HumanoidBone::LeftLowerArm.index()] = Quat::from_rotation_x(-0.35);
                local_rots[HumanoidBone::RightUpperArm.index()] =
                    Quat::from_rotation_x(arm_swing) * Quat::from_rotation_z(-0.12);
                local_rots[HumanoidBone::RightLowerArm.index()] = Quat::from_rotation_x(-0.35);
            }
            CharacterState::Running => {
                let leg_swing = phase.sin() * 0.85;
                let knee_flex_l = (phase.sin().max(0.0)) * 1.2;
                let knee_flex_r = ((-phase).sin().max(0.0)) * 1.2;
                let arm_swing = phase.sin() * 0.75;

                // Forward lean in run
                local_rots[HumanoidBone::Hips.index()] = root_rot * Quat::from_rotation_x(0.15);
                local_rots[HumanoidBone::Spine.index()] = Quat::from_rotation_x(0.12);
                local_rots[HumanoidBone::Head.index()] = Quat::from_rotation_x(-0.1);

                local_rots[HumanoidBone::LeftUpperLeg.index()] = Quat::from_rotation_x(leg_swing);
                local_rots[HumanoidBone::LeftLowerLeg.index()] = Quat::from_rotation_x(-knee_flex_l);
                local_rots[HumanoidBone::RightUpperLeg.index()] = Quat::from_rotation_x(-leg_swing);
                local_rots[HumanoidBone::RightLowerLeg.index()] = Quat::from_rotation_x(-knee_flex_r);

                local_rots[HumanoidBone::LeftUpperArm.index()] =
                    Quat::from_rotation_x(-arm_swing) * Quat::from_rotation_z(0.2);
                local_rots[HumanoidBone::LeftLowerArm.index()] = Quat::from_rotation_x(-0.7);
                local_rots[HumanoidBone::RightUpperArm.index()] =
                    Quat::from_rotation_x(arm_swing) * Quat::from_rotation_z(-0.2);
                local_rots[HumanoidBone::RightLowerArm.index()] = Quat::from_rotation_x(-0.7);
            }
            CharacterState::Falling => {
                // Arms up and legs flailing
                let flail = (phase * 3.0).sin() * 0.2;
                local_rots[HumanoidBone::Hips.index()] = root_rot * Quat::from_rotation_x(-0.1);
                local_rots[HumanoidBone::Spine.index()] = Quat::from_rotation_x(-0.15);
                local_rots[HumanoidBone::Head.index()] = Quat::from_rotation_x(0.2);

                local_rots[HumanoidBone::LeftUpperLeg.index()] = Quat::from_rotation_x(0.2 + flail);
                local_rots[HumanoidBone::LeftLowerLeg.index()] = Quat::from_rotation_x(-0.4);
                local_rots[HumanoidBone::RightUpperLeg.index()] = Quat::from_rotation_x(0.2 - flail);
                local_rots[HumanoidBone::RightLowerLeg.index()] = Quat::from_rotation_x(-0.4);

                local_rots[HumanoidBone::LeftUpperArm.index()] =
                    Quat::from_rotation_z(0.9) * Quat::from_rotation_x(-0.3);
                local_rots[HumanoidBone::LeftLowerArm.index()] = Quat::from_rotation_x(-0.4);
                local_rots[HumanoidBone::RightUpperArm.index()] =
                    Quat::from_rotation_z(-0.9) * Quat::from_rotation_x(-0.3);
                local_rots[HumanoidBone::RightLowerArm.index()] = Quat::from_rotation_x(-0.4);
            }
        }

        // Compute forward kinematics world transforms for all 11 bones
        let world_transforms = compute_humanoid_fk(self.position, &local_rots);

        MasterSkeletonPose {
            world_transforms,
            local_rotations: local_rots.to_vec(),
        }
    }
}

/// Computed pose of the master skeleton for active ragdoll animation tracking.
#[derive(Debug, Clone, PartialEq)]
pub struct MasterSkeletonPose {
    pub world_transforms: Vec<Transform3d>,
    pub local_rotations: Vec<Quat>,
}

/// Compute forward kinematics for standard 11 humanoid bones from local rotations and base dimensions.
pub fn compute_humanoid_fk(root_pos: Vec3, local_rots: &[Quat; 11]) -> Vec<Transform3d> {
    let mut xfs = vec![Transform3d::IDENTITY; 11];

    // Standard bone offsets (realistic humanoid proportions)
    // 0: Hips (Root)
    let hips_rot = local_rots[0];
    xfs[0] = Transform3d::new(root_pos, hips_rot);

    // 1: LeftUpperLeg (offset from hips: -0.1, -0.05, 0.0)
    let l_hip_offset = hips_rot * Vec3::new(-0.12, -0.05, 0.0);
    let l_thigh_rot = hips_rot * local_rots[1];
    xfs[1] = Transform3d::new(xfs[0].position + l_hip_offset, l_thigh_rot);

    // 2: LeftLowerLeg (offset from LeftUpperLeg: 0.0, -0.42, 0.0)
    let l_knee_offset = l_thigh_rot * Vec3::new(0.0, -0.42, 0.0);
    let l_shin_rot = l_thigh_rot * local_rots[2];
    xfs[2] = Transform3d::new(xfs[1].position + l_knee_offset, l_shin_rot);

    // 3: RightUpperLeg (offset from hips: +0.12, -0.05, 0.0)
    let r_hip_offset = hips_rot * Vec3::new(0.12, -0.05, 0.0);
    let r_thigh_rot = hips_rot * local_rots[3];
    xfs[3] = Transform3d::new(xfs[0].position + r_hip_offset, r_thigh_rot);

    // 4: RightLowerLeg (offset from RightUpperLeg: 0.0, -0.42, 0.0)
    let r_knee_offset = r_thigh_rot * Vec3::new(0.0, -0.42, 0.0);
    let r_shin_rot = r_thigh_rot * local_rots[4];
    xfs[4] = Transform3d::new(xfs[3].position + r_knee_offset, r_shin_rot);

    // 5: Spine (offset from hips: 0.0, +0.22, 0.0)
    let spine_offset = hips_rot * Vec3::new(0.0, 0.22, 0.0);
    let spine_rot = hips_rot * local_rots[5];
    xfs[5] = Transform3d::new(xfs[0].position + spine_offset, spine_rot);

    // 6: LeftUpperArm (offset from spine: -0.22, +0.20, 0.0)
    let l_shoulder_offset = spine_rot * Vec3::new(-0.22, 0.20, 0.0);
    let l_arm_rot = spine_rot * local_rots[6];
    xfs[6] = Transform3d::new(xfs[5].position + l_shoulder_offset, l_arm_rot);

    // 7: LeftLowerArm (offset from LeftUpperArm: 0.0, -0.30, 0.0)
    let l_elbow_offset = l_arm_rot * Vec3::new(0.0, -0.30, 0.0);
    let l_forearm_rot = l_arm_rot * local_rots[7];
    xfs[7] = Transform3d::new(xfs[6].position + l_elbow_offset, l_forearm_rot);

    // 8: Head (offset from spine: 0.0, +0.32, 0.0)
    let neck_offset = spine_rot * Vec3::new(0.0, 0.32, 0.0);
    let head_rot = spine_rot * local_rots[8];
    xfs[8] = Transform3d::new(xfs[5].position + neck_offset, head_rot);

    // 9: RightUpperArm (offset from spine: +0.22, +0.20, 0.0)
    let r_shoulder_offset = spine_rot * Vec3::new(0.22, 0.20, 0.0);
    let r_arm_rot = spine_rot * local_rots[9];
    xfs[9] = Transform3d::new(xfs[5].position + r_shoulder_offset, r_arm_rot);

    // 10: RightLowerArm (offset from RightUpperArm: 0.0, -0.30, 0.0)
    let r_elbow_offset = r_arm_rot * Vec3::new(0.0, -0.30, 0.0);
    let r_forearm_rot = r_arm_rot * local_rots[10];
    xfs[10] = Transform3d::new(xfs[9].position + r_elbow_offset, r_forearm_rot);

    xfs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_push_back_movement() {
        let master = MasterController::new(
            MasterControllerConfig {
                max_possible_distance_from_ragdoll: 2.0,
                ..Default::default()
            },
            Vec3::new(0.0, 0.0, 0.0),
            0.0,
        );

        // Ragdoll is close (0.5m) -> small push-back
        let pb_close = master.calculate_push_back_movement(Vec3::new(0.5, 0.0, 0.0));
        assert!(pb_close.x > 0.0);
        assert!(pb_close.x < 0.5);

        // Ragdoll is at max distance (2.0m) -> strong push-back
        let pb_far = master.calculate_push_back_movement(Vec3::new(2.0, 0.0, 0.0));
        assert!((pb_far.x - 1.0).abs() < 1e-4);
    }

    #[test]
    fn test_locomotion_step() {
        let mut master = MasterController::default();
        let input = LocomotionInput {
            horizontal: 0.0,
            vertical: 1.0,
            run: false,
        };

        let initial_pos = master.position;
        master.step(&input, 0.0, initial_pos, true, 0.1);

        assert_eq!(master.state, CharacterState::Walking);
        assert!(master.position.z > initial_pos.z);
    }

    #[test]
    fn test_master_poses_generation() {
        let master = MasterController::default();
        let pose = master.generate_master_poses();

        assert_eq!(pose.world_transforms.len(), 11);
        assert_eq!(pose.local_rotations.len(), 11);
    }
}
