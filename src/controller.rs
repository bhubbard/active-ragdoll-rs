#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// High-level lifecycle states of an active ragdoll.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum RagdollState {
    /// Character is alive, free of obstacle collisions, and following animation with full strength.
    #[default]
    FollowingAnimation,
    /// Character is in contact with an obstacle/collider and actively losing muscle strength.
    LoosingStrength,
    /// Contact has ended, and character is gradually regaining muscle strength back to 100%.
    GainingStrength,
    /// Character is dead/knocked out, with zero muscle torque (fully limp physics ragdoll).
    Dead,
}

/// Configuration settings for [`SlaveController`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SlaveControllerConfig {
    /// Rate at which ragdoll loses strength per second while in contact with obstacles.
    ///
    /// Default: `1.0` (reaches minimum contact strength in 1 second of continuous contact).
    pub loose_strength_lerp: f32,

    /// Rate at which ragdoll regains strength per second after contacts cease.
    ///
    /// Default: `0.05` (takes ~20 seconds to fully regain strength, providing realistic recovery).
    pub gain_strength_lerp: f32,

    /// Minimum force multiplier during contact (0.0 to 1.0).
    ///
    /// Default: `0.1` (keeps 10% force to prevent total collapse immediately).
    pub min_contact_force: f32,

    /// Minimum torque multiplier during contact (0.0 to 1.0).
    ///
    /// Default: `0.1` (keeps 10% joint stiffness).
    pub min_contact_torque: f32,

    /// Duration of being dead in seconds before automatically reviving.
    ///
    /// Default: `4.0` seconds. Set to `f32::INFINITY` to prevent automatic revival.
    pub dead_time: f32,

    /// Maximum force coefficient at full strength.
    ///
    /// Default: `1.0`.
    pub max_force_coefficient: f32,

    /// Maximum torque coefficient at full strength.
    ///
    /// Default: `1.0`.
    pub max_torque_coefficient: f32,
}

impl Default for SlaveControllerConfig {
    fn default() -> Self {
        Self {
            loose_strength_lerp: 1.0,
            gain_strength_lerp: 0.05,
            min_contact_force: 0.1,
            min_contact_torque: 0.1,
            dead_time: 4.0,
            max_force_coefficient: 1.0,
            max_torque_coefficient: 1.0,
        }
    }
}

/// Controls ragdoll state transitions, muscle strength loss/gain, and death/revival.
///
/// Direct port of Unity's `SlaveController.cs`.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SlaveController {
    /// Configuration settings.
    pub config: SlaveControllerConfig,

    /// Current operational state.
    pub state: RagdollState,

    /// Current muscle strength ratio [0.0, 1.0].
    pub current_strength: f32,

    /// Number of active contacts currently affecting the ragdoll.
    pub current_number_of_collisions: usize,

    /// Elapsed time since death in seconds.
    pub current_dead_step: f32,

    /// Whether the character is alive. When false, ragdoll becomes completely limp.
    pub is_alive: bool,

    /// Output force coefficient applied to linear PD controllers.
    pub force_coefficient: f32,

    /// Output torque coefficient applied to joint rotational springs.
    pub torque_coefficient: f32,
}

impl Default for SlaveController {
    fn default() -> Self {
        Self::new(SlaveControllerConfig::default())
    }
}

impl SlaveController {
    /// Construct a new controller with the specified configuration.
    pub fn new(config: SlaveControllerConfig) -> Self {
        let max_force = config.max_force_coefficient;
        let max_torque = config.max_torque_coefficient;
        let dead_time = config.dead_time;

        Self {
            config,
            state: RagdollState::FollowingAnimation,
            current_strength: 1.0,
            current_number_of_collisions: 0,
            current_dead_step: dead_time,
            is_alive: true,
            force_coefficient: max_force,
            torque_coefficient: max_torque,
        }
    }

    /// Evaluates current state based on life status, collisions, and strength level.
    #[inline]
    pub fn determine_state(&self) -> RagdollState {
        if !self.is_alive {
            RagdollState::Dead
        } else if self.current_number_of_collisions > 0 {
            RagdollState::LoosingStrength
        } else if self.current_strength < 1.0 {
            RagdollState::GainingStrength
        } else {
            RagdollState::FollowingAnimation
        }
    }

    /// Advance the controller by a physics time step `dt` (in seconds).
    pub fn update(&mut self, dt: f32) {
        if dt <= 0.0 || !dt.is_finite() {
            return;
        }

        self.state = self.determine_state();

        match self.state {
            RagdollState::Dead => {
                self.current_dead_step += dt;
                if self.current_dead_step >= self.config.dead_time {
                    self.come_alive();
                }
            }
            RagdollState::LoosingStrength => {
                self.current_strength = (self.current_strength - self.config.loose_strength_lerp * dt).clamp(0.0, 1.0);
                self.interpolate_strength(self.current_strength);
            }
            RagdollState::GainingStrength => {
                self.current_strength = (self.current_strength + self.config.gain_strength_lerp * dt).clamp(0.0, 1.0);
                self.interpolate_strength(self.current_strength);
            }
            RagdollState::FollowingAnimation => {
                self.current_strength = 1.0;
                self.interpolate_strength(1.0);
            }
        }
    }

    /// Linear interpolation between minimum contact strength and maximum strength.
    #[inline]
    pub fn interpolate_strength(&mut self, ratio: f32) {
        let r = ratio.clamp(0.0, 1.0);
        self.force_coefficient = self.config.min_contact_force
            + (self.config.max_force_coefficient - self.config.min_contact_force) * r;
        self.torque_coefficient = self.config.min_contact_torque
            + (self.config.max_torque_coefficient - self.config.min_contact_torque) * r;
    }

    /// Kills the character, making it limp and disabling active forces.
    pub fn die(&mut self) {
        self.is_alive = false;
        self.current_dead_step = 0.0;
        self.reset_forces();
        self.state = RagdollState::Dead;
    }

    /// Revives the character, enabling animation following again.
    pub fn come_alive(&mut self) {
        self.is_alive = true;
        self.state = self.determine_state();
    }

    /// Sets animation following forces and strength to zero.
    ///
    /// After calling this, the ragdoll will gradually regain strength if alive.
    pub fn reset_forces(&mut self) {
        self.force_coefficient = 0.0;
        self.torque_coefficient = 0.0;
        self.current_strength = 0.0;
    }

    /// Increment collision count when a valid obstacle contact begins.
    #[inline]
    pub fn on_collision_enter(&mut self) {
        self.current_number_of_collisions = self.current_number_of_collisions.saturating_add(1);
    }

    /// Decrement collision count when a valid obstacle contact ends.
    #[inline]
    pub fn on_collision_exit(&mut self) {
        self.current_number_of_collisions = self.current_number_of_collisions.saturating_sub(1);
    }
}
