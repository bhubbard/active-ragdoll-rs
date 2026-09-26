use bitflags::bitflags;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

bitflags! {
    /// 32-bit layer mask compatible with Unity's layer mask system.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
    pub struct LayerMask: u32 {
        const DEFAULT        = 1 << 0;
        const TRANSPARENT_FX = 1 << 1;
        const IGNORE_RAYCAST = 1 << 2;
        const WATER          = 1 << 4;
        const UI             = 1 << 5;
        const PLAYER         = 1 << 8;
        const RAGDOLL        = 1 << 9;
        const ENVIRONMENT    = 1 << 10;
        const ALL            = u32::MAX;
    }
}

#[cfg(feature = "serde")]
impl<'de> Deserialize<'de> for LayerMask {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let bits = u32::deserialize(deserializer)?;
        Ok(LayerMask::from_bits_truncate(bits))
    }
}

#[cfg(feature = "serde")]
impl Serialize for LayerMask {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.bits().serialize(serializer)
    }
}

impl LayerMask {
    /// Check whether a 0-indexed layer (0..31) is contained in this mask.
    #[inline]
    pub fn contains_layer(&self, layer: u8) -> bool {
        if layer >= 32 {
            false
        } else {
            (self.bits() & (1u32 << layer)) != 0
        }
    }

    /// Add a layer (0..31) to this mask.
    #[inline]
    pub fn with_layer(mut self, layer: u8) -> Self {
        if layer < 32 {
            self |= LayerMask::from_bits_truncate(1u32 << layer);
        }
        self
    }

    /// Remove a layer (0..31) from this mask.
    #[inline]
    pub fn without_layer(mut self, layer: u8) -> Self {
        if layer < 32 {
            self &= !LayerMask::from_bits_truncate(1u32 << layer);
        }
        self
    }
}

/// Tracks collision contacts for ragdoll limbs, filtering ignored layers.
#[derive(Debug, Clone, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CollisionDetector {
    /// Mask containing layers that do NOT cause the ragdoll to lose strength.
    pub ignore_mask: LayerMask,
    /// Active contact count on this specific limb.
    pub active_collisions: usize,
}

impl CollisionDetector {
    pub const fn new(ignore_mask: LayerMask) -> Self {
        Self {
            ignore_mask,
            active_collisions: 0,
        }
    }

    /// Returns true if a collision on `other_layer` should cause strength degradation.
    #[inline]
    pub fn should_count_collision(&self, other_layer: u8) -> bool {
        !self.ignore_mask.contains_layer(other_layer)
    }

    /// Called when a collision begins. Returns true if this collision counts towards weakening.
    pub fn on_collision_enter(&mut self, other_layer: u8) -> bool {
        if self.should_count_collision(other_layer) {
            self.active_collisions = self.active_collisions.saturating_add(1);
            true
        } else {
            false
        }
    }

    /// Called when a collision ends. Returns true if this collision counted towards weakening.
    pub fn on_collision_exit(&mut self, other_layer: u8) -> bool {
        if self.should_count_collision(other_layer) {
            self.active_collisions = self.active_collisions.saturating_sub(1);
            true
        } else {
            false
        }
    }

    /// Reset collision counter.
    #[inline]
    pub fn reset(&mut self) {
        self.active_collisions = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_layer_mask_operations() {
        let mask = LayerMask::PLAYER | LayerMask::RAGDOLL;
        assert!(mask.contains_layer(8));
        assert!(mask.contains_layer(9));
        assert!(!mask.contains_layer(0));
        assert!(!mask.contains_layer(10));

        let updated = mask.with_layer(10);
        assert!(updated.contains_layer(10));

        let cleared = updated.without_layer(8);
        assert!(!cleared.contains_layer(8));
    }

    #[test]
    fn test_collision_detector() {
        let mask = LayerMask::PLAYER; // ignore player
        let mut detector = CollisionDetector::new(mask);

        // Player layer collision (layer 8) -> ignored
        assert!(!detector.on_collision_enter(8));
        assert_eq!(detector.active_collisions, 0);

        // Environment obstacle collision (layer 10) -> counted
        assert!(detector.on_collision_enter(10));
        assert_eq!(detector.active_collisions, 1);

        assert!(detector.on_collision_exit(10));
        assert_eq!(detector.active_collisions, 0);
    }
}
