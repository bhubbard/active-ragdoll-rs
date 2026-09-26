use active_ragdoll_rs::prelude::*;

/// Verifies layer mask bitwise filtering and contact counting
/// matching Unity's CollisionDetector behavior.
#[test]
fn test_collision_filtering_with_layer_mask() {
    // Exclude PLAYER (layer 8) and RAGDOLL (layer 9) from causing strength loss
    let ignore_mask = LayerMask::PLAYER | LayerMask::RAGDOLL;
    let mut detector = CollisionDetector::new(ignore_mask);

    // Collision with Player -> ignored
    assert!(!detector.on_collision_enter(8));
    assert_eq!(detector.active_collisions, 0);

    // Collision with Ragdoll own limbs -> ignored
    assert!(!detector.on_collision_enter(9));
    assert_eq!(detector.active_collisions, 0);

    // Collision with obstacle on layer 10 (Environment) -> counted
    assert!(detector.on_collision_enter(10));
    assert_eq!(detector.active_collisions, 1);

    // Collision with another obstacle on layer 0 (Default) -> counted
    assert!(detector.on_collision_enter(0));
    assert_eq!(detector.active_collisions, 2);

    // Obstacle 1 exits
    assert!(detector.on_collision_exit(10));
    assert_eq!(detector.active_collisions, 1);

    // Obstacle 2 exits
    assert!(detector.on_collision_exit(0));
    assert_eq!(detector.active_collisions, 0);
}

#[test]
fn test_layer_mask_custom_layer_operations() {
    let mut mask = LayerMask::default();
    assert!(!mask.contains_layer(15));

    mask = mask.with_layer(15);
    assert!(mask.contains_layer(15));

    mask = mask.without_layer(15);
    assert!(!mask.contains_layer(15));
}
