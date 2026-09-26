use active_ragdoll_rs::prelude::*;

#[test]
fn test_layer_mask_bitwise_operations() {
    let empty = LayerMask::empty();
    assert_eq!(empty.bits(), 0);

    let default_layer = LayerMask::DEFAULT;
    assert!(default_layer.contains_layer(0));
    assert!(!default_layer.contains_layer(1));

    let player_and_ragdoll = LayerMask::PLAYER | LayerMask::RAGDOLL;
    assert!(player_and_ragdoll.contains_layer(8));
    assert!(player_and_ragdoll.contains_layer(9));
    assert!(!player_and_ragdoll.contains_layer(0));

    // Dynamic addition & removal
    let with_env = player_and_ragdoll.with_layer(10);
    assert!(with_env.contains_layer(10));

    let without_player = with_env.without_layer(8);
    assert!(!without_player.contains_layer(8));
    assert!(without_player.contains_layer(9));
    assert!(without_player.contains_layer(10));
}

#[test]
fn test_layer_mask_out_of_bounds() {
    let mask = LayerMask::ALL;
    assert!(!mask.contains_layer(32));
    assert!(!mask.contains_layer(255));

    let unchanged = mask.with_layer(32);
    assert_eq!(unchanged, mask);
}

#[test]
fn test_collision_detector_layer_filtering() {
    // Ignore ground / environment
    let ignore_mask = LayerMask::ENVIRONMENT;
    let mut detector = CollisionDetector::new(ignore_mask);

    // Environment collision (layer 10) -> should be ignored
    assert!(!detector.should_count_collision(10));
    assert!(!detector.on_collision_enter(10));
    assert_eq!(detector.active_collisions, 0);

    // Obstacle on default layer (layer 0) -> should count
    assert!(detector.should_count_collision(0));
    assert!(detector.on_collision_enter(0));
    assert_eq!(detector.active_collisions, 1);

    assert!(detector.on_collision_exit(0));
    assert_eq!(detector.active_collisions, 0);
}
