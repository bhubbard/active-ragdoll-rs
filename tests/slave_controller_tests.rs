use active_ragdoll_rs::prelude::*;

#[test]
fn test_slave_strength_loss_and_recovery() {
    let mut ctrl = SlaveController::new(SlaveControllerConfig {
        loose_strength_lerp: 1.0,
        gain_strength_lerp: 0.1,
        min_contact_force: 0.2,
        min_contact_torque: 0.15,
        ..Default::default()
    });

    assert_eq!(ctrl.state, RagdollState::FollowingAnimation);
    assert_eq!(ctrl.current_strength, 1.0);
    assert_eq!(ctrl.force_coefficient, 1.0);
    assert_eq!(ctrl.torque_coefficient, 1.0);

    // Collision starts
    ctrl.on_collision_enter();
    assert_eq!(ctrl.current_number_of_collisions, 1);

    // After 0.4s: strength = 1.0 - 1.0 * 0.4 = 0.6
    ctrl.update(0.4);
    assert_eq!(ctrl.state, RagdollState::LoosingStrength);
    assert!((ctrl.current_strength - 0.6).abs() < 1e-4);

    // Coefficients should lerp: 0.2 + (1.0 - 0.2) * 0.6 = 0.68
    assert!((ctrl.force_coefficient - 0.68).abs() < 1e-4);
    // 0.15 + (1.0 - 0.15) * 0.6 = 0.66
    assert!((ctrl.torque_coefficient - 0.66).abs() < 1e-4);

    // Contact ends -> enters GainingStrength
    ctrl.on_collision_exit();
    assert_eq!(ctrl.current_number_of_collisions, 0);

    ctrl.update(1.0); // +0.1 strength -> 0.7
    assert_eq!(ctrl.state, RagdollState::GainingStrength);
    assert!((ctrl.current_strength - 0.7).abs() < 1e-4);

    // Full recovery past 1.0 clamps strength to 1.0
    ctrl.update(5.0);
    assert_eq!(ctrl.current_strength, 1.0);
    assert_eq!(ctrl.force_coefficient, 1.0);

    // Next step evaluates state as FollowingAnimation
    ctrl.update(0.02);
    assert_eq!(ctrl.state, RagdollState::FollowingAnimation);
}

#[test]
fn test_slave_death_and_automatic_revival() {
    let mut ctrl = SlaveController::new(SlaveControllerConfig {
        dead_time: 3.0,
        ..Default::default()
    });

    ctrl.die();
    assert_eq!(ctrl.state, RagdollState::Dead);
    assert_eq!(ctrl.force_coefficient, 0.0);
    assert_eq!(ctrl.torque_coefficient, 0.0);
    assert_eq!(ctrl.current_strength, 0.0);
    assert!(!ctrl.is_alive);

    // Step 2.0s -> still dead
    ctrl.update(2.0);
    assert_eq!(ctrl.state, RagdollState::Dead);
    assert!(!ctrl.is_alive);

    // Step 1.1s -> exceeds 3.0s -> revives!
    ctrl.update(1.1);
    assert!(ctrl.is_alive);
    assert_eq!(ctrl.state, RagdollState::GainingStrength);
}

#[test]
fn test_collision_counter_underflow_safety() {
    let mut ctrl = SlaveController::default();
    assert_eq!(ctrl.current_number_of_collisions, 0);

    // Calling exit when count is 0 shouldn't underflow
    ctrl.on_collision_exit();
    assert_eq!(ctrl.current_number_of_collisions, 0);
}
