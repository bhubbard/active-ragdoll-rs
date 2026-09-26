use active_ragdoll_rs::prelude::*;

/// Verifies SlaveController state machine, strength degradation,
/// contact coefficients, knockout (die), and timed revival.
#[test]
fn test_slave_controller_lifecycle() {
    let mut ctrl = SlaveController::new(SlaveControllerConfig {
        loose_strength_lerp: 1.0,
        gain_strength_lerp: 0.1,
        min_contact_force: 0.2,
        min_contact_torque: 0.15,
        dead_time: 3.0,
        max_force_coefficient: 1.0,
        max_torque_coefficient: 1.0,
    });

    assert_eq!(ctrl.state, RagdollState::FollowingAnimation);
    assert_eq!(ctrl.current_strength, 1.0);
    assert_eq!(ctrl.force_coefficient, 1.0);
    assert_eq!(ctrl.torque_coefficient, 1.0);

    // 1. Collision occurs -> enter LoosingStrength
    ctrl.on_collision_enter();
    assert_eq!(ctrl.current_number_of_collisions, 1);

    ctrl.update(0.5); // dt = 0.5s -> strength drops from 1.0 to 0.5
    assert_eq!(ctrl.state, RagdollState::LoosingStrength);
    assert!((ctrl.current_strength - 0.5).abs() < 1e-4);

    // Force coeff: lerp(0.2, 1.0, 0.5) = 0.6
    assert!((ctrl.force_coefficient - 0.6).abs() < 1e-4);
    // Torque coeff: lerp(0.15, 1.0, 0.5) = 0.575
    assert!((ctrl.torque_coefficient - 0.575).abs() < 1e-4);

    // Strength floor clamp at 0
    ctrl.update(1.0); // dt = 1.0s -> strength hits 0.0
    assert_eq!(ctrl.current_strength, 0.0);
    assert!((ctrl.force_coefficient - 0.2).abs() < 1e-4);
    assert!((ctrl.torque_coefficient - 0.15).abs() < 1e-4);

    // 2. Collision ceases -> enter GainingStrength
    ctrl.on_collision_exit();
    assert_eq!(ctrl.current_number_of_collisions, 0);

    ctrl.update(1.0); // dt = 1.0s -> gains 0.1 strength
    assert_eq!(ctrl.state, RagdollState::GainingStrength);
    assert!((ctrl.current_strength - 0.1).abs() < 1e-4);

    // 3. Knockout / Die
    ctrl.die();
    assert_eq!(ctrl.state, RagdollState::Dead);
    assert!(!ctrl.is_alive);
    assert_eq!(ctrl.force_coefficient, 0.0);
    assert_eq!(ctrl.torque_coefficient, 0.0);
    assert_eq!(ctrl.current_strength, 0.0);

    // Advance 2.0s (< 3.0s dead_time) -> still dead
    ctrl.update(2.0);
    assert_eq!(ctrl.state, RagdollState::Dead);
    assert!(!ctrl.is_alive);

    // Advance 1.1s (total 3.1s >= 3.0s) -> automatically revives!
    ctrl.update(1.1);
    assert!(ctrl.is_alive);
    assert_eq!(ctrl.state, RagdollState::GainingStrength);
}

#[test]
fn test_slave_controller_reset_forces() {
    let mut ctrl = SlaveController::default();
    ctrl.reset_forces();

    assert_eq!(ctrl.force_coefficient, 0.0);
    assert_eq!(ctrl.torque_coefficient, 0.0);
    assert_eq!(ctrl.current_strength, 0.0);
}
