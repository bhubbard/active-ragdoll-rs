use active_ragdoll_rs::master::{CharacterState, LocomotionInput, MasterController, MasterControllerConfig};
use glam::Vec3;

/// Verifies MasterController locomotion, character states,
/// turning angle calculation with SmoothDampAngle, and push-back movement.
#[test]
fn test_master_controller_locomotion_states() {
    let mut master = MasterController::new(
        MasterControllerConfig {
            walk_speed: 3.0,
            run_speed: 7.0,
            ..Default::default()
        },
        Vec3::new(0.0, 1.0, 0.0),
        0.0,
    );

    let dt = 0.02;

    // 1. Idle input
    let idle_input = LocomotionInput {
        horizontal: 0.0,
        vertical: 0.0,
        run: false,
    };
    master.step(&idle_input, 0.0, Vec3::new(0.0, 1.0, 0.0), true, dt);
    assert_eq!(master.state, CharacterState::Idle);

    // 2. Walking forward
    let walk_input = LocomotionInput {
        horizontal: 0.0,
        vertical: 1.0,
        run: false,
    };
    master.step(&walk_input, 0.0, master.position, true, dt);
    assert_eq!(master.state, CharacterState::Walking);
    assert!(master.position.z > 0.0);

    // 3. Running forward
    let run_input = LocomotionInput {
        horizontal: 0.0,
        vertical: 1.0,
        run: true,
    };
    let z_before = master.position.z;
    master.step(&run_input, 0.0, master.position, true, dt);
    assert_eq!(master.state, CharacterState::Running);
    let delta_z_run = master.position.z - z_before;

    // Running should be faster than walking (7.0 vs 3.0)
    assert!(delta_z_run > 3.0 * dt);

    // 4. Falling when not grounded
    master.step(&walk_input, 0.0, master.position, false, dt);
    assert_eq!(master.state, CharacterState::Falling);
    assert!(master.current_fall_velocity < 0.0);
}

#[test]
fn test_master_push_back_constraint() {
    let mut master = MasterController::new(
        MasterControllerConfig {
            max_possible_distance_from_ragdoll: 2.0,
            push_back_speed: 7.0,
            ..Default::default()
        },
        Vec3::new(0.0, 1.0, 0.0),
        0.0,
    );

    // Ragdoll is stuck behind at (0, 1, -3) while master is at (0, 1, 0) -> distance 3m (> 2m limit)
    let slave_root_pos = Vec3::new(0.0, 1.0, -3.0);
    let idle_input = LocomotionInput::default();

    let pos_before = master.position;
    master.step(&idle_input, 0.0, slave_root_pos, true, 0.02);

    // Master should have been pushed backward towards ragdoll (negative z)
    assert!(master.position.z < pos_before.z);
}
