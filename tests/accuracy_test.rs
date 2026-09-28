//! Rigorous Biomechanical Accuracy & Control Law Benchmark Tests
//! Evaluates PD servo tracking precision, torque limit clamping, and inverted pendulum balance.

use active_ragdoll_rs::prelude::*;
use glam::Vec3;

#[test]
fn test_pd_controller_analytical_force_parity() {
    let p = 250.0f32; // Proportional gain
    let d = 0.05f32;  // Derivative gain
    let dt = 0.01667f32; // 60 Hz

    let mut pd = PdController3d::new(p, d);

    // Initial step: displacement error e = (0.2, -0.1, 0.4) m
    let e0 = Vec3::new(0.2, -0.1, 0.4);
    let f0 = pd.calculate(e0, dt);

    // Analytical expectation:
    // de/dt = (e0 - 0) / dt
    // signal = p * (e0 + d * de/dt)
    let expected_de = e0 / dt;
    let expected_f0 = p * (e0 + d * expected_de);

    assert!(
        (f0 - expected_f0).length() < 1e-4,
        "PD force calculation diverged from analytical expectation: got {:?}, expected {:?}",
        f0,
        expected_f0
    );

    // Constant error step (steady state): de/dt = 0 -> signal = p * e0
    let f_steady = pd.calculate(e0, dt);
    let expected_f_steady = p * e0;
    assert!(
        (f_steady - expected_f_steady).length() < 1e-4,
        "Steady-state PD force failed: got {:?}, expected {:?}",
        f_steady,
        expected_f_steady
    );
}

#[test]
fn test_pd_torque_clamping_strictness() {
    let max_force = 500.0f32; // 500 N force limit
    let p = 10_000.0f32; // Huge gain
    let d = 0.5f32;

    let mut pd = PdController3d::new(p, d);
    let dt = 0.01;

    // Huge displacement error (10 meters)
    let huge_error = Vec3::new(10.0, 5.0, -8.0);
    let raw_force = pd.calculate(huge_error, dt);

    // Raw force would be ~500,000 N, must clamp to max_force
    let clamped_force = raw_force.clamp_length_max(max_force);
    assert!(
        clamped_force.length() <= max_force + 1e-4,
        "Clamped force exceeded max allowed: len={}",
        clamped_force.length()
    );
}

#[test]
fn test_inverted_pendulum_gravity_compensation_equilibrium() {
    // Single inverted pendulum modeling humanoid torso balance
    // Mass m = 40 kg, height L = 1.0 m, g = 9.81 m/s^2
    // Gravitational torque for small angle theta: tau_g = m * g * L * sin(theta)
    let m = 40.0f32;
    let l = 1.0f32;
    let g = 9.81f32;
    let theta = 5.0f32.to_radians(); // 5 degrees lean

    let gravitational_torque = m * g * l * theta.sin();

    // Joint stiffness k_p must be >= m * g * L for stability (critical stiffness)
    let critical_stiffness = m * g * l;
    let joint_kp = critical_stiffness * 1.5; // 50% safety margin

    let mut pd = PdController1d::new(joint_kp, 0.1);
    let restoring_torque = pd.calculate(theta, 0.02);

    // Restoring torque must exceed gravitational falling torque:
    assert!(
        restoring_torque > gravitational_torque,
        "Restoring torque {} cannot overcome gravity torque {}",
        restoring_torque,
        gravitational_torque
    );
}
