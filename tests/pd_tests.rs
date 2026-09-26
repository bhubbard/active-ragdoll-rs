use active_ragdoll_rs::prelude::*;
use glam::Vec3;

#[test]
fn test_pd_3d_proportional_and_derivative() {
    let mut pd = PdController3d::new(10.0, 0.02);
    let dt = 0.02;

    // First step: error changes from 0 to 1.0 on X
    let signal1 = pd.calculate(Vec3::new(1.0, 0.0, 0.0), dt);
    // error = 1.0, derivative = (1.0 - 0.0) / 0.02 = 50.0
    // signal = 10.0 * (1.0 + 0.02 * 50.0) = 10.0 * 2.0 = 20.0
    assert!((signal1.x - 20.0).abs() < 1e-4);

    // Second step: error stays constant
    let signal2 = pd.calculate(Vec3::new(1.0, 0.0, 0.0), dt);
    // derivative = 0 -> signal = 10.0 * 1.0 = 10.0
    assert!((signal2.x - 10.0).abs() < 1e-4);
}

#[test]
fn test_pd_3d_invalid_dt() {
    let mut pd = PdController3d::new(5.0, 0.1);
    let err = Vec3::new(2.0, 3.0, 4.0);

    // Zero dt should return proportional only (no NaN or divide-by-zero)
    let s_zero = pd.calculate(err, 0.0);
    assert_eq!(s_zero, 5.0 * err);

    // Negative dt
    let s_neg = pd.calculate(err, -0.01);
    assert_eq!(s_neg, 5.0 * err);

    // Infinite dt
    let s_inf = pd.calculate(err, f32::INFINITY);
    assert_eq!(s_inf, 5.0 * err);
}

#[test]
fn test_pd_3d_reset() {
    let mut pd = PdController3d::new(10.0, 0.02);
    let _ = pd.calculate(Vec3::new(1.0, 0.0, 0.0), 0.02);
    assert_eq!(pd.last_error, Vec3::new(1.0, 0.0, 0.0));

    pd.reset();
    assert_eq!(pd.last_error, Vec3::ZERO);
}

#[test]
fn test_pd_1d_controller() {
    let mut pd1 = PdController1d::new(4.0, 0.05);
    let dt = 0.02;

    let s1 = pd1.calculate(2.0, dt);
    // derivative = (2.0 - 0.0) / 0.02 = 100.0
    // signal = 4.0 * (2.0 + 0.05 * 100.0) = 4.0 * 7.0 = 28.0
    assert!((s1 - 28.0).abs() < 1e-4);

    pd1.reset();
    assert_eq!(pd1.last_error, 0.0);
}

#[test]
fn test_clamp_magnitude_edge_cases() {
    assert_eq!(clamp_magnitude(Vec3::ZERO, 5.0), Vec3::ZERO);

    let v = Vec3::new(0.0, 100.0, 0.0);
    let clamped = clamp_magnitude(v, 10.0);
    assert!((clamped.length() - 10.0).abs() < 1e-4);
    assert!((clamped.y - 10.0).abs() < 1e-4);
}
