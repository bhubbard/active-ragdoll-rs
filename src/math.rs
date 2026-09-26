use glam::Vec3;

/// Clamps the magnitude of a vector to a maximum length.
#[inline]
pub fn clamp_magnitude(v: Vec3, max_length: f32) -> Vec3 {
    let len_sq = v.length_squared();
    if len_sq > max_length * max_length && len_sq > 0.0 {
        v * (max_length / len_sq.sqrt())
    } else {
        v
    }
}

/// Hermite interpolation between 0.0 and 1.0 (Unity `Mathf.SmoothStep`).
///
/// Returns 0.0 when $x \le edge0$, 1.0 when $x \ge edge1$, and smoothly
/// interpolates between them using $3t^2 - 2t^3$ where $t = \frac{x - edge0}{edge1 - edge0}$.
#[inline]
pub fn smooth_step(edge0: f32, edge1: f32, x: f32) -> f32 {
    if edge0 >= edge1 {
        return if x >= edge1 { 1.0 } else { 0.0 };
    }
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Normalizes an angle in degrees to the [-180, 180] range.
#[inline]
pub fn delta_angle_deg(current: f32, target: f32) -> f32 {
    let mut diff = (target - current) % 360.0;
    if diff > 180.0 {
        diff -= 360.0;
    } else if diff < -180.0 {
        diff += 360.0;
    }
    diff
}

/// Gradually changes an angle given in degrees towards a desired goal angle over time.
///
/// Direct port of Unity's `Mathf.SmoothDampAngle`.
///
/// # Arguments
/// * `current` - Current angle in degrees.
/// * `target` - Target angle in degrees.
/// * `current_velocity` - In/out current rotational velocity in degrees per second.
/// * `smooth_time` - Approximately the time it takes to reach the target in seconds.
/// * `max_speed` - Maximum allowed angular velocity in degrees per second.
/// * `delta_time` - Time step in seconds.
pub fn smooth_damp_angle(
    current: f32,
    target: f32,
    current_velocity: &mut f32,
    smooth_time: f32,
    max_speed: f32,
    delta_time: f32,
) -> f32 {
    let target_adjusted = current + delta_angle_deg(current, target);
    smooth_damp(
        current,
        target_adjusted,
        current_velocity,
        smooth_time,
        max_speed,
        delta_time,
    )
}

/// Smoothly damps a scalar value towards a target using a spring-damper model.
///
/// Direct port of Unity's `Mathf.SmoothDamp`.
pub fn smooth_damp(
    current: f32,
    target: f32,
    current_velocity: &mut f32,
    smooth_time: f32,
    max_speed: f32,
    delta_time: f32,
) -> f32 {
    let smooth_time = smooth_time.max(0.0001);
    let omega = 2.0 / smooth_time;

    let x = omega * delta_time;
    let exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);

    let mut change = current - target;
    let original_to = target;

    // Clamp maximum speed
    let max_change = max_speed * smooth_time;
    change = change.clamp(-max_change, max_change);

    let target = current - change;
    let temp = (*current_velocity + omega * change) * delta_time;
    *current_velocity = (*current_velocity - omega * temp) * exp;

    let mut output = target + (change + temp) * exp;

    // Prevent overshooting past target
    if (original_to - current > 0.0) == (output > original_to) {
        output = original_to;
        *current_velocity = (output - original_to) / delta_time;
    }

    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_smooth_step() {
        assert_eq!(smooth_step(0.0, 1.0, -0.5), 0.0);
        assert_eq!(smooth_step(0.0, 1.0, 0.0), 0.0);
        assert_eq!(smooth_step(0.0, 1.0, 0.5), 0.5);
        assert_eq!(smooth_step(0.0, 1.0, 1.0), 1.0);
        assert_eq!(smooth_step(0.0, 1.0, 1.5), 1.0);
    }

    #[test]
    fn test_clamp_magnitude() {
        let v = Vec3::new(10.0, 0.0, 0.0);
        let clamped = clamp_magnitude(v, 4.0);
        assert!((clamped.length() - 4.0).abs() < 1e-5);

        let small = Vec3::new(2.0, 1.0, 0.0);
        let same = clamp_magnitude(small, 5.0);
        assert_eq!(same, small);
    }

    #[test]
    fn test_delta_angle_deg() {
        assert_eq!(delta_angle_deg(0.0, 90.0), 90.0);
        assert_eq!(delta_angle_deg(350.0, 10.0), 20.0);
        assert_eq!(delta_angle_deg(10.0, 350.0), -20.0);
        assert_eq!(delta_angle_deg(0.0, 180.0), 180.0);
    }

    #[test]
    fn test_smooth_damp_angle() {
        let mut vel = 0.0;
        let mut angle = 0.0;
        let target = 90.0;

        for _ in 0..120 {
            angle = smooth_damp_angle(angle, target, &mut vel, 0.35, f32::INFINITY, 1.0 / 60.0);
        }

        // After 2.0 seconds with smooth_time = 0.35s, angle should be converged to 90
        assert!((angle - target).abs() < 0.5);
    }
}
