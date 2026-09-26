use active_ragdoll_rs::prelude::*;

#[test]
fn test_fps_display_moving_average() {
    let mut fps = FpsDisplay::new(0.5); // fast smoothing
    assert_eq!(fps.fps(), 0.0);
    assert_eq!(fps.frame_time_ms(), 0.0);

    // First frame: 16.6ms (60 fps)
    fps.update(0.0166667);
    assert!((fps.fps() - 60.0).abs() < 0.5);
    assert!((fps.frame_time_ms() - 16.6667).abs() < 0.5);

    // Frame with drop to 30 fps (33.3ms)
    fps.update(0.0333333);
    // delta_time = 0.0166667 + (0.0333333 - 0.0166667) * 0.5 = 0.025s (40 fps)
    assert!((fps.fps() - 40.0).abs() < 0.5);

    // Negative or zero delta time should be safely ignored
    fps.update(0.0);
    fps.update(-0.05);
    assert!((fps.fps() - 40.0).abs() < 0.5);
}
