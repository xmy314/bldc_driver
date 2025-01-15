/// electromagnetic quantities.
pub mod em;

/// clamps a value between a min and a max.
pub fn clamp(v1: f32, low: f32, high: f32) -> f32 {
    if v1 > high {
        high
    } else if v1 < low {
        low
    } else {
        v1
    }
}
