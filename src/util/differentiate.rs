//! Numerical differentiation.

/// Implements numerical differentiation.
pub struct Differentiate {
    /// Previous value
    prev: Option<f64>,
}

impl Differentiate {
    /// Construct a new instance of [`Differentiate`] with no samples.
    pub fn new() -> Self {
        Self { prev: None }
    }

    /// Get the next value
    pub fn next(&mut self, value: f64) -> Option<f64> {
        let next = value - self.prev?;
        self.prev = Some(value);
        Some(next)
    }
}
