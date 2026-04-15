//! Numerical differentiation.

/// Implements numerical differentiation.
pub struct Differentiate {
    /// Previous value
    prev: Option<f64>,
}

impl Default for Differentiate {
    fn default() -> Self {
        Self::new()
    }
}

impl Differentiate {
    /// Construct a new instance of [`Differentiate`] with no samples.
    #[must_use]
    pub fn new() -> Self {
        Self { prev: None }
    }

    /// Get the next value
    pub fn next(&mut self, value: f64) -> Option<f64> {
        let next = self.prev.map(|prev| value - prev);
        self.prev = Some(value);
        next
    }
}
