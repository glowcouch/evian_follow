//! Exponential moving average implementation.
#![allow(dead_code, reason = "some functions could be useful later")]

/// Exponential moving average implementation.
pub struct Ema {
    /// The "smoothing factor".
    pub alpha: f64,

    /// The current value.
    value: Option<f64>,
}

impl Ema {
    /// Construct a new instance with no samples and a given smoothing factor.
    pub fn new(alpha: f64) -> Self {
        Self { alpha, value: None }
    }

    /// Get the current value
    pub fn value(&self) -> Option<f64> {
        self.value
    }

    /// Get the next value.
    pub fn next(&mut self, value: f64) -> f64 {
        if let Some(prev) = self.value {
            let next = self.alpha * value + (1. - self.alpha) * prev;
            self.value = Some(next);
            next
        } else {
            self.value = Some(value);
            value
        }
    }
}
