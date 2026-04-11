//! Vision sensor object-level comparison implementations.
//!
//! Use these to sort objects by higher-preference to lower-preference.

use core::cmp::Ordering;

use super::Color;

/// Object-level comparison implementation.
pub trait VisionSorter {
    /// Compare one object with another.
    fn cmp(&self, a: &Color, b: &Color) -> Ordering;
}

/// Sort objects by area (larger first).
#[derive(Debug, Clone)]
pub struct AreaSorter;

impl VisionSorter for AreaSorter {
    fn cmp(&self, a: &Color, b: &Color) -> Ordering {
        (b.width * b.height).cmp(&(a.width * a.height))
    }
}
