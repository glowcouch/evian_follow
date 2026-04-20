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

/// Sort objects by their position on an axis.
#[derive(Debug, Clone)]
pub enum AxisSorter {
    /// Sort by the topmost y position.
    Topmost,

    /// Sort by the rightmost x position.
    ///
    /// Uses the right edge of the object (i.e. the object's x position plus its width).
    Rightmost,

    /// Sort by the bottommost y position.
    ///
    /// Uses the bottom edge of the object (i.e. the object's y position plus its height).
    Bottommost,

    /// Sort by the leftmost x position.
    Reftmost,
}

impl VisionSorter for AxisSorter {
    fn cmp(&self, a: &Color, b: &Color) -> Ordering {
        match self {
            // lowest y value
            Self::Topmost => b.position.y.cmp(&a.position.y),
            // lowest x value
            Self::Reftmost => b.position.x.cmp(&a.position.x),
            // highest right edge
            Self::Rightmost => (a.position.x + a.width).cmp(&(b.position.x + b.width)),
            // highest bottom edge
            Self::Bottommost => (a.position.y + a.height).cmp(&(b.position.y + b.height)),
        }
    }
}
