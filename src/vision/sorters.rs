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

// Sort objects by their position on an axis.
#[allow(missing_docs, reason = "it's fairly obvious")]
#[derive(Debug, Clone)]
pub enum AxisSorter {
    HighestY,
    HighestX,
    LowestY,
    LowestX,
}

impl VisionSorter for AxisSorter {
    fn cmp(&self, a: &Color, b: &Color) -> Ordering {
        match self {
            Self::HighestY => a.position.y.cmp(&b.position.y),
            Self::HighestX => a.position.x.cmp(&b.position.x),
            Self::LowestY => b.position.y.cmp(&a.position.y),
            Self::LowestX => b.position.x.cmp(&a.position.x),
        }
    }
}
