//! Vision sensor object-level filters.
//!
//! Use these to filter out unwanted objects.

use core::ops::Range;

use vexide::math::Point2;

use super::Color;

/// Object-level filter.
pub trait VisionFilter {
    /// Returns true if the object is wanted, returns false if it is unwanted.
    fn filter(&self, object: &Color) -> bool;
}

/// Filter out objects that are outside of the bounding box
pub struct AABBFilter {
    /// The top left corner of box (in pixels).
    pub position: Point2<u16>,

    /// The size of the box (in pixels).
    pub size: Point2<u16>,
}

impl VisionFilter for AABBFilter {
    fn filter(&self, object: &Color) -> bool {
        object.position.x > self.position.x
            && object.position.y > self.position.y
            && object.position.x + object.width < self.position.x + self.size.x
            && object.position.y + object.height < self.position.y + self.size.y
    }
}

/// Filter out objects that are too big or small.
pub struct AreaFilter {
    /// The range of allowed areas (in square pixels).
    ///
    /// If the size is a [`u16`], then [`u16::MAX`] squared fits inside a [`u32`].
    pub range: Range<u32>,
}

impl VisionFilter for AreaFilter {
    fn filter(&self, object: &Color) -> bool {
        let area = u32::from(object.width) * u32::from(object.height);
        self.range.contains(&area)
    }
}

/// Filter out objects outside of width and height ranges.
pub struct SizeFilter {
    /// The range of allowed heights.
    pub height_range: Range<u16>,

    /// The range of allowed widths.
    pub width_range: Range<u16>,
}

impl VisionFilter for SizeFilter {
    fn filter(&self, object: &Color) -> bool {
        self.height_range.contains(&object.height) && self.width_range.contains(&object.width)
    }
}
