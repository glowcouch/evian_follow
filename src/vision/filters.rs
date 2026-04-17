//! Vision sensor object-level filters.
//!
//! Use these to filter out unwanted objects.
//!
//! You can combine filters using tuples. Filters to the left will short-circuit filters to the
//! right.

use core::ops::Range;

use impl_trait_for_tuples::impl_for_tuples;
use vexide::math::Point2;

use super::Color;

/// Object-level filter.
pub trait VisionFilter {
    /// Returns true if the object is wanted, returns false if it is unwanted.
    fn filter(&self, object: &Color) -> bool;
}

impl VisionFilter for () {
    fn filter(&self, _: &Color) -> bool {
        true
    }
}

#[impl_for_tuples(1, 10)]
impl VisionFilter for Tuple {
    fn filter(&self, object: &Color) -> bool {
        for_tuples!( #(
            if !Tuple.filter(object) { return false; }
        )* );

        true
    }
}

/// Filter out objects that do not intersect with the bounding box
#[derive(Debug, Clone)]
pub struct AABBFilter {
    /// The top left corner of box (in pixels).
    pub position: Point2<u16>,

    /// The size of the box (in pixels).
    pub size: Point2<u16>,
}

impl VisionFilter for AABBFilter {
    fn filter(&self, object: &Color) -> bool {
        let a_min = self.position;
        let a_max = Point2 {
            x: self.position.x + self.size.x,
            y: self.position.y + self.size.y,
        };

        let b_min = object.position;
        let b_max = Point2 {
            x: object.position.x + object.width,
            y: object.position.y + object.height,
        };

        // AABB-AABB intersection test
        (a_min.x <= b_max.x && a_max.x >= b_min.x) && (a_min.y <= b_max.y && a_max.y >= b_min.y)
    }
}

/// Filter out objects that are too big or small.
#[derive(Debug, Clone)]
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
#[derive(Debug, Clone)]
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

/// Inverts the filter `T`.
#[derive(Debug, Clone)]
pub struct Not<T: VisionFilter>(pub T);

impl<T: VisionFilter> VisionFilter for Not<T> {
    fn filter(&self, object: &Color) -> bool {
        !self.0.filter(object)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_aabb_filter_inside() {
        let filter = AABBFilter {
            position: Point2 { x: 10, y: 10 },
            size: Point2 { x: 100, y: 100 },
        };
        let obj = Color {
            position: Point2 { x: 20, y: 20 },
            width: 10,
            height: 10,
            id: 0,
        };
        assert!(filter.filter(&obj));
    }

    #[test]
    fn test_aabb_filter_outside_left() {
        let filter = AABBFilter {
            position: Point2 { x: 10, y: 10 },
            size: Point2 { x: 100, y: 100 },
        };
        let obj = Color {
            position: Point2 { x: 0, y: 20 },
            width: 5,
            height: 5,
            id: 0,
        };
        assert!(!filter.filter(&obj));
    }

    #[test]
    fn test_aabb_filter_outside_bottom_right() {
        let filter = AABBFilter {
            position: Point2 { x: 10, y: 10 },
            size: Point2 { x: 100, y: 100 },
        };
        let obj = Color {
            position: Point2 { x: 111, y: 111 },
            width: 10,
            height: 10,
            id: 0,
        };
        assert!(!filter.filter(&obj));
    }

    #[test]
    fn test_aabb_filter_on_edge() {
        let filter = AABBFilter {
            position: Point2 { x: 10, y: 10 },
            size: Point2 { x: 100, y: 100 },
        };
        let obj = Color {
            position: Point2 { x: 0, y: 0 },
            width: 20,
            height: 20,
            id: 0,
        };
        assert!(filter.filter(&obj));
    }
}
