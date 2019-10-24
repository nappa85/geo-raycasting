// Copyright 2019 Marco Napetti
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.
#![deny(warnings)]
#![deny(missing_docs)]

//! # geo-raycasting
//!
//! Ray Casting algorithm for the geo crate

use geo_types::{Coordinate, CoordinateType, Line, LineString, Point, Polygon};

use num_traits::float::Float;

fn pt_in_polygon<T: CoordinateType + Float>(pt: &Coordinate<T>, poly: &LineString<T>) -> bool {
    let count = poly.lines()
        .filter(|line| ray_intersect_seg(pt, line))
        .count();
 
    count % 2 == 1
}

fn ray_intersect_seg<T: CoordinateType + Float>(p: &Coordinate<T>, line: &Line<T>) -> bool {
    let (pt_x, mut pt_y) = p.x_y();
    let (a, b) = if line.start.y > line.end.y {
        (&line.end, &line.start)
    }
    else {
        (&line.start, &line.end)
    };

    if pt_y == a.y || pt_y == b.y {
        pt_y = pt_y + T::min_positive_value();
    }
 
    if (pt_y > b.y || pt_y < a.y) || pt_x > a.x.max(b.x) {
        false
    } else if pt_x < a.x.min(b.x) {
        true
    } else {
        let m_red = if (a.x - b.x).abs() > T::min_positive_value() {
            (b.y - a.y) / (b.x - a.x)
        } else {
            T::max_value()
        };
        let m_blue = if (a.x - pt_x).abs() > T::max_value() {
            (pt_y - a.y) / (pt_x - a.x)
        } else {
            T::max_value()
        };
        m_blue >= m_red
    }
}

/// Trait implementing Ray Casting algorith
pub trait RayCasting<T: CoordinateType + Float, P: Into<Coordinate<T>>> {
    /// Checks if a point is within a polygonal area
    fn within(&self, pt: &P) -> bool;
}

impl<T: CoordinateType + Float> RayCasting<T, Point<T>> for LineString<T> {
    fn within(&self, pt: &Point<T>) -> bool {
        pt_in_polygon(&pt.x_y().into(), self)
    }
}

impl<T: CoordinateType + Float> RayCasting<T, Coordinate<T>> for LineString<T> {
    fn within(&self, pt: &Coordinate<T>) -> bool {
        pt_in_polygon(&pt, self)
    }
}

impl<T: CoordinateType + Float> RayCasting<T, Point<T>> for Polygon<T> {
    fn within(&self, pt: &Point<T>) -> bool {
        let coord = pt.x_y().into();
        pt_in_polygon(&coord, self.exterior()) &&
            !self.interiors().iter().any(|line| !pt_in_polygon(&coord, line))
    }
}

impl<T: CoordinateType + Float> RayCasting<T, Coordinate<T>> for Polygon<T> {
    fn within(&self, pt: &Coordinate<T>) -> bool {
        pt_in_polygon(pt, self.exterior()) &&
            !self.interiors().iter().any(|line| !pt_in_polygon(pt, line))
    }
}

#[cfg(test)]
mod tests {
    use super::RayCasting;

    use geo_types::{LineString, Polygon};

    #[test]
    fn poly_square() {
        let poly_square: LineString<f64> = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)].into();
        assert!(poly_square.within(&(5.0, 5.0).into()));
        assert!(poly_square.within(&(5.0, 8.0).into()));
        assert!(poly_square.within(&(-10.0, 5.0).into()) == false);
        assert!(poly_square.within(&(0.0, 5.0).into()) == false);
        assert!(poly_square.within(&(10.0, 5.0).into()));
        assert!(poly_square.within(&(8.0, 5.0).into()));
        assert!(poly_square.within(&(10.0, 10.0).into()) == false);
    }

    #[test]
    fn poly_square_hole() {
        let poly_square_hole: Polygon<f64> = Polygon::new(
            LineString::from(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)]),
            vec![LineString::from(vec![(2.5, 2.5), (7.5, 2.5), (7.5, 7.5), (2.5, 7.5), (2.5, 2.5)])]
        );
        assert!(poly_square_hole.within(&(5.0, 5.0).into()));
        assert!(poly_square_hole.within(&(5.0, 8.0).into()) == false);
        assert!(poly_square_hole.within(&(-10.0, 5.0).into()) == false);
        assert!(poly_square_hole.within(&(0.0, 5.0).into()) == false);
        assert!(poly_square_hole.within(&(10.0, 5.0).into()) == false);
        assert!(poly_square_hole.within(&(8.0, 5.0).into()) == false);
        assert!(poly_square_hole.within(&(10.0, 10.0).into()) == false);
    }

    #[test]
    fn poly_strange() {
        let poly_strange: LineString<f64> = vec![(0.0, 0.0), (2.5, 2.5), (0.0, 10.0), (2.5, 7.5), (7.5, 7.5), (10.0, 10.0), (10.0, 0.0), (2.5, 2.5)].into();
        assert!(poly_strange.within(&(5.0, 5.0).into()));
        assert!(poly_strange.within(&(5.0, 8.0).into()) == false);
        assert!(poly_strange.within(&(-10.0, 5.0).into()) == false);
        assert!(poly_strange.within(&(0.0, 5.0).into()) == false);
        assert!(poly_strange.within(&(10.0, 5.0).into()));
        assert!(poly_strange.within(&(8.0, 5.0).into()));
        assert!(poly_strange.within(&(10.0, 10.0).into()) == false);
    }

    #[test]
    fn poly_hexagon() {
        let poly_hexagon: LineString<f64> = vec![(3.0, 0.0), (7.0, 0.0), (10.0, 5.0), (7.0, 10.0), (3.0, 10.0), (0.0, 5.0), (3.0, 0.0)].into();
        assert!(poly_hexagon.within(&(5.0, 5.0).into()) == false);
        assert!(poly_hexagon.within(&(5.0, 8.0).into()));
        assert!(poly_hexagon.within(&(-10.0, 5.0).into()) == false);
        assert!(poly_hexagon.within(&(0.0, 5.0).into()) == false);
        assert!(poly_hexagon.within(&(10.0, 5.0).into()) == false);
        assert!(poly_hexagon.within(&(8.0, 5.0).into()) == false);
        assert!(poly_hexagon.within(&(10.0, 10.0).into()));
    }
}
