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
        let m_blue = if (a.x - pt_x).abs() > T::min_positive_value() {
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
            !self.interiors().iter().any(|line| pt_in_polygon(&coord, line))
    }
}

impl<T: CoordinateType + Float> RayCasting<T, Coordinate<T>> for Polygon<T> {
    fn within(&self, pt: &Coordinate<T>) -> bool {
        pt_in_polygon(pt, self.exterior()) &&
            !self.interiors().iter().any(|line| pt_in_polygon(pt, line))
    }
}

#[cfg(test)]
mod tests {
    use super::RayCasting;

    use geo_types::{Coordinate, LineString, Polygon, Point};

    fn p(x: f64, y: f64) -> Coordinate<f64> {
        (x, y).into()
    }

    #[test]
    fn poly_square() {
        let poly_square: LineString<f64> = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)].into();
        assert!(poly_square.within(&p(5.0, 5.0)));
        assert!(poly_square.within(&p(5.0, 8.0)));
        assert!(poly_square.within(&p(-10.0, 5.0)) == false);
        assert!(poly_square.within(&p(0.0, 5.0)) == false);
        assert!(poly_square.within(&p(8.0, 5.0)));
        assert!(poly_square.within(&p(10.0, 10.0)) == false);
    }

    #[test]
    fn poly_square_hole() {
        let poly_square_hole: Polygon<f64> = Polygon::new(
            LineString::from(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)]),
            vec![LineString::from(vec![(2.5, 2.5), (7.5, 2.5), (7.5, 7.5), (2.5, 7.5), (2.5, 2.5)])]
        );
        assert!(poly_square_hole.within(&p(5.0, 5.0)) == false);
        assert!(poly_square_hole.within(&p(5.0, 8.0)));
        assert!(poly_square_hole.within(&p(-10.0, 5.0)) == false);
        assert!(poly_square_hole.within(&p(0.0, 5.0)) == false);
        assert!(poly_square_hole.within(&p(10.0, 5.0)));
        assert!(poly_square_hole.within(&p(8.0, 5.0)));
        assert!(poly_square_hole.within(&p(10.0, 10.0)) == false);
    }

    #[test]
    fn poly_strange() {
        let poly_strange: LineString<f64> = vec![(0.0, 0.0), (2.5, 2.5), (0.0, 10.0), (2.5, 7.5), (7.5, 7.5), (10.0, 10.0), (10.0, 0.0), (2.5, 2.5)].into();
        assert!(poly_strange.within(&p(5.0, 5.0)));
        assert!(poly_strange.within(&p(5.0, 8.0)) == false);
        assert!(poly_strange.within(&p(-10.0, 5.0)) == false);
        assert!(poly_strange.within(&p(0.0, 5.0)) == false);
        assert!(poly_strange.within(&p(10.0, 5.0)));
        assert!(poly_strange.within(&p(8.0, 5.0)));
        assert!(poly_strange.within(&p(10.0, 10.0)) == false);
    }

    #[test]
    fn poly_hexagon() {
        let poly_hexagon: LineString<f64> = vec![(3.0, 0.0), (7.0, 0.0), (10.0, 5.0), (7.0, 10.0), (3.0, 10.0), (0.0, 5.0), (3.0, 0.0)].into();
        assert!(poly_hexagon.within(&p(5.0, 5.0)) == false);
        assert!(poly_hexagon.within(&p(5.0, 8.0)));
        assert!(poly_hexagon.within(&p(-10.0, 5.0)) == false);
        assert!(poly_hexagon.within(&p(0.0, 5.0)) == false);
        assert!(poly_hexagon.within(&p(10.0, 5.0)) == false);
        assert!(poly_hexagon.within(&p(8.0, 5.0)) == false);
        assert!(poly_hexagon.within(&p(10.0, 10.0)));
    }

    #[test]
    fn real_coords() {
        let cell1 = Polygon::new(LineString(vec![Coordinate { x: 45.3563321662796, y: 11.9147053956319 }, Coordinate { x: 45.4293499926637, y: 11.9455630525467 }, Coordinate { x: 45.4392542159797, y: 11.8515426867682 }, Coordinate { x: 45.3661863570488, y: 11.8209138798751 }]), vec![]);
        let cell2 = Polygon::new(LineString(vec![Coordinate { x: 45.4293499926637, y: 11.9455630525467 }, Coordinate { x: 45.5024707283596, y: 11.9765478474091 }, Coordinate { x: 45.5124252464723, y: 11.8822977972565 }, Coordinate { x: 45.4392542159797, y: 11.8515426867682 }]), vec![]);
        let cell3 = Polygon::new(LineString(vec![Coordinate { x: 45.3661863570488, y: 11.8209138798751 }, Coordinate { x: 45.4392542159797, y: 11.8515426867682 }, Coordinate { x: 45.4490695215551, y: 11.7576024308158 }, Coordinate { x: 45.3759520538385, y: 11.7272026072339 }]), vec![]);
        let point1 = Point(Coordinate { x: 45.429671680421, y: 11.887047957258 });
        let point2 = Point(Coordinate { x: 45.412408636479, y: 11.866946356603 });
        let point3 = Point(Coordinate { x: 45.390711713006, y: 11.868550140008 });
        let point4 = Point(Coordinate { x: 45.421928106575, y: 11.897589742744 });
        let point5 = Point(Coordinate { x: 45.414838131946, y: 11.811773142492 });
        let point6 = Point(Coordinate { x: 45.41341604488, y: 11.802568326636 });
        let point7 = Point(Coordinate { x: 45.395726701315, y: 11.833525908467 });
        assert!(cell1.within(&point1));
        assert!(cell2.within(&point1) == false);
        assert!(cell3.within(&point1) == false);
        assert!(cell1.within(&point2));
        assert!(cell2.within(&point2) == false);
        assert!(cell3.within(&point2) == false);
        assert!(cell1.within(&point3));
        assert!(cell2.within(&point3) == false);
        assert!(cell3.within(&point3) == false);
        assert!(cell1.within(&point4));
        assert!(cell2.within(&point4) == false);
        assert!(cell3.within(&point4) == false);
        assert!(cell1.within(&point5) == false);
        assert!(cell2.within(&point5) == false);
        assert!(cell3.within(&point5));
        assert!(cell1.within(&point6) == false);
        assert!(cell2.within(&point6) == false);
        assert!(cell3.within(&point6));
        assert!(cell1.within(&point7));
        assert!(cell2.within(&point7) == false);
        assert!(cell3.within(&point7) == false);
    }
}
