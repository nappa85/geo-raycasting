#![deny(warnings)]
#![deny(missing_docs)]

//! # geo-raycasting
//!
//! Ray Casting algorithm for geo crate

use geo_types::{Point, Line, LineString, Polygon, CoordinateType};

use num_traits::float::Float;

fn pt_in_polygon<T: CoordinateType + Float>(pt: &Point<T>, poly: &LineString<T>) -> bool {
    let count = poly.lines()
        .filter(|line| ray_intersect_seg(pt, line))
        .count();
 
    count % 2 == 1
}

fn ray_intersect_seg<T: CoordinateType + Float>(p: &Point<T>, line: &Line<T>) -> bool {
    let mut pt = p.clone();
    let (a, b) = if line.start.y > line.end.y {
        (&line.end, &line.start)
    }
    else {
        (&line.start, &line.end)
    };

    if pt.y() == a.y || pt.y() == b.y {
        pt.set_y(pt.y() + T::min_positive_value());
    }
 
    if (pt.y() > b.y || pt.y() < a.y) || pt.x() > a.x.max(b.x) {
        false
    } else if pt.x() < a.x.min(b.x) {
        true
    } else {
        let m_red = if (a.x - b.x).abs() > T::min_positive_value() {
            (b.y - a.y) / (b.x - a.x)
        } else {
            T::max_value()
        };
        let m_blue = if (a.x - pt.x()).abs() > T::max_value() {
            (pt.y() - a.y) / (pt.x() - a.x)
        } else {
            T::max_value()
        };
        m_blue >= m_red
    }
}

pub trait RayCasting<T: CoordinateType + Float> {
    fn within(&self, pt: &Point<T>) -> bool;
}

impl<T: CoordinateType + Float> RayCasting<T> for LineString<T> {
    fn within(&self, pt: &Point<T>) -> bool {
        pt_in_polygon(pt, self)
    }
}

impl<T: CoordinateType + Float> RayCasting<T> for Polygon<T> {
    fn within(&self, pt: &Point<T>) -> bool {
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
