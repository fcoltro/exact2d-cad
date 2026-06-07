use exact2d_geometry::{Curve, CurveSegment};

/// A planar region defined by boundary loops.
///
/// - The outer boundary is wound CCW (positive area).
/// - Each hole is wound CW (negative area).
/// - Interior: { p | winding_number(p, boundaries) is odd }
#[derive(Clone, Debug)]
pub struct Region {
    /// Outer boundary — closed loop of curve segments (CCW).
    pub outer: Vec<Curve>,
    /// Holes — each is a closed loop wound CW.
    pub holes: Vec<Vec<Curve>>,
}

impl Region {
    pub fn new(outer: Vec<Curve>) -> Self {
        Region { outer, holes: Vec::new() }
    }

    pub fn with_holes(outer: Vec<Curve>, holes: Vec<Vec<Curve>>) -> Self {
        Region { outer, holes }
    }

    /// Compute the signed area using the shoelace formula (float approximation).
    pub fn signed_area_f64(&self) -> f64 {
        boundary_signed_area(&self.outer)
            - self.holes.iter().map(|h| boundary_signed_area(h).abs()).sum::<f64>()
    }

    /// Winding number of a point with respect to the outer boundary.
    /// Used for inside/outside classification.
    pub fn winding_number(&self, px: f64, py: f64) -> i32 {
        // Sum the SIGNED winding of every loop. By convention the outer boundary is
        // wound CCW (+1 inside) and holes are wound CW (−1 inside the hole). Adding
        // the signed contributions makes the hole interior cancel to 0 (outside),
        // the ring between outer and hole stay at +1 (inside), and the exterior 0.
        let mut wn = winding_number_boundary(&self.outer, px, py);
        for hole in &self.holes {
            wn += winding_number_boundary(hole, px, py);
        }
        wn
    }

    pub fn contains_point(&self, px: f64, py: f64) -> bool {
        self.winding_number(px, py) != 0
    }
}

/// Shoelace formula for the signed area of a closed boundary.
fn boundary_signed_area(boundary: &[Curve]) -> f64 {
    let steps = 20usize;
    let mut area = 0.0;
    for seg in boundary {
        let (t0, t1) = seg.domain();
        let dt = (t1 - t0) / steps as f64;
        for i in 0..steps {
            let t_a = t0 + dt * i as f64;
            let t_b = t0 + dt * (i + 1) as f64;
            let (x1, y1) = seg.evaluate_f64(t_a);
            let (x2, y2) = seg.evaluate_f64(t_b);
            area += (x1 + x2) * (y2 - y1);
        }
    }
    area / 2.0
}

/// Winding number of a point (px, py) with respect to a closed boundary loop.
/// Uses the ray-casting + signed crossing count algorithm.
fn winding_number_boundary(boundary: &[Curve], px: f64, py: f64) -> i32 {
    let mut wn = 0i32;
    let steps = 32usize;
    for seg in boundary {
        let (t0, t1) = seg.domain();
        let dt = (t1 - t0) / steps as f64;
        for i in 0..steps {
            let ta = t0 + dt * i as f64;
            let tb = t0 + dt * (i + 1) as f64;
            let (x1, y1) = seg.evaluate_f64(ta);
            let (x2, y2) = seg.evaluate_f64(tb);
            if y1 <= py {
                if y2 > py && cross_sign(x1, y1, x2, y2, px, py) > 0.0 { wn += 1; }
            } else {
                if y2 <= py && cross_sign(x1, y1, x2, y2, px, py) < 0.0 { wn -= 1; }
            }
        }
    }
    wn
}

fn cross_sign(x1: f64, y1: f64, x2: f64, y2: f64, px: f64, py: f64) -> f64 {
    (x2 - x1) * (py - y1) - (px - x1) * (y2 - y1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_geometry::{LineSeg, Point2d, Curve};

    fn square_region() -> Region {
        // Unit square from (0,0) to (2,2)
        Region::new(vec![
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(0,0), Point2d::from_i64(2,0))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(2,0), Point2d::from_i64(2,2))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(2,2), Point2d::from_i64(0,2))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(0,2), Point2d::from_i64(0,0))),
        ])
    }

    #[test]
    fn interior_point() {
        let r = square_region();
        assert!(r.contains_point(1.0, 1.0));
    }

    #[test]
    fn exterior_point() {
        let r = square_region();
        assert!(!r.contains_point(5.0, 5.0));
    }

    #[test]
    fn signed_area_positive_ccw() {
        let r = square_region();
        let area = r.signed_area_f64();
        assert!(area > 0.0, "CCW boundary should have positive area, got {}", area);
        assert!((area - 4.0).abs() < 0.1, "area≈{}", area);
    }
}
