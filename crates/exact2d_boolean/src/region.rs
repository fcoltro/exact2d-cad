use exact2d_geometry::{Curve, CurveSegment, Point2d, tessellate_curve};
use robust::{orient2d, Coord};

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

/// Flatten one boundary segment to a polyline for shoelace / ray-cast
/// accumulation. Lines stay exact (two points); arcs and ellipses tessellate from
/// their exact lowered rational-Bézier form, and the chord tolerance scales with
/// the segment size so the approximation is uniform across magnitudes.
fn flatten_segment(seg: &Curve) -> Vec<Point2d> {
    let bb = seg.bounding_box();
    let diag = ((bb.max.x - bb.min.x).powi(2) + (bb.max.y - bb.min.y).powi(2)).sqrt();
    let tol = (diag * 1e-4).max(1e-9);
    tessellate_curve(seg, tol)
}

/// Shoelace formula for the signed area of a closed boundary.
fn boundary_signed_area(boundary: &[Curve]) -> f64 {
    let mut area = 0.0;
    for seg in boundary {
        for w in flatten_segment(seg).windows(2) {
            area += (w[0].x + w[1].x) * (w[1].y - w[0].y);
        }
    }
    area / 2.0
}

/// Winding number of a point (px, py) with respect to a closed boundary loop.
/// Uses the ray-casting + signed crossing count algorithm.
fn winding_number_boundary(boundary: &[Curve], px: f64, py: f64) -> i32 {
    let mut wn = 0i32;
    for seg in boundary {
        for w in flatten_segment(seg).windows(2) {
            let (x1, y1) = (w[0].x, w[0].y);
            let (x2, y2) = (w[1].x, w[1].y);
            if y1 <= py {
                if y2 > py && cross_sign(x1, y1, x2, y2, px, py) > 0.0 { wn += 1; }
            } else if y2 <= py && cross_sign(x1, y1, x2, y2, px, py) < 0.0 {
                wn -= 1;
            }
        }
    }
    wn
}

/// Orientation sign of point `(px, py)` relative to the directed edge
/// `(x1, y1) → (x2, y2)`: positive if the point lies to the left (CCW), negative to
/// the right, zero if collinear. Uses Shewchuk's adaptive-precision predicate, so
/// the sign is *exact* — the ray-crossing winding count can't misclassify a point
/// that is numerically close to a boundary edge (the classic robustness failure of
/// a naive f64 cross product under catastrophic cancellation).
fn cross_sign(x1: f64, y1: f64, x2: f64, y2: f64, px: f64, py: f64) -> f64 {
    orient2d(
        Coord { x: x1, y: y1 },
        Coord { x: x2, y: y2 },
        Coord { x: px, y: py },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_geometry::{LineSeg, CircularArc, Point2d, Curve};

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

    #[test]
    fn circle_region_area_and_classification() {
        // A region bounded by a single full-circle arc (radius 3, centre origin).
        // The unified tessellation lowers the arc to its exact conic form, so the
        // area converges to π·r² and point classification is correct.
        let r = Region::new(vec![Curve::Arc(CircularArc::new(
            Point2d::from_i64(0, 0), 3.0, 0.0, std::f64::consts::TAU))]);
        let area = r.signed_area_f64();
        let expected = std::f64::consts::PI * 9.0;
        assert!((area - expected).abs() < 1e-2, "circle area ≈ {expected}, got {area}");
        assert!(r.contains_point(0.0, 0.0), "centre is inside");
        assert!(r.contains_point(2.9, 0.0), "just inside the rim");
        assert!(!r.contains_point(3.1, 0.0), "just outside the rim");
        assert!(!r.contains_point(10.0, 10.0), "far point is outside");
    }

    #[test]
    fn rotated_diamond_classification_uses_robust_orientation() {
        // A 45°-rotated square (diamond): every edge is diagonal, so inside/outside is
        // decided purely by the orientation sign of each edge (now Shewchuk-exact via
        // robust::orient2d), not by axis-aligned bounds. Points straddling the x+y=3
        // edge must classify correctly.
        let d = Region::new(vec![
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(3, 0), Point2d::from_i64(0, 3))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(0, 3), Point2d::from_i64(-3, 0))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(-3, 0), Point2d::from_i64(0, -3))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(0, -3), Point2d::from_i64(3, 0))),
        ]);
        assert!(d.contains_point(0.0, 0.0), "centre inside");
        assert!(d.contains_point(1.4, 1.4), "just inside the x+y=3 edge");
        assert!(!d.contains_point(1.6, 1.6), "just outside the x+y=3 edge");
        assert!(!d.contains_point(2.0, 2.0), "corner-diagonal point outside");
    }
}
