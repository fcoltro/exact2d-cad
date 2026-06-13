use crate::point::{Point2d, BoundingBox};
use crate::curve::CurveSegment;

/// A directed line segment from `p0` to `p1` with parameter t ∈ [0, 1].
#[derive(Clone, Debug, PartialEq)]
pub struct LineSeg {
    pub p0: Point2d,
    pub p1: Point2d,
}

impl LineSeg {
    // ── Constructors ──────────────────────────────────────────────────────────

    /// From two endpoints.
    pub fn from_endpoints(p0: Point2d, p1: Point2d) -> Self {
        LineSeg { p0, p1 }
    }

    // ── Properties ────────────────────────────────────────────────────────────

    /// Direction vector (dx, dy) = p1 - p0.
    pub fn direction(&self) -> (f64, f64) {
        (self.p1.x - self.p0.x, self.p1.y - self.p0.y)
    }

    pub fn midpoint(&self) -> Point2d {
        self.p0.midpoint(&self.p1)
    }

    /// Squared length.
    pub fn length_sq(&self) -> f64 {
        self.p0.dist_sq(&self.p1)
    }

    /// Length — |p1 - p0|.
    pub fn length_f64(&self) -> f64 {
        self.length_sq().sqrt()
    }

    /// Tangent direction (unnormalized): (dx, dy) = p1 - p0.
    pub fn tangent_exact(&self) -> (f64, f64) {
        self.direction()
    }

    /// Normal direction (unnormalized): perpendicular to tangent (CCW).
    pub fn normal_exact(&self) -> (f64, f64) {
        let (dx, dy) = self.direction();
        (-dy, dx)
    }

    /// Evaluate at parameter t ∈ [0, 1].
    pub fn evaluate_exact(&self, t: f64) -> Point2d {
        self.p0.lerp(&self.p1, t)
    }

    /// Split into two sub-segments at parameter t ∈ (0, 1).
    pub fn split_at_exact(&self, t: f64) -> (LineSeg, LineSeg) {
        let mid = self.evaluate_exact(t);
        (
            LineSeg { p0: self.p0, p1: mid },
            LineSeg { p0: mid,     p1: self.p1 },
        )
    }

    pub fn reverse(&self) -> LineSeg {
        LineSeg { p0: self.p1, p1: self.p0 }
    }

    /// Offset: a parallel line segment displaced by `dist` perpendicular.
    /// Positive dist = left side (CCW from direction).
    pub fn offset_exact(&self, dist: f64) -> LineSeg {
        let (nx, ny) = self.normal_exact();
        let len = self.length_f64();
        let scale = if len > 0.0 { dist / len } else { 0.0 };
        let (ox, oy) = (nx * scale, ny * scale);
        LineSeg {
            p0: Point2d { x: self.p0.x + ox, y: self.p0.y + oy },
            p1: Point2d { x: self.p1.x + ox, y: self.p1.y + oy },
        }
    }
}

// ── CurveSegment impl ─────────────────────────────────────────────────────────

impl CurveSegment for LineSeg {
    fn domain(&self) -> (f64, f64) { (0.0, 1.0) }

    fn evaluate_f64(&self, t: f64) -> (f64, f64) {
        let (x0, y0) = self.p0.to_f64();
        let (x1, y1) = self.p1.to_f64();
        (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
    }

    fn bounding_box(&self) -> BoundingBox {
        
        let (xmin, xmax) = if self.p0.x <= self.p1.x {
            (self.p0.x, self.p1.x)
        } else {
            (self.p1.x, self.p0.x)
        };
        let (ymin, ymax) = if self.p0.y <= self.p1.y {
            (self.p0.y, self.p1.y)
        } else {
            (self.p1.y, self.p0.y)
        };
        BoundingBox {
            min: Point2d { x: xmin, y: ymin },
            max: Point2d { x: xmax, y: ymax },
        }
    }

    fn tangent_f64(&self, _t: f64) -> (f64, f64) {
        self.direction()
    }

    fn arc_length(&self) -> f64 { self.length_f64() }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn midpoint_and_split() {
        let seg = LineSeg::from_endpoints(pt(0, 0), pt(4, 6));
        let m = seg.midpoint();
        assert_eq!(m, Point2d::new(2.0, 3.0));

        let (left, right) = seg.split_at_exact(0.5);
        assert_eq!(left.p1, m.clone());
        assert_eq!(right.p0, m);
        assert_eq!(left.p0, pt(0, 0));
        assert_eq!(right.p1, pt(4, 6));
    }

    #[test]
    fn normal_perpendicular_to_tangent() {
        let seg = LineSeg::from_endpoints(pt(0, 0), pt(3, 4));
        let (tx, ty) = seg.tangent_exact();
        let (nx, ny) = seg.normal_exact();
        // Dot product must be zero
        let dot = tx * nx + ty * ny;
        assert!(dot.abs() < 1e-12);
    }

    #[test]
    fn arc_length() {
        // 3-4-5 right triangle: length = 5
        let seg = LineSeg::from_endpoints(pt(0, 0), pt(3, 4));
        assert!((seg.arc_length() - 5.0).abs() < 1e-10);
    }
}
