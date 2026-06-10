use exact2d_algebra::{Rational, BivariatePoly};
use crate::point::{Point2d, BoundingBox};
use crate::curve::CurveSegment;

/// A directed line segment from `p0` to `p1` with parameter t ∈ [0, 1].
///
/// Implicit: (y0−y1)·x + (x1−x0)·y + (x0·y1 − x1·y0) = 0
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

    // ── Implicit form ─────────────────────────────────────────────────────────

    /// Returns the coefficients (a, b, c) for ax + by + c = 0.
    /// a = y0 - y1,  b = x1 - x0,  c = x0*y1 - x1*y0
    pub fn implicit_coefficients(&self) -> (Rational, Rational, Rational) {
        let a = self.p0.y.clone() - self.p1.y.clone();
        let b = self.p1.x.clone() - self.p0.x.clone();
        let c = self.p0.x.clone() * self.p1.y.clone()
              - self.p1.x.clone() * self.p0.y.clone();
        (a, b, c)
    }

    // ── Properties (exact) ────────────────────────────────────────────────────

    pub fn direction(&self) -> (Rational, Rational) {
        (self.p1.x.clone() - self.p0.x.clone(), self.p1.y.clone() - self.p0.y.clone())
    }

    pub fn midpoint(&self) -> Point2d {
        self.p0.midpoint(&self.p1)
    }

    /// Squared length (exact rational).
    pub fn length_sq(&self) -> Rational {
        self.p0.dist_sq(&self.p1)
    }

    /// Float length — |p1 - p0|, may be irrational.
    pub fn length_f64(&self) -> f64 {
        self.length_sq().to_f64().sqrt()
    }

    /// Tangent direction (unnormalized, exact): (dx, dy) = p1 - p0.
    pub fn tangent_exact(&self) -> (Rational, Rational) {
        self.direction()
    }

    /// Normal direction (unnormalized, exact): perpendicular to tangent (CCW).
    pub fn normal_exact(&self) -> (Rational, Rational) {
        let (dx, dy) = self.direction();
        (-dy, dx)
    }

    /// Evaluate at exact Rational parameter t ∈ [0, 1].
    pub fn evaluate_exact(&self, t: &Rational) -> Point2d {
        self.p0.lerp(&self.p1, t)
    }

    /// Split into two sub-segments at parameter t ∈ (0, 1).
    pub fn split_at_exact(&self, t: &Rational) -> (LineSeg, LineSeg) {
        let mid = self.evaluate_exact(t);
        (
            LineSeg { p0: self.p0.clone(), p1: mid.clone() },
            LineSeg { p0: mid,              p1: self.p1.clone() },
        )
    }

    pub fn reverse(&self) -> LineSeg {
        LineSeg { p0: self.p1.clone(), p1: self.p0.clone() }
    }

    /// Exact offset: returns a parallel line segment displaced by `dist` perpendicular.
    /// Positive dist = left side (CCW from direction).
    pub fn offset_exact(&self, dist: &Rational) -> LineSeg {
        let (nx, ny) = self.normal_exact();
        // Normalise: scale n by dist / |n|. Since |n| = |direction|, which may be irrational,
        // we compute the normalisation factor as dist / sqrt(length_sq).
        // For exact output, we just scale by dist/length_sq * direction length — but that's
        // irrational. Return a version scaled by dist/(length²) for symbolic use, or use f64.
        let len_sq = self.length_sq();
        // Offset by dist * n_hat: n_hat = (nx, ny) / |n| = (nx, ny) / sqrt(len_sq).
        // We compute the displacement in f64 and convert back to Rational for the points.
        let scale = dist.to_f64() / len_sq.to_f64().sqrt();
        let ox = Rational::from_f64_approx(nx.to_f64() * scale);
        let oy = Rational::from_f64_approx(ny.to_f64() * scale);
        LineSeg {
            p0: Point2d { x: self.p0.x.clone() + ox.clone(), y: self.p0.y.clone() + oy.clone() },
            p1: Point2d { x: self.p1.x.clone() + ox,         y: self.p1.y.clone() + oy },
        }
    }
}

// ── CurveSegment impl ─────────────────────────────────────────────────────────

impl CurveSegment for LineSeg {
    fn implicit_form(&self) -> BivariatePoly {
        let (a, b, c) = self.implicit_coefficients();
        BivariatePoly::from_terms(&[
            ((1, 0), a),
            ((0, 1), b),
            ((0, 0), c),
        ])
    }

    fn domain(&self) -> (f64, f64) { (0.0, 1.0) }

    fn evaluate_f64(&self, t: f64) -> (f64, f64) {
        let (x0, y0) = self.p0.to_f64();
        let (x1, y1) = self.p1.to_f64();
        (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
    }

    fn bounding_box(&self) -> BoundingBox {
        
        let (xmin, xmax) = if self.p0.x <= self.p1.x {
            (self.p0.x.clone(), self.p1.x.clone())
        } else {
            (self.p1.x.clone(), self.p0.x.clone())
        };
        let (ymin, ymax) = if self.p0.y <= self.p1.y {
            (self.p0.y.clone(), self.p1.y.clone())
        } else {
            (self.p1.y.clone(), self.p0.y.clone())
        };
        BoundingBox {
            min: Point2d { x: xmin, y: ymin },
            max: Point2d { x: xmax, y: ymax },
        }
    }

    fn tangent_f64(&self, _t: f64) -> (f64, f64) {
        let (dx, dy) = self.direction();
        (dx.to_f64(), dy.to_f64())
    }

    fn arc_length(&self) -> f64 { self.length_f64() }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(n: i64) -> Rational { Rational::from(n) }
    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn implicit_form_horizontal() {
        // y = 2 horizontal segment from (0,2) to (5,2)
        let seg = LineSeg::from_endpoints(pt(0, 2), pt(5, 2));
        let (a, b, c) = seg.implicit_coefficients();
        // a = y0 - y1 = 0, b = x1 - x0 = 5, c = x0*y1 - x1*y0 = 0*2 - 5*2 = -10
        // => 0*x + 5*y - 10 = 0  =>  y = 2  ✓
        assert_eq!(a, r(0));
        assert_eq!(b, r(5));
        assert_eq!(c, r(-10));
        let f = seg.implicit_form();
        // Should evaluate to 0 at both endpoints
        assert!(f.eval_rational(&r(0), &r(2)).is_zero());
        assert!(f.eval_rational(&r(5), &r(2)).is_zero());
        // Should be non-zero off the line
        assert!(!f.eval_rational(&r(3), &r(3)).is_zero());
    }

    #[test]
    fn implicit_form_diagonal() {
        // y = x  diagonal: (0,0) → (3,3)
        let seg = LineSeg::from_endpoints(pt(0, 0), pt(3, 3));
        let f = seg.implicit_form();
        // Every point on y=x should satisfy f=0
        for v in [0i64, 1, 2, 3] {
            assert!(f.eval_rational(&r(v), &r(v)).is_zero());
        }
        assert!(!f.eval_rational(&r(1), &r(2)).is_zero());
    }

    #[test]
    fn midpoint_and_split() {
        let seg = LineSeg::from_endpoints(pt(0, 0), pt(4, 6));
        let m = seg.midpoint();
        assert_eq!(m, Point2d::new(r(2), r(3)));

        let t_half = Rational::new(exact2d_integer::Integer::one(), exact2d_integer::Integer::from(2i64));
        let (left, right) = seg.split_at_exact(&t_half);
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
        assert!(dot.is_zero());
    }

    #[test]
    fn arc_length() {
        // 3-4-5 right triangle: length = 5
        let seg = LineSeg::from_endpoints(pt(0, 0), pt(3, 4));
        assert!((seg.arc_length() - 5.0).abs() < 1e-10);
    }
}
