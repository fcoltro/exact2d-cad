use exact2d_algebra::{Rational, BivariatePoly, UnivariatePoly};
use crate::point::{Point2d, BoundingBox};
use crate::curve::CurveSegment;

/// Cubic Bézier curve defined by four control points P₀…P₃, t ∈ [0, 1].
///
/// Parametric: B(t) = (1−t)³P₀ + 3(1−t)²tP₁ + 3(1−t)t²P₂ + t³P₃
/// Implicit:   computed on demand via `BivariatePoly::implicitize`
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezier {
    pub p0: Point2d,
    pub p1: Point2d,
    pub p2: Point2d,
    pub p3: Point2d,
}

impl CubicBezier {
    // ── Constructors ──────────────────────────────────────────────────────────

    pub fn new(p0: Point2d, p1: Point2d, p2: Point2d, p3: Point2d) -> Self {
        CubicBezier { p0, p1, p2, p3 }
    }

    // ── Bernstein polynomial form (exact Rational) ─────────────────────────────

    /// x(t) as a `UnivariatePoly` in t.
    pub fn x_poly(&self) -> UnivariatePoly {
        let (x0, x1, x2, x3) = (
            self.p0.x.clone(), self.p1.x.clone(),
            self.p2.x.clone(), self.p3.x.clone(),
        );
        // B(t) = x0 + (-3x0+3x1)t + (3x0-6x1+3x2)t² + (-x0+3x1-3x2+x3)t³
        let r3 = Rational::from(3i64);
        let r6 = Rational::from(6i64);
        UnivariatePoly::from_coeffs(vec![
            x0.clone(),
            -r3.clone() * x0.clone() + r3.clone() * x1.clone(),
            r3.clone() * x0.clone() - r6.clone() * x1.clone() + r3.clone() * x2.clone(),
            -x0 + r3.clone() * x1 - r3 * x2 + x3,
        ])
    }

    /// y(t) as a `UnivariatePoly` in t.
    pub fn y_poly(&self) -> UnivariatePoly {
        let (y0, y1, y2, y3) = (
            self.p0.y.clone(), self.p1.y.clone(),
            self.p2.y.clone(), self.p3.y.clone(),
        );
        let r3 = Rational::from(3i64);
        let r6 = Rational::from(6i64);
        UnivariatePoly::from_coeffs(vec![
            y0.clone(),
            -r3.clone() * y0.clone() + r3.clone() * y1.clone(),
            r3.clone() * y0.clone() - r6.clone() * y1.clone() + r3.clone() * y2.clone(),
            -y0 + r3.clone() * y1 - r3 * y2 + y3,
        ])
    }

    /// Exact evaluation at parameter t ∈ [0,1] using the polynomial form.
    pub fn evaluate_exact(&self, t: &Rational) -> Point2d {
        Point2d {
            x: self.x_poly().eval(t),
            y: self.y_poly().eval(t),
        }
    }

    // ── de Casteljau subdivision ──────────────────────────────────────────────

    /// Split into two sub-curves at exact parameter t ∈ (0,1).
    /// Returns (left_curve [0..t], right_curve [t..1]).
    pub fn split_at_exact(&self, t: &Rational) -> (CubicBezier, CubicBezier) {
        let lerp = |a: &Point2d, b: &Point2d| a.lerp(b, t);

        // Level 1
        let q0 = lerp(&self.p0, &self.p1);
        let q1 = lerp(&self.p1, &self.p2);
        let q2 = lerp(&self.p2, &self.p3);
        // Level 2
        let r0 = lerp(&q0, &q1);
        let r1 = lerp(&q1, &q2);
        // Level 3 — the split point
        let s  = lerp(&r0, &r1);

        (
            CubicBezier { p0: self.p0.clone(), p1: q0, p2: r0, p3: s.clone() },
            CubicBezier { p0: s, p1: r1, p2: q2, p3: self.p3.clone() },
        )
    }

    // ── Degree elevation ──────────────────────────────────────────────────────

    /// Elevate from degree 3 to degree 4.
    /// Returns five control points Q₀…Q₄ such that the resulting quartic Bézier
    /// is geometrically identical.
    /// Formula: Qᵢ = i/4 · P_{i-1} + (1 - i/4) · Pᵢ  (for i = 0..4)
    pub fn degree_elevate(&self) -> [Point2d; 5] {
        // Q0 = P0
        // Q1 = 1/4*P0 + 3/4*P1
        // Q2 = 2/4*P1 + 2/4*P2 = 1/2*(P1+P2)
        // Q3 = 3/4*P2 + 1/4*P3
        // Q4 = P3
        let r1 = Rational::from(1i64);
        let r3 = Rational::from(3i64);
        let r4 = Rational::from(4i64);
        let frac14 = r1.clone() / r4.clone();
        let frac34 = r3.clone() / r4.clone();
        let frac12 = r1.clone() / Rational::from(2i64);

        let q0 = self.p0.clone();
        let q1 = self.p0.lerp(&self.p1, &frac34);
        let q2 = self.p1.lerp(&self.p2, &frac12);
        let q3 = self.p2.lerp(&self.p3, &frac14);
        let q4 = self.p3.clone();
        [q0, q1, q2, q3, q4]
    }

    // ── Properties ────────────────────────────────────────────────────────────

    /// Convex hull of the four control points (just returns the points; the hull is their
    /// bounding box for axis-aligned rendering, exact polygon in general).
    pub fn convex_hull_points(&self) -> [&Point2d; 4] {
        [&self.p0, &self.p1, &self.p2, &self.p3]
    }

    /// Inflection points: parameters where signed curvature changes sign.
    /// Returned as a (possibly empty) Vec of f64 parameters in (0, 1).
    pub fn inflection_points(&self) -> Vec<f64> {
        // The inflection condition: B'(t) × B''(t) = 0 (cross product in 2D = determinant)
        // Let x'=dx/dt, y'=dy/dt, x''=d²x/dt², y''=d²y/dt²
        // Inflection: x' * y'' - y' * x'' = 0  (a cubic polynomial in t in general)
        let xp = self.x_poly().derivative();
        let yp = self.y_poly().derivative();
        let xpp = xp.derivative();
        let ypp = yp.derivative();
        // cross = xp * ypp - yp * xpp
        let cross = xp.clone() * ypp.clone() - yp.clone() * xpp.clone();
        cross.real_roots_f64(1e-10)
            .into_iter()
            .filter(|&t| t > 1e-10 && t < 1.0 - 1e-10)
            .collect()
    }

    /// Signed curvature at float parameter t.
    pub fn curvature_at_f64(&self, t: f64) -> f64 {
        let (xp, yp)   = self.tangent_f64(t);
        let (xpp, ypp) = self.second_derivative_f64(t);
        let speed_sq = xp * xp + yp * yp;
        let cross = xp * ypp - yp * xpp;
        cross / speed_sq.powf(1.5)
    }

    fn second_derivative_f64(&self, t: f64) -> (f64, f64) {
        let xpp = self.x_poly().derivative().derivative();
        let ypp = self.y_poly().derivative().derivative();
        (xpp.eval_f64(t), ypp.eval_f64(t))
    }
}

// ── CurveSegment impl ─────────────────────────────────────────────────────────

impl CurveSegment for CubicBezier {
    fn implicit_form(&self) -> BivariatePoly {
        BivariatePoly::implicitize(&self.x_poly(), &self.y_poly())
    }

    fn domain(&self) -> (f64, f64) { (0.0, 1.0) }

    fn evaluate_f64(&self, t: f64) -> (f64, f64) {
        let (x0, y0) = self.p0.to_f64();
        let (x1, y1) = self.p1.to_f64();
        let (x2, y2) = self.p2.to_f64();
        let (x3, y3) = self.p3.to_f64();

        let mt = 1.0 - t;
        let mt2 = mt * mt;
        let mt3 = mt2 * mt;

        let t2 = t * t;
        let t3 = t2 * t;

        let x = mt3 * x0 + 3.0 * mt2 * t * x1 + 3.0 * mt * t2 * x2 + t3 * x3;
        let y = mt3 * y0 + 3.0 * mt2 * t * y1 + 3.0 * mt * t2 * y2 + t3 * y3;
        (x, y)
    }

    fn bounding_box(&self) -> BoundingBox {
        // Start with convex hull bounding box (conservative), then refine with extrema.
        let pts: Vec<(f64, f64)> = self.convex_hull_points()
            .iter().map(|p| p.to_f64()).collect();

        let mut xmin = pts.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let mut xmax = pts.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
        let mut ymin = pts.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let mut ymax = pts.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);

        // Add derivative roots (where dx/dt = 0 or dy/dt = 0)
        let xp = self.x_poly().derivative();
        let yp = self.y_poly().derivative();
        for t in xp.real_roots_f64(1e-10) {
            if t > 0.0 && t < 1.0 {
                let (x, _) = self.evaluate_f64(t);
                xmin = xmin.min(x); xmax = xmax.max(x);
            }
        }
        for t in yp.real_roots_f64(1e-10) {
            if t > 0.0 && t < 1.0 {
                let (_, y) = self.evaluate_f64(t);
                ymin = ymin.min(y); ymax = ymax.max(y);
            }
        }
        BoundingBox::from_corners(xmin, ymin, xmax, ymax)
    }

    fn tangent_f64(&self, t: f64) -> (f64, f64) {
        let (x0, y0) = self.p0.to_f64();
        let (x1, y1) = self.p1.to_f64();
        let (x2, y2) = self.p2.to_f64();
        let (x3, y3) = self.p3.to_f64();

        let mt = 1.0 - t;
        let mt2 = mt * mt;
        let t2 = t * t;

        let x = 3.0 * mt2 * (x1 - x0) + 6.0 * mt * t * (x2 - x1) + 3.0 * t2 * (x3 - x2);
        let y = 3.0 * mt2 * (y1 - y0) + 6.0 * mt * t * (y2 - y1) + 3.0 * t2 * (y3 - y2);
        (x, y)
    }

    fn arc_length(&self) -> f64 {
        // 5-point Gauss-Legendre quadrature on [0,1]
        const NODES: [f64; 5] = [0.046910077, 0.230765346, 0.5, 0.769234654, 0.953089923];
        const WEIGHTS: [f64; 5] = [0.118463442, 0.239314335, 0.284444444, 0.239314335, 0.118463442];
        NODES.iter().zip(WEIGHTS.iter()).fold(0.0, |acc, (&t, &w)| {
            let (dx, dy) = self.tangent_f64(t);
            acc + w * (dx * dx + dy * dy).sqrt()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn evaluate_endpoints() {
        let bz = CubicBezier::new(pt(0, 0), pt(1, 2), pt(3, 4), pt(4, 0));
        let p0 = bz.evaluate_exact(&Rational::zero());
        let p1 = bz.evaluate_exact(&Rational::one());
        assert_eq!(p0, pt(0, 0));
        assert_eq!(p1, pt(4, 0));
    }

    #[test]
    fn split_reconstructs_original() {
        let bz = CubicBezier::new(pt(0, 0), pt(1, 3), pt(3, 3), pt(4, 0));
        let t_half = Rational::new(
            exact2d_integer::Integer::one(),
            exact2d_integer::Integer::from(2i64),
        );
        let (left, right) = bz.split_at_exact(&t_half);
        // Endpoints must match
        assert_eq!(left.p0, bz.p0);
        assert_eq!(right.p3, bz.p3);
        // Join point must be on the original curve
        let mid_orig = bz.evaluate_exact(&t_half);
        assert_eq!(left.p3, mid_orig.clone());
        assert_eq!(right.p0, mid_orig);
    }

    #[test]
    fn degree_elevate_preserves_shape() {
        let bz = CubicBezier::new(pt(0, 0), pt(1, 2), pt(3, 2), pt(4, 0));
        let elevated = bz.degree_elevate();
        // The endpoints must not change
        assert_eq!(elevated[0], bz.p0);
        assert_eq!(elevated[4], bz.p3);
        // Sample a few interior points and verify they match
        for &t_val in &[0.25f64, 0.5, 0.75] {
            let orig = bz.evaluate_exact(&Rational::from_f64_approx(t_val));
            // Elevated should give same point (to float precision)
            let (ox, oy) = orig.to_f64();
            let (ex, ey) = bz.evaluate_f64(t_val);
            assert!((ox - ex).abs() < 1e-10, "x mismatch at t={}", t_val);
            assert!((oy - ey).abs() < 1e-10, "y mismatch at t={}", t_val);
        }
    }

    #[test]
    fn implicit_form_on_curve() {
        let bz = CubicBezier::new(pt(0, 0), pt(1, 2), pt(3, 2), pt(4, 0));
        let f = bz.implicit_form();
        // Every sampled point on the curve should satisfy f ≈ 0
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            let (x, y) = bz.evaluate_f64(t);
            let val = f.eval_f64(x, y).abs();
            assert!(val < 1.0, "t={} f={:.6}", t, val);
        }
    }

    #[test]
    fn bounding_box_contains_all_points() {
        let bz = CubicBezier::new(pt(0, 0), pt(2, 4), pt(3, 4), pt(5, 0));
        let bb = bz.bounding_box();
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            let (x, y) = bz.evaluate_f64(t);
            assert!(bb.contains_point_f64(x, y),
                "t={}: ({},{}) outside {:?}", t, x, y, bb);
        }
    }

    #[test]
    fn arc_length_straight_line() {
        // Control points on a straight horizontal line from (0,0) to (4,0)
        let bz = CubicBezier::new(
            Point2d::from_i64(0, 0),
            Point2d::new(Rational::new(exact2d_integer::Integer::from(4i64), exact2d_integer::Integer::from(3i64)), Rational::zero()),
            Point2d::new(Rational::new(exact2d_integer::Integer::from(8i64), exact2d_integer::Integer::from(3i64)), Rational::zero()),
            Point2d::from_i64(4, 0),
        );
        // A Bézier on a straight line has length = chord length = 4
        assert!((bz.arc_length() - 4.0).abs() < 1e-5, "length={}", bz.arc_length());
    }
}
