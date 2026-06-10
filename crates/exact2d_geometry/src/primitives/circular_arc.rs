use exact2d_algebra::{Rational, BivariatePoly};
use crate::point::{Point2d, BoundingBox};
use crate::curve::CurveSegment;

/// A circular arc.
///
/// The **implicit form** is exact: `(x−cx)² + (y−cy)² − r² = 0`.
/// The **parametric form** `(cx + r·cos(t), cy + r·sin(t))` involves transcendental
/// functions, so evaluation is done in f64.  Angular domain: t ∈ [start_angle, end_angle].
#[derive(Clone, Debug)]
pub struct CircularArc {
    /// Center (exact rational).
    pub center: Point2d,
    /// Radius (exact rational — must be positive).
    pub radius: Rational,
    /// Start angle in radians (CCW from positive x-axis).
    pub start_angle: f64,
    /// End angle in radians.
    pub end_angle: f64,
}

impl CircularArc {
    // ── Constructors ──────────────────────────────────────────────────────────

    /// Direct construction from center, radius, and angles.
    pub fn new(center: Point2d, radius: Rational, start_angle: f64, end_angle: f64) -> Self {
        assert!(radius.is_positive(), "Radius must be positive");
        CircularArc { center, radius, start_angle, end_angle }
    }

    /// Circumscribed circle through three distinct points.
    /// Returns `None` if the three points are collinear.
    pub fn from_three_points(p1: &Point2d, p2: &Point2d, p3: &Point2d) -> Option<Self> {
        // Solve for center: intersection of perpendicular bisectors of P1P2 and P2P3.
        // Perpendicular bisector of P1P2: passes through midpoint M12 with normal = (P2-P1).
        // In equation form (exact Rationals):
        //   (x2-x1)*(x - (x1+x2)/2) + (y2-y1)*(y - (y1+y2)/2) = 0
        //   => (x2-x1)*x + (y2-y1)*y = ((x2-x1)*(x1+x2) + (y2-y1)*(y1+y2)) / 2
        let ax = p2.x.clone() - p1.x.clone();
        let ay = p2.y.clone() - p1.y.clone();
        let bx = p3.x.clone() - p2.x.clone();
        let by = p3.y.clone() - p2.y.clone();

        // RHS for each bisector
        let r1 = (ax.clone() * (p1.x.clone() + p2.x.clone())
                + ay.clone() * (p1.y.clone() + p2.y.clone()))
               / Rational::from(2i64);
        let r2 = (bx.clone() * (p2.x.clone() + p3.x.clone())
                + by.clone() * (p2.y.clone() + p3.y.clone()))
               / Rational::from(2i64);

        // Solve 2×2 linear system: [ax ay; bx by] * [cx; cy] = [r1; r2]
        let det = ax.clone() * by.clone() - ay.clone() * bx.clone();
        if det.is_zero() { return None; } // collinear

        let cx = (r1.clone() * by.clone() - r2.clone() * ay.clone()) / det.clone();
        let cy = (ax.clone() * r2.clone() - bx.clone() * r1.clone()) / det;

        let center = Point2d { x: cx, y: cy };
        let r_sq = center.dist_sq(p1);
        let radius = Rational::from_f64_approx(r_sq.to_f64().sqrt());

        // Compute angles at the three points.
        let angle_of = |p: &Point2d| {
            (p.y.to_f64() - center.y.to_f64()).atan2(p.x.to_f64() - center.x.to_f64())
        };
        let a1 = angle_of(p1);
        let a2 = angle_of(p2);
        let a3 = angle_of(p3);

        // A `CircularArc` always sweeps CCW with `end_angle > start_angle`, and
        // consumers interpolate linearly start→end. Choose `start` ∈ {a1, a3} so the
        // CCW arc passes through the middle point p2 (the defining requirement of a
        // 3-point arc), keeping end > start by lifting it by 2π as needed.
        let pi2 = 2.0 * std::f64::consts::PI;
        let lift = |start: f64, mut end: f64| { while end <= start { end += pi2; } end };
        let on_arc = |start: f64, end: f64, mut a: f64| { while a < start { a += pi2; } a <= end + 1e-12 };

        let (start_angle, end_angle) = {
            // Candidate 1: arc p1 → p3 (CCW). Candidate 2: arc p3 → p1 (CCW).
            let e1 = lift(a1, a3);
            if on_arc(a1, e1, a2) {
                (a1, e1)
            } else {
                (a3, lift(a3, a1))
            }
        };

        Some(CircularArc { center, radius, start_angle, end_angle })
    }

    // ── Properties ────────────────────────────────────────────────────────────

    /// Start point.
    pub fn start_point(&self) -> (f64, f64) {
        self.evaluate_f64(self.start_angle)
    }

    /// End point.
    pub fn end_point(&self) -> (f64, f64) {
        self.evaluate_f64(self.end_angle)
    }

    /// Included angle (positive, CCW).
    pub fn included_angle(&self) -> f64 {
        let mut a = self.end_angle - self.start_angle;
        // Normalize to (0, 2π]
        while a <= 0.0 { a += 2.0 * std::f64::consts::PI; }
        a
    }

    /// Sagitta: the height from the chord to the arc midpoint.
    pub fn sagitta(&self) -> f64 {
        let r = self.radius.to_f64();
        r - r * (self.included_angle() / 2.0).cos()
    }

}

// ── CurveSegment impl ─────────────────────────────────────────────────────────

impl CurveSegment for CircularArc {
    fn implicit_form(&self) -> BivariatePoly {
        // (x - cx)² + (y - cy)² - r² = 0
        // = x² - 2cx·x + cx² + y² - 2cy·y + cy² - r² = 0
        let cx = self.center.x.clone();
        let cy = self.center.y.clone();
        let r2 = self.radius.clone() * self.radius.clone();

        let const_term = cx.clone() * cx.clone()
            + cy.clone() * cy.clone()
            - r2;
        let minus_two = Rational::from(-2i64);

        BivariatePoly::from_terms(&[
            ((2, 0), Rational::one()),
            ((0, 2), Rational::one()),
            ((1, 0), minus_two.clone() * cx),
            ((0, 1), minus_two * cy),
            ((0, 0), const_term),
        ])
    }

    fn domain(&self) -> (f64, f64) { (self.start_angle, self.end_angle) }

    fn evaluate_f64(&self, t: f64) -> (f64, f64) {
        let (cx, cy) = self.center.to_f64();
        let r = self.radius.to_f64();
        (cx + r * t.cos(), cy + r * t.sin())
    }

    fn bounding_box(&self) -> BoundingBox {
        let _r = self.radius.to_f64();
        let (_cx, _cy) = self.center.to_f64();
        let (sx, sy) = self.start_point();
        let (ex, ey) = self.end_point();

        let mut xmin = sx.min(ex);
        let mut xmax = sx.max(ex);
        let mut ymin = sy.min(ey);
        let mut ymax = sy.max(ey);

        // Expand to include axis-crossing extrema within the arc
        let mut a = self.start_angle;
        let end = self.start_angle + self.included_angle();
        while a < end {
            let (x, y) = self.evaluate_f64(a);
            xmin = xmin.min(x); xmax = xmax.max(x);
            ymin = ymin.min(y); ymax = ymax.max(y);
            a += std::f64::consts::FRAC_PI_2;
        }
        // Also check exact axis-crossing angles
        for k in 0..4 {
            let angle = k as f64 * std::f64::consts::FRAC_PI_2;
            // Normalise angle into [start, start + included]
            let mut rel = angle - self.start_angle;
            while rel < 0.0 { rel += 2.0 * std::f64::consts::PI; }
            if rel <= self.included_angle() + 1e-12 {
                let (x, y) = self.evaluate_f64(self.start_angle + rel);
                xmin = xmin.min(x); xmax = xmax.max(x);
                ymin = ymin.min(y); ymax = ymax.max(y);
            }
        }

        BoundingBox::from_corners(xmin, ymin, xmax, ymax)
    }

    fn tangent_f64(&self, t: f64) -> (f64, f64) {
        let r = self.radius.to_f64();
        (-r * t.sin(), r * t.cos())
    }

    fn arc_length(&self) -> f64 {
        self.radius.to_f64() * self.included_angle()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(n: i64) -> Rational { Rational::from(n) }
    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn implicit_unit_circle_at_origin() {
        let arc = CircularArc::new(
            pt(0, 0), r(1),
            0.0, std::f64::consts::PI,
        );
        let f = arc.implicit_form();
        // Points on unit circle: (1,0), (0,1), (-1,0)
        assert!(f.eval_rational(&r(1), &r(0)).is_zero());
        assert!(f.eval_rational(&r(0), &r(1)).is_zero());
        assert!(f.eval_rational(&r(-1), &r(0)).is_zero());
        // Off circle
        assert!(!f.eval_rational(&r(1), &r(1)).is_zero());
    }

    #[test]
    fn implicit_shifted_circle() {
        // Circle (x-3)²+(y-4)²=25: center (3,4), r=5
        let arc = CircularArc::new(
            pt(3, 4), r(5),
            0.0, 2.0 * std::f64::consts::PI,
        );
        let f = arc.implicit_form();
        // (8,4) is on it: (8-3)²+(4-4)²=25 ✓
        assert!(f.eval_rational(&r(8), &r(4)).is_zero());
        // (3,9) is on it: (0)²+(5)²=25 ✓
        assert!(f.eval_rational(&r(3), &r(9)).is_zero());
    }

    #[test]
    fn three_point_construction() {
        // Three points on circle (x-1)²+(y-2)²=9, r=3
        let p1 = Point2d::from_f64(4.0, 2.0); // (3+1, 2)
        let p2 = Point2d::from_f64(1.0, 5.0); // (1, 3+2)
        let p3 = Point2d::from_f64(-2.0, 2.0); // (-3+1, 2)
        let arc = CircularArc::from_three_points(&p1, &p2, &p3).unwrap();

        let (cx, cy) = arc.center.to_f64();
        assert!((cx - 1.0).abs() < 1e-6, "cx={}", cx);
        assert!((cy - 2.0).abs() < 1e-6, "cy={}", cy);
        assert!((arc.radius.to_f64() - 3.0).abs() < 1e-4, "r={}", arc.radius.to_f64());
    }

    #[test]
    fn arc_length_quarter_circle() {
        let arc = CircularArc::new(
            Point2d::from_i64(0, 0), r(5),
            0.0, std::f64::consts::FRAC_PI_2,
        );
        let expected = 5.0 * std::f64::consts::FRAC_PI_2;
        assert!((arc.arc_length() - expected).abs() < 1e-10);
    }

    #[test]
    fn sagitta_semicircle() {
        // Sagitta of a semicircle = radius
        let arc = CircularArc::new(
            Point2d::from_i64(0, 0), r(4),
            0.0, std::f64::consts::PI,
        );
        assert!((arc.sagitta() - 4.0).abs() < 1e-10);
    }
}
