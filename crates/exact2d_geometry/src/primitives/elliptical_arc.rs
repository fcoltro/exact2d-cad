use exact2d_algebra::{Rational, BivariatePoly};
use crate::point::{Point2d, BoundingBox};
use crate::curve::CurveSegment;

/// An elliptical arc defined by center, semi-axes, rotation, and angular extent.
///
/// For axis-aligned ellipses (rotation = 0) the implicit form has exact rational
/// coefficients.  For rotated ellipses the coefficients are computed in f64 and
/// converted to rational approximations.
///
/// Implicit form: A·x² + B·x·y + C·y² + D·x + E·y + F = 0  with B²−4AC < 0.
#[derive(Clone, Debug)]
pub struct EllipticalArc {
    /// Center.
    pub center: Point2d,
    /// Semi-major axis length (along the rotated x-axis).
    pub semi_major: Rational,
    /// Semi-minor axis length.
    pub semi_minor: Rational,
    /// Rotation angle of the major axis from the positive x-axis (radians).
    pub rotation: f64,
    /// Start and end parametric angles.
    pub start_angle: f64,
    pub end_angle: f64,
}

impl EllipticalArc {
    // ── Constructors ──────────────────────────────────────────────────────────

    pub fn new(
        center: Point2d,
        semi_major: Rational,
        semi_minor: Rational,
        rotation: f64,
        start_angle: f64,
        end_angle: f64,
    ) -> Self {
        EllipticalArc { center, semi_major, semi_minor, rotation, start_angle, end_angle }
    }

    /// Axis-aligned ellipse (rotation = 0) — all coefficients are exact rationals.
    pub fn axis_aligned(
        center: Point2d,
        semi_major: Rational,
        semi_minor: Rational,
        start_angle: f64,
        end_angle: f64,
    ) -> Self {
        EllipticalArc {
            center, semi_major, semi_minor,
            rotation: 0.0, start_angle, end_angle,
        }
    }

    // ── Conic coefficients ─────────────────────────────────────────────────────

    /// Returns (A, B, C, D, E, F) for the conic A·x²+B·xy+C·y²+D·x+E·y+F=0.
    ///
    /// For axis-aligned (rotation≈0) the result is exact rational; otherwise f64.
    #[allow(non_snake_case)] // A,B,C,D,E,F are the standard conic coefficient names
    pub fn conic_coefficients_f64(&self) -> (f64, f64, f64, f64, f64, f64) {
        let a = self.semi_major.to_f64();
        let b = self.semi_minor.to_f64();
        let (cx, cy) = self.center.to_f64();
        let phi = self.rotation;
        let cos_phi = phi.cos();
        let sin_phi = phi.sin();

        // Coefficients of (u/a)² + (v/b)² = 1 after substitution
        // u = cos(φ)(x-cx) + sin(φ)(y-cy)
        // v = -sin(φ)(x-cx) + cos(φ)(y-cy)
        let a2 = a * a;
        let b2 = b * b;

        let A = cos_phi * cos_phi / a2 + sin_phi * sin_phi / b2;
        let B = 2.0 * cos_phi * sin_phi * (1.0 / a2 - 1.0 / b2);
        let C = sin_phi * sin_phi / a2 + cos_phi * cos_phi / b2;
        let D = -2.0 * A * cx - B * cy;
        let E = -B * cx - 2.0 * C * cy;
        let F = A * cx * cx + B * cx * cy + C * cy * cy - 1.0;

        (A, B, C, D, E, F)
    }

    /// Exact conic coefficients for axis-aligned case.
    #[allow(non_snake_case)] // A,B,C,D,E,F are the standard conic coefficient names
    pub fn conic_coefficients_exact(&self) -> Option<(Rational, Rational, Rational, Rational, Rational, Rational)> {
        if self.rotation.abs() > 1e-12 { return None; }
        let cx = self.center.x.clone();
        let cy = self.center.y.clone();
        let a2 = self.semi_major.clone() * self.semi_major.clone();
        let b2 = self.semi_minor.clone() * self.semi_minor.clone();
        // (x-cx)²/a² + (y-cy)²/b² = 1
        // b²(x-cx)² + a²(y-cy)² = a²b²
        // b²x² - 2b²cx·x + b²cx² + a²y² - 2a²cy·y + a²cy² - a²b² = 0
        let A = b2.clone();
        let B = Rational::zero();
        let C = a2.clone();
        let D = -Rational::from(2i64) * b2.clone() * cx.clone();
        let E = -Rational::from(2i64) * a2.clone() * cy.clone();
        let F = b2.clone() * cx.clone() * cx
              + a2.clone() * cy.clone() * cy
              - a2 * b2;
        Some((A, B, C, D, E, F))
    }

    // ── Properties ────────────────────────────────────────────────────────────

    /// Focus points (exact for axis-aligned ellipse).
    pub fn foci(&self) -> ((f64, f64), (f64, f64)) {
        let a = self.semi_major.to_f64();
        let b = self.semi_minor.to_f64();
        let c = (a * a - b * b).sqrt();
        let (cx, cy) = self.center.to_f64();
        let phi = self.rotation;
        let f1 = (cx + c * phi.cos(), cy + c * phi.sin());
        let f2 = (cx - c * phi.cos(), cy - c * phi.sin());
        (f1, f2)
    }

    /// Eccentricity e = c/a.
    pub fn eccentricity(&self) -> f64 {
        let a = self.semi_major.to_f64();
        let b = self.semi_minor.to_f64();
        let c = (a * a - b * b).max(0.0).sqrt();
        c / a
    }

    pub fn included_angle(&self) -> f64 {
        let mut a = self.end_angle - self.start_angle;
        while a <= 0.0 { a += 2.0 * std::f64::consts::PI; }
        a
    }
}

// ── CurveSegment impl ─────────────────────────────────────────────────────────

impl CurveSegment for EllipticalArc {
    #[allow(non_snake_case)] // A,B,C,D,E,F are the standard conic coefficient names
    fn implicit_form(&self) -> BivariatePoly {
        if let Some((A, B, C, D, E, F)) = self.conic_coefficients_exact() {
            BivariatePoly::from_terms(&[
                ((2, 0), A),
                ((1, 1), B),
                ((0, 2), C),
                ((1, 0), D),
                ((0, 1), E),
                ((0, 0), F),
            ])
        } else {
            // Rotated ellipse: f64 approximation
            let (A, B, C, D, E, F) = self.conic_coefficients_f64();
            BivariatePoly::from_terms(&[
                ((2, 0), Rational::from_f64_approx(A)),
                ((1, 1), Rational::from_f64_approx(B)),
                ((0, 2), Rational::from_f64_approx(C)),
                ((1, 0), Rational::from_f64_approx(D)),
                ((0, 1), Rational::from_f64_approx(E)),
                ((0, 0), Rational::from_f64_approx(F)),
            ])
        }
    }

    fn domain(&self) -> (f64, f64) { (self.start_angle, self.end_angle) }

    fn evaluate_f64(&self, t: f64) -> (f64, f64) {
        let (cx, cy) = self.center.to_f64();
        let a = self.semi_major.to_f64();
        let b = self.semi_minor.to_f64();
        let phi = self.rotation;
        let u = a * t.cos();
        let v = b * t.sin();
        let x = cx + u * phi.cos() - v * phi.sin();
        let y = cy + u * phi.sin() + v * phi.cos();
        (x, y)
    }

    fn bounding_box(&self) -> BoundingBox {
        let steps = 64usize;
        let (t0, t1) = (self.start_angle, self.start_angle + self.included_angle());
        let mut xmin = f64::INFINITY;
        let mut xmax = f64::NEG_INFINITY;
        let mut ymin = f64::INFINITY;
        let mut ymax = f64::NEG_INFINITY;
        for i in 0..=steps {
            let t = t0 + (t1 - t0) * i as f64 / steps as f64;
            let (x, y) = self.evaluate_f64(t);
            xmin = xmin.min(x); xmax = xmax.max(x);
            ymin = ymin.min(y); ymax = ymax.max(y);
        }
        BoundingBox::from_corners(xmin, ymin, xmax, ymax)
    }

    fn tangent_f64(&self, t: f64) -> (f64, f64) {
        let a = self.semi_major.to_f64();
        let b = self.semi_minor.to_f64();
        let phi = self.rotation;
        let du = -a * t.sin();
        let dv =  b * t.cos();
        let dx = du * phi.cos() - dv * phi.sin();
        let dy = du * phi.sin() + dv * phi.cos();
        (dx, dy)
    }

    fn arc_length(&self) -> f64 {
        // Numerical integration: Ramanujan's approximation for full ellipse,
        // or Gaussian quadrature for the arc portion.
        let steps = 128usize;
        let (t0, t1) = (self.start_angle, self.start_angle + self.included_angle());
        let dt = (t1 - t0) / steps as f64;
        let mut length = 0.0;
        for i in 0..steps {
            let t = t0 + dt * (i as f64 + 0.5);
            let (dx, dy) = self.tangent_f64(t);
            length += (dx * dx + dy * dy).sqrt() * dt;
        }
        length
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(n: i64) -> Rational { Rational::from(n) }

    #[test]
    fn axis_aligned_implicit_exact() {
        // (x/3)²+(y/4)²=1 centered at origin
        let ell = EllipticalArc::axis_aligned(
            Point2d::from_i64(0, 0), r(3), r(4),
            0.0, 2.0 * std::f64::consts::PI,
        );
        let f = ell.implicit_form();
        // Points on the ellipse: (3,0), (-3,0), (0,4), (0,-4)
        assert!(f.eval_rational(&r(3), &r(0)).is_zero(), "f(3,0)={}", f.eval_rational(&r(3),&r(0)));
        assert!(f.eval_rational(&r(-3), &r(0)).is_zero());
        assert!(f.eval_rational(&r(0), &r(4)).is_zero());
        assert!(f.eval_rational(&r(0), &r(-4)).is_zero());
        // Off ellipse
        assert!(!f.eval_rational(&r(1), &r(1)).is_zero());
    }

    #[test]
    fn foci_axis_aligned() {
        // Ellipse a=5, b=4: c=3, foci at (±3, 0)
        let ell = EllipticalArc::axis_aligned(
            Point2d::from_i64(0, 0), r(5), r(4),
            0.0, 2.0 * std::f64::consts::PI,
        );
        let ((f1x, f1y), (f2x, f2y)) = ell.foci();
        assert!((f1x.abs() - 3.0).abs() < 1e-8);
        assert!(f1y.abs() < 1e-8);
        assert!((f2x.abs() - 3.0).abs() < 1e-8);
        assert!(f2y.abs() < 1e-8);
    }

    #[test]
    fn eccentricity() {
        // Circle: a=b=5, e=0
        let circle = EllipticalArc::axis_aligned(
            Point2d::from_i64(0, 0), r(5), r(5),
            0.0, 2.0 * std::f64::consts::PI,
        );
        assert!(circle.eccentricity().abs() < 1e-10);
    }
}
