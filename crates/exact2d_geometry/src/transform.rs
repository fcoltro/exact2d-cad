use exact2d_algebra::Rational;
use crate::point::Point2d;
use crate::curve::Curve;
use crate::primitives::{LineSeg, CircularArc, EllipticalArc, CubicBezier, PolyCurve};

/// A 2-D affine transform with exact rational coefficients.
///
/// Maps `(x, y) → (m00·x + m01·y + tx,  m10·x + m11·y + ty)`.
///
/// Translation, scaling, mirroring, and 90°-multiple rotations are fully exact.
/// Rotation by an arbitrary angle uses a rational approximation of cos/sin
/// (consistent with how the rest of the codebase stores arc angles in f64).
#[derive(Clone, Debug, PartialEq)]
pub struct Transform2d {
    pub m00: Rational, pub m01: Rational, pub tx: Rational,
    pub m10: Rational, pub m11: Rational, pub ty: Rational,
}

impl Transform2d {
    pub fn identity() -> Self {
        Transform2d {
            m00: Rational::one(),  m01: Rational::zero(), tx: Rational::zero(),
            m10: Rational::zero(), m11: Rational::one(),  ty: Rational::zero(),
        }
    }

    /// Pure translation by (dx, dy) — exact.
    pub fn translation(dx: Rational, dy: Rational) -> Self {
        let mut t = Self::identity();
        t.tx = dx; t.ty = dy;
        t
    }

    /// Non-uniform scale about the origin — exact.
    pub fn scale(sx: Rational, sy: Rational) -> Self {
        Transform2d {
            m00: sx, m01: Rational::zero(), tx: Rational::zero(),
            m10: Rational::zero(), m11: sy, ty: Rational::zero(),
        }
    }

    /// Uniform scale about the origin — exact.
    pub fn scale_uniform(s: Rational) -> Self {
        Self::scale(s.clone(), s)
    }

    /// Scale about an arbitrary center point — exact.
    pub fn scale_about(center: &Point2d, sx: Rational, sy: Rational) -> Self {
        Self::translation(center.x.clone(), center.y.clone())
            .compose(&Self::scale(sx, sy))
            .compose(&Self::translation(-center.x.clone(), -center.y.clone()))
    }

    /// Mirror across the x-axis (y → −y) — exact.
    pub fn mirror_x() -> Self {
        Self::scale(Rational::one(), Rational::minus_one())
    }

    /// Mirror across the y-axis (x → −x) — exact.
    pub fn mirror_y() -> Self {
        Self::scale(Rational::minus_one(), Rational::one())
    }

    /// Mirror across the line through two points — exact.
    /// Reflection matrix for a line with direction (dx, dy):
    ///   R = 1/(dx²+dy²) · [[dx²−dy², 2·dx·dy], [2·dx·dy, dy²−dx²]]
    pub fn mirror_line(p0: &Point2d, p1: &Point2d) -> Self {
        let dx = p1.x.clone() - p0.x.clone();
        let dy = p1.y.clone() - p0.y.clone();
        let len_sq = dx.clone() * dx.clone() + dy.clone() * dy.clone();
        assert!(!len_sq.is_zero(), "mirror line needs two distinct points");
        let two = Rational::from(2i64);
        let r00 = (dx.clone() * dx.clone() - dy.clone() * dy.clone()) / len_sq.clone();
        let r01 = (two.clone() * dx.clone() * dy.clone()) / len_sq.clone();
        let r11 = (dy.clone() * dy.clone() - dx.clone() * dx.clone()) / len_sq.clone();
        let refl = Transform2d {
            m00: r00, m01: r01.clone(), tx: Rational::zero(),
            m10: r01, m11: r11, ty: Rational::zero(),
        };
        // Conjugate by translation so the mirror passes through p0
        Self::translation(p0.x.clone(), p0.y.clone())
            .compose(&refl)
            .compose(&Self::translation(-p0.x.clone(), -p0.y.clone()))
    }

    /// Rotation by `n` quarter-turns (90° each) about the origin — exact.
    pub fn rotation_quarter_turns(n: i32) -> Self {
        let (c, s) = match n.rem_euclid(4) {
            0 => (1i64, 0i64),
            1 => (0, 1),
            2 => (-1, 0),
            _ => (0, -1),
        };
        Transform2d {
            m00: Rational::from(c), m01: Rational::from(-s), tx: Rational::zero(),
            m10: Rational::from(s), m11: Rational::from(c),  ty: Rational::zero(),
        }
    }

    /// Rotation by an arbitrary angle (radians) about the origin.
    /// cos/sin are converted to rational approximations.
    pub fn rotation(angle: f64) -> Self {
        let c = Rational::from_f64_approx(angle.cos());
        let s = Rational::from_f64_approx(angle.sin());
        Transform2d {
            m00: c.clone(), m01: -s.clone(), tx: Rational::zero(),
            m10: s, m11: c, ty: Rational::zero(),
        }
    }

    /// Rotation about an arbitrary center point.
    pub fn rotation_about(center: &Point2d, angle: f64) -> Self {
        Self::translation(center.x.clone(), center.y.clone())
            .compose(&Self::rotation(angle))
            .compose(&Self::translation(-center.x.clone(), -center.y.clone()))
    }

    /// Compose: `self ∘ other` (apply `other` first, then `self`).
    pub fn compose(&self, other: &Transform2d) -> Transform2d {
        // [self] · [other]  (as 3×3 augmented matrices)
        Transform2d {
            m00: self.m00.clone() * other.m00.clone() + self.m01.clone() * other.m10.clone(),
            m01: self.m00.clone() * other.m01.clone() + self.m01.clone() * other.m11.clone(),
            tx:  self.m00.clone() * other.tx.clone()  + self.m01.clone() * other.ty.clone()  + self.tx.clone(),
            m10: self.m10.clone() * other.m00.clone() + self.m11.clone() * other.m10.clone(),
            m11: self.m10.clone() * other.m01.clone() + self.m11.clone() * other.m11.clone(),
            ty:  self.m10.clone() * other.tx.clone()  + self.m11.clone() * other.ty.clone()  + self.ty.clone(),
        }
    }

    /// Apply to a point — exact.
    pub fn apply_point(&self, p: &Point2d) -> Point2d {
        Point2d {
            x: self.m00.clone() * p.x.clone() + self.m01.clone() * p.y.clone() + self.tx.clone(),
            y: self.m10.clone() * p.x.clone() + self.m11.clone() * p.y.clone() + self.ty.clone(),
        }
    }

    /// Determinant of the linear part. <0 means the transform includes a reflection.
    pub fn determinant(&self) -> Rational {
        self.m00.clone() * self.m11.clone() - self.m01.clone() * self.m10.clone()
    }

    /// Uniform scale factor = sqrt(|det|) (valid for similarity transforms).
    pub fn scale_factor(&self) -> f64 {
        self.determinant().to_f64().abs().sqrt()
    }

    /// The rotation angle the linear part applies to the x-axis (radians).
    pub fn rotation_angle(&self) -> f64 {
        self.m10.to_f64().atan2(self.m00.to_f64())
    }

    /// Whether this transform reflects orientation (det < 0).
    pub fn is_reflection(&self) -> bool {
        self.determinant().is_negative()
    }
}

// ── Applying transforms to curves ─────────────────────────────────────────────

impl Transform2d {
    /// Transform any curve. Lines and Béziers are exact (affine-invariant via
    /// their defining points). Arcs/ellipses are exact for similarity transforms
    /// (translate/rotate/uniform-scale/mirror); non-uniform scale on arcs is
    /// approximated (a circle would become an ellipse — not yet modelled).
    pub fn apply_curve(&self, curve: &Curve) -> Curve {
        match curve {
            Curve::Line(l) => Curve::Line(LineSeg::from_endpoints(
                self.apply_point(&l.p0), self.apply_point(&l.p1),
            )),
            Curve::Bezier(b) => Curve::Bezier(CubicBezier::new(
                self.apply_point(&b.p0), self.apply_point(&b.p1),
                self.apply_point(&b.p2), self.apply_point(&b.p3),
            )),
            Curve::Arc(a) => Curve::Arc(self.apply_arc(a)),
            Curve::Ellipse(e) => Curve::Ellipse(self.apply_ellipse(e)),
            Curve::Poly(pc) => {
                let segs = pc.segments.iter().map(|s| self.apply_curve(s)).collect();
                Curve::Poly(Box::new(PolyCurve::new(segs)))
            }
        }
    }

    fn apply_arc(&self, a: &CircularArc) -> CircularArc {
        let new_center = self.apply_point(&a.center);
        let new_radius = Rational::from_f64_approx(a.radius.to_f64() * self.scale_factor());
        let rot = self.rotation_angle();
        let (start, end) = if self.is_reflection() {
            // Reflection flips angle direction and swaps orientation
            (-a.start_angle + rot, -a.end_angle + rot)
        } else {
            (a.start_angle + rot, a.end_angle + rot)
        };
        CircularArc::new(new_center, new_radius, start, end)
    }

    fn apply_ellipse(&self, e: &EllipticalArc) -> EllipticalArc {
        let new_center = self.apply_point(&e.center);
        let sf = self.scale_factor();
        let new_major = Rational::from_f64_approx(e.semi_major.to_f64() * sf);
        let new_minor = Rational::from_f64_approx(e.semi_minor.to_f64() * sf);
        let rot = self.rotation_angle();
        let new_rotation = e.rotation + rot;
        let (start, end) = if self.is_reflection() {
            (-e.start_angle + rot, -e.end_angle + rot)
        } else {
            (e.start_angle + rot, e.end_angle + rot)
        };
        EllipticalArc::new(new_center, new_major, new_minor, new_rotation, start, end)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::CurveSegment;

    fn r(n: i64) -> Rational { Rational::from(n) }
    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn translate_point_exact() {
        let t = Transform2d::translation(r(3), r(-2));
        assert_eq!(t.apply_point(&pt(5, 5)), pt(8, 3));
    }

    #[test]
    fn scale_about_center() {
        // Scale by 2 about (1,1): point (3,3) → (5,5)
        let t = Transform2d::scale_about(&pt(1, 1), r(2), r(2));
        assert_eq!(t.apply_point(&pt(3, 3)), pt(5, 5));
        // Center is fixed
        assert_eq!(t.apply_point(&pt(1, 1)), pt(1, 1));
    }

    #[test]
    fn quarter_turn_exact() {
        // 90° CCW: (1,0) → (0,1)
        let t = Transform2d::rotation_quarter_turns(1);
        assert_eq!(t.apply_point(&pt(1, 0)), pt(0, 1));
        assert_eq!(t.apply_point(&pt(0, 1)), pt(-1, 0));
    }

    #[test]
    fn mirror_x_axis() {
        let t = Transform2d::mirror_x();
        assert_eq!(t.apply_point(&pt(3, 4)), pt(3, -4));
        assert!(t.is_reflection());
    }

    #[test]
    fn mirror_diagonal_line() {
        // Mirror across y = x (line through (0,0)-(1,1)): (3,0) → (0,3)
        let t = Transform2d::mirror_line(&pt(0, 0), &pt(1, 1));
        assert_eq!(t.apply_point(&pt(3, 0)), pt(0, 3));
    }

    #[test]
    fn compose_translate_then_scale() {
        // scale∘translate: translate (1,1) then scale 2 → (p+1)*2
        let t = Transform2d::scale(r(2), r(2)).compose(&Transform2d::translation(r(1), r(1)));
        assert_eq!(t.apply_point(&pt(2, 3)), pt(6, 8));
    }

    #[test]
    fn bezier_is_affine_invariant() {
        let bz = Curve::Bezier(CubicBezier::new(pt(0,0), pt(1,2), pt(3,2), pt(4,0)));
        let t = Transform2d::translation(r(10), r(5));
        let moved = t.apply_curve(&bz);
        // A point on the original maps to the same point on the transformed curve
        let (x, y) = bz.evaluate_f64(0.5);
        let (mx, my) = moved.evaluate_f64(0.5);
        assert!((mx - (x + 10.0)).abs() < 1e-9 && (my - (y + 5.0)).abs() < 1e-9);
    }

    #[test]
    fn line_transform_endpoints() {
        let l = Curve::Line(LineSeg::from_endpoints(pt(0, 0), pt(2, 0)));
        let t = Transform2d::rotation_quarter_turns(1); // 90°
        if let Curve::Line(moved) = t.apply_curve(&l) {
            assert_eq!(moved.p0, pt(0, 0));
            assert_eq!(moved.p1, pt(0, 2)); // (2,0) rotated 90° → (0,2)
        } else { panic!("expected line"); }
    }

    #[test]
    fn arc_translate_and_scale() {
        // Circle r=2 at origin; scale 3 about origin → r=6 at origin
        let arc = Curve::Arc(CircularArc::new(pt(0,0), r(2), 0.0, std::f64::consts::PI));
        let t = Transform2d::scale_uniform(r(3));
        if let Curve::Arc(a) = t.apply_curve(&arc) {
            assert!((a.radius.to_f64() - 6.0).abs() < 1e-6);
            assert_eq!(a.center, pt(0, 0));
        } else { panic!("expected arc"); }
    }
}
