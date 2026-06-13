use crate::point::Point2d;
use crate::curve::Curve;
use crate::primitives::{LineSeg, CircularArc, EllipticalArc, CubicBezier, PolyCurve};
use crate::nurbs::RationalBezier;

/// A 2-D affine transform with f64 coefficients.
///
/// Maps `(x, y) → (m00·x + m01·y + tx,  m10·x + m11·y + ty)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform2d {
    pub m00: f64, pub m01: f64, pub tx: f64,
    pub m10: f64, pub m11: f64, pub ty: f64,
}

impl Transform2d {
    pub fn identity() -> Self {
        Transform2d {
            m00: 1.0, m01: 0.0, tx: 0.0,
            m10: 0.0, m11: 1.0, ty: 0.0,
        }
    }

    /// Pure translation by (dx, dy).
    pub fn translation(dx: f64, dy: f64) -> Self {
        let mut t = Self::identity();
        t.tx = dx; t.ty = dy;
        t
    }

    /// Non-uniform scale about the origin.
    pub fn scale(sx: f64, sy: f64) -> Self {
        Transform2d {
            m00: sx, m01: 0.0, tx: 0.0,
            m10: 0.0, m11: sy, ty: 0.0,
        }
    }

    /// Uniform scale about the origin.
    pub fn scale_uniform(s: f64) -> Self {
        Self::scale(s, s)
    }

    /// Scale about an arbitrary center point.
    pub fn scale_about(center: &Point2d, sx: f64, sy: f64) -> Self {
        Self::translation(center.x, center.y)
            .compose(&Self::scale(sx, sy))
            .compose(&Self::translation(-center.x, -center.y))
    }

    /// Mirror across the x-axis (y → −y).
    pub fn mirror_x() -> Self {
        Self::scale(1.0, -1.0)
    }

    /// Mirror across the line through two points.
    /// Reflection matrix for a line with direction (dx, dy):
    ///   R = 1/(dx²+dy²) · [[dx²−dy², 2·dx·dy], [2·dx·dy, dy²−dx²]]
    pub fn mirror_line(p0: &Point2d, p1: &Point2d) -> Self {
        let dx = p1.x - p0.x;
        let dy = p1.y - p0.y;
        let len_sq = dx * dx + dy * dy;
        assert!(len_sq != 0.0, "mirror line needs two distinct points");
        let r00 = (dx * dx - dy * dy) / len_sq;
        let r01 = (2.0 * dx * dy) / len_sq;
        let r11 = (dy * dy - dx * dx) / len_sq;
        let refl = Transform2d {
            m00: r00, m01: r01, tx: 0.0,
            m10: r01, m11: r11, ty: 0.0,
        };
        // Conjugate by translation so the mirror passes through p0
        Self::translation(p0.x, p0.y)
            .compose(&refl)
            .compose(&Self::translation(-p0.x, -p0.y))
    }

    /// Rotation by `n` quarter-turns (90° each) about the origin — exact.
    pub fn rotation_quarter_turns(n: i32) -> Self {
        let (c, s) = match n.rem_euclid(4) {
            0 => (1.0, 0.0),
            1 => (0.0, 1.0),
            2 => (-1.0, 0.0),
            _ => (0.0, -1.0),
        };
        Transform2d {
            m00: c, m01: -s, tx: 0.0,
            m10: s, m11: c,  ty: 0.0,
        }
    }

    /// Rotation by an arbitrary angle (radians) about the origin.
    pub fn rotation(angle: f64) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Transform2d {
            m00: c, m01: -s, tx: 0.0,
            m10: s, m11: c, ty: 0.0,
        }
    }

    /// Rotation about an arbitrary center point.
    pub fn rotation_about(center: &Point2d, angle: f64) -> Self {
        Self::translation(center.x, center.y)
            .compose(&Self::rotation(angle))
            .compose(&Self::translation(-center.x, -center.y))
    }

    /// Compose: `self ∘ other` (apply `other` first, then `self`).
    pub fn compose(&self, other: &Transform2d) -> Transform2d {
        Transform2d {
            m00: self.m00 * other.m00 + self.m01 * other.m10,
            m01: self.m00 * other.m01 + self.m01 * other.m11,
            tx:  self.m00 * other.tx  + self.m01 * other.ty  + self.tx,
            m10: self.m10 * other.m00 + self.m11 * other.m10,
            m11: self.m10 * other.m01 + self.m11 * other.m11,
            ty:  self.m10 * other.tx  + self.m11 * other.ty  + self.ty,
        }
    }

    /// Apply to a point.
    pub fn apply_point(&self, p: &Point2d) -> Point2d {
        Point2d {
            x: self.m00 * p.x + self.m01 * p.y + self.tx,
            y: self.m10 * p.x + self.m11 * p.y + self.ty,
        }
    }

    /// Determinant of the linear part. <0 means the transform includes a reflection.
    pub fn determinant(&self) -> f64 {
        self.m00 * self.m11 - self.m01 * self.m10
    }

    /// Uniform scale factor = sqrt(|det|) (valid for similarity transforms).
    pub fn scale_factor(&self) -> f64 {
        self.determinant().abs().sqrt()
    }

    /// The rotation angle the linear part applies to the x-axis (radians).
    pub fn rotation_angle(&self) -> f64 {
        self.m10.atan2(self.m00)
    }

    /// Whether this transform reflects orientation (det < 0).
    pub fn is_reflection(&self) -> bool {
        self.determinant() < 0.0
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
            // Rational Béziers are affine-invariant: transform the control points,
            // keep the weights (the homogeneous denominator normalizes the affine part).
            Curve::Rational(rb) => {
                let points = rb.points.iter().map(|p| self.apply_point(p)).collect();
                Curve::Rational(RationalBezier::new(points, rb.weights.clone()))
            }
        }
    }

    fn apply_arc(&self, a: &CircularArc) -> CircularArc {
        let new_center = self.apply_point(&a.center);
        let new_radius = a.radius * self.scale_factor();
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
        let new_major = e.semi_major * sf;
        let new_minor = e.semi_minor * sf;
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

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn translate_point_exact() {
        let t = Transform2d::translation(3.0, -2.0);
        assert_eq!(t.apply_point(&pt(5, 5)), pt(8, 3));
    }

    #[test]
    fn scale_about_center() {
        // Scale by 2 about (1,1): point (3,3) → (5,5)
        let t = Transform2d::scale_about(&pt(1, 1), 2.0, 2.0);
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
        let t = Transform2d::scale(2.0, 2.0).compose(&Transform2d::translation(1.0, 1.0));
        assert_eq!(t.apply_point(&pt(2, 3)), pt(6, 8));
    }

    #[test]
    fn bezier_is_affine_invariant() {
        let bz = Curve::Bezier(CubicBezier::new(pt(0,0), pt(1,2), pt(3,2), pt(4,0)));
        let t = Transform2d::translation(10.0, 5.0);
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
        let arc = Curve::Arc(CircularArc::new(pt(0,0), 2.0, 0.0, std::f64::consts::PI));
        let t = Transform2d::scale_uniform(3.0);
        if let Curve::Arc(a) = t.apply_curve(&arc) {
            assert!((a.radius - 6.0).abs() < 1e-6);
            assert_eq!(a.center, pt(0, 0));
        } else { panic!("expected arc"); }
    }
}
