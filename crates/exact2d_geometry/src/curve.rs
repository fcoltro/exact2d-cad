use exact2d_algebra::BivariatePoly;
use crate::point::BoundingBox;
use crate::primitives::{LineSeg, CircularArc, EllipticalArc, CubicBezier, PolyCurve};

// ── CurveSegment trait ────────────────────────────────────────────────────────

/// Common interface for all curve primitives.
///
/// Representations:
/// - `implicit_form()` — the exact algebraic equation f(x,y)=0 (Rational coefficients)
/// - `evaluate_f64(t)` — fast float evaluation along the parameter domain
///
/// The split between exact (implicit) and numerical (evaluate/ops) is intentional:
/// Phase 3's GPU renderer will evaluate polynomials directly; geometric operations
/// in Phase 2 use f64 for speed and fall back to exact arithmetic where necessary.
pub trait CurveSegment {
    /// Exact algebraic equation of the underlying infinite curve.
    fn implicit_form(&self) -> BivariatePoly;

    /// Parameter domain `[t_min, t_max]` as floats.
    fn domain(&self) -> (f64, f64);

    /// Evaluate the curve at parameter t (float).
    fn evaluate_f64(&self, t: f64) -> (f64, f64);

    /// Axis-aligned bounding box (conservative).
    fn bounding_box(&self) -> BoundingBox;

    /// Tangent direction (unnormalized) at parameter t.
    fn tangent_f64(&self, t: f64) -> (f64, f64);

    /// Normal direction (unnormalized, 90° CCW from tangent) at parameter t.
    fn normal_f64(&self, t: f64) -> (f64, f64) {
        let (tx, ty) = self.tangent_f64(t);
        (-ty, tx)
    }

    /// Total arc length (float).
    fn arc_length(&self) -> f64;

    /// Check whether (px, py) is approximately on the curve.
    fn contains_point_f64(&self, px: f64, py: f64, tol: f64) -> bool {
        let imp = self.implicit_form();
        imp.eval_f64(px, py).abs() < tol && self.bounding_box().contains_point_f64(px, py)
    }
}

// ── Curve enum ────────────────────────────────────────────────────────────────

/// A single curve segment — the central type of the geometry layer.
/// Wraps all primitive types under a common enumeration.
///
/// Variants are kept inline (not boxed) by design: `Curve` is matched and passed
/// by value on hot paths (rendering, intersection), so the conventional geometry-
/// kernel choice is to avoid the per-access heap indirection that boxing adds.
/// `PolyCurve` is the one exception — it is recursive, so it must be boxed.
#[derive(Clone, Debug)]
#[allow(clippy::large_enum_variant)]
pub enum Curve {
    Line(LineSeg),
    Arc(CircularArc),
    Ellipse(EllipticalArc),
    Bezier(CubicBezier),
    Poly(Box<PolyCurve>),
}

impl Curve {
    pub fn as_line(&self)    -> Option<&LineSeg>       { if let Curve::Line(v)    = self { Some(v) } else { None } }
}

/// Dispatch CurveSegment to each variant.
impl CurveSegment for Curve {
    fn implicit_form(&self) -> BivariatePoly {
        match self {
            Curve::Line(v)    => v.implicit_form(),
            Curve::Arc(v)     => v.implicit_form(),
            Curve::Ellipse(v) => v.implicit_form(),
            Curve::Bezier(v)  => v.implicit_form(),
            Curve::Poly(v)    => v.implicit_form(),
        }
    }
    fn domain(&self) -> (f64, f64) {
        match self {
            Curve::Line(v)    => v.domain(),
            Curve::Arc(v)     => v.domain(),
            Curve::Ellipse(v) => v.domain(),
            Curve::Bezier(v)  => v.domain(),
            Curve::Poly(v)    => v.domain(),
        }
    }
    fn evaluate_f64(&self, t: f64) -> (f64, f64) {
        match self {
            Curve::Line(v)    => v.evaluate_f64(t),
            Curve::Arc(v)     => v.evaluate_f64(t),
            Curve::Ellipse(v) => v.evaluate_f64(t),
            Curve::Bezier(v)  => v.evaluate_f64(t),
            Curve::Poly(v)    => v.evaluate_f64(t),
        }
    }
    fn bounding_box(&self) -> BoundingBox {
        match self {
            Curve::Line(v)    => v.bounding_box(),
            Curve::Arc(v)     => v.bounding_box(),
            Curve::Ellipse(v) => v.bounding_box(),
            Curve::Bezier(v)  => v.bounding_box(),
            Curve::Poly(v)    => v.bounding_box(),
        }
    }
    fn tangent_f64(&self, t: f64) -> (f64, f64) {
        match self {
            Curve::Line(v)    => v.tangent_f64(t),
            Curve::Arc(v)     => v.tangent_f64(t),
            Curve::Ellipse(v) => v.tangent_f64(t),
            Curve::Bezier(v)  => v.tangent_f64(t),
            Curve::Poly(v)    => v.tangent_f64(t),
        }
    }
    fn arc_length(&self) -> f64 {
        match self {
            Curve::Line(v)    => v.arc_length(),
            Curve::Arc(v)     => v.arc_length(),
            Curve::Ellipse(v) => v.arc_length(),
            Curve::Bezier(v)  => v.arc_length(),
            Curve::Poly(v)    => v.arc_length(),
        }
    }
}
