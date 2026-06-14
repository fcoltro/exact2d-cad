use crate::point::BoundingBox;
use crate::primitives::{LineSeg, CircularArc, EllipticalArc, CubicBezier, PolyCurve};
use crate::nurbs::{RationalBezier, NurbsCurve};

// ── CurveSegment trait ────────────────────────────────────────────────────────

/// Common f64 interface for all curve primitives — evaluation, tangents, bounds,
/// and length over the parameter domain. (Geometry is f64 + tolerance; the former
/// exact `implicit_form()` representation was removed in the f64 migration.)
pub trait CurveSegment {
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
    /// A weighted rational Bézier — a first-class authored NURBS/spline segment
    /// (and the kernel's unified "lowered" form). Parameter `t ∈ [0,1]`.
    Rational(RationalBezier),
    /// A clamped cubic NURBS / control-vertex spline (control vertices + weights) —
    /// the authored, editable spline. Decomposes to rational Béziers on demand.
    Nurbs(NurbsCurve),
}

impl Curve {
    pub fn as_line(&self)    -> Option<&LineSeg>       { if let Curve::Line(v)    = self { Some(v) } else { None } }
}

/// Dispatch CurveSegment to each variant.
impl CurveSegment for Curve {
    fn domain(&self) -> (f64, f64) {
        match self {
            Curve::Line(v)     => v.domain(),
            Curve::Arc(v)      => v.domain(),
            Curve::Ellipse(v)  => v.domain(),
            Curve::Bezier(v)   => v.domain(),
            Curve::Poly(v)     => v.domain(),
            Curve::Rational(v) => v.domain(),
            Curve::Nurbs(v)    => v.domain(),
        }
    }
    fn evaluate_f64(&self, t: f64) -> (f64, f64) {
        match self {
            Curve::Line(v)     => v.evaluate_f64(t),
            Curve::Arc(v)      => v.evaluate_f64(t),
            Curve::Ellipse(v)  => v.evaluate_f64(t),
            Curve::Bezier(v)   => v.evaluate_f64(t),
            Curve::Poly(v)     => v.evaluate_f64(t),
            Curve::Rational(v) => v.evaluate_f64(t),
            Curve::Nurbs(v)    => v.evaluate_f64(t),
        }
    }
    fn bounding_box(&self) -> BoundingBox {
        match self {
            Curve::Line(v)     => v.bounding_box(),
            Curve::Arc(v)      => v.bounding_box(),
            Curve::Ellipse(v)  => v.bounding_box(),
            Curve::Bezier(v)   => v.bounding_box(),
            Curve::Poly(v)     => v.bounding_box(),
            Curve::Rational(v) => v.bounding_box(),
            Curve::Nurbs(v)    => v.bounding_box(),
        }
    }
    fn tangent_f64(&self, t: f64) -> (f64, f64) {
        match self {
            Curve::Line(v)     => v.tangent_f64(t),
            Curve::Arc(v)      => v.tangent_f64(t),
            Curve::Ellipse(v)  => v.tangent_f64(t),
            Curve::Bezier(v)   => v.tangent_f64(t),
            Curve::Poly(v)     => v.tangent_f64(t),
            Curve::Rational(v) => v.tangent_f64(t),
            Curve::Nurbs(v)    => v.tangent_f64(t),
        }
    }
    fn arc_length(&self) -> f64 {
        match self {
            Curve::Line(v)     => v.arc_length(),
            Curve::Arc(v)      => v.arc_length(),
            Curve::Ellipse(v)  => v.arc_length(),
            Curve::Bezier(v)   => v.arc_length(),
            Curve::Poly(v)     => v.arc_length(),
            Curve::Rational(v) => v.arc_length(),
            Curve::Nurbs(v)    => v.arc_length(),
        }
    }
}
