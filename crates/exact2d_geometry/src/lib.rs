pub mod point;
pub mod curve;
pub mod primitives;
pub mod ops;
pub mod transform;
pub mod nurbs;

pub use point::{Point2d, BoundingBox};
pub use curve::{Curve, CurveSegment};
pub use primitives::{LineSeg, CircularArc, EllipticalArc, CubicBezier, PolyCurve};
pub use transform::Transform2d;
pub use nurbs::{RationalBezier, NurbsCurve, lower, tessellate_curve, cv_spline_segments};
pub use ops::{
    intersect, CurveIntersection,
    intersect_line_line, intersect_line_circle, intersect_circle_circle,
    point_to_curve_distance, project_point_onto_curve, curve_to_curve_distance,
    ProjectionResult,
    offset_curve,
    tangent_at, normal_at, curvature_at,
    split_curve, reverse_curve,
};
