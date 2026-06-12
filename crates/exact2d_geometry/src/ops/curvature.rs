use exact2d_algebra::BivariatePoly;
use crate::curve::{Curve, CurveSegment};

/// Tangent direction (unnormalized) at parameter t.
pub fn tangent_at(curve: &Curve, t: f64) -> (f64, f64) {
    curve.tangent_f64(t)
}

/// Normal direction (90° CCW from tangent, unnormalized).
pub fn normal_at(curve: &Curve, t: f64) -> (f64, f64) {
    let (tx, ty) = curve.tangent_f64(t);
    (-ty, tx)
}

/// Signed curvature κ at a world point (px, py) on an implicit curve f(x,y)=0.
///
/// Formula (Differential Geometry):
///   κ = −(f_xx·f_y² − 2·f_xy·f_x·f_y + f_yy·f_x²) / (f_x² + f_y²)^{3/2}
///
/// Returns `None` if the gradient is (near) zero at the point.
pub fn curvature_at_point(f: &BivariatePoly, px: f64, py: f64) -> Option<f64> {
    let fx  = f.partial_x().eval_f64(px, py);
    let fy  = f.partial_y().eval_f64(px, py);
    let fxx = f.partial_x().partial_x().eval_f64(px, py);
    let fxy = f.partial_x().partial_y().eval_f64(px, py);
    let fyy = f.partial_y().partial_y().eval_f64(px, py);

    let grad_sq = fx * fx + fy * fy;
    if grad_sq < 1e-20 { return None; }

    let numerator = fxx * fy * fy - 2.0 * fxy * fx * fy + fyy * fx * fx;
    Some(-numerator / grad_sq.powf(1.5))
}

/// Signed curvature κ at parameter t on a curve.
/// Uses the implicit form for accuracy.
pub fn curvature_at(curve: &Curve, t: f64) -> Option<f64> {
    let (px, py) = curve.evaluate_f64(t);
    let f = curve.implicit_form();
    curvature_at_point(&f, px, py)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{LineSeg, CircularArc};
    use crate::point::Point2d;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn curvature_of_circle_is_1_over_r() {
        // Circle radius r: κ = 1/r everywhere
        let r_val = 3.0;
        let arc = CircularArc::new(
            pt(0,0), 3.0,
            0.0, 2.0 * std::f64::consts::PI,
        );
        let c = Curve::Arc(arc);
        let kappa = curvature_at(&c, 0.0).unwrap();
        // Convention: CCW circle → κ = +1/r
        assert!((kappa.abs() - 1.0 / r_val).abs() < 1e-6,
            "κ={}, expected ±{}", kappa, 1.0 / r_val);
    }

    #[test]
    fn curvature_of_line_is_zero() {
        let line = Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(4,0)));
        let kappa = curvature_at(&line, 0.5).unwrap();
        assert!(kappa.abs() < 1e-6, "κ={}", kappa);
    }

    #[test]
    fn tangent_perpendicular_to_normal() {
        let arc = Curve::Arc(CircularArc::new(
            pt(0,0), 2.0,
            0.0, 2.0 * std::f64::consts::PI,
        ));
        let (tx, ty) = tangent_at(&arc, 0.0);
        let (nx, ny) = normal_at(&arc, 0.0);
        let dot = tx * nx + ty * ny;
        assert!(dot.abs() < 1e-10, "dot={}", dot);
    }
}
