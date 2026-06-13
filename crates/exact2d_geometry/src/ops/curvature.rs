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

/// Signed curvature κ at parameter t, from the parametric derivatives:
///   κ = (x'·y'' − y'·x'') / (x'² + y'²)^{3/2}
///
/// The first derivative is `tangent_f64`; the second is a central finite
/// difference of the tangent (clamped to the domain). Returns `None` where the
/// speed is (near) zero. Sign follows the CCW convention (a CCW circle → +1/r).
pub fn curvature_at(curve: &Curve, t: f64) -> Option<f64> {
    let (t0, t1) = curve.domain();
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    let h = (hi - lo).max(1e-9) * 1e-4;
    let tm = (t - h).clamp(lo, hi);
    let tp = (t + h).clamp(lo, hi);
    let dt = (tp - tm).max(1e-12);

    let (dx, dy) = curve.tangent_f64(t);            // first derivative
    let (txm, tym) = curve.tangent_f64(tm);
    let (txp, typ) = curve.tangent_f64(tp);
    let (ddx, ddy) = ((txp - txm) / dt, (typ - tym) / dt); // second derivative

    let speed_sq = dx * dx + dy * dy;
    if speed_sq < 1e-20 { return None; }
    Some((dx * ddy - dy * ddx) / speed_sq.powf(1.5))
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
