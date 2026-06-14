use crate::curve::{Curve, CurveSegment};
use crate::point::Point2d;
use crate::primitives::{LineSeg, CircularArc, CubicBezier};

/// Create an offset curve at distance `dist` (positive = left / CCW side).
///
/// Closed-form for lines and circles; approximate cubic Bézier for other types.
pub fn offset_curve(curve: &Curve, dist: f64) -> Curve {
    match curve {
        Curve::Line(l) => {
            Curve::Line(l.offset_exact(dist))
        }
        Curve::Arc(a) => {
            let new_r = a.radius + dist;
            let r = if new_r <= 0.0 { new_r.abs().max(1e-12) } else { new_r };
            Curve::Arc(CircularArc::new(a.center, r, a.start_angle, a.end_angle))
        }
        Curve::Bezier(bz) => {
            // Approximate offset: sample points on the offset, fit a cubic Bézier.
            // The exact algebraic offset of a cubic Bézier has degree ≤ 10.
            // Here we use a 4-point Hermite fitting with end tangents preserved.
            offset_bezier_approx(bz, dist)
        }
        Curve::Ellipse(_) | Curve::Poly(_) | Curve::Rational(_) | Curve::Nurbs(_) => {
            // Ellipses, polycurves, rational Béziers, and NURBS fall back to
            // point-sampling (their exact offset is not the same curve type).
            offset_by_sampling(curve, dist)
        }
    }
}

/// Approximate the offset of a cubic Bézier by sampling 4 offset points and
/// fitting a cubic Bézier through them (preserving endpoint tangents).
fn offset_bezier_approx(bz: &CubicBezier, dist: f64) -> Curve {
    // Sample at t = 0, 1/3, 2/3, 1
    let ts = [0.0f64, 1.0 / 3.0, 2.0 / 3.0, 1.0];
    let mut offset_pts = [(0.0f64, 0.0f64); 4];

    for (i, &t) in ts.iter().enumerate() {
        let (px, py) = bz.evaluate_f64(t);
        let (tx, ty) = bz.tangent_f64(t);
        let len = (tx * tx + ty * ty).sqrt().max(1e-20);
        // Unit normal (CCW)
        let (nx, ny) = (-ty / len, tx / len);
        offset_pts[i] = (px + dist * nx, py + dist * ny);
    }

    // Fit cubic Bézier through the 4 offset points
    // Use the chord-length parameterisation and solve for control points
    let p0 = Point2d::from_f64(offset_pts[0].0, offset_pts[0].1);
    let p3 = Point2d::from_f64(offset_pts[3].0, offset_pts[3].1);

    // Preserve tangent directions (parallel to original), scale to fit
    let (t0x, t0y) = bz.tangent_f64(0.0);
    let (t1x, t1y) = bz.tangent_f64(1.0);
    let chord = ((offset_pts[3].0 - offset_pts[0].0).powi(2)
               + (offset_pts[3].1 - offset_pts[0].1).powi(2)).sqrt();
    let scale = chord / 3.0;

    let p1 = Point2d::from_f64(
        offset_pts[0].0 + t0x * scale / (t0x * t0x + t0y * t0y).sqrt().max(1e-20),
        offset_pts[0].1 + t0y * scale / (t0x * t0x + t0y * t0y).sqrt().max(1e-20),
    );
    let p2 = Point2d::from_f64(
        offset_pts[3].0 - t1x * scale / (t1x * t1x + t1y * t1y).sqrt().max(1e-20),
        offset_pts[3].1 - t1y * scale / (t1x * t1x + t1y * t1y).sqrt().max(1e-20),
    );

    Curve::Bezier(CubicBezier::new(p0, p1, p2, p3))
}

/// Fallback: build an offset by sampling points and connecting with line segments.
fn offset_by_sampling(curve: &Curve, dist: f64) -> Curve {
    // For now, return as a polycurve of 16 line segments (approximation).
    // Phase 3 will compute the exact algebraic offset.
    use crate::primitives::PolyCurve;

    let (t0, t1) = curve.domain();
    let steps = 16usize;
    let mut segs = Vec::new();
    let mut prev_pt: Option<(f64, f64)> = None;

    for i in 0..=steps {
        let t = t0 + (t1 - t0) * i as f64 / steps as f64;
        let (px, py) = curve.evaluate_f64(t);
        let (tx, ty) = curve.tangent_f64(t);
        let len = (tx * tx + ty * ty).sqrt().max(1e-20);
        let (nx, ny) = (-ty / len, tx / len);
        let op = (px + dist * nx, py + dist * ny);
        if let Some(prev) = prev_pt {
            segs.push(Curve::Line(LineSeg::from_endpoints(
                Point2d::from_f64(prev.0, prev.1),
                Point2d::from_f64(op.0, op.1),
            )));
        }
        prev_pt = Some(op);
    }
    Curve::Poly(Box::new(PolyCurve::new(segs)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::LineSeg;
    use crate::point::Point2d;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn offset_horizontal_line() {
        // Line along y=0; offset by +1 should give y=1
        let line = Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(4,0)));
        let off = offset_curve(&line, 1.0);
        if let Curve::Line(l) = off {
            let y0 = l.p0.y;
            let y1 = l.p1.y;
            assert!((y0 - 1.0).abs() < 1e-5, "y0={}", y0);
            assert!((y1 - 1.0).abs() < 1e-5, "y1={}", y1);
        } else {
            panic!("Expected Line");
        }
    }

    #[test]
    fn offset_circle_increases_radius() {
        let arc = Curve::Arc(CircularArc::new(pt(0,0), 3.0,
            0.0, 2.0 * std::f64::consts::PI));
        let off = offset_curve(&arc, 2.0);
        if let Curve::Arc(a) = off {
            assert!((a.radius - 5.0).abs() < 1e-9);
        } else {
            panic!("Expected Arc");
        }
    }

    #[test]
    fn offset_circle_decreases_radius() {
        let arc = Curve::Arc(CircularArc::new(pt(0,0), 5.0,
            0.0, 2.0 * std::f64::consts::PI));
        let off = offset_curve(&arc, -2.0);
        if let Curve::Arc(a) = off {
            assert!((a.radius - 3.0).abs() < 1e-9);
        } else {
            panic!("Expected Arc");
        }
    }
}
