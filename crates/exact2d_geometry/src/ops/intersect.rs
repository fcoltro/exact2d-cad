use exact2d_algebra::Rational;
use crate::curve::{Curve, CurveSegment};
use crate::primitives::{LineSeg, CircularArc};

/// Result of a curve-curve intersection.
#[derive(Clone, Debug)]
pub struct CurveIntersection {
    /// Approximate world coordinates.
    pub point: (f64, f64),
    /// Parameter on the first curve.
    pub t1: f64,
    /// Parameter on the second curve.
    pub t2: f64,
}

// ── Fast-path specializations ─────────────────────────────────────────────────

/// Line–line intersection (exact rational arithmetic).
/// Returns `None` if the lines are parallel.
pub fn intersect_line_line(l1: &LineSeg, l2: &LineSeg) -> Option<CurveIntersection> {
    let (a1, b1, c1) = l1.implicit_coefficients();
    let (a2, b2, c2) = l2.implicit_coefficients();
    let det = a1.clone() * b2.clone() - a2.clone() * b1.clone();
    if det.is_zero() { return None; }
    let x = (b1.clone() * c2.clone() - b2.clone() * c1.clone()) / det.clone();
    let y = (a2.clone() * c1.clone() - a1.clone() * c2.clone()) / det;

    // Find parameters t1, t2 on each segment
    let t1 = param_on_line(l1, &x, &y);
    let t2 = param_on_line(l2, &x, &y);

    if (-1e-10..=1.0 + 1e-10).contains(&t1) && (-1e-10..=1.0 + 1e-10).contains(&t2) {
        Some(CurveIntersection { point: (x.to_f64(), y.to_f64()), t1, t2 })
    } else {
        None
    }
}

fn param_on_line(l: &LineSeg, x: &Rational, y: &Rational) -> f64 {
    let dx = l.p1.x.clone() - l.p0.x.clone();
    let dy = l.p1.y.clone() - l.p0.y.clone();
    let dx_sq = dx.clone() * dx.clone();
    let dy_sq = dy.clone() * dy.clone();
    let len_sq = dx_sq + dy_sq;
    if len_sq.is_zero() { return 0.0; }
    let nx = x.clone() - l.p0.x.clone();
    let ny = y.clone() - l.p0.y.clone();
    (nx * dx.clone() + ny * dy.clone()).to_f64() / len_sq.to_f64()
}

/// Line–circle intersection (quadratic formula, exact rational discriminant).
pub fn intersect_line_circle(line: &LineSeg, arc: &CircularArc) -> Vec<CurveIntersection> {
    let (a, b, c) = line.implicit_coefficients();
    let cx = arc.center.x.clone();
    let cy = arc.center.y.clone();
    let r2 = arc.radius.clone() * arc.radius.clone();

    // Substitute line into circle and solve the resulting quadratic.
    // Case 1: b ≠ 0 → y = -(ax + c)/b, substitute into circle.
    // Case 2: b = 0 → x = -c/a (vertical line).
    let mut results = Vec::new();

    let intersections: Vec<(Rational, Rational)> = if !b.is_zero() {
        // y = -(a/b)x - c/b
        // (x-cx)² + (-(a/b)x - c/b - cy)² = r²
        let ab = a.clone() / b.clone();           // a/b
        let cb = c.clone() / b.clone();            // c/b
        // let u = -(a/b)x - c/b - cy = -ab*x - (cb + cy)
        let k = cb.clone() + cy.clone();            // offset constant
        // (x-cx)² + (ab*x + k)² = r²
        // x² - 2cx*x + cx² + ab²*x² + 2ab*k*x + k² - r² = 0
        // (1 + ab²)*x² + (-2cx + 2ab*k)*x + (cx² + k² - r²) = 0
        let one = Rational::one();
        let quad_a = one.clone() + ab.clone() * ab.clone();
        let quad_b = -Rational::from(2i64) * cx.clone()
                   + Rational::from(2i64) * ab.clone() * k.clone();
        let quad_c = cx.clone() * cx.clone() + k.clone() * k.clone() - r2.clone();

        let disc = quad_b.clone() * quad_b.clone()
                 - Rational::from(4i64) * quad_a.clone() * quad_c.clone();
        // Classify the discriminant by its EXACT sign, not a float round of it: a
        // tiny-but-nonzero rational near a tangency must never be mistaken for zero
        // or flipped negative. `disc.is_zero()` is therefore an exact tangency.
        if disc.is_negative() { vec![] }
        else {
            let sqrt_disc = disc.to_f64().sqrt();
            let qa_f = quad_a.to_f64();
            let qb_f = quad_b.to_f64();
            // Exact tangency emits the single touch point, not two coincident hits.
            let signs: &[f64] = if disc.is_zero() { &[0.0] } else { &[-1.0, 1.0] };
            let mut pts = Vec::new();
            for &sign in signs {
                let xv = (-qb_f + sign * sqrt_disc) / (2.0 * qa_f);
                let yv = -ab.to_f64() * xv - cb.to_f64();
                pts.push((Rational::from_f64_approx(xv), Rational::from_f64_approx(yv)));
            }
            pts
        }
    } else {
        // Vertical line: x = -c/a
        let xv = -c.clone() / a.clone();
        let dx = xv.clone() - cx.clone();
        let rem = r2.clone() - dx.clone() * dx;
        // Exact sign classification (see the quad branch). A tangent vertical line
        // touches at exactly (xv, cy) — an exact point, not a float pair.
        if rem.is_negative() { vec![] }
        else if rem.is_zero() {
            vec![(xv.clone(), cy.clone())]
        } else {
            let sqrt_rem = rem.to_f64().sqrt();
            let cy_f = cy.to_f64();
            vec![
                (xv.clone(), Rational::from_f64_approx(cy_f - sqrt_rem)),
                (xv.clone(), Rational::from_f64_approx(cy_f + sqrt_rem)),
            ]
        }
    };

    for (xv, yv) in intersections {
        let t1 = param_on_line(line, &xv, &yv);
        // Angle on circle
        let angle = {
            let dx = xv.to_f64() - arc.center.x.to_f64();
            let dy = yv.to_f64() - arc.center.y.to_f64();
            dy.atan2(dx)
        };
        // Check domain restrictions
        let in_segment = (-1e-9..=1.0 + 1e-9).contains(&t1);
        let in_arc = angle_in_arc(angle, arc.start_angle, arc.end_angle);
        if in_segment && in_arc {
            results.push(CurveIntersection {
                point: (xv.to_f64(), yv.to_f64()),
                t1,
                t2: angle_on_domain(angle, arc.start_angle, arc.end_angle),
            });
        }
    }
    results
}

fn angle_in_arc(angle: f64, start: f64, end: f64) -> bool {
    let pi2 = 2.0 * std::f64::consts::PI;
    let mut a = angle - start;
    while a < 0.0 { a += pi2; }
    let mut span = end - start;
    while span <= 0.0 { span += pi2; }
    a <= span + 1e-9
}

/// Map a raw `atan2` angle onto the arc's parameter domain: the equivalent
/// angle in `[start, start+2π)`, clamped to the span. Consumers (trim, split)
/// treat `t` as a position within `domain()`, so a hit must never be reported
/// at e.g. −3π/4 on an arc parameterized `[0, 3π/2]` — that's 5π/4 there.
fn angle_on_domain(angle: f64, start: f64, end: f64) -> f64 {
    let pi2 = 2.0 * std::f64::consts::PI;
    let mut a = angle - start;
    while a < 0.0 { a += pi2; }
    while a > pi2 { a -= pi2; }
    let mut span = end - start;
    while span <= 0.0 { span += pi2; }
    start + a.min(span)
}

/// Circle–circle intersection — direct geometric computation.
///
/// Uses the distance formula to find the intersection points directly,
/// avoiding the radical-axis/LineSeg approach whose domain restrictions
/// would incorrectly filter valid points.
pub fn intersect_circle_circle(c1: &CircularArc, c2: &CircularArc) -> Vec<CurveIntersection> {
    let (cx1, cy1) = c1.center.to_f64();
    let (cx2, cy2) = c2.center.to_f64();
    let r1 = c1.radius.to_f64();
    let r2 = c2.radius.to_f64();

    let dx = cx2 - cx1;
    let dy = cy2 - cy1;
    let d  = (dx * dx + dy * dy).sqrt();

    // Non-intersecting cases
    if d < 1e-12 || d > r1 + r2 + 1e-10 || d < (r1 - r2).abs() - 1e-10 {
        return vec![];
    }

    // Distance from c1-center to the radical axis along the line joining centers
    let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h_sq = r1 * r1 - a * a;
    let h = h_sq.max(0.0).sqrt();

    let ux = dx / d;
    let uy = dy / d;

    // Foot of the perpendicular on the line between centres
    let mx = cx1 + a * ux;
    let my = cy1 + a * uy;

    let mut results = Vec::new();
    let signs: &[f64] = if h < 1e-9 { &[0.0] } else { &[-1.0, 1.0] };

    for &sign in signs {
        let px = mx + sign * h * (-uy);
        let py = my + sign * h * ux;

        let angle1 = (py - cy1).atan2(px - cx1);
        let angle2 = (py - cy2).atan2(px - cx2);

        if angle_in_arc(angle1, c1.start_angle, c1.end_angle) &&
           angle_in_arc(angle2, c2.start_angle, c2.end_angle) {
            results.push(CurveIntersection {
                point: (px, py),
                t1: angle_on_domain(angle1, c1.start_angle, c1.end_angle),
                t2: angle_on_domain(angle2, c2.start_angle, c2.end_angle),
            });
        }
    }
    results
}

// ── General intersection via resultant ───────────────────────────────────────

/// General curve-curve intersection using the algebraic resultant.
/// Returns a list of approximate intersection points.
fn intersect_segments_f64(
    pa: (f64, f64), pb: (f64, f64),
    qa: (f64, f64), qb: (f64, f64),
) -> Option<((f64, f64), f64, f64)> {
    let ux = pb.0 - pa.0;
    let uy = pb.1 - pa.1;
    let vx = qb.0 - qa.0;
    let vy = qb.1 - qa.1;

    let denom = ux * vy - uy * vx;
    if denom.abs() < 1e-12 { return None; }

    let dx = qa.0 - pa.0;
    let dy = qa.1 - pa.1;

    let t = (dx * vy - dy * vx) / denom;
    let s = (dx * uy - dy * ux) / denom;

    let eps = 1e-9;
    if (-eps..=1.0 + eps).contains(&t) && (-eps..=1.0 + eps).contains(&s) {
        let t_clamped = t.clamp(0.0, 1.0);
        let s_clamped = s.clamp(0.0, 1.0);
        let x = pa.0 + t_clamped * ux;
        let y = pa.1 + t_clamped * uy;
        Some(((x, y), t_clamped, s_clamped))
    } else {
        None
    }
}

fn refine_intersection(
    c1: &Curve, c2: &Curve,
    t1_init: f64, t2_init: f64,
) -> CurveIntersection {
    let (t0_1, t1_1) = c1.domain();
    let (t0_2, t1_2) = c2.domain();

    let mut t1 = t1_init;
    let mut t2 = t2_init;

    for _ in 0..6 {
        let (x1, y1) = c1.evaluate_f64(t1);
        let (x2, y2) = c2.evaluate_f64(t2);

        let rx = x1 - x2;
        let ry = y1 - y2;

        if (rx * rx + ry * ry).sqrt() < 1e-12 {
            break;
        }

        let (dx1, dy1) = c1.tangent_f64(t1);
        let (dx2, dy2) = c2.tangent_f64(t2);

        let det = -dx1 * dy2 + dy1 * dx2;
        if det.abs() < 1e-12 {
            break; // Parallel or singular, keep current values
        }

        let dt1 = (rx * dy2 - ry * dx2) / det;
        let dt2 = (-dx1 * ry + dy1 * rx) / det;

        // If step is too large, it is diverging
        if dt1.abs() > 0.1 || dt2.abs() > 0.1 {
            break;
        }

        t1 = (t1 + dt1).clamp(t0_1, t1_1);
        t2 = (t2 + dt2).clamp(t0_2, t1_2);
    }

    let point = c1.evaluate_f64(t1);
    CurveIntersection { point, t1, t2 }
}

/// General curve-curve intersection.
///
/// For pairs whose implicit forms are genuine single-curve equations (conics and
/// Béziers — everything except `PolyCurve`), this routes through the **exact
/// algebraic kernel**: it intersects the implicit forms symbolically, then maps
/// each exact point back to a curve parameter, discarding points that fall outside
/// either bounded domain (the implicit forms are unbounded and may carry extra
/// branches). PolyCurves and degenerate/identical curves fall back to numeric
/// subdivision.
pub fn intersect_general(c1: &Curve, c2: &Curve) -> Vec<CurveIntersection> {
    // PolyCurve's implicit form is a product over its segments (a union locus),
    // not a single curve, so it can't use the exact path; its piecewise parameter
    // is handled correctly by the numeric routine.
    if matches!(c1, Curve::Poly(_)) || matches!(c2, Curve::Poly(_)) {
        return intersect_general_numeric(c1, c2);
    }

    let f = c1.implicit_form();
    let g = c2.implicit_form();
    let exact_points = match f.intersect(&g) {
        Ok(pts) => pts,
        // Resultant vanished (shared component / identical) — defer to numeric.
        Err(_) => return intersect_general_numeric(c1, c2),
    };

    // Keep only exact points that lie on BOTH bounded curves, recovering the
    // parameter on each via projection (which clamps to the domain, so an
    // off-segment point projects to an endpoint and is rejected by distance).
    let on_curve_tol = 1e-7;
    let mut results: Vec<CurveIntersection> = Vec::new();
    for (xa, ya) in &exact_points {
        let x = xa.to_f64(1e-12);
        let y = ya.to_f64(1e-12);
        let p1 = crate::ops::distance::project_point_onto_curve(c1, x, y);
        if p1.distance > on_curve_tol { continue; }
        let p2 = crate::ops::distance::project_point_onto_curve(c2, x, y);
        if p2.distance > on_curve_tol { continue; }
        if results.iter().all(|h| (h.point.0 - x).hypot(h.point.1 - y) > 1e-7) {
            results.push(CurveIntersection { point: (x, y), t1: p1.t, t2: p2.t });
        }
    }
    results
}

/// Numerical fallback: polyline subdivision + Newton-Raphson refinement. Used for
/// `PolyCurve` (no single-curve implicit form) and for degenerate/identical curves
/// where the exact resultant path can't apply.
fn intersect_general_numeric(c1: &Curve, c2: &Curve) -> Vec<CurveIntersection> {
    // PolyCurves are intersected per segment, with each pair routed through the
    // full dispatch (so zigzag line segments hit the exact line/line and
    // line/arc paths). Sampling the WHOLE polyline with a fixed 32 chords cut
    // corners on many-segment zigzags and silently missed crossings.
    if let Curve::Poly(p) = c1 {
        let n = p.segments.len().max(1) as f64;
        let mut out: Vec<CurveIntersection> = Vec::new();
        for (i, seg) in p.segments.iter().enumerate() {
            let (s0, s1) = seg.domain();
            for h in intersect_numeric(seg, c2) {
                let local = if (s1 - s0).abs() < 1e-12 { 0.0 } else { (h.t1 - s0) / (s1 - s0) };
                let global = (i as f64 + local.clamp(0.0, 1.0)) / n;
                if out.iter().all(|o| (o.point.0 - h.point.0).hypot(o.point.1 - h.point.1) >= 1e-5) {
                    out.push(CurveIntersection { point: h.point, t1: global, t2: h.t2 });
                }
            }
        }
        return out;
    }
    if let Curve::Poly(p) = c2 {
        let n = p.segments.len().max(1) as f64;
        let mut out: Vec<CurveIntersection> = Vec::new();
        for (i, seg) in p.segments.iter().enumerate() {
            let (s0, s1) = seg.domain();
            for h in intersect_numeric(c1, seg) {
                let local = if (s1 - s0).abs() < 1e-12 { 0.0 } else { (h.t2 - s0) / (s1 - s0) };
                let global = (i as f64 + local.clamp(0.0, 1.0)) / n;
                if out.iter().all(|o| (o.point.0 - h.point.0).hypot(o.point.1 - h.point.1) >= 1e-5) {
                    out.push(CurveIntersection { point: h.point, t1: h.t1, t2: global });
                }
            }
        }
        return out;
    }

    let (t0_1, t1_1) = c1.domain();
    let (t0_2, t1_2) = c2.domain();

    let n1 = match c1 {
        Curve::Line(_) => 1,
        _ => 32,
    };
    let n2 = match c2 {
        Curve::Line(_) => 1,
        _ => 32,
    };

    let mut pts1 = Vec::with_capacity(n1 + 1);
    for i in 0..=n1 {
        let t = t0_1 + (t1_1 - t0_1) * (i as f64) / (n1 as f64);
        pts1.push((t, c1.evaluate_f64(t)));
    }

    let mut pts2 = Vec::with_capacity(n2 + 1);
    for j in 0..=n2 {
        let t = t0_2 + (t1_2 - t0_2) * (j as f64) / (n2 as f64);
        pts2.push((t, c2.evaluate_f64(t)));
    }

    let mut intersections = Vec::new();
    for i in 0..n1 {
        let (u0, pa) = pts1[i];
        let (u1, pb) = pts1[i + 1];
        for j in 0..n2 {
            let (v0, qa) = pts2[j];
            let (v1, qb) = pts2[j + 1];
            if let Some((_, t_seg, s_seg)) = intersect_segments_f64(pa, pb, qa, qb) {
                let t1_approx = u0 + t_seg * (u1 - u0);
                let t2_approx = v0 + s_seg * (v1 - v0);

                let hit = refine_intersection(c1, c2, t1_approx, t2_approx);

                if !intersections.iter().any(|other: &CurveIntersection| {
                    let dx = other.point.0 - hit.point.0;
                    let dy = other.point.1 - hit.point.1;
                    (dx * dx + dy * dy).sqrt() < 1e-5
                }) {
                    intersections.push(hit);
                }
            }
        }
    }

    intersections
}


// ── Dispatch ──────────────────────────────────────────────────────────────────

/// Dispatch to the most efficient intersection algorithm for the given curve types.
pub fn intersect(c1: &Curve, c2: &Curve) -> Vec<CurveIntersection> {
    match (c1, c2) {
        (Curve::Line(l1), Curve::Line(l2)) =>
            intersect_line_line(l1, l2).into_iter().collect(),
        (Curve::Line(l), Curve::Arc(a)) =>
            intersect_line_circle(l, a),
        // t1 must be the parameter on c1: swap the line/arc params back.
        (Curve::Arc(a), Curve::Line(l)) =>
            intersect_line_circle(l, a).into_iter()
                .map(|h| CurveIntersection { point: h.point, t1: h.t2, t2: h.t1 })
                .collect(),
        (Curve::Arc(a1), Curve::Arc(a2)) =>
            intersect_circle_circle(a1, a2),
        _ =>
            intersect_general(c1, c2),
    }
}

/// Fast, numeric-only intersection dispatch for **interactive** use (snapping,
/// hover previews) where pixel accuracy is enough and per-frame latency matters.
///
/// Identical to [`intersect`] for the cheap exact fast paths (line/line,
/// line/arc, arc/arc), but routes every general case — anything involving a
/// Bézier or PolyCurve — through the numeric subdivision routine instead of the
/// exact algebraic kernel. The exact `intersect_general` path can take ~0.16s for
/// a single Bézier×Bézier pair, which freezes the UI when run every mouse move;
/// the numeric path is microseconds and accurate to sub-pixel for snapping.
pub fn intersect_numeric(c1: &Curve, c2: &Curve) -> Vec<CurveIntersection> {
    match (c1, c2) {
        (Curve::Line(l1), Curve::Line(l2)) =>
            intersect_line_line(l1, l2).into_iter().collect(),
        (Curve::Line(l), Curve::Arc(a)) =>
            intersect_line_circle(l, a),
        // t1 must be the parameter on c1: swap the line/arc params back.
        (Curve::Arc(a), Curve::Line(l)) =>
            intersect_line_circle(l, a).into_iter()
                .map(|h| CurveIntersection { point: h.point, t1: h.t2, t2: h.t1 })
                .collect(),
        (Curve::Arc(a1), Curve::Arc(a2)) =>
            intersect_circle_circle(a1, a2),
        _ =>
            intersect_general_numeric(c1, c2),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::point::Point2d;
    use crate::primitives::LineSeg;

    fn r(n: i64) -> Rational { Rational::from(n) }
    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    /// When the arc is the FIRST argument, t1 must be the arc's parameter,
    /// normalized into its domain — even past the atan2 seam.
    #[test]
    fn arc_first_dispatch_returns_arc_param_in_t1() {
        use crate::primitives::CircularArc;
        // 270° arc from angle 0; vertical line cuts it at angle 5π/4 whose raw
        // atan2 is −3π/4.
        let arc = Curve::Arc(CircularArc::new(
            pt(0, 0), r(5), 0.0, 1.5 * std::f64::consts::PI));
        let x = -5.0 / 2f64.sqrt();
        let line = Curve::Line(LineSeg::from_endpoints(
            Point2d::from_f64(x, -6.0), Point2d::from_f64(x, 0.0)));
        for hits in [intersect(&arc, &line), intersect_numeric(&arc, &line)] {
            assert_eq!(hits.len(), 1);
            let h = &hits[0];
            let expected = 1.25 * std::f64::consts::PI;
            assert!((h.t1 - expected).abs() < 1e-6,
                "t1 must be the arc angle 5π/4, got {}", h.t1);
            let (ex, ey) = arc.evaluate_f64(h.t1);
            assert!((ex - h.point.0).abs() < 1e-6 && (ey - h.point.1).abs() < 1e-6,
                "evaluating the arc at t1 must reproduce the hit point");
        }
    }

    /// Regression: a many-segment zigzag POLYLINE must yield every crossing.
    /// The old fixed 32-chord sampling of the whole polyline cut corners and
    /// silently dropped hits once the polyline had ~32+ segments.
    #[test]
    fn polyline_zigzag_crossings_all_found() {
        use crate::primitives::PolyCurve;
        let mut segs = Vec::new();
        for i in 0..40 {
            let x0 = 0.25 * i as f64;
            let x1 = 0.25 * (i + 1) as f64;
            let y0 = if i % 2 == 0 { -2.0 } else { 2.0 };
            segs.push(Curve::Line(LineSeg::from_endpoints(
                Point2d::from_f64(x0, y0), Point2d::from_f64(x1, -y0))));
        }
        let poly = Curve::Poly(Box::new(PolyCurve::new(segs)));
        let line = Curve::Line(LineSeg::from_endpoints(pt(0, 0), pt(10, 0)));
        let hits = intersect_numeric(&line, &poly);
        assert_eq!(hits.len(), 40, "every zigzag crossing must be found");
        for h in &hits {
            let (x, y) = poly.evaluate_f64(h.t2);
            assert!((x - h.point.0).abs() < 1e-6 && (y - h.point.1).abs() < 1e-6,
                "poly param t2 must reproduce the hit point");
        }
    }

    #[test]
    fn line_line_crossing() {
        // x=y and x+y=4: intersection at (2,2)
        let l1 = LineSeg::from_endpoints(pt(0,0), pt(4,4));
        let l2 = LineSeg::from_endpoints(pt(0,4), pt(4,0));
        let hit = intersect_line_line(&l1, &l2).unwrap();
        assert!((hit.point.0 - 2.0).abs() < 1e-9);
        assert!((hit.point.1 - 2.0).abs() < 1e-9);
    }

    #[test]
    fn line_line_parallel() {
        let l1 = LineSeg::from_endpoints(pt(0,0), pt(2,0));
        let l2 = LineSeg::from_endpoints(pt(0,1), pt(2,1));
        assert!(intersect_line_line(&l1, &l2).is_none());
    }

    #[test]
    fn line_circle_two_points() {
        // Line y=0, circle x²+y²=25: intersects at (±5, 0)
        let line = LineSeg::from_endpoints(
            Point2d::from_f64(-10.0, 0.0),
            Point2d::from_f64(10.0, 0.0),
        );
        let arc = CircularArc::new(pt(0,0), r(5), -std::f64::consts::PI, std::f64::consts::PI);
        let hits = intersect_line_circle(&line, &arc);
        assert_eq!(hits.len(), 2, "Expected 2 intersections, got {}", hits.len());
        let mut xs: Vec<f64> = hits.iter().map(|h| h.point.0).collect();
        xs.sort_by(|a,b| a.partial_cmp(b).unwrap());
        assert!((xs[0] + 5.0).abs() < 1e-4);
        assert!((xs[1] - 5.0).abs() < 1e-4);
    }

    #[test]
    fn circle_circle_two_circles() {
        // (x)²+y²=4 and (x-2)²+y²=4: intersect where x=1, y=±√3
        let c1 = CircularArc::new(pt(0,0), r(2),
            -std::f64::consts::PI, std::f64::consts::PI);
        let c2 = CircularArc::new(pt(2,0), r(2),
            -std::f64::consts::PI, std::f64::consts::PI);
        let hits = intersect_circle_circle(&c1, &c2);
        assert_eq!(hits.len(), 2, "Expected 2 intersections, got {:?}", hits);
        for h in &hits {
            assert!((h.point.0 - 1.0).abs() < 1e-4, "x={}", h.point.0);
            assert!((h.point.1.abs() - 3f64.sqrt()).abs() < 1e-3);
        }
    }

    #[test]
    fn line_circle_intersect_shifted_center() {
        // Circle (x-3)² + (y-4)² = 25: center (3,4), r=5
        // Line y = 4 (horizontal through center): intersects at (3-5, 4) = (-2, 4) and (3+5, 4) = (8, 4)
        let line = LineSeg::from_endpoints(
            Point2d::from_f64(-10.0, 4.0),
            Point2d::from_f64(10.0, 4.0),
        );
        let arc = CircularArc::new(pt(3,4), r(5), -std::f64::consts::PI, std::f64::consts::PI);
        let hits = intersect_line_circle(&line, &arc);
        assert_eq!(hits.len(), 2, "Expected 2 intersections, got {}", hits.len());
        let mut pts: Vec<(f64, f64)> = hits.iter().map(|h| h.point).collect();
        pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        // Verify intersection points are exactly (-2, 4) and (8, 4)
        assert!((pts[0].0 - -2.0).abs() < 1e-4);
        assert!((pts[0].1 - 4.0).abs() < 1e-4);
        assert!((pts[1].0 - 8.0).abs() < 1e-4);
        assert!((pts[1].1 - 4.0).abs() < 1e-4);
    }

    #[test]
    fn line_circle_exact_tangent_is_single_point() {
        // Circle r=5 at origin; horizontal line y=5 is exactly tangent at (0,5).
        // The discriminant is exactly 0 (rational), so we must report ONE touch
        // point, not two coincident ones — and classify it without a float sign.
        let line = LineSeg::from_endpoints(pt(-8, 5), pt(8, 5));
        let arc = CircularArc::new(pt(0,0), r(5), -std::f64::consts::PI, std::f64::consts::PI);
        let hits = intersect_line_circle(&line, &arc);
        assert_eq!(hits.len(), 1, "exact tangent should be a single touch point");
        assert!((hits[0].point.0).abs() < 1e-9, "x≈0, got {}", hits[0].point.0);
        assert!((hits[0].point.1 - 5.0).abs() < 1e-9, "y≈5, got {}", hits[0].point.1);
    }

    #[test]
    fn line_circle_exact_vertical_tangent_is_exact_point() {
        // Vertical line x=5 tangent to the r=5 circle at exactly (5,0). The vertical
        // branch returns the touch point as an exact rational, not a float pair.
        let line = LineSeg::from_endpoints(pt(5, -8), pt(5, 8));
        let arc = CircularArc::new(pt(0,0), r(5), -std::f64::consts::PI, std::f64::consts::PI);
        let hits = intersect_line_circle(&line, &arc);
        assert_eq!(hits.len(), 1, "vertical tangent should be a single touch point");
        assert!((hits[0].point.0 - 5.0).abs() < 1e-12, "x≈5, got {}", hits[0].point.0);
        assert!((hits[0].point.1).abs() < 1e-12, "y≈0, got {}", hits[0].point.1);
    }

    #[test]
    fn ellipse_ellipse_four_points_via_exact_kernel() {
        // E1: x²/4 + y² = 1 (wide) and E2: x² + y²/4 = 1 (tall) cross at the four
        // points (±2/√5, ±2/√5). This conic∩conic case routes through the exact
        // implicit-kernel path (general dispatch), with domain-filtered parameters.
        use crate::primitives::EllipticalArc;
        let tau = std::f64::consts::TAU;
        let e1 = Curve::Ellipse(EllipticalArc::axis_aligned(pt(0, 0), r(2), r(1), 0.0, tau));
        let e2 = Curve::Ellipse(EllipticalArc::axis_aligned(pt(0, 0), r(1), r(2), 0.0, tau));

        let hits = intersect(&e1, &e2);
        assert_eq!(hits.len(), 4, "two crossing ellipses meet in 4 points, got {}", hits.len());

        let f1 = e1.implicit_form();
        let f2 = e2.implicit_form();
        let expect = 2.0 / 5f64.sqrt();
        for h in &hits {
            let (x, y) = h.point;
            // On both conics to high precision — far tighter than the numeric path.
            assert!(f1.eval_f64(x, y).abs() < 1e-7, "off E1: {}", f1.eval_f64(x, y));
            assert!(f2.eval_f64(x, y).abs() < 1e-7, "off E2: {}", f2.eval_f64(x, y));
            assert!((x.abs() - expect).abs() < 1e-6, "x={}", x);
            assert!((y.abs() - expect).abs() < 1e-6, "y={}", y);
            // Recovered parameters must lie within each ellipse's [0, τ] domain.
            assert!((0.0..=tau).contains(&h.t1) && (0.0..=tau).contains(&h.t2));
        }
    }
}
