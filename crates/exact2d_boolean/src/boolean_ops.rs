use exact2d_geometry::{Curve, CurveSegment, Point2d, LineSeg, CubicBezier, PolyCurve,
                       intersect, split_curve};
use crate::region::Region;
use crate::weld::{weld_region, WELD_TOL};

// ── Public boolean operations ─────────────────────────────────────────────────

/// Boolean union of two regions: A ∪ B.
pub fn union(a: &Region, b: &Region) -> Region {
    boolean_op(a, b, |in_a, in_b| in_a || in_b)
}

/// Boolean intersection of two regions: A ∩ B.
pub fn intersection(a: &Region, b: &Region) -> Region {
    boolean_op(a, b, |in_a, in_b| in_a && in_b)
}

/// Boolean difference: A − B.
pub fn difference(a: &Region, b: &Region) -> Region {
    boolean_op(a, b, |in_a, in_b| in_a && !in_b)
}

/// Symmetric difference (XOR): A △ B.
pub fn xor(a: &Region, b: &Region) -> Region {
    boolean_op(a, b, |in_a, in_b| in_a ^ in_b)
}

// ── Core algorithm ────────────────────────────────────────────────────────────

/// General boolean operation driver (spec §2.4).
///
/// Phase 2 implementation:
///   1. Collect boundary curves for A and B.
///   2. Split each curve at its intersections with the other region's boundary.
///   3. For each piece, classify midpoint (inside A? inside B?) and keep if predicate holds.
///   4. Chain surviving pieces into connected loops by endpoint proximity.
///
/// Phase 3 will add: exact algebraic intersections, hole classification, and
/// symbolic perturbation for degenerate (shared boundary) cases.
fn boolean_op<F>(a: &Region, b: &Region, keep: F) -> Region
where
    F: Fn(bool, bool) -> bool,
{
    // Weld each region's boundary seams first: trimmed/float-derived joints only
    // coincide to ~1e-9, which would otherwise leave the loop-chainer with open
    // loops. Welding snaps them to shared vertices (and requantises coordinates).
    let a = &weld_region(a, WELD_TOL);
    let b = &weld_region(b, WELD_TOL);

    let a_boundary: Vec<&Curve> = boundary_curves(a);
    let b_boundary: Vec<&Curve> = boundary_curves(b);

    let mut selected: Vec<Curve> = Vec::new();

    // Process A boundary curves split at B intersections
    for ca in &a_boundary {
        for piece in split_at_intersections(ca, &b_boundary) {
            let (mx, my) = midpoint_f64(&piece);
            if keep(a.contains_point(mx, my), b.contains_point(mx, my)) {
                selected.push(piece);
            }
        }
    }

    // Process B boundary curves split at A intersections
    for cb in &b_boundary {
        for piece in split_at_intersections(cb, &a_boundary) {
            let (mx, my) = midpoint_f64(&piece);
            if keep(a.contains_point(mx, my), b.contains_point(mx, my)) {
                selected.push(piece);
            }
        }
    }

    // Chain pieces into ordered loop(s)
    let outer = chain_into_loop(selected);
    Region::new(outer)
}

fn boundary_curves(r: &Region) -> Vec<&Curve> {
    r.outer.iter()
        .chain(r.holes.iter().flatten())
        .collect()
}

// ── Curve splitting ───────────────────────────────────────────────────────────

/// Split `curve` at all intersections with `others`.
/// Returns the resulting pieces in parameter order.
fn split_at_intersections(curve: &Curve, others: &[&Curve]) -> Vec<Curve> {
    let (domain_lo, domain_hi) = curve.domain();
    let domain_len = domain_hi - domain_lo;
    if domain_len.abs() < 1e-12 { return vec![curve.clone()]; }

    // Collect intersection parameters, normalized to [0, 1]
    let mut params: Vec<f64> = vec![0.0, 1.0];
    for other in others {
        // Numeric intersection: booleans run on float-derived boundaries whose exact
        // resultants are both slow (BigInt) and not-actually-exact after welding.
        for hit in intersect(curve, other) {
            let t_norm = (hit.t1 - domain_lo) / domain_len;
            if t_norm > 1e-8 && t_norm < 1.0 - 1e-8 {
                params.push(t_norm);
            }
        }
    }
    params.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    params.dedup_by(|a, b| (*a - *b).abs() < 1e-8);

    if params.len() <= 2 { return vec![curve.clone()]; }

    // Build pieces between consecutive split points
    params.windows(2)
        .filter(|w| (w[1] - w[0]).abs() > 1e-9)
        .map(|w| extract_piece(curve, w[0], w[1]))
        .collect()
}

/// Extract the piece of `curve` from normalized parameter `t0` to `t1` (both in [0,1]).
fn extract_piece(curve: &Curve, t0: f64, t1: f64) -> Curve {
    // Split at t1, take the left part; then split that at the rescaled t0.
    let left = if t1 < 1.0 - 1e-9 {
        split_curve(curve, t1).0
    } else {
        curve.clone()
    };
    if t0 < 1e-9 {
        return requantize(left);
    }
    // Scale t0 into the coordinate system of `left` (whose domain is [0, t1])
    let t0_scaled = (t0 / t1).min(1.0);
    requantize(split_curve(&left, t0_scaled).1)
}

/// Round a piece's defining points back to ~12 significant digits.
///
/// The split parameters came from f64 intersections, so the exact de Casteljau /
/// endpoint splits on float-derived rationals multiply denominators on every cut.
/// Chained booleans would otherwise accumulate swollen BigInt coordinates that
/// make every downstream rational op allocate and GCD huge integers. Arcs/ellipses
/// keep their exact centre/radius (their split only changes f64 angles).
fn requantize(c: Curve) -> Curve {
    let q = |p: &Point2d| { let (x, y) = p.to_f64(); Point2d::from_f64(x, y) };
    match c {
        Curve::Line(l) => Curve::Line(LineSeg::from_endpoints(q(&l.p0), q(&l.p1))),
        Curve::Bezier(b) =>
            Curve::Bezier(CubicBezier::new(q(&b.p0), q(&b.p1), q(&b.p2), q(&b.p3))),
        Curve::Poly(pc) =>
            Curve::Poly(Box::new(PolyCurve::new(pc.segments.into_iter().map(requantize).collect()))),
        other => other,
    }
}

// ── Loop chaining ─────────────────────────────────────────────────────────────

/// Chain an unordered set of curve pieces into connected loops by endpoint proximity.
/// Returns the pieces in traversal order.  Multiple loops are concatenated.
fn chain_into_loop(segments: Vec<Curve>) -> Vec<Curve> {
    if segments.is_empty() { return segments; }

    let mut result: Vec<Curve> = Vec::new();
    let mut used = vec![false; segments.len()];
    let mut chains: Vec<Vec<usize>> = Vec::new();

    let end_pt = |seg: &Curve| {
        let (_, t1) = seg.domain();
        seg.evaluate_f64(t1)
    };
    let start_pt = |seg: &Curve| {
        let (t0, _) = seg.domain();
        seg.evaluate_f64(t0)
    };

    // Greedy chain building
    for start_idx in 0..segments.len() {
        if used[start_idx] { continue; }
        used[start_idx] = true;
        let mut chain = vec![start_idx];

        loop {
            let last_end = end_pt(&segments[*chain.last().unwrap()]);
            let (sx, sy) = start_pt(&segments[chain[0]]);

            // Check if the chain is already closed
            let gap_sq = (last_end.0 - sx).powi(2) + (last_end.1 - sy).powi(2);
            if gap_sq < 0.01 && chain.len() > 1 { break; } // closed loop

            // Find the nearest unused segment whose start is close to last_end
            let next = (0..segments.len())
                .filter(|&i| !used[i])
                .min_by(|&i, &j| {
                    let (ix, iy) = start_pt(&segments[i]);
                    let (jx, jy) = start_pt(&segments[j]);
                    let di = (ix - last_end.0).powi(2) + (iy - last_end.1).powi(2);
                    let dj = (jx - last_end.0).powi(2) + (jy - last_end.1).powi(2);
                    di.partial_cmp(&dj).unwrap_or(std::cmp::Ordering::Equal)
                });

            match next {
                Some(idx) => {
                    let (nx, ny) = start_pt(&segments[idx]);
                    let d = (nx - last_end.0).powi(2) + (ny - last_end.1).powi(2);
                    if d > 4.0 { break; } // too far — different loop or gap
                    used[idx] = true;
                    chain.push(idx);
                }
                None => break,
            }
        }
        chains.push(chain);
    }

    for chain in chains {
        for idx in chain {
            result.push(segments[idx].clone());
        }
    }
    result
}

/// Midpoint of a curve in float coordinates.
fn midpoint_f64(curve: &Curve) -> (f64, f64) {
    let (t0, t1) = curve.domain();
    curve.evaluate_f64((t0 + t1) / 2.0)
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_geometry::{LineSeg, Point2d, CubicBezier};

    fn square(x0: i64, y0: i64, x1: i64, y1: i64) -> Region {
        Region::new(vec![
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(x0,y0), Point2d::from_i64(x1,y0))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(x1,y0), Point2d::from_i64(x1,y1))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(x1,y1), Point2d::from_i64(x0,y1))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(x0,y1), Point2d::from_i64(x0,y0))),
        ])
    }

    #[test]
    fn difference_excludes_b_segments() {
        // A = (0,0)-(4,4), B = (3,0)-(5,4): A−B removes the strip x∈[3,4]
        let a = square(0, 0, 4, 4);
        let b = square(3, 0, 5, 4);
        let diff = difference(&a, &b);

        // All midpoints of selected segments should satisfy: in_A AND NOT in_B
        for seg in &diff.outer {
            let (mx, my) = midpoint_f64(seg);
            assert!(
                a.contains_point(mx, my) && !b.contains_point(mx, my),
                "Segment midpoint ({:.2}, {:.2}) violates A-B predicate", mx, my
            );
        }
    }

    #[test]
    fn intersection_keeps_overlap_segments() {
        // A = (0,0)-(3,3), B = (2,2)-(5,5): overlap square is (2,2)-(3,3)
        let a = square(0, 0, 3, 3);
        let b = square(2, 2, 5, 5);
        let inter = intersection(&a, &b);

        // All midpoints should be inside BOTH A and B
        for seg in &inter.outer {
            let (mx, my) = midpoint_f64(seg);
            assert!(
                a.contains_point(mx, my) && b.contains_point(mx, my),
                "Intersection segment midpoint ({:.2},{:.2}) not in both regions", mx, my
            );
        }
        // The result should not be empty
        assert!(!inter.outer.is_empty(), "Intersection of overlapping squares should not be empty");
    }

    #[test]
    fn union_no_segments_outside_both() {
        let a = square(0, 0, 3, 3);
        let b = square(2, 2, 5, 5);
        let u = union(&a, &b);

        // All midpoints should be inside A OR B (or both)
        for seg in &u.outer {
            let (mx, my) = midpoint_f64(seg);
            assert!(
                a.contains_point(mx, my) || b.contains_point(mx, my),
                "Union segment midpoint ({:.2},{:.2}) is outside both regions", mx, my
            );
        }
        assert!(!u.outer.is_empty(), "Union should have segments");
    }

    #[test]
    fn boolean_welds_open_input_boundary() {
        // Square A with a ~1e-9 gap at the bottom-left corner; B overlaps its right
        // strip. Welding must close A so the difference still selects segments.
        let g = 1e-9;
        let a = Region::new(vec![
            Curve::Line(LineSeg::from_endpoints(Point2d::from_f64(0.0, 0.0), Point2d::from_f64(4.0, 0.0))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_f64(4.0, 0.0), Point2d::from_f64(4.0, 4.0))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_f64(4.0, 4.0), Point2d::from_f64(0.0, 4.0))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_f64(g, 4.0), Point2d::from_f64(g, g))),
        ]);
        let b = square(3, 0, 5, 4);
        let diff = difference(&a, &b);
        assert!(!diff.outer.is_empty(), "welded difference should not be empty");
        // Predicate is checked against a CLEAN reference square — the open input `a`
        // has unreliable winding (that is exactly what welding repairs internally).
        let a_ref = square(0, 0, 4, 4);
        for seg in &diff.outer {
            let (mx, my) = midpoint_f64(seg);
            assert!(a_ref.contains_point(mx, my) && !b.contains_point(mx, my),
                    "segment midpoint ({:.3},{:.3}) violates A−B", mx, my);
        }
    }

    #[test]
    fn boolean_over_bezier_boundary_is_fast() {
        // Regression/perf canary: a region whose boundary includes a cubic Bézier
        // used to route boolean splitting through the exact symbolic kernel
        // (~seconds per spline pair). It now uses intersect.
        let a = Region::new(vec![
            Curve::Bezier(CubicBezier::new(
                Point2d::from_f64(0.0, 0.0), Point2d::from_f64(1.0, 3.0),
                Point2d::from_f64(3.0, -3.0), Point2d::from_f64(4.0, 0.0))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_f64(4.0, 0.0), Point2d::from_f64(4.0, 4.0))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_f64(4.0, 4.0), Point2d::from_f64(0.0, 4.0))),
            Curve::Line(LineSeg::from_endpoints(Point2d::from_f64(0.0, 4.0), Point2d::from_f64(0.0, 0.0))),
        ]);
        let b = square(1, 1, 3, 5);
        let t = std::time::Instant::now();
        let _ = difference(&a, &b);
        let _ = union(&a, &b);
        let _ = intersection(&a, &b);
        assert!(t.elapsed().as_millis() < 500, "boolean over Bézier too slow: {:?}", t.elapsed());
    }

    #[test]
    fn xor_excludes_overlap() {
        let a = square(0, 0, 3, 3);
        let b = square(2, 2, 5, 5);
        let x = xor(&a, &b);

        // XOR midpoints must be in exactly one region
        for seg in &x.outer {
            let (mx, my) = midpoint_f64(seg);
            let in_a = a.contains_point(mx, my);
            let in_b = b.contains_point(mx, my);
            assert!(
                in_a ^ in_b,
                "XOR segment midpoint ({:.2},{:.2}): in_a={} in_b={}", mx, my, in_a, in_b
            );
        }
    }
}
