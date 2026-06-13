//! Object snapping (spec §4.2). Snaps are computed in f64 — they run on every
//! mouse move and only need pixel accuracy; the snapped position is surfaced as
//! f64 to the UI either way. (Exact rational math here proved far too slow once
//! trimmed entities carried float-derived coordinates.)

use exact2d_geometry::{
    Curve, CurveSegment, Point2d,
    intersect, project_point_onto_curve,
};
use exact2d_document::{Document, EntityKind, EntityId};

/// The kind of geometric feature a snap point came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SnapKind {
    Endpoint,
    Midpoint,
    Center,
    Intersection,
    Perpendicular,
    Tangent,
    Nearest,
    Node,
    Insertion,
}

impl SnapKind {
    /// Selection priority (lower wins). Precise feature snaps (endpoint,
    /// intersection, node) outrank Nearest, which is the lowest-priority fallback —
    /// matching CAD osnap behaviour where a nearby endpoint beats a closer
    /// point-on-curve.
    pub fn priority(self) -> u8 {
        match self {
            SnapKind::Endpoint => 0,
            SnapKind::Intersection | SnapKind::Node | SnapKind::Insertion => 1,
            SnapKind::Center | SnapKind::Midpoint => 2,
            SnapKind::Perpendicular | SnapKind::Tangent => 3,
            SnapKind::Nearest => 9,
        }
    }
}

/// A candidate snap target.
#[derive(Clone, Debug)]
pub struct SnapPoint {
    pub kind: SnapKind,
    pub pos: (f64, f64),
    /// The entity (or first of two for Intersection) the snap belongs to.
    pub entity: EntityId,
}

/// Which snap kinds are currently enabled (the "running osnap" set).
#[derive(Clone, Debug)]
pub struct SnapSettings {
    pub enabled: Vec<SnapKind>,
    /// Pixel/world tolerance for accepting a snap near the cursor.
    pub tolerance: f64,
}

impl Default for SnapSettings {
    fn default() -> Self {
        SnapSettings {
            enabled: vec![
                SnapKind::Endpoint, SnapKind::Midpoint, SnapKind::Center,
                SnapKind::Intersection, SnapKind::Perpendicular, SnapKind::Tangent,
            ],
            tolerance: 0.5,
        }
    }
}

fn dist((ax, ay): (f64, f64), (bx, by): (f64, f64)) -> f64 {
    ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt()
}

/// Collect all snap candidates near `cursor`, honoring `settings`.
/// `reference` is the active "from" point (needed for Perpendicular/Tangent).
pub fn find_snaps(
    doc: &Document,
    cursor: (f64, f64),
    settings: &SnapSettings,
    reference: Option<(f64, f64)>,
) -> Vec<SnapPoint> {
    let mut out = Vec::new();
    let tol = settings.tolerance;
    let on = |k: SnapKind| settings.enabled.contains(&k);

    // Gather candidate entities (editable layers only) once.
    let entities: Vec<_> = doc.editable_entities().collect();

    for e in &entities {
        match &e.kind {
            EntityKind::Curve(c) => {
                // Cheap spatial pre-filter: every curve snap position (endpoint,
                // midpoint, nearest/perpendicular/tangent foot) lies inside the
                // (conservative) bbox — so far-away entities are skipped before
                // any projection math. Uses the pure-f64 bbox: the exact
                // `bounding_box()` clones and compares rationals, which gets
                // expensive once trimmed pieces carry float-derived coordinates.
                // The arc center is checked separately (a shallow arc's center
                // lies outside its bbox).
                let pad = tol * 4.0;
                let (minx, miny, maxx, maxy) = fast_bbox_f64(c);
                let near_bbox = cursor.0 >= minx - pad && cursor.0 <= maxx + pad
                    && cursor.1 >= miny - pad && cursor.1 <= maxy + pad;
                let near_center = on(SnapKind::Center)
                    && center(c).map(|p| dist(p, cursor) <= tol).unwrap_or(false);
                if !near_bbox && !near_center { continue; }

                if on(SnapKind::Endpoint) {
                    for p in endpoints(c) { push_if_near(&mut out, SnapKind::Endpoint, p, e.id, cursor, tol); }
                }
                if on(SnapKind::Midpoint) {
                    if let Some(p) = midpoint(c) { push_if_near(&mut out, SnapKind::Midpoint, p, e.id, cursor, tol); }
                }
                if on(SnapKind::Center) {
                    if let Some(p) = center(c) { push_if_near(&mut out, SnapKind::Center, p, e.id, cursor, tol); }
                }
                if on(SnapKind::Nearest) {
                    let pr = project_point_onto_curve(c, cursor.0, cursor.1);
                    push_if_near(&mut out, SnapKind::Nearest, pr.point, e.id, cursor, tol);
                }
                if on(SnapKind::Perpendicular) {
                    if let Some(r) = reference {
                        if let Some(p) = perpendicular_foot(c, r) {
                            let pr = project_point_onto_curve(c, cursor.0, cursor.1);
                            if dist(pr.point, cursor) <= tol && dist(p, cursor) <= tol * 4.0 {
                                out.push(SnapPoint { kind: SnapKind::Perpendicular, pos: p, entity: e.id });
                            }
                        }
                    }
                }
                if on(SnapKind::Tangent) {
                    if let Some(r) = reference {
                        let pr = project_point_onto_curve(c, cursor.0, cursor.1);
                        if dist(pr.point, cursor) <= tol {
                            for p in tangent_points(c, r) {
                                if dist(p, cursor) <= tol * 4.0 {
                                    out.push(SnapPoint { kind: SnapKind::Tangent, pos: p, entity: e.id });
                                }
                            }
                        }
                    }
                }
            }
            EntityKind::Point(p)
                if on(SnapKind::Node) => { push_if_near(&mut out, SnapKind::Node, p.to_f64(), e.id, cursor, tol); }
            EntityKind::Insert { transform, .. }
                if on(SnapKind::Insertion) => {
                    let base = transform.apply_point(&Point2d::new(0.0, 0.0));
                    push_if_near(&mut out, SnapKind::Insertion, base.to_f64(), e.id, cursor, tol);
                }
            _ => {}
        }
    }

    // Intersection snaps: pairwise over curve entities whose bboxes are near the cursor.
    if on(SnapKind::Intersection) {
        let pad = tol * 5.0;
        let curves: Vec<_> = entities.iter()
            .filter_map(|e| {
                e.as_curve().and_then(|c| {
                    // f64 bbox filter — the exact rational bbox allocated and
                    // compared BigInts per entity per mouse move.
                    let (minx, miny, maxx, maxy) = fast_bbox_f64(c);
                    let near = cursor.0 >= minx - pad && cursor.0 <= maxx + pad
                        && cursor.1 >= miny - pad && cursor.1 <= maxy + pad;
                    if near { Some((e.id, c)) } else { None }
                })
            })
            .collect();
        // Use the fast numeric intersection here, not the exact algebraic kernel:
        // snapping runs on every pointer move and only needs pixel accuracy, while
        // the exact Bézier×Bézier path takes ~0.16s per pair and freezes the UI.
        for i in 0..curves.len() {
            for j in (i + 1)..curves.len() {
                for hit in intersect(curves[i].1, curves[j].1) {
                    push_if_near(&mut out, SnapKind::Intersection, hit.point, curves[i].0, cursor, tol);
                }
            }
        }
    }

    // Order by snap priority first, then by distance to the cursor — so a nearby
    // endpoint beats a closer "nearest point on curve".
    out.sort_by(|a, b| {
        a.kind.priority().cmp(&b.kind.priority())
            .then(dist(a.pos, cursor).partial_cmp(&dist(b.pos, cursor)).unwrap_or(std::cmp::Ordering::Equal))
    });
    out
}

/// The single best snap near the cursor, if any.
pub fn best_snap(
    doc: &Document,
    cursor: (f64, f64),
    settings: &SnapSettings,
    reference: Option<(f64, f64)>,
) -> Option<SnapPoint> {
    find_snaps(doc, cursor, settings, reference).into_iter().next()
}

fn push_if_near(out: &mut Vec<SnapPoint>, kind: SnapKind, pos: (f64, f64), entity: EntityId, cursor: (f64, f64), tol: f64) {
    if dist(pos, cursor) <= tol {
        out.push(SnapPoint { kind, pos, entity });
    }
}

// ── Per-curve snap geometry ───────────────────────────────────────────────────

fn endpoints(c: &Curve) -> Vec<(f64, f64)> {
    match c {
        Curve::Arc(a) => {
            // Full circle (≈2π span) has no distinct endpoints to snap.
            let span = (a.end_angle - a.start_angle).abs();
            if (span - 2.0 * std::f64::consts::PI).abs() < 1e-9 { return vec![]; }
            vec![c.evaluate_f64(a.start_angle), c.evaluate_f64(a.end_angle)]
        }
        _ => {
            let (t0, t1) = c.domain();
            vec![c.evaluate_f64(t0), c.evaluate_f64(t1)]
        }
    }
}

/// Conservative f64 bounding box — no rational clones/compares (those allocate
/// and GCD BigInts, which is too slow to run per entity per mouse move).
fn fast_bbox_f64(c: &Curve) -> (f64, f64, f64, f64) {
    fn join(a: (f64, f64, f64, f64), b: (f64, f64, f64, f64)) -> (f64, f64, f64, f64) {
        (a.0.min(b.0), a.1.min(b.1), a.2.max(b.2), a.3.max(b.3))
    }
    match c {
        Curve::Line(l) => {
            let (x0, y0) = l.p0.to_f64();
            let (x1, y1) = l.p1.to_f64();
            (x0.min(x1), y0.min(y1), x0.max(x1), y0.max(y1))
        }
        Curve::Arc(a) => {
            // Full-circle box regardless of span — conservative is fine here.
            let (cx, cy) = a.center.to_f64();
            let r = a.radius;
            (cx - r, cy - r, cx + r, cy + r)
        }
        Curve::Ellipse(e) => {
            let (cx, cy) = e.center.to_f64();
            let r = e.semi_major.max(e.semi_minor);
            (cx - r, cy - r, cx + r, cy + r)
        }
        Curve::Bezier(b) => {
            // Control-point hull contains the curve.
            let pts = [b.p0.to_f64(), b.p1.to_f64(), b.p2.to_f64(), b.p3.to_f64()];
            pts.iter().fold((f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                |acc, &(x, y)| join(acc, (x, y, x, y)))
        }
        Curve::Poly(p) => {
            p.segments.iter().map(fast_bbox_f64)
                .fold((f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY), join)
        }
    }
}

fn midpoint(c: &Curve) -> Option<(f64, f64)> {
    match c {
        Curve::Line(l) => {
            // f64 average — snapping is pixel-accuracy; the exact rational
            // midpoint paid a BigInt add+GCD per line per mouse move.
            let (x0, y0) = l.p0.to_f64();
            let (x1, y1) = l.p1.to_f64();
            Some(((x0 + x1) / 2.0, (y0 + y1) / 2.0))
        }
        Curve::Arc(a) => {
            let span = (a.end_angle - a.start_angle).abs();
            if (span - 2.0 * std::f64::consts::PI).abs() < 1e-9 { return None; }
            let mid = (a.start_angle + a.end_angle) / 2.0; // half arc-length = mid angle
            Some(c.evaluate_f64(mid))
        }
        _ => {
            let (t0, t1) = c.domain();
            Some(c.evaluate_f64((t0 + t1) / 2.0))
        }
    }
}

fn center(c: &Curve) -> Option<(f64, f64)> {
    match c {
        Curve::Arc(a) => Some(a.center.to_f64()),
        Curve::Ellipse(e) => Some(e.center.to_f64()),
        _ => None,
    }
}

/// Perpendicular foot: point P on the curve such that (P − reference) ⟂ tangent at P.
fn perpendicular_foot(c: &Curve, reference: (f64, f64)) -> Option<(f64, f64)> {
    match c {
        Curve::Line(l) => {
            let (ax, ay) = l.p0.to_f64();
            let (bx, by) = l.p1.to_f64();
            let (dx, dy) = (bx - ax, by - ay);
            let len_sq = dx * dx + dy * dy;
            if len_sq < 1e-20 { return None; }
            let t = ((reference.0 - ax) * dx + (reference.1 - ay) * dy) / len_sq;
            if (-1e-9..=1.0 + 1e-9).contains(&t) {
                let t_clamped = t.clamp(0.0, 1.0);
                Some((ax + t_clamped * dx, ay + t_clamped * dy))
            } else {
                None
            }
        }
        Curve::Arc(a) => {
            // The perpendicular from any external point to a circle passes through
            // the center; the foot is where the center→reference ray meets the circle.
            let (cx, cy) = a.center.to_f64();
            let r = a.radius;
            let (dx, dy) = (reference.0 - cx, reference.1 - cy);
            let len = (dx * dx + dy * dy).sqrt();
            if len < 1e-12 { return None; }
            let angle = dy.atan2(dx);
            let pi2 = 2.0 * std::f64::consts::PI;
            let mut diff = angle - a.start_angle;
            while diff < 0.0 { diff += pi2; }
            while diff >= pi2 { diff -= pi2; }
            if diff <= a.included_angle() + 1e-9 {
                Some((cx + r * dx / len, cy + r * dy / len))
            } else {
                None
            }
        }
        _ => {
            // General: nearest point is the perpendicular foot for smooth curves.
            let pr = project_point_onto_curve(c, reference.0, reference.1);
            Some(pr.point)
        }
    }
}

/// Tangent points: points on the curve where a line from `reference` is tangent.
fn tangent_points(c: &Curve, reference: (f64, f64)) -> Vec<(f64, f64)> {
    match c {
        Curve::Arc(a) => {
            // Tangent lines from an external point to a circle touch at two points.
            // Geometry: if d = |reference − center| > r, the tangent points lie at
            // angle ± acos(r/d) off the center→reference direction.
            let (cx, cy) = a.center.to_f64();
            let r = a.radius;
            let (dx, dy) = (reference.0 - cx, reference.1 - cy);
            let d = (dx * dx + dy * dy).sqrt();
            if d <= r + 1e-12 { return vec![]; } // inside or on the circle
            let base = dy.atan2(dx);
            let off = (r / d).acos();
            let pi2 = 2.0 * std::f64::consts::PI;
            let inc = a.included_angle();
            let mut result = Vec::new();
            for angle in [base + off, base - off] {
                let mut diff = angle - a.start_angle;
                while diff < 0.0 { diff += pi2; }
                while diff >= pi2 { diff -= pi2; }
                if diff <= inc + 1e-9 {
                    result.push((cx + r * angle.cos(), cy + r * angle.sin()));
                }
            }
            result
        }
        _ => vec![], // tangents to general curves: deferred (degree-dependent solve)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_geometry::{LineSeg, CircularArc, CubicBezier};
    use exact2d_document::EntityKind;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    fn doc_with_line() -> (Document, EntityId) {
        let mut doc = Document::new();
        let id = doc.add(EntityKind::Curve(Curve::Line(
            LineSeg::from_endpoints(pt(0, 0), pt(10, 0)))));
        (doc, id)
    }

    // Regression (user report): after a long trim session — many entities whose
    // coordinates carry float-derived rationals — the per-frame snap scan with
    // the default set (incl. Perpendicular/Tangent) must stay interactive. The
    // rational midpoint/bbox math and per-eval Bézier conversions used to make
    // the line tool visibly lag.
    #[test]
    fn snap_scan_stays_fast_after_many_trims() {
        use std::time::Instant;
        let mut doc = Document::new();
        for i in 0..150 {
            let x = 0.123456789012 + i as f64 * 0.37;
            let y = 0.987654321098 + i as f64 * 0.11;
            doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(
                Point2d::from_f64(x, y),
                Point2d::from_f64(x + 1.234567890123, y + 0.55)))));
        }
        for i in 0..20 {
            let x = i as f64 * 2.345678901234;
            doc.add(EntityKind::Curve(Curve::Bezier(CubicBezier::new(
                Point2d::from_f64(x, 0.1), Point2d::from_f64(x + 0.7, 2.3),
                Point2d::from_f64(x + 1.3, -1.7), Point2d::from_f64(x + 2.1, 0.4)))));
        }
        let s = SnapSettings::default(); // Endpoint..Perpendicular/Tangent
        let start = Instant::now();
        // 50 mouse-move frames with an active reference point (line tool state).
        for k in 0..50 {
            let cx = 10.0 + (k as f64) * 0.05;
            let _ = find_snaps(&doc, (cx, 5.0), &s, Some((0.0, 0.0)));
        }
        assert!(start.elapsed().as_millis() < 300,
            "snap scan too slow for interactive use: {:?}", start.elapsed());
    }

    // Regression: two crossing cubic Béziers must not invoke the exact algebraic
    // intersection kernel during snapping (it takes ~0.16s per pair and froze the
    // UI on every mouse move). Intersection snapping uses `intersect`, so a
    // full `find_snaps` over two splines should complete in well under a frame.
    #[test]
    fn intersection_snap_over_two_beziers_is_fast() {
        use std::time::Instant;
        let mut doc = Document::new();
        // Two cubic Béziers that cross near (5, 5).
        doc.add(EntityKind::Curve(Curve::Bezier(CubicBezier::new(
            pt(0, 0), pt(3, 10), pt(7, 10), pt(10, 0)))));
        doc.add(EntityKind::Curve(Curve::Bezier(CubicBezier::new(
            pt(0, 8), pt(3, -2), pt(7, -2), pt(10, 8)))));
        let s = SnapSettings {
            enabled: vec![SnapKind::Intersection, SnapKind::Nearest,
                          SnapKind::Endpoint, SnapKind::Midpoint],
            tolerance: 1.0,
        };
        let start = Instant::now();
        // Simulate 50 mouse-move frames near the crossing.
        for _ in 0..50 {
            let _ = find_snaps(&doc, (5.0, 5.2), &s, None);
        }
        let elapsed = start.elapsed();
        // 50 frames of the exact kernel would be ~8s; numeric is milliseconds.
        // Generous bound (debug build, slow CI) that still catches a regression.
        assert!(elapsed.as_millis() < 500,
            "intersection snapping over two Béziers too slow: {elapsed:?} for 50 frames");
    }

    #[test]
    fn snap_endpoint() {
        let (doc, _) = doc_with_line();
        let s = SnapSettings { enabled: vec![SnapKind::Endpoint], tolerance: 0.5 };
        let snaps = find_snaps(&doc, (0.1, 0.1), &s, None);
        assert!(snaps.iter().any(|sp| sp.kind == SnapKind::Endpoint
            && dist(sp.pos, (0.0, 0.0)) < 1e-9));
    }

    #[test]
    fn snap_midpoint_exact() {
        let (doc, _) = doc_with_line();
        let s = SnapSettings { enabled: vec![SnapKind::Midpoint], tolerance: 0.5 };
        let snaps = find_snaps(&doc, (5.1, 0.1), &s, None);
        let mid = snaps.iter().find(|sp| sp.kind == SnapKind::Midpoint).unwrap();
        assert!((mid.pos.0 - 5.0).abs() < 1e-9 && mid.pos.1.abs() < 1e-9);
    }

    #[test]
    fn snap_center_of_circle() {
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Arc(CircularArc::new(
            pt(3, 4), 5.0, 0.0, 2.0 * std::f64::consts::PI))));
        let s = SnapSettings { enabled: vec![SnapKind::Center], tolerance: 0.5 };
        let snaps = find_snaps(&doc, (3.2, 4.1), &s, None);
        let c = snaps.iter().find(|sp| sp.kind == SnapKind::Center).unwrap();
        assert!((c.pos.0 - 3.0).abs() < 1e-9 && (c.pos.1 - 4.0).abs() < 1e-9);
    }

    #[test]
    fn snap_intersection_of_two_lines() {
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,10)))));
        doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,10), pt(10,0)))));
        let s = SnapSettings { enabled: vec![SnapKind::Intersection], tolerance: 0.5 };
        let snaps = find_snaps(&doc, (5.2, 4.9), &s, None);
        let x = snaps.iter().find(|sp| sp.kind == SnapKind::Intersection).unwrap();
        assert!((x.pos.0 - 5.0).abs() < 1e-6 && (x.pos.1 - 5.0).abs() < 1e-6);
    }

    #[test]
    fn snap_perpendicular_to_line() {
        let (doc, _) = doc_with_line(); // line y=0 from (0,0)-(10,0)
        let s = SnapSettings { enabled: vec![SnapKind::Perpendicular], tolerance: 1.0 };
        // From reference (3, 5), the perpendicular foot is (3, 0).
        let snaps = find_snaps(&doc, (3.1, 0.1), &s, Some((3.0, 5.0)));
        let p = snaps.iter().find(|sp| sp.kind == SnapKind::Perpendicular).unwrap();
        assert!((p.pos.0 - 3.0).abs() < 1e-9 && p.pos.1.abs() < 1e-9);
    }

    #[test]
    fn snap_tangent_to_circle() {
        let mut doc = Document::new();
        // Unit circle at origin
        doc.add(EntityKind::Curve(Curve::Arc(CircularArc::new(
            pt(0, 0), 1.0, 0.0, 2.0 * std::f64::consts::PI))));
        let s = SnapSettings { enabled: vec![SnapKind::Tangent], tolerance: 5.0 };
        // From (2,0): tangent points are at (0.5, ±√3/2)
        let snaps = find_snaps(&doc, (0.5, 0.9), &s, Some((2.0, 0.0)));
        assert!(snaps.iter().any(|sp| sp.kind == SnapKind::Tangent
            && (sp.pos.0 - 0.5).abs() < 1e-6
            && (sp.pos.1.abs() - (3f64.sqrt() / 2.0)).abs() < 1e-6));
    }

    #[test]
    fn snap_nearest_on_line() {
        let (doc, _) = doc_with_line();
        let s = SnapSettings { enabled: vec![SnapKind::Nearest], tolerance: 1.0 };
        let snaps = find_snaps(&doc, (7.0, 0.3), &s, None);
        let n = snaps.iter().find(|sp| sp.kind == SnapKind::Nearest).unwrap();
        assert!((n.pos.0 - 7.0).abs() < 1e-6 && n.pos.1.abs() < 1e-6);
    }
}
