//! Edit commands (spec §4.3): MOVE, COPY, ROTATE, SCALE, MIRROR, OFFSET, ERASE,
//! TRIM, BREAK, ARRAY. All transforms are exact where the geometry permits.

use exact2d_geometry::{Curve, CurveSegment, Point2d, Transform2d, LineSeg, CircularArc, offset_curve, intersect, point_to_curve_distance, split_curve};
use exact2d_document::{Document, EntityId, EntityKind};

// ── Public commands ────────────────────────────────────────────────────────────

/// ERASE: delete entities.
pub fn erase(doc: &mut Document, ids: &[EntityId]) {
    for &id in ids { doc.remove(id); }
}

/// MOVE: translate entities in place by (dx, dy).
pub fn move_by(doc: &mut Document, ids: &[EntityId], dx: f64, dy: f64) {
    let t = Transform2d::translation(dx, dy);
    apply_to(doc, ids, &t);
}

/// COPY: duplicate entities with a displacement; returns the new ids.
pub fn copy_by(doc: &mut Document, ids: &[EntityId], dx: f64, dy: f64) -> Vec<EntityId> {
    let t = Transform2d::translation(dx, dy);
    duplicate_with(doc, ids, &t)
}

/// ROTATE: rotate entities about `center` by `angle` radians (in place).
pub fn rotate(doc: &mut Document, ids: &[EntityId], center: &Point2d, angle: f64) {
    let t = Transform2d::rotation_about(center, angle);
    apply_to(doc, ids, &t);
}

/// SCALE: scale entities about `base` by uniform factor `s` (in place).
pub fn scale(doc: &mut Document, ids: &[EntityId], base: &Point2d, s: f64) {
    let t = Transform2d::scale_about(base, s, s);
    apply_to(doc, ids, &t);
}

/// MIRROR: reflect entities across the line through (p0, p1).
/// If `keep_original` is false the originals are replaced; otherwise copies are added.
pub fn mirror(doc: &mut Document, ids: &[EntityId], p0: &Point2d, p1: &Point2d, keep_original: bool) -> Vec<EntityId> {
    let t = Transform2d::mirror_line(p0, p1);
    if keep_original {
        duplicate_with(doc, ids, &t)
    } else {
        apply_to(doc, ids, &t);
        ids.to_vec()
    }
}

/// OFFSET: create a parallel copy of each entity at signed distance `dist`.
/// Returns the new ids. (Exact for lines/circles; approximate for Béziers.)
pub fn offset(doc: &mut Document, ids: &[EntityId], dist: f64) -> Vec<EntityId> {
    let mut new_ids = Vec::new();
    for &id in ids {
        if let Some(e) = doc.get(id) {
            if let Some(c) = e.as_curve() {
                let off = offset_curve(c, dist);
                let layer = e.layer;
                new_ids.push(doc.add_on_layer(EntityKind::Curve(off), layer));
            }
        }
    }
    new_ids
}

/// Rectangular ARRAY: `rows`×`cols` grid copies spaced (dx, dy). Includes the
/// original position. Returns all new ids (original excluded).
pub fn array_rect(doc: &mut Document, ids: &[EntityId], rows: u32, cols: u32, dx: f64, dy: f64) -> Vec<EntityId> {
    let mut new_ids = Vec::new();
    for r in 0..rows {
        for c in 0..cols {
            if r == 0 && c == 0 { continue; }
            let tx = dx * c as f64;
            let ty = dy * r as f64;
            let t = Transform2d::translation(tx, ty);
            new_ids.extend(duplicate_with(doc, ids, &t));
        }
    }
    new_ids
}

/// Polar ARRAY: `count` copies evenly around `center` spanning `total_angle`
/// radians. Includes the original. Returns new ids.
pub fn array_polar(doc: &mut Document, ids: &[EntityId], center: &Point2d, count: u32, total_angle: f64) -> Vec<EntityId> {
    let mut new_ids = Vec::new();
    if count < 2 { return new_ids; }
    let step = total_angle / count as f64;
    for k in 1..count {
        let t = Transform2d::rotation_about(center, step * k as f64);
        new_ids.extend(duplicate_with(doc, ids, &t));
    }
    new_ids
}

/// TRIM: cut a curve entity at its intersections with `cutters`, keeping only the
/// piece that does NOT contain the picked point `(px, py)`. Replaces the entity.
/// Returns the surviving entity ids (0, 1, or 2 pieces).
pub fn trim(doc: &mut Document, target: EntityId, cutters: &[EntityId], px: f64, py: f64) -> Vec<EntityId> {
    let (curve, layer) = match doc.get(target) {
        Some(e) => match e.as_curve() {
            Some(c) => (c.clone(), e.layer),
            None => return vec![target],
        },
        None => return vec![],
    };

    let (t0, t1) = curve.domain();
    let span = t1 - t0;
    let mut params: Vec<f64> = vec![0.0, 1.0];
    // Interactive op: the split parameters below are f64 anyway, so use the fast
    // numeric intersector — the exact symbolic kernel froze the UI for seconds on
    // Bézier cutters (same lesson as the snapping freeze). A bbox pre-filter
    // skips cutters that cannot touch the target at all.
    let target_bb = curve.bounding_box();
    for &cid in cutters {
        if cid == target { continue; }
        if let Some(cc) = doc.get(cid).and_then(|e| e.as_curve()) {
            if !target_bb.intersects(&cc.bounding_box()) { continue; }
            for hit in intersect(&curve, cc) {
                let tn = (hit.t1 - t0) / span;
                if tn > 1e-6 && tn < 1.0 - 1e-6 { params.push(tn); }
            }
        }
    }
    params.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    params.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    if params.len() <= 2 {
        // No interior cut. If the picked piece is *bounded* by a cutting edge at
        // an endpoint — the normal state of every leftover right after a trim —
        // clicking it removes the whole piece (AutoCAD quick-trim behavior).
        // A piece touching no cutter at all is left alone.
        let eps = 1e-6;
        let touches = |x: f64, y: f64| cutters.iter().any(|&cid| {
            cid != target && doc.get(cid).and_then(|e| e.as_curve())
                .map(|c| point_to_curve_distance(c, x, y) < eps)
                .unwrap_or(false)
        });
        let (sx, sy) = curve.evaluate_f64(t0);
        let (ex, ey) = curve.evaluate_f64(t1);
        if touches(sx, sy) || touches(ex, ey) {
            doc.remove(target);
            return vec![];
        }
        return vec![target];
    }

    // Remove ONLY the picked span — the one between the two cut parameters
    // adjacent to the pick. Everything on either side stays contiguous (a line
    // crossed ten times yields two pieces, not ten fragments); the other
    // crossings remain available for further trims.
    let pick_t = normalized_pick_param(&curve, px, py);
    let (mut lo, mut hi) = (0.0, 1.0);
    for w in params.windows(2) {
        if pick_t >= w[0] && pick_t <= w[1] { lo = w[0]; hi = w[1]; break; }
    }
    let mut survivors = Vec::new();
    doc.remove(target);
    if lo > 1e-6 {
        let piece = extract_piece(&curve, 0.0, lo);
        survivors.push(doc.add_on_layer(EntityKind::Curve(piece), layer));
    }
    if hi < 1.0 - 1e-6 {
        let piece = extract_piece(&curve, hi, 1.0);
        survivors.push(doc.add_on_layer(EntityKind::Curve(piece), layer));
    }
    survivors
}

/// BREAK: split a curve entity into two at parameter `t∈(0,1)`. Replaces it.
pub fn break_at(doc: &mut Document, target: EntityId, t: f64) -> Vec<EntityId> {
    let (curve, layer) = match doc.get(target).and_then(|e| e.as_curve().map(|c| (c.clone(), e.layer))) {
        Some(v) => v,
        None => return vec![target],
    };
    let (left, right) = split_curve(&curve, t);
    doc.remove(target);
    vec![
        doc.add_on_layer(EntityKind::Curve(left), layer),
        doc.add_on_layer(EntityKind::Curve(right), layer),
    ]
}

/// EXTEND: lengthen a line `target` until it meets `boundary`, moving whichever
/// endpoint is nearer the pick `(px,py)`. Exact for line→line; line→circle/arc uses
/// the closed-form ray–circle hit. Returns true if the target was extended.
pub fn extend(doc: &mut Document, target: EntityId, boundary: EntityId, px: f64, py: f64) -> bool {
    let ((x0, y0), (x1, y1)) = match line_endpoints(doc, target) {
        Some(v) => v,
        None => return false,
    };
    let d0 = (x0 - px).powi(2) + (y0 - py).powi(2);
    let d1 = (x1 - px).powi(2) + (y1 - py).powi(2);
    let (mx, my, fx, fy) = if d0 < d1 { (x0, y0, x1, y1) } else { (x1, y1, x0, y0) };
    let (dx, dy) = (mx - fx, my - fy);
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-12 { return false; }
    let (ux, uy) = (dx / len, dy / len);

    let hit = match doc.get(boundary).and_then(|e| e.as_curve()) {
        Some(Curve::Line(bl)) => {
            ray_line_hit((mx, my), (ux, uy), bl.p0.to_f64(), bl.p1.to_f64())
        }
        Some(Curve::Arc(ba)) => {
            let (cx, cy) = ba.center.to_f64();
            ray_circle_hit(mx, my, ux, uy, cx, cy, ba.radius)
        }
        _ => None,
    };
    let (hx, hy) = match hit { Some(p) => p, None => return false };
    set_line_endpoint(doc, target, d0 >= d1, hx, hy)
}

/// FILLET: round the corner between two curves with an arc of `radius`, trimming
/// each curve back to its tangent point. Handles line-line, line-arc, and arc-arc
/// pairs. The pick coordinates `(px, py)` are used to select the correct candidate
/// when the geometry has multiple solutions (line-arc and arc-arc cases).
/// Returns the new fillet arc's id.
pub fn fillet(doc: &mut Document, a: EntityId, b: EntityId, radius: f64, px: f64, py: f64) -> Option<EntityId> {
    if radius <= 0.0 || a == b { return None; }
    let layer = doc.get(a)?.layer;
    let ctx = FilletCtx { radius, px, py, layer };
    let la = line_endpoints(doc, a);
    let lb = line_endpoints(doc, b);
    let aa = arc_snap(doc, a);
    let ab = arc_snap(doc, b);
    match (la, lb, aa, ab) {
        (Some(la), Some(lb), _, _)      => fillet_ll(doc, a, la, b, lb, ctx),
        (Some(la), None, _, Some(ab))   => fillet_la(doc, a, la, b, ab, ctx),
        (None, Some(lb), Some(aa), _)   => fillet_la(doc, b, lb, a, aa, ctx),
        (None, None, Some(aa), Some(ab)) => fillet_aa(doc, a, aa, b, ab, ctx),
        _ => None,
    }
}

/// CHAMFER: bevel the corner between two line segments, trimming line `a` back by
/// `dist_a` and line `b` by `dist_b` and inserting the connecting line. Returns the
/// new connecting line's id.
pub fn chamfer(doc: &mut Document, a: EntityId, b: EntityId, dist_a: f64, dist_b: f64) -> Option<EntityId> {
    if a == b { return None; }
    let la = line_endpoints(doc, a)?;
    let lb = line_endpoints(doc, b)?;
    let layer = doc.get(a)?.layer;
    let (cx, cy) = infinite_line_intersection(la, lb)?;
    let dir_a = dir_from_corner(cx, cy, la);
    let dir_b = dir_from_corner(cx, cy, lb);
    let pa = (cx + dir_a.0 * dist_a, cy + dir_a.1 * dist_a);
    let pb = (cx + dir_b.0 * dist_b, cy + dir_b.1 * dist_b);
    set_line_endpoint(doc, a, endpoint_nearer_is_p1(la, cx, cy), pa.0, pa.1);
    set_line_endpoint(doc, b, endpoint_nearer_is_p1(lb, cx, cy), pb.0, pb.1);
    let conn = LineSeg::from_endpoints(Point2d::from_f64(pa.0, pa.1), Point2d::from_f64(pb.0, pb.1));
    Some(doc.add_on_layer(EntityKind::Curve(Curve::Line(conn)), layer))
}

/// STRETCH: translate by (dx,dy) only those defining points of `ids` that fall
/// inside the window `(xmin,ymin,xmax,ymax)`.
pub fn stretch(doc: &mut Document, ids: &[EntityId], window: (f64, f64, f64, f64), dx: f64, dy: f64) {
    let (xmin, ymin, xmax, ymax) = window;
    let inside = |x: f64, y: f64| x >= xmin && x <= xmax && y >= ymin && y <= ymax;
    let nudge = |p: &Point2d| -> Point2d {
        let (x, y) = p.to_f64();
        if inside(x, y) { Point2d::from_f64(x + dx, y + dy) } else { *p }
    };
    for &id in ids {
        if let Some(e) = doc.get_mut(id) {
            match &mut e.kind {
                EntityKind::Curve(Curve::Line(l)) => { l.p0 = nudge(&l.p0); l.p1 = nudge(&l.p1); }
                EntityKind::Curve(Curve::Bezier(bz)) => {
                    bz.p0 = nudge(&bz.p0); bz.p1 = nudge(&bz.p1);
                    bz.p2 = nudge(&bz.p2); bz.p3 = nudge(&bz.p3);
                }
                EntityKind::Point(p) => { *p = nudge(p); }
                _ => {}
            }
        }
    }
}

// ── Fillet dispatch helpers ───────────────────────────────────────────────────

/// Line-line fillet (inner loop; caller already extracted endpoints).
fn fillet_ll(
    doc: &mut Document,
    a: EntityId, la: LineData,
    b: EntityId, lb: LineData,
    ctx: FilletCtx,
) -> Option<EntityId> {
    let FilletCtx { radius, layer, .. } = ctx;
    let (cx, cy) = infinite_line_intersection(la, lb)?;
    let dir_a = dir_from_corner(cx, cy, la);
    let dir_b = dir_from_corner(cx, cy, lb);
    let cos_t = (dir_a.0 * dir_b.0 + dir_a.1 * dir_b.1).clamp(-1.0, 1.0);
    let theta = cos_t.acos();
    if theta < 1e-6 || (std::f64::consts::PI - theta) < 1e-6 { return None; }

    let tan_dist   = radius / (theta / 2.0).tan();
    let center_dist = radius / (theta / 2.0).sin();
    let ta = (cx + dir_a.0 * tan_dist, cy + dir_a.1 * tan_dist);
    let tb = (cx + dir_b.0 * tan_dist, cy + dir_b.1 * tan_dist);
    let (mut bx, mut by) = (dir_a.0 + dir_b.0, dir_a.1 + dir_b.1);
    let bl = (bx * bx + by * by).sqrt();
    if bl < 1e-12 { return None; }
    bx /= bl; by /= bl;
    let center = (cx + bx * center_dist, cy + by * center_dist);

    set_line_endpoint(doc, a, endpoint_nearer_is_p1(la, cx, cy), ta.0, ta.1);
    set_line_endpoint(doc, b, endpoint_nearer_is_p1(lb, cx, cy), tb.0, tb.1);

    let arc = arc_between(center, ta, tb, radius);
    Some(doc.add_on_layer(EntityKind::Curve(Curve::Arc(arc)), layer))
}

/// Line-arc fillet. Finds the fillet center on the intersection of an offset-line
/// and a concentric circle, then trims both curves to the tangent points.
fn fillet_la(
    doc: &mut Document,
    lid: EntityId, la: LineData,
    aid: EntityId, arc: ArcSnap,
    ctx: FilletCtx,
) -> Option<EntityId> {
    let FilletCtx { radius, px, py, layer } = ctx;
    // Try all combinations: offset line ±radius × arc circle (ar+r / ar-r).
    let mut best_dist = f64::MAX;
    let mut best: Option<LaCandidate> = None; // (fillet_ctr, line_tangent, arc_tangent_angle)

    for &side in &[radius, -radius] {
        for &cr in &[arc.r + radius, arc.r - radius] {
            if cr < 1e-9 { continue; }
            for fc in line_offset_circle_intersects(la.0, la.1, side, arc.cx, arc.cy, cr) {
                let ta_angle = (fc.1 - arc.cy).atan2(fc.0 - arc.cx);
                if !angle_on_arc(ta_angle, arc.start, arc.end) { continue; }
                let tl = foot_on_line(la.0, la.1, fc);
                let d = sq_dist(fc, (px, py));
                if d < best_dist {
                    best_dist = d;
                    best = Some((fc, tl, ta_angle));
                }
            }
        }
    }

    let (fc, tl, ta_angle) = best?;
    let ta = (arc.cx + arc.r * ta_angle.cos(), arc.cy + arc.r * ta_angle.sin());

    set_line_endpoint(doc, lid, endpoint_nearer_is_p1(la, px, py), tl.0, tl.1);
    set_arc_endpoint(doc, aid, arc_endpoint_nearer(&arc, px, py), ta_angle);

    let fillet_arc = arc_between(fc, tl, ta, radius);
    Some(doc.add_on_layer(EntityKind::Curve(Curve::Arc(fillet_arc)), layer))
}

/// Arc-arc fillet. Finds the fillet center on the intersection of two concentric
/// circles (one per arc), then trims both arcs.
fn fillet_aa(
    doc: &mut Document,
    id_a: EntityId, a: ArcSnap,
    id_b: EntityId, b: ArcSnap,
    ctx: FilletCtx,
) -> Option<EntityId> {
    let FilletCtx { radius, px, py, layer } = ctx;
    let mut best_dist = f64::MAX;
    let mut best: Option<AaCandidate> = None; // (fillet_ctr, ta_angle, tb_angle)

    for &ra in &[a.r + radius, a.r - radius] {
        if ra < 1e-9 { continue; }
        for &rb in &[b.r + radius, b.r - radius] {
            if rb < 1e-9 { continue; }
            for fc in circle_circle_intersects(a.cx, a.cy, ra, b.cx, b.cy, rb) {
                let ta = (fc.1 - a.cy).atan2(fc.0 - a.cx);
                let tb = (fc.1 - b.cy).atan2(fc.0 - b.cx);
                if !angle_on_arc(ta, a.start, a.end) { continue; }
                if !angle_on_arc(tb, b.start, b.end) { continue; }
                let d = sq_dist(fc, (px, py));
                if d < best_dist {
                    best_dist = d;
                    best = Some((fc, ta, tb));
                }
            }
        }
    }

    let (fc, ta_angle, tb_angle) = best?;
    let ta = (a.cx + a.r * ta_angle.cos(), a.cy + a.r * ta_angle.sin());
    let tb = (b.cx + b.r * tb_angle.cos(), b.cy + b.r * tb_angle.sin());

    set_arc_endpoint(doc, id_a, arc_endpoint_nearer(&a, px, py), ta_angle);
    set_arc_endpoint(doc, id_b, arc_endpoint_nearer(&b, px, py), tb_angle);

    let fillet_arc = arc_between(fc, ta, tb, radius);
    Some(doc.add_on_layer(EntityKind::Curve(Curve::Arc(fillet_arc)), layer))
}

// ── Helpers ───────────────────────────────────────────────────────────────────

type LineData = ((f64, f64), (f64, f64));

/// Shared parameters threaded through the fillet dispatch helpers: the requested
/// radius, the pick point used to disambiguate multi-solution cases, and the layer
/// the new arc lands on.
#[derive(Clone, Copy)]
struct FilletCtx { radius: f64, px: f64, py: f64, layer: usize }

/// A line-arc fillet candidate: (fillet centre, line tangent point, arc tangent angle).
type LaCandidate = ((f64, f64), (f64, f64), f64);
/// An arc-arc fillet candidate: (fillet centre, arc-a tangent angle, arc-b tangent angle).
type AaCandidate = ((f64, f64), f64, f64);

/// f64 snapshot of a circular-arc entity.
#[derive(Clone, Copy)]
struct ArcSnap { cx: f64, cy: f64, r: f64, start: f64, end: f64 }

fn arc_snap(doc: &Document, id: EntityId) -> Option<ArcSnap> {
    match doc.get(id)?.as_curve()? {
        Curve::Arc(a) => {
            let (cx, cy) = a.center.to_f64();
            Some(ArcSnap { cx, cy, r: a.radius, start: a.start_angle, end: a.end_angle })
        }
        _ => None,
    }
}

/// Returns true when the arc's end_angle endpoint is nearer to (px, py).
fn arc_endpoint_nearer(a: &ArcSnap, px: f64, py: f64) -> bool {
    let sp = (a.cx + a.r * a.start.cos(), a.cy + a.r * a.start.sin());
    let ep = (a.cx + a.r * a.end.cos(),   a.cy + a.r * a.end.sin());
    sq_dist(ep, (px, py)) < sq_dist(sp, (px, py))
}

/// Move one endpoint of an arc to a new angle (set_end=true → end_angle, else start_angle).
/// Normalises the angle so the arc stays CCW with end > start.
fn set_arc_endpoint(doc: &mut Document, id: EntityId, set_end: bool, new_angle: f64) -> bool {
    let tau = std::f64::consts::TAU;
    if let Some(e) = doc.get_mut(id) {
        if let EntityKind::Curve(Curve::Arc(arc)) = &mut e.kind {
            if set_end {
                let mut a = new_angle;
                while a <= arc.start_angle { a += tau; }
                while a > arc.start_angle + tau { a -= tau; }
                arc.end_angle = a;
            } else {
                let mut a = new_angle;
                while a >= arc.end_angle { a -= tau; }
                while a < arc.end_angle - tau { a += tau; }
                arc.start_angle = a;
            }
            return true;
        }
    }
    false
}

/// True when `angle` (after normalisation) lies within the arc domain [start, end].
fn angle_on_arc(angle: f64, start: f64, end: f64) -> bool {
    let tau = std::f64::consts::TAU;
    let mut a = angle;
    while a < start - 1e-9  { a += tau; }
    while a > start + tau + 1e-9 { a -= tau; }
    a <= end + 1e-9
}

/// Intersect the infinite line offset by `side` (signed perpendicular distance,
/// positive = left of direction) with the circle of radius `cr` centred at (cx, cy).
fn line_offset_circle_intersects(
    p0: (f64, f64), p1: (f64, f64), side: f64,
    cx: f64, cy: f64, cr: f64,
) -> Vec<(f64, f64)> {
    let (dx, dy) = (p1.0 - p0.0, p1.1 - p0.1);
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-12 { return vec![]; }
    let (ux, uy) = (dx / len, dy / len);
    let (nx, ny) = (-uy, ux);                          // left-hand unit normal
    let (ox, oy) = (p0.0 + side * nx, p0.1 + side * ny); // offset-line origin
    let (fx, fy) = (ox - cx, oy - cy);
    let b = 2.0 * (fx * ux + fy * uy);
    let c = fx * fx + fy * fy - cr * cr;
    let disc = b * b - 4.0 * c;
    if disc < 0.0 { return vec![]; }
    let sq = disc.sqrt();
    vec![
        (ox + ((-b - sq) / 2.0) * ux, oy + ((-b - sq) / 2.0) * uy),
        (ox + ((-b + sq) / 2.0) * ux, oy + ((-b + sq) / 2.0) * uy),
    ]
}

/// Intersection points of two circles.
fn circle_circle_intersects(
    cx1: f64, cy1: f64, r1: f64,
    cx2: f64, cy2: f64, r2: f64,
) -> Vec<(f64, f64)> {
    let dx = cx2 - cx1;
    let dy = cy2 - cy1;
    let d2 = dx * dx + dy * dy;
    let d  = d2.sqrt();
    if d < 1e-12 || d > r1 + r2 + 1e-9 || d < (r1 - r2).abs() - 1e-9 { return vec![]; }
    let a = (r1 * r1 - r2 * r2 + d2) / (2.0 * d);
    let h2 = r1 * r1 - a * a;
    if h2 < 0.0 { return vec![]; }
    let h  = h2.sqrt();
    let mx = cx1 + a * dx / d;
    let my = cy1 + a * dy / d;
    if h < 1e-9 { return vec![(mx, my)]; }
    let px = h * dy / d;
    let py = h * dx / d;
    vec![(mx + px, my - py), (mx - px, my + py)]
}

/// Foot of perpendicular from `pt` onto the infinite line through `p0`–`p1`.
fn foot_on_line(p0: (f64, f64), p1: (f64, f64), pt: (f64, f64)) -> (f64, f64) {
    let (dx, dy) = (p1.0 - p0.0, p1.1 - p0.1);
    let len2 = dx * dx + dy * dy;
    if len2 < 1e-24 { return p0; }
    let t = ((pt.0 - p0.0) * dx + (pt.1 - p0.1) * dy) / len2;
    (p0.0 + t * dx, p0.1 + t * dy)
}

fn sq_dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)
}

/// The two endpoints of a line entity as f64, or None if it isn't a line.
fn line_endpoints(doc: &Document, id: EntityId) -> Option<LineData> {
    match doc.get(id).and_then(|e| e.as_curve()) {
        Some(Curve::Line(l)) => Some((l.p0.to_f64(), l.p1.to_f64())),
        _ => None,
    }
}

/// Replace endpoint p1 (if `which_p1`) or p0 of a line entity with (x,y).
fn set_line_endpoint(doc: &mut Document, id: EntityId, which_p1: bool, x: f64, y: f64) -> bool {
    if let Some(e) = doc.get_mut(id) {
        if let EntityKind::Curve(Curve::Line(l)) = &mut e.kind {
            if which_p1 { l.p1 = Point2d::from_f64(x, y); } else { l.p0 = Point2d::from_f64(x, y); }
            return true;
        }
    }
    false
}

/// Whether the line endpoint nearer to (cx,cy) is p1 (else p0).
fn endpoint_nearer_is_p1(l: LineData, cx: f64, cy: f64) -> bool {
    let d0 = (l.0.0 - cx).powi(2) + (l.0.1 - cy).powi(2);
    let d1 = (l.1.0 - cx).powi(2) + (l.1.1 - cy).powi(2);
    d1 < d0
}

/// Unit direction from the corner toward the line's far endpoint.
fn dir_from_corner(cx: f64, cy: f64, l: LineData) -> (f64, f64) {
    let far = if endpoint_nearer_is_p1(l, cx, cy) { l.0 } else { l.1 };
    let (dx, dy) = (far.0 - cx, far.1 - cy);
    let n = (dx * dx + dy * dy).sqrt().max(1e-12);
    (dx / n, dy / n)
}

/// Intersection of the two infinite lines through the given segments, in f64.
fn infinite_line_intersection(la: LineData, lb: LineData) -> Option<(f64, f64)> {
    let (a1, b1, c1) = implicit(la);
    let (a2, b2, c2) = implicit(lb);
    let det = a1 * b2 - a2 * b1;
    if det.abs() < 1e-12 { return None; }
    Some(((b1 * c2 - b2 * c1) / det, (a2 * c1 - a1 * c2) / det))
}

/// ax+by+c=0 coefficients for an f64 segment.
fn implicit(l: LineData) -> (f64, f64, f64) {
    let ((x0, y0), (x1, y1)) = l;
    (y0 - y1, x1 - x0, x0 * y1 - x1 * y0)
}

/// First forward hit of the ray (origin `o`, unit dir `u`) with the infinite line
/// through `a0`-`a1`.
fn ray_line_hit(o: (f64, f64), u: (f64, f64), a0: (f64, f64), a1: (f64, f64)) -> Option<(f64, f64)> {
    let (a, b, c) = implicit((a0, a1));
    let denom = a * u.0 + b * u.1;
    if denom.abs() < 1e-12 { return None; }
    let t = -(a * o.0 + b * o.1 + c) / denom;
    if t <= 1e-9 { return None; }
    Some((o.0 + u.0 * t, o.1 + u.1 * t))
}

/// First forward hit of the ray (origin o, unit dir u) with the circle (center, r).
fn ray_circle_hit(ox: f64, oy: f64, ux: f64, uy: f64, cx: f64, cy: f64, r: f64) -> Option<(f64, f64)> {
    let (fx, fy) = (ox - cx, oy - cy);
    let b = 2.0 * (fx * ux + fy * uy);
    let c = fx * fx + fy * fy - r * r;
    let disc = b * b - 4.0 * c;
    if disc < 0.0 { return None; }
    let sq = disc.sqrt();
    [(-b - sq) / 2.0, (-b + sq) / 2.0].into_iter()
        .filter(|&t| t > 1e-9)
        .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .map(|t| (ox + ux * t, oy + uy * t))
}

/// Build the minor arc between tangent points `ta`,`tb` about `center` with `radius`.
fn arc_between(center: (f64, f64), ta: (f64, f64), tb: (f64, f64), radius: f64) -> CircularArc {
    let a0 = (ta.1 - center.1).atan2(ta.0 - center.0);
    let mut a1 = (tb.1 - center.1).atan2(tb.0 - center.0);
    let mut sweep = a1 - a0;
    while sweep <= -std::f64::consts::PI { sweep += std::f64::consts::TAU; a1 += std::f64::consts::TAU; }
    while sweep >   std::f64::consts::PI { sweep -= std::f64::consts::TAU; a1 -= std::f64::consts::TAU; }
    let (start, end) = if a1 >= a0 { (a0, a1) } else { (a1, a0) };
    CircularArc::new(Point2d::from_f64(center.0, center.1), radius, start, end)
}

fn apply_to(doc: &mut Document, ids: &[EntityId], t: &Transform2d) {
    for &id in ids {
        if let Some(e) = doc.get_mut(id) { e.transform(t); }
    }
}

fn duplicate_with(doc: &mut Document, ids: &[EntityId], t: &Transform2d) -> Vec<EntityId> {
    let mut new_ids = Vec::new();
    for &id in ids {
        if let Some(e) = doc.get(id) {
            new_ids.push(doc.add_entity(e.transformed(t)));
        }
    }
    new_ids
}

fn normalized_pick_param(curve: &Curve, px: f64, py: f64) -> f64 {
    let pr = exact2d_geometry::project_point_onto_curve(curve, px, py);
    let (t0, t1) = curve.domain();
    ((pr.t - t0) / (t1 - t0)).clamp(0.0, 1.0)
}

/// Extract the sub-curve between normalized params [a, b].
fn extract_piece(curve: &Curve, a: f64, b: f64) -> Curve {
    let left = if b < 1.0 - 1e-9 { split_curve(curve, b).0 } else { curve.clone() };
    let piece = if a < 1e-9 { left } else {
        let a_scaled = (a / b).min(1.0);
        split_curve(&left, a_scaled).1
    };
    requantize(piece)
}

/// Round a piece's defining points back to ~12 significant digits.
///
/// The split parameter came from an f64 intersection, so the piece carries no
/// real precision beyond f64 — but the *exact* de Casteljau / endpoint split on
/// float-derived rationals multiplies denominators on every cut. After a long
/// trim session those swollen coordinates made every per-frame rational op
/// (snap scans, bboxes, projections) allocate and GCD huge BigInts, visibly
/// degrading the UI. Arcs/ellipses keep their exact center/radius (their split
/// only changes f64 angles).
fn requantize(c: Curve) -> Curve {
    let q = |p: &Point2d| { let (x, y) = p.to_f64(); Point2d::from_f64(x, y) };
    match c {
        Curve::Line(l) => Curve::Line(LineSeg::from_endpoints(q(&l.p0), q(&l.p1))),
        Curve::Bezier(b) =>
            Curve::Bezier(exact2d_geometry::CubicBezier::new(q(&b.p0), q(&b.p1), q(&b.p2), q(&b.p3))),
        Curve::Poly(pc) => Curve::Poly(Box::new(exact2d_geometry::PolyCurve::new(
            pc.segments.into_iter().map(requantize).collect()))),
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_geometry::CircularArc;
    use crate::draw;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }
    fn r(n: i64) -> f64 { n as f64 }

    /// Regression: trimming against Bézier cutters must use the fast numeric
    /// intersector. The exact symbolic kernel took ~seconds per spline pair and
    /// froze the UI on click (same root cause as the old snapping freeze).
    #[test]
    fn trim_with_bezier_cutters_is_fast_and_correct() {
        let mut doc = Document::new();
        // Horizontal target line, each spline crossing it exactly once.
        let target = draw::line(&mut doc, pt(0, 0), pt(10, 0));
        let c1 = draw::bezier(&mut doc, pt(2, -3), pt(2, -1), pt(3, 1), pt(3, 3));
        let c2 = draw::bezier(&mut doc, pt(7, -3), pt(7, -1), pt(8, 1), pt(8, 3));
        let start = std::time::Instant::now();
        // Pick the middle (x≈5): keep the two outer pieces.
        let survivors = trim(&mut doc, target, &[c1, c2], 5.0, 0.0);
        assert!(start.elapsed().as_millis() < 500,
            "trim took {:?} — exact kernel is back in the interactive path?", start.elapsed());
        assert_eq!(survivors.len(), 2);
    }

    /// Regression (user report): a polyline-zigzag cutter must register its
    /// FIRST crossing — the old whole-polyline chord sampling skipped early
    /// crossings, so picking the start segment trimmed to the SECOND one.
    #[test]
    fn trim_against_polyline_zigzag_cuts_at_first_crossing() {
        use exact2d_geometry::Point2d;
        let mut doc = Document::new();
        let target = draw::line(&mut doc, pt(0, 0), pt(10, 0));
        let mut segs = Vec::new();
        for i in 0..40 {
            let x0 = 0.25 * i as f64;
            let x1 = 0.25 * (i + 1) as f64;
            let y0 = if i % 2 == 0 { -2.0 } else { 2.0 };
            segs.push(Curve::Line(exact2d_geometry::LineSeg::from_endpoints(
                Point2d::from_f64(x0, y0), Point2d::from_f64(x1, -y0))));
        }
        let zig = draw::polycurve(&mut doc, segs);
        // Pick before the first crossing (x = 0.125): only [0, 0.125] is removed.
        let survivors = trim(&mut doc, target, &[zig], 0.05, 0.0);
        assert_eq!(survivors.len(), 1);
        if let Some(Curve::Line(l)) = doc.get(survivors[0]).and_then(|e| e.as_curve()) {
            let x0 = l.p0.x.min(l.p1.x);
            let x1 = l.p0.x.max(l.p1.x);
            assert!((x0 - 0.125).abs() < 1e-6,
                "survivor must start at the FIRST zigzag crossing, got {x0}");
            assert!((x1 - 10.0).abs() < 1e-6);
        } else { panic!("survivor is not a line"); }
    }

    /// Regression (user report): trim must NOT shatter the target at every
    /// crossing — only the two boundaries adjacent to the pick cut it, and each
    /// side stays one contiguous entity (still crossing the other cutters).
    #[test]
    fn trim_cuts_only_adjacent_boundaries() {
        let mut doc = Document::new();
        let target = draw::line(&mut doc, pt(0, 0), pt(10, 0));
        let v: Vec<_> = [2, 5, 8].iter()
            .map(|&x| draw::line(&mut doc, pt(x, -2), pt(x, 2)))
            .collect();
        // Pick between x=2 and x=5: only that span goes.
        let survivors = trim(&mut doc, target, &v, 3.5, 0.0);
        assert_eq!(survivors.len(), 2, "exactly two contiguous sides, not fragments");
        let mut spans: Vec<(f64, f64)> = survivors.iter().map(|&id| {
            match doc.get(id).and_then(|e| e.as_curve()) {
                Some(Curve::Line(l)) => {
                    let (a, b) = (l.p0.x, l.p1.x);
                    (a.min(b), a.max(b))
                }
                _ => panic!("survivor is not a line"),
            }
        }).collect();
        spans.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        assert!((spans[0].0 - 0.0).abs() < 1e-6 && (spans[0].1 - 2.0).abs() < 1e-6);
        // The right side must remain ONE piece [5,10] — still crossing x=8.
        assert!((spans[1].0 - 5.0).abs() < 1e-6 && (spans[1].1 - 10.0).abs() < 1e-6,
            "right side must stay contiguous across the x=8 cutter, got {:?}", spans[1]);
    }

    /// Regression (user report): zigzag workflow — after the first trim splits a
    /// curve into pieces bounded by the cutters, clicking another piece must
    /// remove it too (AutoCAD quick-trim), not silently do nothing.
    #[test]
    fn trim_removes_bounded_leftover_pieces() {
        let mut doc = Document::new();
        let target = draw::line(&mut doc, pt(0, 0), pt(10, 0));
        let v: Vec<_> = [2, 5].iter()
            .map(|&x| draw::line(&mut doc, pt(x, -2), pt(x, 2)))
            .collect();
        let first = trim(&mut doc, target, &v, 3.5, 0.0);
        assert_eq!(first.len(), 2);
        // Click the left piece [0,2]: bounded by the x=2 cutter → deleted whole.
        let left = *first.iter().find(|&&id| {
            matches!(doc.get(id).and_then(|e| e.as_curve()),
                Some(Curve::Line(l)) if l.p0.x.min(l.p1.x) < 1.0)
        }).expect("left piece exists");
        let cutters: Vec<_> = doc.iter().map(|e| e.id).filter(|&i| i != left).collect();
        let second = trim(&mut doc, left, &cutters, 1.0, 0.0);
        assert!(second.is_empty(), "bounded leftover must be deleted");
        assert!(doc.get(left).is_none(), "the piece must be gone from the document");
    }

    /// Same zigzag workflow on an arc: leftover arc pieces bounded by the
    /// cutters must also be removable with a click.
    #[test]
    fn trim_removes_bounded_leftover_arc_piece() {
        let mut doc = Document::new();
        let target = draw::arc(&mut doc, pt(0, 0), r(5), 0.0, std::f64::consts::PI);
        let l1 = draw::line(&mut doc, pt(3, 0), pt(3, 6));
        let l2 = draw::line(&mut doc, pt(-3, 0), pt(-3, 6));
        let first = trim(&mut doc, target, &[l1, l2], 0.0, 5.0);
        assert_eq!(first.len(), 2);
        // Click the right piece (from (5,0) to (3,4)): bounded at (3,4) → deleted.
        let right = *first.iter().find(|&&id| {
            matches!(doc.get(id).and_then(|e| e.as_curve()),
                Some(Curve::Arc(a)) if a.start_angle < 0.1)
        }).expect("right piece exists");
        let cutters: Vec<_> = doc.iter().map(|e| e.id).filter(|&i| i != right).collect();
        let second = trim(&mut doc, right, &cutters, 4.8, 1.0);
        assert!(second.is_empty(), "bounded arc piece must be deleted");
        assert!(doc.get(right).is_none());
    }

    /// Safety: trim must NOT delete an entity that touches no cutting edge.
    #[test]
    fn trim_leaves_untouched_entities_alone() {
        let mut doc = Document::new();
        let lonely = draw::line(&mut doc, pt(20, 20), pt(30, 20));
        let far = draw::line(&mut doc, pt(0, 0), pt(0, 5));
        let result = trim(&mut doc, lonely, &[far], 25.0, 20.0);
        assert_eq!(result, vec![lonely], "no intersection, no endpoint contact → no-op");
        assert!(doc.get(lonely).is_some());
    }

    /// Regression (user report): after trimming once, the surviving pieces must
    /// remain trimmable at their remaining crossings.
    #[test]
    fn trim_same_line_twice() {
        let mut doc = Document::new();
        // Horizontal line crossed by verticals at x = 2 and 5.
        let target = draw::line(&mut doc, pt(0, 0), pt(10, 0));
        let v: Vec<_> = [2, 5].iter()
            .map(|&x| draw::line(&mut doc, pt(x, -2), pt(x, 2)))
            .collect();
        // First trim: remove the span between x=2 and x=5 (pick x=3.5).
        let first = trim(&mut doc, target, &v, 3.5, 0.0);
        assert_eq!(first.len(), 2);
        // Draw a new cutter at x=8 and trim the surviving [5..10] piece again —
        // its endpoints come from f64 round-trips and must stay trimmable.
        let right = *first.iter().find(|&&id| {
            matches!(doc.get(id).and_then(|e| e.as_curve()),
                Some(Curve::Line(l)) if l.p0.x.max(l.p1.x) > 9.0)
        }).expect("right piece exists");
        draw::line(&mut doc, pt(8, -2), pt(8, 2));
        let cutters: Vec<_> = doc.iter().map(|e| e.id).filter(|&i| i != right).collect();
        let second = trim(&mut doc, right, &cutters, 6.5, 0.0);
        assert_eq!(second.len(), 1, "second trim on the same line must still cut");
        if let Some(Curve::Line(l)) = doc.get(second[0]).and_then(|e| e.as_curve()) {
            let (x0, x1) = (l.p0.x.min(l.p1.x),
                            l.p0.x.max(l.p1.x));
            assert!((x0 - 8.0).abs() < 1e-6 && (x1 - 10.0).abs() < 1e-6,
                "expected the [8,10] piece, got [{x0},{x1}]");
        } else { panic!("survivor is not a line"); }
    }

    /// Regression (user report): trimming an arc crossed by two lines must cut
    /// at BOTH lines and remove only the picked middle span — one cutter was
    /// being ignored because the (Arc, Line) dispatch returned the line's
    /// parameter in t1 instead of the arc angle.
    #[test]
    fn trim_arc_between_two_lines() {
        let mut doc = Document::new();
        // Upper semicircle r=5 about the origin; verticals at x=±3 cross it at
        // (±3, 4). Pick the top (0,5) → keep the two outer pieces.
        let target = draw::arc(&mut doc, pt(0, 0), r(5), 0.0, std::f64::consts::PI);
        let l1 = draw::line(&mut doc, pt(3, 0), pt(3, 6));
        let l2 = draw::line(&mut doc, pt(-3, 0), pt(-3, 6));
        let survivors = trim(&mut doc, target, &[l1, l2], 0.0, 5.0);
        assert_eq!(survivors.len(), 2, "both line cutters must register");
        for id in &survivors {
            if let Some(Curve::Arc(a)) = doc.get(*id).and_then(|e| e.as_curve()) {
                // Each survivor must stop at one of the cut points (±3, 4).
                let hits_cut = [a.start_point(), a.end_point()].iter().any(|(x, y)|
                    (x.abs() - 3.0).abs() < 1e-6 && (y - 4.0).abs() < 1e-6);
                assert!(hits_cut, "piece does not end at a cut point");
            } else { panic!("survivor is not an arc"); }
        }
    }

    /// Regression (user report): a hit past the atan2 seam (raw angle negative,
    /// arc domain positive) was dropped or mapped outside the arc, trimming all
    /// the way to the arc's far end. The 270° arc here is cut at 5π/4.
    #[test]
    fn trim_arc_with_wrapped_angle_cut() {
        let mut doc = Document::new();
        let target = draw::arc(&mut doc, pt(0, 0), r(5),
            0.0, 1.5 * std::f64::consts::PI);
        // Vertical line through the cut point (−5/√2, −5/√2) ≈ angle 5π/4,
        // whose raw atan2 is −3π/4 (outside the domain until normalized).
        let x = -5.0 / 2f64.sqrt();
        let l = draw::line(&mut doc, Point2d::from_f64(x, -6.0), Point2d::from_f64(x, 0.0));
        // Pick near the arc's end (angle ≈ 4.3 rad) → drop the short end piece.
        let (px, py) = (5.0 * 4.3f64.cos(), 5.0 * 4.3f64.sin());
        let survivors = trim(&mut doc, target, &[l], px, py);
        assert_eq!(survivors.len(), 1, "the wrapped-angle cut must register");
        assert_ne!(survivors[0], target, "trim must actually split the arc");
        if let Some(Curve::Arc(a)) = doc.get(survivors[0]).and_then(|e| e.as_curve()) {
            let expected = 1.25 * std::f64::consts::PI;
            assert!((a.end_angle - expected).abs() < 1e-3,
                "survivor must end at 5π/4, got {}", a.end_angle);
        } else { panic!("survivor is not an arc"); }
    }

    #[test]
    fn move_translates() {
        let mut doc = Document::new();
        let id = draw::line(&mut doc, pt(0,0), pt(2,0));
        move_by(&mut doc, &[id], r(5), r(3));
        if let Curve::Line(l) = doc.get(id).unwrap().as_curve().unwrap() {
            assert_eq!(l.p0, pt(5,3));
            assert_eq!(l.p1, pt(7,3));
        } else { panic!() }
    }

    #[test]
    fn copy_keeps_original_and_adds_new() {
        let mut doc = Document::new();
        let id = draw::line(&mut doc, pt(0,0), pt(1,0));
        let new = copy_by(&mut doc, &[id], r(10), r(0));
        assert_eq!(doc.len(), 2);
        assert_ne!(new[0], id);
        if let Curve::Line(l) = doc.get(new[0]).unwrap().as_curve().unwrap() {
            assert_eq!(l.p0, pt(10,0));
        } else { panic!() }
    }

    #[test]
    fn rotate_90_about_origin() {
        let mut doc = Document::new();
        let id = draw::line(&mut doc, pt(1,0), pt(2,0));
        rotate(&mut doc, &[id], &pt(0,0), std::f64::consts::FRAC_PI_2);
        if let Curve::Line(l) = doc.get(id).unwrap().as_curve().unwrap() {
            assert!((l.p0.x).abs() < 1e-9 && (l.p0.y - 1.0).abs() < 1e-6);
            assert!((l.p1.x).abs() < 1e-9 && (l.p1.y - 2.0).abs() < 1e-6);
        } else { panic!() }
    }

    #[test]
    fn scale_doubles_size() {
        let mut doc = Document::new();
        let id = draw::line(&mut doc, pt(1,1), pt(3,1));
        scale(&mut doc, &[id], &pt(1,1), r(2));
        if let Curve::Line(l) = doc.get(id).unwrap().as_curve().unwrap() {
            assert_eq!(l.p0, pt(1,1));
            assert_eq!(l.p1, pt(5,1));
        } else { panic!() }
    }

    #[test]
    fn mirror_keep_original_adds_copy() {
        let mut doc = Document::new();
        let id = draw::line(&mut doc, pt(1,2), pt(3,4));
        let new = mirror(&mut doc, &[id], &pt(0,0), &pt(1,0), true);
        assert_eq!(doc.len(), 2);
        if let Curve::Line(l) = doc.get(new[0]).unwrap().as_curve().unwrap() {
            assert_eq!(l.p0, pt(1,-2));
            assert_eq!(l.p1, pt(3,-4));
        } else { panic!() }
    }

    #[test]
    fn offset_circle_grows() {
        let mut doc = Document::new();
        let id = doc.add(EntityKind::Curve(Curve::Arc(CircularArc::new(
            pt(0,0), r(5), 0.0, 2.0*std::f64::consts::PI))));
        let new = offset(&mut doc, &[id], 2.0);
        if let Curve::Arc(a) = doc.get(new[0]).unwrap().as_curve().unwrap() {
            assert!((a.radius - 7.0).abs() < 1e-6);
        } else { panic!() }
    }

    #[test]
    fn rect_array_count() {
        let mut doc = Document::new();
        let id = draw::line(&mut doc, pt(0,0), pt(1,0));
        let new = array_rect(&mut doc, &[id], 2, 3, r(5), r(5));
        assert_eq!(new.len(), 5);
        assert_eq!(doc.len(), 6);
    }

    #[test]
    fn polar_array_count() {
        let mut doc = Document::new();
        let id = draw::line(&mut doc, pt(1,0), pt(2,0));
        let new = array_polar(&mut doc, &[id], &pt(0,0), 4, 2.0*std::f64::consts::PI);
        assert_eq!(new.len(), 3);
    }

    #[test]
    fn break_splits_in_two() {
        let mut doc = Document::new();
        let id = draw::line(&mut doc, pt(0,0), pt(10,0));
        let pieces = break_at(&mut doc, id, 0.5);
        assert_eq!(pieces.len(), 2);
        assert!(doc.get(id).is_none());
    }

    #[test]
    fn extend_line_to_line_boundary() {
        let mut doc = Document::new();
        let target = draw::line(&mut doc, pt(0,0), pt(4,0));
        let boundary = draw::line(&mut doc, pt(10,-5), pt(10,5));
        assert!(extend(&mut doc, target, boundary, 4.0, 0.0));
        if let Curve::Line(l) = doc.get(target).unwrap().as_curve().unwrap() {
            let (x, y) = l.p1.to_f64();
            assert!((x - 10.0).abs() < 1e-6 && y.abs() < 1e-6, "got ({x},{y})");
        } else { panic!() }
    }

    #[test]
    fn fillet_two_perpendicular_lines() {
        // Lines meeting at origin at 90°. Radius-2 fillet → arc center (2,2).
        let mut doc = Document::new();
        let a = draw::line(&mut doc, pt(10,0), pt(0,0));
        let b = draw::line(&mut doc, pt(0,0), pt(0,10));
        let arc_id = fillet(&mut doc, a, b, 2.0, 0.0, 0.0).expect("fillet should succeed");
        if let Curve::Arc(arc) = doc.get(arc_id).unwrap().as_curve().unwrap() {
            let (ccx, ccy) = arc.center.to_f64();
            assert!((ccx - 2.0).abs() < 1e-6 && (ccy - 2.0).abs() < 1e-6, "center ({ccx},{ccy})");
            assert!((arc.radius - 2.0).abs() < 1e-6);
        } else { panic!() }
        if let Curve::Line(l) = doc.get(a).unwrap().as_curve().unwrap() {
            let (x, y) = l.p1.to_f64();
            assert!((x - 2.0).abs() < 1e-6 && y.abs() < 1e-6, "a tangent ({x},{y})");
        } else { panic!() }
    }

    #[test]
    fn fillet_line_arc_at_shared_point() {
        // Horizontal line (10,0)→(5,0) meets quarter-arc (center=(0,0), r=5, [0,π/2])
        // at their shared point (5,0), forming a 90° corner. Fillet radius=1.
        //
        // Fillet centre lies at distance 1 from y=0 (above) and 5-1=4 from (0,0):
        //   fc = (√15, 1).  Tangent on line = (√15, 0); tangent on arc = angle ≈ 14.5°.
        // Pick inside the inner-corner area (upper-left of the shared point) so the
        // proximity test selects the correct inner candidate.
        let mut doc = Document::new();
        let line_id = draw::line(&mut doc, pt(10, 0), pt(5, 0));
        let arc_id = doc.add(EntityKind::Curve(Curve::Arc(
            CircularArc::new(pt(0, 0), r(5), 0.0, std::f64::consts::FRAC_PI_2),
        )));
        let fid = fillet(&mut doc, line_id, arc_id, 1.0, 4.0, 0.5)
            .expect("line-arc fillet should succeed");

        if let Curve::Arc(fa) = doc.get(fid).unwrap().as_curve().unwrap() {
            let (cx, cy) = fa.center.to_f64();
            assert!((cx - 15f64.sqrt()).abs() < 1e-3, "fillet cx ≈ √15, got {cx:.5}");
            assert!((cy - 1.0).abs() < 1e-3,          "fillet cy ≈ 1,   got {cy:.5}");
            assert!((fa.radius - 1.0).abs() < 1e-4);
        } else { panic!("expected Arc") }

        // Line p1 should have moved to the tangent point (√15, 0).
        if let Curve::Line(l) = doc.get(line_id).unwrap().as_curve().unwrap() {
            let (x, y) = l.p1.to_f64();
            assert!((x - 15f64.sqrt()).abs() < 1e-3, "line tangent x ≈ √15, got {x:.5}");
            assert!(y.abs() < 1e-6, "line tangent y = 0, got {y:.9}");
        } else { panic!("expected Line") }
    }

    #[test]
    fn fillet_arc_arc_at_shared_point() {
        // Arc A: centre (0,5), r=5, from −π/2 to 0  → starts at (0,0), ends at (5,5).
        // Arc B: centre (−5,0), r=5, from 0 to π/2  → starts at (0,0), ends at (−5,5).
        // Both arcs start at (0,0) forming a 90° corner. Fillet radius=1, pick near (0,0).
        //
        // Fillet centre (via arc-arc geometry): d_a=4 (inner), d_b=6 (outer), or vice-versa.
        use std::f64::consts::FRAC_PI_2;
        let mut doc = Document::new();
        let id_a = doc.add(EntityKind::Curve(Curve::Arc(
            CircularArc::new(pt(0, 5), r(5), -FRAC_PI_2, 0.0),
        )));
        let id_b = doc.add(EntityKind::Curve(Curve::Arc(
            CircularArc::new(Point2d::from_i64(-5, 0), r(5), 0.0, FRAC_PI_2),
        )));
        let fid = fillet(&mut doc, id_a, id_b, 1.0, 0.5, 0.5)
            .expect("arc-arc fillet should succeed");

        if let Curve::Arc(fa) = doc.get(fid).unwrap().as_curve().unwrap() {
            assert!((fa.radius - 1.0).abs() < 1e-4, "fillet arc radius should be 1");
            let (fx, fy) = fa.center.to_f64();
            // Fillet centre must lie on circles r±1 from each arc centre.
            let d_a = (fx.powi(2) + (fy - 5.0).powi(2)).sqrt();       // dist to centre A (0,5)
            let d_b = ((fx + 5.0).powi(2) + fy.powi(2)).sqrt();        // dist to centre B (−5,0)
            let (dlo, dhi) = if d_a < d_b { (d_a, d_b) } else { (d_b, d_a) };
            assert!((dlo - 4.0).abs() < 0.01, "near dist should be 4 (r−1), got {dlo:.4}");
            assert!((dhi - 6.0).abs() < 0.01, "far  dist should be 6 (r+1), got {dhi:.4}");
        } else { panic!("expected Arc") }
    }

    #[test]
    fn chamfer_two_perpendicular_lines() {
        let mut doc = Document::new();
        let a = draw::line(&mut doc, pt(10,0), pt(0,0));
        let b = draw::line(&mut doc, pt(0,0), pt(0,10));
        let conn = chamfer(&mut doc, a, b, 3.0, 3.0).expect("chamfer should succeed");
        if let Curve::Line(l) = doc.get(conn).unwrap().as_curve().unwrap() {
            let (x0, y0) = l.p0.to_f64();
            let (x1, y1) = l.p1.to_f64();
            let ok = ((x0-3.0).abs()<1e-6 && y0.abs()<1e-6 && x1.abs()<1e-6 && (y1-3.0).abs()<1e-6)
                  || ((x1-3.0).abs()<1e-6 && y1.abs()<1e-6 && x0.abs()<1e-6 && (y0-3.0).abs()<1e-6);
            assert!(ok, "chamfer endpoints ({x0},{y0})-({x1},{y1})");
        } else { panic!() }
    }

    #[test]
    fn stretch_moves_only_windowed_endpoints() {
        let mut doc = Document::new();
        let id = draw::line(&mut doc, pt(0,0), pt(10,0));
        stretch(&mut doc, &[id], (9.0, -1.0, 11.0, 1.0), 0.0, 5.0);
        if let Curve::Line(l) = doc.get(id).unwrap().as_curve().unwrap() {
            assert_eq!(l.p0, pt(0,0));
            let (x, y) = l.p1.to_f64();
            assert!((x - 10.0).abs() < 1e-6 && (y - 5.0).abs() < 1e-6, "stretched end ({x},{y})");
        } else { panic!() }
    }

    #[test]
    fn trim_removes_middle_piece() {
        let mut doc = Document::new();
        let target = draw::line(&mut doc, pt(0,0), pt(10,0));
        let c1 = draw::line(&mut doc, pt(3,-1), pt(3,1));
        let c2 = draw::line(&mut doc, pt(7,-1), pt(7,1));
        let survivors = trim(&mut doc, target, &[c1, c2], 5.0, 0.0);
        assert_eq!(survivors.len(), 2, "middle trimmed → 2 outer pieces");
        assert!(doc.get(target).is_none());
    }
}
