//! Selection (spec §4.2): single-click, window, crossing, fence, and property-based.

use exact2d_geometry::{Curve, CurveSegment, BoundingBox, Point2d, LineSeg, intersect};
use exact2d_document::{Document, EntityId, EntityKind};

/// Pick the topmost entity under a point (single click), within `tol`.
/// Iterates in reverse draw order so the most-recently-added wins.
pub fn pick_at(doc: &Document, x: f64, y: f64, tol: f64) -> Option<EntityId> {
    for e in doc.editable_entities().collect::<Vec<_>>().into_iter().rev() {
        match &e.kind {
            EntityKind::Curve(c) => {
                let pr = exact2d_geometry::project_point_onto_curve(c, x, y);
                if pr.distance <= tol { return Some(e.id); }
            }
            EntityKind::Point(p) => {
                let (px, py) = p.to_f64();
                if ((px - x).powi(2) + (py - y).powi(2)).sqrt() <= tol { return Some(e.id); }
            }
            _ => {}
        }
    }
    None
}

/// Window selection: entities **fully inside** the rectangle.
pub fn select_window(doc: &Document, rect: &BoundingBox) -> Vec<EntityId> {
    doc.editable_entities()
        .filter(|e| e.bounding_box().is_some_and(|bb| bbox_inside(&bb, rect)))
        .map(|e| e.id)
        .collect()
}

/// Crossing selection: entities **inside or touching** the rectangle.
pub fn select_crossing(doc: &Document, rect: &BoundingBox) -> Vec<EntityId> {
    doc.editable_entities()
        .filter(|e| match &e.kind {
            EntityKind::Curve(c) => curve_touches_rect(c, rect),
            _ => e.bounding_box().is_some_and(|bb| bb.intersects(rect)),
        })
        .map(|e| e.id)
        .collect()
}

/// Fence selection: entities crossed by an open polyline fence.
pub fn select_fence(doc: &Document, fence: &[Point2d]) -> Vec<EntityId> {
    if fence.len() < 2 { return vec![]; }
    let segs: Vec<LineSeg> = fence.windows(2)
        .map(|w| LineSeg::from_endpoints(w[0].clone(), w[1].clone()))
        .collect();

    doc.editable_entities()
        .filter(|e| {
            if let EntityKind::Curve(c) = &e.kind {
                segs.iter().any(|s| !intersect(&Curve::Line(s.clone()), c).is_empty())
            } else { false }
        })
        .map(|e| e.id)
        .collect()
}

/// Select all entities matching a predicate (Quick Select / Select Similar).
pub fn select_by<F: Fn(&exact2d_document::Entity) -> bool>(doc: &Document, pred: F) -> Vec<EntityId> {
    doc.editable_entities().filter(|e| pred(e)).map(|e| e.id).collect()
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

fn bbox_inside(inner: &BoundingBox, outer: &BoundingBox) -> bool {
    inner.min.x >= outer.min.x && inner.max.x <= outer.max.x &&
    inner.min.y >= outer.min.y && inner.max.y <= outer.max.y
}

/// Whether a curve is inside or touches the rectangle (for crossing selection).
fn curve_touches_rect(c: &Curve, rect: &BoundingBox) -> bool {
    // Quick accept: bbox disjoint → no.
    if !c.bounding_box().intersects(rect) { return false; }
    // Accept if any sampled point is inside the rect (handles fully-inside + crossing).
    let (t0, t1) = c.domain();
    for i in 0..=8 {
        let t = t0 + (t1 - t0) * i as f64 / 8.0;
        let (x, y) = c.evaluate_f64(t);
        if rect.contains_point_f64(x, y) { return true; }
    }
    // Else check edge intersections with the 4 rectangle sides.
    let (x0, y0) = rect.min.to_f64();
    let (x1, y1) = rect.max.to_f64();
    let corners = [
        Point2d::from_f64(x0, y0), Point2d::from_f64(x1, y0),
        Point2d::from_f64(x1, y1), Point2d::from_f64(x0, y1),
    ];
    for i in 0..4 {
        let side = Curve::Line(LineSeg::from_endpoints(
            corners[i].clone(), corners[(i + 1) % 4].clone()));
        if !intersect(&side, c).is_empty() { return true; }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_document::EntityKind;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }
    fn line(x0: i64, y0: i64, x1: i64, y1: i64) -> EntityKind {
        EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(x0,y0), pt(x1,y1))))
    }

    fn sample_doc() -> (Document, EntityId, EntityId) {
        let mut doc = Document::new();
        let a = doc.add(line(1, 1, 3, 3));     // fully inside [0,5]
        let b = doc.add(line(4, 4, 8, 8));     // crosses the [0,5] boundary
        (doc, a, b)
    }

    #[test]
    fn window_selects_only_fully_inside() {
        let (doc, a, b) = sample_doc();
        let rect = BoundingBox::from_corners(0.0, 0.0, 5.0, 5.0);
        let sel = select_window(&doc, &rect);
        assert!(sel.contains(&a));
        assert!(!sel.contains(&b), "partially-outside entity must not be window-selected");
    }

    #[test]
    fn crossing_selects_touching() {
        let (doc, a, b) = sample_doc();
        let rect = BoundingBox::from_corners(0.0, 0.0, 5.0, 5.0);
        let sel = select_crossing(&doc, &rect);
        assert!(sel.contains(&a));
        assert!(sel.contains(&b), "crossing entity must be selected");
    }

    #[test]
    fn pick_at_finds_curve() {
        let (doc, a, _) = sample_doc();
        // Point on the line (1,1)-(3,3): (2,2)
        assert_eq!(pick_at(&doc, 2.0, 2.0, 0.1), Some(a));
        // Far away → nothing
        assert_eq!(pick_at(&doc, 100.0, 100.0, 0.1), None);
    }

    #[test]
    fn fence_crosses_entities() {
        let (doc, _a, b) = sample_doc();
        // Fence from (5,3) to (5,9) — a vertical line crossing segment b (4,4)-(8,8)
        let fence = vec![pt(5, 3), pt(5, 9)];
        let sel = select_fence(&doc, &fence);
        assert!(sel.contains(&b));
    }

    #[test]
    fn select_by_layer() {
        let mut doc = Document::new();
        doc.layers.add(exact2d_document::Layer::new("special"));
        let special_idx = doc.layers.index_of("special").unwrap();
        doc.add(line(0,0,1,1)); // layer 0
        let s = doc.add_on_layer(line(2,2,3,3), special_idx);
        let sel = select_by(&doc, |e| e.layer == special_idx);
        assert_eq!(sel, vec![s]);
    }

    #[test]
    fn pick_respects_layer_lock() {
        let mut doc = Document::new();
        let id = doc.add(line(0, 0, 4, 0));
        // Lock layer 0 → entity no longer pickable
        doc.layers.get_mut(0).unwrap().locked = true;
        assert_eq!(pick_at(&doc, 2.0, 0.0, 0.1), None);
        let _ = id;
    }
}
