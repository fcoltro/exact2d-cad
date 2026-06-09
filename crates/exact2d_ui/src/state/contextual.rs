//! Contextual direct-manipulation actions tethered to the selection (the modern
//! "selection-first, tool-follows" interface). First feature: a corner formed by
//! two selected lines offers an interactive Fillet/Chamfer you size *visually*
//! (drag the cursor to set the radius with a live preview) instead of typing it.

use exact2d_document::{EntityId, EntityKind};
use exact2d_geometry::Curve;

use super::AppState;

/// Geometry of a corner where two selected lines meet at a shared endpoint.
#[derive(Clone, Copy, Debug)]
pub struct CornerGeom {
    pub a: EntityId,
    pub b: EntityId,
    /// The shared vertex (world).
    pub corner: (f64, f64),
    /// Unit direction from the corner toward line a's other end, and its length.
    pub dir_a: (f64, f64),
    pub len_a: f64,
    pub dir_b: (f64, f64),
    pub len_b: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CornerKind { Fillet, Chamfer }

/// An in-progress, visually-sized corner action.
#[derive(Clone, Copy, Debug)]
pub struct CornerAction {
    pub geom: CornerGeom,
    pub kind: CornerKind,
    /// Current radius (fillet) or setback distance (chamfer), updated live.
    pub size: f64,
}

impl CornerGeom {
    /// Largest valid `size` for `kind` (so the trim fits within both lines).
    pub fn max_size(&self, kind: CornerKind) -> f64 {
        let min_len = self.len_a.min(self.len_b);
        match kind {
            CornerKind::Chamfer => min_len * 0.98,
            CornerKind::Fillet => {
                // tangent length t = r / tan(θ/2) must be ≤ min_len ⇒ r ≤ min_len·tan(θ/2)
                let half = self.interior_angle() * 0.5;
                (min_len * half.tan()).max(1e-6)
            }
        }
    }

    /// Interior angle between the two edges at the corner (radians).
    pub fn interior_angle(&self) -> f64 {
        let cos = (self.dir_a.0 * self.dir_b.0 + self.dir_a.1 * self.dir_b.1).clamp(-1.0, 1.0);
        cos.acos()
    }
}

/// A world point (x, y).
type Pt = (f64, f64);

/// The fillet arc tangent to both edges: returns (tangent point on a, tangent
/// point on b, arc centre). `None` for a degenerate (near-straight) corner.
pub fn fillet_arc(corner: Pt, da: Pt, db: Pt, r: f64) -> Option<(Pt, Pt, Pt)> {
    let cos = (da.0 * db.0 + da.1 * db.1).clamp(-1.0, 1.0);
    let half = cos.acos() * 0.5;
    let s = half.sin();
    let tan = half.tan();
    if s < 1e-6 || !tan.is_finite() || tan.abs() < 1e-9 {
        return None;
    }
    let t = r / tan;
    let p1 = (corner.0 + da.0 * t, corner.1 + da.1 * t);
    let p2 = (corner.0 + db.0 * t, corner.1 + db.1 * t);
    let (mut bx, mut by) = (da.0 + db.0, da.1 + db.1);
    let bl = (bx * bx + by * by).sqrt();
    if bl < 1e-9 { return None; }
    bx /= bl; by /= bl;
    let d = r / s;
    Some((p1, p2, (corner.0 + bx * d, corner.1 + by * d)))
}

fn line_ends(app: &AppState, id: EntityId) -> Option<((f64, f64), (f64, f64))> {
    match &app.document.get(id)?.kind {
        EntityKind::Curve(Curve::Line(l)) => Some((l.p0.to_f64(), l.p1.to_f64())),
        _ => None,
    }
}

impl AppState {
    /// Detect a fillettable/chamferable corner: exactly two selected line entities
    /// sharing an endpoint, and not (near-)collinear.
    pub fn detect_corner(&self) -> Option<CornerGeom> {
        if self.selection.len() != 2 { return None; }
        let (a, b) = (self.selection[0], self.selection[1]);
        let (a0, a1) = line_ends(self, a)?;
        let (b0, b1) = line_ends(self, b)?;

        // Find the shared endpoint (the corner) and each line's far end.
        let tol = 1e-6;
        let near = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).hypot(p.1 - q.1) < tol;
        let (corner, oa, ob) = if near(a0, b0) { (a0, a1, b1) }
            else if near(a0, b1) { (a0, a1, b0) }
            else if near(a1, b0) { (a1, a0, b1) }
            else if near(a1, b1) { (a1, a0, b0) }
            else { return None };

        let mk = |o: (f64, f64)| {
            let (dx, dy) = (o.0 - corner.0, o.1 - corner.1);
            let len = (dx * dx + dy * dy).sqrt();
            if len < 1e-9 { None } else { Some(((dx / len, dy / len), len)) }
        };
        let (dir_a, len_a) = mk(oa)?;
        let (dir_b, len_b) = mk(ob)?;

        let cos = dir_a.0 * dir_b.0 + dir_a.1 * dir_b.1;
        if cos.abs() > 0.999 { return None; } // collinear → no real corner

        Some(CornerGeom { a, b, corner, dir_a, len_a, dir_b, len_b })
    }

    /// Begin a visually-sized corner action with a sensible initial size.
    pub fn begin_corner_action(&mut self, geom: CornerGeom, kind: CornerKind) {
        let size = (geom.max_size(kind) * 0.5).max(1e-3);
        self.corner_action = Some(CornerAction { geom, kind, size });
    }

    /// Update the in-progress size from the cursor's distance to the corner.
    pub fn update_corner_size(&mut self) {
        if let Some(mut ca) = self.corner_action {
            let (cx, cy) = ca.geom.corner;
            let d = (self.cursor_world.0 - cx).hypot(self.cursor_world.1 - cy);
            ca.size = d.clamp(1e-3, ca.geom.max_size(ca.kind));
            self.corner_action = Some(ca);
        }
    }

    /// Apply the in-progress corner action via the exact kernel, then clear it.
    pub fn apply_corner_action(&mut self) {
        if let Some(ca) = self.corner_action.take() {
            self.history.snapshot(&self.document, &self.sketch, &self.entity_points);
            match ca.kind {
                CornerKind::Fillet => {
                    exact2d_cad::edit::fillet(
                        &mut self.document, ca.geom.a, ca.geom.b, ca.size,
                        ca.geom.corner.0, ca.geom.corner.1);
                }
                CornerKind::Chamfer => {
                    exact2d_cad::edit::chamfer(
                        &mut self.document, ca.geom.a, ca.geom.b, ca.size, ca.size);
                }
            }
            self.resync_after_edit();
            self.selection.clear();
        }
    }

    pub fn cancel_corner_action(&mut self) {
        self.corner_action = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fillet_arc_right_angle() {
        // Corner at origin, edges along +x and +y, radius 2 → quarter circle
        // tangent at (2,0) and (0,2), centre (2,2).
        let (p1, p2, c) = fillet_arc((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), 2.0).unwrap();
        assert!((p1.0 - 2.0).abs() < 1e-9 && p1.1.abs() < 1e-9);
        assert!(p2.0.abs() < 1e-9 && (p2.1 - 2.0).abs() < 1e-9);
        assert!((c.0 - 2.0).abs() < 1e-9 && (c.1 - 2.0).abs() < 1e-9);
    }

    #[test]
    fn fillet_arc_rejects_straight() {
        // Opposite directions (a straight line) has no corner.
        assert!(fillet_arc((0.0, 0.0), (1.0, 0.0), (-1.0, 0.0), 1.0).is_none());
    }
}
