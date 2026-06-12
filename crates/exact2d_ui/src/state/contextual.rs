//! Contextual direct-manipulation actions tethered to the selection (the modern
//! "selection-first, tool-follows" interface). First feature: a corner formed by
//! two selected curves offers an interactive Fillet/Chamfer you size *visually*
//! (drag the cursor to set the radius with a live preview) instead of typing it.
//! Works for line–line, line–arc, and arc–arc corners (the exact kernel handles
//! all three); chamfer is offered only for line–line.

use exact2d_document::{EntityId, EntityKind};
use exact2d_geometry::Curve;

use super::AppState;

/// Geometry of a corner where two selected curves meet at a shared endpoint.
#[derive(Clone, Copy, Debug)]
pub struct CornerGeom {
    pub a: EntityId,
    pub b: EntityId,
    /// The shared vertex (world).
    pub corner: (f64, f64),
    /// Unit direction from the corner into curve a (tangent for arcs), and the
    /// usable length along the curve.
    pub dir_a: (f64, f64),
    pub len_a: f64,
    pub dir_b: (f64, f64),
    pub len_b: f64,
    /// Chamfer only makes sense for line–line corners.
    pub chamfer_ok: bool,
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

/// One free end of a curve: its position, the unit tangent pointing *into* the
/// curve, the usable length along the curve, and whether the curve is a line.
struct EndInfo {
    pos: (f64, f64),
    dir: (f64, f64),
    len: f64,
    is_line: bool,
}

/// Both ends of a line or (non-closed) circular arc. Other kinds → empty.
fn curve_ends(app: &AppState, id: EntityId) -> Vec<EndInfo> {
    let kind = match app.document.get(id) { Some(e) => &e.kind, None => return vec![] };
    match kind {
        EntityKind::Curve(Curve::Line(l)) => {
            let (p0, p1) = (l.p0.to_f64(), l.p1.to_f64());
            let (dx, dy) = (p1.0 - p0.0, p1.1 - p0.1);
            let len = (dx * dx + dy * dy).sqrt();
            if len < 1e-9 { return vec![]; }
            let d = (dx / len, dy / len);
            vec![
                EndInfo { pos: p0, dir: d, len, is_line: true },
                EndInfo { pos: p1, dir: (-d.0, -d.1), len, is_line: true },
            ]
        }
        EntityKind::Curve(Curve::Arc(a)) => {
            // Skip (near-)full circles — they have no free ends to fillet.
            let sweep = a.included_angle();
            if sweep >= std::f64::consts::TAU - 1e-6 { return vec![]; }
            let len = a.radius * sweep;
            if len < 1e-9 { return vec![]; }
            let (t0, t1) = (a.start_angle, a.end_angle);
            // CCW parametric tangent is (−sin t, cos t); at the start it points
            // into the arc, at the end the into-curve direction is its negative.
            vec![
                EndInfo { pos: a.start_point(), dir: (-t0.sin(), t0.cos()), len, is_line: false },
                EndInfo { pos: a.end_point(),   dir: (t1.sin(), -t1.cos()), len, is_line: false },
            ]
        }
        _ => vec![],
    }
}

impl AppState {
    /// Detect every fillettable corner in the selection: any two selected
    /// lines/arcs sharing an endpoint where the tangents are not (near-)collinear.
    /// With a whole chain of segments selected this returns one grip per corner.
    pub fn detect_corners(&self) -> Vec<CornerGeom> {
        // O(n²) over the selection — cap it so a huge selection stays cheap.
        if self.selection.len() < 2 || self.selection.len() > 24 { return vec![]; }
        let ends: Vec<(EntityId, Vec<EndInfo>)> = self.selection.iter()
            .map(|&id| (id, curve_ends(self, id)))
            .collect();

        let tol = 1e-6;
        let mut out = Vec::new();
        for i in 0..ends.len() {
            for j in (i + 1)..ends.len() {
                let (a, ea) = &ends[i];
                let (b, eb) = &ends[j];
                for fa in ea {
                    for fb in eb {
                        if (fa.pos.0 - fb.pos.0).hypot(fa.pos.1 - fb.pos.1) >= tol { continue; }
                        let cos = fa.dir.0 * fb.dir.0 + fa.dir.1 * fb.dir.1;
                        if cos.abs() > 0.999 { continue; } // tangent/collinear join
                        out.push(CornerGeom {
                            a: *a, b: *b,
                            corner: fa.pos,
                            dir_a: fa.dir, len_a: fa.len,
                            dir_b: fb.dir, len_b: fb.len,
                            chamfer_ok: fa.is_line && fb.is_line,
                        });
                    }
                }
            }
        }
        out
    }

    /// Begin a corner action (Inventor-style combined grip). Kind and size are then
    /// driven by cursor direction/distance via `update_corner_drag`.
    pub fn begin_corner_action(&mut self, geom: CornerGeom) {
        let size = (geom.max_size(CornerKind::Fillet) * 0.3).max(1e-3);
        self.corner_action = Some(CornerAction { geom, kind: CornerKind::Fillet, size });
    }

    /// Update the in-progress action from the cursor: moving to the **right** of the
    /// corner chooses Fillet, to the **left** chooses Chamfer; distance sets the size.
    /// Corners involving an arc only offer Fillet.
    pub fn update_corner_drag(&mut self) {
        if let Some(mut ca) = self.corner_action {
            let (cx, cy) = ca.geom.corner;
            ca.kind = if !ca.geom.chamfer_ok || self.cursor_world.0 >= cx {
                CornerKind::Fillet
            } else {
                CornerKind::Chamfer
            };
            let d = (self.cursor_world.0 - cx).hypot(self.cursor_world.1 - cy);
            ca.size = d.clamp(1e-3, ca.geom.max_size(ca.kind));
            self.corner_action = Some(ca);
        }
    }

    /// Override the in-progress size with a typed value (clamped to what fits).
    pub fn set_corner_size(&mut self, val: f64) {
        if let Some(mut ca) = self.corner_action {
            ca.size = val.clamp(1e-3, ca.geom.max_size(ca.kind));
            self.corner_action = Some(ca);
        }
    }

    /// Apply the in-progress corner action via the exact kernel, then clear it.
    pub fn apply_corner_action(&mut self) {
        if let Some(ca) = self.corner_action.take() {
            self.history.snapshot(&self.document);
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
