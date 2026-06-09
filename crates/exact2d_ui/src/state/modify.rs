//! Interactive modify-tool click handling (TRIM/EXTEND/OFFSET/FILLET/CHAMFER/STRETCH).
//!
//! These tools pick entities and call the `exact2d_cad::edit` kernel ops, so unlike
//! the pure `Tool` state machines they need document access and live on `AppState`.

use super::AppState;
use exact2d_geometry::Point2d;
use exact2d_document::EntityId;
use exact2d_cad::pick_at;
use crate::tools::Tool;

impl AppState {
    /// Post-edit hook. (Previously resynced the parametric sketch; now a no-op,
    /// kept as the single place to re-add any post-modify housekeeping.)
    pub(crate) fn resync_after_edit(&mut self) {}

    /// Handle a click for the entity-picking modify tools. Returns true if the click
    /// was consumed (so `canvas_click` should stop). These tools need document access
    /// for picking and the kernel edit ops, so they live here rather than in `Tool`.
    pub(crate) fn handle_modify_click(&mut self, p: &Point2d) -> bool {
        use exact2d_cad::edit;
        let px = p.x.to_f64();
        let py = p.y.to_f64();
        let tol = self.view.pixel_world_size() * 6.0;
        let pick = |s: &Self| pick_at(&s.document, px, py, tol).filter(|&id| id != s.origin_id);

        match self.tool.clone() {
            Tool::Trim => {
                if let Some(id) = pick(self) {
                    self.history.snapshot(&self.document);
                    let cutters: Vec<EntityId> = self.document.iter().map(|e| e.id)
                        .filter(|&i| i != id && i != self.origin_id).collect();
                    edit::trim(&mut self.document, id, &cutters, px, py);
                    self.selection.clear();
                    self.resync_after_edit();
                }
                true
            }
            Tool::Extend => {
                if let Some(id) = pick(self) {
                    // Try boundaries nearest the pick first; stop at the first that reaches.
                    let mut bs: Vec<(f64, EntityId)> = self.document.iter()
                        .filter(|e| e.id != id && e.id != self.origin_id)
                        .filter_map(|e| e.as_curve()
                            .map(|c| (exact2d_geometry::point_to_curve_distance(c, px, py), e.id)))
                        .collect();
                    bs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
                    self.history.snapshot(&self.document);
                    let mut done = false;
                    for (_, bid) in bs {
                        if edit::extend(&mut self.document, id, bid, px, py) { done = true; break; }
                    }
                    if done { self.resync_after_edit(); } else { self.history.discard_last(); }
                }
                true
            }
            Tool::Offset { dist, source } => {
                match source {
                    None => {
                        if let Some(id) = pick(self) {
                            self.tool = Tool::Offset { dist, source: Some(id) };
                        }
                    }
                    Some(src) => {
                        if let Some(c) = self.document.get(src).and_then(|e| e.as_curve()).cloned() {
                            // Pick the side (sign) whose offset lands nearer the cursor.
                            let plus = exact2d_geometry::offset_curve(&c, dist.abs());
                            let minus = exact2d_geometry::offset_curve(&c, -dist.abs());
                            let dp = exact2d_geometry::point_to_curve_distance(&plus, px, py);
                            let dm = exact2d_geometry::point_to_curve_distance(&minus, px, py);
                            let signed = if dp <= dm { dist.abs() } else { -dist.abs() };
                            self.history.snapshot(&self.document);
                            edit::offset(&mut self.document, &[src], signed);
                            self.resync_after_edit();
                        }
                        self.tool = Tool::Offset { dist, source: None };
                    }
                }
                true
            }
            Tool::Fillet { radius, first } => {
                if let Some(id) = pick(self) {
                    match first {
                        None => self.tool = Tool::Fillet { radius, first: Some(id) },
                        Some(a) => {
                            if a != id {
                                self.history.snapshot(&self.document);
                                edit::fillet(&mut self.document, a, id, radius, px, py);
                                self.resync_after_edit();
                            }
                            self.tool = Tool::Fillet { radius, first: None };
                        }
                    }
                }
                true
            }
            Tool::Chamfer { dist, first } => {
                if let Some(id) = pick(self) {
                    match first {
                        None => self.tool = Tool::Chamfer { dist, first: Some(id) },
                        Some(a) => {
                            if a != id {
                                self.history.snapshot(&self.document);
                                edit::chamfer(&mut self.document, a, id, dist, dist);
                                self.resync_after_edit();
                            }
                            self.tool = Tool::Chamfer { dist, first: None };
                        }
                    }
                }
                true
            }
            Tool::Stretch { c1, c2, base, ids } => {
                match (c1, c2, base) {
                    (None, _, _) => {
                        let ids = if self.selection.is_empty() {
                            self.document.iter().map(|e| e.id).filter(|&i| i != self.origin_id).collect()
                        } else { self.selection.clone() };
                        self.tool = Tool::Stretch { c1: Some(p.clone()), c2: None, base: None, ids };
                    }
                    (Some(a), None, _) =>
                        self.tool = Tool::Stretch { c1: Some(a), c2: Some(p.clone()), base: None, ids },
                    (Some(a), Some(b), None) =>
                        self.tool = Tool::Stretch { c1: Some(a), c2: Some(b), base: Some(p.clone()), ids },
                    (Some(a), Some(b), Some(bp)) => {
                        let (ax, ay) = a.to_f64();
                        let (bx, by) = b.to_f64();
                        let window = (ax.min(bx), ay.min(by), ax.max(bx), ay.max(by));
                        let dx = px - bp.x.to_f64();
                        let dy = py - bp.y.to_f64();
                        self.history.snapshot(&self.document);
                        edit::stretch(&mut self.document, &ids, window, dx, dy);
                        self.resync_after_edit();
                        self.tool = Tool::Stretch { c1: None, c2: None, base: None, ids: vec![] };
                    }
                }
                true
            }
            _ => false,
        }
    }
}
