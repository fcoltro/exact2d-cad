//! The headless application state (spec §6.1) — the testable "brain" the egui view
//! drives. Owns the document, view, active tool, selection, snap settings, status
//! toggles, command log, and undo/redo history. No egui or GPU dependencies.

use exact2d_algebra::Rational;
use exact2d_geometry::{Point2d, Curve};
use exact2d_document::{Document, EntityKind, EntityId, Layer};
use exact2d_cad::{SnapSettings, SnapPoint, SnapKind, best_snap, pick_at};
use exact2d_constraint::{Sketch, Constraint, PointId};
use std::collections::HashMap;

use crate::view_transform::ViewTransform;
use crate::tools::{Tool, ToolEvent};
use crate::command::{Command, parse_command, parse_coordinate, CoordInput};
use crate::history::History;

/// Interactive modify-tool click handling (TRIM/EXTEND/OFFSET/FILLET/CHAMFER/STRETCH).
mod modify;
/// Parametric-constraint integration (sketch sync, solve cycle, add_constraint).
mod constraints;

pub struct AppState {
    pub document: Document,
    pub view: ViewTransform,
    pub tool: Tool,
    pub selection: Vec<EntityId>,
    pub snap: SnapSettings,
    pub snap_on: bool,
    pub grid_on: bool,
    pub ortho_on: bool,
    /// Polar tracking (45° increments). Ignored while Ortho is on.
    pub polar_on: bool,
    /// Dynamic input: cursor-side length/angle/coordinate tooltip.
    pub dyn_on: bool,
    /// Last executed command text, for right-click / Enter repeat.
    pub last_command: Option<String>,
    pub history: History,
    pub command_log: Vec<String>,
    /// Last cursor position in world coordinates (status-bar readout).
    pub cursor_world: (f64, f64),
    /// The snap currently under the cursor, if any (for the snap marker).
    pub active_snap: Option<SnapPoint>,
    /// Number of canvas clicks received (diagnostic HUD; proves input is live).
    pub click_count: u32,
    /// Active Ortho or Polar tracking guide: Some(((rx, ry), angle_radians))
    pub active_guide: Option<((f64, f64), f64)>,

    // Parametric constraints solver integration
    pub constraints_enabled: bool,
    pub sketch: Sketch,
    pub entity_points: HashMap<EntityId, Vec<PointId>>,
    pub origin_id: EntityId,
    /// Snap-to-endpoint connections recorded during the current draw (parametric
    /// mode only), materialized into Coincident constraints when the entity is
    /// created (Stage 3). Each entry is (snapped world x, y, the snapped-to entity).
    pending_snap_links: Vec<(f64, f64, EntityId)>,

    /// Path of the currently open file, if any.
    pub current_file_path: Option<std::path::PathBuf>,
}

impl AppState {
    pub fn new(canvas_w: f64, canvas_h: f64) -> Self {
        let mut document = Document::new();
        let origin_id = document.add(EntityKind::Point(Point2d::from_i64(0, 0)));

        let app = AppState {
            document,
            view: ViewTransform::new(canvas_w, canvas_h),
            tool: Tool::Select,
            selection: Vec::new(),
            snap: SnapSettings::default(),
            snap_on: true,
            grid_on: true,
            ortho_on: false,
            polar_on: true,
            dyn_on: true,
            last_command: None,
            history: History::new(),
            command_log: Vec::new(),
            cursor_world: (0.0, 0.0),
            active_snap: None,
            click_count: 0,
            active_guide: None,

            constraints_enabled: false,
            sketch: Sketch::new(),
            entity_points: HashMap::new(),
            origin_id,
            pending_snap_links: Vec::new(),

            current_file_path: None,
        };
        // No sketch overlay at startup: parametric mode is OFF by default (free
        // drafting on the exact document). The overlay is built on demand by
        // enter_parametric(); see docs/constraint-refactor-plan.md.
        app
    }

    // ── Pointer input ─────────────────────────────────────────────────────────

    /// Update the cursor position (screen pixels) and recompute the active snap.
    pub fn pointer_moved(&mut self, sx: f64, sy: f64) {
        let (wx, wy) = self.view.screen_to_world(sx, sy);
        
        self.active_snap = if self.snap_on {
            let mut s = self.snap.clone();
            s.tolerance = self.view.pixel_world_size() * 6.0; // ~6px (tighter snap grip)
            let ref_pt = self.tool.reference_point().map(|p| p.to_f64());
            best_snap(&self.document, (wx, wy), &s, ref_pt)
        } else {
            None
        };

        self.active_guide = None;

        if let Some(ref sp) = self.active_snap {
            self.cursor_world = sp.pos;
        } else if self.ortho_on {
            if let Some(ref_pt) = self.tool.reference_point() {
                let (rx, ry) = ref_pt.to_f64();
                let dx = wx - rx;
                let dy = wy - ry;
                let angle_rad = if dx.abs() >= dy.abs() {
                    self.cursor_world = (wx, ry);
                    if wx >= rx { 0.0 } else { std::f64::consts::PI }
                } else {
                    self.cursor_world = (rx, wy);
                    if wy >= ry { std::f64::consts::FRAC_PI_2 } else { -std::f64::consts::FRAC_PI_2 }
                };
                self.active_guide = Some(((rx, ry), angle_rad));
            } else {
                self.cursor_world = (wx, wy);
            }
        } else {
            if let Some(ref_pt) = self.tool.reference_point() {
                let (rx, ry) = ref_pt.to_f64();
                let dx = wx - rx;
                let dy = wy - ry;
                let dist = (dx * dx + dy * dy).sqrt();
                if self.polar_on && dist > 1e-4 {
                    let angle_rad = dy.atan2(dx);
                    let angle_deg = angle_rad.to_degrees();
                    let angle_deg_wrapped = if angle_deg < 0.0 { angle_deg + 360.0 } else { angle_deg };
                    let nearest_45 = (angle_deg_wrapped / 45.0).round() * 45.0;
                    let diff = (angle_deg_wrapped - nearest_45).abs();
                    let diff = diff.min(360.0 - diff);
                    
                    if diff <= 3.0 {
                        let snapped_rad = nearest_45.to_radians();
                        self.cursor_world = (rx + dist * snapped_rad.cos(), ry + dist * snapped_rad.sin());
                        self.active_guide = Some(((rx, ry), snapped_rad));
                    } else {
                        self.cursor_world = (wx, wy);
                    }
                } else {
                    self.cursor_world = (wx, wy);
                }
            } else {
                self.cursor_world = (wx, wy);
            }
        }
    }

    /// The world point a click resolves to (snapped if a snap is active).
    pub fn resolved_point(&self) -> Point2d {
        match &self.active_snap {
            Some(sp) => Point2d::from_f64(sp.pos.0, sp.pos.1),
            None => Point2d::from_f64(self.cursor_world.0, self.cursor_world.1),
        }
    }

    /// Handle a left click on the canvas at screen pixel (sx, sy).
    pub fn canvas_click(&mut self, sx: f64, sy: f64) {
        self.click_count = self.click_count.wrapping_add(1);
        self.pointer_moved(sx, sy);
        let p = self.resolved_point();

        // Entity-picking modify tools (TRIM/EXTEND/OFFSET/FILLET/CHAMFER/STRETCH).
        if self.handle_modify_click(&p) {
            return;
        }

        // DIMENSION tool: click points/entities, then click to place.
        if let Tool::Dimension { stage, p1, p2 } = self.tool.clone() {
            // The dimension tool creates a parametric (dimensional) constraint, so
            // using it enters parametric mode and builds the sketch overlay.
            if stage == 0 {
                self.enter_parametric();
            }
            let px = p.x.to_f64();
            let py = p.y.to_f64();
            if stage == 0 {
                let mut best_pt = None;
                let mut min_d = self.view.pixel_world_size() * 10.0;
                for pt_id in 0..self.sketch.num_points() {
                    let (pt_x, pt_y) = self.sketch.point(pt_id);
                    let d = ((pt_x - px).powi(2) + (pt_y - py).powi(2)).sqrt();
                    if d < min_d {
                        min_d = d;
                        best_pt = Some(pt_id);
                    }
                }
                if let Some(pt_id) = best_pt {
                    self.tool = Tool::Dimension { stage: 1, p1: Some(pt_id), p2: None };
                } else if let Some(ent_id) = pick_at(&self.document, px, py, self.view.pixel_world_size() * 6.0) {
                    if let Some(pts) = self.entity_points.get(&ent_id) {
                        // A line (2 points) or an arc (center + 2 endpoints) both
                        // dimension across their first two points.
                        if pts.len() == 2 || pts.len() == 3 {
                            self.tool = Tool::Dimension { stage: 2, p1: Some(pts[0]), p2: Some(pts[1]) };
                        }
                    }
                }
            } else if stage == 1 {
                let mut best_pt = None;
                let mut min_d = self.view.pixel_world_size() * 10.0;
                for pt_id in 0..self.sketch.num_points() {
                    let (pt_x, pt_y) = self.sketch.point(pt_id);
                    let d = ((pt_x - px).powi(2) + (pt_y - py).powi(2)).sqrt();
                    if d < min_d {
                        min_d = d;
                        best_pt = Some(pt_id);
                    }
                }
                if let Some(pt_id) = best_pt {
                    self.tool = Tool::Dimension { stage: 2, p1, p2: Some(pt_id) };
                }
            } else if stage == 2 {
                if let (Some(pt1), Some(pt2)) = (p1, p2) {
                    self.enter_parametric(); // no-op: entered at stage 0
                    self.history.snapshot(&self.document, &self.sketch, &self.entity_points);
                    
                    let (x1, y1) = self.sketch.point(pt1);
                    let (x2, y2) = self.sketch.point(pt2);
                    
                    let is_arc = {
                        let mut found = false;
                        for e in self.document.iter() {
                            if let EntityKind::Curve(Curve::Arc(_)) = &e.kind {
                                if let Some(pts) = self.entity_points.get(&e.id) {
                                    if pts.len() >= 2 && pts[0] == pt1 && pts[1] == pt2 {
                                        found = true;
                                        break;
                                    }
                                }
                            }
                        }
                        found
                    };
                    
                    if is_arc {
                        let d = ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt();
                        self.sketch.add_constraint(Constraint::Distance(pt1, pt2, d));
                    } else {
                        let mid_x = (x1 + x2) / 2.0;
                        let mid_y = (y1 + y2) / 2.0;
                        let angle_deg = (py - mid_y).atan2(px - mid_x).to_degrees().abs();
                        
                        if angle_deg > 67.5 && angle_deg < 112.5 {
                            self.sketch.add_constraint(Constraint::DistanceX(pt1, pt2, (x1 - x2).abs()));
                        } else if !(22.5..=157.5).contains(&angle_deg) {
                            self.sketch.add_constraint(Constraint::DistanceY(pt1, pt2, (y1 - y2).abs()));
                        } else {
                            let d = ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt();
                            self.sketch.add_constraint(Constraint::Distance(pt1, pt2, d));
                        }
                    }
                    self.solve_constraints();
                }
                self.tool = Tool::Select;
            }
            return;
        }

        // SELECT tool: pick the entity under the cursor and toggle it.
        if matches!(self.tool, Tool::Select) {
            if let Some(id) = pick_at(&self.document, p.x.to_f64(), p.y.to_f64(),
                                      self.view.pixel_world_size() * 6.0) {
                self.toggle_selection(id);
            } else {
                self.selection.clear();
            }
            return;
        }

        // Drawing/edit tool: feed the point and apply the resulting event.
        // In parametric mode, remember if this point snapped onto an existing
        // endpoint/node — it becomes a Coincident constraint when the entity is
        // created, so snap-drawn geometry stays connected (Stage 3, by intent).
        if self.constraints_enabled {
            if let Some(sp) = self.active_snap.clone() {
                if matches!(sp.kind, SnapKind::Endpoint | SnapKind::Node) {
                    self.pending_snap_links.push((sp.pos.0, sp.pos.1, sp.entity));
                }
            }
        }
        let ev = self.tool.on_point(p);
        self.apply_tool_event(ev);
    }

    fn apply_tool_event(&mut self, ev: ToolEvent) {
        match ev {
            ToolEvent::Pending => {}
            ToolEvent::Create(kinds) => {
                self.history.snapshot(&self.document, &self.sketch, &self.entity_points);
                let new_ids: Vec<EntityId> = kinds.into_iter().map(|k| self.document.add(k)).collect();
                if self.constraints_enabled {
                    self.sync_sketch_from_document();
                    // Turn snap-to-endpoint picks made while drawing into Coincident
                    // constraints linking the new geometry to what it snapped onto.
                    self.materialize_snap_links(&new_ids);
                    self.solve_constraints();
                }
                self.pending_snap_links.clear();
            }
            ToolEvent::Transform { ids, t } => {
                self.pending_snap_links.clear();
                self.history.snapshot(&self.document, &self.sketch, &self.entity_points);
                if self.constraints_enabled {
                    let mut moved_pts = std::collections::HashSet::new();
                    for id in &ids {
                        if *id != self.origin_id {
                            if let Some(pts) = self.entity_points.get(id) {
                                for &pt in pts {
                                    moved_pts.insert(pt);
                                }
                            }
                        }
                    }
                    let m00 = t.m00.to_f64();
                    let m01 = t.m01.to_f64();
                    let tx  = t.tx.to_f64();
                    let m10 = t.m10.to_f64();
                    let m11 = t.m11.to_f64();
                    let ty  = t.ty.to_f64();
                    for pt in moved_pts {
                        let (x, y) = self.sketch.point(pt);
                        let nx = m00 * x + m01 * y + tx;
                        let ny = m10 * x + m11 * y + ty;
                        self.sketch.set_point(pt, nx, ny);
                    }
                    self.solve_constraints();
                } else {
                    for id in ids {
                        if id != self.origin_id {
                            if let Some(e) = self.document.get_mut(id) { e.transform(&t); }
                        }
                    }
                }
                self.tool = Tool::Select;
            }
            ToolEvent::CopyOf { ids, t } => {
                self.pending_snap_links.clear();
                self.history.snapshot(&self.document, &self.sketch, &self.entity_points);
                for id in ids {
                    if id != self.origin_id {
                        if let Some(e) = self.document.get(id) {
                            let copy = e.transformed(&t);
                            self.document.add_entity(copy);
                        }
                    }
                }
                self.tool = Tool::Select;
                if self.constraints_enabled {
                    self.sync_sketch_from_document();
                    self.solve_constraints();
                }
            }
        }
    }

    fn toggle_selection(&mut self, id: EntityId) {
        if id == self.origin_id {
            return;
        }
        if let Some(pos) = self.selection.iter().position(|&s| s == id) {
            self.selection.remove(pos);
        } else {
            self.selection.push(id);
        }
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /// Parse and run a command-line string.
    pub fn run_command(&mut self, text: &str) {
        let trimmed = text.trim();

        // 1. Intercept Polyline commit / close commands
        if let Tool::Polyline { .. } = self.tool {
            if trimmed.is_empty() {
                let ev = self.tool.commit();
                self.apply_tool_event(ev);
                self.tool = Tool::Select;
                return;
            }
            let upper = trimmed.to_ascii_uppercase();
            if upper == "C" || upper == "CLOSE" {
                let ev = self.tool.close_and_commit();
                self.apply_tool_event(ev);
                self.tool = Tool::Select;
                self.command_log.push(trimmed.to_string());
                return;
            }
        }

        // 2. Intercept Polygon side updates before center is placed
        if let Tool::Polygon { center: None, .. } = self.tool {
            if let Ok(n) = trimmed.parse::<usize>() {
                if n >= 3 {
                    self.tool = Tool::Polygon { center: None, sides: n };
                    self.command_log.push(trimmed.to_string());
                    return;
                }
            }
        }

        // 2b. A typed number updates the active modify tool's distance/radius.
        if let Ok(v) = trimmed.parse::<f64>() {
            if v > 0.0 {
                match &self.tool {
                    Tool::Offset { source, .. } => {
                        self.tool = Tool::Offset { dist: v, source: *source };
                        self.command_log.push(trimmed.to_string());
                        return;
                    }
                    Tool::Fillet { first, .. } => {
                        self.tool = Tool::Fillet { radius: v, first: *first };
                        self.command_log.push(trimmed.to_string());
                        return;
                    }
                    Tool::Chamfer { first, .. } => {
                        self.tool = Tool::Chamfer { dist: v, first: *first };
                        self.command_log.push(trimmed.to_string());
                        return;
                    }
                    _ => {}
                }
            }
        }

        if let Ok(dist) = trimmed.parse::<f64>() {
            if let Some(ref_pt) = self.tool.reference_point() {
                let (rx, ry) = ref_pt.to_f64();
                let (cx, cy) = self.cursor_world;
                let dx = cx - rx;
                let dy = cy - ry;
                let len = (dx * dx + dy * dy).sqrt();
                let (ux, uy) = if len > 1e-9 {
                    (dx / len, dy / len)
                } else if let Some((_, angle_rad)) = self.active_guide {
                    (angle_rad.cos(), angle_rad.sin())
                } else {
                    (1.0, 0.0)
                };
                let target_pt = Point2d::from_f64(rx + dist * ux, ry + dist * uy);
                let ev = self.tool.on_point(target_pt);
                self.apply_tool_event(ev);
                self.command_log.push(trimmed.to_string());
                return;
            }
        }

        // 3. Typed coordinate (x,y / @dx,dy / d<a / @d<a) feeds the active tool a
        //    point, AutoCAD-style. Relative/polar-relative forms offset the last
        //    point (the tool's reference, or the origin if none yet).
        if let Some(coord) = parse_coordinate(trimmed) {
            let (rx, ry) = self.tool.reference_point().map(|p| p.to_f64()).unwrap_or((0.0, 0.0));
            let (x, y) = match coord {
                CoordInput::Absolute(x, y) => (x, y),
                CoordInput::Relative(dx, dy) => (rx + dx, ry + dy),
                CoordInput::PolarAbsolute { dist, angle_deg } => {
                    let a = angle_deg.to_radians();
                    (dist * a.cos(), dist * a.sin())
                }
                CoordInput::PolarRelative { dist, angle_deg } => {
                    let a = angle_deg.to_radians();
                    (rx + dist * a.cos(), ry + dist * a.sin())
                }
            };
            let ev = self.tool.on_point(Point2d::from_f64(x, y));
            self.apply_tool_event(ev);
            self.command_log.push(trimmed.to_string());
            return;
        }

        let cmd = parse_command(text);
        self.command_log.push(text.trim().to_string());
        // Remember real commands (not Cancel/Unknown) so right-click can repeat them.
        if !matches!(cmd, Command::Cancel | Command::Unknown(_)) {
            self.last_command = Some(trimmed.to_string());
        }
        self.execute(cmd);
    }

    /// Re-run the last real command (AutoCAD right-click / Enter-at-empty-prompt).
    pub fn repeat_last_command(&mut self) {
        if let Some(cmd) = self.last_command.clone() {
            self.run_command(&cmd);
        }
    }

    pub fn execute(&mut self, cmd: Command) {
        match cmd {
            Command::Activate(mut tool) => {
                // Transform tools operate on the current selection.
                match &mut tool {
                    Tool::Move { ids, .. } | Tool::Copy { ids, .. }
                    | Tool::Rotate { ids, .. } | Tool::Scale { ids, .. }
                    | Tool::Mirror { ids, .. } | Tool::Stretch { ids, .. } =>
                        *ids = self.selection.clone(),
                    _ => {}
                }
                self.pending_snap_links.clear();
                self.tool = tool;
            }
            Command::Cancel => { self.pending_snap_links.clear(); self.tool.reset(); if matches!(self.tool, Tool::Select) { self.selection.clear(); } self.tool = Tool::Select; }
            Command::Undo => self.undo(),
            Command::Redo => self.redo(),
            Command::Erase => self.erase_selection(),
            Command::SelectAll => { self.selection = self.document.iter().map(|e| e.id).filter(|&id| id != self.origin_id).collect(); }
            Command::ZoomExtents => self.zoom_extents(),
            Command::ZoomScale(s) => { self.view.zoom = s.clamp(1e-9, 1e12); }
            Command::LayerSet(name) => { self.document.layers.set_current(&name); }
            Command::LayerNew(name) => { let idx = self.document.layers.add(Layer::new(name)); self.document.layers.current = idx; }
            Command::ToggleConstraints => {
                if self.constraints_enabled {
                    self.exit_parametric();
                } else {
                    self.enter_parametric();
                }
            }
            Command::AddConstraint(ctype) => self.add_constraint(ctype),
            Command::Unknown(_) => {}
        }
    }

    // ── Actions ───────────────────────────────────────────────────────────────

    pub fn undo(&mut self) {
        if let Some((prev_doc, prev_sketch, prev_map)) = self.history.undo(&self.document, &self.sketch, &self.entity_points) {
            self.document = prev_doc;
            self.sketch = prev_sketch;
            self.entity_points = prev_map;
            self.selection.clear();
        }
    }

    pub fn redo(&mut self) {
        if let Some((next_doc, next_sketch, next_map)) = self.history.redo(&self.document, &self.sketch, &self.entity_points) {
            self.document = next_doc;
            self.sketch = next_sketch;
            self.entity_points = next_map;
            self.selection.clear();
        }
    }

    pub fn erase_selection(&mut self) {
        if self.selection.is_empty() { return; }
        self.history.snapshot(&self.document, &self.sketch, &self.entity_points);
        for id in std::mem::take(&mut self.selection) {
            if id != self.origin_id {
                self.document.remove(id);
            }
        }
        if self.constraints_enabled {
            self.sync_sketch_from_document();
            self.solve_constraints();
        }
    }

    pub fn zoom_extents(&mut self) {
        if let Some(bb) = self.document.extents() {
            let (x0, y0) = bb.min.to_f64();
            let (x1, y1) = bb.max.to_f64();
            self.view.zoom_to_bounds(x0, y0, x1, y1);
        }
    }

    /// Directly add a finished entity (used by the egui view / tests).
    pub fn add_entity(&mut self, kind: EntityKind) -> EntityId {
        self.history.snapshot(&self.document, &self.sketch, &self.entity_points);
        let id = self.document.add(kind);
        if self.constraints_enabled {
            self.sync_sketch_from_document();
            self.solve_constraints();
        }
        id
    }

    // ── File operations ───────────────────────────────────────────────────────

    /// Reset to a blank document (File > New).
    pub fn new_document(&mut self) {
        self.document = Document::new();
        self.origin_id = self.document.add(EntityKind::Point(Point2d::from_i64(0, 0)));
        self.selection.clear();
        self.history = History::new();
        self.tool = Tool::Select;
        self.constraints_enabled = false;
        self.current_file_path = None;
        self.sketch = Sketch::new();
        self.entity_points = HashMap::new();
        self.sync_sketch_from_document();
    }

    /// Load a `.e2d` file into the document (File > Open). Logs errors to command_log.
    pub fn open_file(&mut self, path: std::path::PathBuf) {
        match exact2d_io::load_native(&path) {
            Ok(mut doc) => {
                let origin_id = doc.add(EntityKind::Point(Point2d::from_i64(0, 0)));
                self.document = doc;
                self.origin_id = origin_id;
                self.selection.clear();
                self.history = History::new();
                self.tool = Tool::Select;
                self.constraints_enabled = false;
                self.current_file_path = Some(path);
                self.sketch = Sketch::new();
                self.entity_points = HashMap::new();
                self.sync_sketch_from_document();
            }
            Err(e) => self.command_log.push(format!("Cannot open: {e}")),
        }
    }

    /// Save to the current path. Returns false (no error logged) when no path is set yet.
    pub fn save_file(&mut self) -> bool {
        if let Some(path) = self.current_file_path.clone() {
            self.save_file_to(path)
        } else {
            false
        }
    }

    /// Save to `path` and update `current_file_path`. Logs errors to command_log.
    pub fn save_file_to(&mut self, path: std::path::PathBuf) -> bool {
        // Save without the internal origin marker entity.
        let mut save_doc = self.document.clone();
        save_doc.remove(self.origin_id);
        match exact2d_io::save_native(&save_doc, &path) {
            Ok(()) => { self.current_file_path = Some(path); true }
            Err(e) => { self.command_log.push(format!("Save failed: {e}")); false }
        }
    }

    /// Window title: "filename — Exact2D CAD" or "Untitled — Exact2D CAD".
    pub fn window_title(&self) -> String {
        let name = self.current_file_path.as_ref()
            .and_then(|p| p.file_name())
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "Untitled".to_string());
        format!("{name} — Exact2D CAD")
    }

    // ── Status-bar info ───────────────────────────────────────────────────────

    /// Coordinate readout string for the status bar.
    pub fn coord_readout(&self) -> String {
        format!("{:.4}, {:.4}", self.cursor_world.0, self.cursor_world.1)
    }

    pub fn current_layer_name(&self) -> &str {
        &self.document.layers.current_layer().name
    }

    pub fn status_flags(&self) -> String {
        format!("SNAP:{} GRID:{} ORTHO:{}",
            on_off(self.snap_on), on_off(self.grid_on), on_off(self.ortho_on))
    }

    /// Human-readable name of the active drawing unit for the status bar.
    pub fn units_label(&self) -> &'static str {
        match self.document.settings.units.short_name() {
            "" => "none",
            s => s,
        }
    }

    /// Push the active unit's precision-safe zoom range into the view. Called each
    /// frame so changing the document unit instantly re-bounds the zoom.
    pub fn sync_zoom_limits(&mut self) {
        let (mn, mx) = self.document.settings.units.visible_range();
        self.view.set_visible_range(mn, mx);
    }

}

fn on_off(b: bool) -> &'static str { if b { "ON" } else { "off" } }

#[allow(dead_code)]
fn rat(x: f64) -> Rational { Rational::from_f64_approx(x) }

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_geometry::{Curve, LineSeg, CircularArc};

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    fn app() -> AppState { AppState::new(800.0, 600.0) }

    #[test]
    fn line_command_then_two_clicks_creates_segment() {
        let mut a = app();
        a.run_command("LINE");
        assert_eq!(a.tool.name(), "LINE");
        // Click two world points (convert via view: place near center).
        // Use direct world coords by setting cursor through screen mapping.
        let (s1x, s1y) = a.view.world_to_screen(0.0, 0.0);
        let (s2x, s2y) = a.view.world_to_screen(5.0, 0.0);
        a.snap_on = false; // avoid snapping in an empty doc anyway
        a.canvas_click(s1x, s1y);
        assert_eq!(a.document.len(), 1); // origin only
        a.canvas_click(s2x, s2y);
        assert_eq!(a.document.len(), 2); // segment created
    }

    #[test]
    fn undo_redo_through_state() {
        let mut a = app();
        a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(1,1)))));
        assert_eq!(a.document.len(), 2);
        a.undo();
        assert_eq!(a.document.len(), 1);
        a.redo();
        assert_eq!(a.document.len(), 2);
    }

    #[test]
    fn erase_removes_selection() {
        let mut a = app();
        let id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(2,2)))));
        a.selection = vec![id];
        a.run_command("ERASE");
        assert_eq!(a.document.len(), 1);
    }

    #[test]
    fn select_all_then_erase() {
        let mut a = app();
        a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(1,0)))));
        a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(0,1)))));
        a.run_command("ALL");
        assert_eq!(a.selection.len(), 2);
        a.run_command("ERASE");
        assert_eq!(a.document.len(), 1);
    }

    #[test]
    fn layer_commands() {
        let mut a = app();
        a.run_command("LAYER NEW walls");
        assert_eq!(a.current_layer_name(), "walls");
        a.run_command("LAYER SET 0");
        assert_eq!(a.current_layer_name(), "0");
    }

    #[test]
    fn move_command_uses_selection() {
        let mut a = app();
        let id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(2,0)))));
        a.selection = vec![id];
        a.run_command("MOVE");
        a.snap_on = false;
        // base point at world (0,0), destination at world (10,5)
        let (b1x, b1y) = a.view.world_to_screen(0.0, 0.0);
        let (b2x, b2y) = a.view.world_to_screen(10.0, 5.0);
        a.canvas_click(b1x, b1y);
        a.canvas_click(b2x, b2y);
        // Entity should be translated by (10,5)
        if let Some(Curve::Line(l)) = a.document.get(id).unwrap().as_curve() {
            assert!((l.p0.x.to_f64() - 10.0).abs() < 1e-4);
            assert!((l.p0.y.to_f64() - 5.0).abs() < 1e-4);
        } else { panic!() }
    }

    #[test]
    fn zoom_extents_frames_geometry() {
        let mut a = app();
        a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(100,80)))));
        a.run_command("ZOOM E");
        let (x0, y0, x1, y1) = a.view.visible_bounds();
        assert!(x0 <= 0.0 && x1 >= 100.0 && y0 <= 0.0 && y1 >= 80.0);
    }

    #[test]
    fn coord_readout_tracks_cursor() {
        let mut a = app();
        let (sx, sy) = a.view.world_to_screen(3.0, 7.0);
        a.pointer_moved(sx, sy);
        let r = a.coord_readout();
        assert!(r.starts_with("3.0000, 7.0000"));
    }

    #[test]
    fn perpendicular_snapping_uses_tool_reference_point() {
        let mut a = app();
        a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));
        a.snap.enabled = vec![exact2d_cad::SnapKind::Perpendicular];
        a.snap_on = true;

        a.run_command("LINE");

        let (s1x, s1y) = a.view.world_to_screen(3.0, 5.0);
        a.canvas_click(s1x, s1y);

        let (s2x, s2y) = a.view.world_to_screen(3.1, 0.1);
        a.pointer_moved(s2x, s2y);

        assert!(a.active_snap.is_some());
        let sp = a.active_snap.as_ref().unwrap();
        assert_eq!(sp.kind, exact2d_cad::SnapKind::Perpendicular);
        assert!((sp.pos.0 - 3.0).abs() < 1e-4);
        assert!(sp.pos.1.abs() < 1e-4);
    }

    #[test]
    fn ortho_mode_constrains_cursor_to_axis() {
        let mut a = app();
        a.snap_on = false;
        a.ortho_on = true;

        a.run_command("LINE");
        let (s1x, s1y) = a.view.world_to_screen(0.0, 0.0);
        a.canvas_click(s1x, s1y);

        let (s2x, s2y) = a.view.world_to_screen(8.0, 3.0);
        a.pointer_moved(s2x, s2y);
        assert!((a.cursor_world.0 - 8.0).abs() < 1e-4);
        assert!(a.cursor_world.1.abs() < 1e-4);

        let (s3x, s3y) = a.view.world_to_screen(2.0, 9.0);
        a.pointer_moved(s3x, s3y);
        assert!(a.cursor_world.0.abs() < 1e-4);
        assert!((a.cursor_world.1 - 9.0).abs() < 1e-4);
    }

    #[test]
    fn perpendicular_snapping_triggers_anywhere_near_line() {
        let mut a = app();
        a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));
        a.snap.enabled = vec![exact2d_cad::SnapKind::Perpendicular];
        a.snap_on = true;

        a.run_command("LINE");
        let (s1x, s1y) = a.view.world_to_screen(5.0, 5.0);
        a.canvas_click(s1x, s1y);

        // Hover near (5.3, 0.1) - slightly offset from (5.0, 0.0) but close to the line B (y = 0)
        let (s2x, s2y) = a.view.world_to_screen(5.3, 0.1);
        a.pointer_moved(s2x, s2y);

        assert!(a.active_snap.is_some());
        let sp = a.active_snap.as_ref().unwrap();
        assert_eq!(sp.kind, exact2d_cad::SnapKind::Perpendicular);
        assert!((sp.pos.0 - 5.0).abs() < 1e-4);
        assert!(sp.pos.1.abs() < 1e-4);
    }

    #[test]
    fn direct_distance_entry_projects_along_cursor() {
        let mut a = app();
        a.snap_on = false;
        a.run_command("LINE");

        // Click start point at (0.0, 0.0)
        let (s1x, s1y) = a.view.world_to_screen(0.0, 0.0);
        a.canvas_click(s1x, s1y);

        // Move cursor to indicate direction (3.0, 4.0) -> length 5.0, direction 3/5, 4/5
        let (s2x, s2y) = a.view.world_to_screen(3.0, 4.0);
        a.pointer_moved(s2x, s2y);

        // Enter a distance of 10.0
        a.run_command("10.0");

        // The document should now have a line from (0,0) to (6,8)
        assert_eq!(a.document.len(), 2);
        let first = a.document.iter().find(|e| e.id != a.origin_id).unwrap();
        if let EntityKind::Curve(Curve::Line(l)) = &first.kind {
            assert!((l.p0.x.to_f64() - 0.0).abs() < 1e-4);
            assert!((l.p0.y.to_f64() - 0.0).abs() < 1e-4);
            assert!((l.p1.x.to_f64() - 6.0).abs() < 1e-4);
            assert!((l.p1.y.to_f64() - 8.0).abs() < 1e-4);
        } else {
            panic!("expected line");
        }
    }

    #[test]
    fn typed_coordinates_build_a_line() {
        let mut a = app();
        a.snap_on = false;
        a.run_command("LINE");
        a.run_command("0,0");     // absolute start
        a.run_command("@10,0");   // relative → (10,0): commits the first segment

        assert_eq!(a.document.len(), 2); // origin + one line
        let line = a.document.iter().find(|e| e.id != a.origin_id).unwrap();
        if let EntityKind::Curve(Curve::Line(l)) = &line.kind {
            assert!((l.p0.x.to_f64()).abs() < 1e-9 && (l.p0.y.to_f64()).abs() < 1e-9);
            assert!((l.p1.x.to_f64() - 10.0).abs() < 1e-9 && (l.p1.y.to_f64()).abs() < 1e-9);
        } else {
            panic!("expected line");
        }
    }

    #[test]
    fn relative_polar_coordinate_places_point() {
        let mut a = app();
        a.snap_on = false;
        a.run_command("LINE");
        a.run_command("0,0");      // start at origin
        a.run_command("@5<90");    // 5 units at 90° → (0,5)

        let line = a.document.iter().find(|e| e.id != a.origin_id).unwrap();
        if let EntityKind::Curve(Curve::Line(l)) = &line.kind {
            assert!((l.p1.x.to_f64()).abs() < 1e-6, "x should be ~0, got {}", l.p1.x.to_f64());
            assert!((l.p1.y.to_f64() - 5.0).abs() < 1e-6, "y should be ~5, got {}", l.p1.y.to_f64());
        } else {
            panic!("expected line");
        }
    }

    #[test]
    fn right_click_repeat_reactivates_last_command() {
        let mut a = app();
        a.run_command("CIRCLE");
        assert!(matches!(a.tool, Tool::Circle { .. }));
        assert_eq!(a.last_command.as_deref(), Some("CIRCLE"));
        // Finish/cancel back to Select, then repeat.
        a.run_command(""); // Cancel → Select (does not overwrite last_command)
        assert!(matches!(a.tool, Tool::Select));
        a.repeat_last_command();
        assert!(matches!(a.tool, Tool::Circle { .. }));
    }

    #[test]
    fn polygon_command_allows_side_update() {
        let mut a = app();
        a.run_command("POLYGON");
        assert!(matches!(a.tool, Tool::Polygon { center: None, sides: 4 }));
        
        a.run_command("6");
        assert!(matches!(a.tool, Tool::Polygon { center: None, sides: 6 }));
        
        let (s1x, s1y) = a.view.world_to_screen(0.0, 0.0);
        a.canvas_click(s1x, s1y);
        
        let (s2x, s2y) = a.view.world_to_screen(10.0, 0.0);
        a.canvas_click(s2x, s2y);
        
        assert_eq!(a.document.len(), 7);
    }

    #[test]
    fn polyline_command_commits_on_empty_command() {
        let mut a = app();
        a.run_command("PL");
        assert!(matches!(a.tool, Tool::Polyline { .. }));
        
        let (s1x, s1y) = a.view.world_to_screen(0.0, 0.0);
        a.canvas_click(s1x, s1y);
        let (s2x, s2y) = a.view.world_to_screen(5.0, 5.0);
        a.canvas_click(s2x, s2y);
        let (s3x, s3y) = a.view.world_to_screen(10.0, 0.0);
        a.canvas_click(s3x, s3y);
        
        a.run_command("");
        assert!(matches!(a.tool, Tool::Select));
        assert_eq!(a.document.len(), 2); // 1 PolyCurve entity + 1 origin
    }

    #[test]
    fn polyline_command_closes_on_c_command() {
        let mut a = app();
        a.run_command("PL");
        
        let (s1x, s1y) = a.view.world_to_screen(0.0, 0.0);
        a.canvas_click(s1x, s1y);
        let (s2x, s2y) = a.view.world_to_screen(5.0, 5.0);
        a.canvas_click(s2x, s2y);
        let (s3x, s3y) = a.view.world_to_screen(10.0, 0.0);
        a.canvas_click(s3x, s3y);
        
        a.run_command("c");
        assert!(matches!(a.tool, Tool::Select));
        assert_eq!(a.document.len(), 2); // 1 PolyCurve entity + 1 origin
        
        let entity = a.document.iter().find(|e| e.id != a.origin_id).unwrap();
        if let EntityKind::Curve(Curve::Poly(poly)) = &entity.kind {
            assert_eq!(poly.segments.len(), 3); // closed!
        } else {
            panic!("expected PolyCurve");
        }
    }

    #[test]
    fn constraint_solving_and_toggling() {
        let mut a = app();
        // 1. Draw a line from (0,0) to (3,4).
        let id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0, 0), pt(3, 4)))));
        assert_eq!(a.document.len(), 2);

        // 2. Selection and command "CON H" to constrain it horizontally.
        a.selection = vec![id];
        a.run_command("CON H");
        assert!(a.constraints_enabled);
        assert_eq!(a.sketch.constraints().len(), 2); // origin Fix + horizontal

        // 3. Let's verify the solved coordinates.
        if let Some(EntityKind::Curve(Curve::Line(l))) = a.document.get(id).map(|e| &e.kind) {
            let y0 = l.p0.y.to_f64();
            let y1 = l.p1.y.to_f64();
            assert!((y0 - y1).abs() < 1e-5, "horizontal constraint should align y endpoints: y0={}, y1={}", y0, y1);
        } else {
            panic!("expected line");
        }

        // 4. Undo should restore constraints and coordinates.
        a.undo();
        assert_eq!(a.sketch.constraints().len(), 1); // origin Fix
        if let Some(EntityKind::Curve(Curve::Line(l))) = a.document.get(id).map(|e| &e.kind) {
            assert_eq!(l.p0, pt(0, 0));
            assert_eq!(l.p1, pt(3, 4));
        }

        // 5. Redo should re-apply horizontal constraint.
        a.redo();
        assert_eq!(a.sketch.constraints().len(), 2);
        if let Some(EntityKind::Curve(Curve::Line(l))) = a.document.get(id).map(|e| &e.kind) {
            let y0 = l.p0.y.to_f64();
            let y1 = l.p1.y.to_f64();
            assert!((y0 - y1).abs() < 1e-5);
        }
    }

    #[test]
    fn concentric_solving_test() {
        let mut a = app();
        let c1_id = a.add_entity(EntityKind::Curve(Curve::Arc(CircularArc::new(pt(0,0), Rational::from(2i64), 0.0, std::f64::consts::TAU))));
        let c2_id = a.add_entity(EntityKind::Curve(Curve::Arc(CircularArc::new(pt(5,5), Rational::from(3i64), 0.0, std::f64::consts::TAU))));
        
        a.selection = vec![c1_id, c2_id];
        a.run_command("CON CONCENTRIC");
        
        if let (Some(EntityKind::Curve(Curve::Arc(a1))), Some(EntityKind::Curve(Curve::Arc(a2)))) = 
            (a.document.get(c1_id).map(|e| &e.kind), a.document.get(c2_id).map(|e| &e.kind)) {
            let (c1x, c1y) = a1.center.to_f64();
            let (c2x, c2y) = a2.center.to_f64();
            assert!((c1x - c2x).abs() < 1e-5);
            assert!((c1y - c2y).abs() < 1e-5);
        } else {
            panic!("expected two arcs");
        }
    }

    #[test]
    fn tangent_solving_test() {
        let mut a = app();
        let c_id = a.add_entity(EntityKind::Curve(Curve::Arc(CircularArc::new(pt(0,0), Rational::from(2i64), 0.0, std::f64::consts::TAU))));
        let l_id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,3), pt(4,3)))));
        
        // Fix the circle center and start point first
        a.selection = vec![c_id];
        a.run_command("CON FIX");
        
        a.selection = vec![c_id, l_id];
        a.run_command("CON TANGENT");
        
        if let (Some(EntityKind::Curve(Curve::Arc(arc))), Some(EntityKind::Curve(Curve::Line(line)))) = 
            (a.document.get(c_id).map(|e| &e.kind), a.document.get(l_id).map(|e| &e.kind)) {
            // Distance from center to line should equal radius (which is 2)
            let (cx, cy) = arc.center.to_f64();
            let (ax, ay) = line.p0.to_f64();
            let (bx, by) = line.p1.to_f64();
            let ux = bx - ax;
            let uy = by - ay;
            let len = (ux * ux + uy * uy).sqrt();
            let dist = (ux * (ay - cy) - uy * (ax - cx)).abs() / len;
            assert!((dist - 2.0).abs() < 1e-4);
        } else {
            panic!("expected arc and line");
        }
    }

    #[test]
    fn midpoint_solving_test() {
        let mut a = app();
        let pt_id = a.add_entity(EntityKind::Point(pt(1,1)));
        let l_id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));
        
        // Fix the line endpoints first
        a.selection = vec![l_id];
        a.run_command("CON FIX");
        
        a.selection = vec![pt_id, l_id];
        a.run_command("CON MIDPOINT");
        
        if let Some(EntityKind::Point(p)) = a.document.get(pt_id).map(|e| &e.kind) {
            let (px, py) = p.to_f64();
            assert!((px - 5.0).abs() < 1e-5);
            assert!(py.abs() < 1e-5);
        } else {
            panic!("expected point");
        }
    }

    #[test]
    fn equal_solving_test() {
        let mut a = app();
        let l1_id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(5,0)))));
        let l2_id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(0,10)))));
        
        a.selection = vec![l1_id, l2_id];
        a.run_command("CON EQUAL");
        
        if let (Some(EntityKind::Curve(Curve::Line(line1))), Some(EntityKind::Curve(Curve::Line(line2)))) = 
            (a.document.get(l1_id).map(|e| &e.kind), a.document.get(l2_id).map(|e| &e.kind)) {
            let len1 = line1.p0.dist_f64(&line1.p1);
            let len2 = line2.p0.dist_f64(&line2.p1);
            assert!((len1 - len2).abs() < 1e-5);
        } else {
            panic!("expected two lines");
        }
    }

    #[test]
    fn auto_dimension_solving_test() {
        let mut a = app();
        let l_id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(3,4)))));
        
        a.selection = vec![l_id];
        a.run_command("CON DISTANCE");
        
        assert_eq!(a.sketch.constraints().len(), 2); // origin Fix + distance
        if let exact2d_constraint::Constraint::Distance(_, _, d) = a.sketch.constraints()[1] {
            assert!((d - 5.0).abs() < 1e-5);
        } else {
            panic!("expected distance constraint");
        }
    }

    #[test]
    fn grip_editing_integration_test() {
        let mut a = app();
        let l_id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));
        a.sync_sketch_from_document();
        
        // Retrieve point IDs from entity_points
        let pts = a.entity_points.get(&l_id).cloned().expect("should have points registered");
        assert_eq!(pts.len(), 2);
        
        // Move the second endpoint
        a.sketch.set_point(pts[1], 10.0, 10.0);
        a.solve_constraints();
        a.sync_document_from_sketch();
        
        // Check document coordinates updated
        if let Some(EntityKind::Curve(Curve::Line(line))) = a.document.get(l_id).map(|e| &e.kind) {
            assert_eq!(line.p0.to_f64(), (0.0, 0.0));
            assert_eq!(line.p1.to_f64(), (10.0, 10.0));
        } else {
            panic!("expected line entity");
        }
    }

    #[test]
    fn fixed_origin_test() {
        let mut a = app();
        // Check origin exists and is at 0,0
        if let Some(EntityKind::Point(p)) = a.document.get(a.origin_id).map(|e| &e.kind) {
            assert_eq!(p.to_f64(), (0.0, 0.0));
        } else {
            panic!("expected origin point");
        }

        // Try to toggle selection on origin point (should return early without doing anything)
        a.toggle_selection(a.origin_id);
        assert!(!a.selection.contains(&a.origin_id));

        // Try to erase selection on origin point (even if somehow forced in selection)
        a.selection = vec![a.origin_id];
        a.erase_selection();
        assert!(a.document.get(a.origin_id).is_some());

        // Try to move origin point
        let t = exact2d_geometry::Transform2d::translation(rat(10.0), rat(10.0));
        let ev = ToolEvent::Transform { ids: vec![a.origin_id], t };
        a.apply_tool_event(ev);
        if let Some(EntityKind::Point(p)) = a.document.get(a.origin_id).map(|e| &e.kind) {
            assert_eq!(p.to_f64(), (0.0, 0.0));
        } else {
            panic!("expected origin point");
        }
    }

    #[test]
    fn smart_dimension_tool_test() {
        let mut a = app();
        let _l_id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));
        
        // Activate smart dimension tool
        a.tool = Tool::Dimension { stage: 0, p1: None, p2: None };
        
        // Stage 0: click near the line center to select the line directly
        let (s1x, s1y) = a.view.world_to_screen(5.0, 0.1);
        a.pointer_moved(s1x, s1y);
        a.canvas_click(s1x, s1y);
        
        // Should transition directly to Stage 2 with endpoints set as p1 and p2
        match a.tool {
            Tool::Dimension { stage: 2, p1: Some(_), p2: Some(_) } => {},
            ref t => panic!("expected stage 2 with p1 and p2 set, got {:?}", t),
        }
        
        // Stage 2: place the dimension. Move cursor straight up to select DistanceX (horizontal distance)
        let (s2x, s2y) = a.view.world_to_screen(5.0, 5.0);
        a.pointer_moved(s2x, s2y);
        a.canvas_click(s2x, s2y);
        
        // Tool should return to Select and have added a DistanceX constraint (or DistanceX under solver constraints)
        assert!(matches!(a.tool, Tool::Select));
        assert_eq!(a.sketch.constraints().len(), 2); // 1 Fix for origin + 1 DistanceX
        
        // Find DistanceX constraint
        let found_dist = a.sketch.constraints().iter().any(|c| matches!(c, Constraint::DistanceX(..)));
        assert!(found_dist, "expected a DistanceX constraint added");
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Stage 0 — constraint refactor safety net (see docs/constraint-refactor-plan.md).
    // These lock in the OBSERVABLE behavior the refactor must preserve. They assert
    // geometry/document outcomes (not internal sketch structure, which will change),
    // so they stay green across the session-scoped rewrite.
    // ──────────────────────────────────────────────────────────────────────────

    /// Horizontal aligns a line's endpoint Y; geometry survives toggling parametric off.
    #[test]
    fn stage0_horizontal_solves_and_survives_toggle_off() {
        let mut a = app();
        let id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(3,4)))));
        a.selection = vec![id];
        a.run_command("CON H");
        assert!(a.constraints_enabled);

        let aligned = |a: &AppState| -> (f64, f64) {
            if let Some(EntityKind::Curve(Curve::Line(l))) = a.document.get(id).map(|e| &e.kind) {
                (l.p0.y.to_f64(), l.p1.y.to_f64())
            } else { panic!("expected line"); }
        };
        let (y0, y1) = aligned(&a);
        assert!((y0 - y1).abs() < 1e-5, "horizontal: y0={y0} y1={y1}");

        // Toggle parametric OFF — geometry must persist unchanged (single source of truth).
        a.run_command("CONSTRAINTS");
        assert!(!a.constraints_enabled);
        let (y0b, y1b) = aligned(&a);
        assert!((y0 - y0b).abs() < 1e-9 && (y1 - y1b).abs() < 1e-9,
            "geometry changed when parametric toggled off");
    }

    /// Vertical aligns a line's endpoint X.
    #[test]
    fn stage0_vertical_aligns_x() {
        let mut a = app();
        let id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(4,3)))));
        a.selection = vec![id];
        a.run_command("CON V");
        if let Some(EntityKind::Curve(Curve::Line(l))) = a.document.get(id).map(|e| &e.kind) {
            assert!((l.p0.x.to_f64() - l.p1.x.to_f64()).abs() < 1e-5);
        } else { panic!("expected line"); }
    }

    /// A typed distance value sets the segment length.
    #[test]
    fn stage0_distance_value_sets_length() {
        let mut a = app();
        let id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(3,4)))));
        a.selection = vec![id];
        a.run_command("CON D 7"); // constrain length to 7
        if let Some(EntityKind::Curve(Curve::Line(l))) = a.document.get(id).map(|e| &e.kind) {
            let len = l.p0.dist_f64(&l.p1);
            assert!((len - 7.0).abs() < 1e-4, "length should be 7, got {len}");
        } else { panic!("expected line"); }
    }

    /// Perpendicular: two lines sharing a corner become perpendicular (dot ≈ 0).
    #[test]
    fn stage0_perpendicular_solves() {
        let mut a = app();
        let l1 = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(5,0)))));
        let l2 = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(3,1)))));
        a.selection = vec![l1, l2];
        a.run_command("CON PERP");
        if let (Some(EntityKind::Curve(Curve::Line(a1))), Some(EntityKind::Curve(Curve::Line(b1)))) =
            (a.document.get(l1).map(|e| &e.kind), a.document.get(l2).map(|e| &e.kind)) {
            let (ux, uy) = (a1.p1.x.to_f64() - a1.p0.x.to_f64(), a1.p1.y.to_f64() - a1.p0.y.to_f64());
            let (vx, vy) = (b1.p1.x.to_f64() - b1.p0.x.to_f64(), b1.p1.y.to_f64() - b1.p0.y.to_f64());
            let dot = ux * vx + uy * vy;
            assert!(dot.abs() < 1e-4, "lines should be perpendicular, dot={dot}");
        } else { panic!("expected two lines"); }
    }

    /// A polyline's shared interior vertex resolves to a single point (intended sharing).
    /// The refactor must preserve this connectivity (Stage 3 makes it by construction).
    #[test]
    fn stage0_polyline_shared_vertex_is_single_point() {
        let mut a = app();
        a.run_command("PL");
        for (wx, wy) in [(0.0, 0.0), (5.0, 5.0), (10.0, 0.0)] {
            let (sx, sy) = a.view.world_to_screen(wx, wy);
            a.canvas_click(sx, sy);
        }
        a.run_command(""); // commit polyline
        let poly_id = a.document.iter().find(|e| e.id != a.origin_id).map(|e| e.id).unwrap();

        a.run_command("CONSTRAINTS"); // enter parametric → build the session
        let pts = a.entity_points.get(&poly_id).cloned().expect("polyline points");
        assert_eq!(pts.len(), 4, "two segments register four endpoints");
        assert_eq!(pts[1], pts[2], "the shared interior vertex must be one point");
    }

    /// Auto radius-equality keeps an arc circular after a point is dragged off-radius.
    #[test]
    fn stage0_arc_stays_circular_after_solve() {
        let mut a = app();
        let arc_id = a.add_entity(EntityKind::Curve(Curve::Arc(
            CircularArc::new(pt(0,0), Rational::from(5i64), 0.0, std::f64::consts::PI))));
        a.run_command("CONSTRAINTS"); // build session; auto EqualLength(center-start, center-end)
        let pts = a.entity_points.get(&arc_id).cloned().expect("arc points"); // [center, start, end]
        assert_eq!(pts.len(), 3);

        // Drag the end point to a different radius, then re-solve.
        a.sketch.set_point(pts[2], 10.0, 0.0);
        a.solve_constraints();

        let (cx, cy) = a.sketch.point(pts[0]);
        let (sx, sy) = a.sketch.point(pts[1]);
        let (ex, ey) = a.sketch.point(pts[2]);
        let r_start = ((sx - cx).powi(2) + (sy - cy).powi(2)).sqrt();
        let r_end = ((ex - cx).powi(2) + (ey - cy).powi(2)).sqrt();
        assert!((r_start - r_end).abs() < 1e-4,
            "arc should stay circular: r_start={r_start} r_end={r_end}");
    }

    /// Two unrelated lines that merely share a coordinate must NOT be welded into
    /// the same point. Fixed in Stage 2 (per-entity `push_point` replaced the global
    /// proximity dedup), so this now passes — independent geometry stays distinct.
    #[test]
    fn stage0_independent_coincident_located_endpoints_stay_independent() {
        let mut a = app();
        let l1 = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));
        let l2 = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(0,10)))));
        a.run_command("CONSTRAINTS"); // build session
        let p1 = a.entity_points.get(&l1).cloned().unwrap();
        let p2 = a.entity_points.get(&l2).cloned().unwrap();
        assert_ne!(p1[0], p2[0],
            "endpoints of unrelated lines at the same location must not be welded");
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Stage 1 — ParametricSession lifecycle (see docs/constraint-refactor-plan.md).
    // ──────────────────────────────────────────────────────────────────────────

    /// Parametric mode is OFF by default: no sketch overlay exists at startup.
    #[test]
    fn stage1_no_sketch_overlay_at_startup() {
        let a = app();
        assert!(!a.constraints_enabled);
        assert_eq!(a.sketch.num_points(), 0, "no sketch points before entering parametric");
        assert!(a.entity_points.is_empty(), "no overlay map before entering parametric");
        assert_eq!(a.sketch.constraints().len(), 0);
    }

    /// Exiting parametric discards the overlay (Option A) but keeps the geometry,
    /// and re-entering rebuilds a fresh overlay from the current document.
    #[test]
    fn stage1_exit_discards_overlay_keeps_geometry_reenter_rebuilds() {
        let mut a = app();
        let id = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(3,4)))));
        a.selection = vec![id];
        a.run_command("CON H"); // enter parametric + horizontal
        assert!(a.constraints_enabled);
        assert!(!a.entity_points.is_empty());

        let geom = |a: &AppState| -> ((f64,f64),(f64,f64)) {
            if let Some(EntityKind::Curve(Curve::Line(l))) = a.document.get(id).map(|e| &e.kind) {
                (l.p0.to_f64(), l.p1.to_f64())
            } else { panic!("expected line"); }
        };
        let before = geom(&a);

        // Exit: overlay dropped, geometry untouched.
        a.run_command("CONSTRAINTS");
        assert!(!a.constraints_enabled);
        assert_eq!(a.sketch.num_points(), 0, "sketch dropped on exit");
        assert_eq!(a.sketch.constraints().len(), 0, "constraints discarded on exit");
        assert!(a.entity_points.is_empty(), "overlay map dropped on exit");
        assert_eq!(geom(&a), before, "geometry must survive exit unchanged");

        // Re-enter: a fresh overlay is built from current geometry.
        a.run_command("CONSTRAINTS");
        assert!(a.constraints_enabled);
        assert!(!a.entity_points.is_empty(), "re-entering rebuilds the overlay");
        assert!(a.entity_points.contains_key(&id), "the line is registered again");
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Stage 3 — coincidence by intent: snap-to-endpoint while drawing links the new
    // geometry to what it snapped onto (see docs/constraint-refactor-plan.md).
    // ──────────────────────────────────────────────────────────────────────────

    /// Snapping a new line's start onto an existing endpoint adds a Coincident
    /// constraint linking them.
    #[test]
    fn stage3_snap_to_endpoint_while_drawing_adds_coincidence() {
        let mut a = app();
        a.run_command("CONSTRAINTS"); // enter parametric
        let la = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));

        a.tool = Tool::Line { last: None };
        a.snap.enabled = vec![SnapKind::Endpoint];
        a.snap_on = true;
        let (s1x, s1y) = a.view.world_to_screen(10.0, 0.0); // onto A's endpoint
        a.canvas_click(s1x, s1y);
        a.snap_on = false; // second point free
        let (s2x, s2y) = a.view.world_to_screen(10.0, 10.0);
        a.canvas_click(s2x, s2y);

        let lb = a.document.iter().find(|e| e.id != a.origin_id && e.id != la).map(|e| e.id)
            .expect("line B created");
        let a_end = a.entity_points.get(&la).unwrap()[1];  // A's (10,0) endpoint
        let b_start = a.entity_points.get(&lb).unwrap()[0]; // B's start
        let linked = a.sketch.constraints().iter().any(|c| matches!(c,
            Constraint::Coincident(x, y)
                if (*x == a_end && *y == b_start) || (*x == b_start && *y == a_end)));
        assert!(linked, "snapping B's start onto A's endpoint should add Coincident(A_end, B_start)");
    }

    /// Without an active snap, drawing a point at the same location as an existing
    /// endpoint does NOT auto-link — coincidence is by intent only (not proximity).
    #[test]
    fn stage3_no_coincidence_without_snap() {
        let mut a = app();
        a.run_command("CONSTRAINTS");
        let _la = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));

        a.tool = Tool::Line { last: None };
        a.snap_on = false; // no snapping
        let (s1x, s1y) = a.view.world_to_screen(10.0, 0.0); // same spot as A's end, but no snap
        a.canvas_click(s1x, s1y);
        let (s2x, s2y) = a.view.world_to_screen(10.0, 10.0);
        a.canvas_click(s2x, s2y);

        let has_coincident = a.sketch.constraints().iter().any(|c| matches!(c, Constraint::Coincident(..)));
        assert!(!has_coincident, "no snap → no auto-coincidence, even at the same location");
    }

    /// The dimension tool enters parametric mode on first use (dimensions are
    /// dimensional constraints in this app).
    #[test]
    fn stage1_dimension_tool_enters_parametric() {
        let mut a = app();
        let _l = a.add_entity(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));
        assert!(!a.constraints_enabled);
        a.tool = Tool::Dimension { stage: 0, p1: None, p2: None };
        let (sx, sy) = a.view.world_to_screen(5.0, 0.1);
        a.pointer_moved(sx, sy);
        a.canvas_click(sx, sy); // stage 0 pick → enters parametric
        assert!(a.constraints_enabled, "using the dimension tool enters parametric mode");
    }
}
