//! The headless application state (spec §6.1) — the testable "brain" the egui view
//! drives. Owns the document, view, active tool, selection, snap settings, status
//! toggles, command log, and undo/redo history. No egui or GPU dependencies.

use exact2d_geometry::{Point2d, Curve};
use exact2d_document::{Document, EntityKind, EntityId, Layer};
use exact2d_cad::{SnapSettings, SnapPoint, best_snap, pick_at};

use crate::view_transform::ViewTransform;
use crate::tools::{Tool, ToolEvent};
use crate::command::{Command, parse_command, parse_coordinate, CoordInput};
use crate::history::History;

/// Interactive modify-tool click handling (TRIM/EXTEND/OFFSET/FILLET/CHAMFER/STRETCH).
mod modify;
/// Contextual, selection-tethered direct-manipulation actions (corner fillet/chamfer).
mod contextual;

pub use contextual::{CornerGeom, CornerAction, CornerKind, fillet_arc};

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
    /// A fixed origin marker point at (0,0); excluded from selection/erase.
    pub origin_id: EntityId,
    /// An in-progress contextual corner action (fillet/chamfer being sized visually).
    pub corner_action: Option<CornerAction>,

    /// Path of the currently open file, if any.
    pub current_file_path: Option<std::path::PathBuf>,
}

impl AppState {
    pub fn new(canvas_w: f64, canvas_h: f64) -> Self {
        let mut document = Document::new();
        let origin_id = document.add(EntityKind::Point(Point2d::from_i64(0, 0)));

        AppState {
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
            origin_id,
            corner_action: None,

            current_file_path: None,
        }
    }

    // ── Pointer input ─────────────────────────────────────────────────────────

    /// Update the cursor position (screen pixels) and recompute the active snap.
    pub fn pointer_moved(&mut self, sx: f64, sy: f64) {
        let (wx, wy) = self.view.screen_to_world(sx, sy);
        
        self.active_snap = if self.snap_on && self.tool.wants_point_snap() {
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

        // TEXT tool: the first click sets the anchor; the content is then typed at
        // the command line (handled in run_command).
        if let Tool::Text { anchor, height } = &self.tool {
            let height = *height;
            let need_anchor = anchor.is_none();
            if need_anchor {
                self.tool = Tool::Text { anchor: Some(p), height };
            }
            return;
        }

        // SELECT tool: pick the entity under the cursor and toggle it.
        if matches!(self.tool, Tool::Select) {
            if let Some(id) = pick_at(&self.document, p.x, p.y,
                                      self.view.pixel_world_size() * 6.0) {
                self.toggle_selection(id);
            } else {
                self.selection.clear();
            }
            return;
        }

        // Drawing/edit tool: feed the point and apply the resulting event.
        let ev = self.tool.on_point(p);
        self.apply_tool_event(ev);
    }

    fn apply_tool_event(&mut self, ev: ToolEvent) {
        match ev {
            ToolEvent::Pending => {}
            ToolEvent::Create(kinds) => {
                self.history.snapshot(&self.document);
                for k in kinds { self.document.add(k); }
            }
            ToolEvent::Transform { ids, t } => {
                self.history.snapshot(&self.document);
                for id in ids {
                    if id != self.origin_id {
                        if let Some(e) = self.document.get_mut(id) { e.transform(&t); }
                    }
                }
                self.tool = Tool::Select;
            }
            ToolEvent::CopyOf { ids, t } => {
                self.history.snapshot(&self.document);
                for id in ids {
                    if id != self.origin_id {
                        if let Some(e) = self.document.get(id) {
                            let copy = e.transformed(&t);
                            self.document.add_entity(copy);
                        }
                    }
                }
                self.tool = Tool::Select;
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

        // 0. TEXT tool: once the anchor is placed, the next command-line entry is
        // the text content (taken literally; `\n` becomes a line break, so one
        // unified tool handles single- and multi-line text). Empty cancels.
        if let Tool::Text { anchor: Some(p), height } = self.tool.clone() {
            if !trimmed.is_empty() {
                self.history.snapshot(&self.document);
                self.document.add(EntityKind::Text {
                    anchor: p,
                    content: trimmed.replace("\\n", "\n"),
                    height,
                    rotation: 0.0,
                });
            }
            self.tool = Tool::Select;
            self.command_log.push(trimmed.to_string());
            return;
        }

        // 1. Intercept Polyline / CV-spline commit / close commands
        if matches!(self.tool, Tool::Polyline { .. } | Tool::Spline { .. }) {
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
                self.tool = tool;
            }
            Command::Cancel => { self.tool.reset(); if matches!(self.tool, Tool::Select) { self.selection.clear(); } self.tool = Tool::Select; }
            Command::Undo => self.undo(),
            Command::Redo => self.redo(),
            Command::Erase => self.erase_selection(),
            Command::SelectAll => { self.selection = self.document.iter().map(|e| e.id).filter(|&id| id != self.origin_id).collect(); }
            Command::ZoomExtents => self.zoom_extents(),
            Command::ZoomScale(s) => { self.view.zoom = s.clamp(1e-9, 1e12); }
            Command::LayerSet(name) => { self.document.layers.set_current(&name); }
            Command::LayerNew(name) => { let idx = self.document.layers.add(Layer::new(name)); self.document.layers.current = idx; }
            Command::Unknown(_) => {}
        }
    }

    // ── Actions ───────────────────────────────────────────────────────────────

    pub fn undo(&mut self) {
        if let Some(prev) = self.history.undo(&self.document) {
            self.document = prev;
            self.selection.clear();
        }
    }

    pub fn redo(&mut self) {
        if let Some(next) = self.history.redo(&self.document) {
            self.document = next;
            self.selection.clear();
        }
    }

    pub fn erase_selection(&mut self) {
        if self.selection.is_empty() { return; }
        self.history.snapshot(&self.document);
        for id in std::mem::take(&mut self.selection) {
            if id != self.origin_id {
                self.document.remove(id);
            }
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
        self.history.snapshot(&self.document);
        self.document.add(kind)
    }

    // ── NURBS spline grip editing (spec §6 direct manipulation) ─────────────────

    /// If exactly one selected entity is a NURBS spline, return its id, control
    /// vertices, and weights (for the canvas to draw grips and hit-test).
    pub fn selected_nurbs(&self) -> Option<(EntityId, Vec<Point2d>, Vec<f64>)> {
        if self.selection.len() != 1 { return None; }
        let id = self.selection[0];
        if let EntityKind::Curve(Curve::Nurbs(nc)) = &self.document.get(id)?.kind {
            Some((id, nc.control.clone(), nc.weights.clone()))
        } else {
            None
        }
    }

    /// Snapshot history before an interactive edit (call once at a grip-drag start).
    pub fn begin_edit(&mut self) {
        self.history.snapshot(&self.document);
    }

    /// Move a NURBS control vertex to `p`. No history snapshot — the caller takes one
    /// at drag start via [`begin_edit`], so a whole drag is a single undo step.
    pub fn set_nurbs_control(&mut self, id: EntityId, index: usize, p: Point2d) {
        if let Some(e) = self.document.get_mut(id) {
            if let EntityKind::Curve(Curve::Nurbs(nc)) = &mut e.kind {
                if index < nc.control.len() { nc.control[index] = p; }
            }
        }
    }

    /// Multiply a NURBS control vertex's weight by `factor` (clamped to a sane
    /// positive range), snapshotting history. Returns true if it was applied.
    pub fn adjust_nurbs_weight(&mut self, id: EntityId, index: usize, factor: f64) -> bool {
        let ok = matches!(self.document.get(id).map(|e| &e.kind),
            Some(EntityKind::Curve(Curve::Nurbs(nc))) if index < nc.weights.len());
        if !ok { return false; }
        self.history.snapshot(&self.document);
        if let Some(EntityKind::Curve(Curve::Nurbs(nc))) = self.document.get_mut(id).map(|e| &mut e.kind) {
            nc.weights[index] = (nc.weights[index] * factor).clamp(0.05, 20.0);
        }
        true
    }

    // ── File operations ───────────────────────────────────────────────────────

    /// Reset to a blank document (File > New).
    pub fn new_document(&mut self) {
        self.document = Document::new();
        self.origin_id = self.document.add(EntityKind::Point(Point2d::from_i64(0, 0)));
        self.selection.clear();
        self.history = History::new();
        self.tool = Tool::Select;
        self.current_file_path = None;
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
                self.current_file_path = Some(path);
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

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_geometry::{Curve, LineSeg};

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
            assert!((l.p0.x - 10.0).abs() < 1e-4);
            assert!((l.p0.y - 5.0).abs() < 1e-4);
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
            assert!((l.p0.x - 0.0).abs() < 1e-4);
            assert!((l.p0.y - 0.0).abs() < 1e-4);
            assert!((l.p1.x - 6.0).abs() < 1e-4);
            assert!((l.p1.y - 8.0).abs() < 1e-4);
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
            assert!((l.p0.x).abs() < 1e-9 && (l.p0.y).abs() < 1e-9);
            assert!((l.p1.x - 10.0).abs() < 1e-9 && (l.p1.y).abs() < 1e-9);
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
            assert!((l.p1.x).abs() < 1e-6, "x should be ~0, got {}", l.p1.x);
            assert!((l.p1.y - 5.0).abs() < 1e-6, "y should be ~5, got {}", l.p1.y);
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
    fn cv_spline_command_commits_to_nurbs() {
        let mut a = app();
        a.run_command("SPLINE");
        assert!(matches!(a.tool, Tool::Spline { .. }));

        for (wx, wy) in [(0.0, 0.0), (5.0, 8.0), (10.0, -4.0), (15.0, 0.0)] {
            let (sx, sy) = a.view.world_to_screen(wx, wy);
            a.canvas_click(sx, sy);
        }
        a.run_command(""); // Enter finishes the CV spline
        assert!(matches!(a.tool, Tool::Select));
        assert_eq!(a.document.len(), 2); // 1 NURBS curve + origin

        let entity = a.document.iter().find(|e| e.id != a.origin_id).unwrap();
        match &entity.kind {
            EntityKind::Curve(Curve::Nurbs(nc)) => assert_eq!(nc.control.len(), 4),
            other => panic!("expected a NURBS curve, got {:?}", other),
        }
    }

    #[test]
    fn nurbs_grip_edit_moves_control_and_weight() {
        let mut a = app();
        let nc = exact2d_geometry::NurbsCurve::uniform(vec![
            Point2d::from_i64(0, 0), Point2d::from_i64(2, 4), Point2d::from_i64(6, 4),
            Point2d::from_i64(8, 0), Point2d::from_i64(10, 4)]);
        let id = a.add_entity(EntityKind::Curve(Curve::Nurbs(nc)));
        a.selection = vec![id];

        // selected_nurbs surfaces the control vertices for the grip UI.
        let (sid, control, weights) = a.selected_nurbs().expect("a NURBS is selected");
        assert_eq!(sid, id);
        assert_eq!(control.len(), 5);
        assert!(weights.iter().all(|&w| w == 1.0));

        // Drag-move control vertex 2 (one undo step via begin_edit).
        a.begin_edit();
        a.set_nurbs_control(id, 2, Point2d::from_f64(6.0, 9.0));
        let weight_at = |a: &AppState, i: usize| {
            if let EntityKind::Curve(Curve::Nurbs(nc)) = &a.document.get(id).unwrap().kind {
                (nc.control[i], nc.weights[i])
            } else { panic!("expected NURBS") }
        };
        assert_eq!(weight_at(&a, 2).0, Point2d::from_f64(6.0, 9.0));

        // Weight edits (clamped); undo restores the prior weight.
        assert!(a.adjust_nurbs_weight(id, 2, 5.0));
        assert!((weight_at(&a, 2).1 - 5.0).abs() < 1e-9);
        a.adjust_nurbs_weight(id, 2, 100.0);            // would be 500 → clamped
        assert!(weight_at(&a, 2).1 <= 20.0 + 1e-9);
        a.undo();
        assert!((weight_at(&a, 2).1 - 5.0).abs() < 1e-9, "undo restores the prior weight");
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
        let t = exact2d_geometry::Transform2d::translation(10.0, 10.0);
        let ev = ToolEvent::Transform { ids: vec![a.origin_id], t };
        a.apply_tool_event(ev);
        if let Some(EntityKind::Point(p)) = a.document.get(a.origin_id).map(|e| &e.kind) {
            assert_eq!(p.to_f64(), (0.0, 0.0));
        } else {
            panic!("expected origin point");
        }
    }

    #[test]
    fn text_tool_places_text_entity() {
        let mut a = app();
        a.run_command("TEXT");
        assert!(matches!(a.tool, Tool::Text { anchor: None, .. }));
        let (sx, sy) = a.view.world_to_screen(2.0, 3.0);
        a.canvas_click(sx, sy); // set anchor
        assert!(matches!(a.tool, Tool::Text { anchor: Some(_), .. }));
        a.run_command("Hello\\nWorld"); // typed content; \n → line break
        assert!(matches!(a.tool, Tool::Select));
        let content = a.document.iter().find_map(|e| match &e.kind {
            EntityKind::Text { content, .. } => Some(content.clone()),
            _ => None,
        }).expect("a Text entity should be created");
        assert_eq!(content, "Hello\nWorld", "single unified tool handles multi-line via \\n");
    }
}
