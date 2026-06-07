//! egui view layer (spec §6.1 window layout, §6.2 egui).
//!
//! Renders the full application chrome — menu bar, ribbon, layer/properties panels,
//! drawing canvas, command line, and status bar — by reading and driving `AppState`.
//! This compiles against `egui` without a windowing backend; a host (`exact2d_app`,
//! eframe) supplies the `egui::Context` each frame.

use egui::{Context, SidePanel, CentralPanel, Sense, Stroke, Color32, pos2, vec2};
use exact2d_geometry::{Curve, Point2d};
use exact2d_document::{Color, EntityKind};

use crate::state::AppState;
use crate::tools::Tool;
use crate::command::Command;

mod chrome;
mod tessellate;
use chrome::{menu_bar, ribbon, status_and_command, layer_panel};
use tessellate::draw_curve;

/// Per-frame UI state that the host owns across frames.
#[derive(Default)]
pub struct UiState {
    /// Current text in the command-line input box.
    pub command_input: String,
}

/// Build the entire UI for one frame.
pub fn draw_ui(ctx: &Context, app: &mut AppState, ui_state: &mut UiState) {
    menu_bar(ctx, app);
    ribbon(ctx, app);
    status_and_command(ctx, app, ui_state);
    layer_panel(ctx, app);
    constraints_panel(ctx, app);
    canvas(ctx, app, ui_state);

    // Floating window to edit selected constraint parameters on double-click
    if app.constraints_enabled {
        let edit_idx: Option<usize> = ctx.data(|d| d.get_temp(egui::Id::new("edit_constraint_idx")));
        if let Some(idx) = edit_idx {
            if let Some(c) = app.sketch.constraints().get(idx).cloned() {
                let mut new_val_str: String = ctx.data(|d| d.get_temp(egui::Id::new("edit_constraint_str")).unwrap_or_default());
                let is_initialized: bool = ctx.data(|d| d.get_temp(egui::Id::new("edit_constraint_init")).unwrap_or(false));
                
                let current_val = match c {
                    exact2d_constraint::Constraint::Distance(_, _, d) |
                    exact2d_constraint::Constraint::DistanceX(_, _, d) |
                    exact2d_constraint::Constraint::DistanceY(_, _, d) => d,
                    exact2d_constraint::Constraint::Angle(_, _, theta) => theta.to_degrees(),
                    _ => 0.0,
                };
                
                if !is_initialized {
                    new_val_str = format!("{:.2}", current_val);
                    ctx.data_mut(|d| {
                        d.insert_temp(egui::Id::new("edit_constraint_str"), new_val_str.clone());
                        d.insert_temp(egui::Id::new("edit_constraint_init"), true);
                    });
                }
                
                let mut open = true;
                let title = match c {
                    exact2d_constraint::Constraint::Distance(..) => "Edit Distance",
                    exact2d_constraint::Constraint::DistanceX(..) => "Edit Horizontal Distance",
                    exact2d_constraint::Constraint::DistanceY(..) => "Edit Vertical Distance",
                    exact2d_constraint::Constraint::Angle(..) => "Edit Angle (deg)",
                    _ => "Constraint Details",
                };
                
                egui::Window::new(title)
                    .open(&mut open)
                    .default_width(150.0)
                    .resizable(false)
                    .show(ctx, |ui| {
                        let has_param = matches!(c, 
                            exact2d_constraint::Constraint::Distance(..) |
                            exact2d_constraint::Constraint::DistanceX(..) |
                            exact2d_constraint::Constraint::DistanceY(..) |
                            exact2d_constraint::Constraint::Angle(..)
                        );
                        if has_param {
                            ui.horizontal(|ui| {
                                ui.text_edit_singleline(&mut new_val_str);
                                if ui.button("Apply").clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                                    if let Ok(new_val) = new_val_str.parse::<f64>() {
                                        let success = if let Some(mut_c) = app.sketch.constraints_mut().get_mut(idx) {
                                            match mut_c {
                                                exact2d_constraint::Constraint::Distance(_, _, val) |
                                                exact2d_constraint::Constraint::DistanceX(_, _, val) |
                                                exact2d_constraint::Constraint::DistanceY(_, _, val) => {
                                                    *val = new_val;
                                                    true
                                                }
                                                exact2d_constraint::Constraint::Angle(_, _, val) => {
                                                    *val = new_val.to_radians();
                                                    true
                                                }
                                                _ => false,
                                            }
                                        } else {
                                            false
                                        };
                                        if success {
                                            app.solve_constraints();
                                        }
                                        ctx.data_mut(|d| {
                                            d.insert_temp::<Option<usize>>(egui::Id::new("edit_constraint_idx"), None);
                                            d.insert_temp(egui::Id::new("edit_constraint_init"), false);
                                        });
                                    }
                                }
                            });
                        } else {
                            ui.label("Geometric constraint has no parameters.");
                            if ui.button("Delete Constraint").clicked() {
                                app.remove_constraint(idx);
                                ctx.data_mut(|d| {
                                    d.insert_temp::<Option<usize>>(egui::Id::new("edit_constraint_idx"), None);
                                    d.insert_temp(egui::Id::new("edit_constraint_init"), false);
                                });
                            }
                        }
                    });
                    
                ctx.data_mut(|d| {
                    d.insert_temp(egui::Id::new("edit_constraint_str"), new_val_str);
                });
                
                if !open {
                    ctx.data_mut(|d| {
                        d.insert_temp::<Option<usize>>(egui::Id::new("edit_constraint_idx"), None);
                        d.insert_temp(egui::Id::new("edit_constraint_init"), false);
                    });
                }
            }
        }
    }
}

// ── Menu bar (spec: File/Edit/View/Draw/Modify/Tools/Help) ────────────────────

fn canvas(ctx: &Context, app: &mut AppState, ui_state: &mut UiState) {
    CentralPanel::default().show(ctx, |ui| {
        let avail = ui.available_size();
        app.view.width = avail.x as f64;
        app.view.height = avail.y as f64;
        // Re-bound the zoom to the active unit's precision-safe range (cheap; the
        // unit can change via the Units menu at any time).
        app.sync_zoom_limits();

        // Sense clicks AND drags over the whole canvas.
        let (rect, response) = ui.allocate_exact_size(avail, Sense::click_and_drag());
        let origin = rect.min;
        let painter = ui.painter_at(rect);

        // ── Input (all mutations happen here, before drawing) ──
        let to_screen_input = |wx: f64, wy: f64, view: &crate::view_transform::ViewTransform| {
            let (sx, sy) = view.world_to_screen(wx, wy);
            pos2(origin.x + sx as f32, origin.y + sy as f32)
        };

        #[derive(Clone, Copy)]
        enum UiGripKind {
            NormalPoint(exact2d_constraint::PointId),
            Midpoint,
            Quadrant(usize),
        }

        #[derive(Clone, Copy)]
        struct UiGrip {
            entity_id: exact2d_document::EntityId,
            kind: UiGripKind,
            pos: egui::Pos2,
        }

        let mut grips = Vec::new();
        if matches!(app.tool, Tool::Select) {
            for &sel_id in &app.selection {
                if let Some(e) = app.document.get(sel_id) {
                    if let Some(pts) = app.entity_points.get(&sel_id) {
                        match &e.kind {
                            EntityKind::Curve(Curve::Line(_)) => {
                                if pts.len() == 2 {
                                    let (x0, y0) = app.sketch.point(pts[0]);
                                    let (x1, y1) = app.sketch.point(pts[1]);
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::NormalPoint(pts[0]),
                                        pos: to_screen_input(x0, y0, &app.view),
                                    });
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::NormalPoint(pts[1]),
                                        pos: to_screen_input(x1, y1, &app.view),
                                    });
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::Midpoint,
                                        pos: to_screen_input((x0 + x1) / 2.0, (y0 + y1) / 2.0, &app.view),
                                    });
                                }
                            }
                            EntityKind::Curve(Curve::Arc(_)) => {
                                if pts.len() == 3 {
                                    let (cx, cy) = app.sketch.point(pts[0]);
                                    let (ax, ay) = app.sketch.point(pts[1]);
                                    let (bx, by) = app.sketch.point(pts[2]);
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::NormalPoint(pts[0]),
                                        pos: to_screen_input(cx, cy, &app.view),
                                    });
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::NormalPoint(pts[1]),
                                        pos: to_screen_input(ax, ay, &app.view),
                                    });
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::NormalPoint(pts[2]),
                                        pos: to_screen_input(bx, by, &app.view),
                                    });
                                    let r = ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt();
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::Quadrant(0),
                                        pos: to_screen_input(cx + r, cy, &app.view),
                                    });
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::Quadrant(1),
                                        pos: to_screen_input(cx, cy + r, &app.view),
                                    });
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::Quadrant(2),
                                        pos: to_screen_input(cx - r, cy, &app.view),
                                    });
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::Quadrant(3),
                                        pos: to_screen_input(cx, cy - r, &app.view),
                                    });
                                }
                            }
                            EntityKind::Curve(Curve::Bezier(_)) => {
                                if pts.len() == 4 {
                                    for &pt in pts {
                                        let (px, py) = app.sketch.point(pt);
                                        grips.push(UiGrip {
                                            entity_id: sel_id,
                                            kind: UiGripKind::NormalPoint(pt),
                                            pos: to_screen_input(px, py, &app.view),
                                        });
                                    }
                                }
                            }
                            _ => {
                                for &pt in pts {
                                    let (px, py) = app.sketch.point(pt);
                                    grips.push(UiGrip {
                                        entity_id: sel_id,
                                        kind: UiGripKind::NormalPoint(pt),
                                        pos: to_screen_input(px, py, &app.view),
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }

        let mut drag_grip_active: bool = ctx.data(|d| d.get_temp(egui::Id::new("drag_grip_active")).unwrap_or(false));
        
        if matches!(app.tool, Tool::Select) {
            if response.drag_started_by(egui::PointerButton::Primary) {
                if let Some(p) = response.interact_pointer_pos().or_else(|| response.hover_pos()) {
                    let mut clicked_grip = None;
                    for grip in &grips {
                        let dist = (p.x - grip.pos.x).hypot(p.y - grip.pos.y);
                        if dist <= 8.0 {
                            clicked_grip = Some(*grip);
                            break;
                        }
                    }
                    if let Some(grip) = clicked_grip {
                        app.history.snapshot(&app.document, &app.sketch, &app.entity_points);
                        ctx.data_mut(|d| {
                            d.insert_temp(egui::Id::new("drag_grip_active"), true);
                            d.insert_temp(egui::Id::new("drag_grip_entity"), grip.entity_id.0);
                            d.insert_temp(egui::Id::new("drag_start_world"), app.cursor_world);
                        });
                        drag_grip_active = true;
                        
                        match grip.kind {
                            UiGripKind::NormalPoint(pt_id) => {
                                let (px, py) = app.sketch.point(pt_id);
                                ctx.data_mut(|d| {
                                    d.insert_temp(egui::Id::new("drag_grip_kind"), 0usize);
                                    d.insert_temp(egui::Id::new("drag_grip_point"), Some(pt_id));
                                    d.insert_temp(egui::Id::new("drag_grip_init_points"), vec![(pt_id, px, py)]);
                                });
                            }
                            UiGripKind::Midpoint => {
                                if let Some(pts) = app.entity_points.get(&grip.entity_id) {
                                    let init_points: Vec<(exact2d_constraint::PointId, f64, f64)> = pts.iter().map(|&pt_id| {
                                        let (px, py) = app.sketch.point(pt_id);
                                        (pt_id, px, py)
                                    }).collect();
                                    ctx.data_mut(|d| {
                                        d.insert_temp(egui::Id::new("drag_grip_kind"), 1usize);
                                        d.insert_temp(egui::Id::new("drag_grip_init_points"), init_points);
                                    });
                                }
                            }
                            UiGripKind::Quadrant(q_idx) => {
                                if let Some(pts) = app.entity_points.get(&grip.entity_id) {
                                    let init_points: Vec<(exact2d_constraint::PointId, f64, f64)> = pts.iter().map(|&pt_id| {
                                        let (px, py) = app.sketch.point(pt_id);
                                        (pt_id, px, py)
                                    }).collect();
                                    ctx.data_mut(|d| {
                                        d.insert_temp(egui::Id::new("drag_grip_kind"), 2usize);
                                        d.insert_temp(egui::Id::new("drag_grip_quadrant_idx"), q_idx);
                                        d.insert_temp(egui::Id::new("drag_grip_init_points"), init_points);
                                    });
                                }
                            }
                        }
                    }
                }
            }
            
            if drag_grip_active && response.dragged_by(egui::PointerButton::Primary) {
                let drag_start_world: (f64, f64) = ctx.data(|d| d.get_temp(egui::Id::new("drag_start_world")).unwrap_or_default());
                let drag_grip_kind: usize = ctx.data(|d| d.get_temp(egui::Id::new("drag_grip_kind")).unwrap_or(0));
                let drag_grip_init_points: Vec<(exact2d_constraint::PointId, f64, f64)> = ctx.data(|d| d.get_temp(egui::Id::new("drag_grip_init_points")).unwrap_or_default());
                
                let current_world = app.resolved_point();
                let dx = current_world.x.to_f64() - drag_start_world.0;
                let dy = current_world.y.to_f64() - drag_start_world.1;
                
                match drag_grip_kind {
                    0 => {
                        if let Some(&(pt_id, ix, iy)) = drag_grip_init_points.first() {
                            app.sketch.set_point(pt_id, ix + dx, iy + dy);
                        }
                    }
                    1 => {
                        for &(pt_id, ix, iy) in &drag_grip_init_points {
                            app.sketch.set_point(pt_id, ix + dx, iy + dy);
                        }
                    }
                    2
                        if drag_grip_init_points.len() >= 2 => {
                            let (_center_pt_id, cx, cy) = drag_grip_init_points[0];
                            let (_start_pt_id, ax, ay) = drag_grip_init_points[1];
                            
                            let r_old = ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().max(1e-3);
                            let r_new = ((current_world.x.to_f64() - cx).powi(2) + (current_world.y.to_f64() - cy).powi(2)).sqrt().max(1e-3);
                            let scale = r_new / r_old;
                            
                            for &(pt_id, ix, iy) in &drag_grip_init_points[1..] {
                                let new_x = cx + (ix - cx) * scale;
                                let new_y = cy + (iy - cy) * scale;
                                app.sketch.set_point(pt_id, new_x, new_y);
                            }
                        }
                    _ => {}
                }
                app.solve_constraints();
                app.sync_document_from_sketch();
            }
            
            if response.drag_stopped() {
                ctx.data_mut(|d| {
                    d.insert_temp(egui::Id::new("drag_grip_active"), false);
                });
            }
        }

        let mut hovered_grip_idx = None;
        if matches!(app.tool, Tool::Select) {
            if let Some(p) = response.hover_pos() {
                for (idx, grip) in grips.iter().enumerate() {
                    let dist = (p.x - grip.pos.x).hypot(p.y - grip.pos.y);
                    if dist <= 8.0 {
                        hovered_grip_idx = Some(idx);
                        break;
                    }
                }
            }
        }

        // Track the cursor whenever it is over the canvas.
        if let Some(p) = response.hover_pos() {
            app.pointer_moved((p.x - origin.x) as f64, (p.y - origin.y) as f64);
        }
        // A left click feeds the active tool. For drawing/picking tools we trigger
        // on the button *press* rather than egui's `clicked()`: `clicked()` is
        // suppressed when the pointer moves even a hair between press and release
        // (egui reclassifies it as a drag, since the canvas senses click_and_drag),
        // so a fast or slightly-moving click was being silently dropped. Acting on
        // press makes placement instant and drag-tolerant — and matches how CAD
        // tools place points. Select mode keeps `clicked()` so press-and-drag still
        // drives grip editing and rubber-band selection.
        let place_point = if matches!(app.tool, Tool::Select) {
            response.clicked()
        } else {
            response.contains_pointer() && ui.input(|i| i.pointer.primary_pressed())
        };
        if place_point {
            if let Some(p) = response.interact_pointer_pos().or_else(|| response.hover_pos()) {
                let mut clicked_any_grip = false;
                if matches!(app.tool, Tool::Select) {
                    for grip in &grips {
                        let dist = (p.x - grip.pos.x).hypot(p.y - grip.pos.y);
                        if dist <= 8.0 {
                            clicked_any_grip = true;
                            break;
                        }
                    }
                }
                if !clicked_any_grip {
                    let was_dimension_stage2 = match &app.tool {
                        Tool::Dimension { stage: 2, p1: Some(pt1), p2: Some(pt2) } => Some((*pt1, *pt2)),
                        _ => None,
                    };

                    app.canvas_click((p.x - origin.x) as f64, (p.y - origin.y) as f64);

                    if let Some((pt1, pt2)) = was_dimension_stage2 {
                        if let Some(new_c_idx) = app.sketch.constraints().len().checked_sub(1) {
                            let (x1, y1) = app.sketch.point(pt1);
                            let (x2, y2) = app.sketch.point(pt2);
                            let (mx, my) = app.cursor_world;
                            
                            let is_arc = find_associated_arc(app, pt1, pt2);
                            let offset = if is_arc {
                                let dx = mx - x1;
                                let dy = my - y1;
                                (dx * dx + dy * dy).sqrt()
                            } else {
                                let dx = x2 - x1;
                                let dy = y2 - y1;
                                let len = (dx * dx + dy * dy).sqrt();
                                let mid_x = (x1 + x2) / 2.0;
                                let mid_y = (y1 + y2) / 2.0;
                                let (nx, ny) = if len > 1e-6 { (-dy / len, dx / len) } else { (0.0, 1.0) };
                                
                                let angle_deg = (my - mid_y).atan2(mx - mid_x).to_degrees().abs();
                                if angle_deg > 67.5 && angle_deg < 112.5 {
                                    my - y1
                                } else if !(22.5..=157.5).contains(&angle_deg) {
                                    mx - x1
                                } else {
                                    (mx - mid_x) * nx + (my - mid_y) * ny
                                }
                            };
                            
                            ctx.data_mut(|d| {
                                d.insert_temp(egui::Id::new(("dim_offset", new_c_idx)), offset);
                            });
                        }
                    }
                }
            }
        }
        // Esc cancels the in-progress tool input and returns to SELECT tool.
        if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
            app.execute(Command::Cancel);
        }
        // Enter or Space commits the active drawing tool (like Polyline)
        if ui.input(|i| i.key_pressed(egui::Key::Enter) || i.key_pressed(egui::Key::Space))
            && ctx.memory(|mem| mem.focused()) != Some(egui::Id::new("command_line_input")) {
                app.run_command("");
            }
        // Right click: commit a polyline; repeat the last command when idle;
        // otherwise act like Enter (finish/cancel the active tool). Mirrors AutoCAD.
        if response.secondary_clicked() {
            match app.tool {
                Tool::Polyline { .. } => app.run_command(""),
                Tool::Select => app.repeat_last_command(),
                _ => app.run_command(""),
            }
        }
        // Automatically focus command input if user starts typing when hovering the canvas and command input is not focused
        let focused_id = ctx.memory(|mem| mem.focused());
        let cmd_input_id = egui::Id::new("command_line_input");
        let mut focus_cmd = false;
        let mut text_to_append = String::new();
        if response.hovered() && focused_id != Some(cmd_input_id) {
            ui.input(|i| {
                for event in &i.events {
                    if let egui::Event::Text(text) = event {
                        let clean: String = text.chars().filter(|c| !c.is_control()).collect();
                        if !clean.is_empty() {
                            text_to_append.push_str(&clean);
                            focus_cmd = true;
                        }
                    }
                }
            });
        }
        if focus_cmd {
            ui_state.command_input.push_str(&text_to_append);
            ctx.memory_mut(|mem| mem.request_focus(cmd_input_id));
        }
        // AutoCAD hotkeys: F7 = Grid, F8 = Ortho, F9 = Snap
        if ui.input(|i| i.key_pressed(egui::Key::F7)) {
            app.grid_on = !app.grid_on;
        }
        if ui.input(|i| i.key_pressed(egui::Key::F8)) {
            app.ortho_on = !app.ortho_on;
        }
        if ui.input(|i| i.key_pressed(egui::Key::F9)) {
            app.snap_on = !app.snap_on;
        }
        // Middle-drag pans.
        if response.dragged_by(egui::PointerButton::Middle) {
            let d = response.drag_delta();
            app.view.pan_pixels(d.x as f64, d.y as f64);
        }
        // Scroll wheel zooms at the cursor (only while hovering the canvas).
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.0 {
                let factor = (scroll as f64 / 200.0).exp();
                let (wx, wy) = app.cursor_world;
                app.view.zoom_at(wx, wy, factor);
            }
        }

        // ── Drawing (immutable borrows only) ──
        let to_screen = |wx: f64, wy: f64| {
            let (sx, sy) = app.view.world_to_screen(wx, wy);
            pos2(origin.x + sx as f32, origin.y + sy as f32)
        };

        // Background, grid, then entities — drawn with the adaptive-tessellation
        // egui painter (smooth at any zoom).
        painter.rect_filled(rect, 0.0, Color32::from_rgb(20, 26, 36));
        if app.grid_on { draw_grid(&painter, app, rect, &to_screen); }
        for e in app.document.iter() {
            let (r, g, b) = resolve_color(app, e);
            let selected = app.selection.contains(&e.id);
            let color = if selected { Color32::from_rgb(0, 200, 255) } else { Color32::from_rgb(r, g, b) };
            let stroke = Stroke::new(if selected { 2.5 } else { 1.5 }, color);
            draw_entity(&painter, app, e, origin, stroke);
        }

        // Draw transparent dashed CV polygon for selected splines
        for e in app.document.iter() {
            if app.selection.contains(&e.id) {
                if let EntityKind::Curve(Curve::Bezier(_)) = &e.kind {
                    if let Some(pts) = app.entity_points.get(&e.id) {
                        if pts.len() == 4 {
                            let (x0, y0) = app.sketch.point(pts[0]);
                            let (x1, y1) = app.sketch.point(pts[1]);
                            let (x2, y2) = app.sketch.point(pts[2]);
                            let (x3, y3) = app.sketch.point(pts[3]);
                            let p0 = to_screen(x0, y0);
                            let p1 = to_screen(x1, y1);
                            let p2 = to_screen(x2, y2);
                            let p3 = to_screen(x3, y3);
                            
                            let cv_poly_stroke = Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 255, 100)); // transparent dashed white
                            draw_dashed_line(&painter, p0, p1, cv_poly_stroke, 4.0, 4.0);
                            draw_dashed_line(&painter, p1, p2, cv_poly_stroke, 4.0, 4.0);
                            draw_dashed_line(&painter, p2, p3, cv_poly_stroke, 4.0, 4.0);
                        }
                    }
                }
            }
        }

        // Active tracking guide (dashed orange line)
        if let Some(((rx, ry), angle_rad)) = app.active_guide {
            let view_diag = (app.view.width * app.view.width + app.view.height * app.view.height).sqrt();
            let world_length = view_diag * app.view.pixel_world_size() * 2.0;
            let p_start = to_screen(rx, ry);
            let p_end = to_screen(rx + world_length * angle_rad.cos(), ry + world_length * angle_rad.sin());
            let guide_stroke = Stroke::new(1.0, Color32::from_rgb(255, 140, 0)); // dashed orange
            draw_dashed_line(&painter, p_start, p_end, guide_stroke, 6.0, 6.0);
        }

        // Rubber-band preview for the active tool.
        let cursor = Point2d::from_f64(app.cursor_world.0, app.cursor_world.1);
        let preview_stroke = Stroke::new(1.5, Color32::from_rgb(130, 200, 130));
        for c in app.tool.preview(&cursor) {
            draw_curve(&painter, &c, &to_screen, preview_stroke);
        }

        if let Tool::Dimension { stage: 2, p1: Some(pt1), p2: Some(pt2) } = &app.tool {
            let (x1, y1) = app.sketch.point(*pt1);
            let (x2, y2) = app.sketch.point(*pt2);
            let sc1 = to_screen(x1, y1);
            let sc2 = to_screen(x2, y2);
            let (mx, my) = app.cursor_world;
            let cursor_screen = to_screen(mx, my);
            
            let is_arc = find_associated_arc(app, *pt1, *pt2);
            let preview_color = Color32::from_rgba_unmultiplied(130, 200, 130, 180);
            let preview_stroke_dim = Stroke::new(1.0, preview_color);
            
            if is_arc {
                let dx = mx - x1;
                let dy = my - y1;
                let dist = (dx * dx + dy * dy).sqrt();
                if dist > 1e-6 {
                    let r = ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt();
                    let ux = dx / dist;
                    let uy = dy / dist;
                    let b_w_x = x1 + ux * r;
                    let b_w_y = y1 + uy * r;
                    let b_screen = to_screen(b_w_x, b_w_y);
                    
                    painter.line_segment([b_screen, cursor_screen], preview_stroke_dim);
                    
                    let dir_away = egui::vec2(ux as f32, uy as f32);
                    draw_arrowhead_points(&painter, b_screen, dir_away, preview_color);
                    
                    let sign = if ux >= 0.0 { 1.0f32 } else { -1.0f32 };
                    let shoulder_end = pos2(cursor_screen.x + 10.0 * sign, cursor_screen.y);
                    painter.line_segment([cursor_screen, shoulder_end], preview_stroke_dim);
                    
                    let label = format!("R {:.2}", r);
                    let align = if ux >= 0.0 { egui::Align2::LEFT_BOTTOM } else { egui::Align2::RIGHT_BOTTOM };
                    painter.text(pos2(cursor_screen.x + 2.0 * sign, cursor_screen.y - 2.0), align, label,
                        egui::FontId::monospace(10.0), preview_color);
                }
            } else {
                let mid_x = (x1 + x2) / 2.0;
                let mid_y = (y1 + y2) / 2.0;
                let angle_deg = (my - mid_y).atan2(mx - mid_x).to_degrees().abs();
                
                let dim_type = if angle_deg > 67.5 && angle_deg < 112.5 {
                    DimType::Horizontal
                } else if !(22.5..=157.5).contains(&angle_deg) {
                    DimType::Vertical
                } else {
                    DimType::Aligned
                };
                
                let val = match dim_type {
                    DimType::Horizontal => (x1 - x2).abs(),
                    DimType::Vertical => (y1 - y2).abs(),
                    DimType::Aligned => ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt(),
                };
                
                let (o1, o2, text_pos, align) = match dim_type {
                    DimType::Horizontal => {
                        let o1 = pos2(sc1.x, cursor_screen.y);
                        let o2 = pos2(sc2.x, cursor_screen.y);
                        let tx = (o1.x + o2.x) / 2.0;
                        (o1, o2, pos2(tx, cursor_screen.y - 8.0), egui::Align2::CENTER_BOTTOM)
                    }
                    DimType::Vertical => {
                        let o1 = pos2(cursor_screen.x, sc1.y);
                        let o2 = pos2(cursor_screen.x, sc2.y);
                        let ty = (o1.y + o2.y) / 2.0;
                        (o1, o2, pos2(cursor_screen.x - 8.0, ty), egui::Align2::RIGHT_CENTER)
                    }
                    DimType::Aligned => {
                        let dx = x2 - x1;
                        let dy = y2 - y1;
                        let len = (dx * dx + dy * dy).sqrt();
                        let (nx, ny) = if len > 1e-6 { (-dy / len, dx / len) } else { (0.0, 1.0) };
                        let offset = (mx - mid_x) * nx + (my - mid_y) * ny;
                        
                        let o1 = to_screen(x1 + nx * offset, y1 + ny * offset);
                        let o2 = to_screen(x2 + nx * offset, y2 + ny * offset);
                        let tx = (o1.x + o2.x) / 2.0;
                        let ty = (o1.y + o2.y) / 2.0;
                        
                        let dx_s = o2.x - o1.x;
                        let dy_s = o2.y - o1.y;
                        let len_s = (dx_s * dx_s + dy_s * dy_s).sqrt();
                        let (nx_s, ny_s) = if len_s > 1e-6 { (-dy_s / len_s, dx_s / len_s) } else { (0.0, 1.0) };
                        
                        (o1, o2, pos2(tx + nx_s * 8.0, ty + ny_s * 8.0), egui::Align2::CENTER_CENTER)
                    }
                };
                
                painter.line_segment([o1, o2], preview_stroke_dim);
                
                let ext_stroke = Stroke::new(0.5, Color32::from_rgba_unmultiplied(255, 255, 255, 60));
                painter.line_segment([sc1, o1], ext_stroke);
                painter.line_segment([sc2, o2], ext_stroke);
                
                let d_vec = o2 - o1;
                let d_len = d_vec.length();
                if d_len > 12.0 {
                    let dir1 = d_vec / d_len;
                    let dir2 = -dir1;
                    draw_arrowhead_points(&painter, o1, dir1, preview_color);
                    draw_arrowhead_points(&painter, o2, dir2, preview_color);
                }
                
                let label = format!("{:.2}", val);
                painter.text(text_pos, align, label, egui::FontId::monospace(10.0), preview_color);
            }
        }

        // Constraint overlays on the canvas (only shown for selected entities)
        if app.constraints_enabled {
            let stroke_const = Stroke::new(1.5, Color32::from_rgb(0, 220, 100)); // Constraint bright green

            // Collect selected points so we only render constraints associated with selected entities!
            let mut selected_point_ids = std::collections::HashSet::new();
            for &sel_id in &app.selection {
                if let Some(pts) = app.entity_points.get(&sel_id) {
                    for &p in pts {
                        selected_point_ids.insert(p);
                    }
                }
            }

            for (idx, c) in app.sketch.constraints().iter().enumerate() {
                let is_dimension = matches!(c,
                    exact2d_constraint::Constraint::Distance(..) |
                    exact2d_constraint::Constraint::DistanceX(..) |
                    exact2d_constraint::Constraint::DistanceY(..)
                );
                if !is_dimension && !constraint_references_any_point(c, &selected_point_ids) {
                    continue;
                }

                let mut badge_pos = None;
                match *c {
                    exact2d_constraint::Constraint::Horizontal(p1, p2) => {
                        let (x1, y1) = app.sketch.point(p1);
                        let (x2, y2) = app.sketch.point(p2);
                        let mx = (x1 + x2) / 2.0;
                        let my = (y1 + y2) / 2.0;
                        let sc = to_screen(mx, my);
                        painter.line_segment([pos2(sc.x - 6.0, sc.y), pos2(sc.x + 6.0, sc.y)], stroke_const);
                        badge_pos = Some(sc);
                    }
                    exact2d_constraint::Constraint::Vertical(p1, p2) => {
                        let (x1, y1) = app.sketch.point(p1);
                        let (x2, y2) = app.sketch.point(p2);
                        let mx = (x1 + x2) / 2.0;
                        let my = (y1 + y2) / 2.0;
                        let sc = to_screen(mx, my);
                        painter.line_segment([pos2(sc.x, sc.y - 6.0), pos2(sc.x, sc.y + 6.0)], stroke_const);
                        badge_pos = Some(sc);
                    }
                    exact2d_constraint::Constraint::Fix(_p, x, y) => {
                        let sc = to_screen(x, y);
                        painter.circle_stroke(sc, 5.0, Stroke::new(1.5, Color32::from_rgb(255, 100, 100)));
                        painter.line_segment([pos2(sc.x - 3.0, sc.y - 3.0), pos2(sc.x + 3.0, sc.y + 3.0)], Stroke::new(1.0, Color32::from_rgb(255, 100, 100)));
                        painter.line_segment([pos2(sc.x - 3.0, sc.y + 3.0), pos2(sc.x + 3.0, sc.y - 3.0)], Stroke::new(1.0, Color32::from_rgb(255, 100, 100)));
                        badge_pos = Some(sc);
                    }
                    exact2d_constraint::Constraint::Parallel(l1, l2) => {
                        let draw_parallel = |p1, p2| {
                            let (x1, y1) = app.sketch.point(p1);
                            let (x2, y2) = app.sketch.point(p2);
                            let mx = (x1 + x2) / 2.0;
                            let my = (y1 + y2) / 2.0;
                            let sc = to_screen(mx, my);
                            painter.line_segment([pos2(sc.x - 3.0, sc.y - 5.0), pos2(sc.x - 1.0, sc.y + 5.0)], stroke_const);
                            painter.line_segment([pos2(sc.x + 1.0, sc.y - 5.0), pos2(sc.x + 3.0, sc.y + 5.0)], stroke_const);
                            sc
                        };
                        let sc1 = draw_parallel(l1.0, l1.1);
                        let _sc2 = draw_parallel(l2.0, l2.1);
                        badge_pos = Some(sc1);
                    }
                    exact2d_constraint::Constraint::Perpendicular(l1, l2) => {
                        let draw_perp = |p1, p2| {
                            let (x1, y1) = app.sketch.point(p1);
                            let (x2, y2) = app.sketch.point(p2);
                            let mx = (x1 + x2) / 2.0;
                            let my = (y1 + y2) / 2.0;
                            let sc = to_screen(mx, my);
                            painter.line_segment([pos2(sc.x - 4.0, sc.y + 4.0), pos2(sc.x + 4.0, sc.y + 4.0)], stroke_const);
                            painter.line_segment([pos2(sc.x, sc.y + 4.0), pos2(sc.x, sc.y - 4.0)], stroke_const);
                            sc
                        };
                        let sc1 = draw_perp(l1.0, l1.1);
                        let _sc2 = draw_perp(l2.0, l2.1);
                        badge_pos = Some(sc1);
                    }
                    exact2d_constraint::Constraint::Coincident(p1, _p2) => {
                        let (x, y) = app.sketch.point(p1);
                        let sc = to_screen(x, y);
                        painter.rect_stroke(egui::Rect::from_center_size(sc, vec2(5.0, 5.0)), 0.0, stroke_const);
                        badge_pos = Some(sc);
                    }
                    exact2d_constraint::Constraint::Midpoint(m, _a, _b) => {
                        let (x, y) = app.sketch.point(m);
                        let sc = to_screen(x, y);
                        let p1 = pos2(sc.x, sc.y - 4.0);
                        let p2 = pos2(sc.x - 4.0, sc.y + 3.0);
                        let p3 = pos2(sc.x + 4.0, sc.y + 3.0);
                        painter.line_segment([p1, p2], stroke_const);
                        painter.line_segment([p2, p3], stroke_const);
                        painter.line_segment([p3, p1], stroke_const);
                        badge_pos = Some(sc);
                    }
                    exact2d_constraint::Constraint::Collinear(p1, _p2, _p3) => {
                        let (x1, y1) = app.sketch.point(p1);
                        let sc = to_screen(x1, y1);
                        painter.text(sc, egui::Align2::CENTER_CENTER, "c", egui::FontId::monospace(8.0), stroke_const.color);
                        badge_pos = Some(sc);
                    }
                    exact2d_constraint::Constraint::EqualLength(l1, l2) => {
                        let draw_equal = |p1, p2| {
                            let (x1, y1) = app.sketch.point(p1);
                            let (x2, y2) = app.sketch.point(p2);
                            let mx = (x1 + x2) / 2.0;
                            let my = (y1 + y2) / 2.0;
                            let sc = to_screen(mx, my);
                            painter.text(sc, egui::Align2::CENTER_CENTER, "=", egui::FontId::monospace(10.0), stroke_const.color);
                            sc
                        };
                        let sc1 = draw_equal(l1.0, l1.1);
                        let _sc2 = draw_equal(l2.0, l2.1);
                        badge_pos = Some(sc1);
                    }
                    exact2d_constraint::Constraint::TangentLineCircle(line, center, _start) => {
                        let (cx, cy) = app.sketch.point(center);
                        let (ax, ay) = app.sketch.point(line.0);
                        let (bx, by) = app.sketch.point(line.1);
                        let ux = bx - ax;
                        let uy = by - ay;
                        let l_sq = ux * ux + uy * uy;
                        if l_sq > 1e-9 {
                            let t = ((cx - ax) * ux + (cy - ay) * uy) / l_sq;
                            let px = ax + t.clamp(0.0, 1.0) * ux;
                            let py = ay + t.clamp(0.0, 1.0) * uy;
                            let sc = to_screen(px, py);
                            painter.circle_stroke(sc, 4.0, stroke_const);
                            painter.line_segment([pos2(sc.x - 6.0, sc.y + 4.0), pos2(sc.x + 6.0, sc.y + 4.0)], stroke_const);
                            badge_pos = Some(sc);
                        }
                    }
                    exact2d_constraint::Constraint::TangentCircleCircle(c1, _s1, c2, _s2, _ext) => {
                        let (x1, y1) = app.sketch.point(c1);
                        let (x2, y2) = app.sketch.point(c2);
                        let mx = (x1 + x2) / 2.0;
                        let my = (y1 + y2) / 2.0;
                        let sc = to_screen(mx, my);
                        painter.circle_stroke(sc, 4.0, stroke_const);
                        painter.circle_stroke(pos2(sc.x + 3.0, sc.y), 3.0, stroke_const);
                        badge_pos = Some(sc);
                    }
                    exact2d_constraint::Constraint::Symmetric(p1, p2, _axis) => {
                        let (x1, y1) = app.sketch.point(p1);
                        let (x2, y2) = app.sketch.point(p2);
                        let sc1 = to_screen(x1, y1);
                        let sc2 = to_screen(x2, y2);
                        let sym_stroke = Stroke::new(0.5, Color32::from_rgb(0, 220, 100));
                        painter.line_segment([sc1, sc2], sym_stroke);
                        let mx = (sc1.x + sc2.x) / 2.0;
                        let my = (sc1.y + sc2.y) / 2.0;
                        painter.text(pos2(mx, my), egui::Align2::CENTER_CENTER, "s", egui::FontId::monospace(8.0), stroke_const.color);
                        badge_pos = Some(pos2(mx, my));
                    }
                    exact2d_constraint::Constraint::Angle(l1, l2, theta) => {
                        let (ax1, ay1) = app.sketch.point(l1.0);
                        let (bx1, by1) = app.sketch.point(l1.1);
                        let (ax2, ay2) = app.sketch.point(l2.0);
                        let (bx2, by2) = app.sketch.point(l2.1);
                        let mx = (ax1 + bx1 + ax2 + bx2) / 4.0;
                        let my = (ay1 + by1 + ay2 + by2) / 4.0;
                        let sc = to_screen(mx, my);
                        let label = format!("{:.1}°", theta.to_degrees());
                        painter.text(sc, egui::Align2::CENTER_CENTER, label,
                            egui::FontId::monospace(10.0), Color32::from_rgb(0, 180, 255));
                        badge_pos = Some(sc);
                    }
                    exact2d_constraint::Constraint::Distance(p1, p2, val) |
                    exact2d_constraint::Constraint::DistanceX(p1, p2, val) |
                    exact2d_constraint::Constraint::DistanceY(p1, p2, val) => {
                        let (x1, y1) = app.sketch.point(p1);
                        let (x2, y2) = app.sketch.point(p2);
                        let sc1 = to_screen(x1, y1);
                        let sc2 = to_screen(x2, y2);
                        
                        let is_arc = find_associated_arc(app, p1, p2);
                        let dim_color = Color32::from_rgb(0, 180, 255);
                        let dim_stroke = Stroke::new(1.0, dim_color);
                        
                        let offset = ctx.data(|d| d.get_temp(egui::Id::new(("dim_offset", idx)))).unwrap_or(25.0 * app.view.pixel_world_size());
                        
                        if is_arc {
                            let center_screen = sc1;
                            let start_screen = sc2;
                            let dx = start_screen.x - center_screen.x;
                            let dy = start_screen.y - center_screen.y;
                            let r_screen = (dx * dx + dy * dy).sqrt();
                            
                            let angle_rad = 45.0f64.to_radians();
                            let ux = angle_rad.cos() as f32;
                            let uy = angle_rad.sin() as f32;
                            
                            let leader_len_screen = (offset / app.view.pixel_world_size()) as f32;
                            let boundary_screen = pos2(center_screen.x + ux * r_screen, center_screen.y + uy * r_screen);
                            let tip_screen = pos2(center_screen.x + ux * leader_len_screen, center_screen.y + uy * leader_len_screen);
                            
                            painter.line_segment([boundary_screen, tip_screen], dim_stroke);
                            
                            let dir_away = egui::vec2(ux, uy);
                            draw_arrowhead_points(&painter, boundary_screen, dir_away, dim_color);
                            
                            let sign = if ux >= 0.0 { 1.0f32 } else { -1.0f32 };
                            let shoulder_end = pos2(tip_screen.x + 10.0 * sign, tip_screen.y);
                            painter.line_segment([tip_screen, shoulder_end], dim_stroke);
                            
                            let label = format!("R {:.2}", val);
                            let align = if ux >= 0.0 { egui::Align2::LEFT_BOTTOM } else { egui::Align2::RIGHT_BOTTOM };
                            let text_pos = pos2(tip_screen.x + 2.0 * sign, tip_screen.y - 2.0);
                            
                            painter.text(text_pos, align, label.clone(), egui::FontId::monospace(10.0), dim_color);
                            badge_pos = Some(text_pos);
                            
                            let galley = painter.layout_no_wrap(label, egui::FontId::monospace(10.0), dim_color);
                            let label_rect = egui::Rect::from_center_size(text_pos, galley.size()).expand(4.0);
                            let response = ui.interact(label_rect, egui::Id::new(("dim_label_drag", idx)), egui::Sense::drag());
                            if response.dragged() {
                                if let Some(mouse_pos) = ctx.pointer_latest_pos() {
                                    let (mx, my) = app.view.screen_to_world(mouse_pos.x as f64, mouse_pos.y as f64);
                                    let dx = mx - x1;
                                    let dy = my - y1;
                                    let new_offset = (dx * dx + dy * dy).sqrt();
                                    ctx.data_mut(|d| d.insert_temp(egui::Id::new(("dim_offset", idx)), new_offset));
                                }
                            }
                            if response.hovered() {
                                painter.rect_stroke(label_rect, 2.0, Stroke::new(1.0, Color32::from_rgb(255, 140, 0)));
                            }
                        } else {
                            let (o1, o2, text_pos, align, actual_dim_type) = match *c {
                                exact2d_constraint::Constraint::DistanceX(..) => {
                                    let offset_screen = (offset / app.view.pixel_world_size()) as f32;
                                    let o1 = pos2(sc1.x, sc1.y + offset_screen);
                                    let o2 = pos2(sc2.x, sc1.y + offset_screen);
                                    let tx = (o1.x + o2.x) / 2.0;
                                    (o1, o2, pos2(tx, sc1.y + offset_screen - 8.0), egui::Align2::CENTER_BOTTOM, DimType::Horizontal)
                                }
                                exact2d_constraint::Constraint::DistanceY(..) => {
                                    let offset_screen = (offset / app.view.pixel_world_size()) as f32;
                                    let o1 = pos2(sc1.x + offset_screen, sc1.y);
                                    let o2 = pos2(sc1.x + offset_screen, sc2.y);
                                    let ty = (o1.y + o2.y) / 2.0;
                                    (o1, o2, pos2(sc1.x + offset_screen - 8.0, ty), egui::Align2::RIGHT_CENTER, DimType::Vertical)
                                }
                                _ => {
                                    let dx = x2 - x1;
                                    let dy = y2 - y1;
                                    let len = (dx * dx + dy * dy).sqrt();
                                    let (nx, ny) = if len > 1e-6 { (-dy / len, dx / len) } else { (0.0, 1.0) };
                                    
                                    let o1_w_x = x1 + nx * offset;
                                    let o1_w_y = y1 + ny * offset;
                                    let o2_w_x = x2 + nx * offset;
                                    let o2_w_y = y2 + ny * offset;
                                    
                                    let o1 = to_screen(o1_w_x, o1_w_y);
                                    let o2 = to_screen(o2_w_x, o2_w_y);
                                    let tx = (o1.x + o2.x) / 2.0;
                                    let ty = (o1.y + o2.y) / 2.0;
                                    
                                    let dx_s = o2.x - o1.x;
                                    let dy_s = o2.y - o1.y;
                                    let len_s = (dx_s * dx_s + dy_s * dy_s).sqrt();
                                    let (nx_s, ny_s) = if len_s > 1e-6 { (-dy_s / len_s, dx_s / len_s) } else { (0.0, 1.0) };
                                    
                                    (o1, o2, pos2(tx + nx_s * 8.0, ty + ny_s * 8.0), egui::Align2::CENTER_CENTER, DimType::Aligned)
                                }
                            };
                            
                            painter.line_segment([o1, o2], dim_stroke);
                            
                            let ext_stroke = Stroke::new(0.5, Color32::from_rgba_unmultiplied(255, 255, 255, 60));
                            painter.line_segment([sc1, o1], ext_stroke);
                            painter.line_segment([sc2, o2], ext_stroke);
                            
                            let d_vec = o2 - o1;
                            let d_len = d_vec.length();
                            if d_len > 12.0 {
                                let dir1 = d_vec / d_len;
                                let dir2 = -dir1;
                                draw_arrowhead_points(&painter, o1, dir1, dim_color);
                                draw_arrowhead_points(&painter, o2, dir2, dim_color);
                            }
                            
                            let label = format!("{:.2}", val);
                            painter.text(text_pos, align, label.clone(), egui::FontId::monospace(10.0), dim_color);
                            badge_pos = Some(text_pos);
                            
                            let galley = painter.layout_no_wrap(label, egui::FontId::monospace(10.0), dim_color);
                            let label_rect = egui::Rect::from_center_size(text_pos, galley.size()).expand(4.0);
                            let response = ui.interact(label_rect, egui::Id::new(("dim_label_drag", idx)), egui::Sense::drag());
                            if response.dragged() {
                                if let Some(mouse_pos) = ctx.pointer_latest_pos() {
                                    let (mx, my) = app.view.screen_to_world(mouse_pos.x as f64, mouse_pos.y as f64);
                                    let new_offset = match actual_dim_type {
                                        DimType::Horizontal => my - y1,
                                        DimType::Vertical => mx - x1,
                                        DimType::Aligned => {
                                            let dx = x2 - x1;
                                            let dy = y2 - y1;
                                            let len = (dx * dx + dy * dy).sqrt();
                                            let mid_x = (x1 + x2) / 2.0;
                                            let mid_y = (y1 + y2) / 2.0;
                                            let (nx, ny) = if len > 1e-6 { (-dy / len, dx / len) } else { (0.0, 1.0) };
                                            (mx - mid_x) * nx + (my - mid_y) * ny
                                        }
                                    };
                                    ctx.data_mut(|d| d.insert_temp(egui::Id::new(("dim_offset", idx)), new_offset));
                                }
                            }
                            if response.hovered() {
                                painter.rect_stroke(label_rect, 2.0, Stroke::new(1.0, Color32::from_rgb(255, 140, 0)));
                            }
                        }
                    }
                }

                if let Some(sc) = badge_pos {
                    let double_click = ui.input(|i| i.pointer.button_double_clicked(egui::PointerButton::Primary));
                    if double_click {
                        if let Some(click_pos) = ui.input(|i| i.pointer.interact_pos()) {
                            let dx = click_pos.x - sc.x;
                            let dy = click_pos.y - sc.y;
                            if (dx * dx + dy * dy).sqrt() < 15.0 {
                                ui.data_mut(|d| {
                                    d.insert_temp::<Option<usize>>(egui::Id::new("edit_constraint_idx"), Some(idx));
                                    d.insert_temp(egui::Id::new("edit_constraint_init"), false);
                                });
                            }
                        }
                    }
                }
            }
        }

        // Render grips for selected entities
        if matches!(app.tool, Tool::Select) {
            let active_grip_entity: Option<u64> = ctx.data(|d| d.get_temp(egui::Id::new("drag_grip_entity")));
            let active_grip_kind_val: Option<usize> = ctx.data(|d| d.get_temp(egui::Id::new("drag_grip_kind")));
            let active_grip_pt: Option<exact2d_constraint::PointId> = ctx.data(|d| d.get_temp(egui::Id::new("drag_grip_point")).flatten());
            let active_grip_quad_idx: Option<usize> = ctx.data(|d| d.get_temp(egui::Id::new("drag_grip_quadrant_idx")));
            let is_active = ctx.data(|d| d.get_temp(egui::Id::new("drag_grip_active")).unwrap_or(false));

            for (idx, grip) in grips.iter().enumerate() {
                let is_dragging_this = is_active && Some(grip.entity_id.0) == active_grip_entity && match grip.kind {
                    UiGripKind::NormalPoint(pt) => active_grip_kind_val == Some(0) && Some(pt) == active_grip_pt,
                    UiGripKind::Midpoint => active_grip_kind_val == Some(1),
                    UiGripKind::Quadrant(q_idx) => active_grip_kind_val == Some(2) && Some(q_idx) == active_grip_quad_idx,
                };

                let is_hovered = Some(idx) == hovered_grip_idx;
                
                let fill_color = if is_dragging_this {
                    Color32::from_rgb(255, 61, 0) // bright red
                } else if is_hovered {
                    Color32::from_rgb(255, 145, 0) // bright orange
                } else {
                    Color32::from_rgb(0, 168, 255) // nice blue
                };

                let stroke = Stroke::new(1.0, Color32::WHITE);
                let size = if is_hovered || is_dragging_this { 8.0 } else { 6.0 };
                
                painter.rect_filled(
                    egui::Rect::from_center_size(grip.pos, vec2(size, size)),
                    0.0,
                    fill_color,
                );
                painter.rect_stroke(
                    egui::Rect::from_center_size(grip.pos, vec2(size, size)),
                    0.0,
                    stroke,
                );
            }
        }

        // Snap marker (AutoCAD LT style)
        if let Some(sp) = &app.active_snap {
            let c = to_screen(sp.pos.0, sp.pos.1);
            let stroke = Stroke::new(2.0, Color32::from_rgb(0, 255, 0)); // AutoCAD LT green

            match sp.kind {
                exact2d_cad::SnapKind::Endpoint => {
                    // Endpoint: Square
                    painter.rect_stroke(egui::Rect::from_center_size(c, vec2(12.0, 12.0)), 0.0, stroke);
                }
                exact2d_cad::SnapKind::Midpoint => {
                    // Midpoint: Triangle
                    let top = pos2(c.x, c.y - 7.0);
                    let left = pos2(c.x - 7.0, c.y + 6.0);
                    let right = pos2(c.x + 7.0, c.y + 6.0);
                    painter.line_segment([top, left], stroke);
                    painter.line_segment([left, right], stroke);
                    painter.line_segment([right, top], stroke);
                }
                exact2d_cad::SnapKind::Center => {
                    // Center: Circle
                    painter.circle_stroke(c, 6.0, stroke);
                }
                exact2d_cad::SnapKind::Intersection => {
                    // Intersection: X
                    let p1 = pos2(c.x - 6.0, c.y - 6.0);
                    let p2 = pos2(c.x + 6.0, c.y + 6.0);
                    let p3 = pos2(c.x + 6.0, c.y - 6.0);
                    let p4 = pos2(c.x - 6.0, c.y + 6.0);
                    painter.line_segment([p1, p2], stroke);
                    painter.line_segment([p3, p4], stroke);
                }
                exact2d_cad::SnapKind::Perpendicular => {
                    // Perpendicular: Right angle symbol
                    let p1 = pos2(c.x - 6.0, c.y - 6.0);
                    let p2 = pos2(c.x - 6.0, c.y + 6.0);
                    let p3 = pos2(c.x + 6.0, c.y + 6.0);
                    painter.line_segment([p1, p2], stroke);
                    painter.line_segment([p2, p3], stroke);

                    let p4 = pos2(c.x, c.y + 6.0);
                    let p5 = pos2(c.x, c.y);
                    let p6 = pos2(c.x - 6.0, c.y);
                    painter.line_segment([p4, p5], stroke);
                    painter.line_segment([p5, p6], stroke);
                }
                exact2d_cad::SnapKind::Tangent => {
                    // Tangent: Circle with a horizontal line on top
                    painter.circle_stroke(pos2(c.x, c.y + 2.0), 5.0, stroke);
                    let p1 = pos2(c.x - 7.0, c.y - 4.0);
                    let p2 = pos2(c.x + 7.0, c.y - 4.0);
                    painter.line_segment([p1, p2], stroke);
                }
                exact2d_cad::SnapKind::Nearest => {
                    // Nearest: Hourglass
                    let tl = pos2(c.x - 6.0, c.y - 6.0);
                    let tr = pos2(c.x + 6.0, c.y - 6.0);
                    let bl = pos2(c.x - 6.0, c.y + 6.0);
                    let br = pos2(c.x + 6.0, c.y + 6.0);
                    painter.line_segment([tl, tr], stroke);
                    painter.line_segment([tr, bl], stroke);
                    painter.line_segment([bl, br], stroke);
                    painter.line_segment([br, tl], stroke);
                }
                exact2d_cad::SnapKind::Node => {
                    // Node: Circle with a cross (X) inside
                    painter.circle_stroke(c, 6.0, stroke);
                    let p1 = pos2(c.x - 4.2, c.y - 4.2);
                    let p2 = pos2(c.x + 4.2, c.y + 4.2);
                    let p3 = pos2(c.x + 4.2, c.y - 4.2);
                    let p4 = pos2(c.x - 4.2, c.y + 4.2);
                    painter.line_segment([p1, p2], stroke);
                    painter.line_segment([p3, p4], stroke);
                }
                exact2d_cad::SnapKind::Insertion => {
                    // Insertion: Two offset/overlapping squares
                    painter.rect_stroke(egui::Rect::from_center_size(pos2(c.x - 2.0, c.y + 2.0), vec2(8.0, 8.0)), 0.0, stroke);
                    painter.rect_stroke(egui::Rect::from_center_size(pos2(c.x + 2.0, c.y - 2.0), vec2(8.0, 8.0)), 0.0, stroke);
                }
            }
        }
        // Full-canvas crosshair cursor (AutoCAD-style), only while the pointer is
        // over the canvas. In Select mode, add the small square "pickbox" at the
        // centre that AutoCAD shows when no command is active.
        let cc = to_screen(app.cursor_world.0, app.cursor_world.1);
        if response.hovered() {
            let cross = Stroke::new(1.0, Color32::from_rgb(140, 150, 170));
            painter.line_segment([pos2(rect.left(), cc.y), pos2(rect.right(), cc.y)], cross);
            painter.line_segment([pos2(cc.x, rect.top()), pos2(cc.x, rect.bottom())], cross);
            if matches!(app.tool, Tool::Select) {
                painter.rect_stroke(
                    egui::Rect::from_center_size(cc, vec2(10.0, 10.0)), 0.0, cross);
            }
        }

        // Dynamic dimension/distance tooltip next to the cursor (spec §4.2 dynamic input).
        let has_dims = app.tool.has_pending_input();
        let is_drawing = !matches!(app.tool, Tool::Select);
        let has_input = is_drawing || !ui_state.command_input.is_empty() || has_dims;

        if app.dyn_on && (has_dims || has_input) {
            let font_id = egui::FontId::monospace(11.0);
            let text_color = Color32::from_rgb(230, 240, 255);
            let bg_color = Color32::from_rgba_unmultiplied(20, 26, 36, 225);
            let dim_border = Stroke::new(1.0, Color32::from_rgb(80, 95, 115));
            let input_border = Stroke::new(1.0, Color32::from_rgb(0, 255, 0)); // AutoCAD green border

            // Calculate dimensions text (Box 1)
            let dims_text = if has_dims {
                let cursor = Point2d::from_f64(app.cursor_world.0, app.cursor_world.1);
                match &app.tool {
                    Tool::Line { last: Some(p0) } => {
                        let d = p0.dist_f64(&cursor);
                        let (x0, y0) = p0.to_f64();
                        let (x1, y1) = cursor.to_f64();
                        let dx = x1 - x0;
                        let dy = y1 - y0;
                        let mut angle_deg = dy.atan2(dx).to_degrees();
                        if angle_deg < 0.0 { angle_deg += 360.0; }
                        Some(format!("L: {:.4}\nA: {:.1}°", d, angle_deg))
                    }
                    Tool::Circle { center: Some(c) } => {
                        let r = c.dist_f64(&cursor);
                        Some(format!("R: {:.4}", r))
                    }
                    Tool::Rectangle { first: Some(c0) } => {
                        let (x0, y0) = c0.to_f64();
                        let (x1, y1) = cursor.to_f64();
                        let w = (x1 - x0).abs();
                        let h = (y1 - y0).abs();
                        Some(format!("W: {:.4}\nH: {:.4}", w, h))
                    }
                    Tool::Arc3 { pts } => {
                        if pts.len() == 1 {
                            let d = pts[0].dist_f64(&cursor);
                            Some(format!("Dist: {:.4}", d))
                        } else if pts.len() == 2 {
                            match exact2d_geometry::CircularArc::from_three_points(&pts[0], &pts[1], &cursor) {
                                Some(arc) => {
                                    let r = arc.radius.to_f64();
                                    Some(format!("R: {:.4}", r))
                                }
                                None => Some("Collinear".to_string()),
                            }
                        } else {
                            None
                        }
                    }
                    Tool::Move { base: Some(b), .. } => {
                        let d = b.dist_f64(&cursor);
                        let (x0, y0) = b.to_f64();
                        let (x1, y1) = cursor.to_f64();
                        let dx = x1 - x0;
                        let dy = y1 - y0;
                        Some(format!("D: {:.4}\ndx: {:.4}\ndy: {:.4}", d, dx, dy))
                    }
                    Tool::Copy { base: Some(b), .. } => {
                        let d = b.dist_f64(&cursor);
                        let (x0, y0) = b.to_f64();
                        let (x1, y1) = cursor.to_f64();
                        let dx = x1 - x0;
                        let dy = y1 - y0;
                        Some(format!("D: {:.4}\ndx: {:.4}\ndy: {:.4}", d, dx, dy))
                    }
                    Tool::Polygon { center: Some(c), sides } => {
                        let d = c.dist_f64(&cursor);
                        let (x0, y0) = c.to_f64();
                        let (x1, y1) = cursor.to_f64();
                        let dx = x1 - x0;
                        let dy = y1 - y0;
                        let mut angle_deg = dy.atan2(dx).to_degrees();
                        if angle_deg < 0.0 { angle_deg += 360.0; }
                        Some(format!("R: {:.4}\nA: {:.1}°\nSides: {}", d, angle_deg, sides))
                    }
                    Tool::Spline { pts } => {
                        if let Some(last) = pts.last() {
                            let d = last.dist_f64(&cursor);
                            Some(format!("Dist: {:.4}\nPoints: {}/4", d, pts.len()))
                        } else {
                            None
                        }
                    }
                    Tool::Polyline { pts } => {
                        if let Some(last) = pts.last() {
                            let d = last.dist_f64(&cursor);
                            Some(format!("L: {:.4}\nPoints: {}", d, pts.len()))
                        } else {
                            None
                        }
                    }
                    _ => None,
                }
            } else {
                None
            };

            // Calculate input text (Box 2)
            let input_text = if is_drawing {
                let prompt_base = match &app.tool {
                    Tool::Line { last } => {
                        if last.is_none() { "Specify start point: " } else { "Specify length: " }
                    }
                    Tool::Circle { center } => {
                        if center.is_none() { "Specify center point: " } else { "Specify radius: " }
                    }
                    Tool::Rectangle { first } => {
                        if first.is_none() { "Specify first corner: " } else { "Specify opposite corner: " }
                    }
                    Tool::Arc3 { pts } => {
                        if pts.is_empty() {
                            "Specify start point: "
                        } else if pts.len() == 1 {
                            "Specify second point: "
                        } else {
                            "Specify third point: "
                        }
                    }
                    Tool::Move { base, .. } => {
                        if base.is_none() { "Specify base point: " } else { "Specify displacement: " }
                    }
                    Tool::Copy { base, .. } => {
                        if base.is_none() { "Specify base point: " } else { "Specify displacement: " }
                    }
                    Tool::Polygon { center, .. } => {
                        if center.is_none() { "" } else { "Specify radius: " }
                    }
                    Tool::Spline { pts } => {
                        if pts.is_empty() { "Specify start point: " } else { "Specify next point: " }
                    }
                    Tool::Polyline { pts } => {
                        if pts.is_empty() { "Specify start point: " } else { "Specify next point or [Close]: " }
                    }
                    _ => "Specify point: ",
                };
                let prompt = if let Tool::Polygon { center: None, sides } = &app.tool {
                    format!("Specify number of sides <{}>: ", sides)
                } else {
                    prompt_base.to_string()
                };
                Some(format!("{}{}_", prompt, ui_state.command_input))
            } else if !ui_state.command_input.is_empty() {
                Some(format!("Command: {}_", ui_state.command_input))
            } else {
                None
            };

            if dims_text.is_some() || input_text.is_some() {
                let offset = vec2(15.0, 15.0);
                let padding = vec2(6.0, 4.0);

                let mut combined_rect = egui::Rect::NOTHING;
                let mut size1 = vec2(0.0, 0.0);
                let mut size2 = vec2(0.0, 0.0);
                let mut galley1 = None;
                let mut galley2 = None;

                if let Some(t1) = &dims_text {
                    let g1 = painter.layout_no_wrap(t1.clone(), font_id.clone(), text_color);
                    size1 = g1.size() + padding * 2.0;
                    galley1 = Some(g1);
                }
                if let Some(t2) = &input_text {
                    let g2 = painter.layout_no_wrap(t2.clone(), font_id.clone(), text_color);
                    size2 = g2.size() + padding * 2.0;
                    galley2 = Some(g2);
                }

                let mut rect1 = egui::Rect::NOTHING;
                let mut rect2 = egui::Rect::NOTHING;

                if galley1.is_some() && galley2.is_some() {
                    rect1 = egui::Rect::from_min_size(cc + offset, size1);
                    rect2 = egui::Rect::from_min_size(rect1.left_bottom() + vec2(0.0, 5.0), size2);
                    combined_rect = rect1.union(rect2);
                } else if galley1.is_some() {
                    rect1 = egui::Rect::from_min_size(cc + offset, size1);
                    combined_rect = rect1;
                } else if galley2.is_some() {
                    rect2 = egui::Rect::from_min_size(cc + offset, size2);
                    combined_rect = rect2;
                }

                // Constrain combined block to the canvas bounds
                let mut translation = vec2(0.0, 0.0);
                if combined_rect.right() > rect.right() {
                    translation.x = rect.right() - combined_rect.right();
                }
                if combined_rect.bottom() > rect.bottom() {
                    translation.y = rect.bottom() - combined_rect.bottom();
                }
                if combined_rect.left() + translation.x < rect.left() {
                    translation.x = rect.left() - combined_rect.left();
                }
                if combined_rect.top() + translation.y < rect.top() {
                    translation.y = rect.top() - combined_rect.top();
                }

                if let Some(g1) = galley1 {
                    let final_rect1 = rect1.translate(translation);
                    painter.rect(final_rect1, 3.0, bg_color, dim_border);
                    painter.galley(final_rect1.min + padding, g1, text_color);
                }
                if let Some(g2) = galley2 {
                    let final_rect2 = rect2.translate(translation);
                    painter.rect(final_rect2, 3.0, bg_color, input_border);
                    painter.galley(final_rect2.min + padding, g2, text_color);
                }
            }
        }

        // Scale bar (bottom-right): a "nice" round distance whose pixel length and
        // label update live as you zoom.
        draw_scale_bar(&painter, app, rect);

        // Diagnostic HUD (top-left): if "Clicks" rises when you click, input works.
        let hud = format!(
            "Tool: {}   Cursor: {}   Entities: {}   Clicks: {}",
            app.tool.name(), app.coord_readout(), app.document.len(), app.click_count);
        painter.text(rect.left_top() + vec2(8.0, 6.0), egui::Align2::LEFT_TOP, hud,
            egui::FontId::monospace(13.0), Color32::from_rgb(150, 200, 150));
        // Hint line.
        let hint = match app.tool {
            crate::tools::Tool::Select => "Click a ribbon tool (Line/Circle/…) or type a command, then click in the canvas.",
            _ => "Click in the canvas to place points. Esc cancels. Right-drag pans, wheel zooms.",
        };
        painter.text(rect.left_top() + vec2(8.0, 24.0), egui::Align2::LEFT_TOP, hint,
            egui::FontId::proportional(12.0), Color32::from_rgb(120, 130, 150));
    });
}

/// Draw a faint adaptive grid at "nice" world spacing.
fn draw_grid(painter: &egui::Painter, app: &AppState, rect: egui::Rect, to_screen: &impl Fn(f64, f64) -> egui::Pos2) {
    let (major, _minor) = nice_grid_spacing(app.view.pixel_world_size());
    let (x0, y0, x1, y1) = app.view.visible_bounds();
    let line = Stroke::new(1.0, Color32::from_rgb(34, 42, 54));
    let axis = Stroke::new(1.2, Color32::from_rgb(60, 72, 90));

    // Vertical grid lines.
    let mut gx = (x0 / major).floor() * major;
    while gx <= x1 {
        let a = to_screen(gx, y0);
        let b = to_screen(gx, y1);
        painter.line_segment([pos2(a.x, rect.top()), pos2(b.x, rect.bottom())],
            if gx.abs() < major * 0.5 { axis } else { line });
        gx += major;
    }
    // Horizontal grid lines.
    let mut gy = (y0 / major).floor() * major;
    while gy <= y1 {
        let a = to_screen(x0, gy);
        painter.line_segment([pos2(rect.left(), a.y), pos2(rect.right(), a.y)],
            if gy.abs() < major * 0.5 { axis } else { line });
        gy += major;
    }
}

/// Draw a map-style scale bar in the bottom-right corner. It picks a "nice"
/// round world distance (1-2-5 sequence) close to a target on-screen length and
/// labels it with the active drawing unit, so its length and text update as the
/// user zooms.
fn draw_scale_bar(painter: &egui::Painter, app: &AppState, rect: egui::Rect) {
    let pws = app.view.pixel_world_size();
    if !(pws.is_finite() && pws > 0.0) { return; }

    // Target ~120px; round the matching world distance to a nice 1-2-5 value.
    let target_px = 120.0_f64;
    let raw = target_px * pws; // world units spanning the target length
    let mag = raw.log10().floor();
    let base = 10f64.powf(mag);
    let nice = if raw / base < 1.5 { base }
               else if raw / base < 3.5 { 2.0 * base }
               else if raw / base < 7.5 { 5.0 * base }
               else { 10.0 * base };
    let bar_px = (nice / pws) as f32; // actual pixel length of the nice distance
    if !bar_px.is_finite() || bar_px <= 0.0 { return; }

    let unit = app.document.settings.units.short_name();
    let label = format!("{} {}", format_distance(nice), unit);
    let label = label.trim_end().to_string();

    // Geometry: bar with end caps, sitting above the bottom-right margin.
    let margin = 16.0;
    let y = rect.bottom() - margin;
    let x1 = rect.right() - margin;
    let x0 = x1 - bar_px;
    let cap = 5.0;
    let bar = Stroke::new(2.0, Color32::from_rgb(210, 220, 235));
    let shadow = Stroke::new(3.5, Color32::from_rgba_unmultiplied(0, 0, 0, 160));

    // Drop shadow first for legibility over any background.
    for s in [shadow, bar] {
        painter.line_segment([pos2(x0, y), pos2(x1, y)], s);
        painter.line_segment([pos2(x0, y - cap), pos2(x0, y + cap)], s);
        painter.line_segment([pos2(x1, y - cap), pos2(x1, y + cap)], s);
    }

    // Label centred above the bar.
    let tx = (x0 + x1) / 2.0;
    painter.text(pos2(tx + 1.0, y - cap - 2.0 + 1.0), egui::Align2::CENTER_BOTTOM, &label,
        egui::FontId::monospace(12.0), Color32::from_rgba_unmultiplied(0, 0, 0, 180));
    painter.text(pos2(tx, y - cap - 2.0), egui::Align2::CENTER_BOTTOM, &label,
        egui::FontId::monospace(12.0), Color32::from_rgb(220, 230, 245));
}

/// Format a scale-bar distance compactly: integers without a decimal point,
/// fractions trimmed of trailing zeros (e.g. 0.5, 2, 50, 1000).
fn format_distance(d: f64) -> String {
    if d >= 1.0 && (d.fract()).abs() < 1e-9 {
        format!("{}", d.round() as i64)
    } else {
        let s = format!("{:.6}", d);
        let s = s.trim_end_matches('0').trim_end_matches('.');
        s.to_string()
    }
}

/// Nice 1-2-5 grid spacing for the current zoom (mirrors render::grid_spacing).
fn nice_grid_spacing(pixel_world_size: f64) -> (f64, f64) {
    let raw = 80.0 * pixel_world_size;
    let mag = raw.log10().floor();
    let base = 10f64.powf(mag);
    let nice = if raw / base < 1.5 { base }
               else if raw / base < 3.5 { 2.0 * base }
               else if raw / base < 7.5 { 5.0 * base }
               else { 10.0 * base };
    (nice, nice / 5.0)
}

fn draw_dashed_line(
    painter: &egui::Painter,
    start: egui::Pos2,
    end: egui::Pos2,
    stroke: Stroke,
    dash_length: f32,
    gap_length: f32,
) {
    if !start.x.is_finite() || !start.y.is_finite() || !end.x.is_finite() || !end.y.is_finite() {
        return;
    }
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let len = (dx * dx + dy * dy).sqrt();
    if !len.is_finite() || len < 1e-6 {
        return;
    }
    let ux = dx / len;
    let uy = dy / len;
    
    let mut dist = 0.0;
    let mut count = 0;
    while dist < len && count < 1000 {
        let next_dist = (dist + dash_length).min(len);
        let p1 = pos2(start.x + ux * dist, start.y + uy * dist);
        let p2 = pos2(start.x + ux * next_dist, start.y + uy * next_dist);
        painter.line_segment([p1, p2], stroke);
        dist += dash_length + gap_length;
        count += 1;
    }
}

fn resolve_color(app: &AppState, e: &exact2d_document::Entity) -> (u8, u8, u8) {
    match &e.color {
        Color::Rgb(r, g, b) => (*r, *g, *b),
        _ => app.document.layers.get(e.layer).map(|l| l.color).unwrap_or((220, 220, 220)),
    }
}

fn constraints_panel(ctx: &Context, app: &mut AppState) {
    if app.constraints_enabled {
        SidePanel::right("constraints_panel").default_width(200.0).show(ctx, |ui| {
            ui.heading("Constraints Solver");
            ui.separator();
            
            // DOF & Status
            let dof = app.sketch.degrees_of_freedom();
            ui.label(format!("Degrees of Freedom: {}", dof));
            
            let is_over = app.sketch.is_over_constrained();
            if is_over {
                ui.colored_label(Color32::from_rgb(255, 100, 100), "⚠ OVER-CONSTRAINED!");
            } else if dof == 0 {
                ui.colored_label(Color32::from_rgb(0, 220, 100), "✓ Fully Constrained");
            } else {
                ui.colored_label(Color32::from_rgb(200, 200, 100), "ℹ Under-constrained");
            }
            
            ui.separator();
            ui.heading("Active Constraints");
            
            let mut to_remove = None;
            egui::ScrollArea::vertical().max_height(250.0).show(ui, |ui| {
                if app.sketch.constraints().is_empty() {
                    ui.label("No active constraints.");
                } else {
                    for (idx, c) in app.sketch.constraints().iter().enumerate() {
                        let label = match c {
                            exact2d_constraint::Constraint::Coincident(p1, p2) => format!("Coincident(P{}, P{})", p1, p2),
                            exact2d_constraint::Constraint::Fix(p, x, y) => format!("Fix(P{}, {:.2}, {:.2})", p, x, y),
                            exact2d_constraint::Constraint::Horizontal(p1, p2) => format!("Horizontal(P{}—P{})", p1, p2),
                            exact2d_constraint::Constraint::Vertical(p1, p2) => format!("Vertical(P{}—P{})", p1, p2),
                            exact2d_constraint::Constraint::Parallel(l1, l2) => format!("Parallel(L{}—L{})", format_line(l1), format_line(l2)),
                            exact2d_constraint::Constraint::Perpendicular(l1, l2) => format!("Perp(L{}—L{})", format_line(l1), format_line(l2)),
                            exact2d_constraint::Constraint::Collinear(p1, p2, p3) => format!("Collinear(P{}, P{}, P{})", p1, p2, p3),
                            exact2d_constraint::Constraint::EqualLength(l1, l2) => format!("Equal(L{}—L{})", format_line(l1), format_line(l2)),
                            exact2d_constraint::Constraint::Symmetric(p1, p2, axis) => format!("Symmetric(P{}, P{} across L{})", p1, p2, format_line(axis)),
                            exact2d_constraint::Constraint::Distance(p1, p2, d) => format!("Distance(P{}—P{}, {:.2})", p1, p2, d),
                            exact2d_constraint::Constraint::DistanceX(p1, p2, d) => format!("DistX(P{}—P{}, {:.2})", p1, p2, d),
                            exact2d_constraint::Constraint::DistanceY(p1, p2, d) => format!("DistY(P{}—P{}, {:.2})", p1, p2, d),
                            exact2d_constraint::Constraint::Angle(l1, l2, theta) => format!("Angle(L{}—L{}, {:.1}°)", format_line(l1), format_line(l2), theta.to_degrees()),
                            exact2d_constraint::Constraint::Midpoint(m, a, b) => format!("Midpoint(P{}, segment P{}—P{})", m, a, b),
                            exact2d_constraint::Constraint::TangentLineCircle(line, center, start) => format!("Tangent(L{}, Circle C{}—S{})", format_line(line), center, start),
                            exact2d_constraint::Constraint::TangentCircleCircle(c1, s1, c2, s2, ext) => format!("Tangent(C{}—S{}, C{}—S{}, {})", c1, s1, c2, s2, if *ext { "Ext" } else { "Int" }),
                        };
                        ui.horizontal(|ui| {
                            ui.label(&label);
                            if ui.small_button("x").clicked() {
                                to_remove = Some(idx);
                            }
                        });
                    }
                }
            });
            
            if let Some(idx) = to_remove {
                app.remove_constraint(idx);
            }
            
            ui.separator();
            ui.heading("Add Constraint");

            let mut dist_str = ui.data_mut(|d| d.get_temp_mut_or_default::<String>(egui::Id::new("con_dist_val")).clone());
            let mut angle_str = ui.data_mut(|d| d.get_temp_mut_or_default::<String>(egui::Id::new("con_angle_val")).clone());

            ui.horizontal(|ui| {
                ui.label("Distance (opt):");
                ui.text_edit_singleline(&mut dist_str);
            });
            ui.horizontal(|ui| {
                ui.label("Angle deg (opt):");
                ui.text_edit_singleline(&mut angle_str);
            });

            ui.data_mut(|d| {
                *d.get_temp_mut_or_default::<String>(egui::Id::new("con_dist_val")) = dist_str.clone();
                *d.get_temp_mut_or_default::<String>(egui::Id::new("con_angle_val")) = angle_str.clone();
            });

            ui.horizontal_wrapped(|ui| {
                if ui.button("Horizontal").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Horizontal));
                }
                if ui.button("Vertical").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Vertical));
                }
                if ui.button("Fix").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Fix));
                }
                if ui.button("Parallel").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Parallel));
                }
                if ui.button("Perpendicular").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Perpendicular));
                }
                if ui.button("Tangent").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Tangent));
                }
                if ui.button("Concentric").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Concentric));
                }
                if ui.button("Coincident").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Coincident));
                }
                if ui.button("Equal").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Equal));
                }
                if ui.button("Symmetric").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Symmetric));
                }
                if ui.button("Midpoint").clicked() {
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Midpoint));
                }
                if ui.button("Distance").clicked() {
                    let val = dist_str.trim().parse::<f64>().ok();
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Distance(val)));
                }
                if ui.button("Angle").clicked() {
                    let deg = angle_str.trim().parse::<f64>().ok();
                    let val = deg.map(|d| d.to_radians());
                    app.execute(Command::AddConstraint(crate::command::ConstraintType::Angle(val)));
                }
            });
        });
    }
}

fn format_line((p1, p2): &(exact2d_constraint::PointId, exact2d_constraint::PointId)) -> String {
    format!("P{}—P{}", p1, p2)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum DimType {
    Aligned,
    Horizontal,
    Vertical,
}

fn find_associated_arc(app: &AppState, p1: exact2d_constraint::PointId, p2: exact2d_constraint::PointId) -> bool {
    for e in app.document.iter() {
        if let EntityKind::Curve(Curve::Arc(_)) = &e.kind {
            if let Some(pts) = app.entity_points.get(&e.id) {
                if pts.len() >= 2 && pts[0] == p1 && pts[1] == p2 {
                    return true;
                }
            }
        }
    }
    false
}

fn draw_arrowhead_points(painter: &egui::Painter, tip: egui::Pos2, dir: egui::Vec2, color: Color32) {
    if !tip.x.is_finite() || !tip.y.is_finite() || !dir.x.is_finite() || !dir.y.is_finite() {
        return;
    }
    let normal = egui::vec2(-dir.y, dir.x);
    let side1 = tip + dir * 6.0 + normal * 2.0;
    let side2 = tip + dir * 6.0 - normal * 2.0;
    painter.add(egui::Shape::convex_polygon(vec![tip, side1, side2], color, egui::Stroke::NONE));
}

fn draw_entity(painter: &egui::Painter, app: &AppState, e: &exact2d_document::Entity, origin: egui::Pos2, stroke: Stroke) {
    let to_screen = |wx: f64, wy: f64| {
        let (sx, sy) = app.view.world_to_screen(wx, wy);
        pos2(origin.x + sx as f32, origin.y + sy as f32)
    };

    if e.id == app.origin_id {
        let origin_screen = to_screen(0.0, 0.0);
        let stroke_x = Stroke::new(1.5, Color32::from_rgb(255, 60, 60)); // Red for X
        let stroke_y = Stroke::new(1.5, Color32::from_rgb(60, 220, 60)); // Green for Y

        // X axis line:
        painter.line_segment([origin_screen, pos2(origin_screen.x + 18.0, origin_screen.y)], stroke_x);
        // X arrowhead (facing right):
        painter.line_segment([pos2(origin_screen.x + 18.0, origin_screen.y), pos2(origin_screen.x + 14.0, origin_screen.y - 3.0)], stroke_x);
        painter.line_segment([pos2(origin_screen.x + 18.0, origin_screen.y), pos2(origin_screen.x + 14.0, origin_screen.y + 3.0)], stroke_x);
        // X Label
        painter.text(pos2(origin_screen.x + 24.0, origin_screen.y), egui::Align2::CENTER_CENTER, "X", egui::FontId::proportional(10.0), stroke_x.color);

        // Y axis line:
        painter.line_segment([origin_screen, pos2(origin_screen.x, origin_screen.y - 18.0)], stroke_y);
        // Y arrowhead (facing up):
        painter.line_segment([pos2(origin_screen.x, origin_screen.y - 18.0), pos2(origin_screen.x - 3.0, origin_screen.y - 14.0)], stroke_y);
        painter.line_segment([pos2(origin_screen.x, origin_screen.y - 18.0), pos2(origin_screen.x + 3.0, origin_screen.y - 14.0)], stroke_y);
        // Y Label
        painter.text(pos2(origin_screen.x, origin_screen.y - 24.0), egui::Align2::CENTER_CENTER, "Y", egui::FontId::proportional(10.0), stroke_y.color);

        // Origin center circle
        painter.circle_filled(origin_screen, 3.0, Color32::from_rgb(180, 195, 220));
        painter.circle_stroke(origin_screen, 5.0, Stroke::new(1.0, Color32::from_rgb(80, 90, 110)));
        return;
    }

    match &e.kind {
        EntityKind::Curve(c) => draw_curve(painter, c, &to_screen, stroke),
        EntityKind::Point(p) => {
            let (x, y) = p.to_f64();
            painter.circle_filled(to_screen(x, y), 2.0, stroke.color);
        }
        EntityKind::Text { anchor, content, height, .. } => {
            let (x, y) = anchor.to_f64();
            painter.text(to_screen(x, y), egui::Align2::LEFT_BOTTOM, content,
                egui::FontId::proportional(*height as f32 * app.view.zoom as f32), stroke.color);
        }
        _ => {}
    }
}

fn constraint_references_any_point(
    c: &exact2d_constraint::Constraint,
    points: &std::collections::HashSet<exact2d_constraint::PointId>,
) -> bool {
    let has = |p| points.contains(&p);
    let has_line = |(p1, p2)| has(p1) || has(p2);
    match *c {
        exact2d_constraint::Constraint::Coincident(p1, p2) => has(p1) || has(p2),
        exact2d_constraint::Constraint::Fix(p, _, _) => has(p),
        exact2d_constraint::Constraint::Horizontal(p1, p2) => has(p1) || has(p2),
        exact2d_constraint::Constraint::Vertical(p1, p2) => has(p1) || has(p2),
        exact2d_constraint::Constraint::Parallel(l1, l2) => has_line(l1) || has_line(l2),
        exact2d_constraint::Constraint::Perpendicular(l1, l2) => has_line(l1) || has_line(l2),
        exact2d_constraint::Constraint::Collinear(p1, p2, p3) => has(p1) || has(p2) || has(p3),
        exact2d_constraint::Constraint::EqualLength(l1, l2) => has_line(l1) || has_line(l2),
        exact2d_constraint::Constraint::Symmetric(p1, p2, axis) => has(p1) || has(p2) || has_line(axis),
        exact2d_constraint::Constraint::Distance(p1, p2, _) => has(p1) || has(p2),
        exact2d_constraint::Constraint::DistanceX(p1, p2, _) => has(p1) || has(p2),
        exact2d_constraint::Constraint::DistanceY(p1, p2, _) => has(p1) || has(p2),
        exact2d_constraint::Constraint::Angle(l1, l2, _) => has_line(l1) || has_line(l2),
        exact2d_constraint::Constraint::Midpoint(m, a, b) => has(m) || has(a) || has(b),
        exact2d_constraint::Constraint::TangentLineCircle(line, center, start) => has_line(line) || has(center) || has(start),
        exact2d_constraint::Constraint::TangentCircleCircle(c1, s1, c2, s2, _) => has(c1) || has(s1) || has(c2) || has(s2),
    }
}

#[cfg(test)]
mod tess_tests {
    use super::*;
    use super::tessellate::{flatten_curve, point_seg_dist};
    use crate::view_transform::ViewTransform;
    use exact2d_geometry::{CircularArc, CurveSegment, Point2d};

    fn circle(r: f64) -> Curve {
        Curve::Arc(CircularArc::new(Point2d::from_i64(0, 0),
            exact2d_algebra::Rational::from_f64_approx(r), 0.0, std::f64::consts::TAU))
    }

    fn screen_polyline(view: &ViewTransform, c: &Curve) -> Vec<egui::Pos2> {
        let to_screen = |wx: f64, wy: f64| {
            let (sx, sy) = view.world_to_screen(wx, wy);
            egui::pos2(sx as f32, sy as f32)
        };
        flatten_curve(c, &to_screen)
    }

    /// The flattened polyline must hug the true curve to ~1px everywhere — i.e.
    /// no visible faceting — even when the circle is far larger than the screen.
    #[test]
    fn circle_stays_smooth_when_zoomed_in() {
        let mut view = ViewTransform::new(1000.0, 1000.0);
        view.zoom = 500.0; // a radius-2 circle spans ~2000px (bigger than screen)
        let c = circle(2.0);
        let poly = screen_polyline(&view, &c);

        let to_screen = |wx: f64, wy: f64| {
            let (sx, sy) = view.world_to_screen(wx, wy);
            egui::pos2(sx as f32, sy as f32)
        };
        // Check many true points lie within ~1px of some polyline segment.
        let mut worst = 0.0f32;
        for k in 0..2000 {
            let t = std::f64::consts::TAU * k as f64 / 2000.0;
            let (x, y) = c.evaluate_f64(t);
            let p = to_screen(x, y);
            let mut best = f32::INFINITY;
            for w in poly.windows(2) {
                best = best.min(point_seg_dist(p, w[0], w[1]));
            }
            worst = worst.max(best);
        }
        assert!(worst < 1.0, "max chord deviation {:.3}px exceeds 1px (faceting)", worst);
    }

    /// Segment count adapts to zoom: more detail when larger on screen, far less
    /// when small — so it is cheap zoomed out and crisp zoomed in.
    #[test]
    fn segment_count_tracks_zoom() {
        let c = circle(1.0);
        let mut small = ViewTransform::new(800.0, 600.0);
        small.zoom = 2.0; // ~2px radius
        let mut big = ViewTransform::new(800.0, 600.0);
        big.zoom = 2000.0; // ~2000px radius
        let n_small = screen_polyline(&small, &c).len();
        let n_big = screen_polyline(&big, &c).len();
        assert!(n_big > n_small * 4, "expected far more segments when zoomed in: {} vs {}", n_big, n_small);
        assert!(n_small < 40, "tiny circle should be cheap, got {} points", n_small);
    }
}
