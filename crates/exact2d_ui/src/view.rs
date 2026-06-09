//! egui view layer (spec §6.1 window layout, §6.2 egui).
//!
//! Renders the full application chrome — menu bar, ribbon, layer/properties panels,
//! drawing canvas, command line, and status bar — by reading and driving `AppState`.
//! This compiles against `egui` without a windowing backend; a host (`exact2d_app`,
//! eframe) supplies the `egui::Context` each frame.

use egui::{Context, CentralPanel, Sense, Stroke, Color32, pos2, vec2};
use exact2d_geometry::Point2d;
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
    /// Dynamic-input HUD (editable Length/Angle floating by the cursor while
    /// drawing a line). `dyn_active` tracks whether it was shown last frame, so it
    /// can auto-focus the Length field the moment it first appears.
    pub dyn_length: String,
    pub dyn_angle: String,
    pub dyn_active: bool,
    /// Typed value buffer for the contextual corner fillet/chamfer grip.
    pub corner_input: String,
}

/// Build the entire UI for one frame.
pub fn draw_ui(ctx: &Context, app: &mut AppState, ui_state: &mut UiState) {
    menu_bar(ctx, app);
    ribbon(ctx, app);
    status_and_command(ctx, app, ui_state);
    layer_panel(ctx, app);
    canvas(ctx, app, ui_state);
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


        // ── Marquee box selection (AutoCAD window/crossing) ──
        // Left-drag on empty space draws a box: left→right = WINDOW (only entities
        // fully inside, blue solid); right→left = CROSSING (anything touched, green
        // dashed). Drags that begin on a grip or while the corner grip is active are
        // left to those interactions.
        if matches!(app.tool, Tool::Select) {
            if response.drag_started_by(egui::PointerButton::Primary) && app.corner_action.is_none() {
                if let Some(p) = response.interact_pointer_pos() {
                    ctx.data_mut(|d| {
                        d.insert_temp(egui::Id::new("marquee_start"), p);
                        d.insert_temp(egui::Id::new("marquee_on"), true);
                    });
                }
            }
            if response.drag_stopped()
                && ctx.data(|d| d.get_temp::<bool>(egui::Id::new("marquee_on")).unwrap_or(false))
            {
                let start: Option<egui::Pos2> = ctx.data(|d| d.get_temp(egui::Id::new("marquee_start")));
                let end = response.interact_pointer_pos().or_else(|| response.hover_pos());
                if let (Some(s), Some(e)) = (start, end) {
                    if (e - s).length() > 3.0 {
                        let (x0, y0) = app.view.screen_to_world((s.x - origin.x) as f64, (s.y - origin.y) as f64);
                        let (x1, y1) = app.view.screen_to_world((e.x - origin.x) as f64, (e.y - origin.y) as f64);
                        let rect = exact2d_geometry::BoundingBox::from_corners(x0, y0, x1, y1);
                        let sel = if e.x < s.x {
                            exact2d_cad::select_crossing(&app.document, &rect)
                        } else {
                            exact2d_cad::select_window(&app.document, &rect)
                        };
                        app.selection = sel.into_iter().filter(|&id| id != app.origin_id).collect();
                    }
                }
                ctx.data_mut(|d| d.insert_temp(egui::Id::new("marquee_on"), false));
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
        // ── Contextual corner grip (Inventor-style): a small blue dot sits inside
        // the corner. Hover for the hint; click and move RIGHT to fillet / LEFT to
        // chamfer, sized live (or type a value); click again (or Enter) to apply.
        // The dot/tooltip/preview are painted later; here we just hit-test + drive. ──
        let corner_geom = if app.corner_action.is_none() && matches!(app.tool, Tool::Select) {
            app.detect_corner()
        } else {
            None
        };
        // Dot ~16px into the interior wedge from the vertex.
        let corner_dot = corner_geom.map(|g| {
            let scr = |wx: f64, wy: f64| {
                let (sx, sy) = app.view.world_to_screen(wx, wy);
                pos2(origin.x + sx as f32, origin.y + sy as f32)
            };
            let c = scr(g.corner.0, g.corner.1);
            let a = (scr(g.corner.0 + g.dir_a.0 * g.len_a, g.corner.1 + g.dir_a.1 * g.len_a) - c).normalized();
            let b = (scr(g.corner.0 + g.dir_b.0 * g.len_b, g.corner.1 + g.dir_b.1 * g.len_b) - c).normalized();
            let mut bis = a + b;
            if bis.length() < 1e-3 { bis = egui::vec2(-a.y, a.x); }
            (g, c + bis.normalized() * 30.0) // sit a little clear of the lines
        });
        let over_dot = corner_dot
            .and_then(|(_, dp)| response.hover_pos().map(|p| (p - dp).length() <= 9.0))
            .unwrap_or(false);

        let corner_busy = app.corner_action.is_some() || over_dot;
        if app.corner_action.is_some() {
            app.update_corner_drag(); // direction → fillet/chamfer, distance → size
            // Optional typed value (digits / '.') overrides the dragged size.
            let typed: String = ui.input(|i| i.events.iter().filter_map(|e| match e {
                egui::Event::Text(t) => Some(t.clone()),
                _ => None,
            }).collect());
            for ch in typed.chars() {
                if ch.is_ascii_digit() || ch == '.' { ui_state.corner_input.push(ch); }
            }
            if ui.input(|i| i.key_pressed(egui::Key::Backspace)) { ui_state.corner_input.pop(); }
            if let Ok(v) = ui_state.corner_input.parse::<f64>() {
                if v > 0.0 { app.set_corner_size(v); }
            }
            let enter = ui.input(|i| i.key_pressed(egui::Key::Enter));
            if response.clicked() || enter {
                app.apply_corner_action();
                ui_state.corner_input.clear();
            }
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                app.cancel_corner_action();
                ui_state.corner_input.clear();
            }
        } else if over_dot && response.clicked() {
            if let Some((g, _)) = corner_dot {
                app.begin_corner_action(g);
                ui_state.corner_input.clear();
            }
        }

        // ── Dynamic-input HUD: an editable Length/Angle box floats by the cursor
        // while drawing a line (guided drawing). Commits via the polar-coordinate
        // path (@len<angle), so the geometry math is the tested one. ──
        let line_ref = if let Tool::Line { last: Some(p0) } = &app.tool {
            Some(p0.to_f64())
        } else {
            None
        };
        if let (true, Some((rx, ry))) = (app.dyn_on, line_ref) {
            let (cx, cy) = app.cursor_world;
            let live_len = ((cx - rx).powi(2) + (cy - ry).powi(2)).sqrt();
            let mut live_ang = (cy - ry).atan2(cx - rx).to_degrees();
            if live_ang < 0.0 { live_ang += 360.0; }

            let len_id = egui::Id::new("dyn_len");
            let ang_id = egui::Id::new("dyn_ang");
            if !ctx.memory(|m| m.has_focus(len_id)) { ui_state.dyn_length = format!("{:.2}", live_len); }
            if !ctx.memory(|m| m.has_focus(ang_id)) { ui_state.dyn_angle = format!("{:.1}", live_ang); }

            let cur = app.view.world_to_screen(cx, cy);
            let hud_pos = pos2(origin.x + cur.0 as f32 + 18.0, origin.y + cur.1 as f32 - 38.0);
            let first_show = !ui_state.dyn_active;
            let mut commit = false;
            egui::Area::new(egui::Id::new("dyn_input_hud"))
                .fixed_pos(hud_pos)
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    corner_glass_frame().show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("L").size(12.0).color(Color32::from_gray(170)));
                            let lr = ui.add(egui::TextEdit::singleline(&mut ui_state.dyn_length)
                                .id(len_id).desired_width(58.0));
                            ui.label(egui::RichText::new("∠").size(12.0).color(Color32::from_gray(170)));
                            let ar = ui.add(egui::TextEdit::singleline(&mut ui_state.dyn_angle)
                                .id(ang_id).desired_width(48.0));
                            if first_show { lr.request_focus(); } // type immediately
                            if ui.input(|i| i.key_pressed(egui::Key::Enter)) && (lr.lost_focus() || ar.lost_focus()) {
                                commit = true;
                            }
                        });
                    });
                });
            ui_state.dyn_active = true;
            if commit {
                let cmd = format!("@{}<{}", ui_state.dyn_length.trim(), ui_state.dyn_angle.trim());
                app.run_command(&cmd);
                ui_state.dyn_active = false; // re-focus Length for the next segment
            }
        } else {
            ui_state.dyn_active = false;
        }

        // While interacting with the corner grip the click was consumed above — don't pick.
        let place_point = !corner_busy && if matches!(app.tool, Tool::Select) {
            response.clicked()
        } else {
            response.contains_pointer() && ui.input(|i| i.pointer.primary_pressed())
        };
        if place_point {
            if let Some(p) = response.interact_pointer_pos().or_else(|| response.hover_pos()) {
                app.canvas_click((p.x - origin.x) as f64, (p.y - origin.y) as f64);
            }
        }
        // Esc cancels the in-progress tool input and returns to SELECT tool.
        if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
            app.execute(Command::Cancel);
        }
        // Enter or Space commits the active drawing tool (like Polyline) — but not
        // while a text field (command line or the dynamic-input HUD) has focus.
        let in_text_field = {
            let f = ctx.memory(|mem| mem.focused());
            f == Some(egui::Id::new("command_line_input"))
                || f == Some(egui::Id::new("dyn_len"))
                || f == Some(egui::Id::new("dyn_ang"))
        };
        if ui.input(|i| i.key_pressed(egui::Key::Enter) || i.key_pressed(egui::Key::Space))
            && !in_text_field {
                app.run_command("");
            }
        // Entity under the cursor (Select mode) — drives the hover highlight and is
        // the target for a right-click context menu.
        let hovered_id = if matches!(app.tool, Tool::Select) {
            response.hover_pos().and_then(|p| {
                let (wx, wy) = app.view.screen_to_world((p.x - origin.x) as f64, (p.y - origin.y) as f64);
                exact2d_cad::pick_at(&app.document, wx, wy, app.view.pixel_world_size() * 6.0)
            }).filter(|&id| id != app.origin_id)
        } else {
            None
        };

        // Right click: in Select mode show a context-sensitive menu (modern direct
        // manipulation); during an active tool it confirms/cancels (AutoCAD-like).
        if matches!(app.tool, Tool::Select) {
            // Right-clicking an unselected entity targets it.
            if response.secondary_clicked() && app.selection.is_empty() {
                if let Some(h) = hovered_id { app.selection = vec![h]; }
            }
            response.context_menu(|ui| {
                if !app.selection.is_empty() {
                    if ui.button("🗑  Delete").clicked() { app.erase_selection(); ui.close_menu(); }
                    ui.separator();
                    let acts = [
                        ("Move", Command::Activate(Tool::Move { base: None, ids: vec![] })),
                        ("Copy", Command::Activate(Tool::Copy { base: None, ids: vec![] })),
                        ("Rotate", Command::Activate(Tool::Rotate { base: None, ids: vec![] })),
                        ("Scale", Command::Activate(Tool::Scale { base: None, reference: None, ids: vec![] })),
                        ("Mirror", Command::Activate(Tool::Mirror { first: None, ids: vec![] })),
                    ];
                    for (label, cmd) in acts {
                        if ui.button(label).clicked() { app.execute(cmd); ui.close_menu(); }
                    }
                    ui.separator();
                }
                if let Some(last) = app.last_command.clone() {
                    if ui.button(format!("🔁  Repeat: {last}")).clicked() { app.repeat_last_command(); ui.close_menu(); }
                }
                if ui.button("Select All").clicked() { app.execute(Command::SelectAll); ui.close_menu(); }
                if ui.button("Zoom Extents").clicked() { app.zoom_extents(); ui.close_menu(); }
                ui.separator();
                ui.checkbox(&mut app.grid_on, "Grid");
                ui.checkbox(&mut app.snap_on, "Object Snap");
            });
        } else if response.secondary_clicked() {
            app.run_command(""); // active tool: confirm/cancel
        }
        // Automatically focus command input if user starts typing when hovering the canvas and command input is not focused
        let focused_id = ctx.memory(|mem| mem.focused());
        let cmd_input_id = egui::Id::new("command_line_input");
        // Don't hijack typing when a dynamic-input HUD field already has focus.
        let hud_focused = focused_id == Some(egui::Id::new("dyn_len"))
            || focused_id == Some(egui::Id::new("dyn_ang"));
        let mut focus_cmd = false;
        let mut text_to_append = String::new();
        if response.hovered() && focused_id != Some(cmd_input_id) && !hud_focused {
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

        // On-selection floating mini-toolbar (Plasticity/Illustrator-style): quick
        // action icons hovering just above the selection. Deferred actions keep it
        // from borrowing `app` while the toolbar UI is built.
        if matches!(app.tool, Tool::Select) && !app.selection.is_empty() {
            let mut bb: Option<exact2d_geometry::BoundingBox> = None;
            for &id in &app.selection {
                if let Some(b) = app.document.get(id).and_then(|e| e.bounding_box()) {
                    bb = Some(match bb { Some(a) => a.union(&b), None => b });
                }
            }
            if let Some(bb) = bb {
                let (minx, _) = bb.min.to_f64();
                let (maxx, maxy) = bb.max.to_f64();
                let (sx, sy) = app.view.world_to_screen((minx + maxx) / 2.0, maxy);
                let bar_pos = pos2(origin.x + sx as f32, origin.y + sy as f32 - 44.0);
                let mut action: Option<Command> = None;
                let mut do_erase = false;
                egui::Area::new(egui::Id::new("selection_toolbar"))
                    .fixed_pos(bar_pos)
                    .order(egui::Order::Foreground)
                    .show(ctx, |ui| {
                        egui::Frame::popup(ui.style()).show(ui, |ui| {
                            ui.horizontal(|ui| {
                                use crate::icons::{Icon, icon_button};
                                if icon_button(ui, Icon::Move, "Move", false).clicked() { action = Some(Command::Activate(Tool::Move { base: None, ids: vec![] })); }
                                if icon_button(ui, Icon::Copy, "Copy", false).clicked() { action = Some(Command::Activate(Tool::Copy { base: None, ids: vec![] })); }
                                if icon_button(ui, Icon::Rotate, "Rotate", false).clicked() { action = Some(Command::Activate(Tool::Rotate { base: None, ids: vec![] })); }
                                if icon_button(ui, Icon::Scale, "Scale", false).clicked() { action = Some(Command::Activate(Tool::Scale { base: None, reference: None, ids: vec![] })); }
                                if icon_button(ui, Icon::Mirror, "Mirror", false).clicked() { action = Some(Command::Activate(Tool::Mirror { first: None, ids: vec![] })); }
                                if icon_button(ui, Icon::Erase, "Delete", false).clicked() { do_erase = true; }
                            });
                        });
                    });
                if let Some(cmd) = action { app.execute(cmd); }
                if do_erase { app.erase_selection(); }
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
            let hovered = !selected && Some(e.id) == hovered_id;
            let color = if selected {
                Color32::from_rgb(0, 200, 255)
            } else if hovered {
                Color32::from_rgb(120, 230, 255) // pre-selection hover glow
            } else {
                Color32::from_rgb(r, g, b)
            };
            let width = if selected { 2.5 } else if hovered { 2.0 } else { 1.5 };
            draw_entity(&painter, app, e, origin, Stroke::new(width, color));
        }

        // Contextual corner grip: the blue dot (idle) or the live fillet/chamfer
        // preview (while dragging).
        if let Some(ca) = app.corner_action {
            draw_corner_preview(&painter, app, &ca, &to_screen);
        } else if let Some((_, dp)) = corner_dot {
            let r = if over_dot { 7.0 } else { 5.0 };
            painter.circle_filled(dp, r, Color32::from_rgb(0, 150, 255));
            painter.circle_stroke(dp, r, Stroke::new(1.5, Color32::from_rgb(190, 225, 255)));
            if over_dot {
                let txt = "◂ Chamfer    Fillet ▸";
                let tp = pos2(dp.x + 12.0, dp.y - 22.0);
                let galley = painter.layout_no_wrap(txt.to_string(),
                    egui::FontId::proportional(12.0), Color32::WHITE);
                let bg = egui::Rect::from_min_size(tp, galley.size()).expand(5.0);
                painter.rect_filled(bg, 6.0, Color32::from_rgba_unmultiplied(26, 32, 42, 235));
                painter.rect_stroke(bg, 6.0, Stroke::new(1.0, Color32::from_rgb(0, 200, 255)));
                painter.galley(tp, galley, Color32::WHITE);
            }
        }

        // Marquee selection box overlay (blue solid window / green dashed crossing).
        if ctx.data(|d| d.get_temp::<bool>(egui::Id::new("marquee_on")).unwrap_or(false)) {
            if let (Some(start), Some(cur)) = (
                ctx.data(|d| d.get_temp::<egui::Pos2>(egui::Id::new("marquee_start"))),
                response.hover_pos().or_else(|| response.interact_pointer_pos()),
            ) {
                let crossing = cur.x < start.x;
                let rect = egui::Rect::from_two_pos(start, cur);
                let (fill, line) = if crossing {
                    (Color32::from_rgba_unmultiplied(0, 200, 90, 32), Color32::from_rgb(0, 220, 110))
                } else {
                    (Color32::from_rgba_unmultiplied(0, 150, 240, 32), Color32::from_rgb(0, 180, 255))
                };
                painter.rect_filled(rect, 0.0, fill);
                if crossing {
                    let c = [rect.left_top(), rect.right_top(), rect.right_bottom(), rect.left_bottom()];
                    let st = Stroke::new(1.0, line);
                    for i in 0..4 {
                        draw_dashed_line(&painter, c[i], c[(i + 1) % 4], st, 6.0, 4.0);
                    }
                } else {
                    painter.rect_stroke(rect, 0.0, Stroke::new(1.0, line));
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
                    // The editable Length/Angle HUD replaces the read-only readout
                    // for the line tool when dynamic input is on.
                    Tool::Line { last: Some(p0) } if !app.dyn_on => {
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

// ── Contextual corner micro-menu (glass-lite) + live preview ───────────────────

/// A "glass-lite" floating frame: rounded, translucent, soft shadow, accent edge.
/// (True backdrop blur isn't available in egui; this is the translucent-shadow
/// approximation of the glassmorphism look.)
fn corner_glass_frame() -> egui::Frame {
    egui::Frame {
        inner_margin: egui::Margin::symmetric(4.0, 3.0),
        rounding: egui::Rounding::same(8.0),
        fill: Color32::from_rgba_unmultiplied(26, 32, 42, 235),
        stroke: Stroke::new(1.0, Color32::from_rgb(0, 200, 255)),
        shadow: egui::epaint::Shadow {
            offset: egui::vec2(0.0, 3.0),
            blur: 14.0,
            spread: 0.0,
            color: Color32::from_black_alpha(130),
        },
        outer_margin: egui::Margin::ZERO,
    }
}

/// Draw the live fillet/chamfer preview (trimmed edges + arc/bevel) in the accent
/// colour, plus a sized handle and value label at the cursor.
fn draw_corner_preview(
    painter: &egui::Painter,
    app: &AppState,
    ca: &crate::state::CornerAction,
    to_screen: &impl Fn(f64, f64) -> egui::Pos2,
) {
    let accent = Color32::from_rgb(0, 220, 255);
    let stroke = Stroke::new(2.0, accent);
    let g = &ca.geom;
    let far_a = (g.corner.0 + g.dir_a.0 * g.len_a, g.corner.1 + g.dir_a.1 * g.len_a);
    let far_b = (g.corner.0 + g.dir_b.0 * g.len_b, g.corner.1 + g.dir_b.1 * g.len_b);
    let seg = |p: (f64, f64), q: (f64, f64)| [to_screen(p.0, p.1), to_screen(q.0, q.1)];

    match ca.kind {
        crate::state::CornerKind::Fillet => {
            if let Some((p1, p2, c)) = crate::state::fillet_arc(g.corner, g.dir_a, g.dir_b, ca.size) {
                painter.line_segment(seg(far_a, p1), stroke);
                painter.line_segment(seg(far_b, p2), stroke);
                let a1 = (p1.1 - c.1).atan2(p1.0 - c.0);
                let a2 = (p2.1 - c.1).atan2(p2.0 - c.0);
                let mut d = a2 - a1;
                while d > std::f64::consts::PI { d -= std::f64::consts::TAU; }
                while d < -std::f64::consts::PI { d += std::f64::consts::TAU; }
                let n = 28;
                let pts: Vec<_> = (0..=n).map(|i| {
                    let a = a1 + d * (i as f64 / n as f64);
                    to_screen(c.0 + ca.size * a.cos(), c.1 + ca.size * a.sin())
                }).collect();
                painter.add(egui::Shape::line(pts, stroke));
            }
        }
        crate::state::CornerKind::Chamfer => {
            let p1 = (g.corner.0 + g.dir_a.0 * ca.size, g.corner.1 + g.dir_a.1 * ca.size);
            let p2 = (g.corner.0 + g.dir_b.0 * ca.size, g.corner.1 + g.dir_b.1 * ca.size);
            painter.line_segment(seg(far_a, p1), stroke);
            painter.line_segment(seg(far_b, p2), stroke);
            painter.line_segment(seg(p1, p2), stroke);
        }
    }

    let cur = to_screen(app.cursor_world.0, app.cursor_world.1);
    painter.circle_filled(cur, 4.0, accent);
    let label = match ca.kind {
        crate::state::CornerKind::Fillet => format!("R {:.2}", ca.size),
        crate::state::CornerKind::Chamfer => format!("{:.2}", ca.size),
    };
    painter.text(pos2(cur.x + 9.0, cur.y - 9.0), egui::Align2::LEFT_BOTTOM, label,
        egui::FontId::monospace(12.0), accent);
}


#[cfg(test)]
mod tess_tests {
    use super::tessellate::{flatten_curve, point_seg_dist};
    use crate::view_transform::ViewTransform;
    use exact2d_geometry::{Curve, CircularArc, CurveSegment, Point2d};

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
