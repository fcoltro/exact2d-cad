//! Window chrome: menu bar, icon ribbon, status/command bar, and the layer panel.
//! Split out of `view.rs` so the file root keeps `draw_ui` + the drawing canvas.

use egui::{Context, TopBottomPanel, SidePanel, Color32};
use exact2d_document::{Units, Layer};
use crate::state::AppState;
use crate::tools::Tool;
use crate::command::Command;
use super::UiState;
use rfd::FileDialog;

pub(super) fn menu_bar(ctx: &Context, app: &mut AppState) {
    // Update window title every frame to reflect the current file name.
    ctx.send_viewport_cmd(egui::ViewportCommand::Title(app.window_title()));

    // Keyboard shortcuts for file operations (handled outside the menu so they
    // work even when the menu is closed).
    let (ctrl, shift) = ctx.input(|i| (i.modifiers.ctrl, i.modifiers.shift));
    let s_key = ctx.input(|i| i.key_pressed(egui::Key::S));
    let n_key = ctx.input(|i| i.key_pressed(egui::Key::N));
    let o_key = ctx.input(|i| i.key_pressed(egui::Key::O));

    if ctrl && n_key { app.new_document(); }
    if ctrl && o_key { file_open(app); }
    // Ctrl+Shift+S always prompts; plain Ctrl+S saves to the current path and only
    // prompts when there isn't one yet. save_file() must not run for the Save As
    // shortcut, so it sits behind the short-circuiting && below.
    let save_as_key = ctrl && shift && s_key;
    let save_key    = ctrl && !shift && s_key;
    if save_as_key || (save_key && !app.save_file()) {
        file_save_as(app);
    }

    TopBottomPanel::top("menu_bar").show(ctx, |ui| {
        egui::menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| {
                if ui.add(egui::Button::new("New").shortcut_text("Ctrl+N")).clicked() {
                    app.new_document();
                    ui.close_menu();
                }
                if ui.add(egui::Button::new("Open…").shortcut_text("Ctrl+O")).clicked() {
                    file_open(app);
                    ui.close_menu();
                }
                if ui.add(egui::Button::new("Save").shortcut_text("Ctrl+S")).clicked() {
                    if !app.save_file() { file_save_as(app); }
                    ui.close_menu();
                }
                if ui.add(egui::Button::new("Save As…").shortcut_text("Ctrl+Shift+S")).clicked() {
                    file_save_as(app);
                    ui.close_menu();
                }
                ui.separator();
                if ui.button("Export DXF…").clicked() {
                    if let Some(path) = FileDialog::new()
                        .add_filter("AutoCAD DXF", &["dxf"])
                        .save_file()
                    {
                        let content = exact2d_io::export_dxf(&app.document);
                        if let Err(e) = std::fs::write(&path, content) {
                            app.command_log.push(format!("DXF export failed: {e}"));
                        }
                    }
                    ui.close_menu();
                }
                if ui.button("Export SVG…").clicked() {
                    if let Some(path) = FileDialog::new()
                        .add_filter("SVG image", &["svg"])
                        .save_file()
                    {
                        let content = exact2d_io::export_svg(&app.document);
                        if let Err(e) = std::fs::write(&path, content) {
                            app.command_log.push(format!("SVG export failed: {e}"));
                        }
                    }
                    ui.close_menu();
                }
            });
            ui.menu_button("Edit", |ui| {
                if ui.add_enabled(app.history.can_undo(), egui::Button::new("Undo")).clicked() { app.undo(); }
                if ui.add_enabled(app.history.can_redo(), egui::Button::new("Redo")).clicked() { app.redo(); }
                ui.separator();
                if ui.button("Erase").clicked() { app.erase_selection(); }
                if ui.button("Select All").clicked() { app.execute(Command::SelectAll); }
            });
            ui.menu_button("View", |ui| {
                if ui.button("Zoom Extents").clicked() { app.zoom_extents(); }
                ui.checkbox(&mut app.grid_on, "Grid");
                ui.checkbox(&mut app.snap_on, "Snap");
            });
            ui.menu_button("Draw", |ui| {
                tool_menu_item(ui, app, "Select", Tool::Select);
                ui.separator();
                tool_menu_item(ui, app, "Line", Tool::Line { last: None });
                tool_menu_item(ui, app, "Circle", Tool::Circle { center: None });
                tool_menu_item(ui, app, "Arc", Tool::Arc3 { pts: vec![] });
                tool_menu_item(ui, app, "Rectangle", Tool::Rectangle { first: None });
                tool_menu_item(ui, app, "Polygon", Tool::Polygon { center: None, sides: 4 });
                tool_menu_item(ui, app, "Spline", Tool::Spline { pts: vec![] });
                tool_menu_item(ui, app, "Polyline", Tool::Polyline { pts: vec![] });
                tool_menu_item(ui, app, "Smart Dimension", Tool::Dimension { stage: 0, p1: None, p2: None });
            });
            ui.menu_button("Modify", |ui| {
                tool_menu_item(ui, app, "Move", Tool::Move { base: None, ids: vec![] });
                tool_menu_item(ui, app, "Copy", Tool::Copy { base: None, ids: vec![] });
            });
            ui.menu_button("Units", |ui| {
                ui.label("Drawing units (sets the zoom range)");
                ui.separator();
                units_menu_item(ui, app, "Millimeters (mm)", Units::Millimeters);
                units_menu_item(ui, app, "Centimeters (cm)", Units::Centimeters);
                units_menu_item(ui, app, "Meters (m)", Units::Meters);
                units_menu_item(ui, app, "Kilometers (km)", Units::Kilometers);
                ui.separator();
                units_menu_item(ui, app, "Inches (in)", Units::Inches);
                units_menu_item(ui, app, "Feet (ft)", Units::Feet);
                units_menu_item(ui, app, "Unitless", Units::Unitless);
            });
            ui.menu_button("Help", |ui| { let _ = ui.button("About Exact2D CAD"); });
        });
    });
}

fn tool_menu_item(ui: &mut egui::Ui, app: &mut AppState, label: &str, tool: Tool) {
    if ui.button(label).clicked() {
        app.execute(Command::Activate(tool));
        ui.close_menu();
    }
}

fn units_menu_item(ui: &mut egui::Ui, app: &mut AppState, label: &str, units: Units) {
    let selected = app.document.settings.units == units;
    if ui.selectable_label(selected, label).clicked() {
        app.document.settings.units = units;
        app.sync_zoom_limits(); // re-bound the zoom immediately
        ui.close_menu();
    }
}

// ── Ribbon (Home tab: icon tool palette with tooltips) ─────────────────────────

pub(super) fn ribbon(ctx: &Context, app: &mut AppState) {
    use crate::icons::Icon;
    TopBottomPanel::top("ribbon").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 3.0;
            tool_icon(ui, app, Icon::Select, "Select  (SE)", Tool::Select);
            ui.separator();

            // Draw group.
            tool_icon(ui, app, Icon::Line, "Line  (L)", Tool::Line { last: None });
            tool_icon(ui, app, Icon::Polyline, "Polyline  (PL)", Tool::Polyline { pts: vec![] });
            tool_icon(ui, app, Icon::Circle, "Circle  (C)", Tool::Circle { center: None });
            tool_icon(ui, app, Icon::Arc, "Arc — 3 points  (A)", Tool::Arc3 { pts: vec![] });
            tool_icon(ui, app, Icon::Rectangle, "Rectangle  (REC)", Tool::Rectangle { first: None });
            tool_icon(ui, app, Icon::Polygon, "Polygon  (POL)", Tool::Polygon { center: None, sides: 4 });
            tool_icon(ui, app, Icon::Spline, "Spline  (SPL)", Tool::Spline { pts: vec![] });
            tool_icon(ui, app, Icon::Dimension, "Smart Dimension  (DIM)", Tool::Dimension { stage: 0, p1: None, p2: None });
            ui.separator();

            // Modify group.
            tool_icon(ui, app, Icon::Move, "Move selection  (M)", Tool::Move { base: None, ids: vec![] });
            tool_icon(ui, app, Icon::Copy, "Copy selection  (CO)", Tool::Copy { base: None, ids: vec![] });
            tool_icon(ui, app, Icon::Rotate, "Rotate selection  (RO)", Tool::Rotate { base: None, ids: vec![] });
            tool_icon(ui, app, Icon::Scale, "Scale selection  (SC)", Tool::Scale { base: None, reference: None, ids: vec![] });
            tool_icon(ui, app, Icon::Mirror, "Mirror selection  (MI)", Tool::Mirror { first: None, ids: vec![] });
            tool_icon(ui, app, Icon::Offset, "Offset  (O) — type a distance, click curve, click side", Tool::Offset { dist: 1.0, source: None });
            tool_icon(ui, app, Icon::Trim, "Trim  (TR) — click the piece to cut", Tool::Trim);
            tool_icon(ui, app, Icon::Extend, "Extend  (EX) — click the end to lengthen", Tool::Extend);
            tool_icon(ui, app, Icon::Fillet, "Fillet  (F) — type radius, pick 2 lines", Tool::Fillet { radius: 1.0, first: None });
            tool_icon(ui, app, Icon::Chamfer, "Chamfer  (CHA) — type distance, pick 2 lines", Tool::Chamfer { dist: 1.0, first: None });
            tool_icon(ui, app, Icon::Stretch, "Stretch  (S) — window, then base→destination", Tool::Stretch { c1: None, c2: None, base: None, ids: vec![] });
            if crate::icons::icon_button(ui, Icon::Erase, "Erase selection  (E / Del)", false).clicked() {
                app.erase_selection();
            }
            ui.separator();

            let con_enabled = app.constraints_enabled;
            if ui.selectable_label(con_enabled, "⛓ Parametric").clicked() {
                app.execute(Command::ToggleConstraints);
            }
        });
    });
}

/// One icon tool button: active-highlighted when it is the current tool.
fn tool_icon(ui: &mut egui::Ui, app: &mut AppState, icon: crate::icons::Icon, tip: &str, tool: Tool) {
    let active = app.tool.name() == tool.name();
    if crate::icons::icon_button(ui, icon, tip, active).clicked() {
        app.execute(Command::Activate(tool));
    }
}

// ── Command line + status bar (bottom) ────────────────────────────────────────

pub(super) fn status_and_command(ctx: &Context, app: &mut AppState, ui_state: &mut UiState) {
    TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.label(format!("⌖ {}", app.coord_readout()));
            ui.separator();
            ui.label(format!("Layer: {}", app.current_layer_name()));
            ui.separator();
            ui.toggle_value(&mut app.snap_on, "OSNAP");
            ui.menu_button("▾", |ui| {
                ui.label("Object Snapping");
                ui.separator();
                let kinds = [
                    (exact2d_cad::SnapKind::Endpoint, "Endpoint"),
                    (exact2d_cad::SnapKind::Midpoint, "Midpoint"),
                    (exact2d_cad::SnapKind::Center, "Center"),
                    (exact2d_cad::SnapKind::Intersection, "Intersection"),
                    (exact2d_cad::SnapKind::Perpendicular, "Perpendicular"),
                    (exact2d_cad::SnapKind::Tangent, "Tangent"),
                    (exact2d_cad::SnapKind::Nearest, "Nearest"),
                    (exact2d_cad::SnapKind::Node, "Node"),
                    (exact2d_cad::SnapKind::Insertion, "Insertion"),
                ];
                for (kind, label) in kinds {
                    let mut enabled = app.snap.enabled.contains(&kind);
                    if ui.checkbox(&mut enabled, label).changed() {
                        if enabled {
                            if !app.snap.enabled.contains(&kind) {
                                app.snap.enabled.push(kind);
                            }
                        } else {
                            app.snap.enabled.retain(|&k| k != kind);
                        }
                    }
                }
            });
            ui.toggle_value(&mut app.grid_on, "GRID");
            // Ortho and Polar are mutually exclusive (as in AutoCAD).
            if ui.toggle_value(&mut app.ortho_on, "ORTHO").changed() && app.ortho_on {
                app.polar_on = false;
            }
            if ui.toggle_value(&mut app.polar_on, "POLAR").changed() && app.polar_on {
                app.ortho_on = false;
            }
            ui.toggle_value(&mut app.dyn_on, "DYN");
            let mut con_enabled = app.constraints_enabled;
            if ui.toggle_value(&mut con_enabled, "CONSTRAINTS").changed() {
                app.execute(Command::ToggleConstraints);
            }
            ui.separator();
            ui.label(format!("Units: {}", app.units_label()));
            ui.separator();
            ui.label(format!("Tool: {}", app.tool.name()));
        });
    });
    TopBottomPanel::bottom("command_line").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.label("Command:");
            let resp = ui.add(
                egui::TextEdit::singleline(&mut ui_state.command_input)
                    .id(egui::Id::new("command_line_input"))
            );
            let is_enter = ui.input(|i| i.key_pressed(egui::Key::Enter));
            if (resp.lost_focus() || resp.has_focus()) && is_enter {
                let text = std::mem::take(&mut ui_state.command_input);
                let trimmed = text.trim();
                app.run_command(trimmed);
                ctx.memory_mut(|mem| mem.surrender_focus(egui::Id::new("command_line_input")));
            }
        });
    });
}

// ── Layer manager panel (left dock) ───────────────────────────────────────────

/// A snapshot of one layer row, collected up-front so the list can be iterated
/// while the panel mutably borrows the document for edits.
struct LayerRow { idx: usize, name: String, rgb: (u8, u8, u8), on: bool }

pub(super) fn layer_panel(ctx: &Context, app: &mut AppState) {
    SidePanel::left("layers").default_width(180.0).show(ctx, |ui| {
        ui.heading("Layers");
        let current = app.document.layers.current;
        let rows: Vec<LayerRow> = app.document.layers.layers.iter().enumerate()
            .map(|(i, l)| LayerRow { idx: i, name: l.name.clone(), rgb: l.color, on: l.on })
            .collect();
        for LayerRow { idx: i, name, rgb, on } in rows {
            ui.horizontal(|ui| {
                let (r, g, b) = rgb;
                let _swatch = ui.colored_label(Color32::from_rgb(r, g, b), "■");
                if ui.selectable_label(i == current, &name).clicked() {
                    app.document.layers.current = i;
                }
                let mut on_flag = on;
                if ui.checkbox(&mut on_flag, "").changed() {
                    if let Some(l) = app.document.layers.get_mut(i) { l.on = on_flag; }
                }
            });
        }
        if ui.button("+ New Layer").clicked() {
            let n = app.document.layers.layers.len();
            app.document.layers.add(Layer::new(format!("Layer{}", n)));
        }

        ui.separator();
        ui.heading("Properties");
        ui.label(format!("Selected: {}", app.selection.len()));
        ui.label(format!("Entities: {}", app.document.len()));
    });
}

// ── File dialog helpers ───────────────────────────────────────────────────────

fn file_open(app: &mut AppState) {
    if let Some(path) = FileDialog::new()
        .add_filter("Exact2D CAD drawing", &["e2d"])
        .pick_file()
    {
        app.open_file(path);
    }
}

fn file_save_as(app: &mut AppState) {
    let suggested = app.current_file_path.as_ref()
        .and_then(|p| p.file_name())
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "untitled.e2d".to_string());
    if let Some(path) = FileDialog::new()
        .add_filter("Exact2D CAD drawing", &["e2d"])
        .set_file_name(&suggested)
        .save_file()
    {
        app.save_file_to(path);
    }
}

// ── Drawing canvas (central) ──────────────────────────────────────────────────

