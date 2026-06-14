//! Window chrome: menu bar, icon ribbon, status/command bar, and the layer panel.
//! Split out of `view.rs` so the file root keeps `draw_ui` + the drawing canvas.

use egui::{Context, TopBottomPanel, SidePanel};
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

    // Edit shortcuts — only when no text field has focus, so typing in the
    // command line / palette / HUD never deletes geometry or undoes the model.
    let typing = ctx.memory(|m| m.focused().is_some());
    if !typing {
        let z = ctx.input(|i| i.key_pressed(egui::Key::Z));
        let y = ctx.input(|i| i.key_pressed(egui::Key::Y));
        if ctrl && ((z && shift) || y) { app.redo(); }
        else if ctrl && z { app.undo(); }
        if ctx.input(|i| i.key_pressed(egui::Key::Delete)) {
            app.erase_selection();
        }
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
                if ui.add_enabled(app.history.can_undo(),
                    egui::Button::new("Undo").shortcut_text("Ctrl+Z")).clicked() { app.undo(); }
                if ui.add_enabled(app.history.can_redo(),
                    egui::Button::new("Redo").shortcut_text("Ctrl+Y")).clicked() { app.redo(); }
                ui.separator();
                if ui.add(egui::Button::new("Erase").shortcut_text("Del")).clicked() { app.erase_selection(); }
                if ui.button("Select All").clicked() { app.execute(Command::SelectAll); }
                ui.separator();
                if ui.add(egui::Button::new("Command Palette…").shortcut_text("Ctrl+K")).clicked() {
                    // Toggled by the palette itself next frame via this marker.
                    ui.ctx().data_mut(|d| d.insert_temp(egui::Id::new("open_palette"), true));
                }
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
                tool_menu_item(ui, app, "Text", Tool::Text { anchor: None, height: 2.5 });
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
            group_label(ui, "DRAW");

            // Draw group.
            tool_icon(ui, app, Icon::Line, "Line  (L)", Tool::Line { last: None });
            tool_icon(ui, app, Icon::Polyline, "Polyline  (PL)", Tool::Polyline { pts: vec![] });
            tool_icon(ui, app, Icon::Circle, "Circle  (C)", Tool::Circle { center: None });
            tool_icon(ui, app, Icon::Arc, "Arc — 3 points  (A)", Tool::Arc3 { pts: vec![] });
            tool_icon(ui, app, Icon::Rectangle, "Rectangle  (REC)", Tool::Rectangle { first: None });
            tool_icon(ui, app, Icon::Polygon, "Polygon  (POL)", Tool::Polygon { center: None, sides: 4 });
            tool_icon(ui, app, Icon::Spline, "Spline  (SPL)", Tool::Spline { pts: vec![] });
            tool_icon(ui, app, Icon::Text, "Text  (T)", Tool::Text { anchor: None, height: 2.5 });
            ui.separator();
            group_label(ui, "MODIFY");

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

            // Undo/redo at the far right.
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_enabled_ui(app.history.can_redo(), |ui| {
                    if crate::icons::icon_button(ui, Icon::Redo, "Redo  (Ctrl+Y)", false).clicked() {
                        app.redo();
                    }
                });
                ui.add_enabled_ui(app.history.can_undo(), |ui| {
                    if crate::icons::icon_button(ui, Icon::Undo, "Undo  (Ctrl+Z)", false).clicked() {
                        app.undo();
                    }
                });
            });
        });
    });
}

/// Small dim group caption between ribbon sections.
fn group_label(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).size(9.5).color(crate::theme::TEXT_DIM));
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
            // Version at the bottom-left (tracks the crate version at compile time).
            ui.label(egui::RichText::new(concat!("Exact2D CAD v", env!("CARGO_PKG_VERSION")))
                .size(11.0).color(egui::Color32::from_gray(140)));
            ui.separator();
            // Monospace + minimum width so the readout doesn't jitter the bar.
            let coords = egui::RichText::new(format!("{:>22}", app.coord_readout()))
                .monospace().size(12.0);
            ui.add(egui::Label::new(coords));
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
            ui.separator();
            ui.label(format!("Units: {}", app.units_label()));
            ui.separator();
            ui.label(format!("Tool: {}", app.tool.name()));

            // Zoom cluster, right-aligned (Figma-style − / fit / +).
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                use crate::icons::{Icon, icon_button_sized};
                let (cx, cy) = app.view.screen_to_world(app.view.width / 2.0, app.view.height / 2.0);
                if icon_button_sized(ui, Icon::ZoomIn, "Zoom in", false, 22.0).clicked() {
                    app.view.zoom_at(cx, cy, 1.25);
                }
                if icon_button_sized(ui, Icon::ZoomFit, "Zoom extents — fit the whole drawing", false, 22.0).clicked() {
                    app.zoom_extents();
                }
                if icon_button_sized(ui, Icon::ZoomOut, "Zoom out", false, 22.0).clicked() {
                    app.view.zoom_at(cx, cy, 0.8);
                }
            });
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
struct LayerRow { idx: usize, name: String, rgb: [u8; 3], on: bool }

pub(super) fn layer_panel(ctx: &Context, app: &mut AppState) {
    SidePanel::left("layers").default_width(190.0).show(ctx, |ui| {
        ui.add_space(4.0);
        ui.heading("Layers");
        ui.add_space(2.0);
        let current = app.document.layers.current;
        let rows: Vec<LayerRow> = app.document.layers.layers.iter().enumerate()
            .map(|(i, l)| LayerRow {
                idx: i, name: l.name.clone(),
                rgb: [l.color.0, l.color.1, l.color.2], on: l.on,
            })
            .collect();
        for LayerRow { idx: i, name, rgb, on } in rows {
            ui.horizontal(|ui| {
                // Click the swatch to recolour the layer (live).
                let mut c = rgb;
                if ui.color_edit_button_srgb(&mut c).changed() {
                    if let Some(l) = app.document.layers.get_mut(i) {
                        l.color = (c[0], c[1], c[2]);
                    }
                }
                if ui.selectable_label(i == current, &name)
                    .on_hover_text("Set as the current drawing layer").clicked() {
                    app.document.layers.current = i;
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    use crate::icons::{Icon, icon_button_sized};
                    let icon = if on { Icon::Eye } else { Icon::EyeOff };
                    if icon_button_sized(ui, icon, "Show / hide this layer", false, 20.0).clicked() {
                        if let Some(l) = app.document.layers.get_mut(i) { l.on = !on; }
                    }
                });
            });
        }
        if ui.button("+ New Layer").clicked() {
            let n = app.document.layers.layers.len();
            app.document.layers.add(Layer::new(format!("Layer{}", n)));
        }

        ui.add_space(8.0);
        ui.separator();
        ui.heading("Properties");
        ui.add_space(2.0);
        selection_properties(ui, app);
    });
}

/// Illustrator-style selection inspector: shows what is selected and lets the
/// user re-layer or recolour it without any command.
fn selection_properties(ui: &mut egui::Ui, app: &mut AppState) {
    let sel: Vec<_> = app.selection.clone();
    if sel.is_empty() {
        ui.label(egui::RichText::new("Nothing selected").color(crate::theme::TEXT_DIM));
        ui.label(egui::RichText::new(format!("{} entities in drawing", app.document.len()))
            .size(11.0).color(crate::theme::TEXT_DIM));
        return;
    }

    // What is selected.
    if sel.len() == 1 {
        if let Some(info) = exact2d_cad::inquiry::list_entity(&app.document, sel[0]) {
            ui.label(egui::RichText::new(info).size(11.0).monospace());
        }
    } else {
        ui.label(format!("{} entities selected", sel.len()));
    }
    ui.add_space(4.0);

    // Layer assignment (applies to the whole selection).
    let layer_names: Vec<String> =
        app.document.layers.layers.iter().map(|l| l.name.clone()).collect();
    let first_layer = sel.first()
        .and_then(|&id| app.document.get(id)).map(|e| e.layer).unwrap_or(0);
    let mixed = sel.iter().any(|&id|
        app.document.get(id).map(|e| e.layer) != Some(first_layer));
    let mut chosen = first_layer;
    egui::ComboBox::from_label("Layer")
        .selected_text(if mixed { "(mixed)".to_string() }
                       else { layer_names.get(first_layer).cloned().unwrap_or_default() })
        .show_ui(ui, |ui| {
            for (i, name) in layer_names.iter().enumerate() {
                ui.selectable_value(&mut chosen, i, name);
            }
        });
    if chosen != first_layer {
        app.history.snapshot(&app.document);
        for &id in &sel {
            if let Some(e) = app.document.get_mut(id) { e.layer = chosen; }
        }
    }

    // Colour override: by-layer or a custom RGB for the selection.
    let first_color = sel.first()
        .and_then(|&id| app.document.get(id)).map(|e| e.color.clone());
    let mut by_layer = matches!(first_color, Some(exact2d_document::Color::ByLayer));
    if ui.checkbox(&mut by_layer, "Colour by layer").changed() {
        app.history.snapshot(&app.document);
        for &id in &sel {
            if let Some(e) = app.document.get_mut(id) {
                e.color = if by_layer { exact2d_document::Color::ByLayer }
                          else { exact2d_document::Color::Rgb(220, 220, 220) };
            }
        }
    }
    if !by_layer {
        let mut rgb = match first_color {
            Some(exact2d_document::Color::Rgb(r, g, b)) => [r, g, b],
            _ => [220, 220, 220],
        };
        ui.horizontal(|ui| {
            ui.label("Colour");
            if ui.color_edit_button_srgb(&mut rgb).changed() {
                for &id in &sel {
                    if let Some(e) = app.document.get_mut(id) {
                        e.color = exact2d_document::Color::Rgb(rgb[0], rgb[1], rgb[2]);
                    }
                }
            }
        });
    }
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

