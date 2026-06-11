//! Ctrl+K command palette — fuzzy quick-action search (Figma/VS Code pattern).
//! The discoverable, keyboard-first complement to the classic command line:
//! type a few letters, arrow keys to choose, Enter to run.

use egui::{Color32, Context, Key, RichText};
use crate::state::AppState;
use crate::theme;
use super::UiState;

/// What a palette entry does when chosen.
enum Action {
    /// Run through the normal command-line parser (the tested path).
    Cmd(&'static str),
    ToggleGrid,
    ToggleSnap,
    ToggleOrtho,
    TogglePolar,
    ToggleDyn,
}

struct Entry {
    name: &'static str,
    /// Shown right-aligned: the classic command alias / shortcut.
    hint: &'static str,
    /// Extra match words (lowercase).
    keywords: &'static str,
    action: Action,
}

const ENTRIES: &[Entry] = &[
    Entry { name: "Select",        hint: "SE",   keywords: "pick arrow",                 action: Action::Cmd("SELECT") },
    Entry { name: "Line",          hint: "L",    keywords: "segment draw",               action: Action::Cmd("LINE") },
    Entry { name: "Polyline",      hint: "PL",   keywords: "draw connected",             action: Action::Cmd("POLYLINE") },
    Entry { name: "Circle",        hint: "C",    keywords: "draw round",                 action: Action::Cmd("CIRCLE") },
    Entry { name: "Arc (3 points)",hint: "A",    keywords: "draw curve",                 action: Action::Cmd("ARC") },
    Entry { name: "Rectangle",     hint: "REC",  keywords: "draw box square",            action: Action::Cmd("RECTANGLE") },
    Entry { name: "Polygon",       hint: "POL",  keywords: "draw hexagon sides",         action: Action::Cmd("POLYGON") },
    Entry { name: "Spline",        hint: "SPL",  keywords: "draw bezier curve",          action: Action::Cmd("SPLINE") },
    Entry { name: "Text",          hint: "T",    keywords: "draw label annotate",        action: Action::Cmd("TEXT") },
    Entry { name: "Move",          hint: "M",    keywords: "modify translate",           action: Action::Cmd("MOVE") },
    Entry { name: "Copy",          hint: "CO",   keywords: "modify duplicate",           action: Action::Cmd("COPY") },
    Entry { name: "Rotate",        hint: "RO",   keywords: "modify turn angle",          action: Action::Cmd("ROTATE") },
    Entry { name: "Scale",         hint: "SC",   keywords: "modify resize",              action: Action::Cmd("SCALE") },
    Entry { name: "Mirror",        hint: "MI",   keywords: "modify reflect flip",        action: Action::Cmd("MIRROR") },
    Entry { name: "Offset",        hint: "O",    keywords: "modify parallel",            action: Action::Cmd("OFFSET") },
    Entry { name: "Trim",          hint: "TR",   keywords: "modify cut",                 action: Action::Cmd("TRIM") },
    Entry { name: "Extend",        hint: "EX",   keywords: "modify lengthen",            action: Action::Cmd("EXTEND") },
    Entry { name: "Fillet",        hint: "F",    keywords: "modify round corner radius", action: Action::Cmd("FILLET") },
    Entry { name: "Chamfer",       hint: "CHA",  keywords: "modify bevel corner",        action: Action::Cmd("CHAMFER") },
    Entry { name: "Stretch",       hint: "S",    keywords: "modify deform window",       action: Action::Cmd("STRETCH") },
    Entry { name: "Erase",         hint: "E",    keywords: "delete remove",              action: Action::Cmd("ERASE") },
    Entry { name: "Undo",          hint: "Ctrl+Z", keywords: "back revert",              action: Action::Cmd("UNDO") },
    Entry { name: "Redo",          hint: "Ctrl+Y", keywords: "forward again",            action: Action::Cmd("REDO") },
    Entry { name: "Select All",    hint: "ALL",  keywords: "everything",                 action: Action::Cmd("ALL") },
    Entry { name: "Zoom Extents",  hint: "Z E",  keywords: "fit view all frame",         action: Action::Cmd("ZOOM E") },
    Entry { name: "Toggle Grid",   hint: "F7",   keywords: "view background lines",      action: Action::ToggleGrid },
    Entry { name: "Toggle Object Snap", hint: "F9", keywords: "osnap endpoint midpoint", action: Action::ToggleSnap },
    Entry { name: "Toggle Ortho",  hint: "F8",   keywords: "horizontal vertical lock",   action: Action::ToggleOrtho },
    Entry { name: "Toggle Polar Tracking", hint: "", keywords: "angle 45 guide",         action: Action::TogglePolar },
    Entry { name: "Toggle Dynamic Input",  hint: "", keywords: "dyn hud length angle",   action: Action::ToggleDyn },
];

/// Lower is better; `None` = no match.
fn score(entry: &Entry, q: &str) -> Option<u8> {
    if q.is_empty() { return Some(3); }
    let name = entry.name.to_ascii_lowercase();
    if name.starts_with(q) { return Some(0); }
    if name.split_whitespace().any(|w| w.starts_with(q)) { return Some(1); }
    if name.contains(q) { return Some(2); }
    if entry.hint.to_ascii_lowercase() == q { return Some(0); }
    if entry.keywords.split_whitespace().any(|w| w.starts_with(q)) { return Some(2); }
    None
}

/// Returns true while the palette is open (the canvas suppresses its own
/// keyboard handling for that frame).
pub(super) fn command_palette(ctx: &Context, app: &mut AppState, ui_state: &mut UiState) -> bool {
    // Ctrl+K toggles; the Edit menu can also request it via a data marker.
    let menu_request = ctx.data(|d| d.get_temp::<bool>(egui::Id::new("open_palette")).unwrap_or(false));
    if menu_request {
        ctx.data_mut(|d| d.insert_temp(egui::Id::new("open_palette"), false));
    }
    if ctx.input_mut(|i| i.consume_key(egui::Modifiers::CTRL, Key::K)) || menu_request {
        ui_state.palette_open = !ui_state.palette_open || menu_request;
        ui_state.palette_query.clear();
        ui_state.palette_index = 0;
    }
    if !ui_state.palette_open { return false; }

    // Esc closes (consumed here, before the canvas cancel handler runs).
    if ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, Key::Escape)) {
        ui_state.palette_open = false;
        return false;
    }

    let q = ui_state.palette_query.trim().to_ascii_lowercase();
    let mut matches: Vec<(&Entry, u8)> =
        ENTRIES.iter().filter_map(|e| score(e, &q).map(|s| (e, s))).collect();
    matches.sort_by_key(|&(_, s)| s);
    matches.truncate(9);
    if ui_state.palette_index >= matches.len() {
        ui_state.palette_index = matches.len().saturating_sub(1);
    }

    let nav_down = ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, Key::ArrowDown));
    let nav_up = ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, Key::ArrowUp));
    if nav_down && !matches.is_empty() {
        ui_state.palette_index = (ui_state.palette_index + 1) % matches.len();
    }
    if nav_up && !matches.is_empty() {
        ui_state.palette_index =
            (ui_state.palette_index + matches.len() - 1) % matches.len();
    }
    let run = ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, Key::Enter));

    let screen = ctx.screen_rect();
    let width = 440.0_f32.min(screen.width() - 40.0);
    let pos = egui::pos2(screen.center().x - width / 2.0, screen.top() + 90.0);

    let mut clicked: Option<usize> = None;
    egui::Area::new(egui::Id::new("command_palette"))
        .fixed_pos(pos)
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            ui.set_width(width);
            egui::Frame::window(ui.style()).show(ui, |ui| {
                let edit = egui::TextEdit::singleline(&mut ui_state.palette_query)
                    .id(egui::Id::new("palette_input"))
                    .hint_text("Type a command…  (↑↓ choose, Enter run, Esc close)")
                    .desired_width(f32::INFINITY)
                    .lock_focus(true);
                let resp = ui.add(edit);
                resp.request_focus();
                if resp.changed() { ui_state.palette_index = 0; }

                ui.add_space(4.0);
                for (i, (e, _)) in matches.iter().enumerate() {
                    let selected = i == ui_state.palette_index;
                    let text = RichText::new(e.name).size(14.0);
                    let r = ui.horizontal(|ui| {
                        let r = ui.selectable_label(selected, text);
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if !e.hint.is_empty() {
                                ui.label(RichText::new(e.hint).size(11.0)
                                    .color(theme::TEXT_DIM));
                            }
                        });
                        r
                    }).inner;
                    if r.clicked() { clicked = Some(i); }
                    if r.hovered() { ui_state.palette_index = i; }
                }
                if matches.is_empty() {
                    ui.label(RichText::new("No matching command")
                        .color(theme::TEXT_DIM).italics());
                }
                ui.add_space(2.0);
                ui.label(RichText::new("Ctrl+K opens this anytime").size(10.0)
                    .color(Color32::from_gray(110)));
            });
        });

    let choice = clicked.or(if run { Some(ui_state.palette_index) } else { None });
    if let Some(i) = choice {
        if let Some((e, _)) = matches.get(i) {
            match e.action {
                Action::Cmd(c) => app.run_command(c),
                Action::ToggleGrid => app.grid_on = !app.grid_on,
                Action::ToggleSnap => app.snap_on = !app.snap_on,
                Action::ToggleOrtho => { app.ortho_on = !app.ortho_on; if app.ortho_on { app.polar_on = false; } }
                Action::TogglePolar => { app.polar_on = !app.polar_on; if app.polar_on { app.ortho_on = false; } }
                Action::ToggleDyn => app.dyn_on = !app.dyn_on,
            }
            ui_state.palette_open = false;
        }
    }
    ui_state.palette_open
}
