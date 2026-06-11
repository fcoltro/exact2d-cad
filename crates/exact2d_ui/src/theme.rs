//! Application theme: a cohesive dark palette matched to the canvas background,
//! with one cyan accent. Replaces egui's default grey so chrome, overlays, and
//! the drawing area read as a single surface (Figma/Plasticity-style).

use egui::{Color32, Context, Rounding, Stroke, Visuals};

/// Canvas background — the darkest surface; everything else is tinted from it.
pub const CANVAS_BG: Color32 = Color32::from_rgb(20, 26, 36);
/// Panels / bars sit one step lighter than the canvas.
pub const PANEL_BG: Color32 = Color32::from_rgb(27, 34, 46);
/// Raised widgets (buttons, inputs) one step lighter again.
pub const WIDGET_BG: Color32 = Color32::from_rgb(36, 44, 58);
pub const WIDGET_HOVER: Color32 = Color32::from_rgb(48, 58, 75);
/// The single accent — selection, active tool, focus.
pub const ACCENT: Color32 = Color32::from_rgb(0, 200, 255);
pub const ACCENT_DIM: Color32 = Color32::from_rgb(0, 110, 150);
pub const TEXT: Color32 = Color32::from_rgb(214, 222, 235);
pub const TEXT_DIM: Color32 = Color32::from_rgb(140, 152, 170);
pub const OUTLINE: Color32 = Color32::from_rgb(52, 62, 78);

/// Apply the theme to the egui context. Cheap; called once per frame so it also
/// covers contexts the host recreates.
pub fn apply(ctx: &Context) {
    let mut v = Visuals::dark();

    v.panel_fill = PANEL_BG;
    v.window_fill = PANEL_BG;
    v.extreme_bg_color = CANVAS_BG;          // text-edit backgrounds
    v.faint_bg_color = WIDGET_BG;
    v.window_stroke = Stroke::new(1.0, OUTLINE);
    v.window_rounding = Rounding::same(8.0);
    v.menu_rounding = Rounding::same(6.0);
    v.popup_shadow.color = Color32::from_black_alpha(120);

    v.selection.bg_fill = ACCENT_DIM;
    v.selection.stroke = Stroke::new(1.0, ACCENT);
    v.hyperlink_color = ACCENT;

    v.override_text_color = Some(TEXT);

    let r = Rounding::same(5.0);
    v.widgets.noninteractive.bg_fill = PANEL_BG;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, OUTLINE);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, TEXT_DIM);
    v.widgets.noninteractive.rounding = r;

    v.widgets.inactive.bg_fill = WIDGET_BG;
    v.widgets.inactive.weak_bg_fill = WIDGET_BG;
    v.widgets.inactive.bg_stroke = Stroke::NONE;
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, TEXT);
    v.widgets.inactive.rounding = r;

    v.widgets.hovered.bg_fill = WIDGET_HOVER;
    v.widgets.hovered.weak_bg_fill = WIDGET_HOVER;
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT_DIM);
    v.widgets.hovered.fg_stroke = Stroke::new(1.2, Color32::WHITE);
    v.widgets.hovered.rounding = r;

    v.widgets.active.bg_fill = ACCENT_DIM;
    v.widgets.active.weak_bg_fill = ACCENT_DIM;
    v.widgets.active.bg_stroke = Stroke::new(1.0, ACCENT);
    v.widgets.active.fg_stroke = Stroke::new(1.2, Color32::WHITE);
    v.widgets.active.rounding = r;

    v.widgets.open.bg_fill = WIDGET_HOVER;
    v.widgets.open.weak_bg_fill = WIDGET_HOVER;
    v.widgets.open.bg_stroke = Stroke::new(1.0, ACCENT_DIM);
    v.widgets.open.rounding = r;

    ctx.set_visuals(v);

    ctx.style_mut(|s| {
        s.spacing.item_spacing = egui::vec2(6.0, 5.0);
        s.spacing.button_padding = egui::vec2(7.0, 4.0);
        s.spacing.menu_margin = egui::Margin::same(8.0);
    });
}

/// A floating "glass" chip frame for on-canvas overlays (prompt chip, HUDs).
pub fn glass_chip() -> egui::Frame {
    egui::Frame {
        inner_margin: egui::Margin::symmetric(12.0, 6.0),
        rounding: Rounding::same(16.0),
        fill: Color32::from_rgba_unmultiplied(27, 34, 46, 235),
        stroke: Stroke::new(1.0, OUTLINE),
        shadow: egui::epaint::Shadow {
            offset: egui::vec2(0.0, 2.0),
            blur: 10.0,
            spread: 0.0,
            color: Color32::from_black_alpha(110),
        },
        outer_margin: egui::Margin::ZERO,
    }
}
