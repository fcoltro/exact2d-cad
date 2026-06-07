//! Vector tool icons drawn directly with the egui painter.
//!
//! No font glyphs or image assets are used — each icon is a few strokes laid out in
//! a unit box, so they stay crisp at any DPI and match the vector-CAD aesthetic.
//! `icon_button` renders one in a fixed-size, toggle-style button with a tooltip.

use egui::{Color32, Pos2, Rect, Response, Sense, Stroke, Ui, Vec2, vec2, pos2};

/// Which tool an icon represents. Drawing for each lives in [`Icon::draw`].
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Icon {
    Select,
    Line, Circle, Arc, Rectangle, Polygon, Spline, Polyline, Point, Dimension,
    Move, Copy, Rotate, Scale, Mirror, Offset, Trim, Extend, Fillet, Chamfer,
    Stretch, Erase, Array,
}

const ICON_SIZE: f32 = 30.0;

/// Draw `icon` as a clickable, fixed-size button. `active` shows the pressed state.
/// Returns the egui `Response` (use `.clicked()`); `tooltip` shows on hover.
pub fn icon_button(ui: &mut Ui, icon: Icon, tooltip: &str, active: bool) -> Response {
    let (rect, mut response) = ui.allocate_exact_size(Vec2::splat(ICON_SIZE), Sense::click());
    let hovered = response.hovered();

    // Button chrome.
    let visuals = ui.visuals();
    let bg = if active {
        visuals.selection.bg_fill
    } else if hovered {
        visuals.widgets.hovered.bg_fill
    } else {
        visuals.widgets.inactive.bg_fill
    };
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, bg);
    if active || hovered {
        painter.rect_stroke(rect, 4.0, Stroke::new(1.0, visuals.widgets.active.bg_stroke.color));
    }

    // Foreground glyph.
    let fg = if active { visuals.selection.stroke.color } else { visuals.text_color() };
    let inset = rect.shrink(ICON_SIZE * 0.26);
    icon.draw(&painter, inset, fg);

    if hovered {
        response = response.on_hover_text(tooltip);
    }
    response
}

/// Map unit coordinates (fx,fy ∈ [0,1], y-down) into the icon's inset rect.
fn p(r: Rect, fx: f32, fy: f32) -> Pos2 {
    pos2(r.left() + fx * r.width(), r.top() + fy * r.height())
}

/// Polyline approximation of an arc (angles in radians, y-down screen space).
fn arc_pts(center: Pos2, radius: f32, a0: f32, a1: f32, n: usize) -> Vec<Pos2> {
    (0..=n).map(|i| {
        let t = a0 + (a1 - a0) * (i as f32 / n as f32);
        center + vec2(t.cos() * radius, t.sin() * radius)
    }).collect()
}

/// A small arrowhead at `tip` pointing along unit direction (dx,dy).
fn arrowhead(painter: &egui::Painter, tip: Pos2, dx: f32, dy: f32, size: f32, s: Stroke) {
    // Two barbs swept back ~150° from the direction of travel.
    for ang in [2.6f32, -2.6f32] {
        let (c, sn) = (ang.cos(), ang.sin());
        let bx = dx * c - dy * sn;
        let by = dx * sn + dy * c;
        painter.line_segment([tip, tip + vec2(bx, by) * size], s);
    }
}

impl Icon {
    fn draw(self, painter: &egui::Painter, r: Rect, color: Color32) {
        let s = Stroke::new(1.6, color);
        let thin = Stroke::new(1.0, color);
        let ah = r.width() * 0.22;
        match self {
            Icon::Select => {
                // Mouse-cursor arrow.
                let tip = p(r, 0.15, 0.05);
                let pts = vec![
                    tip, p(r, 0.15, 0.85), p(r, 0.37, 0.63),
                    p(r, 0.52, 0.95), p(r, 0.64, 0.88), p(r, 0.49, 0.57),
                    p(r, 0.8, 0.55), tip,
                ];
                painter.add(egui::Shape::line(pts, s));
            }
            Icon::Line => { painter.line_segment([p(r, 0.05, 0.95), p(r, 0.95, 0.05)], s); }
            Icon::Circle => { painter.circle_stroke(r.center(), r.width() * 0.46, s); }
            Icon::Arc => {
                painter.add(egui::Shape::line(
                    arc_pts(p(r, 0.5, 0.95), r.width() * 0.85, -2.8, -0.34, 24), s));
            }
            Icon::Rectangle => { painter.rect_stroke(r.shrink(r.width() * 0.05), 0.0, s); }
            Icon::Polygon => {
                let c = r.center();
                let rad = r.width() * 0.5;
                let pts: Vec<Pos2> = (0..6).map(|i| {
                    let a = std::f32::consts::FRAC_PI_2 + i as f32 * std::f32::consts::TAU / 6.0;
                    c + vec2(a.cos() * rad, -a.sin() * rad)
                }).collect();
                painter.add(egui::Shape::closed_line(pts, s));
            }
            Icon::Spline => {
                // Smooth S-curve via an arc-pair polyline.
                let mut pts = arc_pts(p(r, 0.3, 0.3), r.width() * 0.3, 1.0, -1.6, 12);
                pts.extend(arc_pts(p(r, 0.7, 0.7), r.width() * 0.3, 1.9, 4.2, 12));
                painter.add(egui::Shape::line(pts, s));
            }
            Icon::Polyline => {
                painter.add(egui::Shape::line(vec![
                    p(r, 0.02, 0.85), p(r, 0.35, 0.2), p(r, 0.6, 0.7), p(r, 0.98, 0.1),
                ], s));
            }
            Icon::Point => { painter.circle_filled(r.center(), r.width() * 0.12, color); }
            Icon::Dimension => {
                // Dimension line with end arrows + extension ticks.
                let (l, rr, y) = (p(r, 0.1, 0.55), p(r, 0.9, 0.55), 0.55);
                painter.line_segment([l, rr], s);
                arrowhead(painter, l, 1.0, 0.0, ah, s);
                arrowhead(painter, rr, -1.0, 0.0, ah, s);
                painter.line_segment([p(r, 0.1, 0.15), p(r, 0.1, y + 0.2)], thin);
                painter.line_segment([p(r, 0.9, 0.15), p(r, 0.9, y + 0.2)], thin);
            }
            Icon::Move => {
                let c = r.center();
                for (dx, dy) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
                    let tip = c + vec2(dx, dy) * r.width() * 0.5;
                    painter.line_segment([c, tip], s);
                    arrowhead(painter, tip, dx, dy, ah, s);
                }
            }
            Icon::Copy => {
                painter.rect_stroke(Rect::from_min_size(p(r, 0.05, 0.25), r.size() * 0.55), 0.0, s);
                painter.rect_stroke(Rect::from_min_size(p(r, 0.4, 0.0), r.size() * 0.55), 0.0, s);
            }
            Icon::Rotate => {
                let c = r.center();
                let rad = r.width() * 0.45;
                painter.add(egui::Shape::line(arc_pts(c, rad, -1.2, 3.4, 24), s));
                // Arrowhead at the open end.
                let a = 3.4f32;
                let tip = c + vec2(a.cos() * rad, a.sin() * rad);
                arrowhead(painter, tip, -a.sin(), a.cos(), ah, s);
            }
            Icon::Scale => {
                painter.rect_stroke(Rect::from_min_size(p(r, 0.0, 0.35), r.size() * 0.55), 0.0, thin);
                painter.rect_stroke(Rect::from_min_size(p(r, 0.0, 0.0), r.size() * 0.98), 0.0, s);
                let d = p(r, 0.92, 0.92);
                painter.line_segment([p(r, 0.55, 0.55), d], s);
                arrowhead(painter, d, 0.7, 0.7, ah, s);
            }
            Icon::Mirror => {
                // A shape and its reflection across a dashed axis.
                painter.add(egui::Shape::line(vec![p(r, 0.05, 0.1), p(r, 0.32, 0.5), p(r, 0.05, 0.9)], s));
                painter.add(egui::Shape::line(vec![p(r, 0.95, 0.1), p(r, 0.68, 0.5), p(r, 0.95, 0.9)], s));
                painter.line_segment([p(r, 0.5, 0.0), p(r, 0.5, 1.0)], Stroke::new(1.0, Color32::GRAY));
            }
            Icon::Offset => {
                painter.add(egui::Shape::line(vec![p(r, 0.1, 0.9), p(r, 0.1, 0.1), p(r, 0.7, 0.1)], s));
                painter.add(egui::Shape::line(vec![p(r, 0.35, 0.9), p(r, 0.35, 0.35), p(r, 0.9, 0.35)], thin));
            }
            Icon::Trim => {
                // Scissors: two blades (circles + crossed legs).
                painter.circle_stroke(p(r, 0.2, 0.78), r.width() * 0.14, thin);
                painter.circle_stroke(p(r, 0.2, 0.22), r.width() * 0.14, thin);
                painter.line_segment([p(r, 0.32, 0.68), p(r, 0.95, 0.1)], s);
                painter.line_segment([p(r, 0.32, 0.32), p(r, 0.95, 0.9)], s);
            }
            Icon::Extend => {
                // A short line extended (dashed) up to a wall.
                painter.line_segment([p(r, 0.05, 0.5), p(r, 0.45, 0.5)], s);
                let mut x = 0.45;
                while x < 0.85 {
                    painter.line_segment([p(r, x, 0.5), p(r, (x + 0.08).min(0.85), 0.5)], thin);
                    x += 0.16;
                }
                painter.line_segment([p(r, 0.9, 0.05), p(r, 0.9, 0.95)], s); // wall
                arrowhead(painter, p(r, 0.85, 0.5), 1.0, 0.0, ah, thin);
            }
            Icon::Fillet => {
                // Right-angle corner with a rounded inner arc.
                painter.add(egui::Shape::line(vec![p(r, 0.1, 0.05), p(r, 0.1, 0.6)], s));
                painter.add(egui::Shape::line(vec![p(r, 0.4, 0.9), p(r, 0.95, 0.9)], s));
                painter.add(egui::Shape::line(arc_pts(p(r, 0.4, 0.6), r.width() * 0.3, std::f32::consts::PI, std::f32::consts::FRAC_PI_2, 16), s));
            }
            Icon::Chamfer => {
                // Right-angle corner with a beveled cut.
                painter.add(egui::Shape::line(vec![p(r, 0.1, 0.05), p(r, 0.1, 0.6)], s));
                painter.add(egui::Shape::line(vec![p(r, 0.4, 0.9), p(r, 0.95, 0.9)], s));
                painter.line_segment([p(r, 0.1, 0.6), p(r, 0.4, 0.9)], s); // bevel
            }
            Icon::Stretch => {
                painter.add(egui::Shape::line(vec![p(r, 0.1, 0.7), p(r, 0.45, 0.3), p(r, 0.6, 0.55)], s));
                let tip = p(r, 0.95, 0.5);
                painter.line_segment([p(r, 0.55, 0.5), tip], s);
                arrowhead(painter, tip, 1.0, 0.0, ah, s);
            }
            Icon::Erase => {
                // Eraser block on a baseline.
                let body = Rect::from_min_max(p(r, 0.15, 0.2), p(r, 0.7, 0.6));
                painter.rect_stroke(body, 1.0, s);
                painter.line_segment([p(r, 0.15, 0.45), p(r, 0.7, 0.45)], thin);
                painter.line_segment([p(r, 0.05, 0.85), p(r, 0.95, 0.85)], s);
            }
            Icon::Array => {
                for (gx, gy) in [(0usize, 0usize), (1, 0), (0, 1), (1, 1)] {
                    let o = p(r, 0.05 + gx as f32 * 0.5, 0.05 + gy as f32 * 0.5);
                    painter.rect_stroke(Rect::from_min_size(o, r.size() * 0.4), 0.0, thin);
                }
            }
        }
    }
}
