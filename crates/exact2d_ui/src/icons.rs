//! Vector tool icons drawn directly with the egui painter.
//!
//! No font glyphs or image assets are used — each icon is an SVG-like path laid
//! out in a unit box, so they stay crisp at any DPI and match the vector-CAD
//! aesthetic. The designs follow the conventions users already know from
//! AutoCAD/Fusion (node squares on defining points, dashed construction
//! geometry) and Lucide/Feather (eye, magnifier, undo arrows).
//! `icon_button` renders one in a fixed-size, toggle-style button with a tooltip.

use egui::{Color32, Pos2, Rect, Response, Sense, Stroke, Ui, Vec2, vec2, pos2};

/// Which tool an icon represents. Drawing for each lives in [`Icon::draw`].
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Icon {
    Select,
    Line, Circle, Arc, Rectangle, Polygon, Spline, Polyline, Text,
    Move, Copy, Rotate, Scale, Mirror, Offset, Trim, Extend, Fillet, Chamfer,
    Stretch, Erase,
    Undo, Redo,
    Eye, EyeOff,
    ZoomIn, ZoomOut, ZoomFit,
}

const ICON_SIZE: f32 = 30.0;

/// Draw `icon` as a clickable, fixed-size button. `active` shows the pressed state.
/// Returns the egui `Response` (use `.clicked()`); `tooltip` shows on hover.
pub fn icon_button(ui: &mut Ui, icon: Icon, tooltip: &str, active: bool) -> Response {
    icon_button_sized(ui, icon, tooltip, active, ICON_SIZE)
}

/// `icon_button` with an explicit pixel size (status bar / panels use smaller).
pub fn icon_button_sized(ui: &mut Ui, icon: Icon, tooltip: &str, active: bool, size: f32) -> Response {
    let (rect, mut response) = ui.allocate_exact_size(Vec2::splat(size), Sense::click());
    let hovered = response.hovered() && ui.is_enabled();

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

    // Foreground glyph (dimmed when the button is disabled). Brighter than body
    // text so the pictograms read at small sizes.
    let fg = if !ui.is_enabled() {
        visuals.weak_text_color()
    } else if active {
        visuals.selection.stroke.color
    } else {
        Color32::from_rgb(232, 238, 248)
    };
    let inset = rect.shrink(size * 0.24);
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

/// CAD node marker: the small filled square AutoCAD puts on defining points.
fn node(painter: &egui::Painter, at: Pos2, r: Rect, color: Color32) {
    let s = r.width() * 0.13;
    painter.rect_filled(Rect::from_center_size(at, Vec2::splat(s)), 0.5, color);
}

/// Short-dash segment (construction / original geometry).
fn dashed(painter: &egui::Painter, a: Pos2, b: Pos2, s: Stroke) {
    let v = b - a;
    let len = v.length();
    if len < 1e-3 { return; }
    let dir = v / len;
    let dash = (len / 5.0).clamp(2.0, 4.0);
    let mut d = 0.0;
    while d < len {
        let e = (d + dash).min(len);
        painter.line_segment([a + dir * d, a + dir * e], s);
        d += dash * 2.0;
    }
}

impl Icon {
    fn draw(self, painter: &egui::Painter, r: Rect, color: Color32) {
        let s = Stroke::new(1.7, color);
        let thin = Stroke::new(1.1, color);
        let dim = Stroke::new(1.1, color.gamma_multiply(0.62));
        let ah = r.width() * 0.22;
        match self {
            // ── Pointer ───────────────────────────────────────────────────────
            Icon::Select => {
                // Classic pointer: tip top-left, notch, short tail.
                let pts = vec![
                    p(r, 0.18, 0.00), p(r, 0.18, 0.80), p(r, 0.38, 0.61),
                    p(r, 0.53, 0.97), p(r, 0.67, 0.91), p(r, 0.52, 0.55),
                    p(r, 0.82, 0.55), p(r, 0.18, 0.00),
                ];
                painter.add(egui::Shape::line(pts, s));
            }

            // ── Draw tools: geometry + node squares on the defining points ────
            Icon::Line => {
                let a = p(r, 0.06, 0.94);
                let b = p(r, 0.94, 0.06);
                painter.line_segment([a, b], s);
                node(painter, a, r, color);
                node(painter, b, r, color);
            }
            Icon::Polyline => {
                let pts = [p(r, 0.02, 0.90), p(r, 0.30, 0.14), p(r, 0.62, 0.64), p(r, 0.98, 0.06)];
                painter.add(egui::Shape::line(pts.to_vec(), s));
                for q in pts { node(painter, q, r, color); }
            }
            Icon::Circle => {
                // Slight overshoot: circles need it to look as big as squares.
                painter.circle_stroke(r.center(), r.width() * 0.48, s);
                painter.circle_filled(r.center(), r.width() * 0.07, color); // center point
            }
            Icon::Arc => {
                // Arc through (0.04,0.80) – (0.5,0.12) – (0.96,0.80), fully inside
                // the box (the previous design spilled past it and looked bigger).
                let c = p(r, 0.5, 0.6156);
                let rad = r.width() * 0.4956;
                let pts = arc_pts(c, rad, 0.381, -3.523, 24);
                let (first, mid, last) =
                    (pts[0], pts[pts.len() / 2], *pts.last().unwrap());
                painter.add(egui::Shape::line(pts, s));
                node(painter, first, r, color);
                node(painter, mid, r, color);
                node(painter, last, r, color);
            }
            Icon::Rectangle => {
                let rect = Rect::from_min_max(p(r, 0.04, 0.18), p(r, 0.96, 0.82));
                painter.rect_stroke(rect, 0.0, s);
                node(painter, rect.left_top(), r, color);
                node(painter, rect.right_bottom(), r, color);
            }
            Icon::Polygon => {
                let c = r.center();
                let rad = r.width() * 0.5;
                let pts: Vec<Pos2> = (0..6).map(|i| {
                    let a = std::f32::consts::FRAC_PI_2 + i as f32 * std::f32::consts::TAU / 6.0;
                    c + vec2(a.cos() * rad, -a.sin() * rad)
                }).collect();
                painter.add(egui::Shape::closed_line(pts, s));
                painter.circle_filled(c, r.width() * 0.07, color); // center point
            }
            Icon::Spline => {
                // Cubic Bézier + its dashed control polygon with node squares —
                // the standard spline pictogram.
                let (p0, p1, p2, p3) =
                    (p(r, 0.04, 0.92), p(r, 0.22, 0.02), p(r, 0.78, 0.98), p(r, 0.96, 0.08));
                dashed(painter, p0, p1, dim);
                dashed(painter, p1, p2, dim);
                dashed(painter, p2, p3, dim);
                let n = 18;
                let pts: Vec<Pos2> = (0..=n).map(|i| {
                    let t = i as f32 / n as f32;
                    let u = 1.0 - t;
                    pos2(
                        u*u*u*p0.x + 3.0*u*u*t*p1.x + 3.0*u*t*t*p2.x + t*t*t*p3.x,
                        u*u*u*p0.y + 3.0*u*u*t*p1.y + 3.0*u*t*t*p2.y + t*t*t*p3.y,
                    )
                }).collect();
                painter.add(egui::Shape::line(pts, s));
                for q in [p0, p1, p2, p3] { node(painter, q, r, color); }
            }
            Icon::Text => {
                // A drawn capital A (strokes, not a font glyph).
                painter.line_segment([p(r, 0.12, 0.97), p(r, 0.50, 0.02)], s);
                painter.line_segment([p(r, 0.50, 0.02), p(r, 0.88, 0.97)], s);
                painter.line_segment([p(r, 0.27, 0.62), p(r, 0.73, 0.62)], s);
            }

            // ── Modify tools ──────────────────────────────────────────────────
            Icon::Move => {
                let c = r.center();
                for (dx, dy) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
                    let tip = c + vec2(dx, dy) * r.width() * 0.5;
                    painter.line_segment([c, tip], s);
                    arrowhead(painter, tip, dx, dy, ah, s);
                }
            }
            Icon::Copy => {
                // Two offset sheets: original behind (dim), copy in front.
                painter.rect_stroke(Rect::from_min_max(p(r, 0.05, 0.05), p(r, 0.62, 0.62)), 1.0, dim);
                painter.rect_stroke(Rect::from_min_max(p(r, 0.36, 0.36), p(r, 0.95, 0.95)), 1.0, s);
            }
            Icon::Rotate => {
                let c = r.center();
                let rad = r.width() * 0.42;
                painter.add(egui::Shape::line(arc_pts(c, rad, -1.1, 3.3, 24), s));
                let a = 3.3f32;
                let tip = c + vec2(a.cos() * rad, a.sin() * rad);
                arrowhead(painter, tip, -a.sin(), a.cos(), ah, s);
                painter.circle_filled(c, r.width() * 0.07, color); // pivot
            }
            Icon::Scale => {
                // Small solid square growing into the dashed large one.
                let big = Rect::from_min_max(p(r, 0.05, 0.05), p(r, 0.95, 0.95));
                for (a, b) in [
                    (big.left_top(), big.right_top()),
                    (big.right_top(), big.right_bottom()),
                    (big.right_bottom(), big.left_bottom()),
                    (big.left_bottom(), big.left_top()),
                ] { dashed(painter, a, b, dim); }
                painter.rect_stroke(Rect::from_min_max(p(r, 0.05, 0.55), p(r, 0.45, 0.95)), 0.0, s);
                let tip = p(r, 0.86, 0.14);
                painter.line_segment([p(r, 0.45, 0.55), tip], s);
                arrowhead(painter, tip, 0.7, -0.7, ah, s);
            }
            Icon::Mirror => {
                // Solid shape, dashed axis, dim reflected copy.
                let axis_top = p(r, 0.5, 0.0);
                let axis_bot = p(r, 0.5, 1.0);
                dashed(painter, axis_top, axis_bot, dim);
                painter.add(egui::Shape::closed_line(
                    vec![p(r, 0.06, 0.18), p(r, 0.38, 0.5), p(r, 0.06, 0.82)], s));
                painter.add(egui::Shape::closed_line(
                    vec![p(r, 0.94, 0.18), p(r, 0.62, 0.5), p(r, 0.94, 0.82)], dim));
            }
            Icon::Offset => {
                // A corner path and its parallel copy.
                painter.add(egui::Shape::line(
                    vec![p(r, 0.10, 0.95), p(r, 0.10, 0.10), p(r, 0.95, 0.10)], s));
                painter.add(egui::Shape::line(
                    vec![p(r, 0.42, 0.95), p(r, 0.42, 0.42), p(r, 0.95, 0.42)], dim));
            }
            Icon::Trim => {
                // A curve with the trimmed piece dashed past the cutting edge.
                let cut_a = p(r, 0.62, 0.02);
                let cut_b = p(r, 0.62, 0.98);
                painter.line_segment([cut_a, cut_b], thin);
                painter.line_segment([p(r, 0.02, 0.30), p(r, 0.62, 0.30)], s);
                dashed(painter, p(r, 0.62, 0.30), p(r, 0.98, 0.30), dim);
                painter.line_segment([p(r, 0.02, 0.70), p(r, 0.62, 0.70)], s);
                dashed(painter, p(r, 0.62, 0.70), p(r, 0.98, 0.70), dim);
                // Small X over the discarded piece.
                painter.line_segment([p(r, 0.74, 0.44), p(r, 0.86, 0.56)], thin);
                painter.line_segment([p(r, 0.86, 0.44), p(r, 0.74, 0.56)], thin);
            }
            Icon::Extend => {
                // A line gaining a dashed extension up to a boundary.
                painter.line_segment([p(r, 0.02, 0.5), p(r, 0.45, 0.5)], s);
                dashed(painter, p(r, 0.45, 0.5), p(r, 0.80, 0.5), dim);
                arrowhead(painter, p(r, 0.84, 0.5), 1.0, 0.0, ah, s);
                painter.line_segment([p(r, 0.92, 0.05), p(r, 0.92, 0.95)], s); // boundary
            }
            Icon::Fillet => {
                // Square corner (dashed original) replaced by a rounded arc.
                dashed(painter, p(r, 0.12, 0.58), p(r, 0.12, 0.92), dim);
                dashed(painter, p(r, 0.12, 0.92), p(r, 0.46, 0.92), dim);
                painter.line_segment([p(r, 0.12, 0.04), p(r, 0.12, 0.58)], s);
                painter.line_segment([p(r, 0.46, 0.92), p(r, 0.96, 0.92)], s);
                painter.add(egui::Shape::line(arc_pts(
                    p(r, 0.46, 0.58), r.width() * 0.34,
                    std::f32::consts::PI, std::f32::consts::FRAC_PI_2, 16), s));
            }
            Icon::Chamfer => {
                // Square corner (dashed original) replaced by a bevel.
                dashed(painter, p(r, 0.12, 0.58), p(r, 0.12, 0.92), dim);
                dashed(painter, p(r, 0.12, 0.92), p(r, 0.46, 0.92), dim);
                painter.line_segment([p(r, 0.12, 0.04), p(r, 0.12, 0.58)], s);
                painter.line_segment([p(r, 0.46, 0.92), p(r, 0.96, 0.92)], s);
                painter.line_segment([p(r, 0.12, 0.58), p(r, 0.46, 0.92)], s);
            }
            Icon::Stretch => {
                // Crossing window with a grip pulled to the right.
                let win = Rect::from_min_max(p(r, 0.05, 0.22), p(r, 0.52, 0.78));
                for (a, b) in [
                    (win.left_top(), win.right_top()),
                    (win.right_top(), win.right_bottom()),
                    (win.right_bottom(), win.left_bottom()),
                    (win.left_bottom(), win.left_top()),
                ] { dashed(painter, a, b, dim); }
                let grip = p(r, 0.52, 0.5);
                node(painter, grip, r, color);
                let tip = p(r, 0.92, 0.5);
                painter.line_segment([grip, tip], s);
                arrowhead(painter, tip, 1.0, 0.0, ah, s);
            }
            Icon::Erase => {
                // Tilted eraser block with its band, over a swipe line.
                painter.add(egui::Shape::closed_line(vec![
                    p(r, 0.12, 0.62), p(r, 0.52, 0.10), p(r, 0.82, 0.34), p(r, 0.42, 0.86),
                ], s));
                painter.line_segment([p(r, 0.27, 0.43), p(r, 0.57, 0.67)], thin); // band
                painter.line_segment([p(r, 0.50, 0.95), p(r, 0.95, 0.95)], thin); // swipe
            }

            // ── History ───────────────────────────────────────────────────────
            Icon::Undo => {
                // Counter-clockwise arc, arrowhead on the left end.
                let c = p(r, 0.5, 0.55);
                let rad = r.width() * 0.44;
                painter.add(egui::Shape::line(arc_pts(c, rad, -0.25, -2.90, 18), s));
                let a = -2.90f32;
                let tip = c + vec2(a.cos() * rad, a.sin() * rad);
                arrowhead(painter, tip, a.sin(), -a.cos(), ah, s);
            }
            Icon::Redo => {
                // Mirrored: clockwise arc, arrowhead on the right end.
                let c = p(r, 0.5, 0.55);
                let rad = r.width() * 0.44;
                painter.add(egui::Shape::line(arc_pts(c, rad, -2.89, -0.24, 18), s));
                let a = -0.24f32;
                let tip = c + vec2(a.cos() * rad, a.sin() * rad);
                arrowhead(painter, tip, -a.sin(), a.cos(), ah, s);
            }

            // ── Visibility ────────────────────────────────────────────────────
            Icon::Eye | Icon::EyeOff => {
                // Almond eye outline + pupil (Lucide-style).
                let n = 12;
                let lid = |sign: f32| -> Vec<Pos2> {
                    (0..=n).map(|i| {
                        let t = i as f32 / n as f32;
                        p(r, 0.02 + 0.96 * t,
                          0.5 + sign * 0.40 * (std::f32::consts::PI * t).sin())
                    }).collect()
                };
                painter.add(egui::Shape::line(lid(-1.0), s));
                painter.add(egui::Shape::line(lid(1.0), s));
                painter.circle_stroke(r.center(), r.width() * 0.15, s);
                if self == Icon::EyeOff {
                    painter.line_segment([p(r, 0.12, 0.95), p(r, 0.88, 0.05)], s);
                }
            }

            // ── Zoom ──────────────────────────────────────────────────────────
            Icon::ZoomIn | Icon::ZoomOut => {
                let c = p(r, 0.42, 0.42);
                let rad = r.width() * 0.34;
                painter.circle_stroke(c, rad, s);
                let h0 = c + vec2(rad * 0.72, rad * 0.72);
                painter.line_segment([h0, p(r, 0.96, 0.96)], s);
                painter.line_segment([c - vec2(rad * 0.5, 0.0), c + vec2(rad * 0.5, 0.0)], s);
                if self == Icon::ZoomIn {
                    painter.line_segment([c - vec2(0.0, rad * 0.5), c + vec2(0.0, rad * 0.5)], s);
                }
            }
            Icon::ZoomFit => {
                // Frame corner brackets around a small shape.
                let k = 0.22;
                for (cx, cy, dx, dy) in [
                    (0.04, 0.04, 1.0, 1.0), (0.96, 0.04, -1.0, 1.0),
                    (0.96, 0.96, -1.0, -1.0), (0.04, 0.96, 1.0, -1.0),
                ] {
                    let corner = p(r, cx, cy);
                    painter.line_segment([corner, p(r, cx + dx * k, cy)], s);
                    painter.line_segment([corner, p(r, cx, cy + dy * k)], s);
                }
                painter.rect_stroke(Rect::from_min_max(p(r, 0.32, 0.38), p(r, 0.68, 0.62)), 0.0, thin);
            }
        }
    }
}
