//! Adaptive screen-space curve tessellation and the per-curve painter — the leaf
//! drawing utilities shared by the canvas. Sub-pixel chord tolerance keeps curves
//! smooth at any zoom.

use egui::Stroke;
use exact2d_geometry::{Curve, CurveSegment};

/// Screen-space chord tolerance for adaptive tessellation (sub-pixel → smooth
/// at any zoom).
const TESS_TOL_PX: f32 = 0.3;
/// Recursion / point-count safety caps.
const TESS_MAX_DEPTH: u32 = 18;
const TESS_MAX_POINTS: usize = 20_000;

pub(super) fn draw_curve(painter: &egui::Painter, c: &Curve, to_screen: &impl Fn(f64, f64) -> egui::Pos2, stroke: Stroke) {
    match c {
        Curve::Line(l) => {
            let (x0, y0) = l.p0.to_f64();
            let (x1, y1) = l.p1.to_f64();
            painter.line_segment([to_screen(x0, y0), to_screen(x1, y1)], stroke);
        }
        // Every other curve is flattened by ADAPTIVE subdivision in screen space:
        // a span is split while its midpoint deviates from the chord by more than
        // a fraction of a pixel. The segment count therefore tracks the on-screen
        // size — a circle stays smooth zoomed in, and costs almost nothing zoomed
        // out.
        other => {
            painter.add(egui::Shape::line(flatten_curve(other, to_screen), stroke));
        }
    }
}

/// Flatten a parametric curve to a screen-space polyline with sub-pixel chord
/// error via adaptive subdivision. Shared by rendering and tested directly.
pub(super) fn flatten_curve(c: &Curve, to_screen: &impl Fn(f64, f64) -> egui::Pos2) -> Vec<egui::Pos2> {
    let (t0, t1) = c.domain();
    let eval = |t: f64| { let (x, y) = c.evaluate_f64(t); to_screen(x, y) };
    let mut pts: Vec<egui::Pos2> = Vec::with_capacity(64);
    // Pre-split into a few spans so CLOSED curves (full circle/ellipse, whose
    // endpoints coincide) subdivide evenly instead of collapsing.
    const SPANS: usize = 4;
    pts.push(eval(t0));
    for i in 0..SPANS {
        let a = t0 + (t1 - t0) * i as f64 / SPANS as f64;
        let b = t0 + (t1 - t0) * (i + 1) as f64 / SPANS as f64;
        tessellate(&eval, a, b, 0, &mut pts);
    }
    pts
}

/// Append the flattened points of the parameter span `(t0, t1]` to `out`
/// (assumes the point at `t0` is already the last entry). Recursive midpoint
/// flatness test in screen space.
fn tessellate(eval: &impl Fn(f64) -> egui::Pos2, t0: f64, t1: f64, depth: u32, out: &mut Vec<egui::Pos2>) {
    if out.len() >= TESS_MAX_POINTS { return; }
    let p0 = *out.last().unwrap();
    let p1 = eval(t1);
    let tm = 0.5 * (t0 + t1);
    let pm = eval(tm);
    if depth >= TESS_MAX_DEPTH || point_seg_dist(pm, p0, p1) <= TESS_TOL_PX {
        out.push(p1);
    } else {
        tessellate(eval, t0, tm, depth + 1, out);
        tessellate(eval, tm, t1, depth + 1, out);
    }
}

/// Perpendicular distance (px) from point `p` to segment `a`–`b`.
pub(super) fn point_seg_dist(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let len2 = abx * abx + aby * aby;
    if len2 < 1e-12 {
        // Degenerate chord (closed span): distance to the shared endpoint.
        return ((p.x - a.x).powi(2) + (p.y - a.y).powi(2)).sqrt();
    }
    let t = (((p.x - a.x) * abx + (p.y - a.y) * aby) / len2).clamp(0.0, 1.0);
    let cx = a.x + t * abx;
    let cy = a.y + t * aby;
    ((p.x - cx).powi(2) + (p.y - cy).powi(2)).sqrt()
}

