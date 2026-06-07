//! SVG import/export (spec §5.3). Exact vector output (curves as paths) and a
//! focused importer for the common path/shape elements.
//!
//! SVG uses a y-down coordinate system with the origin at the top-left, whereas
//! CAD uses y-up. We flip the y-axis using the drawing height `H` recorded in the
//! viewBox, so a world point (x, y) maps to SVG (x, H − y) and back.

use exact2d_algebra::Rational;
use exact2d_geometry::{
    Curve, CurveSegment, Point2d, LineSeg, CircularArc, EllipticalArc, CubicBezier, PolyCurve,
};
use exact2d_document::{Document, EntityKind};

const TAU: f64 = std::f64::consts::TAU;

// ── Export ────────────────────────────────────────────────────────────────────

/// Serialize a document to an SVG string. The viewBox is sized to the drawing
/// extents (plus a small margin); y is flipped so the drawing appears upright.
pub fn export_svg(doc: &Document) -> String {
    let (w, h, h_flip) = match doc.extents() {
        Some(bb) => {
            let (x0, y0) = bb.min.to_f64();
            let (x1, y1) = bb.max.to_f64();
            let margin = 0.05 * ((x1 - x0).max(y1 - y0)).max(1.0);
            // Shift so min corner is at margin; height used for the y-flip.
            let w = (x1 - x0) + 2.0 * margin;
            let h = (y1 - y0) + 2.0 * margin;
            // Translate world so x0-margin → 0; y-flip height is y1+margin.
            (w, h, y1 + margin)
        }
        None => (100.0, 100.0, 100.0),
    };
    let x_shift = doc.extents().map(|bb| bb.min.x.to_f64() - 0.05 * ((bb.max.x.to_f64()-bb.min.x.to_f64()).max(bb.max.y.to_f64()-bb.min.y.to_f64())).max(1.0)).unwrap_or(0.0);

    let fy = |y: f64| h_flip - y;
    let fx = |x: f64| x - x_shift;

    let mut s = String::new();
    s.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {:.6} {:.6}\" data-h-flip=\"{:.9}\" data-x-shift=\"{:.9}\">\n",
        w, h, h_flip, x_shift));

    for e in doc.iter() {
        let stroke = stroke_for(doc, e);
        if let Some(path) = entity_to_svg(&e.kind, &fx, &fy, &stroke) {
            s.push_str(&path);
            s.push('\n');
        }
    }
    s.push_str("</svg>\n");
    s
}

fn stroke_for(doc: &Document, e: &exact2d_document::Entity) -> String {
    let (r, g, b) = match &e.color {
        exact2d_document::Color::Rgb(r, g, b) => (*r, *g, *b),
        _ => doc.layers.get(e.layer).map(|l| l.color).unwrap_or((0, 0, 0)),
    };
    format!("#{:02x}{:02x}{:02x}", r, g, b)
}

fn entity_to_svg(kind: &EntityKind, fx: &impl Fn(f64) -> f64, fy: &impl Fn(f64) -> f64, stroke: &str) -> Option<String> {
    let style = format!("fill=\"none\" stroke=\"{}\" stroke-width=\"0.25\"", stroke);
    match kind {
        EntityKind::Curve(Curve::Line(l)) => {
            let (x1, y1) = l.p0.to_f64();
            let (x2, y2) = l.p1.to_f64();
            Some(format!("  <line x1=\"{:.6}\" y1=\"{:.6}\" x2=\"{:.6}\" y2=\"{:.6}\" {}/>",
                fx(x1), fy(y1), fx(x2), fy(y2), style))
        }
        EntityKind::Curve(Curve::Arc(a)) => {
            let (cx, cy) = a.center.to_f64();
            let r = a.radius.to_f64();
            let span = (a.end_angle - a.start_angle).abs();
            if (span - TAU).abs() < 1e-9 {
                Some(format!("  <circle cx=\"{:.6}\" cy=\"{:.6}\" r=\"{:.6}\" {}/>",
                    fx(cx), fy(cy), r, style))
            } else {
                Some(format!("  <path d=\"{}\" {}/>", arc_path(a, fx, fy), style))
            }
        }
        EntityKind::Curve(Curve::Ellipse(e)) => {
            // Axis-aligned ellipse → <ellipse>; rotated → path approximation.
            let (cx, cy) = e.center.to_f64();
            if e.rotation.abs() < 1e-9 {
                Some(format!("  <ellipse cx=\"{:.6}\" cy=\"{:.6}\" rx=\"{:.6}\" ry=\"{:.6}\" {}/>",
                    fx(cx), fy(cy), e.semi_major.to_f64(), e.semi_minor.to_f64(), style))
            } else {
                Some(format!("  <path d=\"{}\" {}/>", sampled_path(&Curve::Ellipse(e.clone()), fx, fy), style))
            }
        }
        EntityKind::Curve(Curve::Bezier(b)) => {
            let (x0, y0) = b.p0.to_f64();
            let (x1, y1) = b.p1.to_f64();
            let (x2, y2) = b.p2.to_f64();
            let (x3, y3) = b.p3.to_f64();
            Some(format!("  <path d=\"M {:.6} {:.6} C {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}\" {}/>",
                fx(x0), fy(y0), fx(x1), fy(y1), fx(x2), fy(y2), fx(x3), fy(y3), style))
        }
        EntityKind::Curve(Curve::Poly(pc)) => {
            Some(format!("  <path d=\"{}\" {}/>", polycurve_path(pc, fx, fy), style))
        }
        EntityKind::Point(p) => {
            let (x, y) = p.to_f64();
            Some(format!("  <circle cx=\"{:.6}\" cy=\"{:.6}\" r=\"0.5\" fill=\"{}\"/>", fx(x), fy(y), stroke))
        }
        EntityKind::Text { anchor, content, height, .. } => {
            let (x, y) = anchor.to_f64();
            Some(format!("  <text x=\"{:.6}\" y=\"{:.6}\" font-size=\"{:.6}\" fill=\"{}\">{}</text>",
                fx(x), fy(y), height, stroke, xml_escape(content)))
        }
        _ => None,
    }
}

fn arc_path(a: &CircularArc, fx: &impl Fn(f64) -> f64, fy: &impl Fn(f64) -> f64) -> String {
    let (sx, sy) = a.start_point();
    let (ex, ey) = a.end_point();
    let r = a.radius.to_f64();
    let large = if a.included_angle() > std::f64::consts::PI { 1 } else { 0 };
    // CCW in world = CW in flipped SVG → sweep flag 0.
    let sweep = 0;
    format!("M {:.6} {:.6} A {:.6} {:.6} 0 {} {} {:.6} {:.6}",
        fx(sx), fy(sy), r, r, large, sweep, fx(ex), fy(ey))
}

fn sampled_path(c: &Curve, fx: &impl Fn(f64) -> f64, fy: &impl Fn(f64) -> f64) -> String {
    let (t0, t1) = c.domain();
    let mut d = String::new();
    for i in 0..=48 {
        let t = t0 + (t1 - t0) * i as f64 / 48.0;
        let (x, y) = c.evaluate_f64(t);
        d.push_str(&format!("{} {:.6} {:.6} ", if i == 0 { "M" } else { "L" }, fx(x), fy(y)));
    }
    d.trim_end().to_string()
}

fn polycurve_path(pc: &PolyCurve, fx: &impl Fn(f64) -> f64, fy: &impl Fn(f64) -> f64) -> String {
    let mut d = String::new();
    let mut first = true;
    for seg in &pc.segments {
        match seg {
            Curve::Line(l) => {
                let (x0, y0) = l.p0.to_f64();
                let (x1, y1) = l.p1.to_f64();
                if first { d.push_str(&format!("M {:.6} {:.6} ", fx(x0), fy(y0))); first = false; }
                d.push_str(&format!("L {:.6} {:.6} ", fx(x1), fy(y1)));
            }
            Curve::Bezier(b) => {
                let (x0, y0) = b.p0.to_f64();
                if first { d.push_str(&format!("M {:.6} {:.6} ", fx(x0), fy(y0))); first = false; }
                let (x1, y1) = b.p1.to_f64();
                let (x2, y2) = b.p2.to_f64();
                let (x3, y3) = b.p3.to_f64();
                d.push_str(&format!("C {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} ",
                    fx(x1), fy(y1), fx(x2), fy(y2), fx(x3), fy(y3)));
            }
            Curve::Arc(a) => {
                let (sx, sy) = a.start_point();
                let (ex, ey) = a.end_point();
                if first { d.push_str(&format!("M {:.6} {:.6} ", fx(sx), fy(sy))); first = false; }
                let r = a.radius.to_f64();
                let large = if a.included_angle() > std::f64::consts::PI { 1 } else { 0 };
                d.push_str(&format!("A {:.6} {:.6} 0 {} 0 {:.6} {:.6} ", r, r, large, fx(ex), fy(ey)));
            }
            other => {
                let (x, y) = other.evaluate_f64(other.domain().1);
                d.push_str(&format!("L {:.6} {:.6} ", fx(x), fy(y)));
            }
        }
    }
    d.trim_end().to_string()
}

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

// ── Import ────────────────────────────────────────────────────────────────────

/// Parse an SVG string into a document. Handles `<line>`, `<circle>`,
/// `<ellipse>`, and `<path>` (M/L/C/Z absolute commands). The y-flip height is
/// taken from the `data-h-flip` attribute when present, else the viewBox height.
pub fn import_svg(svg: &str) -> Document {
    let mut doc = Document::new();
    let h_flip = attr(svg, "data-h-flip").and_then(|v| v.parse().ok())
        .or_else(|| viewbox_height(svg))
        .unwrap_or(0.0);
    let x_shift: f64 = attr(svg, "data-x-shift").and_then(|v| v.parse().ok()).unwrap_or(0.0);
    let fy = |y: f64| h_flip - y;
    let fx = |x: f64| x + x_shift;

    for el in elements(svg) {
        match el.name.as_str() {
            "line" => {
                let x1 = fx(num(&el, "x1")); let y1 = fy(num(&el, "y1"));
                let x2 = fx(num(&el, "x2")); let y2 = fy(num(&el, "y2"));
                doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(
                    Point2d::from_f64(x1, y1), Point2d::from_f64(x2, y2)))));
            }
            "circle" => {
                let cx = fx(num(&el, "cx")); let cy = fy(num(&el, "cy"));
                let r = num(&el, "r");
                if r > 0.5 + 1e-9 || el.attrs.iter().any(|(k, _)| k == "r") && r > 0.0 {
                    doc.add(EntityKind::Curve(Curve::Arc(CircularArc::new(
                        Point2d::from_f64(cx, cy), Rational::from_f64_approx(r), 0.0, TAU))));
                }
            }
            "ellipse" => {
                let cx = fx(num(&el, "cx")); let cy = fy(num(&el, "cy"));
                let rx = num(&el, "rx"); let ry = num(&el, "ry");
                doc.add(EntityKind::Curve(Curve::Ellipse(EllipticalArc::new(
                    Point2d::from_f64(cx, cy),
                    Rational::from_f64_approx(rx), Rational::from_f64_approx(ry),
                    0.0, 0.0, TAU))));
            }
            "path" => {
                if let Some(d) = el.attrs.iter().find(|(k, _)| k == "d").map(|(_, v)| v.clone()) {
                    for c in parse_path(&d, &fx, &fy) {
                        doc.add(EntityKind::Curve(c));
                    }
                }
            }
            _ => {}
        }
    }
    doc
}

struct Element { name: String, attrs: Vec<(String, String)> }

fn elements(svg: &str) -> Vec<Element> {
    let mut out = Vec::new();
    let bytes = svg.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'<' && i + 1 < bytes.len() && bytes[i + 1] != b'/' && bytes[i + 1] != b'?' {
            // Read tag name
            let start = i + 1;
            let mut j = start;
            while j < bytes.len() && !bytes[j].is_ascii_whitespace() && bytes[j] != b'>' && bytes[j] != b'/' { j += 1; }
            let name = svg[start..j].to_string();
            // Read until '>'
            let mut k = j;
            while k < bytes.len() && bytes[k] != b'>' { k += 1; }
            let attr_text = &svg[j..k];
            out.push(Element { name, attrs: parse_attrs(attr_text) });
            i = k + 1;
        } else {
            i += 1;
        }
    }
    out
}

fn parse_attrs(text: &str) -> Vec<(String, String)> {
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(eq) = rest.find('=') {
        let key = rest[..eq].trim().trim_end_matches('/').trim().to_string();
        let after = &rest[eq + 1..];
        if let Some(q1) = after.find('"') {
            if let Some(q2) = after[q1 + 1..].find('"') {
                let val = after[q1 + 1..q1 + 1 + q2].to_string();
                if !key.is_empty() { out.push((key, val)); }
                rest = &after[q1 + 1 + q2 + 1..];
                continue;
            }
        }
        break;
    }
    out
}

fn attr(svg: &str, name: &str) -> Option<String> {
    let needle = format!("{}=\"", name);
    let start = svg.find(&needle)? + needle.len();
    let end = svg[start..].find('"')? + start;
    Some(svg[start..end].to_string())
}

fn viewbox_height(svg: &str) -> Option<f64> {
    let vb = attr(svg, "viewBox")?;
    let parts: Vec<f64> = vb.split_whitespace().filter_map(|p| p.parse().ok()).collect();
    parts.get(3).copied()
}

fn num(el: &Element, key: &str) -> f64 {
    el.attrs.iter().find(|(k, _)| k == key).and_then(|(_, v)| v.parse().ok()).unwrap_or(0.0)
}

/// Parse an SVG path `d` string. Supports M/L/C/Z absolute commands and the
/// `A` arc command, with the y-flip applied to each coordinate.
fn parse_path(d: &str, fx: &impl Fn(f64) -> f64, fy: &impl Fn(f64) -> f64) -> Vec<Curve> {
    let toks = tokenize_path(d);
    let mut curves = Vec::new();
    let mut i = 0;
    let mut cur = (0.0f64, 0.0f64);
    let mut start = (0.0f64, 0.0f64);

    while i < toks.len() {
        let cmd = match &toks[i] { Tok::Cmd(c) => *c, _ => { i += 1; continue; } };
        i += 1;
        match cmd {
            'M' => {
                let (x, y) = (read_num(&toks, &mut i), read_num(&toks, &mut i));
                cur = (fx(x), fy(y)); start = cur;
            }
            'L' => {
                let (x, y) = (read_num(&toks, &mut i), read_num(&toks, &mut i));
                let next = (fx(x), fy(y));
                curves.push(Curve::Line(LineSeg::from_endpoints(
                    Point2d::from_f64(cur.0, cur.1), Point2d::from_f64(next.0, next.1))));
                cur = next;
            }
            'C' => {
                let c1 = (fx(read_num(&toks, &mut i)), fy(read_num(&toks, &mut i)));
                let c2 = (fx(read_num(&toks, &mut i)), fy(read_num(&toks, &mut i)));
                let end = (fx(read_num(&toks, &mut i)), fy(read_num(&toks, &mut i)));
                curves.push(Curve::Bezier(CubicBezier::new(
                    Point2d::from_f64(cur.0, cur.1), Point2d::from_f64(c1.0, c1.1),
                    Point2d::from_f64(c2.0, c2.1), Point2d::from_f64(end.0, end.1))));
                cur = end;
            }
            'A' => {
                let rx = read_num(&toks, &mut i); let _ry = read_num(&toks, &mut i);
                let _rot = read_num(&toks, &mut i);
                let large = read_num(&toks, &mut i) != 0.0;
                let sweep = read_num(&toks, &mut i) != 0.0;
                let end = (fx(read_num(&toks, &mut i)), fy(read_num(&toks, &mut i)));
                if let Some(arc) = svg_arc_to_circular(cur, end, rx, large, sweep) {
                    curves.push(Curve::Arc(arc));
                }
                cur = end;
            }
            'Z' | 'z' => {
                if (cur.0 - start.0).abs() > 1e-12 || (cur.1 - start.1).abs() > 1e-12 {
                    curves.push(Curve::Line(LineSeg::from_endpoints(
                        Point2d::from_f64(cur.0, cur.1), Point2d::from_f64(start.0, start.1))));
                }
                cur = start;
            }
            _ => {}
        }
    }
    curves
}

#[derive(Clone, Debug)]
enum Tok { Cmd(char), Num(f64) }

fn tokenize_path(d: &str) -> Vec<Tok> {
    let mut out = Vec::new();
    let mut chars = d.chars().peekable();
    while let Some(&c) = chars.peek() {
        if c.is_ascii_alphabetic() {
            out.push(Tok::Cmd(c));
            chars.next();
        } else if c.is_ascii_digit() || c == '-' || c == '+' || c == '.' {
            let mut num = String::new();
            while let Some(&c) = chars.peek() {
                if c.is_ascii_digit() || c == '.' || c == 'e' || c == 'E'
                    || ((c == '-' || c == '+') && (num.is_empty() || num.ends_with('e') || num.ends_with('E'))) {
                    num.push(c); chars.next();
                } else { break; }
            }
            if let Ok(v) = num.parse() { out.push(Tok::Num(v)); }
        } else {
            chars.next(); // skip separators
        }
    }
    out
}

fn read_num(toks: &[Tok], i: &mut usize) -> f64 {
    while *i < toks.len() {
        if let Tok::Num(v) = toks[*i] { *i += 1; return v; }
        *i += 1;
    }
    0.0
}

/// Convert an SVG endpoint-arc to a CircularArc (circular only; rx≈ry assumed).
/// In the y-flipped CAD space, recompute center from the two endpoints + radius.
fn svg_arc_to_circular(p0: (f64, f64), p1: (f64, f64), r: f64, large: bool, sweep: bool) -> Option<CircularArc> {
    let (x0, y0) = p0; let (x1, y1) = p1;
    let dx = x1 - x0; let dy = y1 - y0;
    let chord = (dx * dx + dy * dy).sqrt();
    if chord < 1e-12 || r < chord / 2.0 - 1e-9 { return None; }
    let mx = (x0 + x1) / 2.0; let my = (y0 + y1) / 2.0;
    let h = (r * r - (chord / 2.0).powi(2)).max(0.0).sqrt();
    // Perpendicular to chord (unit).
    let (ux, uy) = (-dy / chord, dx / chord);
    // The y-flip swaps the sweep sense; pick center side from flags.
    let side = if large != sweep { 1.0 } else { -1.0 };
    let cx = mx + side * h * ux;
    let cy = my + side * h * uy;
    let start = (y0 - cy).atan2(x0 - cx);
    let end = (y1 - cy).atan2(x1 - cx);
    Some(CircularArc::new(Point2d::from_f64(cx, cy), Rational::from_f64_approx(r), start, end))
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_document::EntityKind;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn export_contains_svg_root_and_line() {
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,10)))));
        let svg = export_svg(&doc);
        assert!(svg.contains("<svg"));
        assert!(svg.contains("<line"));
        assert!(svg.contains("</svg>"));
    }

    #[test]
    fn roundtrip_line_preserves_geometry() {
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(2,3), pt(8,5)))));
        let svg = export_svg(&doc);
        let doc2 = import_svg(&svg);
        assert_eq!(doc2.len(), 1);
        let es: Vec<_> = doc2.iter().collect();
        if let Some(Curve::Line(l)) = es[0].as_curve() {
            // Endpoints preserved (order may vary, check set)
            let p0 = l.p0.to_f64(); let p1 = l.p1.to_f64();
            let ok = (close(p0, (2.0,3.0)) && close(p1, (8.0,5.0)))
                  || (close(p0, (8.0,5.0)) && close(p1, (2.0,3.0)));
            assert!(ok, "got {:?} {:?}", p0, p1);
        } else { panic!() }
    }

    #[test]
    fn roundtrip_circle() {
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Arc(CircularArc::new(pt(5,5), Rational::from(3i64), 0.0, TAU))));
        let svg = export_svg(&doc);
        assert!(svg.contains("<circle"));
        let doc2 = import_svg(&svg);
        let es: Vec<_> = doc2.iter().collect();
        if let Some(Curve::Arc(a)) = es[0].as_curve() {
            assert!((a.center.x.to_f64() - 5.0).abs() < 1e-6);
            assert!((a.center.y.to_f64() - 5.0).abs() < 1e-6);
            assert!((a.radius.to_f64() - 3.0).abs() < 1e-6);
        } else { panic!() }
    }

    #[test]
    fn roundtrip_bezier_native_path() {
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Bezier(CubicBezier::new(pt(0,0), pt(1,3), pt(4,3), pt(5,0)))));
        let svg = export_svg(&doc);
        assert!(svg.contains(" C ")); // native cubic path
        let doc2 = import_svg(&svg);
        let es: Vec<_> = doc2.iter().collect();
        assert!(matches!(es[0].as_curve(), Some(Curve::Bezier(_))));
    }

    #[test]
    fn import_external_path_lines() {
        // A simple triangle drawn by an external tool (no data-h-flip; uses viewBox).
        let svg = "<svg viewBox=\"0 0 10 10\"><path d=\"M 0 0 L 10 0 L 10 10 Z\"/></svg>";
        let doc = import_svg(svg);
        // M,L,L,Z → 3 line segments
        assert_eq!(doc.len(), 3);
    }

    fn close(a: (f64, f64), b: (f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1e-5 && (a.1 - b.1).abs() < 1e-5
    }
}
