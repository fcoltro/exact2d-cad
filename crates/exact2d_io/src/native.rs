//! Native `.e2d` document format (spec §5.1) — a self-contained, human-readable,
//! versioned text format. Coordinates are f64, written with Rust's shortest
//! round-trippable formatting, so geometry round-trips bit-exactly. Saves are
//! atomic (write to a temp file, then rename).
//!
//! Pure Rust, no external database engine. The line-based grammar keeps the file
//! easy to query/repair by hand, which covers the spec's robustness goals.
//! Older files that stored exact rationals as `num/den` still load (parsed as f64).

use std::io::Write;
use exact2d_geometry::{Curve, Point2d, LineSeg, CircularArc, EllipticalArc, CubicBezier, PolyCurve, RationalBezier, NurbsCurve};
use exact2d_document::{Document, EntityKind, Entity, Layer, Color, LineTypeRef, LineTypeDef, Units};

const TAU: f64 = std::f64::consts::TAU;
pub const MAGIC: &str = "E2D";
pub const VERSION: u32 = 1;

// ── Serialize ─────────────────────────────────────────────────────────────────

/// Serialize a document to the `.e2d` text format.
pub fn to_string(doc: &Document) -> String {
    let mut s = String::new();
    s.push_str(&format!("{} {}\n", MAGIC, VERSION));
    s.push_str(&format!("UNITS {}\n", units_name(doc.settings.units)));
    s.push_str(&format!("GRID {}\n", doc.settings.grid_spacing));
    s.push_str(&format!("SNAP {}\n", doc.settings.snap_spacing));

    for lt in &doc.line_types {
        let pat: Vec<String> = lt.pattern.iter().map(|p| p.to_string()).collect();
        s.push_str(&format!("LT {} {}\n", esc(&lt.name), pat.join(",")));
    }
    for l in &doc.layers.layers {
        s.push_str(&format!("LAYER {} {},{},{} {} {} {} {}\n",
            esc(&l.name), l.color.0, l.color.1, l.color.2,
            l.on as u8, l.frozen as u8, l.locked as u8,
            esc(&linetype_name(&l.line_type))));
    }
    for e in doc.iter() {
        write_entity(&mut s, e);
    }
    s
}

/// Save to a file atomically (write temp, then rename over the target).
pub fn save(doc: &Document, path: &std::path::Path) -> std::io::Result<()> {
    let data = to_string(doc);
    let tmp = path.with_extension("e2d.tmp");
    {
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(data.as_bytes())?;
        f.sync_all()?;
    }
    std::fs::rename(&tmp, path)
}

fn write_entity(s: &mut String, e: &Entity) {
    let layer = e.layer;
    let color = color_str(&e.color);
    match &e.kind {
        EntityKind::Curve(Curve::Line(l)) =>
            s.push_str(&format!("E LINE {} {} {} {}\n", layer, color, pt(&l.p0), pt(&l.p1))),
        EntityKind::Curve(Curve::Arc(a)) => {
            let (cx, cy) = (rat(a.center.x), rat(a.center.y));
            s.push_str(&format!("E ARC {} {} {};{} {} {} {}\n",
                layer, color, cx, cy, rat(a.radius), a.start_angle, a.end_angle));
        }
        EntityKind::Curve(Curve::Ellipse(el)) =>
            s.push_str(&format!("E ELLIPSE {} {} {};{} {} {} {} {} {}\n",
                layer, color, rat(el.center.x), rat(el.center.y),
                rat(el.semi_major), rat(el.semi_minor), el.rotation, el.start_angle, el.end_angle)),
        EntityKind::Curve(Curve::Bezier(b)) =>
            s.push_str(&format!("E BEZIER {} {} {} {} {} {}\n",
                layer, color, pt(&b.p0), pt(&b.p1), pt(&b.p2), pt(&b.p3))),
        EntityKind::Curve(Curve::Rational(rb)) =>
            s.push_str(&format!("E RATIONAL {} {} {}\n", layer, color, control_fields(&rb.points, &rb.weights))),
        EntityKind::Curve(Curve::Nurbs(nc)) =>
            s.push_str(&format!("E NURBS {} {} {}\n", layer, color, control_fields(&nc.control, &nc.weights))),
        EntityKind::Curve(Curve::Poly(pc)) => {
            s.push_str(&format!("E POLY {} {} {}\n", layer, color, pc.segments.len()));
            for seg in &pc.segments { write_segment(s, seg); }
        }
        EntityKind::Point(p) =>
            s.push_str(&format!("E POINT {} {} {}\n", layer, color, pt(p))),
        EntityKind::Text { anchor, content, height, rotation } =>
            s.push_str(&format!("E TEXT {} {} {} {} {} {}\n",
                layer, color, pt(anchor), height, rotation, esc(content))),
        _ => {} // XLine/Ray/Insert: not persisted in this subset
    }
}

fn write_segment(s: &mut String, seg: &Curve) {
    match seg {
        Curve::Line(l) => s.push_str(&format!("SEG LINE {} {}\n", pt(&l.p0), pt(&l.p1))),
        Curve::Arc(a) => s.push_str(&format!("SEG ARC {};{} {} {} {}\n",
            rat(a.center.x), rat(a.center.y), rat(a.radius), a.start_angle, a.end_angle)),
        Curve::Bezier(b) => s.push_str(&format!("SEG BEZIER {} {} {} {}\n",
            pt(&b.p0), pt(&b.p1), pt(&b.p2), pt(&b.p3))),
        Curve::Rational(rb) => s.push_str(&format!("SEG RATIONAL {}\n", control_fields(&rb.points, &rb.weights))),
        _ => s.push_str("SEG LINE 0;0 0;0\n"),
    }
}

/// Serialize control data: `n p0 w0 p1 w1 … p(n-1) w(n-1)` (rational Béziers + NURBS).
fn control_fields(points: &[Point2d], weights: &[f64]) -> String {
    let mut out = points.len().to_string();
    for (p, w) in points.iter().zip(weights) {
        out.push_str(&format!(" {} {}", pt(p), rat(*w)));
    }
    out
}

// ── Deserialize ───────────────────────────────────────────────────────────────

/// Parse a document from `.e2d` text. Returns `Err` on a missing/incompatible header.
pub fn from_string(text: &str) -> Result<Document, String> {
    let mut lines = text.lines().peekable();
    let header = lines.next().ok_or("empty file")?;
    let mut hp = header.split_whitespace();
    if hp.next() != Some(MAGIC) { return Err("not an E2D file".into()); }
    let ver: u32 = hp.next().and_then(|v| v.parse().ok()).ok_or("missing version")?;
    if ver > VERSION { return Err(format!("unsupported version {}", ver)); }

    let mut doc = Document::new();
    // Replace the default layer table; we re-add layers from the file.
    doc.layers.layers.clear();
    doc.line_types.clear();

    while let Some(line) = lines.next() {
        let line = line.trim();
        if line.is_empty() { continue; }
        let mut tok = line.split_whitespace();
        match tok.next() {
            Some("UNITS") => doc.settings.units = parse_units(tok.next().unwrap_or("")),
            Some("GRID")  => doc.settings.grid_spacing = tok.next().and_then(|v| v.parse().ok()).unwrap_or(10.0),
            Some("SNAP")  => doc.settings.snap_spacing = tok.next().and_then(|v| v.parse().ok()).unwrap_or(1.0),
            Some("LT")    => { if let Some(lt) = parse_lt(&mut tok) { doc.line_types.push(lt); } }
            Some("LAYER") => { if let Some(l) = parse_layer(&mut tok) { doc.layers.layers.push(l); } }
            Some("E")     => { parse_entity(&mut tok, &mut lines, &mut doc); }
            _ => {}
        }
    }

    if doc.layers.layers.is_empty() { doc.layers.layers.push(Layer::new("0")); }
    Ok(doc)
}

/// Load from a file.
pub fn load(path: &std::path::Path) -> std::io::Result<Document> {
    let text = std::fs::read_to_string(path)?;
    from_string(&text).map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
}

fn parse_entity<'a>(
    tok: &mut impl Iterator<Item = &'a str>,
    lines: &mut std::iter::Peekable<std::str::Lines>,
    doc: &mut Document,
) {
    let etype = match tok.next() { Some(t) => t, None => return };
    let layer: usize = tok.next().and_then(|v| v.parse().ok()).unwrap_or(0);
    let color = parse_color(tok.next().unwrap_or("bylayer"));

    let kind = match etype {
        "LINE" => {
            let p0 = parse_pt(tok.next()); let p1 = parse_pt(tok.next());
            Some(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(p0, p1))))
        }
        "ARC" => {
            let c = parse_pt(tok.next());
            let r = parse_num(tok.next().unwrap_or("1"));
            let start = tok.next().and_then(|v| v.parse().ok()).unwrap_or(0.0);
            let end = tok.next().and_then(|v| v.parse().ok()).unwrap_or(TAU);
            Some(EntityKind::Curve(Curve::Arc(CircularArc::new(c, r, start, end))))
        }
        "ELLIPSE" => {
            let c = parse_pt(tok.next());
            let major = parse_num(tok.next().unwrap_or("1"));
            let minor = parse_num(tok.next().unwrap_or("1"));
            let rot = tok.next().and_then(|v| v.parse().ok()).unwrap_or(0.0);
            let start = tok.next().and_then(|v| v.parse().ok()).unwrap_or(0.0);
            let end = tok.next().and_then(|v| v.parse().ok()).unwrap_or(TAU);
            Some(EntityKind::Curve(Curve::Ellipse(EllipticalArc::new(c, major, minor, rot, start, end))))
        }
        "BEZIER" => {
            let p0 = parse_pt(tok.next()); let p1 = parse_pt(tok.next());
            let p2 = parse_pt(tok.next()); let p3 = parse_pt(tok.next());
            Some(EntityKind::Curve(Curve::Bezier(CubicBezier::new(p0, p1, p2, p3))))
        }
        "RATIONAL" => parse_control_data(tok)
            .map(|(p, w)| EntityKind::Curve(Curve::Rational(RationalBezier::new(p, w)))),
        "NURBS" => parse_control_data(tok)
            .map(|(c, w)| EntityKind::Curve(Curve::Nurbs(NurbsCurve::new(c, w)))),
        "POINT" => Some(EntityKind::Point(parse_pt(tok.next()))),
        "TEXT" => {
            let anchor = parse_pt(tok.next());
            let height = tok.next().and_then(|v| v.parse().ok()).unwrap_or(1.0);
            let rotation = tok.next().and_then(|v| v.parse().ok()).unwrap_or(0.0);
            let content = unesc(tok.next().unwrap_or(""));
            Some(EntityKind::Text { anchor, content, height, rotation })
        }
        "POLY" => {
            let count: usize = tok.next().and_then(|v| v.parse().ok()).unwrap_or(0);
            let mut segs = Vec::new();
            for _ in 0..count {
                if let Some(segline) = lines.next() {
                    if let Some(seg) = parse_segment(segline.trim()) { segs.push(seg); }
                }
            }
            Some(EntityKind::Curve(Curve::Poly(Box::new(PolyCurve::new(segs)))))
        }
        _ => None,
    };

    if let Some(k) = kind {
        let id = doc.add_on_layer(k, layer.min(doc.layers.layers.len().saturating_sub(1)));
        if let Some(e) = doc.get_mut(id) { e.color = color; }
    }
}

fn parse_segment(line: &str) -> Option<Curve> {
    let mut tok = line.split_whitespace();
    if tok.next() != Some("SEG") { return None; }
    match tok.next()? {
        "LINE" => Some(Curve::Line(LineSeg::from_endpoints(parse_pt(tok.next()), parse_pt(tok.next())))),
        "ARC" => {
            let c = parse_pt(tok.next());
            let r = parse_num(tok.next().unwrap_or("1"));
            let start = tok.next().and_then(|v| v.parse().ok()).unwrap_or(0.0);
            let end = tok.next().and_then(|v| v.parse().ok()).unwrap_or(TAU);
            Some(Curve::Arc(CircularArc::new(c, r, start, end)))
        }
        "BEZIER" => Some(Curve::Bezier(CubicBezier::new(
            parse_pt(tok.next()), parse_pt(tok.next()), parse_pt(tok.next()), parse_pt(tok.next())))),
        "RATIONAL" => parse_control_data(&mut tok)
            .map(|(p, w)| Curve::Rational(RationalBezier::new(p, w))),
        _ => None,
    }
}

/// Parse `n p0 w0 p1 w1 … p(n-1) w(n-1)` into a `RationalBezier`; `None` if the
/// control data is malformed (fewer than 2 points or a non-positive weight).
/// Parse `n p0 w0 p1 w1 … p(n-1) w(n-1)` into `(control points, weights)`; `None`
/// if malformed (fewer than 2 points or a non-positive weight). Shared by the
/// rational-Bézier and NURBS records.
fn parse_control_data<'a, I: Iterator<Item = &'a str>>(tok: &mut I) -> Option<(Vec<Point2d>, Vec<f64>)> {
    let n: usize = tok.next().and_then(|v| v.parse().ok())?;
    let mut points = Vec::with_capacity(n);
    let mut weights = Vec::with_capacity(n);
    for _ in 0..n {
        points.push(parse_pt(tok.next()));
        weights.push(parse_num(tok.next().unwrap_or("1")));
    }
    (points.len() >= 2 && points.len() == weights.len() && weights.iter().all(|&w| w > 0.0))
        .then_some((points, weights))
}

// ── Field helpers ─────────────────────────────────────────────────────────────

/// Format an f64 with Rust's shortest round-trippable representation.
fn rat(v: f64) -> String { format!("{}", v) }
fn pt(p: &Point2d) -> String { format!("{};{}", rat(p.x), rat(p.y)) }

/// Parse a coordinate. Accepts plain f64 and (for back-compat) old `num/den` rationals.
fn parse_num(s: &str) -> f64 {
    if let Some((n, d)) = s.split_once('/') {
        let n: f64 = n.parse().unwrap_or(0.0);
        let d: f64 = d.parse().unwrap_or(1.0);
        if d != 0.0 { n / d } else { 0.0 }
    } else {
        s.parse().unwrap_or(0.0)
    }
}

fn parse_pt(s: Option<&str>) -> Point2d {
    let s = s.unwrap_or("0;0");
    let (x, y) = s.split_once(';').unwrap_or(("0", "0"));
    Point2d::new(parse_num(x), parse_num(y))
}

fn color_str(c: &Color) -> String {
    match c {
        Color::ByLayer => "bylayer".into(),
        Color::ByBlock => "byblock".into(),
        Color::Rgb(r, g, b) => format!("rgb:{}:{}:{}", r, g, b),
    }
}
fn parse_color(s: &str) -> Color {
    match s {
        "bylayer" => Color::ByLayer,
        "byblock" => Color::ByBlock,
        other => {
            if let Some(rest) = other.strip_prefix("rgb:") {
                let p: Vec<u8> = rest.split(':').filter_map(|v| v.parse().ok()).collect();
                if p.len() == 3 { return Color::Rgb(p[0], p[1], p[2]); }
            }
            Color::ByLayer
        }
    }
}

fn parse_layer<'a>(tok: &mut impl Iterator<Item = &'a str>) -> Option<Layer> {
    let name = unesc(tok.next()?);
    let rgb: Vec<u8> = tok.next()?.split(',').filter_map(|v| v.parse().ok()).collect();
    let on = tok.next()? == "1";
    let frozen = tok.next()? == "1";
    let locked = tok.next()? == "1";
    let lt = unesc(tok.next().unwrap_or("Continuous"));
    let mut l = Layer::new(name);
    if rgb.len() == 3 { l.color = (rgb[0], rgb[1], rgb[2]); }
    l.on = on; l.frozen = frozen; l.locked = locked;
    l.line_type = LineTypeRef::Named(lt);
    Some(l)
}

fn parse_lt<'a>(tok: &mut impl Iterator<Item = &'a str>) -> Option<LineTypeDef> {
    let name = unesc(tok.next()?);
    let pat: Vec<f64> = tok.next().map(|s| s.split(',').filter_map(|v| v.parse().ok()).collect()).unwrap_or_default();
    Some(LineTypeDef { name, description: String::new(), pattern: pat })
}

fn linetype_name(lt: &LineTypeRef) -> String {
    match lt {
        LineTypeRef::Named(n) => n.clone(),
        LineTypeRef::ByLayer => "ByLayer".into(),
        LineTypeRef::ByBlock => "ByBlock".into(),
    }
}

fn units_name(u: Units) -> &'static str {
    match u {
        Units::Unitless => "Unitless", Units::Millimeters => "Millimeters",
        Units::Centimeters => "Centimeters", Units::Meters => "Meters",
        Units::Kilometers => "Kilometers", Units::Inches => "Inches", Units::Feet => "Feet",
    }
}
fn parse_units(s: &str) -> Units {
    match s {
        "Centimeters" => Units::Centimeters, "Meters" => Units::Meters,
        "Kilometers" => Units::Kilometers,
        "Inches" => Units::Inches, "Feet" => Units::Feet, "Unitless" => Units::Unitless,
        _ => Units::Millimeters,
    }
}

/// Escape spaces (and the escape char) so a value occupies a single token.
fn esc(s: &str) -> String {
    if s.is_empty() { return "_".into(); }
    s.replace('\\', "\\\\").replace(' ', "\\s")
}
fn unesc(s: &str) -> String {
    if s == "_" { return String::new(); }
    s.replace("\\s", " ").replace("\\\\", "\\")
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_document::EntityKind;

    fn pt_i(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn roundtrip_f64_is_lossless() {
        let mut doc = Document::new();
        // 1/3 as f64 — shortest round-trip formatting must reproduce the exact bits.
        let third = 1.0 / 3.0;
        doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(
            Point2d::new(third, 0.0),
            Point2d::new(2.0, third)))));

        let text = to_string(&doc);
        let doc2 = from_string(&text).unwrap();
        let es: Vec<_> = doc2.iter().collect();
        if let Some(Curve::Line(l)) = es[0].as_curve() {
            assert_eq!(l.p0.x, third);              // bit-exact round-trip
            assert_eq!(l.p1.y, third);
        } else { panic!() }
    }

    #[test]
    fn roundtrip_layers_and_settings() {
        let mut doc = Document::new();
        doc.settings.units = Units::Inches;
        doc.layers.add(Layer::new("walls").with_color(255, 0, 0));
        let mut frozen = Layer::new("hidden"); frozen.frozen = true;
        doc.layers.add(frozen);

        let doc2 = from_string(&to_string(&doc)).unwrap();
        assert_eq!(doc2.settings.units, Units::Inches);
        let w = doc2.layers.get(doc2.layers.index_of("walls").unwrap()).unwrap();
        assert_eq!(w.color, (255, 0, 0));
        assert!(doc2.layers.get(doc2.layers.index_of("hidden").unwrap()).unwrap().frozen);
    }

    #[test]
    fn roundtrip_all_entity_types() {
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt_i(0,0), pt_i(5,5)))));
        doc.add(EntityKind::Curve(Curve::Arc(CircularArc::new(pt_i(3,4), 5.0, 0.0, TAU))));
        doc.add(EntityKind::Curve(Curve::Bezier(CubicBezier::new(pt_i(0,0), pt_i(1,2), pt_i(3,2), pt_i(4,0)))));
        doc.add(EntityKind::Point(pt_i(7, 8)));
        doc.add(EntityKind::Text { anchor: pt_i(1,1), content: "hello world".into(), height: 2.5, rotation: 0.0 });

        let doc2 = from_string(&to_string(&doc)).unwrap();
        assert_eq!(doc2.len(), 5);
        // Text content with a space survives escaping.
        let has_text = doc2.iter().any(|e| matches!(&e.kind, EntityKind::Text { content, .. } if content == "hello world"));
        assert!(has_text);
    }

    #[test]
    fn roundtrip_rational_is_lossless() {
        // A weighted rational Bézier (mixed weights) must round-trip bit-exactly:
        // control points and weights both survive (it is the authored NURBS form).
        let mut doc = Document::new();
        let rb = RationalBezier::new(
            vec![pt_i(0, 0), pt_i(2, 4), pt_i(6, 4), pt_i(8, 0)],
            vec![1.0, 2.0, 0.5, 1.0],
        );
        doc.add(EntityKind::Curve(Curve::Rational(rb.clone())));

        let doc2 = from_string(&to_string(&doc)).unwrap();
        let e = doc2.iter().next().expect("one entity");
        if let EntityKind::Curve(Curve::Rational(r2)) = &e.kind {
            assert_eq!(r2.points, rb.points, "control points must survive exactly");
            assert_eq!(r2.weights, rb.weights, "weights must survive exactly");
        } else {
            panic!("expected a Rational curve after round-trip");
        }
    }

    #[test]
    fn roundtrip_polycurve_of_rational_segments() {
        // The CV-spline B-spline output (a PolyCurve of rational segments) must
        // round-trip via the SEG RATIONAL records.
        let seg = || RationalBezier::new(
            vec![pt_i(0, 0), pt_i(1, 2), pt_i(3, 2), pt_i(4, 0)], vec![1.0, 1.0, 1.0, 1.0]);
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Poly(Box::new(
            PolyCurve::new(vec![Curve::Rational(seg()), Curve::Rational(seg())])))));

        let doc2 = from_string(&to_string(&doc)).unwrap();
        let e = doc2.iter().next().expect("one entity");
        if let EntityKind::Curve(Curve::Poly(pc)) = &e.kind {
            assert_eq!(pc.segments.len(), 2);
            assert!(pc.segments.iter().all(|s| matches!(s, Curve::Rational(_))));
        } else {
            panic!("expected a PolyCurve of rational segments");
        }
    }

    #[test]
    fn roundtrip_nurbs_is_lossless() {
        // An editable NURBS spline (control vertices + non-uniform weights) must
        // round-trip bit-exactly via the E NURBS record.
        let mut doc = Document::new();
        let nc = NurbsCurve::new(
            vec![pt_i(0, 0), pt_i(2, 5), pt_i(6, 5), pt_i(9, 0), pt_i(12, 4)],
            vec![1.0, 2.0, 0.5, 1.0, 3.0],
        );
        doc.add(EntityKind::Curve(Curve::Nurbs(nc.clone())));

        let doc2 = from_string(&to_string(&doc)).unwrap();
        let e = doc2.iter().next().expect("one entity");
        if let EntityKind::Curve(Curve::Nurbs(n2)) = &e.kind {
            assert_eq!(n2.control, nc.control, "control vertices must survive exactly");
            assert_eq!(n2.weights, nc.weights, "weights must survive exactly");
        } else {
            panic!("expected a NURBS curve after round-trip");
        }
    }

    #[test]
    fn roundtrip_polycurve() {
        let mut doc = Document::new();
        let segs = vec![
            Curve::Line(LineSeg::from_endpoints(pt_i(0,0), pt_i(4,0))),
            Curve::Arc(CircularArc::new(pt_i(4,2), 2.0, -std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2)),
        ];
        doc.add(EntityKind::Curve(Curve::Poly(Box::new(PolyCurve::new(segs)))));
        let doc2 = from_string(&to_string(&doc)).unwrap();
        let es: Vec<_> = doc2.iter().collect();
        if let Some(Curve::Poly(pc)) = es[0].as_curve() {
            assert_eq!(pc.segments.len(), 2);
        } else { panic!() }
    }

    #[test]
    fn rejects_bad_header() {
        assert!(from_string("NOPE 1\n").is_err());
        assert!(from_string("").is_err());
    }

    #[test]
    fn save_and_load_file_atomic() {
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt_i(1,2), pt_i(3,4)))));
        let dir = std::env::temp_dir();
        let path = dir.join("exact2d_native_test.e2d");
        save(&doc, &path).unwrap();
        assert!(path.exists());
        let doc2 = load(&path).unwrap();
        assert_eq!(doc2.len(), 1);
        let _ = std::fs::remove_file(&path);
    }
}
