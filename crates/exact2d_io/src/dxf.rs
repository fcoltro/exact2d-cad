//! DXF import/export (spec §5.2). ASCII DXF is a flat sequence of
//! (group-code, value) pairs grouped into sections. We support the common 2-D
//! entities and the LAYER table, mapping everything to the exact algebraic kernel.

use exact2d_geometry::{
    Curve, CurveSegment, Point2d, LineSeg, CircularArc, EllipticalArc, PolyCurve,
};
use exact2d_document::{Document, EntityKind, Layer, Color, LineTypeRef};

const TAU: f64 = std::f64::consts::TAU;
const DEG: f64 = std::f64::consts::PI / 180.0;

// ── Tokenizer ─────────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
struct Pair { code: i32, value: String }

fn tokenize(text: &str) -> Vec<Pair> {
    let mut lines = text.lines();
    let mut pairs = Vec::new();
    while let (Some(code_line), Some(val_line)) = (lines.next(), lines.next()) {
        if let Ok(code) = code_line.trim().parse::<i32>() {
            pairs.push(Pair { code, value: val_line.trim().to_string() });
        }
    }
    pairs
}

fn f(p: &Pair) -> f64 { p.value.parse().unwrap_or(0.0) }

// ── Import ────────────────────────────────────────────────────────────────────

/// Parse an ASCII DXF string into a `Document`.
pub fn import_dxf(text: &str) -> Document {
    let pairs = tokenize(text);
    let mut doc = Document::new();
    let mut i = 0;

    while i < pairs.len() {
        if pairs[i].code == 0 && pairs[i].value == "SECTION" {
            // Section name is the next 2-code.
            let name = pairs.get(i + 1).map(|p| p.value.clone()).unwrap_or_default();
            let end = find_endsec(&pairs, i);
            match name.as_str() {
                "TABLES"   => parse_tables(&pairs[i..end], &mut doc),
                "ENTITIES" => parse_entities(&pairs[i..end], &mut doc),
                _ => {}
            }
            i = end;
        } else {
            i += 1;
        }
    }
    doc
}

fn find_endsec(pairs: &[Pair], start: usize) -> usize {
    let mut i = start + 1;
    while i < pairs.len() {
        if pairs[i].code == 0 && pairs[i].value == "ENDSEC" { return i; }
        i += 1;
    }
    pairs.len()
}

/// Split a record run into per-record slices, each starting at a `0/<TYPE>` pair.
fn records(pairs: &[Pair]) -> Vec<&[Pair]> {
    let mut starts = Vec::new();
    for (idx, p) in pairs.iter().enumerate() {
        if p.code == 0 { starts.push(idx); }
    }
    let mut out = Vec::new();
    for w in 0..starts.len() {
        let s = starts[w];
        let e = if w + 1 < starts.len() { starts[w + 1] } else { pairs.len() };
        out.push(&pairs[s..e]);
    }
    out
}

fn parse_tables(pairs: &[Pair], doc: &mut Document) {
    for rec in records(pairs) {
        if rec[0].value != "LAYER" { continue; }
        let mut layer = Layer::new("");
        for p in &rec[1..] {
            match p.code {
                2  => layer.name = p.value.clone(),
                62 => {
                    let aci = p.value.parse::<i32>().unwrap_or(7);
                    if let Color::Rgb(r, g, b) = Color::from_aci(aci.unsigned_abs() as u8) {
                        layer.color = (r, g, b);
                    }
                    if aci < 0 { layer.on = false; } // negative ACI = layer off
                }
                6  => layer.line_type = LineTypeRef::Named(p.value.clone()),
                70 => {
                    let flags = p.value.parse::<i32>().unwrap_or(0);
                    layer.frozen = flags & 1 != 0;
                    layer.locked = flags & 4 != 0;
                }
                _ => {}
            }
        }
        if !layer.name.is_empty() && layer.name != "0" {
            doc.layers.add(layer);
        }
    }
}

fn parse_entities(pairs: &[Pair], doc: &mut Document) {
    for rec in records(pairs) {
        let kind = rec[0].value.as_str();
        if kind == "SECTION" || kind == "ENDSEC" { continue; }
        let layer_name = rec.iter().find(|p| p.code == 8).map(|p| p.value.clone());
        let layer_idx = layer_name
            .and_then(|n| doc.layers.index_of(&n))
            .unwrap_or(0);

        let entities = match kind {
            "LINE"       => parse_line(rec),
            "CIRCLE"     => parse_circle(rec),
            "ARC"        => parse_arc(rec),
            "ELLIPSE"    => parse_ellipse(rec),
            "POINT"      => parse_point(rec),
            "LWPOLYLINE" => parse_lwpolyline(rec),
            "TEXT"       => parse_text(rec),
            _ => vec![],
        };
        for kind in entities {
            doc.add_on_layer(kind, layer_idx);
        }
    }
}

fn get(rec: &[Pair], code: i32) -> Option<f64> {
    rec.iter().find(|p| p.code == code).map(f)
}

fn parse_line(rec: &[Pair]) -> Vec<EntityKind> {
    let (x1, y1) = (get(rec, 10).unwrap_or(0.0), get(rec, 20).unwrap_or(0.0));
    let (x2, y2) = (get(rec, 11).unwrap_or(0.0), get(rec, 21).unwrap_or(0.0));
    vec![EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(
        Point2d::from_f64(x1, y1), Point2d::from_f64(x2, y2))))]
}

fn parse_circle(rec: &[Pair]) -> Vec<EntityKind> {
    let (cx, cy) = (get(rec, 10).unwrap_or(0.0), get(rec, 20).unwrap_or(0.0));
    let r = get(rec, 40).unwrap_or(1.0);
    vec![EntityKind::Curve(Curve::Arc(CircularArc::new(
        Point2d::from_f64(cx, cy), r, 0.0, TAU)))]
}

fn parse_arc(rec: &[Pair]) -> Vec<EntityKind> {
    let (cx, cy) = (get(rec, 10).unwrap_or(0.0), get(rec, 20).unwrap_or(0.0));
    let r = get(rec, 40).unwrap_or(1.0);
    let start = get(rec, 50).unwrap_or(0.0) * DEG;
    let end = get(rec, 51).unwrap_or(360.0) * DEG;
    vec![EntityKind::Curve(Curve::Arc(CircularArc::new(
        Point2d::from_f64(cx, cy), r, start, end)))]
}

fn parse_ellipse(rec: &[Pair]) -> Vec<EntityKind> {
    let (cx, cy) = (get(rec, 10).unwrap_or(0.0), get(rec, 20).unwrap_or(0.0));
    // Major axis endpoint relative to center.
    let (mx, my) = (get(rec, 11).unwrap_or(1.0), get(rec, 21).unwrap_or(0.0));
    let ratio = get(rec, 40).unwrap_or(1.0);
    let start = get(rec, 41).unwrap_or(0.0);
    let end = get(rec, 42).unwrap_or(TAU);
    let major = (mx * mx + my * my).sqrt();
    let rotation = my.atan2(mx);
    vec![EntityKind::Curve(Curve::Ellipse(EllipticalArc::new(
        Point2d::from_f64(cx, cy),
        major,
        major * ratio,
        rotation, start, end)))]
}

fn parse_point(rec: &[Pair]) -> Vec<EntityKind> {
    let (x, y) = (get(rec, 10).unwrap_or(0.0), get(rec, 20).unwrap_or(0.0));
    vec![EntityKind::Point(Point2d::from_f64(x, y))]
}

fn parse_text(rec: &[Pair]) -> Vec<EntityKind> {
    let (x, y) = (get(rec, 10).unwrap_or(0.0), get(rec, 20).unwrap_or(0.0));
    let height = get(rec, 40).unwrap_or(1.0);
    let rotation = get(rec, 50).unwrap_or(0.0) * DEG;
    let content = rec.iter().find(|p| p.code == 1).map(|p| p.value.clone()).unwrap_or_default();
    vec![EntityKind::Text { anchor: Point2d::from_f64(x, y), content, height, rotation }]
}

/// LWPOLYLINE: ordered vertices with optional per-vertex bulge → PolyCurve of
/// line and arc segments. Bulge = tan(included_angle / 4).
fn parse_lwpolyline(rec: &[Pair]) -> Vec<EntityKind> {
    let closed = rec.iter().find(|p| p.code == 70).map(|p| p.value.parse::<i32>().unwrap_or(0) & 1 != 0).unwrap_or(false);

    // Walk in order, collecting (x, y, bulge_to_next).
    let mut verts: Vec<(f64, f64, f64)> = Vec::new();
    let mut cur_x = None;
    let mut cur_bulge = 0.0;
    for p in rec {
        match p.code {
            10 => {
                if let Some(x) = cur_x.take() {
                    // previous vertex had no y? push with 0
                    verts.push((x, 0.0, cur_bulge));
                    cur_bulge = 0.0;
                }
                cur_x = Some(f(p));
            }
            20 => {
                if let Some(x) = cur_x.take() {
                    verts.push((x, f(p), cur_bulge));
                    cur_bulge = 0.0;
                }
            }
            42 => { if let Some(last) = verts.last_mut() { last.2 = f(p); } else { cur_bulge = f(p); } }
            _ => {}
        }
    }

    let n = verts.len();
    if n < 2 { return vec![]; }
    let mut segments: Vec<Curve> = Vec::new();
    let count = if closed { n } else { n - 1 };
    for i in 0..count {
        let (x1, y1, bulge) = verts[i];
        let (x2, y2, _) = verts[(i + 1) % n];
        let p1 = Point2d::from_f64(x1, y1);
        let p2 = Point2d::from_f64(x2, y2);
        if bulge.abs() < 1e-12 {
            segments.push(Curve::Line(LineSeg::from_endpoints(p1, p2)));
        } else {
            segments.push(bulge_arc(x1, y1, x2, y2, bulge));
        }
    }
    vec![EntityKind::Curve(Curve::Poly(Box::new(PolyCurve::new(segments))))]
}

/// Build a CircularArc from a polyline bulge segment.
fn bulge_arc(x1: f64, y1: f64, x2: f64, y2: f64, bulge: f64) -> Curve {
    let theta = 4.0 * bulge.atan();          // signed included angle
    let chord = ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt();
    let radius = (chord / 2.0) / (theta / 2.0).sin().abs();
    // Center is offset from the chord midpoint along the perpendicular.
    let mx = (x1 + x2) / 2.0;
    let my = (y1 + y2) / 2.0;
    let d = (radius.powi(2) - (chord / 2.0).powi(2)).max(0.0).sqrt();
    // Perpendicular direction (unit), sign from bulge.
    let (dx, dy) = ((x2 - x1) / chord, (y2 - y1) / chord);
    let sign = if bulge > 0.0 { 1.0 } else { -1.0 };
    let cx = mx - sign * d * dy;
    let cy = my + sign * d * dx;
    let start = (y1 - cy).atan2(x1 - cx);
    let end = (y2 - cy).atan2(x2 - cx);
    let (start, end) = if bulge > 0.0 { (start, end) } else { (end, start) };
    Curve::Arc(CircularArc::new(Point2d::from_f64(cx, cy), radius, start, end))
}

// ── Export ────────────────────────────────────────────────────────────────────

/// Serialize a `Document` to an ASCII DXF string.
pub fn export_dxf(doc: &Document) -> String {
    let mut s = String::new();
    let mut w = |code: i32, val: &str| { s.push_str(&format!("{}\n{}\n", code, val)); };

    // TABLES → LAYER table
    w(0, "SECTION"); w(2, "TABLES");
    w(0, "TABLE"); w(2, "LAYER");
    for layer in &doc.layers.layers {
        w(0, "LAYER");
        w(2, &layer.name);
        let mut flags = 0;
        if layer.frozen { flags |= 1; }
        if layer.locked { flags |= 4; }
        w(70, &flags.to_string());
        w(62, &aci_for(layer.color, layer.on).to_string());
        if let LineTypeRef::Named(n) = &layer.line_type { w(6, n); }
    }
    w(0, "ENDTAB"); w(0, "ENDSEC");

    // ENTITIES
    w(0, "SECTION"); w(2, "ENTITIES");
    for e in doc.iter() {
        let layer_name = doc.layers.get(e.layer).map(|l| l.name.clone()).unwrap_or_else(|| "0".into());
        write_entity(&mut w, &e.kind, &layer_name);
    }
    w(0, "ENDSEC");
    w(0, "EOF");
    s
}

fn write_entity(w: &mut impl FnMut(i32, &str), kind: &EntityKind, layer: &str) {
    match kind {
        EntityKind::Curve(Curve::Line(l)) => {
            w(0, "LINE"); w(8, layer);
            let (x1, y1) = l.p0.to_f64();
            let (x2, y2) = l.p1.to_f64();
            w(10, &fmt(x1)); w(20, &fmt(y1));
            w(11, &fmt(x2)); w(21, &fmt(y2));
        }
        EntityKind::Curve(Curve::Arc(a)) => {
            let (cx, cy) = a.center.to_f64();
            let span = (a.end_angle - a.start_angle).abs();
            if (span - TAU).abs() < 1e-9 {
                w(0, "CIRCLE"); w(8, layer);
                w(10, &fmt(cx)); w(20, &fmt(cy)); w(40, &fmt(a.radius));
            } else {
                w(0, "ARC"); w(8, layer);
                w(10, &fmt(cx)); w(20, &fmt(cy)); w(40, &fmt(a.radius));
                w(50, &fmt(a.start_angle / DEG)); w(51, &fmt(a.end_angle / DEG));
            }
        }
        EntityKind::Curve(Curve::Ellipse(e)) => {
            let (cx, cy) = e.center.to_f64();
            let major = e.semi_major;
            let mx = major * e.rotation.cos();
            let my = major * e.rotation.sin();
            let ratio = if major.abs() > 1e-12 { e.semi_minor / major } else { 1.0 };
            w(0, "ELLIPSE"); w(8, layer);
            w(10, &fmt(cx)); w(20, &fmt(cy));
            w(11, &fmt(mx)); w(21, &fmt(my));
            w(40, &fmt(ratio));
            w(41, &fmt(e.start_angle)); w(42, &fmt(e.end_angle));
        }
        EntityKind::Curve(Curve::Bezier(b)) => {
            // No native cubic in base DXF here → an adaptive polyline (dense where
            // the curve bends, sparse where it is straight) via unified tessellation.
            let verts = crate::flatten_for_export(&Curve::Bezier(b.clone()));
            w(0, "LWPOLYLINE"); w(8, layer);
            w(90, &verts.len().to_string()); w(70, "0");
            for p in &verts {
                w(10, &fmt(p.x)); w(20, &fmt(p.y));
            }
        }
        EntityKind::Curve(Curve::Rational(rb)) => {
            // No native rational Bézier in base DXF → adaptive tessellated polyline.
            let verts = crate::flatten_for_export(&Curve::Rational(rb.clone()));
            w(0, "LWPOLYLINE"); w(8, layer);
            w(90, &verts.len().to_string()); w(70, "0");
            for p in &verts {
                w(10, &fmt(p.x)); w(20, &fmt(p.y));
            }
        }
        EntityKind::Curve(Curve::Poly(pc)) => {
            write_polyline(w, pc, layer);
        }
        EntityKind::Point(p) => {
            let (x, y) = p.to_f64();
            w(0, "POINT"); w(8, layer); w(10, &fmt(x)); w(20, &fmt(y));
        }
        EntityKind::Text { anchor, content, height, rotation } => {
            let (x, y) = anchor.to_f64();
            w(0, "TEXT"); w(8, layer);
            w(10, &fmt(x)); w(20, &fmt(y)); w(40, &fmt(*height));
            w(1, content); w(50, &fmt(rotation / DEG));
        }
        _ => {} // XLine/Ray/Insert: not emitted in this subset
    }
}

/// Write a PolyCurve as an LWPOLYLINE, converting arc segments to bulges.
fn write_polyline(w: &mut impl FnMut(i32, &str), pc: &PolyCurve, layer: &str) {
    // Build the vertex list (start of each segment + final endpoint).
    let mut verts: Vec<(f64, f64, f64)> = Vec::new(); // (x, y, bulge to next)
    for seg in &pc.segments {
        match seg {
            Curve::Line(l) => {
                let (x, y) = l.p0.to_f64();
                verts.push((x, y, 0.0));
            }
            Curve::Arc(a) => {
                let (sx, sy) = a.start_point();
                let theta = a.included_angle();
                // bulge sign: CCW arc (end>start) positive
                let signed = if a.end_angle >= a.start_angle { theta } else { -theta };
                verts.push((sx, sy, (signed / 4.0).tan()));
            }
            Curve::Rational(_) => {
                // No native rational Bézier in LWPOLYLINE → tessellate to straight
                // vertices (all but the last; the join is the next segment's start).
                let poly = crate::flatten_for_export(seg);
                for p in &poly[..poly.len().saturating_sub(1)] {
                    verts.push((p.x, p.y, 0.0));
                }
            }
            _ => {
                let (x, y) = seg.evaluate_f64(seg.domain().0);
                verts.push((x, y, 0.0));
            }
        }
    }
    // Final endpoint of the last segment.
    if let Some(last) = pc.segments.last() {
        let (ex, ey) = match last {
            Curve::Arc(a) => a.end_point(),
            other => other.evaluate_f64(other.domain().1),
        };
        verts.push((ex, ey, 0.0));
    }

    w(0, "LWPOLYLINE"); w(8, layer);
    w(90, &verts.len().to_string());
    w(70, "0");
    for (x, y, bulge) in &verts {
        w(10, &fmt(*x)); w(20, &fmt(*y));
        if bulge.abs() > 1e-12 { w(42, &fmt(*bulge)); }
    }
}

fn fmt(x: f64) -> String {
    // Compact but precise enough for round-trip.
    format!("{:.9}", x)
}

/// Map an RGB layer color to the nearest ACI index (just the 7 standard ones here).
fn aci_for(rgb: (u8, u8, u8), on: bool) -> i32 {
    let base = match rgb {
        (255, 0, 0) => 1, (255, 255, 0) => 2, (0, 255, 0) => 3,
        (0, 255, 255) => 4, (0, 0, 255) => 5, (255, 0, 255) => 6,
        _ => 7,
    };
    if on { base } else { -base }
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_document::EntityKind;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn import_basic_line() {
        let dxf = "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\n0\n10\n0.0\n20\n0.0\n11\n10.0\n21\n5.0\n0\nENDSEC\n0\nEOF\n";
        let doc = import_dxf(dxf);
        assert_eq!(doc.len(), 1);
        let entities: Vec<_> = doc.iter().collect();
        if let Some(Curve::Line(l)) = entities[0].as_curve() {
            assert!((l.p1.x - 10.0).abs() < 1e-9);
            assert!((l.p1.y - 5.0).abs() < 1e-9);
        } else { panic!("expected line"); }
    }

    #[test]
    fn import_circle_and_arc() {
        let dxf = "0\nSECTION\n2\nENTITIES\n\
                   0\nCIRCLE\n8\n0\n10\n3.0\n20\n4.0\n40\n5.0\n\
                   0\nARC\n8\n0\n10\n0.0\n20\n0.0\n40\n2.0\n50\n0.0\n51\n90.0\n\
                   0\nENDSEC\n0\nEOF\n";
        let doc = import_dxf(dxf);
        assert_eq!(doc.len(), 2);
        let arcs: Vec<_> = doc.iter().filter_map(|e| e.as_curve()).collect();
        if let Curve::Arc(c) = arcs[0] {
            assert!((c.center.x - 3.0).abs() < 1e-9);
            assert!((c.radius - 5.0).abs() < 1e-9);
            assert!((c.included_angle() - TAU).abs() < 1e-6); // full circle
        }
        if let Curve::Arc(a) = arcs[1] {
            assert!((a.included_angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-6); // 90°
        }
    }

    #[test]
    fn roundtrip_line_circle_arc() {
        let mut doc = Document::new();
        doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,5)))));
        doc.add(EntityKind::Curve(Curve::Arc(CircularArc::new(pt(3,4), 5.0, 0.0, TAU))));
        doc.add(EntityKind::Curve(Curve::Arc(CircularArc::new(pt(0,0), 2.0,
            0.0, std::f64::consts::FRAC_PI_2))));

        let dxf = export_dxf(&doc);
        let doc2 = import_dxf(&dxf);
        assert_eq!(doc2.len(), 3);

        // Geometry must survive: compare extents.
        let e1 = doc.extents().unwrap();
        let e2 = doc2.extents().unwrap();
        assert!((e1.min.x - e2.min.x).abs() < 1e-6);
        assert!((e1.max.x - e2.max.x).abs() < 1e-6);
        assert!((e1.max.y - e2.max.y).abs() < 1e-6);
    }

    #[test]
    fn layer_table_roundtrip() {
        let mut doc = Document::new();
        doc.layers.add(Layer::new("walls").with_color(255, 0, 0));
        let mut frozen = Layer::new("hidden");
        frozen.frozen = true;
        doc.layers.add(frozen);

        let dxf = export_dxf(&doc);
        let doc2 = import_dxf(&dxf);
        assert!(doc2.layers.index_of("walls").is_some());
        let w = doc2.layers.get(doc2.layers.index_of("walls").unwrap()).unwrap();
        assert_eq!(w.color, (255, 0, 0));
        let h = doc2.layers.get(doc2.layers.index_of("hidden").unwrap()).unwrap();
        assert!(h.frozen);
    }

    #[test]
    fn lwpolyline_with_bulge_roundtrip() {
        // A closed polyline: line then a semicircle arc bulge.
        // Square corner (0,0)->(4,0), bulge arc to (4,4), line back...
        let dxf = "0\nSECTION\n2\nENTITIES\n\
                   0\nLWPOLYLINE\n8\n0\n90\n3\n70\n0\n\
                   10\n0.0\n20\n0.0\n\
                   10\n4.0\n20\n0.0\n42\n1.0\n\
                   10\n4.0\n20\n4.0\n\
                   0\nENDSEC\n0\nEOF\n";
        let doc = import_dxf(dxf);
        assert_eq!(doc.len(), 1);
        let entities: Vec<_> = doc.iter().collect();
        if let Some(Curve::Poly(pc)) = entities[0].as_curve() {
            assert_eq!(pc.segments.len(), 2);
            // Second segment is the bulge arc (bulge=1 → half circle)
            assert!(matches!(pc.segments[1], Curve::Arc(_)));
        } else { panic!("expected polyline"); }
    }

    #[test]
    fn import_entity_layer_assignment() {
        let dxf = "0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n\
                   0\nLAYER\n2\nred\n62\n1\n70\n0\n0\nENDTAB\n0\nENDSEC\n\
                   0\nSECTION\n2\nENTITIES\n\
                   0\nLINE\n8\nred\n10\n0.0\n20\n0.0\n11\n1.0\n21\n1.0\n\
                   0\nENDSEC\n0\nEOF\n";
        let doc = import_dxf(dxf);
        let red_idx = doc.layers.index_of("red").unwrap();
        let e = doc.iter().next().unwrap();
        assert_eq!(e.layer, red_idx);
    }
}
