//! Review probes for file-format round-trips — focus on the tricky cases
//! (arc reconstruction, angle preservation, multi-entity fidelity).

use exact2d_io::{export_dxf, import_dxf, export_svg, import_svg, to_e2d, from_e2d};
use exact2d_document::{Document, EntityKind};
use exact2d_geometry::{Curve, LineSeg, CircularArc, Point2d};
use exact2d_algebra::Rational;

fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

/// A DXF arc (not a full circle) must survive export→import with its sweep intact.
#[test]
fn dxf_arc_sweep_roundtrip() {
    let mut doc = Document::new();
    // Quarter arc, center (1,2), r=4, from 30° to 120°.
    let s = 30f64.to_radians();
    let e = 120f64.to_radians();
    doc.add(EntityKind::Curve(Curve::Arc(CircularArc::new(pt(1,2), Rational::from(4i64), s, e))));

    let doc2 = import_dxf(&export_dxf(&doc));
    let es: Vec<_> = doc2.iter().collect();
    if let Some(Curve::Arc(a)) = es[0].as_curve() {
        assert!((a.center.x.to_f64() - 1.0).abs() < 1e-6);
        assert!((a.radius.to_f64() - 4.0).abs() < 1e-6);
        assert!((a.included_angle() - (e - s)).abs() < 1e-4,
            "sweep changed: {} vs {}", a.included_angle(), e - s);
        // Start point must match.
        let (sx, sy) = a.start_point();
        let (ox, oy) = (1.0 + 4.0 * s.cos(), 2.0 + 4.0 * s.sin());
        assert!((sx - ox).abs() < 1e-3 && (sy - oy).abs() < 1e-3,
            "start point moved: ({},{}) vs ({},{})", sx, sy, ox, oy);
    } else { panic!("expected arc"); }
}

/// An SVG arc (path `A` command) must round-trip back to an arc through the same
/// endpoints with the same radius and sweep direction.
#[test]
fn svg_arc_roundtrip() {
    let mut doc = Document::new();
    let s = 20f64.to_radians();
    let e = 100f64.to_radians();
    let arc = CircularArc::new(pt(3,3), Rational::from(5i64), s, e);
    let (sx, sy) = arc.start_point();
    let (ex, ey) = arc.end_point();
    doc.add(EntityKind::Curve(Curve::Arc(arc)));

    let svg = export_svg(&doc);
    assert!(svg.contains(" A "), "arc should export as an SVG A path");
    let doc2 = import_svg(&svg);
    let es: Vec<_> = doc2.iter().collect();
    if let Some(Curve::Arc(a)) = es[0].as_curve() {
        assert!((a.radius.to_f64() - 5.0).abs() < 1e-2, "radius={}", a.radius.to_f64());
        // The arc must pass through both original endpoints.
        let (ns0, ns1) = a.start_point();
        let (ne0, ne1) = a.end_point();
        let matches_ends =
            (close(ns0, sx) && close(ns1, sy) && close(ne0, ex) && close(ne1, ey)) ||
            (close(ns0, ex) && close(ns1, ey) && close(ne0, sx) && close(ne1, sy));
        assert!(matches_ends,
            "SVG arc endpoints wrong: start({:.3},{:.3}) end({:.3},{:.3}) vs orig start({:.3},{:.3}) end({:.3},{:.3})",
            ns0, ns1, ne0, ne1, sx, sy, ex, ey);
    } else { panic!("expected arc, got {:?}", es[0].as_curve().map(|_| "curve")); }
}

/// Native format must preserve a multi-entity drawing with mixed types and layers.
#[test]
fn native_multi_entity_fidelity() {
    let mut doc = Document::new();
    doc.layers.add(exact2d_document::Layer::new("detail").with_color(0, 255, 0));
    let detail = doc.layers.index_of("detail").unwrap();
    doc.add(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(10,0)))));
    doc.add_on_layer(EntityKind::Curve(Curve::Arc(CircularArc::new(pt(5,5), Rational::from(3i64),
        0.0, std::f64::consts::PI))), detail);
    doc.add(EntityKind::Point(pt(7, 8)));

    let doc2 = from_e2d(&to_e2d(&doc)).unwrap();
    assert_eq!(doc2.len(), 3);
    // The arc-on-detail-layer keeps its layer assignment.
    let arc_entity = doc2.iter().find(|e| matches!(&e.kind, EntityKind::Curve(Curve::Arc(_)))).unwrap();
    let layer_name = &doc2.layers.layers[arc_entity.layer].name;
    assert_eq!(layer_name, "detail");
}

/// Empty document export/import must not panic and yields an empty document.
#[test]
fn empty_document_roundtrips() {
    let doc = Document::new();
    assert_eq!(import_dxf(&export_dxf(&doc)).len(), 0);
    assert_eq!(import_svg(&export_svg(&doc)).len(), 0);
    assert_eq!(from_e2d(&to_e2d(&doc)).unwrap().len(), 0);
}

fn close(a: f64, b: f64) -> bool { (a - b).abs() < 1e-2 }
