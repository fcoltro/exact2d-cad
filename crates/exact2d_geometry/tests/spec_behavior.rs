//! Behavioral probe tests — verify the geometry engine does what the Phase 2
//! spec actually requires, end-to-end, beyond the per-module unit tests.
//!
//! These are intentionally adversarial: they probe accuracy, general (non-fast-path)
//! code, and combinations that the unit tests don't cover.

use exact2d_geometry::*;

fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

// ── §2.2 Intersection: general resultant path (Bézier × Bézier) ───────────────

#[test]
fn bezier_bezier_intersection_via_resultant() {
    // Two cubic Béziers that visibly cross near the middle.
    // Arch up:  (0,0) → (3,3) → (6,3) → (9,0)   (peaks around y≈2.25)
    // Arch down:(0,3) → (3,0) → (6,0) → (9,3)   (dips around y≈0.75)
    // They must cross twice.
    let up = Curve::Bezier(CubicBezier::new(pt(0,0), pt(3,3), pt(6,3), pt(9,0)));
    let down = Curve::Bezier(CubicBezier::new(pt(0,3), pt(3,0), pt(6,0), pt(9,3)));

    let hits = intersect(&up, &down);
    // Expect at least the 2 crossings
    assert!(hits.len() >= 2, "Bézier×Bézier expected ≥2 intersections, got {}", hits.len());

    // Each reported point must lie on BOTH curves (verify via implicit forms)
    let fu = up.implicit_form();
    let fd = down.implicit_form();
    for h in &hits {
        let (x, y) = h.point;
        assert!(fu.eval_f64(x, y).abs() < 1.0, "pt not on up-curve: f={}", fu.eval_f64(x,y));
        assert!(fd.eval_f64(x, y).abs() < 1.0, "pt not on down-curve: f={}", fd.eval_f64(x,y));
    }
}

// ── §2.2 Line-circle accuracy at a tangent (single intersection) ──────────────

#[test]
fn line_tangent_to_circle_gives_one_point() {
    // Circle radius 5 at origin; horizontal line y=5 is tangent at (0,5).
    let circle = CircularArc::new(pt(0,0), 5.0,
        0.0, 2.0*std::f64::consts::PI);
    let line = LineSeg::from_endpoints(pt(-8, 5), pt(8, 5));
    let hits = intersect_line_circle(&line, &circle);
    // Tangent → exactly 1 point (within tolerance). Accept 1 or a degenerate 2 very close.
    assert!(!hits.is_empty(), "tangent line should touch the circle");
    for h in &hits {
        assert!((h.point.0).abs() < 1e-3, "tangent x≈0, got {}", h.point.0);
        assert!((h.point.1 - 5.0).abs() < 1e-3, "tangent y≈5, got {}", h.point.1);
    }
}

// ── §2.2 Curvature sign/magnitude on a known curve ────────────────────────────

#[test]
fn curvature_of_parabola_at_vertex() {
    // Parabola y = x²  ⟺  implicit y - x² = 0. At vertex (0,0): κ = 2 (well known).
    // Build it as a Bézier approximation won't be exact; use the implicit directly via
    // a cubic Bézier that traces y=x² over a small range and check curvature near vertex.
    // Control points for y=x² on [-1,1]: P0=(-1,1) P1=(-1/3,-1/3) P2=(1/3,-1/3) P3=(1,1)
    let para = Curve::Bezier(CubicBezier::new(
        Point2d::new(-1.0, 1.0),
        Point2d::new(-1.0/3.0, -1.0/3.0),
        Point2d::new(1.0/3.0,  -1.0/3.0),
        Point2d::new(1.0, 1.0),
    ));
    // At t=0.5 the Bézier is at the vertex (0, ~-? ) — check curvature is finite & nonzero
    let k = curvature_at(&para, 0.5);
    assert!(k.is_some(), "curvature should be defined at vertex");
    let kv = k.unwrap();
    assert!(kv.abs() > 0.1, "vertex curvature should be substantial, got {}", kv);
}

// ── §2.2 Offset correctness: offset of circle is concentric ───────────────────

#[test]
fn offset_circle_is_concentric_and_correct_radius() {
    let circle = Curve::Arc(CircularArc::new(pt(10, 20), 7.0,
        0.0, 2.0*std::f64::consts::PI));
    let outer = offset_curve(&circle, 3.0);
    if let Curve::Arc(a) = outer {
        let (cx, cy) = a.center.to_f64();
        assert!((cx - 10.0).abs() < 1e-6 && (cy - 20.0).abs() < 1e-6, "center moved");
        assert!((a.radius - 10.0).abs() < 1e-6, "radius should be 7+3=10");
    } else { panic!("offset of arc should be an arc"); }
}

// ── §2.1 PolyCurve evaluate routing across segments ───────────────────────────

#[test]
fn polycurve_evaluation_traverses_all_segments() {
    // Square path: (0,0)→(2,0)→(2,2)→(0,2)→(0,0)
    let pc = PolyCurve::new(vec![
        Curve::Line(LineSeg::from_endpoints(pt(0,0), pt(2,0))),
        Curve::Line(LineSeg::from_endpoints(pt(2,0), pt(2,2))),
        Curve::Line(LineSeg::from_endpoints(pt(2,2), pt(0,2))),
        Curve::Line(LineSeg::from_endpoints(pt(0,2), pt(0,0))),
    ]);
    // t=0 → start, t=1 → end (back to start). Sample the 4 quarter points.
    let (x0, y0) = pc.evaluate_f64(0.0);
    assert!((x0).abs() < 1e-9 && (y0).abs() < 1e-9, "t=0 should be (0,0), got ({},{})", x0, y0);

    // t=0.25 → end of first segment ≈ (2,0)
    let (x1, y1) = pc.evaluate_f64(0.249);
    assert!(x1 > 1.5 && y1.abs() < 0.5, "t≈0.25 should be near (2,0), got ({},{})", x1, y1);

    // Total arc length = perimeter = 8
    assert!((pc.arc_length() - 8.0).abs() < 1e-6, "perimeter should be 8, got {}", pc.arc_length());
}

// ── §2.1 Three-point circle through known points ──────────────────────────────

#[test]
fn three_point_circle_exact_center() {
    // Points (1,0),(0,1),(-1,0) → unit circle centered at origin.
    let p1 = Point2d::from_f64(1.0, 0.0);
    let p2 = Point2d::from_f64(0.0, 1.0);
    let p3 = Point2d::from_f64(-1.0, 0.0);
    let arc = CircularArc::from_three_points(&p1, &p2, &p3).expect("non-collinear");
    let (cx, cy) = arc.center.to_f64();
    assert!(cx.abs() < 1e-9 && cy.abs() < 1e-9, "center should be origin, got ({},{})", cx, cy);
    assert!((arc.radius - 1.0).abs() < 1e-6, "radius should be 1");
}

// ── §2.1 Collinear three points → no circle ───────────────────────────────────

#[test]
fn three_collinear_points_no_circle() {
    let p1 = pt(0,0); let p2 = pt(1,1); let p3 = pt(2,2);
    assert!(CircularArc::from_three_points(&p1, &p2, &p3).is_none(),
        "collinear points must not yield a circle");
}
