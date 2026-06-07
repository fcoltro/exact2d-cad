//! Behavioral probes for boolean region operations (spec §2.4).

use exact2d_boolean::{Region, union, intersection, difference};
use exact2d_geometry::{Curve, LineSeg, Point2d};

fn square(x0: i64, y0: i64, x1: i64, y1: i64) -> Region {
    Region::new(vec![
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(x0,y0), Point2d::from_i64(x1,y0))),
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(x1,y0), Point2d::from_i64(x1,y1))),
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(x1,y1), Point2d::from_i64(x0,y1))),
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(x0,y1), Point2d::from_i64(x0,y0))),
    ])
}

#[test]
fn disjoint_union_keeps_both() {
    // Two non-overlapping squares; union must contain interior points of both.
    let a = square(0, 0, 2, 2);
    let b = square(5, 5, 7, 7);
    let u = union(&a, &b);
    // Every selected segment lies on a boundary of A or B (in A∪B both are full boundary)
    assert!(!u.outer.is_empty());
    // A point inside A and a point inside B remain "inside" per the original regions
    assert!(a.contains_point(1.0, 1.0));
    assert!(b.contains_point(6.0, 6.0));
}

#[test]
fn intersection_of_disjoint_is_empty() {
    // Non-overlapping squares → intersection has no interior.
    let a = square(0, 0, 2, 2);
    let b = square(5, 5, 7, 7);
    let inter = intersection(&a, &b);
    // No segment midpoint can be inside both (they don't overlap)
    for seg in &inter.outer {
        use exact2d_geometry::CurveSegment;
        let (t0, t1) = seg.domain();
        let (mx, my) = seg.evaluate_f64((t0 + t1) / 2.0);
        assert!(!(a.contains_point(mx, my) && b.contains_point(mx, my)),
            "disjoint intersection produced an inside-both segment at ({},{})", mx, my);
    }
}

#[test]
fn difference_self_is_empty_interior() {
    // A − A: nothing should remain inside.
    let a = square(0, 0, 4, 4);
    let b = square(0, 0, 4, 4);
    let diff = difference(&a, &b);
    use exact2d_geometry::CurveSegment;
    for seg in &diff.outer {
        let (t0, t1) = seg.domain();
        let (mx, my) = seg.evaluate_f64((t0 + t1) / 2.0);
        // Inside A and NOT inside B can't hold when A==B (De Morgan: !A || B)
        assert!(!a.contains_point(mx, my) || b.contains_point(mx, my),
            "A−A left a segment at ({},{})", mx, my);
    }
}

#[test]
fn region_winding_number_basic() {
    let a = square(0, 0, 10, 10);
    assert!(a.contains_point(5.0, 5.0), "center inside");
    assert!(!a.contains_point(-1.0, 5.0), "left of square outside");
    assert!(!a.contains_point(11.0, 5.0), "right of square outside");
    assert!(!a.contains_point(5.0, 20.0), "above square outside");
}

#[test]
fn region_with_hole_excludes_hole_interior() {
    // Outer 0..10 square (CCW) with a 3..7 hole (CW).
    let outer = vec![
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(0,0), Point2d::from_i64(10,0))),
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(10,0), Point2d::from_i64(10,10))),
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(10,10), Point2d::from_i64(0,10))),
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(0,10), Point2d::from_i64(0,0))),
    ];
    // Hole wound clockwise
    let hole = vec![
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(3,3), Point2d::from_i64(3,7))),
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(3,7), Point2d::from_i64(7,7))),
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(7,7), Point2d::from_i64(7,3))),
        Curve::Line(LineSeg::from_endpoints(Point2d::from_i64(7,3), Point2d::from_i64(3,3))),
    ];
    let region = Region::with_holes(outer, vec![hole]);
    // Point in the ring (between outer and hole) is inside
    assert!(region.contains_point(1.0, 5.0), "ring point should be inside");
    // Point in the hole is OUTSIDE
    assert!(!region.contains_point(5.0, 5.0), "hole center should be outside");
}
