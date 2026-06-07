//! Behavioral probes for the quadtree spatial index (spec §2.3).

use exact2d_spatial::Quadtree;
use exact2d_geometry::{Curve, LineSeg, Point2d, BoundingBox};

fn line(x0: f64, y0: f64, x1: f64, y1: f64) -> Curve {
    Curve::Line(LineSeg::from_endpoints(
        Point2d::from_f64(x0, y0), Point2d::from_f64(x1, y1)))
}

#[test]
fn many_curves_in_small_region_still_queryable() {
    // Insert 50 tiny segments clustered in one corner; the tree must split
    // adaptively and still return all of them for a covering query.
    let mut qt = Quadtree::new(BoundingBox::from_corners(0.0, 0.0, 100.0, 100.0), 12);
    let mut ids = Vec::new();
    for i in 0..50 {
        let x = (i as f64) * 0.01; // all within [0, 0.5]
        ids.push(qt.insert(line(x, 0.0, x + 0.005, 0.5)));
    }
    // Query the cluster region — must find all 50
    let found = qt.query_rect(&BoundingBox::from_corners(-1.0, -1.0, 1.0, 1.0));
    for id in &ids {
        assert!(found.contains(id), "curve {} missing from query results", id);
    }
}

#[test]
fn query_disjoint_region_returns_nothing() {
    let mut qt = Quadtree::new(BoundingBox::from_corners(-100.0, -100.0, 100.0, 100.0), 12);
    qt.insert(line(0.0, 0.0, 5.0, 5.0));
    qt.insert(line(10.0, 10.0, 15.0, 15.0));
    let found = qt.query_rect(&BoundingBox::from_corners(50.0, 50.0, 60.0, 60.0));
    assert!(found.is_empty(), "expected no curves in far region, got {:?}", found);
}

#[test]
fn nearest_curve_picks_closest() {
    let mut qt = Quadtree::new(BoundingBox::from_corners(-100.0, -100.0, 100.0, 100.0), 12);
    let near = qt.insert(line(0.0, 0.0, 1.0, 0.0));    // near origin
    let _far = qt.insert(line(50.0, 50.0, 60.0, 50.0)); // far away
    let nn = qt.nearest_curve(0.5, 0.1);
    assert_eq!(nn, Some(near), "nearest to (0.5,0.1) should be the segment at origin");
}

#[test]
fn query_point_returns_leaf() {
    let mut qt = Quadtree::new(BoundingBox::from_corners(-10.0, -10.0, 10.0, 10.0), 8);
    qt.insert(line(0.0, 0.0, 2.0, 2.0));
    let leaf = qt.query_point(1.0, 1.0);
    assert!(leaf.is_some(), "point inside model bounds should land in a leaf");
    // Point outside the model bounds → None
    assert!(qt.query_point(100.0, 100.0).is_none());
}

#[test]
fn overlapping_curve_found_by_multiple_cells() {
    // A long diagonal spanning the whole model should be found by queries in
    // both the SW and NE quadrants.
    let mut qt = Quadtree::new(BoundingBox::from_corners(-50.0, -50.0, 50.0, 50.0), 10);
    let id = qt.insert(line(-40.0, -40.0, 40.0, 40.0));
    let sw = qt.query_rect(&BoundingBox::from_corners(-45.0, -45.0, -30.0, -30.0));
    let ne = qt.query_rect(&BoundingBox::from_corners(30.0, 30.0, 45.0, 45.0));
    assert!(sw.contains(&id), "diagonal missing from SW query");
    assert!(ne.contains(&id), "diagonal missing from NE query");
}
