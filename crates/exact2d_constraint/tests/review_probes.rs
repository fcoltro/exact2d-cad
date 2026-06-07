//! Review probes for the constraint solver — stress symmetry, directional
//! distances, angles, and a fuller multi-constraint sketch.

use exact2d_constraint::{Sketch, Constraint, SolveStatus};

fn converged(s: &SolveStatus) -> bool { matches!(s, SolveStatus::Converged { .. }) }

/// A Symmetric constraint must make the two points genuine mirror images across
/// the axis: equal perpendicular distance AND the segment ⟂ to the axis.
#[test]
fn symmetric_makes_true_mirror() {
    let mut s = Sketch::new();
    // Axis = the y-axis (vertical line through (0,0)-(0,1)).
    let a0 = s.add_point(0.0, 0.0);
    let a1 = s.add_point(0.0, 1.0);
    // Two points that should become mirror images across the y-axis.
    let p1 = s.add_point(3.0, 2.0);
    let p2 = s.add_point(2.0, -1.0); // deliberately NOT the mirror of p1 yet
    s.add_constraint(Constraint::Fix(a0, 0.0, 0.0));
    s.add_constraint(Constraint::Fix(a1, 0.0, 1.0));
    s.add_constraint(Constraint::Fix(p1, 3.0, 2.0));     // anchor p1
    s.add_constraint(Constraint::Symmetric(p1, p2, (a0, a1)));

    let status = s.solve(200, 1e-10);
    assert!(converged(&status), "symmetric solve failed: {:?}", status);

    let (p1x, p1y) = s.point(p1);
    let (p2x, p2y) = s.point(p2);
    // True mirror across the y-axis: p2 = (-p1x, p1y).
    assert!((p2x + p1x).abs() < 1e-4, "p2.x should be -p1.x: {} vs {}", p2x, -p1x);
    assert!((p2y - p1y).abs() < 1e-4, "p2.y should equal p1.y: {} vs {}", p2y, p1y);
}

/// DistanceX must set the horizontal gap exactly, even when the points start on
/// the wrong side (the residual must steer the solver to the right magnitude).
#[test]
fn distance_x_sets_horizontal_gap() {
    let mut s = Sketch::new();
    let a = s.add_point(0.0, 0.0);
    let b = s.add_point(1.0, 4.0); // wrong x-gap
    s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
    s.add_constraint(Constraint::DistanceY(a, b, 4.0)); // keep y-gap pinned
    s.add_constraint(Constraint::DistanceX(a, b, 7.0)); // want |bx-ax| = 7
    let status = s.solve(300, 1e-9);
    assert!(converged(&status), "distance_x solve failed: {:?}", status);
    let (bx, _by) = s.point(b);
    assert!((bx.abs() - 7.0).abs() < 1e-3, "horizontal gap = {}, expected 7", bx.abs());
}

/// Angle constraint: make two lines meet at 90° starting from ~45°.
#[test]
fn angle_constraint_right_angle() {
    let mut s = Sketch::new();
    let o = s.add_point(0.0, 0.0);
    let a = s.add_point(2.0, 0.0); // line o→a horizontal (fixed)
    let b = s.add_point(2.0, 2.0); // line o→b at 45°, want 90°
    s.add_constraint(Constraint::Fix(o, 0.0, 0.0));
    s.add_constraint(Constraint::Fix(a, 2.0, 0.0));
    s.add_constraint(Constraint::Angle((o, a), (o, b), std::f64::consts::FRAC_PI_2));
    let status = s.solve(300, 1e-9);
    assert!(converged(&status), "angle solve failed: {:?}", status);
    // o→b must be vertical: bx ≈ 0.
    let (bx, _by) = s.point(b);
    assert!(bx.abs() < 1e-3, "line should be vertical, bx={}", bx);
}

/// A fully-constrained right triangle: legs 3 and 4 along the axes.
#[test]
fn constrained_right_triangle() {
    let mut s = Sketch::new();
    let a = s.add_point(0.0, 0.0);
    let b = s.add_point(2.0, 0.5);
    let c = s.add_point(0.5, 2.0);
    s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
    s.add_constraint(Constraint::Horizontal(a, b));
    s.add_constraint(Constraint::Distance(a, b, 3.0));
    s.add_constraint(Constraint::Vertical(a, c));
    s.add_constraint(Constraint::Distance(a, c, 4.0));
    let status = s.solve(300, 1e-10);
    assert!(converged(&status), "triangle solve failed: {:?}", status);

    let (bx, by) = s.point(b);
    let (cx, cy) = s.point(c);
    assert!((bx.abs() - 3.0).abs() < 1e-4 && by.abs() < 1e-4, "b=({},{})", bx, by);
    assert!(cx.abs() < 1e-4 && (cy.abs() - 4.0).abs() < 1e-4, "c=({},{})", cx, cy);
    // Hypotenuse b→c should be 5 (3-4-5).
    let hyp = ((bx - cx).powi(2) + (by - cy).powi(2)).sqrt();
    assert!((hyp - 5.0).abs() < 1e-3, "hypotenuse = {}, expected 5", hyp);

    // DOF should be 0 (fully constrained).
    assert_eq!(s.degrees_of_freedom(), 0, "triangle should have 0 DOF");
}
