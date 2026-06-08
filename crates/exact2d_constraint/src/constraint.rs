//! Geometric and dimensional constraints (spec §4.4).
//!
//! Each constraint contributes one or more polynomial residual equations on the
//! sketch point coordinates. A residual of 0 means the constraint is satisfied.

/// Index of a point in the sketch (each point owns two variables: x, y).
pub type PointId = usize;

/// A line is defined by two of the sketch's points.
pub type LineRef = (PointId, PointId);

#[derive(Clone, Debug)]
pub enum Constraint {
    // ── Geometric ─────────────────────────────────────────────────────────────
    /// Two points are the same location.
    Coincident(PointId, PointId),
    /// Lock a point at a fixed coordinate.
    Fix(PointId, f64, f64),
    /// The segment p1→p2 is horizontal (y1 = y2).
    Horizontal(PointId, PointId),
    /// The segment p1→p2 is vertical (x1 = x2).
    Vertical(PointId, PointId),
    /// Two lines are parallel (direction cross product = 0).
    Parallel(LineRef, LineRef),
    /// Two lines are perpendicular (direction dot product = 0).
    Perpendicular(LineRef, LineRef),
    /// Three points are collinear.
    Collinear(PointId, PointId, PointId),
    /// Two segments have equal length.
    EqualLength(LineRef, LineRef),
    /// A circle (center, radius var) is tangent to a line — distance(center,line)=r.
    /// Represented by center point + a radius value + the line.
    /// p1,p2 = the two points defining a chord whose midpoint distance equals... —
    /// for simplicity tangency here is line-to-circle via the dimensional radius.
    Symmetric(PointId, PointId, LineRef),

    // ── Dimensional ───────────────────────────────────────────────────────────
    /// Distance between two points = value.
    Distance(PointId, PointId, f64),
    /// Horizontal distance between two points = value.
    DistanceX(PointId, PointId, f64),
    /// Vertical distance between two points = value.
    DistanceY(PointId, PointId, f64),
    /// Angle between two lines = value (radians).
    Angle(LineRef, LineRef, f64),
    /// First point is the midpoint of the segment defined by the second and third.
    Midpoint(PointId, PointId, PointId),
    /// Line is tangent to circle (line, circle_center, circle_start_point).
    TangentLineCircle(LineRef, PointId, PointId),
    /// Two circles are tangent (center1, start1, center2, start2, external).
    TangentCircleCircle(PointId, PointId, PointId, PointId, bool),
}

impl Constraint {
    /// Number of scalar residual equations this constraint contributes.
    pub fn equation_count(&self) -> usize {
        match self {
            // Two equations: position-match / reflection both pin 2 coordinates.
            Constraint::Coincident(..) | Constraint::Fix(..) | Constraint::Symmetric(..) | Constraint::Midpoint(..) => 2,
            _ => 1,
        }
    }

    /// Evaluate the residual(s) at the given variable vector.
    /// `vars[2*p]` = x of point p, `vars[2*p+1]` = y of point p.
    pub fn residuals(&self, vars: &[f64]) -> Vec<f64> {
        let x = |p: PointId| vars[2 * p];
        let y = |p: PointId| vars[2 * p + 1];
        let dir = |(a, b): LineRef| (x(b) - x(a), y(b) - y(a));

        match *self {
            Constraint::Coincident(p1, p2) =>
                vec![x(p1) - x(p2), y(p1) - y(p2)],

            Constraint::Fix(p, fx, fy) =>
                vec![x(p) - fx, y(p) - fy],

            Constraint::Horizontal(p1, p2) =>
                vec![y(p1) - y(p2)],

            Constraint::Vertical(p1, p2) =>
                vec![x(p1) - x(p2)],

            Constraint::Parallel(l1, l2) => {
                let (ux, uy) = dir(l1);
                let (vx, vy) = dir(l2);
                vec![ux * vy - uy * vx] // cross product
            }

            Constraint::Perpendicular(l1, l2) => {
                let (ux, uy) = dir(l1);
                let (vx, vy) = dir(l2);
                vec![ux * vx + uy * vy] // dot product
            }

            Constraint::Collinear(p1, p2, p3) => {
                // cross((p2-p1),(p3-p1)) = 0
                let ux = x(p2) - x(p1); let uy = y(p2) - y(p1);
                let vx = x(p3) - x(p1); let vy = y(p3) - y(p1);
                vec![ux * vy - uy * vx]
            }

            Constraint::EqualLength(l1, l2) => {
                let (ux, uy) = dir(l1);
                let (vx, vy) = dir(l2);
                vec![(ux * ux + uy * uy) - (vx * vx + vy * vy)]
            }

            Constraint::Symmetric(p1, p2, axis) => {
                // p1 and p2 are mirror images across the axis line. This needs BOTH:
                //   (a) their midpoint lies on the axis, and
                //   (b) the segment p1→p2 is perpendicular to the axis.
                // (a) alone leaves the points free to slide along the axis direction.
                let (ax, ay) = (x(axis.0), y(axis.0));
                let (bx, by) = dir(axis);
                let mx = (x(p1) + x(p2)) / 2.0;
                let my = (y(p1) + y(p2)) / 2.0;
                let on_axis = bx * (my - ay) - by * (mx - ax);        // midpoint on axis
                let perp = (x(p2) - x(p1)) * bx + (y(p2) - y(p1)) * by; // p1p2 ⟂ axis
                vec![on_axis, perp]
            }

            Constraint::Distance(p1, p2, d) => {
                let dx = x(p2) - x(p1); let dy = y(p2) - y(p1);
                vec![(dx * dx + dy * dy) - d * d]
            }

            Constraint::DistanceX(p1, p2, d) =>
                vec![(x(p2) - x(p1)).abs() - d],

            Constraint::DistanceY(p1, p2, d) =>
                vec![(y(p2) - y(p1)).abs() - d],

            Constraint::Angle(l1, l2, theta) => {
                // dot = |u||v|cos(theta)  →  residual on the cosine relation
                let (ux, uy) = dir(l1);
                let (vx, vy) = dir(l2);
                let dot = ux * vx + uy * vy;
                let lu = (ux * ux + uy * uy).sqrt();
                let lv = (vx * vx + vy * vy).sqrt();
                vec![dot - lu * lv * theta.cos()]
            }

            Constraint::Midpoint(m, a, b) => {
                vec![2.0 * x(m) - x(a) - x(b), 2.0 * y(m) - y(a) - y(b)]
            }

            Constraint::TangentLineCircle(line, center, start) => {
                let (ax, ay) = (x(line.0), y(line.0));
                let ux = x(line.1) - ax;
                let uy = y(line.1) - ay;
                let l_sq = ux * ux + uy * uy;
                let num = ux * (ay - y(center)) - uy * (ax - x(center));
                let r_sq = (x(start) - x(center)).powi(2) + (y(start) - y(center)).powi(2);
                vec![num * num - r_sq * l_sq]
            }

            Constraint::TangentCircleCircle(c1, s1, c2, s2, external) => {
                let dx = x(c2) - x(c1);
                let dy = y(c2) - y(c1);
                let dc_sq = dx * dx + dy * dy;
                let r1 = ((x(s1) - x(c1)).powi(2) + (y(s1) - y(c1)).powi(2)).sqrt();
                let r2 = ((x(s2) - x(c2)).powi(2) + (y(s2) - y(c2)).powi(2)).sqrt();
                let target = if external { r1 + r2 } else { (r1 - r2).abs() };
                vec![dc_sq - target * target]
            }
        }
    }

    /// Analytic Jacobian rows: one inner `Vec` per residual equation, each a list
    /// of `(variable_index, ∂residual/∂variable)` sparse entries (variable index of
    /// point `p` is `2p` for x, `2p+1` for y). Returns `None` for constraints whose
    /// derivatives are deliberately left to finite differences (Symmetric, Angle,
    /// Tangent*) — their closed forms are fiddly and low-value. The solver assembles
    /// analytic rows where available and finite-differences the rest.
    pub fn jacobian(&self, vars: &[f64]) -> Option<Vec<Vec<(usize, f64)>>> {
        let x = |p: PointId| vars[2 * p];
        let y = |p: PointId| vars[2 * p + 1];
        let xi = |p: PointId| 2 * p;
        let yi = |p: PointId| 2 * p + 1;
        let dir = |(a, b): LineRef| (x(b) - x(a), y(b) - y(a));

        Some(match *self {
            Constraint::Coincident(p1, p2) => vec![
                vec![(xi(p1), 1.0), (xi(p2), -1.0)],
                vec![(yi(p1), 1.0), (yi(p2), -1.0)],
            ],
            Constraint::Fix(p, _, _) => vec![
                vec![(xi(p), 1.0)],
                vec![(yi(p), 1.0)],
            ],
            Constraint::Horizontal(p1, p2) =>
                vec![vec![(yi(p1), 1.0), (yi(p2), -1.0)]],
            Constraint::Vertical(p1, p2) =>
                vec![vec![(xi(p1), 1.0), (xi(p2), -1.0)]],

            Constraint::Parallel(l1, l2) => {
                let (ux, uy) = dir(l1);
                let (vx, vy) = dir(l2);
                vec![vec![
                    (xi(l1.0), -vy), (yi(l1.0), vx), (xi(l1.1), vy), (yi(l1.1), -vx),
                    (xi(l2.0), uy), (yi(l2.0), -ux), (xi(l2.1), -uy), (yi(l2.1), ux),
                ]]
            }
            Constraint::Perpendicular(l1, l2) => {
                let (ux, uy) = dir(l1);
                let (vx, vy) = dir(l2);
                vec![vec![
                    (xi(l1.0), -vx), (yi(l1.0), -vy), (xi(l1.1), vx), (yi(l1.1), vy),
                    (xi(l2.0), -ux), (yi(l2.0), -uy), (xi(l2.1), ux), (yi(l2.1), uy),
                ]]
            }
            Constraint::Collinear(p1, p2, p3) => {
                let ux = x(p2) - x(p1); let uy = y(p2) - y(p1);
                let vx = x(p3) - x(p1); let vy = y(p3) - y(p1);
                vec![vec![
                    (xi(p1), uy - vy), (yi(p1), vx - ux),
                    (xi(p2), vy), (yi(p2), -vx),
                    (xi(p3), -uy), (yi(p3), ux),
                ]]
            }
            Constraint::EqualLength(l1, l2) => {
                let (ux, uy) = dir(l1);
                let (vx, vy) = dir(l2);
                vec![vec![
                    (xi(l1.0), -2.0 * ux), (yi(l1.0), -2.0 * uy), (xi(l1.1), 2.0 * ux), (yi(l1.1), 2.0 * uy),
                    (xi(l2.0), 2.0 * vx), (yi(l2.0), 2.0 * vy), (xi(l2.1), -2.0 * vx), (yi(l2.1), -2.0 * vy),
                ]]
            }
            Constraint::Distance(p1, p2, _) => {
                let dx = x(p2) - x(p1); let dy = y(p2) - y(p1);
                vec![vec![
                    (xi(p1), -2.0 * dx), (yi(p1), -2.0 * dy), (xi(p2), 2.0 * dx), (yi(p2), 2.0 * dy),
                ]]
            }
            Constraint::DistanceX(p1, p2, _) => {
                let s = if x(p2) - x(p1) >= 0.0 { 1.0 } else { -1.0 };
                vec![vec![(xi(p1), -s), (xi(p2), s)]]
            }
            Constraint::DistanceY(p1, p2, _) => {
                let s = if y(p2) - y(p1) >= 0.0 { 1.0 } else { -1.0 };
                vec![vec![(yi(p1), -s), (yi(p2), s)]]
            }
            Constraint::Midpoint(m, a, b) => vec![
                vec![(xi(m), 2.0), (xi(a), -1.0), (xi(b), -1.0)],
                vec![(yi(m), 2.0), (yi(a), -1.0), (yi(b), -1.0)],
            ],

            // Derivatives left to finite differences (fiddly closed forms).
            Constraint::Symmetric(..)
            | Constraint::Angle(..)
            | Constraint::TangentLineCircle(..)
            | Constraint::TangentCircleCircle(..) => return None,
        })
    }
}
