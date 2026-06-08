//! Constraint solver (spec §4.4): builds the residual system from all constraints,
//! solves it by Gauss–Newton with a finite-difference Jacobian, and tracks DOF
//! via the numerical rank of the Jacobian.

// This module is dense linear algebra (Gauss–Newton, Gaussian elimination, rank).
// Index-based matrix loops (a[r][c], diagonal a[i][i], row swaps) are the standard,
// clearest expression of these algorithms — iterators would obscure them.
#![allow(clippy::needless_range_loop)]

use crate::constraint::{Constraint, PointId};

/// A parametric sketch: a set of points plus constraints relating them.
#[derive(Clone, Debug, Default)]
pub struct Sketch {
    /// Flattened coordinates: point p → (vars[2p], vars[2p+1]).
    vars: Vec<f64>,
    constraints: Vec<Constraint>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SolveStatus {
    /// All residuals below tolerance.
    Converged { iterations: u32, residual: f64 },
    /// Did not converge within the iteration budget.
    DidNotConverge { iterations: u32, residual: f64 },
}

/// Overall constrained-ness of a sketch (as shown in parametric CAD UIs).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstraintStatus {
    /// rank == 2·points and no redundancy: a unique solution.
    WellConstrained,
    /// Fewer independent equations than DOF: still movable.
    UnderConstrained { dof: usize },
    /// Redundant (and possibly conflicting) constraints present.
    OverConstrained,
}

/// A diagnosis of the sketch's constraint system, for surfacing in the UI.
#[derive(Clone, Debug)]
pub struct SketchDiagnostics {
    /// Remaining degrees of freedom = 2·points − rank(Jacobian).
    pub dof: usize,
    pub status: ConstraintStatus,
    /// Indices of constraints whose equations are linearly dependent on the others
    /// (redundant) — the ones a user could remove to relieve over-constraining.
    pub redundant: Vec<usize>,
}

impl Sketch {
    pub fn new() -> Self { Sketch::default() }

    /// Add a point at an initial position; returns its id.
    pub fn add_point(&mut self, x: f64, y: f64) -> PointId {
        let id = self.vars.len() / 2;
        self.vars.push(x);
        self.vars.push(y);
        id
    }

    pub fn add_constraint(&mut self, c: Constraint) {
        self.constraints.push(c);
    }

    pub fn point(&self, p: PointId) -> (f64, f64) {
        (self.vars[2 * p], self.vars[2 * p + 1])
    }

    pub fn set_point(&mut self, p: PointId, x: f64, y: f64) {
        self.vars[2 * p] = x;
        self.vars[2 * p + 1] = y;
    }

    pub fn constraints(&self) -> &[Constraint] {
        &self.constraints
    }

    pub fn constraints_mut(&mut self) -> &mut Vec<Constraint> {
        &mut self.constraints
    }

    pub fn num_points(&self) -> usize { self.vars.len() / 2 }

    /// Total residual equation count across all constraints.
    pub fn equation_count(&self) -> usize {
        self.constraints.iter().map(|c| c.equation_count()).sum()
    }

    /// Evaluate the full residual vector at the current variables.
    fn residual_vector(&self, vars: &[f64]) -> Vec<f64> {
        let mut f = Vec::with_capacity(self.equation_count());
        for c in &self.constraints {
            f.extend(c.residuals(vars));
        }
        f
    }

    /// Jacobian J[i][j] = ∂f_i/∂var_j. Hybrid: each constraint supplies analytic
    /// rows where it can (`Constraint::jacobian`), and the rest are filled by
    /// finite differences per-constraint. Analytic gradients are exact and cheaper
    /// (no extra residual evaluations), which also sharpens the rank/redundancy
    /// diagnostics.
    fn jacobian(&self, vars: &[f64]) -> Vec<Vec<f64>> {
        let n = vars.len();
        let m = self.equation_count();
        let mut j = vec![vec![0.0; n]; m];
        let h = 1e-7;
        let mut row = 0;
        for c in &self.constraints {
            let k = c.equation_count();
            match c.jacobian(vars) {
                Some(rows) => {
                    for (i, entries) in rows.iter().enumerate() {
                        for &(col, val) in entries {
                            j[row + i][col] = val;
                        }
                    }
                }
                None => {
                    // Finite-difference just this constraint's residual rows.
                    let f0 = c.residuals(vars);
                    let mut perturbed = vars.to_vec();
                    for col in 0..n {
                        let orig = perturbed[col];
                        perturbed[col] = orig + h;
                        let f1 = c.residuals(&perturbed);
                        perturbed[col] = orig;
                        for i in 0..k {
                            j[row + i][col] = (f1[i] - f0[i]) / h;
                        }
                    }
                }
            }
            row += k;
        }
        j
    }

    /// Solve the constraint system in place by **adaptive Levenberg–Marquardt**:
    /// solve `(JᵀJ + λI) δ = −Jᵀf`, accept the step only if it reduces the residual
    /// norm, and adapt λ — shrink it toward Gauss–Newton on success (fast quadratic
    /// convergence near the solution), grow it toward gradient descent on failure
    /// (a guaranteed-descent direction). This replaces the old fixed λ plus the
    /// "take the full step anyway" hack, which could diverge. λ-damping also keeps
    /// the normal equations solvable under gauge freedom (rank-deficient JᵀJ).
    pub fn solve(&mut self, max_iters: u32, tol: f64) -> SolveStatus {
        let mut vars = self.vars.clone();
        let n = vars.len();
        let mut f = self.residual_vector(&vars);
        let mut cost = norm(&f);
        let mut lambda = 1e-3;
        let mut iters = 0;

        while iters < max_iters {
            if cost < tol {
                self.vars = vars;
                return SolveStatus::Converged { iterations: iters, residual: cost };
            }
            iters += 1;

            let j = self.jacobian(&vars);
            let jt_j = mat_ata(&j, n);
            let jt_f = mat_atb(&j, &f, n);

            // Grow λ until a damped step actually reduces the cost (or give up).
            let mut accepted = false;
            for _ in 0..40 {
                let mut a = jt_j.clone();
                for i in 0..n { a[i][i] += lambda; }
                let rhs: Vec<f64> = jt_f.iter().map(|v| -v).collect();

                if let Some(delta) = solve_linear(a, rhs) {
                    let trial: Vec<f64> = vars.iter().zip(&delta).map(|(v, d)| v + d).collect();
                    let trial_f = self.residual_vector(&trial);
                    let trial_cost = norm(&trial_f);
                    if trial_cost < cost {
                        vars = trial;
                        f = trial_f;
                        cost = trial_cost;
                        lambda = (lambda * 0.5).max(1e-12); // trust the model more
                        accepted = true;
                        break;
                    }
                }
                lambda = (lambda * 4.0).min(1e12); // step rejected: damp harder
            }

            if !accepted {
                // No downhill step exists at any damping — a local min (often a
                // conflicting/over-constrained system). Stop.
                break;
            }
        }

        self.vars = vars;
        if cost < tol {
            SolveStatus::Converged { iterations: iters, residual: cost }
        } else {
            SolveStatus::DidNotConverge { iterations: iters, residual: cost }
        }
    }

    /// Remaining degrees of freedom = 2·points − rank(Jacobian).
    /// 0 = fully constrained, >0 = under-constrained, and over-constrained
    /// (redundant) systems are detected when equation_count > rank.
    pub fn degrees_of_freedom(&self) -> usize {
        let total = self.vars.len();
        if self.constraints.is_empty() { return total; }
        let j = self.jacobian(&self.vars);
        let rank = numerical_rank(&j);
        total.saturating_sub(rank)
    }

    /// Whether the system has redundant (linearly dependent) constraints.
    pub fn is_over_constrained(&self) -> bool {
        let j = self.jacobian(&self.vars);
        numerical_rank(&j) < self.equation_count()
    }

    /// Diagnose the constraint system: DOF, overall status, and which constraints
    /// are redundant. Redundancy is found by adding each constraint's Jacobian rows
    /// to an orthogonal basis (Gram–Schmidt); a constraint whose rows add no rank is
    /// linearly dependent on the others and flagged as redundant.
    pub fn diagnose(&self) -> SketchDiagnostics {
        let total = self.vars.len();
        if self.constraints.is_empty() {
            return SketchDiagnostics {
                dof: total,
                status: ConstraintStatus::UnderConstrained { dof: total },
                redundant: Vec::new(),
            };
        }

        let j = self.jacobian(&self.vars);
        let mut basis: Vec<Vec<f64>> = Vec::new();
        let mut redundant = Vec::new();
        let mut row = 0;
        for (ci, c) in self.constraints.iter().enumerate() {
            let k = c.equation_count();
            let mut any_independent = false;
            for r in row..(row + k) {
                if r < j.len() && add_to_basis(&mut basis, j[r].clone()) {
                    any_independent = true;
                }
            }
            if !any_independent {
                redundant.push(ci);
            }
            row += k;
        }

        let rank = basis.len();
        let dof = total.saturating_sub(rank);
        let status = if !redundant.is_empty() {
            ConstraintStatus::OverConstrained
        } else if dof > 0 {
            ConstraintStatus::UnderConstrained { dof }
        } else {
            ConstraintStatus::WellConstrained
        };
        SketchDiagnostics { dof, status, redundant }
    }
}

// ── Small dense linear algebra ────────────────────────────────────────────────

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// Try to add `row` to an orthogonal `basis` (Gram–Schmidt). Returns true if the
/// row was linearly independent (and was added, normalized); false if it lies in
/// the span of the existing basis (i.e. it's redundant).
fn add_to_basis(basis: &mut Vec<Vec<f64>>, mut row: Vec<f64>) -> bool {
    let tol = 1e-9;
    for b in basis.iter() {
        let dot: f64 = row.iter().zip(b).map(|(r, bb)| r * bb).sum();
        for (r, bb) in row.iter_mut().zip(b) {
            *r -= dot * bb; // basis vectors are unit-norm, so projection = dot·b
        }
    }
    let resid = norm(&row);
    if resid > tol {
        for r in row.iter_mut() { *r /= resid; }
        basis.push(row);
        true
    } else {
        false
    }
}

/// AᵀA for an m×n matrix A → n×n.
fn mat_ata(a: &[Vec<f64>], n: usize) -> Vec<Vec<f64>> {
    let m = a.len();
    let mut out = vec![vec![0.0; n]; n];
    for i in 0..n {
        for k in 0..n {
            let mut s = 0.0;
            for r in 0..m { s += a[r][i] * a[r][k]; }
            out[i][k] = s;
        }
    }
    out
}

/// Aᵀb for an m×n matrix A and length-m vector b → length n.
fn mat_atb(a: &[Vec<f64>], b: &[f64], n: usize) -> Vec<f64> {
    let m = a.len();
    let mut out = vec![0.0; n];
    for i in 0..n {
        let mut s = 0.0;
        for r in 0..m { s += a[r][i] * b[r]; }
        out[i] = s;
    }
    out
}

/// Solve a square linear system A x = b by Gaussian elimination with partial
/// pivoting. Returns None if singular.
fn solve_linear(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        // Pivot
        let mut piv = col;
        for r in (col + 1)..n {
            if a[r][col].abs() > a[piv][col].abs() { piv = r; }
        }
        if a[piv][col].abs() < 1e-14 { return None; }
        a.swap(col, piv);
        b.swap(col, piv);
        // Eliminate
        for r in (col + 1)..n {
            let factor = a[r][col] / a[col][col];
            for c in col..n { a[r][c] -= factor * a[col][c]; }
            b[r] -= factor * b[col];
        }
    }
    // Back-substitute
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = b[i];
        for c in (i + 1)..n { s -= a[i][c] * x[c]; }
        x[i] = s / a[i][i];
    }
    Some(x)
}

/// Numerical rank via Gaussian elimination with a tolerance on pivots.
fn numerical_rank(mat: &[Vec<f64>]) -> usize {
    if mat.is_empty() { return 0; }
    let mut a: Vec<Vec<f64>> = mat.to_vec();
    let rows = a.len();
    let cols = a[0].len();
    let mut rank = 0;
    let tol = 1e-9;
    let mut pivot_col = 0;
    for r in 0..rows {
        if pivot_col >= cols { break; }
        // Find a pivot in column pivot_col at or below row r
        let mut best = r;
        for rr in r..rows {
            if a[rr][pivot_col].abs() > a[best][pivot_col].abs() { best = rr; }
        }
        if a[best][pivot_col].abs() < tol {
            pivot_col += 1;
            // Retry same row with next column
            if pivot_col < cols { continue; } else { break; }
        }
        a.swap(r, best);
        for rr in 0..rows {
            if rr != r {
                let f = a[rr][pivot_col] / a[r][pivot_col];
                for c in pivot_col..cols { a[rr][c] -= f * a[r][c]; }
            }
        }
        rank += 1;
        pivot_col += 1;
    }
    rank
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraint::Constraint;

    #[test]
    fn distance_constraint_pulls_point() {
        // Two points; fix one at origin, constrain distance = 5, expect convergence.
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(3.0, 1.0); // wrong distance initially
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Horizontal(a, b));
        s.add_constraint(Constraint::Distance(a, b, 5.0));
        let status = s.solve(100, 1e-10);
        assert!(matches!(status, SolveStatus::Converged { .. }), "status={:?}", status);
        let (bx, by) = s.point(b);
        assert!((bx.abs() - 5.0).abs() < 1e-5, "bx={}", bx);
        assert!(by.abs() < 1e-5, "by={}", by);
    }

    #[test]
    fn perpendicular_constraint() {
        // Line a-b horizontal, line a-c constrained perpendicular → c goes vertical.
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(5.0, 0.0);
        let c = s.add_point(1.0, 3.0); // not yet perpendicular
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Fix(b, 5.0, 0.0));
        s.add_constraint(Constraint::Perpendicular((a, b), (a, c)));
        let status = s.solve(100, 1e-10);
        assert!(matches!(status, SolveStatus::Converged { .. }));
        // a→c must be vertical: cx ≈ 0
        let (cx, _cy) = s.point(c);
        assert!(cx.abs() < 1e-5, "cx={}", cx);
    }

    #[test]
    fn parallel_constraint() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(4.0, 0.0);
        let c = s.add_point(0.0, 2.0);
        let d = s.add_point(3.0, 3.0);
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Fix(b, 4.0, 0.0));
        s.add_constraint(Constraint::Fix(c, 0.0, 2.0));
        s.add_constraint(Constraint::Parallel((a, b), (c, d)));
        s.solve(100, 1e-10);
        // c→d parallel to a→b (horizontal): dy ≈ 0 → d.y ≈ c.y = 2
        let (_dx, dy) = s.point(d);
        assert!((dy - 2.0).abs() < 1e-4, "dy={}", dy);
    }

    #[test]
    fn dof_fully_constrained_triangle() {
        // 3 points = 6 DOF. Fix one (−2), set two distances and an angle... simpler:
        // fully fix all three points → 0 DOF.
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(1.0, 0.0);
        let c = s.add_point(0.0, 1.0);
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Fix(b, 1.0, 0.0));
        s.add_constraint(Constraint::Fix(c, 0.0, 1.0));
        assert_eq!(s.degrees_of_freedom(), 0);
    }

    #[test]
    fn dof_underconstrained() {
        // 2 points = 4 DOF. Only fix one point (removes 2) → 2 DOF remain.
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let _b = s.add_point(3.0, 4.0);
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        assert_eq!(s.degrees_of_freedom(), 2);
    }

    #[test]
    fn over_constrained_detected() {
        // Fix a point twice → redundant constraint.
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0)); // redundant
        assert!(s.is_over_constrained());
    }

    #[test]
    fn horizontal_and_vertical() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(5.0, 2.0);
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Horizontal(a, b));
        let status = s.solve(100, 1e-12);
        assert!(matches!(status, SolveStatus::Converged { .. }));
        let (_bx, by) = s.point(b);
        assert!(by.abs() < 1e-6, "horizontal → by≈0, got {}", by);
    }

    #[test]
    fn analytic_jacobian_matches_finite_difference() {
        // Representative sketch exercising every analytically-differentiated
        // constraint (plus FD-only ones, which match trivially). Points are at
        // generic, non-degenerate positions.
        let mut s = Sketch::new();
        let a = s.add_point(0.3, 1.1);
        let b = s.add_point(2.7, 0.4);
        let c = s.add_point(1.2, 3.3);
        let d = s.add_point(4.1, 2.2);
        let e = s.add_point(0.9, 5.0);
        s.add_constraint(Constraint::Fix(a, 0.3, 1.1));
        s.add_constraint(Constraint::Coincident(a, b));
        s.add_constraint(Constraint::Horizontal(a, b));
        s.add_constraint(Constraint::Vertical(c, d));
        s.add_constraint(Constraint::Parallel((a, b), (c, d)));
        s.add_constraint(Constraint::Perpendicular((a, b), (c, e)));
        s.add_constraint(Constraint::Collinear(a, c, e));
        s.add_constraint(Constraint::EqualLength((a, b), (c, d)));
        s.add_constraint(Constraint::Distance(a, b, 2.0));
        s.add_constraint(Constraint::DistanceX(b, c, 1.0));
        s.add_constraint(Constraint::DistanceY(c, d, 1.0));
        s.add_constraint(Constraint::Midpoint(e, a, b));
        s.add_constraint(Constraint::Angle((a, b), (c, d), 0.5)); // FD-only

        let vars = s.vars.clone();
        let analytic = s.jacobian(&vars);

        // Independent full finite-difference reference.
        let f0 = s.residual_vector(&vars);
        let (m, n) = (f0.len(), vars.len());
        let h = 1e-7;
        let mut pert = vars.clone();
        for col in 0..n {
            let o = pert[col];
            pert[col] = o + h;
            let f1 = s.residual_vector(&pert);
            pert[col] = o;
            for r in 0..m {
                let fd = (f1[r] - f0[r]) / h;
                assert!((analytic[r][col] - fd).abs() < 1e-4,
                    "Jacobian mismatch at row {r}, col {col}: analytic={}, fd={}",
                    analytic[r][col], fd);
            }
        }
    }

    #[test]
    fn diagnose_well_constrained() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(1.0, 0.0);
        let c = s.add_point(0.0, 1.0);
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Fix(b, 1.0, 0.0));
        s.add_constraint(Constraint::Fix(c, 0.0, 1.0));
        let d = s.diagnose();
        assert_eq!(d.dof, 0);
        assert_eq!(d.status, ConstraintStatus::WellConstrained);
        assert!(d.redundant.is_empty());
    }

    #[test]
    fn diagnose_under_constrained() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let _b = s.add_point(3.0, 4.0);
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        let d = s.diagnose();
        assert_eq!(d.dof, 2);
        assert_eq!(d.status, ConstraintStatus::UnderConstrained { dof: 2 });
        assert!(d.redundant.is_empty());
    }

    #[test]
    fn diagnose_flags_redundant_constraint() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0)); // redundant duplicate
        let d = s.diagnose();
        assert_eq!(d.status, ConstraintStatus::OverConstrained);
        assert_eq!(d.redundant, vec![1], "the second Fix is the redundant one");
    }

    #[test]
    fn adaptive_lm_converges_on_distance_chain() {
        // A small chain that needs several LM steps: fix a, b on a circle of radius 5
        // from a, and horizontal — should pull b to (5,0) robustly.
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(0.2, 4.9); // poor initial guess
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Horizontal(a, b));
        s.add_constraint(Constraint::Distance(a, b, 5.0));
        let status = s.solve(200, 1e-10);
        assert!(matches!(status, SolveStatus::Converged { .. }), "status={status:?}");
        let (bx, by) = s.point(b);
        assert!((bx.abs() - 5.0).abs() < 1e-5, "bx={bx}");
        assert!(by.abs() < 1e-5, "by={by}");
    }

    #[test]
    fn equal_length_constraint() {
        let mut s = Sketch::new();
        let a = s.add_point(0.0, 0.0);
        let b = s.add_point(4.0, 0.0); // length 4
        let c = s.add_point(0.0, 0.0);
        let d = s.add_point(0.0, 2.0); // length 2
        s.add_constraint(Constraint::Fix(a, 0.0, 0.0));
        s.add_constraint(Constraint::Fix(b, 4.0, 0.0));
        s.add_constraint(Constraint::Fix(c, 0.0, 0.0));
        s.add_constraint(Constraint::Vertical(c, d));
        s.add_constraint(Constraint::EqualLength((a, b), (c, d)));
        s.solve(100, 1e-10);
        // d should move so |c-d| = 4 → d.y = ±4
        let (_dx, dy) = s.point(d);
        assert!((dy.abs() - 4.0).abs() < 1e-4, "dy={}", dy);
    }
}
