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

    /// Finite-difference Jacobian: J[i][j] = ∂f_i/∂var_j.
    fn jacobian(&self, vars: &[f64]) -> Vec<Vec<f64>> {
        let n = vars.len();
        let f0 = self.residual_vector(vars);
        let m = f0.len();
        let mut j = vec![vec![0.0; n]; m];
        let h = 1e-7;
        let mut perturbed = vars.to_vec();
        for col in 0..n {
            let orig = perturbed[col];
            perturbed[col] = orig + h;
            let f1 = self.residual_vector(&perturbed);
            perturbed[col] = orig;
            for row in 0..m {
                j[row][col] = (f1[row] - f0[row]) / h;
            }
        }
        j
    }

    /// Solve the constraint system in place. Gauss–Newton with normal equations.
    pub fn solve(&mut self, max_iters: u32, tol: f64) -> SolveStatus {
        let mut vars = self.vars.clone();
        let mut last_residual = f64::INFINITY;

        for iter in 0..max_iters {
            let f = self.residual_vector(&vars);
            let residual_norm = norm(&f);
            last_residual = residual_norm;
            if residual_norm < tol {
                self.vars = vars;
                return SolveStatus::Converged { iterations: iter, residual: residual_norm };
            }

            let j = self.jacobian(&vars);
            // Solve (JᵀJ + λI) δ = −Jᵀf  (Levenberg-style damping for robustness).
            let n = vars.len();
            let jt_j = mat_ata(&j, n);
            let jt_f = mat_atb(&j, &f, n);
            let lambda = 1e-9;
            let mut a = jt_j;
            for i in 0..n { a[i][i] += lambda; }
            let rhs: Vec<f64> = jt_f.iter().map(|v| -v).collect();

            match solve_linear(a, rhs) {
                Some(delta) => {
                    // Damped step with simple backtracking line search.
                    let mut step = 1.0;
                    let mut applied = false;
                    for _ in 0..20 {
                        let trial: Vec<f64> = vars.iter().zip(&delta).map(|(v, d)| v + step * d).collect();
                        if norm(&self.residual_vector(&trial)) < residual_norm {
                            vars = trial; applied = true; break;
                        }
                        step *= 0.5;
                    }
                    if !applied {
                        // No decrease found — take the full step anyway to escape flats.
                        for (v, d) in vars.iter_mut().zip(&delta) { *v += d; }
                    }
                }
                None => break, // singular system
            }
        }

        self.vars = vars;
        SolveStatus::DidNotConverge { iterations: max_iters, residual: last_residual }
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
}

// ── Small dense linear algebra ────────────────────────────────────────────────

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
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
