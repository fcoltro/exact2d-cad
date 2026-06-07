//! Phase 4.4 — Parametric constraint solver for Exact2D CAD sketches.
//!
//! Each geometric/dimensional constraint becomes one or more polynomial residual
//! equations on the sketch point coordinates. The system is solved by Gauss–Newton
//! with a finite-difference Jacobian and Levenberg damping; remaining degrees of
//! freedom are reported via the numerical rank of the Jacobian.

pub mod constraint;
pub mod solver;

pub use constraint::{Constraint, PointId, LineRef};
pub use solver::{Sketch, SolveStatus};
