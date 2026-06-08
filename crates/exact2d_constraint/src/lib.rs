//! Phase 4.4 — Parametric constraint solver for Exact2D CAD sketches.
//!
//! Each geometric/dimensional constraint becomes one or more polynomial residual
//! equations on the sketch point coordinates. The system is solved by adaptive
//! Levenberg–Marquardt with a finite-difference Jacobian; remaining degrees of
//! freedom and redundant constraints are reported via the rank of the Jacobian.

pub mod constraint;
pub mod solver;

pub use constraint::{Constraint, PointId, LineRef};
pub use solver::{Sketch, SolveStatus, SketchDiagnostics, ConstraintStatus};
