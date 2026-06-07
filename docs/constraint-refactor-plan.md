# Constraint System Refactor — Plan

**Status:** Stage 0 in progress · **Decided:** toggle model, discard-on-exit (Option A),
3-point arc model for v1, constraints not persisted to file.

## Why

The constraint code feels like a mess because the **parametric sketch is treated as a
permanent mirror of the document** — always allocated, kept in lockstep, and *rebuilt from
geometry on every sync* (`sync_sketch_from_document`). That rebuild dedups points by 1e-4
proximity (`register_point`, O(n²)) and remaps constraints by a fragile positional zip.
Consequences: independent endpoints at the same location get **welded**; coincidence behaves
differently before vs. after a resync; arc angles are recovered by a TAU-multiple heuristic;
exactness is destroyed (everything round-trips `Rational → f64 → from_f64_approx`).

## Architecture

- **Document (exact geometry) = single source of truth, always.**
- **Parametric state lives in an `Option<ParametricSession>` that exists only while the mode
  is ON.** It holds the `sketch`, the `entity_points` map, and the constraints. When OFF it is
  `None`: no sketch, no sync, no solver, no overhead — pure free drafting (standard AutoCAD).

### Lifecycle
- **Enter** (toggle ON): snapshot for undo → build the session **once** from current geometry.
- **While ON:** new entities/edits update the session **in place**; adding a constraint solves
  and writes back to the document.
- **Exit** (toggle OFF): geometry is already baked into the document (writeback after each
  solve) → **drop the session** → instantly back to free drafting.

Under Option A, constraints never outlive a session, so they are **not** persisted to file.

## Stages

- **Stage 0 — Safety net (no behavior change).** AppState-level tests capturing current good
  behavior (H/V/distance/perp/equal solve; polyline shared vertex; arc stays circular; undo/
  redo across a constraint; toggle-off keeps geometry). Plus an `#[ignore]` marker test
  encoding the target "independent endpoints at the same location must stay independent"
  (proves the Stage 3 welding fix).
- **Stage 1 — `ParametricSession` lifecycle.** Move sketch/entity_points/constraints into an
  optional session; add `enter_parametric`/`exit_parametric`; stop calling
  `sync_sketch_from_document` on every edit — build once on enter, bake + drop on exit. Wire to
  the `CONSTRAINTS` status toggle + `⛓ Parametric` menu item. *Most of the mess disappears here.*
- **Stage 2 — In-session incremental maintenance + kill positional dedup.** New entities/edits
  update the session in place; delete `register_point`'s proximity merge.
- **Stage 3 — Coincidence as shared points.** `merge_points(a,b)` triggered only by intent
  (explicit Coincident, snap-to-endpoint at draw time, shared polyline vertices).
- **Stage 4 — Clean up `add_constraint`.** Replace the 230-line positional mega-match with one
  tested operand-resolution helper.
- **Stage 5 — (later) Solver quality.** Analytic Jacobian, adaptive Levenberg–Marquardt, QR
  least-squares, per-constraint conflict diagnostics. Independent; any time after Stage 1.
- **Stage 6 — (future) Exact-kernel reconnection.** Recover exact coordinates for the subset
  pinned by exact linear constraints after a numeric solve.

## Order
0 → 1 → 2 → 3 → 4, solver later. Stages 0–2 deliver the bulk of the cleanup.

## UX note
Exit discards constraints, so show a subtle one-time hint ("Parametric off — constraints
cleared") rather than a blocking dialog, keeping the revert instant.
