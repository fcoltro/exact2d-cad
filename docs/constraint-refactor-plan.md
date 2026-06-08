# Constraint System Refactor — Plan

**Status:** Stages 0–3 done · Stage 5 first pass done (adaptive LM + diagnostics) ·
**Decided:** toggle model, discard-on-exit (Option A), 3-point arc model for v1, constraints
not persisted to file.

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
- **Stage 1 — Parametric lifecycle. ✅ DONE.** Added `enter_parametric`/`exit_parametric`:
  build the sketch overlay once on enter, **discard it on exit** (Option A), no sketch overlay
  at startup. All enable paths (`ToggleConstraints`, `CON …`, the Dimension tool) route through
  them. *Scope note:* the overlay data still lives as `sketch`/`entity_points` fields on
  AppState gated by `constraints_enabled` — the literal `Option<ParametricSession>` type-level
  move is deferred (it would touch ~80 call sites + document/sketch split-borrows) and is a
  later cleanup. The **per-edit rebuild** of the sketch (`sync_sketch_from_document` after each
  edit while active) is intentionally kept for now; removing it is coupled to Stage 2's
  incremental maintenance.
- **Stage 2 — Kill positional dedup (welding fix). ✅ DONE.** Replaced the global 1e-4
  `register_point` proximity dedup with a per-entity `push_point`: it shares only with points
  already registered for the *same* entity (polyline joints, full-circle start==end collapse),
  so independent geometry that merely touches stays distinct. The
  `stage0_independent_…_stay_independent` marker was un-ignored and now passes. *Deferred:* true
  incremental maintenance (new entities/edits updating the overlay in place). The full
  rebuild-on-edit (`sync_sketch_from_document`) is retained — it's now safe because per-entity
  registration is deterministic, so the constraint remap is stable. Fold incremental into a
  later pass if perf/robustness needs it.
- **Stage 3 — Coincidence by intent. ✅ DONE.** Re-scoped after finding Stage 2 already made
  *explicit* Coincident work robustly (a constraint that survives the rebuild). True shared-id
  merges turned out to be coupled to the deferred incremental maintenance + `History` changes
  (fragile in the current rebuild architecture), so instead of `merge_points` we capture the
  one genuinely-missing piece: **snap-to-endpoint while drawing**. In parametric mode, when a
  draw click resolves via an Endpoint/Node snap onto an existing entity, the new geometry's
  point is linked to the snapped-to point with a **Coincident constraint** (recorded as a
  pending link per click, materialized on entity Create). It connects by *intent* only (snap),
  never by proximity, and survives the per-edit rebuild via the normal constraint remap.
  *Deferred:* true shared-id merges (an optimization; revisit after incremental maintenance);
  connections drawn in *free* mode aren't captured (re-entering parametric rebuilds from
  geometry).
- **Stage 4 — Clean up `add_constraint`.** Replace the 230-line positional mega-match with one
  tested operand-resolution helper.
- **Stage 5 — Solver quality. ✅ FIRST PASS DONE.** (1) `Sketch::solve` rewritten as **adaptive
  Levenberg–Marquardt** — accept a step only if it lowers the residual, shrink λ toward
  Gauss–Newton on success / grow it on failure; removed the fixed λ + "take the full step
  anyway" hack (which could diverge). λ-damping also keeps the normal equations solvable under
  gauge freedom. (2) `Sketch::diagnose() -> SketchDiagnostics { dof, status, redundant }` —
  Gram–Schmidt over the Jacobian rows flags exactly which constraints are redundant; surfaced
  in the constraints panel (status line + redundant rows shown in red). *Deferred:* analytic
  Jacobian (FD works and is correct for the fiddly Angle/Tangent residuals; analytic is a perf
  optimization with the most bug-risk) and QR least-squares (normal equations + LM damping is
  fine for these small systems).
- **Stage 6 — (future) Exact-kernel reconnection.** Recover exact coordinates for the subset
  pinned by exact linear constraints after a numeric solve.

## Order
0 → 1 → 2 → 3 → 4, solver later. Stages 0–2 deliver the bulk of the cleanup.

## UX note
Exit discards constraints, so show a subtle one-time hint ("Parametric off — constraints
cleared") rather than a blocking dialog, keeping the revert instant.
