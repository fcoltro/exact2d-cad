# Exact2D CAD

A from-scratch **2D CAD system built in Rust around an exact algebraic geometry
kernel**. Every curve is represented as an implicit polynomial equation
`f(x, y) = 0` with exact rational coefficients — no floating-point drift in the
core, no tessellation in the math. Intersections, offsets, and booleans are
computed symbolically (resultants, Sturm sequences, square-free factorization)
and only flattened to pixels for display.

## Workspace layout

| Crate | Responsibility |
|-------|----------------|
| `exact2d_integer` | Arbitrary-precision integers (num-bigint backend) |
| `exact2d_algebra` | `Rational`, univariate/bivariate polynomials, `AlgebraicNumber` |
| `exact2d_geometry` | Curve primitives (line, arc, ellipse, Bézier, polycurve), ops, transforms |
| `exact2d_spatial` | Adaptive quadtree + Morton-code spatial index |
| `exact2d_boolean` | Planar region boolean operations |
| `exact2d_document` | Document / layer / entity / block model |
| `exact2d_cad` | Snapping, selection, draw + edit (trim/extend/fillet/chamfer/offset/…) |
| `exact2d_constraint` | Parametric constraint solver (Gauss–Newton + DOF tracking) |
| `exact2d_io` | DXF, SVG, and native `.e2d` import/export (zero-loss rationals) |
| `exact2d_ui` | Headless app logic + egui view (icon ribbon, canvas, panels) |
| `apps/exact2d_app` | eframe GUI host + headless kernel demo |

## Build & run

```sh
cargo build --workspace
cargo test  --workspace

cargo run -p exact2d_app          # launch the interactive CAD window
cargo run -p exact2d_app -- demo  # headless algebraic-kernel demo
```

The interactive viewport uses the egui painter with adaptive, sub-pixel curve
tessellation — exactness lives in the kernel, not the rasterizer.

## Status

Implemented: the algebraic kernel, geometry engine, spatial index, boolean ops,
document model, snapping/selection, the full draw + modify toolset, a parametric
constraint solver, DXF/SVG/native I/O, and an egui application shell.

## License

**Proprietary — Copyright © 2026 Fabio Coltro. All Rights Reserved.** Not open
source; see [LICENSE](LICENSE). (Versions published up to tag `v0.2.0` were
released under the GPL-3.0 and remain available under that license for copies
already distributed.)
