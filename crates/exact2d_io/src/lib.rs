//! Phase 5 — File format & interoperability for Exact2D CAD.
//!
//! DXF (ASCII) and SVG import/export, mapping between the industry formats and
//! the exact algebraic document model.

pub mod dxf;
pub mod svg;
pub mod native;

pub use dxf::{import_dxf, export_dxf};
pub use svg::{import_svg, export_svg};
pub use native::{save as save_native, load as load_native, to_string as to_e2d, from_string as from_e2d};
