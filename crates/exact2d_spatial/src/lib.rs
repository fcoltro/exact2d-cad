// Phase 2.3 — Spatial index (stub; full implementation below)
pub mod quadtree;
pub mod morton;
pub use quadtree::{Quadtree, CellClass, QuadNode};
pub use morton::morton_code;
