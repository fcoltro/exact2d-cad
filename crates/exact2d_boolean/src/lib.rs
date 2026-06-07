// Phase 2.4 — Boolean operations (stub; full implementation below)
pub mod region;
pub mod boolean_ops;
pub use region::Region;
pub use boolean_ops::{union, intersection, difference, xor};
