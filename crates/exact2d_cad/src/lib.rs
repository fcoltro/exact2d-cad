//! Phase 4.2–4.3 — CAD interaction engine: snapping, selection, and commands.
//!
//! Pure, headless logic that a GUI layer drives. Snaps and edits use the exact
//! algebraic geometry kernel; nothing here requires a window or GPU.

pub mod snap;
pub mod selection;
pub mod draw;
pub mod edit;
pub mod inquiry;

pub use snap::{SnapKind, SnapPoint, SnapSettings, find_snaps, best_snap};
pub use selection::{pick_at, select_window, select_crossing, select_fence, select_by};
pub use draw as commands;
