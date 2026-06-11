//! Phase 6 — UI for Exact2D CAD.
//!
//! Split into two layers:
//!   - **Headless application logic** (`state`, `tools`, `command`, `history`,
//!     `view_transform`) — pure Rust, fully unit-tested, no windowing/GPU deps.
//!   - **egui view** (`view`) — renders the spec's window layout (menu bar, ribbon,
//!     panels, canvas, command line, status bar) by reading/driving `AppState`.
//!
//! A windowing host (`exact2d_app`, via eframe) wires the view to a real window;
//! in a headless environment the logic layer is exercised directly by tests.

pub mod view_transform;
pub mod tools;
pub mod command;
pub mod history;
pub mod state;
pub mod view;
pub mod icons;
pub mod theme;

pub use view_transform::ViewTransform;
pub use tools::{Tool, ToolEvent};
pub use command::{Command, parse_command};
pub use history::History;
pub use state::AppState;
pub use view::{draw_ui, UiState};

// Re-export egui so a host binary uses the exact same version as the view layer.
pub use egui;
