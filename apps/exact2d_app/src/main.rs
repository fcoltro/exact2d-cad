//! Exact2D CAD entry point.
//!
//! - `exact2d_app`        → launches the egui CAD application (needs a display).
//! - `exact2d_app demo`   → runs the algebraic-kernel demo (works headless).

use exact2d_algebra::{BivariatePoly, Rational};
use exact2d_ui::{AppState, UiState, draw_ui, egui};

fn main() {
    // Capture any panic to the log file (the console may flash and close).
    std::panic::set_hook(Box::new(|info| {
        log_init();
        log(&format!("PANIC: {info}"));
    }));

    match std::env::args().nth(1).as_deref() {
        Some("demo") | Some("cli") | Some("--demo") => {
            run_demo();
        }
        _ => {
            log_init();
            if let Err(e) = run_gui() {
                log(&format!("GUI failed to start ({e}). Running the kernel demo instead."));
                run_demo();
            }
        }
    }
}

// ── Logging (so a flash-and-close crash leaves a trace) ───────────────────────

fn log_path() -> std::path::PathBuf {
    std::env::current_exe().ok()
        .and_then(|p| p.parent().map(|d| d.to_path_buf()))
        .unwrap_or_else(std::env::temp_dir)
        .join("exact2d_log.txt")
}

fn log_init() {
    let _ = std::fs::write(log_path(), "Exact2D CAD log\n===============\n");
}

fn log(msg: &str) {
    use std::io::Write;
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(log_path()) {
        let _ = writeln!(f, "{msg}");
    }
    eprintln!("{msg}");
}

// ── GUI host (Phase 6) ────────────────────────────────────────────────────────

struct Exact2dCad {
    app: AppState,
    ui: UiState,
}

impl eframe::App for Exact2dCad {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // The viewport is the egui painter with adaptive, zoom-aware tessellation:
        // smooth at any zoom, dependency-free, and exact where it matters (the
        // algebraic kernel), tessellated only for display.
        draw_ui(ctx, &mut self.app, &mut self.ui);
    }
}

fn run_gui() -> eframe::Result<()> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "Exact2D CAD",
        options,
        Box::new(|_cc| {
            log("Window created. Using the adaptive-tessellation egui painter.");
            Ok(Box::new(Exact2dCad {
                app: AppState::new(1200.0, 800.0),
                ui: UiState::default(),
            }))
        }),
    )
}

// ── Kernel demo (Phase 1, headless) ───────────────────────────────────────────

fn run_demo() {
    println!("=== Exact2D CAD — Algebraic Kernel Demo ===\n");

    // Line:   3x + 4y - 5 = 0
    // Circle: x² + y² - 25 = 0
    let line = BivariatePoly::from_terms(&[
        ((1, 0), Rational::from(3i32)),
        ((0, 1), Rational::from(4i32)),
        ((0, 0), Rational::from(-5i32)),
    ]);
    let circle = BivariatePoly::from_terms(&[
        ((2, 0), Rational::from(1i32)),
        ((0, 2), Rational::from(1i32)),
        ((0, 0), Rational::from(-25i32)),
    ]);

    println!("Curve 1 (line):   3x + 4y - 5 = 0");
    println!("Curve 2 (circle): x² + y² - 25 = 0\n");

    match line.intersect(&circle) {
        Ok(intersections) => {
            println!("Found {} intersection point(s) via exact algebraic computation:\n",
                intersections.len());
            for (i, (x_alg, y_alg)) in intersections.iter().enumerate() {
                let x = x_alg.to_f64(1e-12);
                let y = y_alg.to_f64(1e-12);
                println!("  Point {}: x = {:.10},  y = {:.10}", i + 1, x, y);
                let line_err = (3.0 * x + 4.0 * y - 5.0).abs();
                let circle_err = (x * x + y * y - 25.0).abs();
                println!("    Residual on line:   {:.2e}", line_err);
                println!("    Residual on circle: {:.2e}", circle_err);
            }
        }
        Err(e) => println!("Error: {}", e),
    }

    println!("\nAll operations performed with exact rational arithmetic.");
    println!("Run `exact2d_app` (no args) to launch the interactive CAD application.");
}
