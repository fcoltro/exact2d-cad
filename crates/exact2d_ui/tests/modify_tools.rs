//! Integration tests for the interactive modify tools, driven through the public
//! `AppState` API exactly as the egui canvas drives them (command + canvas clicks).

use exact2d_ui::AppState;
use exact2d_document::EntityKind;
use exact2d_geometry::{Curve, LineSeg, Point2d};

fn line(x0: i64, y0: i64, x1: i64, y1: i64) -> EntityKind {
    EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(
        Point2d::from_i64(x0, y0), Point2d::from_i64(x1, y1))))
}

fn click(a: &mut AppState, wx: f64, wy: f64) {
    let (sx, sy) = a.view.world_to_screen(wx, wy);
    a.canvas_click(sx, sy);
}

fn app() -> AppState {
    let mut a = AppState::new(1200.0, 800.0);
    a.snap_on = false; // tests place exact world points
    a
}

#[test]
fn trim_tool_cuts_picked_span() {
    let mut a = app();
    a.add_entity(line(0, 0, 10, 0));   // target
    a.add_entity(line(3, -1, 3, 1));   // cutter
    a.add_entity(line(7, -1, 7, 1));   // cutter
    let before = a.document.len();
    a.run_command("TRIM");
    click(&mut a, 5.0, 0.0);           // pick the middle span
    // The middle span is removed → the target becomes two outer pieces (+1 entity).
    assert_eq!(a.document.len(), before + 1, "trim should split target into two");
}

/// Regression (user report): with OSNAP on, the snap magnet pulled trim picks
/// onto cutter intersections/endpoints, so pieces near a previous cut became
/// un-trimmable. Entity-picking tools must bypass object snap entirely.
#[test]
fn trim_ignores_object_snap_when_picking() {
    let mut a = AppState::new(1200.0, 800.0);
    a.snap_on = true; // deliberately ON — picking must not be affected
    a.add_entity(line(0, 0, 10, 0));
    a.add_entity(line(3, -1, 3, 1));
    a.add_entity(line(7, -1, 7, 1));
    let before = a.document.len();
    a.run_command("TRIM");
    // Hover near the x=3 crossing: this used to magnet the cursor onto the
    // intersection and corrupt the subsequent pick.
    let (sx, sy) = a.view.world_to_screen(3.1, 0.05);
    a.pointer_moved(sx, sy);
    assert!(a.active_snap.is_none(), "entity-picking tools must not object-snap");
    click(&mut a, 5.0, 0.0);
    assert_eq!(a.document.len(), before + 1, "trim must still cut the picked span");
}

#[test]
fn offset_tool_creates_parallel_curve() {
    let mut a = app();
    a.add_entity(line(0, 0, 10, 0));
    let before = a.document.len();
    a.run_command("OFFSET");
    a.run_command("2");                // set distance = 2
    click(&mut a, 5.0, 0.0);           // pick source curve
    click(&mut a, 5.0, 4.0);           // pick the side
    assert_eq!(a.document.len(), before + 1, "offset should add one parallel curve");
}

#[test]
fn fillet_tool_adds_arc() {
    let mut a = app();
    a.add_entity(line(10, 0, 0, 0));
    a.add_entity(line(0, 0, 0, 10));
    let before = a.document.len();
    a.run_command("FILLET");
    a.run_command("2");                // radius = 2
    click(&mut a, 5.0, 0.0);           // first line
    click(&mut a, 0.0, 5.0);           // second line
    assert_eq!(a.document.len(), before + 1, "fillet adds one arc (lines trimmed in place)");
    assert!(a.document.iter().any(|e| matches!(&e.kind, EntityKind::Curve(Curve::Arc(_)))),
        "a fillet arc should exist");
}

#[test]
fn rotate_tool_turns_selection() {
    let mut a = app();
    let id = a.add_entity(line(1, 0, 2, 0));
    a.selection = vec![id];
    a.run_command("ROTATE");
    click(&mut a, 0.0, 0.0);           // base point
    click(&mut a, 0.0, 1.0);           // 90° direction
    if let Some(Curve::Line(l)) = a.document.get(id).unwrap().as_curve() {
        assert!(l.p0.x.abs() < 1e-4 && (l.p0.y - 1.0).abs() < 1e-4,
            "(1,0) → (0,1), got {:?}", l.p0.to_f64());
    } else { panic!("expected a line") }
}

#[test]
fn mirror_tool_reflects_selection() {
    let mut a = app();
    let id = a.add_entity(line(1, 2, 3, 4));
    a.selection = vec![id];
    a.run_command("MIRROR");
    click(&mut a, 0.0, 0.0);           // axis point 1
    click(&mut a, 1.0, 0.0);           // axis point 2 → mirror across x-axis
    if let Some(Curve::Line(l)) = a.document.get(id).unwrap().as_curve() {
        let (x, y) = l.p0.to_f64();
        assert!((x - 1.0).abs() < 1e-4 && (y + 2.0).abs() < 1e-4, "(1,2) → (1,-2), got ({x},{y})");
    } else { panic!("expected a line") }
}

