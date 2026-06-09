//! Command-line parsing and dispatch (spec §4.2 dynamic input, §4.3 commands).
//!
//! Parses an AutoCAD-style command string into a `Command` the `AppState` executes.
//! Drawing commands activate a tool; others are immediate actions.

use crate::tools::Tool;

#[allow(clippy::large_enum_variant)] // Activate(Tool) dominates; commands are transient
#[derive(Clone, Debug)]
pub enum Command {
    /// Start an interactive tool.
    Activate(Tool),
    /// ZOOM extents — frame all geometry.
    ZoomExtents,
    /// ZOOM <scale> — set absolute zoom.
    ZoomScale(f64),
    /// UNDO / REDO.
    Undo,
    Redo,
    /// ERASE selected entities.
    Erase,
    /// Set the current layer.
    LayerSet(String),
    /// Create a new layer.
    LayerNew(String),
    /// Select all entities.
    SelectAll,
    /// Cancel current operation (Esc / blank).
    Cancel,
    /// Unrecognized input.
    Unknown(String),
}

/// A typed coordinate, in AutoCAD's input grammar.
///
/// - `x,y`     → [`CoordInput::Absolute`]
/// - `@dx,dy`  → [`CoordInput::Relative`] (offset from the last point)
/// - `d<a`     → [`CoordInput::PolarAbsolute`] (distance/angle from origin)
/// - `@d<a`    → [`CoordInput::PolarRelative`] (distance/angle from the last point)
///
/// Angles are in degrees, CCW from +X (as AutoCAD).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CoordInput {
    Absolute(f64, f64),
    Relative(f64, f64),
    PolarAbsolute { dist: f64, angle_deg: f64 },
    PolarRelative { dist: f64, angle_deg: f64 },
}

/// Parse a typed coordinate, or `None` if `input` isn't one (e.g. a bare number
/// or a command verb). Recognises a coordinate by the `,` or `<` separator, with
/// an optional leading `@` for relative entry.
pub fn parse_coordinate(input: &str) -> Option<CoordInput> {
    let s = input.trim();
    if s.is_empty() { return None; }
    let (relative, body) = match s.strip_prefix('@') {
        Some(rest) => (true, rest.trim()),
        None => (false, s),
    };
    // Polar: distance<angle
    if let Some((d, a)) = body.split_once('<') {
        let dist = d.trim().parse::<f64>().ok()?;
        let angle_deg = a.trim().parse::<f64>().ok()?;
        return Some(if relative {
            CoordInput::PolarRelative { dist, angle_deg }
        } else {
            CoordInput::PolarAbsolute { dist, angle_deg }
        });
    }
    // Cartesian: x,y
    if let Some((x, y)) = body.split_once(',') {
        let xv = x.trim().parse::<f64>().ok()?;
        let yv = y.trim().parse::<f64>().ok()?;
        return Some(if relative {
            CoordInput::Relative(xv, yv)
        } else {
            CoordInput::Absolute(xv, yv)
        });
    }
    None
}

/// Parse a command string. Command names are matched case-insensitively and
/// accept common AutoCAD aliases (L, C, REC, M, CO, E, U).
pub fn parse_command(input: &str) -> Command {
    let trimmed = input.trim();
    if trimmed.is_empty() { return Command::Cancel; }

    let mut parts = trimmed.split_whitespace();
    let verb = parts.next().unwrap_or("").to_ascii_uppercase();
    let rest: Vec<&str> = parts.collect();

    match verb.as_str() {
        "LINE" | "L"            => Command::Activate(Tool::Line { last: None }),
        "CIRCLE" | "C"          => Command::Activate(Tool::Circle { center: None }),
        "ARC" | "A"             => Command::Activate(Tool::Arc3 { pts: vec![] }),
        "RECTANGLE" | "REC" | "RECTANG"
                                => Command::Activate(Tool::Rectangle { first: None }),
        "MOVE" | "M"            => Command::Activate(Tool::Move { base: None, ids: vec![] }),
        "COPY" | "CO" | "CP"    => Command::Activate(Tool::Copy { base: None, ids: vec![] }),
        "POLYGON" | "POL"       => {
            let sides = rest.first()
                .and_then(|s| s.parse::<usize>().ok())
                .unwrap_or(4)
                .max(3);
            Command::Activate(Tool::Polygon { center: None, sides })
        }
        "SPLINE" | "SPL"        => Command::Activate(Tool::Spline { pts: vec![] }),
        "POLYLINE" | "PLINE" | "PL"
                                => Command::Activate(Tool::Polyline { pts: vec![] }),
        "SELECT" | "SE"         => Command::Activate(Tool::Select),
        // Smart Dimension is hidden for now (see project notes) — revisit later.
        "TEXT" | "T" | "DT" | "DTEXT" | "MTEXT" | "MT"
                                => Command::Activate(Tool::Text { anchor: None, height: 2.5 }),
        "ROTATE" | "RO"         => Command::Activate(Tool::Rotate { base: None, ids: vec![] }),
        "SCALE" | "SC"          => Command::Activate(Tool::Scale { base: None, reference: None, ids: vec![] }),
        "MIRROR" | "MI"         => Command::Activate(Tool::Mirror { first: None, ids: vec![] }),
        "TRIM" | "TR"           => Command::Activate(Tool::Trim),
        "EXTEND" | "EX"         => Command::Activate(Tool::Extend),
        "OFFSET" | "O"          => {
            let dist = rest.first().and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0);
            Command::Activate(Tool::Offset { dist, source: None })
        }
        "FILLET" | "F"          => {
            let radius = rest.first().and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0);
            Command::Activate(Tool::Fillet { radius, first: None })
        }
        "CHAMFER" | "CHA"       => {
            let dist = rest.first().and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0);
            Command::Activate(Tool::Chamfer { dist, first: None })
        }
        "STRETCH" | "S"         => Command::Activate(Tool::Stretch { c1: None, c2: None, base: None, ids: vec![] }),
        "ERASE" | "E" | "DELETE"=> Command::Erase,
        "UNDO" | "U"            => Command::Undo,
        "REDO"                  => Command::Redo,
        "ALL"                   => Command::SelectAll,
        "ZOOM" | "Z"            => parse_zoom(&rest),
        "LAYER" | "LA"          => parse_layer(&rest),
        // Parametric constraints (CON …) are hidden for now — revisit later.
        _                       => Command::Unknown(trimmed.to_string()),
    }
}

fn parse_zoom(rest: &[&str]) -> Command {
    match rest.first().map(|s| s.to_ascii_uppercase()) {
        Some(s) if s == "E" || s == "EXTENTS" => Command::ZoomExtents,
        Some(s) => match s.parse::<f64>() {
            Ok(scale) if scale > 0.0 => Command::ZoomScale(scale),
            _ => Command::ZoomExtents,
        },
        None => Command::ZoomExtents,
    }
}

fn parse_layer(rest: &[&str]) -> Command {
    match (rest.first().map(|s| s.to_ascii_uppercase()), rest.get(1)) {
        (Some(s), Some(name)) if s == "S" || s == "SET" => Command::LayerSet((*name).to_string()),
        (Some(s), Some(name)) if s == "N" || s == "NEW" || s == "M" || s == "MAKE"
            => Command::LayerNew((*name).to_string()),
        _ => Command::Unknown("LAYER".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_drawing_commands() {
        assert!(matches!(parse_command("LINE"), Command::Activate(Tool::Line { .. })));
        assert!(matches!(parse_command("l"), Command::Activate(Tool::Line { .. })));
        assert!(matches!(parse_command("CIRCLE"), Command::Activate(Tool::Circle { .. })));
        assert!(matches!(parse_command("rec"), Command::Activate(Tool::Rectangle { .. })));
        assert!(matches!(parse_command("M"), Command::Activate(Tool::Move { .. })));
        assert!(matches!(parse_command("POLYGON"), Command::Activate(Tool::Polygon { sides: 4, .. })));
        assert!(matches!(parse_command("POL 6"), Command::Activate(Tool::Polygon { sides: 6, .. })));
        assert!(matches!(parse_command("SPLINE"), Command::Activate(Tool::Spline { .. })));
        assert!(matches!(parse_command("spl"), Command::Activate(Tool::Spline { .. })));
        assert!(matches!(parse_command("POLYLINE"), Command::Activate(Tool::Polyline { .. })));
        assert!(matches!(parse_command("pl"), Command::Activate(Tool::Polyline { .. })));
    }

    #[test]
    fn parses_zoom() {
        assert!(matches!(parse_command("ZOOM E"), Command::ZoomExtents));
        assert!(matches!(parse_command("zoom extents"), Command::ZoomExtents));
        assert!(matches!(parse_command("Z 2.5"), Command::ZoomScale(s) if (s - 2.5).abs() < 1e-9));
        assert!(matches!(parse_command("ZOOM"), Command::ZoomExtents));
    }

    #[test]
    fn parses_layer() {
        assert!(matches!(parse_command("LAYER SET walls"), Command::LayerSet(n) if n == "walls"));
        assert!(matches!(parse_command("la new hidden"), Command::LayerNew(n) if n == "hidden"));
    }

    #[test]
    fn constraint_commands_are_disabled_for_now() {
        // Parametric is hidden; CON … no longer maps to constraint commands.
        assert!(matches!(parse_command("CON H"), Command::Unknown(_)));
        assert!(matches!(parse_command("CONSTRAINTS"), Command::Unknown(_)));
    }

    #[test]
    fn parses_coordinates() {
        assert_eq!(parse_coordinate("10,20"), Some(CoordInput::Absolute(10.0, 20.0)));
        assert_eq!(parse_coordinate("  3.5 , -4 "), Some(CoordInput::Absolute(3.5, -4.0)));
        assert_eq!(parse_coordinate("@10,20"), Some(CoordInput::Relative(10.0, 20.0)));
        assert_eq!(parse_coordinate("@-2.5,0"), Some(CoordInput::Relative(-2.5, 0.0)));
        assert_eq!(parse_coordinate("5<90"), Some(CoordInput::PolarAbsolute { dist: 5.0, angle_deg: 90.0 }));
        assert_eq!(parse_coordinate("@12<45"), Some(CoordInput::PolarRelative { dist: 12.0, angle_deg: 45.0 }));
        // Not coordinates:
        assert_eq!(parse_coordinate("10"), None);
        assert_eq!(parse_coordinate("LINE"), None);
        assert_eq!(parse_coordinate(""), None);
        assert_eq!(parse_coordinate("@5"), None);
        assert_eq!(parse_coordinate("a,b"), None);
    }

    #[test]
    fn parses_actions_and_unknown() {
        assert!(matches!(parse_command("UNDO"), Command::Undo));
        assert!(matches!(parse_command("u"), Command::Undo));
        assert!(matches!(parse_command("ERASE"), Command::Erase));
        assert!(matches!(parse_command("ALL"), Command::SelectAll));
        assert!(matches!(parse_command(""), Command::Cancel));
        assert!(matches!(parse_command("FLERP"), Command::Unknown(_)));
    }
}
