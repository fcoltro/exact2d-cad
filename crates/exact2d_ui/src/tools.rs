//! Interactive drawing/edit tools (spec §4.3 / §6.1). Each tool is a small state
//! machine that consumes clicked points and emits `ToolEvent`s. Tools are pure and
//! testable — they never touch the document directly; `AppState` applies the events.

use exact2d_algebra::Rational;
use exact2d_geometry::{Curve, LineSeg, CircularArc, Point2d, Transform2d, CubicBezier, PolyCurve};
use exact2d_document::{EntityKind, EntityId};

/// The active tool and its in-progress state.
///
/// Only ever one `Tool` exists at a time (the active tool), so the size spread
/// between variants is irrelevant — keeping the payloads inline is simpler than
/// boxing them.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug)]
pub enum Tool {
    /// Default: click to select.
    Select,
    /// Multi-segment line; each click after the first creates a segment.
    Line { last: Option<Point2d> },
    /// Center then a radius point.
    Circle { center: Option<Point2d> },
    /// Three points the arc passes through.
    Arc3 { pts: Vec<Point2d> },
    /// Two opposite corners.
    Rectangle { first: Option<Point2d> },
    /// Move pre-selected entities: base point then destination.
    Move { base: Option<Point2d>, ids: Vec<EntityId> },
    /// Copy pre-selected entities: base point then destination.
    Copy { base: Option<Point2d>, ids: Vec<EntityId> },
    /// Spline (Bezier curve): clicks specify control points.
    Spline { pts: Vec<Point2d> },
    /// Polyline: clicks add connected points, committed as a single PolyCurve.
    Polyline { pts: Vec<Point2d> },
    /// Polygon: center point, then radius/vertex point.
    Polygon { center: Option<Point2d>, sides: usize },
    /// Click points/entities to dimension, then click to place.
    Dimension { stage: usize, p1: Option<usize>, p2: Option<usize> },

    // ── Modify tools ────────────────────────────────────────────────────────
    /// Rotate the selection: base point, then a point giving the angle.
    Rotate { base: Option<Point2d>, ids: Vec<EntityId> },
    /// Scale the selection: base point, reference distance, then new distance.
    Scale { base: Option<Point2d>, reference: Option<f64>, ids: Vec<EntityId> },
    /// Mirror the selection across the axis defined by two clicked points.
    Mirror { first: Option<Point2d>, ids: Vec<EntityId> },
    /// Click curve pieces to trim against every other entity (handled in state).
    Trim,
    /// Click a curve end to extend it to the nearest boundary (handled in state).
    Extend,
    /// Pick a curve then a side to offset by `dist` (handled in state).
    Offset { dist: f64, source: Option<EntityId> },
    /// Pick two lines to round their corner with `radius` (handled in state).
    Fillet { radius: f64, first: Option<EntityId> },
    /// Pick two lines to bevel their corner by `dist` (handled in state).
    Chamfer { dist: f64, first: Option<EntityId> },
    /// Window two corners, then base→dest to stretch windowed vertices (in state).
    Stretch { c1: Option<Point2d>, c2: Option<Point2d>, base: Option<Point2d>, ids: Vec<EntityId> },
}

/// What a tool wants the application to do in response to a click.
#[allow(clippy::large_enum_variant)] // transient, one per click; inline is simplest
#[derive(Clone, Debug)]
pub enum ToolEvent {
    /// Nothing yet — the tool needs more input.
    Pending,
    /// Add these entities to the document.
    Create(Vec<EntityKind>),
    /// Apply a transform to existing entities (MOVE).
    Transform { ids: Vec<EntityId>, t: Transform2d },
    /// Add transformed copies of existing entities (COPY).
    CopyOf { ids: Vec<EntityId>, t: Transform2d },
}

impl Tool {
    pub fn name(&self) -> &'static str {
        match self {
            Tool::Select => "SELECT",
            Tool::Line { .. } => "LINE",
            Tool::Circle { .. } => "CIRCLE",
            Tool::Arc3 { .. } => "ARC",
            Tool::Rectangle { .. } => "RECTANGLE",
            Tool::Move { .. } => "MOVE",
            Tool::Copy { .. } => "COPY",
            Tool::Spline { .. } => "SPLINE",
            Tool::Polyline { .. } => "POLYLINE",
            Tool::Polygon { .. } => "POLYGON",
            Tool::Dimension { .. } => "DIMENSION",
            Tool::Rotate { .. } => "ROTATE",
            Tool::Scale { .. } => "SCALE",
            Tool::Mirror { .. } => "MIRROR",
            Tool::Trim => "TRIM",
            Tool::Extend => "EXTEND",
            Tool::Offset { .. } => "OFFSET",
            Tool::Fillet { .. } => "FILLET",
            Tool::Chamfer { .. } => "CHAMFER",
            Tool::Stretch { .. } => "STRETCH",
        }
    }

    /// Whether the tool keeps running after producing an event (e.g. LINE chains).
    pub fn is_continuous(&self) -> bool {
        matches!(self, Tool::Line { .. })
    }

    /// Feed a (possibly snapped) world point to the tool.
    pub fn on_point(&mut self, p: Point2d) -> ToolEvent {
        match self {
            Tool::Select | Tool::Dimension { .. } => ToolEvent::Pending,

            Tool::Line { last } => {
                let ev = match last.take() {
                    Some(prev) => ToolEvent::Create(vec![EntityKind::Curve(
                        Curve::Line(LineSeg::from_endpoints(prev, p.clone())))]),
                    None => ToolEvent::Pending,
                };
                *last = Some(p);
                ev
            }

            Tool::Circle { center } => match center.take() {
                None => { *center = Some(p); ToolEvent::Pending }
                Some(c) => {
                    let d = c.dist_f64(&p);
                    if d < 1e-9 {
                        // Radius point coincides with the center — keep waiting.
                        *center = Some(c);
                        ToolEvent::Pending
                    } else {
                        let r = Rational::from_f64_approx(d);
                        *self = Tool::Circle { center: None };
                        ToolEvent::Create(vec![EntityKind::Curve(Curve::Arc(
                            CircularArc::new(c, r, 0.0, std::f64::consts::TAU)))])
                    }
                }
            },

            Tool::Arc3 { pts } => {
                pts.push(p);
                if pts.len() == 3 {
                    let arc = CircularArc::from_three_points(&pts[0], &pts[1], &pts[2]);
                    *self = Tool::Arc3 { pts: vec![] };
                    match arc {
                        Some(a) => ToolEvent::Create(vec![EntityKind::Curve(Curve::Arc(a))]),
                        None => ToolEvent::Pending, // collinear → no arc
                    }
                } else {
                    ToolEvent::Pending
                }
            }

            Tool::Rectangle { first } => match first.take() {
                None => { *first = Some(p); ToolEvent::Pending }
                Some(c0) => {
                    *self = Tool::Rectangle { first: None };
                    ToolEvent::Create(rectangle_entities(&c0, &p))
                }
            },

            Tool::Move { base, ids } => match base.take() {
                None => { *base = Some(p); ToolEvent::Pending }
                Some(b) => {
                    let t = Transform2d::translation(p.x.clone() - b.x.clone(), p.y.clone() - b.y.clone());
                    let ids = std::mem::take(ids);
                    ToolEvent::Transform { ids, t }
                }
            },

            Tool::Copy { base, ids } => match base.take() {
                None => { *base = Some(p); ToolEvent::Pending }
                Some(b) => {
                    let t = Transform2d::translation(p.x.clone() - b.x.clone(), p.y.clone() - b.y.clone());
                    ToolEvent::CopyOf { ids: ids.clone(), t }
                }
            },

            Tool::Spline { pts } => {
                pts.push(p);
                if pts.len() == 4 {
                    let bezier = CubicBezier::new(
                        pts[0].clone(),
                        pts[1].clone(),
                        pts[2].clone(),
                        pts[3].clone(),
                    );
                    *self = Tool::Spline { pts: vec![] };
                    ToolEvent::Create(vec![EntityKind::Curve(Curve::Bezier(bezier))])
                } else {
                    ToolEvent::Pending
                }
            }

            Tool::Polyline { pts } => {
                pts.push(p);
                ToolEvent::Pending
            }

            Tool::Polygon { center, sides } => match center.take() {
                None => { *center = Some(p); ToolEvent::Pending }
                Some(c) => {
                    let cx = c.x.to_f64();
                    let cy = c.y.to_f64();
                    let rx = p.x.to_f64();
                    let ry = p.y.to_f64();
                    let dx = rx - cx;
                    let dy = ry - cy;
                    let r = (dx * dx + dy * dy).sqrt();
                    if r < 1e-9 {
                        *center = Some(c);
                        ToolEvent::Pending
                    } else {
                        let start_angle = dy.atan2(dx);
                        let n = *sides;
                        let mut verts = Vec::with_capacity(n);
                        for i in 0..n {
                            let angle = start_angle + (i as f64) * std::f64::consts::TAU / (n as f64);
                            verts.push(Point2d::from_f64(cx + r * angle.cos(), cy + r * angle.sin()));
                        }
                        let mut segments = Vec::new();
                        for i in 0..n {
                            segments.push(EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(
                                verts[i].clone(),
                                verts[(i + 1) % n].clone(),
                            ))));
                        }
                        *self = Tool::Polygon { center: None, sides: n };
                        ToolEvent::Create(segments)
                    }
                }
            }

            Tool::Rotate { base, ids } => match base.take() {
                None => { *base = Some(p); ToolEvent::Pending }
                Some(b) => {
                    let angle = (p.y.to_f64() - b.y.to_f64()).atan2(p.x.to_f64() - b.x.to_f64());
                    let t = Transform2d::rotation_about(&b, angle);
                    ToolEvent::Transform { ids: std::mem::take(ids), t }
                }
            },

            Tool::Scale { base, reference, ids } => match base.clone() {
                None => { *base = Some(p); ToolEvent::Pending }
                Some(b) => match *reference {
                    None => { *reference = Some(b.dist_f64(&p).max(1e-9)); ToolEvent::Pending }
                    Some(r1) => {
                        let factor = (b.dist_f64(&p) / r1).max(1e-9);
                        let s = Rational::from_f64_approx(factor);
                        let t = Transform2d::scale_about(&b, s.clone(), s);
                        ToolEvent::Transform { ids: std::mem::take(ids), t }
                    }
                },
            },

            Tool::Mirror { first, ids } => match first.take() {
                None => { *first = Some(p); ToolEvent::Pending }
                Some(f) => {
                    let t = Transform2d::mirror_line(&f, &p);
                    ToolEvent::Transform { ids: std::mem::take(ids), t }
                }
            },

            // Entity-picking modify tools resolve their picks in `AppState`, which
            // has document access; the pure tool state machine stays idle here.
            Tool::Trim | Tool::Extend | Tool::Offset { .. }
            | Tool::Fillet { .. } | Tool::Chamfer { .. } | Tool::Stretch { .. } => ToolEvent::Pending,
        }
    }

    /// Cancel any in-progress input (Esc).
    pub fn reset(&mut self) {
        match self {
            Tool::Line { last } => *last = None,
            Tool::Circle { center } => *center = None,
            Tool::Arc3 { pts } => pts.clear(),
            Tool::Rectangle { first } => *first = None,
            Tool::Move { base, .. } | Tool::Copy { base, .. } => *base = None,
            Tool::Spline { pts } => pts.clear(),
            Tool::Polyline { pts } => pts.clear(),
            Tool::Polygon { center, .. } => *center = None,
            Tool::Dimension { stage, p1, p2 } => {
                *stage = 0;
                *p1 = None;
                *p2 = None;
            }
            Tool::Rotate { base, .. } => *base = None,
            Tool::Scale { base, reference, .. } => { *base = None; *reference = None; }
            Tool::Mirror { first, .. } => *first = None,
            Tool::Offset { source, .. } => *source = None,
            Tool::Fillet { first, .. } => *first = None,
            Tool::Chamfer { first, .. } => *first = None,
            Tool::Stretch { c1, c2, base, .. } => { *c1 = None; *c2 = None; *base = None; }
            Tool::Trim | Tool::Extend | Tool::Select => {}
        }
    }

    /// Whether the tool currently holds a partial input (for status/preview).
    pub fn has_pending_input(&self) -> bool {
        match self {
            Tool::Line { last } => last.is_some(),
            Tool::Circle { center } => center.is_some(),
            Tool::Arc3 { pts } => !pts.is_empty(),
            Tool::Rectangle { first } => first.is_some(),
            Tool::Move { base, .. } | Tool::Copy { base, .. } => base.is_some(),
            Tool::Spline { pts } => !pts.is_empty(),
            Tool::Polyline { pts } => !pts.is_empty(),
            Tool::Polygon { center, .. } => center.is_some(),
            Tool::Dimension { stage, .. } => *stage > 0,
            Tool::Rotate { base, .. } => base.is_some(),
            Tool::Scale { base, .. } => base.is_some(),
            Tool::Mirror { first, .. } => first.is_some(),
            Tool::Offset { source, .. } => source.is_some(),
            Tool::Fillet { first, .. } => first.is_some(),
            Tool::Chamfer { first, .. } => first.is_some(),
            Tool::Stretch { c1, .. } => c1.is_some(),
            Tool::Trim | Tool::Extend | Tool::Select => false,
        }
    }

    /// Rubber-band preview geometry from the tool's pending input to `cursor`.
    /// Lets the canvas show what the next click will create.
    pub fn preview(&self, cursor: &Point2d) -> Vec<Curve> {
        match self {
            Tool::Line { last: Some(p) } =>
                vec![Curve::Line(LineSeg::from_endpoints(p.clone(), cursor.clone()))],
            Tool::Circle { center: Some(c) } => {
                let d = c.dist_f64(cursor);
                if d < 1e-9 {
                    vec![] // zero radius (cursor at center) — nothing to preview yet
                } else {
                    let r = Rational::from_f64_approx(d);
                    vec![Curve::Arc(CircularArc::new(c.clone(), r, 0.0, std::f64::consts::TAU))]
                }
            }
            Tool::Rectangle { first: Some(c0) } => rectangle_curves(c0, cursor),
            Tool::Arc3 { pts } if pts.len() == 1 =>
                vec![Curve::Line(LineSeg::from_endpoints(pts[0].clone(), cursor.clone()))],
            Tool::Arc3 { pts } if pts.len() == 2 => {
                match CircularArc::from_three_points(&pts[0], &pts[1], cursor) {
                    Some(a) => vec![Curve::Arc(a)],
                    None => vec![Curve::Line(LineSeg::from_endpoints(pts[1].clone(), cursor.clone()))],
                }
            }
            Tool::Move { base: Some(b), .. } | Tool::Copy { base: Some(b), .. }
            | Tool::Rotate { base: Some(b), .. } | Tool::Scale { base: Some(b), .. }
            | Tool::Mirror { first: Some(b), .. } | Tool::Stretch { base: Some(b), .. } =>
                vec![Curve::Line(LineSeg::from_endpoints(b.clone(), cursor.clone()))],
            Tool::Spline { pts } => {
                if pts.len() == 1 {
                    vec![Curve::Line(LineSeg::from_endpoints(pts[0].clone(), cursor.clone()))]
                } else if pts.len() == 2 {
                    vec![
                        Curve::Line(LineSeg::from_endpoints(pts[0].clone(), pts[1].clone())),
                        Curve::Line(LineSeg::from_endpoints(pts[1].clone(), cursor.clone())),
                    ]
                } else if pts.len() == 3 {
                    vec![Curve::Bezier(CubicBezier::new(
                        pts[0].clone(),
                        pts[1].clone(),
                        pts[2].clone(),
                        cursor.clone(),
                    ))]
                } else {
                    vec![]
                }
            }
            Tool::Polyline { pts } => {
                let mut curves = Vec::new();
                for i in 0..pts.len().saturating_sub(1) {
                    curves.push(Curve::Line(LineSeg::from_endpoints(pts[i].clone(), pts[i+1].clone())));
                }
                if let Some(last) = pts.last() {
                    curves.push(Curve::Line(LineSeg::from_endpoints(last.clone(), cursor.clone())));
                }
                curves
            }
            Tool::Polygon { center: Some(c), sides } => {
                let cx = c.x.to_f64();
                let cy = c.y.to_f64();
                let rx = cursor.x.to_f64();
                let ry = cursor.y.to_f64();
                let dx = rx - cx;
                let dy = ry - cy;
                let r = (dx * dx + dy * dy).sqrt();
                let start_angle = dy.atan2(dx);
                let n = *sides;
                let mut verts = Vec::with_capacity(n);
                for i in 0..n {
                    let angle = start_angle + (i as f64) * std::f64::consts::TAU / (n as f64);
                    verts.push(Point2d::from_f64(cx + r * angle.cos(), cy + r * angle.sin()));
                }
                let mut curves = Vec::new();
                for i in 0..n {
                    curves.push(Curve::Line(LineSeg::from_endpoints(
                        verts[i].clone(),
                        verts[(i + 1) % n].clone(),
                    )));
                }
                curves
            }
            _ => vec![],
        }
    }

    /// Retrieve the current snapping reference point if the tool is in progress.
    pub fn reference_point(&self) -> Option<Point2d> {
        match self {
            Tool::Line { last } => last.clone(),
            Tool::Circle { center } => center.clone(),
            Tool::Rectangle { first } => first.clone(),
            Tool::Arc3 { pts } => pts.last().cloned(),
            Tool::Move { base, .. } => base.clone(),
            Tool::Copy { base, .. } => base.clone(),
            Tool::Spline { pts } => pts.last().cloned(),
            Tool::Polyline { pts } => pts.last().cloned(),
            Tool::Polygon { center, .. } => center.clone(),
            Tool::Rotate { base, .. } => base.clone(),
            Tool::Scale { base, .. } => base.clone(),
            Tool::Mirror { first, .. } => first.clone(),
            Tool::Stretch { base, c1, .. } => base.clone().or_else(|| c1.clone()),
            Tool::Dimension { .. }
            | Tool::Trim | Tool::Extend
            | Tool::Offset { .. } | Tool::Fillet { .. } | Tool::Chamfer { .. } => None,
            Tool::Select => None,
        }
    }

    /// Commit the current polyline as a single PolyCurve entity.
    pub fn commit(&mut self) -> ToolEvent {
        match self {
            Tool::Polyline { pts } => {
                if pts.len() >= 2 {
                    let mut segments = Vec::new();
                    for i in 0..pts.len() - 1 {
                        segments.push(Curve::Line(LineSeg::from_endpoints(pts[i].clone(), pts[i+1].clone())));
                    }
                    let poly = PolyCurve::new(segments);
                    *self = Tool::Polyline { pts: Vec::new() };
                    ToolEvent::Create(vec![EntityKind::Curve(Curve::Poly(Box::new(poly)))])
                } else {
                    *self = Tool::Polyline { pts: Vec::new() };
                    ToolEvent::Pending
                }
            }
            _ => ToolEvent::Pending,
        }
    }

    /// Close the current polyline (connecting back to the start point) and commit it.
    pub fn close_and_commit(&mut self) -> ToolEvent {
        match self {
            Tool::Polyline { pts } => {
                if pts.len() >= 2 {
                    let mut segments = Vec::new();
                    for i in 0..pts.len() - 1 {
                        segments.push(Curve::Line(LineSeg::from_endpoints(pts[i].clone(), pts[i+1].clone())));
                    }
                    // Connect back to the first point!
                    segments.push(Curve::Line(LineSeg::from_endpoints(pts.last().unwrap().clone(), pts[0].clone())));
                    let poly = PolyCurve::new(segments);
                    *self = Tool::Polyline { pts: Vec::new() };
                    ToolEvent::Create(vec![EntityKind::Curve(Curve::Poly(Box::new(poly)))])
                } else {
                    *self = Tool::Polyline { pts: Vec::new() };
                    ToolEvent::Pending
                }
            }
            _ => ToolEvent::Pending,
        }
    }
}

/// The 4 line segments of an axis-aligned rectangle from opposite corners.
fn rectangle_curves(c0: &Point2d, c1: &Point2d) -> Vec<Curve> {
    let (x0, x1) = order(c0.x.clone(), c1.x.clone());
    let (y0, y1) = order(c0.y.clone(), c1.y.clone());
    let p = |x: &Rational, y: &Rational| Point2d::new(x.clone(), y.clone());
    let corners = [p(&x0, &y0), p(&x1, &y0), p(&x1, &y1), p(&x0, &y1)];
    (0..4).map(|i| Curve::Line(
        LineSeg::from_endpoints(corners[i].clone(), corners[(i + 1) % 4].clone()))).collect()
}

fn rectangle_entities(c0: &Point2d, c1: &Point2d) -> Vec<EntityKind> {
    rectangle_curves(c0, c1).into_iter().map(EntityKind::Curve).collect()
}

fn order(a: Rational, b: Rational) -> (Rational, Rational) {
    if a <= b { (a, b) } else { (b, a) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn line_tool_chains_segments() {
        let mut t = Tool::Line { last: None };
        assert!(matches!(t.on_point(pt(0, 0)), ToolEvent::Pending)); // first click: anchor
        // second click creates a segment
        match t.on_point(pt(5, 0)) {
            ToolEvent::Create(es) => assert_eq!(es.len(), 1),
            o => panic!("{:?}", o),
        }
        // third click chains another segment
        assert!(matches!(t.on_point(pt(5, 5)), ToolEvent::Create(_)));
        assert!(t.is_continuous());
    }

    #[test]
    fn circle_tool_center_radius() {
        let mut t = Tool::Circle { center: None };
        assert!(matches!(t.on_point(pt(0, 0)), ToolEvent::Pending));
        match t.on_point(pt(3, 4)) {
            ToolEvent::Create(es) => {
                assert_eq!(es.len(), 1);
                if let EntityKind::Curve(Curve::Arc(a)) = &es[0] {
                    assert!((a.radius.to_f64() - 5.0).abs() < 1e-6); // 3-4-5
                } else { panic!() }
            }
            o => panic!("{:?}", o),
        }
    }

    #[test]
    fn rectangle_tool_makes_four_sides() {
        let mut t = Tool::Rectangle { first: None };
        assert!(matches!(t.on_point(pt(0, 0)), ToolEvent::Pending));
        match t.on_point(pt(4, 3)) {
            ToolEvent::Create(es) => assert_eq!(es.len(), 4),
            o => panic!("{:?}", o),
        }
    }

    #[test]
    fn move_tool_emits_translation() {
        let ids = vec![EntityId(1), EntityId(2)];
        let mut t = Tool::Move { base: None, ids: ids.clone() };
        assert!(matches!(t.on_point(pt(0, 0)), ToolEvent::Pending));
        match t.on_point(pt(10, 5)) {
            ToolEvent::Transform { ids: got, t } => {
                assert_eq!(got, ids);
                // translation (10,5)
                assert_eq!(t.apply_point(&pt(0,0)), pt(10, 5));
            }
            o => panic!("{:?}", o),
        }
    }

    #[test]
    fn copy_tool_emits_copy() {
        let mut t = Tool::Copy { base: None, ids: vec![EntityId(7)] };
        t.on_point(pt(1, 1));
        assert!(matches!(t.on_point(pt(4, 1)), ToolEvent::CopyOf { .. }));
    }

    #[test]
    fn arc3_needs_three_points() {
        let mut t = Tool::Arc3 { pts: vec![] };
        assert!(matches!(t.on_point(pt(1, 0)), ToolEvent::Pending));
        assert!(matches!(t.on_point(pt(0, 1)), ToolEvent::Pending));
        assert!(matches!(t.on_point(pt(-1, 0)), ToolEvent::Create(_)));
    }

    /// The rubber-band preview (two points placed, cursor = prospective end) must
    /// build the SAME arc that committing the third click produces — otherwise the
    /// drawn arc differs from what the user saw while aiming (the "opposite-side
    /// near-full circle" bug).
    #[test]
    fn arc3_preview_matches_commit() {
        let start = pt(1, 0);
        let mid = pt(0, 1);
        let end = pt(-1, 0);

        // Preview after start+mid are placed, cursor hovering at `end`.
        let prev = Tool::Arc3 { pts: vec![start.clone(), mid.clone()] };
        let preview = prev.preview(&end);
        let pa = match preview.as_slice() {
            [Curve::Arc(a)] => a.clone(),
            other => panic!("expected one arc in preview, got {:?}", other),
        };

        // Commit: three clicks in the same order.
        let mut t = Tool::Arc3 { pts: vec![] };
        t.on_point(start);
        t.on_point(mid);
        let committed = match t.on_point(end) {
            ToolEvent::Create(es) => match es.as_slice() {
                [EntityKind::Curve(Curve::Arc(a))] => a.clone(),
                other => panic!("expected one arc, got {:?}", other),
            },
            o => panic!("{:?}", o),
        };

        // Same centre, radius, and angular sweep.
        assert!((pa.center.to_f64().0 - committed.center.to_f64().0).abs() < 1e-9);
        assert!((pa.center.to_f64().1 - committed.center.to_f64().1).abs() < 1e-9);
        assert!((pa.start_angle - committed.start_angle).abs() < 1e-9);
        assert!((pa.end_angle - committed.end_angle).abs() < 1e-9);
        // And it's the minor (upper) semicircle, not the opposite major arc.
        assert!((pa.included_angle() - std::f64::consts::PI).abs() < 1e-6,
            "expected a 180° arc, got {}", pa.included_angle());
    }

    #[test]
    fn reset_clears_partial() {
        let mut t = Tool::Line { last: None };
        t.on_point(pt(0, 0));
        assert!(t.has_pending_input());
        t.reset();
        assert!(!t.has_pending_input());
    }

    #[test]
    fn polygon_creates_regular_polygon() {
        let mut t = Tool::Polygon { center: None, sides: 5 };
        assert!(matches!(t.on_point(pt(0, 0)), ToolEvent::Pending));
        match t.on_point(pt(10, 0)) {
            ToolEvent::Create(es) => {
                assert_eq!(es.len(), 5);
                for (i, entity) in es.iter().enumerate() {
                    if let EntityKind::Curve(Curve::Line(l)) = entity {
                        // Just verifying it exists and is a LineSeg
                        if i == 0 {
                            assert!((l.p0.x.to_f64() - 10.0).abs() < 1e-6);
                            assert!(l.p0.y.to_f64().abs() < 1e-6);
                        }
                    } else {
                        panic!("expected Line segment");
                    }
                }
            }
            o => panic!("{:?}", o),
        }
    }

    #[test]
    fn spline_needs_four_points() {
        let mut t = Tool::Spline { pts: vec![] };
        assert!(matches!(t.on_point(pt(0, 0)), ToolEvent::Pending));
        assert!(matches!(t.on_point(pt(5, 5)), ToolEvent::Pending));
        assert!(matches!(t.on_point(pt(10, -5)), ToolEvent::Pending));
        match t.on_point(pt(15, 0)) {
            ToolEvent::Create(es) => {
                assert_eq!(es.len(), 1);
                assert!(matches!(es[0], EntityKind::Curve(Curve::Bezier(_))));
            }
            o => panic!("{:?}", o),
        }
    }

    #[test]
    fn polyline_accumulates_and_commits() {
        let mut t = Tool::Polyline { pts: vec![] };
        assert!(matches!(t.on_point(pt(0, 0)), ToolEvent::Pending));
        assert!(matches!(t.on_point(pt(5, 5)), ToolEvent::Pending));
        assert!(matches!(t.on_point(pt(10, 0)), ToolEvent::Pending));
        
        // Commit
        match t.commit() {
            ToolEvent::Create(es) => {
                assert_eq!(es.len(), 1);
                if let EntityKind::Curve(Curve::Poly(poly)) = &es[0] {
                    assert_eq!(poly.segments.len(), 2);
                } else {
                    panic!("expected PolyCurve");
                }
            }
            o => panic!("{:?}", o),
        }
    }

    #[test]
    fn polyline_accumulates_and_closes() {
        let mut t = Tool::Polyline { pts: vec![] };
        assert!(matches!(t.on_point(pt(0, 0)), ToolEvent::Pending));
        assert!(matches!(t.on_point(pt(5, 5)), ToolEvent::Pending));
        assert!(matches!(t.on_point(pt(10, 0)), ToolEvent::Pending));
        
        // Close and Commit
        match t.close_and_commit() {
            ToolEvent::Create(es) => {
                assert_eq!(es.len(), 1);
                if let EntityKind::Curve(Curve::Poly(poly)) = &es[0] {
                    // 2 segments between the 3 points, plus 1 closing segment back to start
                    assert_eq!(poly.segments.len(), 3);
                } else {
                    panic!("expected PolyCurve");
                }
            }
            o => panic!("{:?}", o),
        }
    }
}
