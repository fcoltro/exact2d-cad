//! Entities (spec §4.1): geometry + properties, with non-destructive creation
//! parameters retained for parametric-like editing.

use exact2d_geometry::{Curve, CurveSegment, BoundingBox, Transform2d, Point2d};
use crate::properties::{Color, LineWeight, LineTypeRef, XData};

/// Stable identifier for an entity within a document.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EntityId(pub u64);

/// The geometric/annotation content of an entity.
///
/// Curve entities wrap the exact algebraic `Curve`. Annotations and construction
/// entities carry their own data. `Point` is a node entity (snappable).
#[derive(Clone, Debug)]
pub enum EntityKind {
    /// Any curve primitive (Line, Arc, Circle, Ellipse, Bezier/Spline, Polyline).
    Curve(Curve),
    /// A point/node entity.
    Point(Point2d),
    /// Single-line text anchored at a point, with height and rotation (radians).
    Text { anchor: Point2d, content: String, height: f64, rotation: f64 },
    /// Construction line through a point with a direction (infinite both ways).
    XLine { through: Point2d, dir: (f64, f64) },
    /// Ray from a point in a direction (infinite one way).
    Ray { from: Point2d, dir: (f64, f64) },
    /// Block insertion: references a block by name, with an insertion transform.
    Insert { block: String, transform: Transform2d },
}

/// A drawing entity: content + display properties + provenance.
#[derive(Clone, Debug)]
pub struct Entity {
    pub id: EntityId,
    pub kind: EntityKind,
    /// Index into the document's layer table.
    pub layer: usize,
    pub color: Color,
    pub line_type: LineTypeRef,
    pub line_weight: LineWeight,
    /// 0.0 = opaque, 1.0 = fully transparent.
    pub transparency: f64,
    pub xdata: XData,
}

impl Entity {
    pub fn new(id: EntityId, kind: EntityKind, layer: usize) -> Self {
        Entity {
            id, kind, layer,
            color: Color::ByLayer,
            line_type: LineTypeRef::ByLayer,
            line_weight: LineWeight::ByLayer,
            transparency: 0.0,
            xdata: XData::default(),
        }
    }

    /// Axis-aligned bounding box (None for infinite construction lines).
    pub fn bounding_box(&self) -> Option<BoundingBox> {
        match &self.kind {
            EntityKind::Curve(c) => Some(c.bounding_box()),
            EntityKind::Point(p) => Some(BoundingBox::new(*p, *p)),
            EntityKind::Text { anchor, height, content, .. } => {
                // Approximate text extent: width ≈ 0.6·height·len
                let w = 0.6 * height * content.len() as f64;
                let (ax, ay) = anchor.to_f64();
                Some(BoundingBox::from_corners(ax, ay, ax + w, ay + height))
            }
            EntityKind::Insert { .. } => None, // resolved via block expansion
            EntityKind::XLine { .. } | EntityKind::Ray { .. } => None, // infinite
        }
    }

    /// Apply an affine transform in place (used by MOVE/ROTATE/SCALE/MIRROR).
    pub fn transform(&mut self, t: &Transform2d) {
        self.kind = match &self.kind {
            EntityKind::Curve(c) => EntityKind::Curve(t.apply_curve(c)),
            EntityKind::Point(p) => EntityKind::Point(t.apply_point(p)),
            EntityKind::Text { anchor, content, height, rotation } => EntityKind::Text {
                anchor: t.apply_point(anchor),
                content: content.clone(),
                height: height * t.scale_factor(),
                rotation: rotation + t.rotation_angle(),
            },
            EntityKind::XLine { through, dir } => EntityKind::XLine {
                through: t.apply_point(through),
                dir: transform_dir(t, dir),
            },
            EntityKind::Ray { from, dir } => EntityKind::Ray {
                from: t.apply_point(from),
                dir: transform_dir(t, dir),
            },
            EntityKind::Insert { block, transform } => EntityKind::Insert {
                block: block.clone(),
                transform: t.compose(transform),
            },
        };
    }

    /// A transformed copy (used by COPY/ARRAY/MIRROR-keep-original).
    pub fn transformed(&self, t: &Transform2d) -> Entity {
        let mut e = self.clone();
        e.transform(t);
        e
    }

    /// Convenience: get the underlying curve if this is a curve entity.
    pub fn as_curve(&self) -> Option<&Curve> {
        if let EntityKind::Curve(c) = &self.kind { Some(c) } else { None }
    }
}

/// Apply only the linear part of a transform to a direction vector (no translation).
fn transform_dir(t: &Transform2d, dir: &(f64, f64)) -> (f64, f64) {
    let (dx, dy) = dir;
    (
        t.m00 * dx + t.m01 * dy,
        t.m10 * dx + t.m11 * dy,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_geometry::LineSeg;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }

    #[test]
    fn entity_bbox_for_line() {
        let line = Curve::Line(LineSeg::from_endpoints(pt(0, 0), pt(4, 3)));
        let e = Entity::new(EntityId(1), EntityKind::Curve(line), 0);
        let bb = e.bounding_box().unwrap();
        assert_eq!(bb.min, pt(0, 0));
        assert_eq!(bb.max, pt(4, 3));
    }

    #[test]
    fn move_entity_translates_geometry() {
        let line = Curve::Line(LineSeg::from_endpoints(pt(0, 0), pt(2, 0)));
        let mut e = Entity::new(EntityId(1), EntityKind::Curve(line), 0);
        e.transform(&Transform2d::translation(5.0, 3.0));
        let c = e.as_curve().unwrap();
        if let Curve::Line(l) = c {
            assert_eq!(l.p0, pt(5, 3));
            assert_eq!(l.p1, pt(7, 3));
        } else { panic!() }
    }

    #[test]
    fn transformed_keeps_original() {
        let line = Curve::Line(LineSeg::from_endpoints(pt(0, 0), pt(2, 0)));
        let e = Entity::new(EntityId(1), EntityKind::Curve(line), 0);
        let moved = e.transformed(&Transform2d::translation(10.0, 0.0));
        // Original unchanged
        if let Curve::Line(l) = e.as_curve().unwrap() { assert_eq!(l.p0, pt(0,0)); }
        // Copy moved
        if let Curve::Line(l) = moved.as_curve().unwrap() { assert_eq!(l.p0, pt(10,0)); }
    }

    #[test]
    fn infinite_lines_have_no_bbox() {
        let e = Entity::new(EntityId(1), EntityKind::XLine {
            through: pt(0,0), dir: (1.0, 0.0),
        }, 0);
        assert!(e.bounding_box().is_none());
    }
}
