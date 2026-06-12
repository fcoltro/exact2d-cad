//! The top-level document model (spec §4.1): layers, blocks, entities, views, settings.

use std::collections::HashMap;
use exact2d_geometry::{BoundingBox, Point2d};
use crate::entity::{Entity, EntityId, EntityKind};
use crate::layer::LayerTable;
use crate::properties::LineTypeDef;

/// Drawing units (one drawing-coordinate unit = one of these). Geometry itself is
/// unitless exact rational; this just labels the scale and drives the zoom limits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Units {
    Unitless, Millimeters, Centimeters, Meters, Kilometers, Inches, Feet,
}

impl Units {
    pub fn short_name(self) -> &'static str {
        match self {
            Units::Unitless => "", Units::Millimeters => "mm", Units::Centimeters => "cm",
            Units::Meters => "m", Units::Kilometers => "km", Units::Inches => "in", Units::Feet => "ft",
        }
    }

    /// Precision-safe range of how much of the drawing (in coordinate units) may be
    /// visible across the viewport: `(min_visible_width, max_visible_width)`.
    /// Zooming in is capped at `min` (avoids f32 precision loss); zooming out is
    /// capped at `max` (avoids large-coordinate clipping). Picked so each unit has a
    /// sensible working span — e.g. mm: 0.05 mm … 50 m; km: 0.1 m … 50 000 km.
    pub fn visible_range(self) -> (f64, f64) {
        match self {
            Units::Millimeters => (0.05, 50_000.0),       // 0.05 mm … 50 m
            Units::Centimeters => (0.01, 100_000.0),      // 0.1 mm … 1 km
            Units::Meters      => (0.001, 100_000.0),     // 1 mm … 100 km
            Units::Kilometers  => (0.0001, 50_000.0),     // 0.1 m … 50 000 km
            Units::Inches      => (0.001, 100_000.0),
            Units::Feet        => (0.001, 100_000.0),
            Units::Unitless    => (0.001, 1_000_000.0),   // generic, still bounded
        }
    }
}

/// A named block: a reusable collection of entities with a base point.
#[derive(Clone, Debug)]
pub struct Block {
    pub name: String,
    pub base_point: Point2d,
    pub entities: Vec<Entity>,
}

/// A saved camera/view state.
#[derive(Clone, Debug)]
pub struct NamedView {
    pub name: String,
    pub center: (f64, f64),
    pub zoom: f64,
}

/// Document-wide settings.
#[derive(Clone, Debug)]
pub struct Settings {
    pub units: Units,
    pub grid_spacing: f64,
    pub snap_spacing: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { units: Units::Millimeters, grid_spacing: 10.0, snap_spacing: 1.0 }
    }
}

/// The complete drawing document.
#[derive(Clone)]
pub struct Document {
    pub layers: LayerTable,
    pub line_types: Vec<LineTypeDef>,
    pub blocks: HashMap<String, Block>,
    /// Model-space entities, keyed by id for stable references.
    pub entities: HashMap<EntityId, Entity>,
    /// Insertion order, so iteration/draw order is deterministic.
    pub order: Vec<EntityId>,
    pub views: Vec<NamedView>,
    pub settings: Settings,
    next_id: u64,
}

impl Default for Document {
    fn default() -> Self {
        Document {
            layers: LayerTable::default(),
            line_types: vec![
                LineTypeDef::continuous(),
                LineTypeDef::dashed(),
                LineTypeDef::center(),
            ],
            blocks: HashMap::new(),
            entities: HashMap::new(),
            order: Vec::new(),
            views: Vec::new(),
            settings: Settings::default(),
            next_id: 1,
        }
    }
}

impl Document {
    pub fn new() -> Self { Self::default() }

    fn alloc_id(&mut self) -> EntityId {
        let id = EntityId(self.next_id);
        self.next_id += 1;
        id
    }

    /// Add an entity on the current layer. Returns its new id.
    pub fn add(&mut self, kind: EntityKind) -> EntityId {
        let layer = self.layers.current;
        self.add_on_layer(kind, layer)
    }

    pub fn add_on_layer(&mut self, kind: EntityKind, layer: usize) -> EntityId {
        let id = self.alloc_id();
        let entity = Entity::new(id, kind, layer);
        self.entities.insert(id, entity);
        self.order.push(id);
        id
    }

    pub fn add_entity(&mut self, mut entity: Entity) -> EntityId {
        let id = self.alloc_id();
        entity.id = id;
        self.entities.insert(id, entity);
        self.order.push(id);
        id
    }

    pub fn get(&self, id: EntityId) -> Option<&Entity> { self.entities.get(&id) }
    pub fn get_mut(&mut self, id: EntityId) -> Option<&mut Entity> { self.entities.get_mut(&id) }

    /// Remove an entity (ERASE).
    pub fn remove(&mut self, id: EntityId) -> Option<Entity> {
        self.order.retain(|&e| e != id);
        self.entities.remove(&id)
    }

    pub fn len(&self) -> usize { self.entities.len() }
    pub fn is_empty(&self) -> bool { self.entities.is_empty() }

    /// Iterate entities in insertion (draw) order.
    pub fn iter(&self) -> impl Iterator<Item = &Entity> {
        self.order.iter().filter_map(move |id| self.entities.get(id))
    }

    /// Entities whose layer is currently editable (not locked/frozen/off).
    pub fn editable_entities(&self) -> impl Iterator<Item = &Entity> {
        self.iter().filter(move |e| {
            self.layers.get(e.layer).map(|l| l.is_editable()).unwrap_or(false)
        })
    }

    /// Bounding box of all finite model-space geometry (for ZOOM extents).
    pub fn extents(&self) -> Option<BoundingBox> {
        let mut acc: Option<BoundingBox> = None;
        for e in self.iter() {
            if let Some(bb) = e.bounding_box() {
                acc = Some(match acc {
                    Some(a) => a.union(&bb),
                    None => bb,
                });
            }
        }
        acc
    }

    /// Define (or replace) a block.
    pub fn define_block(&mut self, block: Block) {
        self.blocks.insert(block.name.clone(), block);
    }

    /// Expand a block insert into concrete world-space entities (EXPLODE / render).
    pub fn explode_insert(&self, insert: &Entity) -> Vec<Entity> {
        if let EntityKind::Insert { block, transform } = &insert.kind {
            if let Some(b) = self.blocks.get(block) {
                return b.entities.iter().map(|e| {
                    let mut copy = e.clone();
                    copy.transform(transform);
                    copy.layer = insert.layer; // inserts place onto the insert's layer
                    copy
                }).collect();
            }
        }
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_geometry::{Curve, LineSeg, Transform2d};

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }
    fn line(x0: i64, y0: i64, x1: i64, y1: i64) -> EntityKind {
        EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(pt(x0,y0), pt(x1,y1))))
    }

    #[test]
    fn add_remove_entities() {
        let mut doc = Document::new();
        let a = doc.add(line(0,0,1,1));
        let b = doc.add(line(1,1,2,2));
        assert_eq!(doc.len(), 2);
        doc.remove(a);
        assert_eq!(doc.len(), 1);
        assert!(doc.get(a).is_none());
        assert!(doc.get(b).is_some());
    }

    #[test]
    fn insertion_order_preserved() {
        let mut doc = Document::new();
        let ids: Vec<_> = (0..5).map(|i| doc.add(line(i, 0, i+1, 0))).collect();
        let seen: Vec<_> = doc.iter().map(|e| e.id).collect();
        assert_eq!(seen, ids);
    }

    #[test]
    fn extents_covers_all() {
        let mut doc = Document::new();
        doc.add(line(0, 0, 2, 2));
        doc.add(line(5, 5, 8, 1));
        let bb = doc.extents().unwrap();
        assert_eq!(bb.min, pt(0, 0));
        assert_eq!(bb.max, pt(8, 5));
    }

    #[test]
    fn block_insert_explodes_to_world() {
        let mut doc = Document::new();
        // Define a block with one line from (0,0)-(1,0)
        doc.define_block(Block {
            name: "tick".into(),
            base_point: pt(0, 0),
            entities: vec![Entity::new(EntityId(0), line(0,0,1,0), 0)],
        });
        // Insert it translated by (10, 10)
        let insert = doc.add(EntityKind::Insert {
            block: "tick".into(),
            transform: Transform2d::translation(10.0, 10.0),
        });
        let exploded = doc.explode_insert(doc.get(insert).unwrap());
        assert_eq!(exploded.len(), 1);
        if let Curve::Line(l) = exploded[0].as_curve().unwrap() {
            assert_eq!(l.p0, pt(10, 10));
            assert_eq!(l.p1, pt(11, 10));
        } else { panic!() }
    }
}
