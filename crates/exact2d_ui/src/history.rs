//! Undo/redo history (spec §4.1 / §6.1) — snapshot-based.
//!
//! Before each document mutation the app calls `snapshot`, capturing the current
//! state on the undo stack and clearing the redo stack (a new edit branches off).
//! `undo`/`redo` swap states between the two stacks.

use exact2d_document::{Document, EntityId};
use exact2d_constraint::{Sketch, PointId};
use std::collections::HashMap;

pub struct History {
    past: Vec<(Document, Sketch, HashMap<EntityId, Vec<PointId>>)>,
    future: Vec<(Document, Sketch, HashMap<EntityId, Vec<PointId>>)>,
    /// Maximum retained undo states (older ones are dropped).
    limit: usize,
}

impl Default for History {
    fn default() -> Self { History { past: Vec::new(), future: Vec::new(), limit: 200 } }
}

impl History {
    pub fn new() -> Self { Self::default() }

    pub fn with_limit(limit: usize) -> Self {
        History { past: Vec::new(), future: Vec::new(), limit }
    }

    /// Record the current state as a restore point. Call this just before mutating.
    pub fn snapshot(
        &mut self,
        doc: &Document,
        sketch: &Sketch,
        map: &HashMap<EntityId, Vec<PointId>>,
    ) {
        self.past.push((doc.clone(), sketch.clone(), map.clone()));
        if self.past.len() > self.limit { self.past.remove(0); }
        self.future.clear(); // a new edit invalidates the redo branch
    }

    /// Undo: returns the previous state, given the current one.
    pub fn undo(
        &mut self,
        doc: &Document,
        sketch: &Sketch,
        map: &HashMap<EntityId, Vec<PointId>>,
    ) -> Option<(Document, Sketch, HashMap<EntityId, Vec<PointId>>)> {
        let prev = self.past.pop()?;
        self.future.push((doc.clone(), sketch.clone(), map.clone()));
        Some(prev)
    }

    /// Redo: returns the next state, given the current one.
    pub fn redo(
        &mut self,
        doc: &Document,
        sketch: &Sketch,
        map: &HashMap<EntityId, Vec<PointId>>,
    ) -> Option<(Document, Sketch, HashMap<EntityId, Vec<PointId>>)> {
        let next = self.future.pop()?;
        self.past.push((doc.clone(), sketch.clone(), map.clone()));
        Some(next)
    }

    /// Drop the most recent restore point. Use when a snapshotted edit turned out to
    /// change nothing (e.g. EXTEND found no reachable boundary), so undo stays clean.
    pub fn discard_last(&mut self) { self.past.pop(); }

    pub fn can_undo(&self) -> bool { !self.past.is_empty() }
    pub fn can_redo(&self) -> bool { !self.future.is_empty() }
    pub fn undo_depth(&self) -> usize { self.past.len() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use exact2d_document::EntityKind;
    use exact2d_geometry::{Curve, LineSeg, Point2d};

    fn line(x: i64) -> EntityKind {
        EntityKind::Curve(Curve::Line(LineSeg::from_endpoints(
            Point2d::from_i64(x, 0), Point2d::from_i64(x + 1, 1))))
    }

    #[test]
    fn undo_redo_roundtrip() {
        let mut doc = Document::new();
        let sk = Sketch::new();
        let map = HashMap::new();
        let mut hist = History::new();

        hist.snapshot(&doc, &sk, &map);
        doc.add(line(0));
        assert_eq!(doc.len(), 1);

        hist.snapshot(&doc, &sk, &map);
        doc.add(line(5));
        assert_eq!(doc.len(), 2);

        // Undo back to 1 entity
        let (d, _, _) = hist.undo(&doc, &sk, &map).unwrap();
        doc = d;
        assert_eq!(doc.len(), 1);
        // Undo back to empty
        let (d, _, _) = hist.undo(&doc, &sk, &map).unwrap();
        doc = d;
        assert_eq!(doc.len(), 0);
        assert!(!hist.can_undo());

        // Redo forward
        let (d, _, _) = hist.redo(&doc, &sk, &map).unwrap();
        doc = d;
        assert_eq!(doc.len(), 1);
        let (d, _, _) = hist.redo(&doc, &sk, &map).unwrap();
        doc = d;
        assert_eq!(doc.len(), 2);
        assert!(!hist.can_redo());
    }

    #[test]
    fn new_edit_clears_redo() {
        let mut doc = Document::new();
        let sk = Sketch::new();
        let map = HashMap::new();
        let mut hist = History::new();
        hist.snapshot(&doc, &sk, &map); doc.add(line(0));
        let (d, _, _) = hist.undo(&doc, &sk, &map).unwrap();       // back to empty
        doc = d;
        assert!(hist.can_redo());
        hist.snapshot(&doc, &sk, &map); doc.add(line(9)); // new branch
        assert!(!hist.can_redo(), "new edit must clear redo stack");
    }

    #[test]
    fn limit_bounds_growth() {
        let mut doc = Document::new();
        let sk = Sketch::new();
        let map = HashMap::new();
        let mut hist = History::with_limit(3);
        for i in 0..10 { hist.snapshot(&doc, &sk, &map); doc.add(line(i)); }
        assert_eq!(hist.undo_depth(), 3);
    }
}
