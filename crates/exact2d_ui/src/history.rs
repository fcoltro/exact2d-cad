//! Undo/redo history (spec §4.1 / §6.1) — snapshot-based.
//!
//! Before each document mutation the app calls `snapshot`, capturing the current
//! document on the undo stack and clearing the redo stack (a new edit branches off).
//! `undo`/`redo` swap documents between the two stacks.

use exact2d_document::Document;

pub struct History {
    past: Vec<Document>,
    future: Vec<Document>,
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

    /// Record the current document as a restore point. Call just before mutating.
    pub fn snapshot(&mut self, doc: &Document) {
        self.past.push(doc.clone());
        if self.past.len() > self.limit { self.past.remove(0); }
        self.future.clear(); // a new edit invalidates the redo branch
    }

    /// Undo: returns the previous document, given the current one.
    pub fn undo(&mut self, doc: &Document) -> Option<Document> {
        let prev = self.past.pop()?;
        self.future.push(doc.clone());
        Some(prev)
    }

    /// Redo: returns the next document, given the current one.
    pub fn redo(&mut self, doc: &Document) -> Option<Document> {
        let next = self.future.pop()?;
        self.past.push(doc.clone());
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
        let mut hist = History::new();

        hist.snapshot(&doc);
        doc.add(line(0));
        assert_eq!(doc.len(), 1);

        hist.snapshot(&doc);
        doc.add(line(5));
        assert_eq!(doc.len(), 2);

        doc = hist.undo(&doc).unwrap();
        assert_eq!(doc.len(), 1);
        doc = hist.undo(&doc).unwrap();
        assert_eq!(doc.len(), 0);
        assert!(!hist.can_undo());

        doc = hist.redo(&doc).unwrap();
        assert_eq!(doc.len(), 1);
        doc = hist.redo(&doc).unwrap();
        assert_eq!(doc.len(), 2);
        assert!(!hist.can_redo());
    }

    #[test]
    fn new_edit_clears_redo() {
        let mut doc = Document::new();
        let mut hist = History::new();
        hist.snapshot(&doc); doc.add(line(0));
        doc = hist.undo(&doc).unwrap();
        assert!(hist.can_redo());
        hist.snapshot(&doc); doc.add(line(9));
        assert!(!hist.can_redo(), "new edit must clear redo stack");
    }

    #[test]
    fn limit_bounds_growth() {
        let mut doc = Document::new();
        let mut hist = History::with_limit(3);
        for i in 0..10 { hist.snapshot(&doc); doc.add(line(i)); }
        assert_eq!(hist.undo_depth(), 3);
    }
}
