//! Parametric-constraint integration: sketch<->document sync, the solve cycle, and
//! translating selection picks into solver `Constraint`s. Split out of `state.rs`
//! to keep the core AppState file focused on tool/command dispatch.

use std::collections::HashMap;
use exact2d_algebra::Rational;
use exact2d_geometry::{Point2d, Curve};
use exact2d_document::{EntityKind, EntityId};
use exact2d_constraint::{Sketch, SolveStatus, Constraint, PointId};
use crate::command::ConstraintType;

use super::AppState;

impl AppState {
    /// Build solver constraints from the current selection per the requested type.
    pub(crate) fn add_constraint(&mut self, ctype: ConstraintType) {
                self.enter_parametric(); // builds the overlay if not already active
                self.history.snapshot(&self.document, &self.sketch, &self.entity_points);

                // Collect points, lines, and arcs from the current selection
                let mut pt_ids = Vec::new();
                let mut lines = Vec::new();
                let mut arcs = Vec::new();

                let dist_sq = |p_a, p_b, sketch: &Sketch| {
                    let (ax, ay) = sketch.point(p_a);
                    let (bx, by) = sketch.point(p_b);
                    (ax - bx).powi(2) + (ay - by).powi(2)
                };

                for &id in &self.selection {
                    if let Some(pts) = self.entity_points.get(&id) {
                        if let Some(entity) = self.document.get(id) {
                            match &entity.kind {
                                EntityKind::Point(_)
                                    if pts.len() == 1 => { pt_ids.push(pts[0]); }
                                EntityKind::Curve(Curve::Line(_))
                                    if pts.len() == 2 => { lines.push((pts[0], pts[1])); }
                                EntityKind::Curve(Curve::Arc(_))
                                    if pts.len() == 3 => { arcs.push((pts[0], pts[1], pts[2])); }
                                EntityKind::Curve(Curve::Poly(poly)) => {
                                    let mut idx = 0;
                                    for seg in &poly.segments {
                                        match seg {
                                            Curve::Line(_) => {
                                                if idx + 1 < pts.len() {
                                                    lines.push((pts[idx], pts[idx + 1]));
                                                }
                                                idx += 2;
                                            }
                                            Curve::Arc(_) => {
                                                if idx + 2 < pts.len() {
                                                    arcs.push((pts[idx], pts[idx + 1], pts[idx + 2]));
                                                }
                                                idx += 3;
                                            }
                                            Curve::Bezier(_) => idx += 4,
                                            _ => {}
                                        }
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                }

                match ctype {
                    crate::command::ConstraintType::Horizontal => {
                        for &l in &lines {
                            self.sketch.add_constraint(Constraint::Horizontal(l.0, l.1));
                        }
                    }
                    crate::command::ConstraintType::Vertical => {
                        for &l in &lines {
                            self.sketch.add_constraint(Constraint::Vertical(l.0, l.1));
                        }
                    }
                    crate::command::ConstraintType::Fix => {
                        // Fix selected point entities
                        for &pt_id in &pt_ids {
                            let (x, y) = self.sketch.point(pt_id);
                            self.sketch.add_constraint(Constraint::Fix(pt_id, x, y));
                        }
                        // Fix all points of other selected entities (original logic)
                        for &id in &self.selection {
                            if let Some(pts) = self.entity_points.get(&id) {
                                if let Some(entity) = self.document.get(id) {
                                    if !matches!(entity.kind, EntityKind::Point(_)) {
                                        for &pt_id in pts {
                                            let (x, y) = self.sketch.point(pt_id);
                                            self.sketch.add_constraint(Constraint::Fix(pt_id, x, y));
                                        }
                                    }
                                }
                            }
                        }
                    }
                    crate::command::ConstraintType::Distance(val) => {
                        let mut pts_pair = None;
                        if pt_ids.len() >= 2 {
                            pts_pair = Some((pt_ids[0], pt_ids[1]));
                        } else if !lines.is_empty() {
                            pts_pair = Some((lines[0].0, lines[0].1));
                        } else if pt_ids.len() == 1 && lines.len() == 1 {
                            let l = lines[0];
                            let p_l = if dist_sq(pt_ids[0], l.0, &self.sketch) < dist_sq(pt_ids[0], l.1, &self.sketch) { l.0 } else { l.1 };
                            pts_pair = Some((pt_ids[0], p_l));
                        }
                        if let Some((p1, p2)) = pts_pair {
                            let d = val.unwrap_or_else(|| {
                                let (x1, y1) = self.sketch.point(p1);
                                let (x2, y2) = self.sketch.point(p2);
                                ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt()
                            });
                            self.sketch.add_constraint(Constraint::Distance(p1, p2, d));
                        }
                    }
                    crate::command::ConstraintType::Parallel => {
                        if lines.len() >= 2 {
                            self.sketch.add_constraint(Constraint::Parallel(lines[0], lines[1]));
                        }
                    }
                    crate::command::ConstraintType::Perpendicular => {
                        if lines.len() >= 2 {
                            self.sketch.add_constraint(Constraint::Perpendicular(lines[0], lines[1]));
                        }
                    }
                    crate::command::ConstraintType::Tangent => {
                        if !lines.is_empty() && !arcs.is_empty() {
                            self.sketch.add_constraint(Constraint::TangentLineCircle(lines[0], arcs[0].0, arcs[0].1));
                        } else if arcs.len() >= 2 {
                            let c1 = arcs[0].0;
                            let s1 = arcs[0].1;
                            let c2 = arcs[1].0;
                            let s2 = arcs[1].1;
                            let d = dist_sq(c1, c2, &self.sketch).sqrt();
                            let r1 = dist_sq(c1, s1, &self.sketch).sqrt();
                            let r2 = dist_sq(c2, s2, &self.sketch).sqrt();
                            let d_ext = (d - (r1 + r2)).abs();
                            let d_int = (d - (r1 - r2).abs()).abs();
                            let external = d_ext < d_int;
                            self.sketch.add_constraint(Constraint::TangentCircleCircle(c1, s1, c2, s2, external));
                        }
                    }
                    crate::command::ConstraintType::Concentric => {
                        if arcs.len() >= 2 {
                            self.sketch.add_constraint(Constraint::Coincident(arcs[0].0, arcs[1].0));
                        }
                    }
                    crate::command::ConstraintType::Coincident => {
                        if pt_ids.len() >= 2 {
                            self.sketch.add_constraint(Constraint::Coincident(pt_ids[0], pt_ids[1]));
                        } else if lines.len() >= 2 {
                            let l1 = lines[0];
                            let l2 = lines[1];
                            let pairs = vec![
                                (l1.0, l2.0),
                                (l1.0, l2.1),
                                (l1.1, l2.0),
                                (l1.1, l2.1),
                            ];
                            let best_pair = pairs.into_iter().min_by(|&p1, &p2| {
                                dist_sq(p1.0, p1.1, &self.sketch)
                                    .partial_cmp(&dist_sq(p2.0, p2.1, &self.sketch))
                                    .unwrap_or(std::cmp::Ordering::Equal)
                            });
                            if let Some((p_a, p_b)) = best_pair {
                                self.sketch.add_constraint(Constraint::Coincident(p_a, p_b));
                            }
                        } else if pt_ids.len() == 1 && lines.len() == 1 {
                            self.sketch.add_constraint(Constraint::Collinear(pt_ids[0], lines[0].0, lines[0].1));
                        } else if pt_ids.len() == 1 && arcs.len() == 1 {
                            let (center, start, _) = arcs[0];
                            self.sketch.add_constraint(Constraint::EqualLength((center, start), (center, pt_ids[0])));
                        } else if lines.len() == 1 && arcs.len() == 1 {
                            let (center, start, _) = arcs[0];
                            let l = lines[0];
                            let pt_end = if dist_sq(l.0, center, &self.sketch) < dist_sq(l.1, center, &self.sketch) { l.0 } else { l.1 };
                            self.sketch.add_constraint(Constraint::EqualLength((center, start), (center, pt_end)));
                        }
                    }
                    crate::command::ConstraintType::Equal => {
                        if lines.len() >= 2 {
                            self.sketch.add_constraint(Constraint::EqualLength(lines[0], lines[1]));
                        } else if arcs.len() >= 2 {
                            self.sketch.add_constraint(Constraint::EqualLength((arcs[0].0, arcs[0].1), (arcs[1].0, arcs[1].1)));
                        } else if lines.len() == 1 && arcs.len() == 1 {
                            self.sketch.add_constraint(Constraint::EqualLength(lines[0], (arcs[0].0, arcs[0].1)));
                        }
                    }
                    crate::command::ConstraintType::Symmetric => {
                        if pt_ids.len() >= 2 && !lines.is_empty() {
                            self.sketch.add_constraint(Constraint::Symmetric(pt_ids[0], pt_ids[1], lines[0]));
                        } else if lines.len() >= 3 {
                            self.sketch.add_constraint(Constraint::Symmetric(lines[0].0, lines[1].0, lines[2]));
                            self.sketch.add_constraint(Constraint::Symmetric(lines[0].1, lines[1].1, lines[2]));
                        }
                    }
                    crate::command::ConstraintType::Midpoint => {
                        if !pt_ids.is_empty() && !lines.is_empty() {
                            self.sketch.add_constraint(Constraint::Midpoint(pt_ids[0], lines[0].0, lines[0].1));
                        } else if lines.len() >= 2 {
                            let center2 = lines[1];
                            let (x1, y1) = self.sketch.point(center2.0);
                            let (x2, y2) = self.sketch.point(center2.1);
                            let (mx, my) = ((x1 + x2) / 2.0, (y1 + y2) / 2.0);
                            let dist_m = |p, sketch: &Sketch| {
                                let (x, y) = sketch.point(p);
                                (x - mx).powi(2) + (y - my).powi(2)
                            };
                            let pt_end = if dist_m(lines[0].0, &self.sketch) < dist_m(lines[0].1, &self.sketch) { lines[0].0 } else { lines[0].1 };
                            self.sketch.add_constraint(Constraint::Midpoint(pt_end, center2.0, center2.1));
                        }
                    }
                    crate::command::ConstraintType::Angle(val) => {
                        if lines.len() >= 2 {
                            let l1 = lines[0];
                            let l2 = lines[1];
                            let theta = val.unwrap_or_else(|| {
                                let dx1 = { let (ax, _) = self.sketch.point(l1.0); let (bx, _) = self.sketch.point(l1.1); bx - ax };
                                let dy1 = { let (_, ay) = self.sketch.point(l1.0); let (_, by) = self.sketch.point(l1.1); by - ay };
                                let dx2 = { let (ax, _) = self.sketch.point(l2.0); let (bx, _) = self.sketch.point(l2.1); bx - ax };
                                let dy2 = { let (_, ay) = self.sketch.point(l2.0); let (_, by) = self.sketch.point(l2.1); by - ay };
                                let dot = dx1 * dx2 + dy1 * dy2;
                                let len1 = (dx1 * dx1 + dy1 * dy1).sqrt();
                                let len2 = (dx2 * dx2 + dy2 * dy2).sqrt();
                                if len1 * len2 > 1e-9 {
                                    (dot / (len1 * len2)).clamp(-1.0, 1.0).acos()
                                } else {
                                    0.0
                                }
                            });
                            self.sketch.add_constraint(Constraint::Angle(l1, l2, theta));
                        }
                    }
                }
                self.solve_constraints();
    }

    /// Register a point for the entity currently being built, appending its id to
    /// `pts`. It shares only with points already registered for THAT SAME entity —
    /// so a polyline's shared joints and a full circle's coincident start/end
    /// collapse to one point — but it never welds to *other* entities' points.
    /// Independent geometry that merely touches at a location stays distinct (the
    /// Stage 2 welding fix); intentional connections between entities are made
    /// explicitly via constraints (Stage 3). See docs/constraint-refactor-plan.md.
    fn push_point(&mut self, pts: &mut Vec<PointId>, x: f64, y: f64) {
        let tol = 1e-6;
        for &id in pts.iter() {
            let (px, py) = self.sketch.point(id);
            if (px - x).hypot(py - y) < tol {
                pts.push(id);
                return;
            }
        }
        let id = self.sketch.add_point(x, y);
        pts.push(id);
    }

    pub fn sync_sketch_from_document(&mut self) {
        let old_sketch = self.sketch.clone();
        let old_entity_points = self.entity_points.clone();

        self.sketch = Sketch::new();
        self.entity_points.clear();

        // Collect all editable entities' information first to avoid borrow conflicts!
        let entities: Vec<(EntityId, EntityKind)> = self.document.editable_entities()
            .map(|e| (e.id, e.kind.clone()))
            .collect();

        for (id, kind) in entities {
            let mut pts = Vec::new();
            match &kind {
                EntityKind::Curve(Curve::Line(line)) => {
                    let (x0, y0) = line.p0.to_f64();
                    let (x1, y1) = line.p1.to_f64();
                    self.push_point(&mut pts, x0, y0);
                    self.push_point(&mut pts, x1, y1);
                }
                EntityKind::Curve(Curve::Arc(arc)) => {
                    let (cx, cy) = arc.center.to_f64();
                    let (ax, ay) = arc.start_point();
                    let (bx, by) = arc.end_point();
                    self.push_point(&mut pts, cx, cy);
                    self.push_point(&mut pts, ax, ay);
                    self.push_point(&mut pts, bx, by);
                }
                EntityKind::Curve(Curve::Bezier(bezier)) => {
                    let (x0, y0) = bezier.p0.to_f64();
                    let (x1, y1) = bezier.p1.to_f64();
                    let (x2, y2) = bezier.p2.to_f64();
                    let (x3, y3) = bezier.p3.to_f64();
                    self.push_point(&mut pts, x0, y0);
                    self.push_point(&mut pts, x1, y1);
                    self.push_point(&mut pts, x2, y2);
                    self.push_point(&mut pts, x3, y3);
                }
                EntityKind::Curve(Curve::Poly(poly)) => {
                    for seg in &poly.segments {
                        match seg {
                            Curve::Line(line) => {
                                let (x0, y0) = line.p0.to_f64();
                                let (x1, y1) = line.p1.to_f64();
                                self.push_point(&mut pts, x0, y0);
                                self.push_point(&mut pts, x1, y1);
                            }
                            Curve::Arc(arc) => {
                                let (cx, cy) = arc.center.to_f64();
                                let (ax, ay) = arc.start_point();
                                let (bx, by) = arc.end_point();
                                self.push_point(&mut pts, cx, cy);
                                self.push_point(&mut pts, ax, ay);
                                self.push_point(&mut pts, bx, by);
                            }
                            Curve::Bezier(bezier) => {
                                let (x0, y0) = bezier.p0.to_f64();
                                let (x1, y1) = bezier.p1.to_f64();
                                let (x2, y2) = bezier.p2.to_f64();
                                let (x3, y3) = bezier.p3.to_f64();
                                self.push_point(&mut pts, x0, y0);
                                self.push_point(&mut pts, x1, y1);
                                self.push_point(&mut pts, x2, y2);
                                self.push_point(&mut pts, x3, y3);
                            }
                            _ => {}
                        }
                    }
                }
                EntityKind::Point(pt) => {
                    let (x, y) = pt.to_f64();
                    self.push_point(&mut pts, x, y);
                }
                _ => {}
            }
            if !pts.is_empty() {
                self.entity_points.insert(id, pts);
            }
        }

        let mut old_to_new = HashMap::new();
        for (entity_id, old_pts) in &old_entity_points {
            if let Some(new_pts) = self.entity_points.get(entity_id) {
                for (&old_pt, &new_pt) in old_pts.iter().zip(new_pts.iter()) {
                    old_to_new.insert(old_pt, new_pt);
                }
            }
        }

        for c in old_sketch.constraints() {
            if let Some(new_c) = translate_constraint(c, &old_to_new) {
                let is_origin_fix = match &new_c {
                    Constraint::Fix(pt, _, _) => {
                        if let Some(pts) = self.entity_points.get(&self.origin_id) {
                            pts.first() == Some(pt)
                        } else {
                            false
                        }
                    }
                    _ => false,
                };
                if !is_origin_fix {
                    self.sketch.add_constraint(new_c);
                }
            }
        }

        if let Some(pts) = self.entity_points.get(&self.origin_id) {
            if let Some(&origin_pt_id) = pts.first() {
                self.sketch.add_constraint(Constraint::Fix(origin_pt_id, 0.0, 0.0));
            }
        }

        for entity in self.document.editable_entities() {
            if let EntityKind::Curve(Curve::Arc(_)) = &entity.kind {
                if let Some(pts) = self.entity_points.get(&entity.id) {
                    if pts.len() == 3 {
                        let center_id = pts[0];
                        let start_id = pts[1];
                        let end_id = pts[2];
                        if start_id != end_id {
                            self.sketch.add_constraint(Constraint::EqualLength(
                                (center_id, start_id),
                                (center_id, end_id),
                            ));
                        }
                    }
                }
            } else if let EntityKind::Curve(Curve::Poly(poly)) = &entity.kind {
                if let Some(pts) = self.entity_points.get(&entity.id) {
                    let mut idx = 0;
                    for seg in &poly.segments {
                        match seg {
                            Curve::Line(_) => idx += 2,
                            Curve::Arc(_) => {
                                let center_id = pts[idx];
                                let start_id = pts[idx + 1];
                                let end_id = pts[idx + 2];
                                idx += 3;
                                if start_id != end_id {
                                    self.sketch.add_constraint(Constraint::EqualLength(
                                        (center_id, start_id),
                                        (center_id, end_id),
                                    ));
                                }
                            }
                            Curve::Bezier(_) => idx += 4,
                            _ => {}
                        }
                    }
                }
            }
        }
    }

    pub fn sync_document_from_sketch(&mut self) {
        let entity_points = self.entity_points.clone();
        for (entity_id, pts) in entity_points {
            if let Some(entity) = self.document.get_mut(entity_id) {
                match &mut entity.kind {
                    EntityKind::Curve(Curve::Line(line))
                        if pts.len() == 2 => {
                            let (x0, y0) = self.sketch.point(pts[0]);
                            let (x1, y1) = self.sketch.point(pts[1]);
                            line.p0 = Point2d::from_f64(x0, y0);
                            line.p1 = Point2d::from_f64(x1, y1);
                        }
                    EntityKind::Curve(Curve::Arc(arc))
                        if pts.len() == 3 => {
                            let (cx, cy) = self.sketch.point(pts[0]);
                            let (ax, ay) = self.sketch.point(pts[1]);
                            let (bx, by) = self.sketch.point(pts[2]);
                            let r = ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().max(1e-6);
                            arc.center = Point2d::from_f64(cx, cy);
                            arc.radius = Rational::from_f64_approx(r);
                            let orig_included = arc.end_angle - arc.start_angle;
                            let start_angle = (ay - cy).atan2(ax - cx);
                            let end_angle_raw = (by - cy).atan2(bx - cx);
                            let target = start_angle + orig_included;
                            let mut best_diff = f64::INFINITY;
                            let mut best_end = end_angle_raw;
                            for k in -2..=2 {
                                let candidate = end_angle_raw + (k as f64) * std::f64::consts::TAU;
                                let diff = (candidate - target).abs();
                                if diff < best_diff {
                                    best_diff = diff;
                                    best_end = candidate;
                                }
                            }
                            arc.start_angle = start_angle;
                            arc.end_angle = best_end;
                        }
                    EntityKind::Curve(Curve::Bezier(bezier))
                        if pts.len() == 4 => {
                            let (x0, y0) = self.sketch.point(pts[0]);
                            let (x1, y1) = self.sketch.point(pts[1]);
                            let (x2, y2) = self.sketch.point(pts[2]);
                            let (x3, y3) = self.sketch.point(pts[3]);
                            bezier.p0 = Point2d::from_f64(x0, y0);
                            bezier.p1 = Point2d::from_f64(x1, y1);
                            bezier.p2 = Point2d::from_f64(x2, y2);
                            bezier.p3 = Point2d::from_f64(x3, y3);
                        }
                    EntityKind::Curve(Curve::Poly(poly)) => {
                        let mut idx = 0;
                        for seg in &mut poly.segments {
                            match seg {
                                Curve::Line(line) => {
                                    if idx + 1 < pts.len() {
                                        let (x0, y0) = self.sketch.point(pts[idx]);
                                        let (x1, y1) = self.sketch.point(pts[idx + 1]);
                                        line.p0 = Point2d::from_f64(x0, y0);
                                        line.p1 = Point2d::from_f64(x1, y1);
                                    }
                                    idx += 2;
                                }
                                Curve::Arc(arc) => {
                                    if idx + 2 < pts.len() {
                                        let (cx, cy) = self.sketch.point(pts[idx]);
                                        let (ax, ay) = self.sketch.point(pts[idx + 1]);
                                        let (bx, by) = self.sketch.point(pts[idx + 2]);
                                        let r = ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().max(1e-6);
                                        arc.center = Point2d::from_f64(cx, cy);
                                        arc.radius = Rational::from_f64_approx(r);
                                        let orig_included = arc.end_angle - arc.start_angle;
                                        let start_angle = (ay - cy).atan2(ax - cx);
                                        let end_angle_raw = (by - cy).atan2(bx - cx);
                                        let target = start_angle + orig_included;
                                        let mut best_diff = f64::INFINITY;
                                        let mut best_end = end_angle_raw;
                                        for k in -2..=2 {
                                            let candidate = end_angle_raw + (k as f64) * std::f64::consts::TAU;
                                            let diff = (candidate - target).abs();
                                            if diff < best_diff {
                                                best_diff = diff;
                                                best_end = candidate;
                                            }
                                        }
                                        arc.start_angle = start_angle;
                                        arc.end_angle = best_end;
                                    }
                                    idx += 3;
                                }
                                Curve::Bezier(bezier) => {
                                    if idx + 3 < pts.len() {
                                        let (x0, y0) = self.sketch.point(pts[idx]);
                                        let (x1, y1) = self.sketch.point(pts[idx + 1]);
                                        let (x2, y2) = self.sketch.point(pts[idx + 2]);
                                        let (x3, y3) = self.sketch.point(pts[idx + 3]);
                                        bezier.p0 = Point2d::from_f64(x0, y0);
                                        bezier.p1 = Point2d::from_f64(x1, y1);
                                        bezier.p2 = Point2d::from_f64(x2, y2);
                                        bezier.p3 = Point2d::from_f64(x3, y3);
                                    }
                                    idx += 4;
                                }
                                Curve::Ellipse(_) | Curve::Poly(_) => {}
                            }
                        }
                    }
                    EntityKind::Point(pt)
                        if pts.len() == 1 => {
                            let (x, y) = self.sketch.point(pts[0]);
                            *pt = Point2d::from_f64(x, y);
                        }
                    _ => {}
                }
            }
        }
    }

    pub fn solve_constraints(&mut self) -> SolveStatus {
        if self.constraints_enabled {
            let status = self.sketch.solve(100, 1e-10);
            self.sync_document_from_sketch();
            status
        } else {
            SolveStatus::Converged { iterations: 0, residual: 0.0 }
        }
    }

    // ── Parametric mode lifecycle (Stage 1) ────────────────────────────────────
    // The parametric sketch is an ephemeral overlay on the exact document, which is
    // always the single source of truth. The overlay exists only while parametric
    // mode is ON. See docs/constraint-refactor-plan.md.
    //
    // NOTE: the overlay data (`sketch`, `entity_points`) still lives as fields on
    // AppState rather than inside an `Option<ParametricSession>`; the type-level
    // encapsulation is a later cleanup (it would touch ~80 call sites and the
    // document/sketch split-borrows). `constraints_enabled` is the "session active"
    // flag, and the overlay is empty whenever it is false.

    /// Enter parametric mode: build the sketch overlay from the current geometry
    /// (once) and solve. No-op if already active.
    pub fn enter_parametric(&mut self) {
        if self.constraints_enabled { return; }
        self.constraints_enabled = true;
        self.sync_sketch_from_document(); // build once from current geometry
        self.solve_constraints();
    }

    /// Exit parametric mode: discard the sketch/constraint overlay (Option A —
    /// constraints do not persist). Geometry has already been baked into the
    /// document by prior solves, so dropping the overlay reverts instantly to free
    /// drafting without changing any geometry.
    pub fn exit_parametric(&mut self) {
        if !self.constraints_enabled { return; }
        self.constraints_enabled = false;
        self.sketch = Sketch::new();
        self.entity_points.clear();
        self.pending_snap_links.clear();
        self.command_log.push("Parametric off — constraints cleared".to_string());
    }

    /// Turn the snap-to-endpoint picks recorded during a draw into Coincident
    /// constraints: for each, link the newly-created entity's point at that
    /// location to the existing point it snapped onto. Connection by intent — only
    /// runs for points the user deliberately snapped (Stage 3). The constraint
    /// survives the per-edit rebuild via the normal constraint remap.
    pub(crate) fn materialize_snap_links(&mut self, new_ids: &[EntityId]) {
        let tol = 1e-6;
        let links = std::mem::take(&mut self.pending_snap_links);
        for (px, py, target) in links {
            let new_pt = self.find_entity_point_at(new_ids, px, py, tol);
            let existing_pt = self.find_entity_point_at(std::slice::from_ref(&target), px, py, tol);
            if let (Some(a), Some(b)) = (new_pt, existing_pt) {
                if a != b {
                    self.sketch.add_constraint(Constraint::Coincident(a, b));
                }
            }
        }
    }

    /// First registered point of any of `ids` lying within `tol` of (px, py).
    fn find_entity_point_at(&self, ids: &[EntityId], px: f64, py: f64, tol: f64) -> Option<PointId> {
        for id in ids {
            if let Some(pts) = self.entity_points.get(id) {
                for &pt in pts {
                    let (x, y) = self.sketch.point(pt);
                    if (x - px).hypot(y - py) < tol {
                        return Some(pt);
                    }
                }
            }
        }
        None
    }

    pub fn remove_constraint(&mut self, index: usize) {
        if index < self.sketch.constraints().len() {
            self.history.snapshot(&self.document, &self.sketch, &self.entity_points);
            self.sketch.constraints_mut().remove(index);
            self.solve_constraints();
        }
    }
}

fn translate_constraint(c: &Constraint, map: &HashMap<PointId, PointId>) -> Option<Constraint> {
    let lookup = |p| map.get(&p).copied();
    let lookup_line = |(p1, p2)| {
        if let (Some(n1), Some(n2)) = (lookup(p1), lookup(p2)) {
            Some((n1, n2))
        } else {
            None
        }
    };
    match c {
        Constraint::Coincident(p1, p2) => Some(Constraint::Coincident(lookup(*p1)?, lookup(*p2)?)),
        Constraint::Fix(p, x, y) => Some(Constraint::Fix(lookup(*p)?, *x, *y)),
        Constraint::Horizontal(p1, p2) => Some(Constraint::Horizontal(lookup(*p1)?, lookup(*p2)?)),
        Constraint::Vertical(p1, p2) => Some(Constraint::Vertical(lookup(*p1)?, lookup(*p2)?)),
        Constraint::Parallel(l1, l2) => Some(Constraint::Parallel(lookup_line(*l1)?, lookup_line(*l2)?)),
        Constraint::Perpendicular(l1, l2) => Some(Constraint::Perpendicular(lookup_line(*l1)?, lookup_line(*l2)?)),
        Constraint::Collinear(p1, p2, p3) => Some(Constraint::Collinear(lookup(*p1)?, lookup(*p2)?, lookup(*p3)?)),
        Constraint::EqualLength(l1, l2) => Some(Constraint::EqualLength(lookup_line(*l1)?, lookup_line(*l2)?)),
        Constraint::Symmetric(p1, p2, axis) => Some(Constraint::Symmetric(lookup(*p1)?, lookup(*p2)?, lookup_line(*axis)?)),
        Constraint::Distance(p1, p2, d) => Some(Constraint::Distance(lookup(*p1)?, lookup(*p2)?, *d)),
        Constraint::DistanceX(p1, p2, d) => Some(Constraint::DistanceX(lookup(*p1)?, lookup(*p2)?, *d)),
        Constraint::DistanceY(p1, p2, d) => Some(Constraint::DistanceY(lookup(*p1)?, lookup(*p2)?, *d)),
        Constraint::Angle(l1, l2, theta) => Some(Constraint::Angle(lookup_line(*l1)?, lookup_line(*l2)?, *theta)),
        Constraint::Midpoint(m, a, b) => Some(Constraint::Midpoint(lookup(*m)?, lookup(*a)?, lookup(*b)?)),
        Constraint::TangentLineCircle(line, center, start) => Some(Constraint::TangentLineCircle(lookup_line(*line)?, lookup(*center)?, lookup(*start)?)),
        Constraint::TangentCircleCircle(c1, s1, c2, s2, ext) => Some(Constraint::TangentCircleCircle(lookup(*c1)?, lookup(*s1)?, lookup(*c2)?, lookup(*s2)?, *ext)),
    }
}
