use exact2d_algebra::BivariatePoly;
use crate::point::BoundingBox;
use crate::curve::{Curve, CurveSegment};

/// Geometric continuity between successive segments.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Continuity {
    /// G0 — endpoints match (position continuity).
    G0,
    /// G1 — endpoints match AND tangent directions are parallel (no speed requirement).
    G1,
    /// G2 — G1 plus matching curvature.
    G2,
}

/// A compound curve: an ordered sequence of curve segments joined end-to-end.
///
/// Successive segments share an endpoint (G0 minimum), and continuity is tracked.
#[derive(Clone, Debug)]
pub struct PolyCurve {
    pub segments: Vec<Curve>,
    /// `continuity[i]` describes the join between `segments[i]` and `segments[i+1]`.
    pub continuity: Vec<Continuity>,
}

impl PolyCurve {
    pub fn new(segments: Vec<Curve>) -> Self {
        let n = segments.len().saturating_sub(1);
        PolyCurve {
            segments,
            continuity: vec![Continuity::G0; n],
        }
    }

    pub fn with_continuity(segments: Vec<Curve>, continuity: Vec<Continuity>) -> Self {
        assert!(continuity.len() + 1 == segments.len() || segments.is_empty());
        PolyCurve { segments, continuity }
    }

    // ── Continuity validation ─────────────────────────────────────────────────

    /// Check G0 continuity: end of segment i ≈ start of segment i+1.
    pub fn check_g0(&self, tol: f64) -> bool {
        for i in 0..self.segments.len().saturating_sub(1) {
            let (_, t1) = self.segments[i].domain();
            let (t0, _) = self.segments[i + 1].domain();
            let (ex, ey) = self.segments[i].evaluate_f64(t1);
            let (sx, sy) = self.segments[i + 1].evaluate_f64(t0);
            let d = ((ex - sx).powi(2) + (ey - sy).powi(2)).sqrt();
            if d > tol { return false; }
        }
        true
    }

    /// Check G1 continuity: G0 + tangent directions parallel at each join.
    pub fn check_g1(&self, tol: f64) -> bool {
        if !self.check_g0(tol) { return false; }
        for i in 0..self.segments.len().saturating_sub(1) {
            let (_, t1) = self.segments[i].domain();
            let (t0, _) = self.segments[i + 1].domain();
            let (tx0, ty0) = self.segments[i].tangent_f64(t1);
            let (tx1, ty1) = self.segments[i + 1].tangent_f64(t0);
            // Parallel check: cross product ≈ 0
            let cross = tx0 * ty1 - ty0 * tx1;
            let len0 = (tx0 * tx0 + ty0 * ty0).sqrt().max(1e-15);
            let len1 = (tx1 * tx1 + ty1 * ty1).sqrt().max(1e-15);
            if cross.abs() / (len0 * len1) > tol { return false; }
        }
        true
    }

    /// Check G2 continuity: G1 + curvature match at each join.
    pub fn check_g2(&self, tol: f64) -> bool {
        if !self.check_g1(tol) { return false; }
        for i in 0..self.segments.len().saturating_sub(1) {
            let (_, t1) = self.segments[i].domain();
            let (t0, _) = self.segments[i + 1].domain();
            let k0 = curvature_f64(&self.segments[i], t1);
            let k1 = curvature_f64(&self.segments[i + 1], t0);
            if (k0 - k1).abs() > tol { return false; }
        }
        true
    }

    /// Classify all joins and update `self.continuity`.
    pub fn classify_continuity(&mut self, tol: f64) {
        for i in 0..self.segments.len().saturating_sub(1) {
            self.continuity[i] = {
                let (_, t1) = self.segments[i].domain();
                let (t0, _) = self.segments[i + 1].domain();
                let (ex, ey) = self.segments[i].evaluate_f64(t1);
                let (sx, sy) = self.segments[i + 1].evaluate_f64(t0);
                let d = ((ex - sx).powi(2) + (ey - sy).powi(2)).sqrt();
                if d > tol {
                    // not even G0 — leave as G0 but the check_g0 will return false
                    Continuity::G0
                } else {
                    let (tx0, ty0) = self.segments[i].tangent_f64(t1);
                    let (tx1, ty1) = self.segments[i + 1].tangent_f64(t0);
                    let cross = (tx0 * ty1 - ty0 * tx1).abs();
                    let len0 = (tx0 * tx0 + ty0 * ty0).sqrt().max(1e-15);
                    let len1 = (tx1 * tx1 + ty1 * ty1).sqrt().max(1e-15);
                    if cross / (len0 * len1) > tol {
                        Continuity::G0
                    } else {
                        let k0 = curvature_f64(&self.segments[i], t1);
                        let k1 = curvature_f64(&self.segments[i + 1], t0);
                        if (k0 - k1).abs() > tol { Continuity::G1 } else { Continuity::G2 }
                    }
                }
            };
        }
    }

    // ── Merging ────────────────────────────────────────────────────────────────

    /// Merge consecutive collinear line segments into a single segment.
    pub fn merge_collinear(&self, tol: f64) -> PolyCurve {
        use crate::primitives::LineSeg;
        let mut result: Vec<Curve> = Vec::new();
        let mut i = 0;
        while i < self.segments.len() {
            if let Some(l0) = self.segments[i].as_line() {
                let mut end = l0.p1.clone();
                let start = l0.p0.clone();
                let mut j = i + 1;
                while j < self.segments.len() {
                    if let Some(l1) = self.segments[j].as_line() {
                        let (tx0, ty0) = (
                            (l0.p1.x.to_f64() - l0.p0.x.to_f64()),
                            (l0.p1.y.to_f64() - l0.p0.y.to_f64()),
                        );
                        let (tx1, ty1) = (
                            (l1.p1.x.to_f64() - l1.p0.x.to_f64()),
                            (l1.p1.y.to_f64() - l1.p0.y.to_f64()),
                        );
                        let cross = tx0 * ty1 - ty0 * tx1;
                        let len = ((tx0 * tx0 + ty0 * ty0) * (tx1 * tx1 + ty1 * ty1)).sqrt().max(1e-15);
                        if cross.abs() / len < tol {
                            end = l1.p1.clone();
                            j += 1;
                            continue;
                        }
                    }
                    break;
                }
                result.push(Curve::Line(LineSeg::from_endpoints(start, end)));
                i = j;
            } else {
                result.push(self.segments[i].clone());
                i += 1;
            }
        }
        PolyCurve::new(result)
    }
}

/// Signed curvature of any curve at parameter t (numerical).
fn curvature_f64(curve: &Curve, t: f64) -> f64 {
    let eps = 1e-6;
    let t1 = (t - eps).max(curve.domain().0);
    let t2 = (t + eps).min(curve.domain().1);
    let (x1, y1) = curve.evaluate_f64(t1);
    let (x0, y0) = curve.evaluate_f64(t);
    let (x2, y2) = curve.evaluate_f64(t2);
    let dx1 = x0 - x1;
    let dy1 = y0 - y1;
    let dx2 = x2 - x0;
    let dy2 = y2 - y0;
    let cross = dx1 * dy2 - dy1 * dx2;
    let len = (dx1 * dx1 + dy1 * dy1).sqrt();
    if len < 1e-15 { 0.0 } else { cross / (len * len * len) }
}

// ── CurveSegment impl ─────────────────────────────────────────────────────────

impl CurveSegment for PolyCurve {
    fn implicit_form(&self) -> BivariatePoly {
        // Implicit form of a polycurve is the product of all segment implicit forms.
        // This represents the union of all curve loci (useful for bounding / intersection).
        // Phase 2 note: for boolean ops, individual segment forms are queried; the product
        // is provided here for API completeness.
        if self.segments.is_empty() { return BivariatePoly::zero(); }
        self.segments.iter().skip(1).fold(
            self.segments[0].implicit_form(),
            |acc, seg| acc * seg.implicit_form(),
        )
    }

    fn domain(&self) -> (f64, f64) {
        if self.segments.is_empty() { return (0.0, 1.0); }
        (self.segments[0].domain().0,
         self.segments.last().unwrap().domain().1)
    }

    fn evaluate_f64(&self, t: f64) -> (f64, f64) {
        // Route t to the appropriate segment by dividing the overall parameter range
        // equally among segments.
        let n = self.segments.len();
        if n == 0 { return (0.0, 0.0); }
        let seg_idx = ((t * n as f64) as usize).min(n - 1);
        let t_local = t * n as f64 - seg_idx as f64;
        let (t0, t1) = self.segments[seg_idx].domain();
        let t_mapped = t0 + t_local * (t1 - t0);
        self.segments[seg_idx].evaluate_f64(t_mapped)
    }

    fn bounding_box(&self) -> BoundingBox {
        if self.segments.is_empty() {
            return BoundingBox::from_corners(0.0, 0.0, 0.0, 0.0);
        }
        self.segments.iter().skip(1).fold(
            self.segments[0].bounding_box(),
            |acc, seg| acc.union(&seg.bounding_box()),
        )
    }

    fn tangent_f64(&self, t: f64) -> (f64, f64) {
        let n = self.segments.len();
        if n == 0 { return (1.0, 0.0); }
        let seg_idx = ((t * n as f64) as usize).min(n - 1);
        let t_local = t * n as f64 - seg_idx as f64;
        let (t0, t1) = self.segments[seg_idx].domain();
        let t_mapped = t0 + t_local * (t1 - t0);
        self.segments[seg_idx].tangent_f64(t_mapped)
    }

    fn arc_length(&self) -> f64 {
        self.segments.iter().map(|s| s.arc_length()).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::LineSeg;
    use crate::point::Point2d;

    fn pt(x: i64, y: i64) -> Point2d { Point2d::from_i64(x, y) }
    fn seg(x0: i64, y0: i64, x1: i64, y1: i64) -> Curve {
        Curve::Line(LineSeg::from_endpoints(pt(x0, y0), pt(x1, y1)))
    }

    #[test]
    fn g0_continuity_connected() {
        let pc = PolyCurve::new(vec![
            seg(0, 0, 1, 1),
            seg(1, 1, 2, 0), // connected at (1,1)
        ]);
        assert!(pc.check_g0(1e-9));
    }

    #[test]
    fn g0_continuity_disconnected() {
        let pc = PolyCurve::new(vec![
            seg(0, 0, 1, 1),
            seg(2, 0, 3, 1), // gap at (1,1)→(2,0)
        ]);
        assert!(!pc.check_g0(1e-9));
    }

    #[test]
    fn merge_collinear_lines() {
        // Three collinear segments along y=0: (0,0)→(1,0), (1,0)→(2,0), (2,0)→(5,0)
        let pc = PolyCurve::new(vec![
            seg(0, 0, 1, 0),
            seg(1, 0, 2, 0),
            seg(2, 0, 5, 0),
        ]);
        let merged = pc.merge_collinear(1e-9);
        assert_eq!(merged.segments.len(), 1);
        if let Some(l) = merged.segments[0].as_line() {
            assert_eq!(l.p0, pt(0, 0));
            assert_eq!(l.p1, pt(5, 0));
        } else {
            panic!("Expected a line segment");
        }
    }

    #[test]
    fn total_arc_length() {
        // Two 3-4-5 triangles: total length = 5 + 5 = 10
        let pc = PolyCurve::new(vec![
            seg(0, 0, 3, 4),
            seg(3, 4, 6, 0), // also 3-4-5
        ]);
        assert!((pc.arc_length() - 10.0).abs() < 1e-8);
    }
}
