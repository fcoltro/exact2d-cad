//! Dimension annotations (spec §4.1) — real, associative annotation entities that
//! *measure* geometry and display the value, styled by a `DimStyle`. Unlike the
//! parametric distance constraint (which drives geometry), a dimension only
//! reports; its value is recomputed from its points, so it stays correct as the
//! drawing changes. Works with or without parametric mode.

use exact2d_geometry::{Point2d, BoundingBox, Transform2d};

/// Orientation of a linear dimension.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinearOrient {
    /// Measures the horizontal (X) distance.
    Horizontal,
    /// Measures the vertical (Y) distance.
    Vertical,
    /// Measures the straight-line distance, dimension line parallel to p1→p2.
    Aligned,
}

/// What a dimension measures and the geometry needed to draw it.
#[derive(Clone, Debug)]
pub enum DimKind {
    /// Linear distance between two points; the dimension line passes through
    /// `line_point` (which sets the offset), oriented per `orientation`.
    Linear { p1: Point2d, p2: Point2d, line_point: Point2d, orientation: LinearOrient },
    /// Radius of a circle/arc: `center` to a point `edge` on the circle.
    Radial { center: Point2d, edge: Point2d },
    /// Diameter of a circle: `center` with an `edge` point on the circle.
    Diameter { center: Point2d, edge: Point2d },
}

/// A dimension annotation: what it measures, which style draws it, and an optional
/// manual text override.
#[derive(Clone, Debug)]
pub struct Dimension {
    pub kind: DimKind,
    /// Index into the document's `DimStyleTable`.
    pub style: usize,
    /// Manual text; `None` shows the measured value formatted by the style.
    pub text_override: Option<String>,
}

fn dist(a: &Point2d, b: &Point2d) -> f64 {
    let (ax, ay) = a.to_f64();
    let (bx, by) = b.to_f64();
    ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt()
}

impl Dimension {
    pub fn new(kind: DimKind, style: usize) -> Self {
        Dimension { kind, style, text_override: None }
    }

    /// The measured value: length for linear, radius/diameter for circular.
    pub fn measure(&self) -> f64 {
        match &self.kind {
            DimKind::Linear { p1, p2, orientation, .. } => {
                let (x1, y1) = p1.to_f64();
                let (x2, y2) = p2.to_f64();
                match orientation {
                    LinearOrient::Horizontal => (x2 - x1).abs(),
                    LinearOrient::Vertical => (y2 - y1).abs(),
                    LinearOrient::Aligned => ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt(),
                }
            }
            DimKind::Radial { center, edge } => dist(center, edge),
            DimKind::Diameter { center, edge } => 2.0 * dist(center, edge),
        }
    }

    /// The text to display: the override if set, else the measured value formatted
    /// by `style` (radial/diameter get the R/⌀ prefix).
    pub fn display_text(&self, style: &DimStyle) -> String {
        if let Some(t) = &self.text_override {
            return t.clone();
        }
        let v = style.format(self.measure());
        match &self.kind {
            DimKind::Radial { .. } => format!("R{v}"),
            DimKind::Diameter { .. } => format!("⌀{v}"),
            DimKind::Linear { .. } => v,
        }
    }

    /// Affine-transform all defining points.
    pub fn transformed(&self, t: &Transform2d) -> Dimension {
        let p = |pt: &Point2d| t.apply_point(pt);
        let kind = match &self.kind {
            DimKind::Linear { p1, p2, line_point, orientation } => DimKind::Linear {
                p1: p(p1), p2: p(p2), line_point: p(line_point), orientation: *orientation,
            },
            DimKind::Radial { center, edge } => DimKind::Radial { center: p(center), edge: p(edge) },
            DimKind::Diameter { center, edge } => DimKind::Diameter { center: p(center), edge: p(edge) },
        };
        Dimension { kind, style: self.style, text_override: self.text_override.clone() }
    }

    /// Bounding box covering the defining points (enough for ZOOM extents).
    pub fn bounding_box(&self) -> BoundingBox {
        let pts: Vec<&Point2d> = match &self.kind {
            DimKind::Linear { p1, p2, line_point, .. } => vec![p1, p2, line_point],
            DimKind::Radial { center, edge } | DimKind::Diameter { center, edge } => vec![center, edge],
        };
        let (mut minx, mut miny) = pts[0].to_f64();
        let (mut maxx, mut maxy) = (minx, miny);
        for p in &pts {
            let (x, y) = p.to_f64();
            minx = minx.min(x); miny = miny.min(y);
            maxx = maxx.max(x); maxy = maxy.max(y);
        }
        BoundingBox::from_corners(minx, miny, maxx, maxy)
    }
}

/// A dimension style (DIMSTYLE): appearance + value formatting. World units.
#[derive(Clone, Debug)]
pub struct DimStyle {
    pub name: String,
    /// Text height.
    pub text_height: f64,
    /// Arrowhead length.
    pub arrow_size: f64,
    /// Gap between the measured geometry and where extension lines start.
    pub extension_offset: f64,
    /// How far extension lines continue past the dimension line.
    pub extension_beyond: f64,
    /// Decimal places shown.
    pub precision: usize,
    /// Suffix appended to the value (e.g. " mm").
    pub suffix: String,
}

impl DimStyle {
    /// The default "Standard" style.
    pub fn standard() -> Self {
        DimStyle {
            name: "Standard".to_string(),
            text_height: 2.5,
            arrow_size: 2.5,
            extension_offset: 0.6,
            extension_beyond: 1.25,
            precision: 2,
            suffix: String::new(),
        }
    }

    /// Format a measured value with this style's precision and suffix.
    pub fn format(&self, value: f64) -> String {
        format!("{:.*}{}", self.precision, value, self.suffix)
    }
}

/// The dimension-style table: ordered styles with a "current" one. Mirrors
/// `LayerTable`; style 0 is "Standard" and always present.
#[derive(Clone, Debug)]
pub struct DimStyleTable {
    pub styles: Vec<DimStyle>,
    pub current: usize,
}

impl Default for DimStyleTable {
    fn default() -> Self {
        DimStyleTable { styles: vec![DimStyle::standard()], current: 0 }
    }
}

impl DimStyleTable {
    pub fn current_style(&self) -> &DimStyle { &self.styles[self.current] }

    /// Style at `i`, falling back to "Standard" (index 0) if out of range.
    pub fn get(&self, i: usize) -> &DimStyle {
        self.styles.get(i).unwrap_or(&self.styles[0])
    }

    /// Add a style; returns its index.
    pub fn add(&mut self, style: DimStyle) -> usize {
        self.styles.push(style);
        self.styles.len() - 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> Point2d { Point2d::from_f64(x, y) }

    #[test]
    fn linear_measures_per_orientation() {
        let mk = |o| Dimension::new(DimKind::Linear {
            p1: pt(0.0, 0.0), p2: pt(3.0, 4.0), line_point: pt(0.0, -2.0), orientation: o,
        }, 0);
        assert!((mk(LinearOrient::Horizontal).measure() - 3.0).abs() < 1e-9);
        assert!((mk(LinearOrient::Vertical).measure() - 4.0).abs() < 1e-9);
        assert!((mk(LinearOrient::Aligned).measure() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn radial_and_diameter() {
        let r = Dimension::new(DimKind::Radial { center: pt(0.0, 0.0), edge: pt(5.0, 0.0) }, 0);
        let d = Dimension::new(DimKind::Diameter { center: pt(0.0, 0.0), edge: pt(5.0, 0.0) }, 0);
        assert!((r.measure() - 5.0).abs() < 1e-9);
        assert!((d.measure() - 10.0).abs() < 1e-9);
        let style = DimStyle::standard();
        assert_eq!(r.display_text(&style), "R5.00");
        assert_eq!(d.display_text(&style), "⌀10.00");
    }

    #[test]
    fn measurement_is_associative_under_transform() {
        use exact2d_algebra::Rational;
        let dim = Dimension::new(DimKind::Linear {
            p1: pt(0.0, 0.0), p2: pt(4.0, 0.0), line_point: pt(0.0, -2.0),
            orientation: LinearOrient::Aligned,
        }, 0);
        // Scale ×2 → measured length doubles (dimensions are associative).
        let t = Transform2d::scale(Rational::from(2i64), Rational::from(2i64));
        let moved = dim.transformed(&t);
        assert!((moved.measure() - 8.0).abs() < 1e-6, "got {}", moved.measure());
    }
}
