use exact2d_algebra::Rational;

/// A 2-D point with exact rational coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Point2d {
    pub x: Rational,
    pub y: Rational,
}

impl Point2d {
    pub fn new(x: Rational, y: Rational) -> Self { Point2d { x, y } }

    pub fn from_i64(x: i64, y: i64) -> Self {
        Point2d { x: Rational::from(x), y: Rational::from(y) }
    }

    pub fn from_f64(x: f64, y: f64) -> Self {
        Point2d {
            x: Rational::from_f64_approx(x),
            y: Rational::from_f64_approx(y),
        }
    }

    pub fn to_f64(&self) -> (f64, f64) {
        (self.x.to_f64(), self.y.to_f64())
    }

    /// Exact squared distance.
    pub fn dist_sq(&self, other: &Point2d) -> Rational {
        let dx = self.x.clone() - other.x.clone();
        let dy = self.y.clone() - other.y.clone();
        dx.clone() * dx + dy.clone() * dy
    }

    /// Float distance (sqrt of exact squared distance).
    pub fn dist_f64(&self, other: &Point2d) -> f64 {
        self.dist_sq(other).to_f64().sqrt()
    }

    /// Midpoint (exact).
    pub fn midpoint(&self, other: &Point2d) -> Point2d {
        Point2d {
            x: (self.x.clone() + other.x.clone()) / Rational::from(2i64),
            y: (self.y.clone() + other.y.clone()) / Rational::from(2i64),
        }
    }

    /// Linear interpolation: (1-t)*self + t*other  (exact for Rational t).
    pub fn lerp(&self, other: &Point2d, t: &Rational) -> Point2d {
        let one_minus_t = Rational::one() - t.clone();
        Point2d {
            x: one_minus_t.clone() * self.x.clone() + t.clone() * other.x.clone(),
            y: one_minus_t * self.y.clone() + t.clone() * other.y.clone(),
        }
    }
}

impl std::fmt::Display for Point2d {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

// ── Bounding box ──────────────────────────────────────────────────────────────

/// Axis-aligned bounding box with exact rational corners.
#[derive(Clone, Debug, PartialEq)]
pub struct BoundingBox {
    pub min: Point2d,
    pub max: Point2d,
}

impl BoundingBox {
    pub fn new(min: Point2d, max: Point2d) -> Self { BoundingBox { min, max } }

    pub fn from_corners(x0: f64, y0: f64, x1: f64, y1: f64) -> Self {
        BoundingBox {
            min: Point2d::from_f64(x0.min(x1), y0.min(y1)),
            max: Point2d::from_f64(x0.max(x1), y0.max(y1)),
        }
    }

    pub fn contains_point_f64(&self, x: f64, y: f64) -> bool {
        let (x0, y0) = self.min.to_f64();
        let (x1, y1) = self.max.to_f64();
        x >= x0 && x <= x1 && y >= y0 && y <= y1
    }

    pub fn intersects(&self, other: &BoundingBox) -> bool {
        self.max.x >= other.min.x && self.min.x <= other.max.x &&
        self.max.y >= other.min.y && self.min.y <= other.max.y
    }

    pub fn union(&self, other: &BoundingBox) -> BoundingBox {
        fn rat_min(a: Rational, b: Rational) -> Rational { if a <= b { a } else { b } }
        fn rat_max(a: Rational, b: Rational) -> Rational { if a >= b { a } else { b } }
        BoundingBox {
            min: Point2d {
                x: rat_min(self.min.x.clone(), other.min.x.clone()),
                y: rat_min(self.min.y.clone(), other.min.y.clone()),
            },
            max: Point2d {
                x: rat_max(self.max.x.clone(), other.max.x.clone()),
                y: rat_max(self.max.y.clone(), other.max.y.clone()),
            },
        }
    }

}

impl std::fmt::Display for BoundingBox {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{} → {}]", self.min, self.max)
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn midpoint_exact() {
        let a = Point2d::from_i64(0, 0);
        let b = Point2d::from_i64(4, 6);
        let m = a.midpoint(&b);
        assert_eq!(m, Point2d::new(Rational::from(2i64), Rational::from(3i64)));
    }

    #[test]
    fn lerp_exact() {
        let a = Point2d::from_i64(0, 0);
        let b = Point2d::from_i64(10, 10);
        let t = Rational::new(exact2d_integer::Integer::from(1i64), exact2d_integer::Integer::from(4i64));
        let p = a.lerp(&b, &t);
        assert_eq!(p, Point2d::new(Rational::from_f64_approx(2.5), Rational::from_f64_approx(2.5)));
    }

    #[test]
    fn bbox_intersects() {
        let a = BoundingBox::from_corners(0.0, 0.0, 2.0, 2.0);
        let b = BoundingBox::from_corners(1.0, 1.0, 3.0, 3.0);
        let c = BoundingBox::from_corners(5.0, 5.0, 7.0, 7.0);
        assert!(a.intersects(&b));
        assert!(!a.intersects(&c));
    }
}
