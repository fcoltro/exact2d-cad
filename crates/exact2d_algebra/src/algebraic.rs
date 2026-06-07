use crate::rational::Rational;
use crate::univariate::{UnivariatePoly, sum_resultant, product_resultant};
use std::cmp::Ordering;

/// Exact representation of a real algebraic number.
///
/// Structure matches the spec:
///   - `poly`         — minimal polynomial (or a polynomial it divides; irreducible in Phase 2+)
///   - `lower/upper`  — isolating interval: poly has exactly one root in (lower, upper)
///   - `multiplicity` — root multiplicity (1 for simple roots)
#[derive(Clone, Debug)]
pub struct AlgebraicNumber {
    pub poly: UnivariatePoly,
    pub lower: Rational,
    pub upper: Rational,
    pub multiplicity: u32,
}

impl AlgebraicNumber {
    pub fn new(poly: UnivariatePoly, lower: Rational, upper: Rational) -> Self {
        AlgebraicNumber { poly, lower, upper, multiplicity: 1 }
    }

    pub fn new_with_multiplicity(
        poly: UnivariatePoly, lower: Rational, upper: Rational, mult: u32,
    ) -> Self {
        AlgebraicNumber { poly, lower, upper, multiplicity: mult }
    }

    /// Construct from an exact rational value r. Polynomial is x - r.
    pub fn from_rational(r: Rational) -> Self {
        let p = UnivariatePoly::from_coeffs(vec![-r.clone(), Rational::one()]);
        let small = Rational::new(
            exact2d_integer::Integer::one(),
            exact2d_integer::Integer::from(1_000_000i64),
        );
        AlgebraicNumber {
            poly: p,
            lower: r.clone() - small.clone(),
            upper: r + small,
            multiplicity: 1,
        }
    }

    // ── Simplify to rational ──────────────────────────────────────────────────

    /// If this algebraic number is rational (poly has degree 1), return it as `Rational`.
    /// Also checks if the isolating interval collapses to a point.
    pub fn as_rational(&self) -> Option<Rational> {
        // Degree-1 poly: ax + b = 0  →  x = -b/a
        if self.poly.degree() == 1 {
            let a = self.poly.coeff(1);
            let b = self.poly.coeff(0);
            if !a.is_zero() {
                return Some(-b / a);
            }
        }
        // Collapsed interval
        if self.lower == self.upper {
            return Some(self.lower.clone());
        }
        None
    }

    /// Evaluate to a float with absolute error ≤ `precision`.
    pub fn to_f64(&self, precision: f64) -> f64 {
        if self.lower == self.upper { return self.lower.to_f64(); }
        self.poly.refine_root_f64(&self.lower, &self.upper, precision)
    }

    /// Approximate rational midpoint of the current isolating interval.
    pub fn approx_rational(&self) -> Rational {
        (self.lower.clone() + self.upper.clone()) / Rational::from(2i64)
    }

    /// Refine the isolating interval using exact Rational bisection.
    pub fn refine(&mut self, steps: u32) {
        for _ in 0..steps {
            let mid = (self.lower.clone() + self.upper.clone()) / Rational::from(2i64);
            let f_lo  = self.poly.eval(&self.lower);
            let f_mid = self.poly.eval(&mid);
            if f_mid.is_zero() { self.lower = mid.clone(); self.upper = mid; return; }
            if f_lo.signum() != f_mid.signum() { self.upper = mid; }
            else                               { self.lower = mid; }
        }
    }

    /// Refine until the interval width is less than `target_width` (Rational).
    pub fn refine_to_width(&mut self, target_width: &Rational) {
        loop {
            let width = self.upper.clone() - self.lower.clone();
            if &width <= target_width { break; }
            self.refine(1);
        }
    }

    pub fn compare(&self, other: &Self) -> Ordering {
        let mut a = self.clone();
        let mut b = other.clone();

        // Cheap check: disjoint intervals
        if a.upper <= b.lower { return Ordering::Less; }
        if b.upper <= a.lower { return Ordering::Greater; }

        // Overlapping intervals: check if they share a common root in the overlap.
        let g = UnivariatePoly::gcd(&a.poly, &b.poly);
        if g.degree() >= 1 {
            let lo = a.lower.clone().max(b.lower.clone());
            let hi = a.upper.clone().min(b.upper.clone());

            let g_sqfree = g.make_square_free();
            let seq = g_sqfree.sturm_sequence();

            let vlo = UnivariatePoly::variations_at(&seq, &lo);
            let vhi = UnivariatePoly::variations_at(&seq, &hi);

            let num_roots = (vlo as i64 - vhi as i64)
                + if g_sqfree.eval(&lo).is_zero() { 1 } else { 0 };

            if num_roots > 0 {
                return Ordering::Equal;
            }
        }

        // They do not share a root in the overlap, so they are distinct reals.
        // Refine both isolating intervals until they separate. The loop is bounded
        // so a pathological input can never hang the caller: 200 bisections shrink
        // each interval by 2^-200, far past any genuine separation, after which we
        // fall back to an exact comparison of the refined midpoints.
        for _ in 0..200 {
            if a.upper <= b.lower { return Ordering::Less; }
            if b.upper <= a.lower { return Ordering::Greater; }
            a.refine(1);
            b.refine(1);
        }
        a.approx_rational().cmp(&b.approx_rational())
    }

    // ── Arithmetic via resultant ──────────────────────────────────────────────

    /// Add two algebraic numbers α and β using the exact resultant method.
    ///
    /// Algorithm: R(z) = res_x(p(x), q(z−x)).
    /// Every root of R is of the form α_i + β_j (one per pair).
    /// We use the square-free part of R so bisection works on all multiplicities,
    /// then select the root closest to the float approximation of α+β.
    pub fn add(&self, other: &Self) -> Self {
        let r_raw = sum_resultant(&self.poly, &other.poly);
        // Make square-free: product_resultant can produce repeated roots (e.g. (z²−6)²),
        // and sign-based bisection fails on even-multiplicity roots.
        let r_poly = r_raw.make_square_free();

        let approx = self.to_f64(1e-13) + other.to_f64(1e-13);
        let (lo, hi) = Self::pick_closest_interval(&r_poly, approx,
            self.lower.clone() + other.lower.clone(),
            self.upper.clone() + other.upper.clone());

        AlgebraicNumber { poly: r_poly, lower: lo, upper: hi, multiplicity: 1 }
    }

    /// Multiply two algebraic numbers α and β using the exact resultant method.
    ///
    /// Algorithm: R(z) = res_x(p(x), x^{deg q} · q(z/x)).
    pub fn mul(&self, other: &Self) -> Self {
        let r_raw = product_resultant(&self.poly, &other.poly);
        let r_poly = r_raw.make_square_free();

        let approx = self.to_f64(1e-13) * other.to_f64(1e-13);
        let corners = [
            self.lower.clone() * other.lower.clone(),
            self.lower.clone() * other.upper.clone(),
            self.upper.clone() * other.lower.clone(),
            self.upper.clone() * other.upper.clone(),
        ];
        let fallback_lo = corners.iter().cloned().min().unwrap();
        let fallback_hi = corners.iter().cloned().max().unwrap();
        let (lo, hi) = Self::pick_closest_interval(&r_poly, approx, fallback_lo, fallback_hi);

        AlgebraicNumber { poly: r_poly, lower: lo, upper: hi, multiplicity: 1 }
    }

    /// Among all isolating intervals of `poly`, pick the one whose midpoint is
    /// nearest to `approx`.  Falls back to `(fallback_lo, fallback_hi)` if no roots found.
    fn pick_closest_interval(
        poly: &UnivariatePoly,
        approx: f64,
        fallback_lo: Rational,
        fallback_hi: Rational,
    ) -> (Rational, Rational) {
        let intervals = poly.real_root_isolate();
        intervals.into_iter()
            .min_by(|(a1, b1), (a2, b2)| {
                let m1 = (a1.to_f64() + b1.to_f64()) / 2.0;
                let m2 = (a2.to_f64() + b2.to_f64()) / 2.0;
                // total_cmp, not partial_cmp().unwrap(): a NaN midpoint (degenerate
                // interval) must order deterministically, never panic the kernel.
                (m1 - approx).abs().total_cmp(&(m2 - approx).abs())
            })
            .unwrap_or((fallback_lo, fallback_hi))
    }
}

impl PartialEq for AlgebraicNumber {
    fn eq(&self, other: &Self) -> bool { self.compare(other) == Ordering::Equal }
}

impl Eq for AlgebraicNumber {}

impl PartialOrd for AlgebraicNumber {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}

impl Ord for AlgebraicNumber {
    fn cmp(&self, other: &Self) -> Ordering { self.compare(other) }
}

impl std::fmt::Display for AlgebraicNumber {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let v = self.to_f64(1e-6);
        write!(f, "~{:.6} ∈ [{:.6}, {:.6}] (mult={})",
            v, self.lower.to_f64(), self.upper.to_f64(), self.multiplicity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn alg_sqrt2() -> AlgebraicNumber {
        let poly = UnivariatePoly::from_coeffs(vec![
            Rational::from(-2i64), Rational::zero(), Rational::one(),
        ]);
        AlgebraicNumber::new(poly, Rational::from(1i64), Rational::from(2i64))
    }

    #[test]
    fn sqrt2_as_algebraic() {
        let v = alg_sqrt2().to_f64(1e-12);
        assert!((v - std::f64::consts::SQRT_2).abs() < 1e-10);
    }

    #[test]
    fn multiplicity_field() {
        let a = alg_sqrt2();
        assert_eq!(a.multiplicity, 1);
        let b = AlgebraicNumber::new_with_multiplicity(
            a.poly.clone(), a.lower.clone(), a.upper.clone(), 2,
        );
        assert_eq!(b.multiplicity, 2);
    }

    #[test]
    fn as_rational_degree1() {
        // x - 3/2 = 0  →  rational 3/2
        let poly = UnivariatePoly::from_coeffs(vec![
            Rational::new(exact2d_integer::Integer::from(-3i64), exact2d_integer::Integer::from(2i64)),
            Rational::one(),
        ]);
        let a = AlgebraicNumber::new(poly,
            Rational::from(1i64), Rational::from(2i64));
        assert_eq!(a.as_rational(), Some(
            Rational::new(exact2d_integer::Integer::from(3i64), exact2d_integer::Integer::from(2i64))
        ));
    }

    #[test]
    fn as_rational_irrational_returns_none() {
        assert_eq!(alg_sqrt2().as_rational(), None);
    }

    #[test]
    fn compare_ordering() {
        // √2 > 1
        let sqrt2 = alg_sqrt2();
        let one = AlgebraicNumber::from_rational(Rational::from(1i64));
        assert_eq!(sqrt2.compare(&one), std::cmp::Ordering::Greater);
        assert_eq!(one.compare(&sqrt2), std::cmp::Ordering::Less);
    }

    #[test]
    fn compare_equal() {
        let a = alg_sqrt2();
        let b = alg_sqrt2();
        assert_eq!(a.compare(&b), std::cmp::Ordering::Equal);
    }

    fn alg_sqrt3() -> AlgebraicNumber {
        let poly = UnivariatePoly::from_coeffs(vec![
            Rational::from(-3i64), Rational::zero(), Rational::one(),
        ]);
        AlgebraicNumber::new(poly, Rational::from(1i64), Rational::from(2i64))
    }

    #[test]
    fn add_sqrt2_plus_sqrt3_exact_poly() {
        // √2 + √3 ≈ 3.1462...  Its minimal polynomial is x⁴ - 10x² + 1.
        let s = alg_sqrt2().add(&alg_sqrt3());
        // The result polynomial should have a root near √2 + √3
        let v = s.to_f64(1e-12);
        let expected = std::f64::consts::SQRT_2 + 3f64.sqrt();
        assert!((v - expected).abs() < 1e-8, "got {}, expected {}", v, expected);
        // Verify the result poly has the right root
        let residual = s.poly.eval_f64(v).abs();
        assert!(residual < 1e-6, "poly residual at result: {}", residual);
        // Degree of result poly should be deg(p)*deg(q) = 2*2 = 4
        assert_eq!(s.poly.degree(), 4, "sum poly should be degree 4");
    }

    #[test]
    fn mul_sqrt2_times_sqrt3_exact_poly() {
        // √2 * √3 = √6 ≈ 2.449.  Minimal polynomial of √6: x² − 6.
        // product_resultant gives (z²−6)² which make_square_free reduces to z²−6 (degree 2).
        let p = alg_sqrt2().mul(&alg_sqrt3());
        let v = p.to_f64(1e-12);
        let expected = 6f64.sqrt();
        assert!((v - expected).abs() < 1e-8, "got {}, expected {}", v, expected);
        // After square-free reduction the poly is the minimal polynomial of √6
        assert_eq!(p.poly.degree(), 2, "square-free part should be degree 2 (z²−6)");
        let residual = p.poly.eval_f64(v).abs();
        assert!(residual < 1e-6, "poly residual: {}", residual);
    }

    #[test]
    fn add_two_rationals() {
        // 3/2 + 1/2 = 2
        let a = AlgebraicNumber::from_rational(
            Rational::new(exact2d_integer::Integer::from(3i64), exact2d_integer::Integer::from(2i64))
        );
        let b = AlgebraicNumber::from_rational(
            Rational::new(exact2d_integer::Integer::from(1i64), exact2d_integer::Integer::from(2i64))
        );
        let c = a.add(&b);
        let v = c.to_f64(1e-10);
        assert!((v - 2.0).abs() < 1e-6, "sum={}", v);
    }

    #[test]
    fn mul_two_rationals() {
        // 3 * 4 = 12
        let a = AlgebraicNumber::from_rational(Rational::from(3i64));
        let b = AlgebraicNumber::from_rational(Rational::from(4i64));
        let c = a.mul(&b);
        let v = c.to_f64(1e-10);
        assert!((v - 12.0).abs() < 1e-6, "product={}", v);
    }

    #[test]
    fn exact_equality_with_different_polynomials() {
        // √2 represented by x² - 2
        let poly1 = UnivariatePoly::from_coeffs(vec![
            Rational::from(-2i64), Rational::zero(), Rational::one(),
        ]);
        let a = AlgebraicNumber::new(poly1, Rational::from(1i64), Rational::from(2i64));

        // √2 represented by x⁴ - 4 = (x²-2)(x²+2)
        let poly2 = UnivariatePoly::from_coeffs(vec![
            Rational::from(-4i64), Rational::zero(), Rational::zero(), Rational::zero(), Rational::one(),
        ]);
        let b = AlgebraicNumber::new(poly2, Rational::from(1i64), Rational::from(2i64));

        // Even though they have different polynomials, they represent the same real number root.
        // The exact GCD comparison should successfully identify them as equal.
        assert_eq!(a, b);
        assert_eq!(a.compare(&b), Ordering::Equal);
    }
}
