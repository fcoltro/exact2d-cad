use crate::rational::Rational;
use std::fmt;
use std::ops::{Add, Sub, Mul, Neg};

/// Dense univariate polynomial over ℚ.
/// `coeffs[i]` is the coefficient of x^i.
/// Invariant: the last element of `coeffs` is non-zero (or `coeffs` is empty for the zero polynomial).
#[derive(Clone, PartialEq, Debug)]
pub struct UnivariatePoly {
    pub(crate) coeffs: Vec<Rational>,
}

impl UnivariatePoly {
    // ── Constructors ──────────────────────────────────────────────────────────

    pub fn zero() -> Self { UnivariatePoly { coeffs: vec![] } }
    pub fn one()  -> Self { UnivariatePoly { coeffs: vec![Rational::one()] } }

    pub fn constant(c: Rational) -> Self {
        let mut p = UnivariatePoly { coeffs: if c.is_zero() { vec![] } else { vec![c] } };
        p.trim();
        p
    }

    pub fn from_coeffs(coeffs: Vec<Rational>) -> Self {
        let mut p = UnivariatePoly { coeffs };
        p.trim();
        p
    }

    /// Remove trailing zero coefficients.
    fn trim(&mut self) {
        while self.coeffs.last().is_some_and(|c| c.is_zero()) {
            self.coeffs.pop();
        }
    }

    // ── Properties ────────────────────────────────────────────────────────────

    pub fn is_zero(&self) -> bool { self.coeffs.is_empty() }

    /// Degree of the polynomial (0 for zero poly and constants).
    pub fn degree(&self) -> usize {
        if self.coeffs.is_empty() { 0 } else { self.coeffs.len() - 1 }
    }

    pub fn leading_coeff(&self) -> &Rational {
        self.coeffs.last().expect("leading_coeff called on zero polynomial")
    }

    pub fn coeff(&self, i: usize) -> Rational {
        self.coeffs.get(i).cloned().unwrap_or_else(Rational::zero)
    }

    pub fn set_coeff(&mut self, i: usize, c: Rational) {
        if i >= self.coeffs.len() {
            self.coeffs.resize(i + 1, Rational::zero());
        }
        self.coeffs[i] = c;
        self.trim();
    }

    // ── Evaluation ────────────────────────────────────────────────────────────

    /// Evaluate at a rational point using Horner's method.
    pub fn eval(&self, x: &Rational) -> Rational {
        if self.coeffs.is_empty() { return Rational::zero(); }
        let mut result = self.coeffs.last().unwrap().clone();
        for i in (0..self.coeffs.len() - 1).rev() {
            result = result * x.clone() + self.coeffs[i].clone();
        }
        result
    }

    /// Fast float evaluation for root refinement.
    pub fn eval_f64(&self, x: f64) -> f64 {
        if self.coeffs.is_empty() { return 0.0; }
        let mut result = self.coeffs.last().unwrap().to_f64();
        for i in (0..self.coeffs.len() - 1).rev() {
            result = result * x + self.coeffs[i].to_f64();
        }
        result
    }

    // ── Calculus ──────────────────────────────────────────────────────────────

    pub fn derivative(&self) -> Self {
        if self.coeffs.len() <= 1 { return Self::zero(); }
        let coeffs = self.coeffs.iter().enumerate().skip(1)
            .map(|(i, c)| Rational::from(i as i64) * c.clone())
            .collect();
        UnivariatePoly { coeffs }
    }

    // ── Division ──────────────────────────────────────────────────────────────

    /// Polynomial long division. Returns (quotient, remainder).
    pub fn divrem(dividend: &Self, divisor: &Self) -> (Self, Self) {
        if divisor.is_zero() { panic!("Division by zero polynomial"); }
        if dividend.coeffs.len() < divisor.coeffs.len() {
            return (Self::zero(), dividend.clone());
        }

        let m = dividend.degree();
        let n = divisor.degree();
        let quot_len = m - n + 1;
        let lead_d = divisor.leading_coeff().clone();

        let mut rem = dividend.coeffs.clone();
        let mut quot = vec![Rational::zero(); quot_len];

        for i in (0..quot_len).rev() {
            let idx = i + n;
            if !rem[idx].is_zero() {
                let factor = rem[idx].clone() / lead_d.clone();
                quot[i] = factor.clone();
                for j in 0..=n {
                    let sub = factor.clone() * divisor.coeffs[j].clone();
                    let val = rem[i + j].clone() - sub;
                    rem[i + j] = val;
                }
            }
        }

        let mut q = UnivariatePoly { coeffs: quot };
        q.trim();
        let mut r = UnivariatePoly { coeffs: rem };
        r.trim();
        (q, r)
    }

    /// Exact division — panics if not exact.
    pub fn exact_div(a: &Self, b: &Self) -> Self {
        let (q, r) = Self::divrem(a, b);
        assert!(r.is_zero(), "exact_div: non-zero remainder");
        q
    }

    // ── GCD ───────────────────────────────────────────────────────────────────

    /// Polynomial GCD over ℚ, returned monic.
    pub fn gcd(a: &Self, b: &Self) -> Self {
        if b.is_zero() { return Self::make_monic(a); }
        if a.is_zero() { return Self::make_monic(b); }
        let (_, r) = Self::divrem(a, b);
        if r.is_zero() { Self::make_monic(b) } else { Self::gcd(b, &r) }
    }

    fn make_monic(p: &Self) -> Self {
        if p.is_zero() { return Self::zero(); }
        let lc = p.leading_coeff().clone();
        UnivariatePoly::from_coeffs(p.coeffs.iter().map(|c| c.clone() / lc.clone()).collect())
    }

    // ── Sturm Sequences & Root Isolation ─────────────────────────────────────

    /// Builds the Sturm chain: [p, p', -rem(p, p'), -rem(p', ...), ...]
    pub fn sturm_sequence(&self) -> Vec<Self> {
        let p0 = self.clone();
        let p1 = self.derivative();
        if p1.is_zero() { return vec![p0]; }

        let mut seq = vec![p0, p1];
        loop {
            let n = seq.len();
            let (_, r) = Self::divrem(&seq[n - 2], &seq[n - 1]);
            if r.is_zero() { break; }
            seq.push(-r);
        }
        seq
    }

    fn sign_of(r: &Rational) -> i32 { r.signum() }

    fn count_variations(signs: &[i32]) -> usize {
        let filtered: Vec<i32> = signs.iter().copied().filter(|&s| s != 0).collect();
        filtered.windows(2).filter(|w| w[0] != w[1]).count()
    }

    /// Number of sign variations in the Sturm sequence at a rational point.
    pub fn variations_at(seq: &[Self], x: &Rational) -> usize {
        let signs: Vec<i32> = seq.iter().map(|p| Self::sign_of(&p.eval(x))).collect();
        Self::count_variations(&signs)
    }

    /// Sign variations as x → +∞ (determined by leading coefficient and degree).
    pub fn variations_at_pos_inf(seq: &[Self]) -> usize {
        let signs: Vec<i32> = seq.iter()
            .filter(|p| !p.is_zero())
            .map(|p| p.leading_coeff().signum())
            .collect();
        Self::count_variations(&signs)
    }

    /// Sign variations as x → -∞.
    pub fn variations_at_neg_inf(seq: &[Self]) -> usize {
        let signs: Vec<i32> = seq.iter()
            .filter(|p| !p.is_zero())
            .map(|p| {
                let s = p.leading_coeff().signum();
                if p.degree() % 2 == 1 { -s } else { s }
            })
            .collect();
        Self::count_variations(&signs)
    }

    /// Cauchy's root bound: all real roots satisfy |r| ≤ 1 + max|a_i / a_n|.
    pub fn cauchy_bound(&self) -> Rational {
        if self.degree() == 0 { return Rational::one(); }
        let lc = self.leading_coeff().clone();
        let mut bound = Rational::zero();
        for i in 0..self.coeffs.len() - 1 {
            let ratio = (self.coeffs[i].clone() / lc.clone()).abs();
            if ratio > bound { bound = ratio; }
        }
        bound + Rational::one()
    }

    /// Returns isolating intervals `(lower, upper)` for every distinct real root.
    /// Uses Sturm's theorem: count(roots in (a,b)) = V(a) - V(b).
    pub fn real_root_isolate(&self) -> Vec<(Rational, Rational)> {
        if self.is_zero() || self.degree() == 0 { return vec![]; }

        // Divide by square-free part for Sturm to work correctly with multiple roots
        let sq_free = self.make_square_free();
        let seq = sq_free.sturm_sequence();

        let bound = sq_free.cauchy_bound();
        let neg_bound = -bound.clone();

        let total_roots = Self::variations_at_neg_inf(&seq) as i64
            - Self::variations_at_pos_inf(&seq) as i64;
        if total_roots == 0 { return vec![]; }

        let mut result = Vec::new();
        Self::isolate_recursive(&sq_free, &seq, &neg_bound, &bound, &mut result);
        result
    }

    fn isolate_recursive(
        poly: &Self,
        seq: &[Self],
        a: &Rational,
        b: &Rational,
        result: &mut Vec<(Rational, Rational)>,
    ) {
        let va = Self::variations_at(seq, a) as i64;
        let vb = Self::variations_at(seq, b) as i64;
        let n = va - vb;
        if n <= 0 { return; }
        if n == 1 {
            // Check if a itself is a root (endpoint)
            if !poly.eval(a).is_zero() {
                result.push((a.clone(), b.clone()));
            } else {
                result.push((a.clone(), a.clone())); // exact root at a
            }
            return;
        }
        // Bisect to separate the roots. Sturm counting needs a split point that is
        // not itself a root. If the exact midpoint happens to be one, step toward
        // `b` by a fraction of the interval width, halving until we land off the
        // root. Unlike a fixed epsilon, this can never jump over a neighbouring
        // root: it terminates once the step is smaller than the (always positive)
        // gap to the next isolated root.
        let mut mid = (a.clone() + b.clone()) / Rational::from(2i64);
        if poly.eval(&mid).is_zero() {
            let mut delta = (b.clone() - a.clone()) / Rational::from(4i64);
            loop {
                let cand = mid.clone() + delta.clone();
                if &cand < b && !poly.eval(&cand).is_zero() {
                    mid = cand;
                    break;
                }
                delta = delta / Rational::from(2i64);
            }
        }
        Self::isolate_recursive(poly, seq, a, &mid, result);
        Self::isolate_recursive(poly, seq, &mid, b, result);
    }

    /// Square-free part via Yun's factorization: product of all distinct irreducible factors.
    pub fn make_square_free(&self) -> Self {
        let factors = self.yun_square_free_factorization();
        if factors.is_empty() { return Self::make_monic(self); }
        // Square-free part = product of all factors (each appears once)
        let mut result = Self::one();
        for f in &factors {
            result = result * f.clone();
        }
        Self::make_monic(&result)
    }

    /// Refine a root in interval (a, b) to absolute precision via bisection (using f64 for speed).
    pub fn refine_root_f64(&self, a: &Rational, b: &Rational, precision: f64) -> f64 {
        // Handle exact endpoint roots
        if a == b { return a.to_f64(); }

        let mut lo = a.to_f64();
        let mut hi = b.to_f64();
        let fa = self.eval_f64(lo);
        let fb = self.eval_f64(hi);

        // If both have the same sign, try the original rational interval endpoints
        if fa * fb > 0.0 {
            // Fall back to rational evaluation at boundary
            let ra = self.eval(a);
            let rb = self.eval(b);
            if ra.is_zero() { return lo; }
            if rb.is_zero() { return hi; }
        }

        let sign_lo = fa.signum();

        while (hi - lo).abs() > precision {
            let mid = (lo + hi) / 2.0;
            if mid == lo || mid == hi { break; } // float precision exhausted
            let fm = self.eval_f64(mid);
            if fm == 0.0 { return mid; }
            if fm.signum() == sign_lo { lo = mid; } else { hi = mid; }
        }
        (lo + hi) / 2.0
    }

    /// All real roots as f64 approximations.
    ///
    /// Refinement is done on the square-free part: an even-multiplicity root of
    /// `self` (e.g. a double root from a tangency or symmetric configuration) does
    /// not change sign, which would break the sign-bisection in `refine_root_f64`.
    /// The square-free part turns every root into a simple, sign-changing root.
    pub fn real_roots_f64(&self, precision: f64) -> Vec<f64> {
        let sq_free = self.make_square_free();
        sq_free.real_root_isolate()
            .iter()
            .map(|(a, b)| sq_free.refine_root_f64(a, b, precision))
            .collect()
    }

    // ── Scale by rational ─────────────────────────────────────────────────────

    pub fn scale(&self, s: &Rational) -> Self {
        UnivariatePoly::from_coeffs(self.coeffs.iter().map(|c| c.clone() * s.clone()).collect())
    }

    // ── Content and primitive part ────────────────────────────────────────────

    /// GCD of all rational coefficients (as rationals: min-denominator form).
    /// Returned value c satisfies: every coefficient of `self/c` is an integer
    /// in lowest terms.  For a monic polynomial this is 1.
    pub fn content(&self) -> Rational {
        if self.is_zero() { return Rational::one(); }
        // Content = GCD of all coefficients over ℚ = (GCD of numerators) / (LCM of denominators)
        // Simpler: make all coeffs have integer numerators by multiplying through, then GCD.
        // We expose a well-defined rational content: just return the leading coefficient so
        // that primitive_part() is monic (standard choice over ℚ where content is a unit).
        self.leading_coeff().clone()
    }

    /// Primitive part: `self / content(self)` — always monic over ℚ.
    pub fn primitive_part(&self) -> Self {
        if self.is_zero() { return Self::zero(); }
        Self::make_monic(self)
    }

    // ── Yun's square-free factorization ───────────────────────────────────────

    /// Yun's algorithm: returns the square-free factorization
    ///   `self = c · f₁ · f₂² · f₃³ · ...`
    /// as a `Vec<UnivariatePoly>` where the i-th element (0-indexed) is `f_{i+1}`.
    ///
    /// Each `fₖ` is monic and square-free.  The product of `factors[k]^(k+1)` over
    /// all k equals the primitive part of `self`.
    pub fn yun_square_free_factorization(&self) -> Vec<UnivariatePoly> {
        let p = self.primitive_part();
        if p.degree() == 0 { return vec![]; }

        let dp = p.derivative();
        if dp.is_zero() {
            // Polynomial over ℚ with zero derivative is a constant — no factors
            return vec![p];
        }

        let a = Self::gcd(&p, &dp);                 // a = gcd(p, p')
        let mut b = Self::exact_div(&p, &a);       // b = p / a
        let mut c = Self::exact_div(&dp, &a);      // c = p' / a
        let mut d = c - b.derivative();            // d = c - b'

        let mut factors: Vec<UnivariatePoly> = Vec::new();

        loop {
            let fk = Self::gcd(&b, &d);            // fk is the next square-free factor
            factors.push(fk.clone());
            b = Self::exact_div(&b, &fk);
            c = Self::exact_div(&d, &fk);
            if b.degree() == 0 { break; }
            d = c - b.derivative();
        }

        factors
    }

    // ── Extended Euclidean algorithm ──────────────────────────────────────────

    /// Extended GCD: returns `(g, s, t)` such that `a·s + b·t = g = gcd(a, b)`.
    /// `g` is monic.  Useful for computing Bézout coefficients.
    pub fn extended_gcd(a: &Self, b: &Self) -> (Self, Self, Self) {
        if b.is_zero() {
            if a.is_zero() {
                return (Self::zero(), Self::one(), Self::zero());
            }
            let lc = a.leading_coeff().clone();
            let monic_a = Self::make_monic(a);
            let s = UnivariatePoly::constant(Rational::one() / lc);
            return (monic_a, s, Self::zero());
        }

        let (q, r) = Self::divrem(a, b);
        if r.is_zero() {
            let lc = b.leading_coeff().clone();
            let monic_b = Self::make_monic(b);
            let t = UnivariatePoly::constant(Rational::one() / lc);
            return (monic_b, Self::zero(), t);
        }

        let (g, s1, t1) = Self::extended_gcd(b, &r);
        // a = q*b + r  →  s = t1, t = s1 - q*t1
        let s = t1.clone();
        let t = s1 - q * t1;
        (g, s, t)
    }

    // ── Sturm sequence caching ────────────────────────────────────────────────

    /// Build and return the Sturm sequence, caching it inside a `SturmCache`.
    /// Use this when you need to query many points against the same polynomial.
    pub fn sturm_cache(&self) -> SturmCache {
        SturmCache { seq: self.sturm_sequence() }
    }
}

// ── Sturm cache ───────────────────────────────────────────────────────────────

/// Pre-computed Sturm sequence for repeated root counting against the same polynomial.
pub struct SturmCache {
    seq: Vec<UnivariatePoly>,
}

impl SturmCache {
    pub fn variations_at(&self, x: &Rational) -> usize {
        UnivariatePoly::variations_at(&self.seq, x)
    }
    pub fn variations_at_pos_inf(&self) -> usize {
        UnivariatePoly::variations_at_pos_inf(&self.seq)
    }
    pub fn variations_at_neg_inf(&self) -> usize {
        UnivariatePoly::variations_at_neg_inf(&self.seq)
    }
    pub fn count_roots_in(&self, a: &Rational, b: &Rational) -> i64 {
        self.variations_at(a) as i64 - self.variations_at(b) as i64
    }
}

// ── Shared Bareiss determinant for UnivariatePoly matrices ────────────────────

/// Fraction-free Gaussian (Bareiss) determinant for a square matrix
/// whose entries are `UnivariatePoly`.  Stays in the polynomial ring throughout.
/// Used by both `bivariate.rs` (Sylvester resultant) and `algebraic.rs` (sum/product resultants).
pub(crate) fn poly_matrix_det(mut mat: Vec<Vec<UnivariatePoly>>) -> UnivariatePoly {
    let n = mat.len();
    if n == 0 { return UnivariatePoly::one(); }
    if n == 1 { return mat[0][0].clone(); }

    let mut sign_neg = false;
    let mut prev_pivot = UnivariatePoly::one();

    for k in 0..n {
        let pivot_row = (k..n).find(|&i| !mat[i][k].is_zero());
        let pivot_row = match pivot_row {
            Some(r) => r,
            None => return UnivariatePoly::zero(),
        };
        if pivot_row != k {
            mat.swap(k, pivot_row);
            sign_neg = !sign_neg;
        }
        let pivot = mat[k][k].clone();
        for i in (k + 1)..n {
            for j in (k + 1)..n {
                let num = &pivot * &mat[i][j] - &mat[i][k] * &mat[k][j];
                mat[i][j] = UnivariatePoly::exact_div(&num, &prev_pivot);
            }
            mat[i][k] = UnivariatePoly::zero();
        }
        prev_pivot = pivot;
    }
    let det = mat[n - 1][n - 1].clone();
    if sign_neg { -det } else { det }
}

// ── Sum-resultant and product-resultant ───────────────────────────────────────

fn binom_rat(k: usize, j: usize) -> Rational {
    if j > k { return Rational::zero(); }
    let j = j.min(k - j);
    let mut r = Rational::one();
    for i in 0..j {
        r = r * Rational::from((k - i) as i64) / Rational::from((i + 1) as i64);
    }
    r
}

/// Coefficients of q(z − x) viewed as a polynomial in x.
/// Returns `vec[j]` = coefficient of x^j, which is a UnivariatePoly in z.
/// Formula: coeff[j] = (−1)^j · Σ_{k=j}^{n} b_k · C(k,j) · z^{k−j}
fn q_z_minus_x_coeffs(q: &UnivariatePoly) -> Vec<UnivariatePoly> {
    let n = q.degree();
    let mut result = vec![UnivariatePoly::zero(); n + 1];
    for j in 0..=n {
        let sign = if j % 2 == 0 { Rational::one() } else { Rational::minus_one() };
        let mut z_coeffs = vec![Rational::zero(); n - j + 1];
        for k in j..=n {
            let bk = q.coeff(k);
            if !bk.is_zero() {
                z_coeffs[k - j] = sign.clone() * bk * binom_rat(k, j);
            }
        }
        result[j] = UnivariatePoly::from_coeffs(z_coeffs);
    }
    result
}

/// Compute res_x(p(x), q(z − x)) — the polynomial in z whose roots are
/// all pairwise sums α_i + β_j where p(α_i)=0 and q(β_j)=0.
/// Used for exact `AlgebraicNumber` addition.
pub fn sum_resultant(p: &UnivariatePoly, q: &UnivariatePoly) -> UnivariatePoly {
    let m = p.degree();
    let n = q.degree();
    if m == 0 || n == 0 { return UnivariatePoly::zero(); }

    let p_coeffs: Vec<UnivariatePoly> = (0..=m)
        .map(|k| UnivariatePoly::constant(p.coeff(k)))
        .collect();
    let q_coeffs = q_z_minus_x_coeffs(q);

    let size = m + n;
    let mut mat = vec![vec![UnivariatePoly::zero(); size]; size];
    for row in 0..n {
        for col in 0..=m { mat[row][row + col] = p_coeffs[m - col].clone(); }
    }
    for row in 0..m {
        for col in 0..=n { mat[n + row][row + col] = q_coeffs[n - col].clone(); }
    }
    poly_matrix_det(mat)
}

/// Coefficients of x^n · q(z/x) viewed as a polynomial in x.
/// Returns `vec[j]` = coefficient of x^j = b_{n−j} · z^{n−j} as a UnivariatePoly in z.
fn q_product_x_coeffs(q: &UnivariatePoly, n: usize) -> Vec<UnivariatePoly> {
    let mut result = vec![UnivariatePoly::zero(); n + 1];
    for (j, slot) in result.iter_mut().enumerate() {
        let k = n - j; // index into q
        let bk = q.coeff(k);
        if !bk.is_zero() {
            let mut z_coeffs = vec![Rational::zero(); k + 1];
            z_coeffs[k] = bk;
            *slot = UnivariatePoly::from_coeffs(z_coeffs);
        }
    }
    result
}

/// Compute res_x(p(x), x^n · q(z/x)) — polynomial in z whose roots are
/// all pairwise products α_i · β_j where p(α_i)=0 and q(β_j)=0, n = deg(q).
/// Used for exact `AlgebraicNumber` multiplication.
pub fn product_resultant(p: &UnivariatePoly, q: &UnivariatePoly) -> UnivariatePoly {
    let m = p.degree();
    let n = q.degree();
    if m == 0 || n == 0 { return UnivariatePoly::zero(); }

    let p_coeffs: Vec<UnivariatePoly> = (0..=m)
        .map(|k| UnivariatePoly::constant(p.coeff(k)))
        .collect();
    let q_coeffs = q_product_x_coeffs(q, n);

    let size = m + n;
    let mut mat = vec![vec![UnivariatePoly::zero(); size]; size];
    for row in 0..n {
        for col in 0..=m { mat[row][row + col] = p_coeffs[m - col].clone(); }
    }
    for row in 0..m {
        for col in 0..=n { mat[n + row][row + col] = q_coeffs[n - col].clone(); }
    }
    poly_matrix_det(mat)
}

// ── Arithmetic operators ──────────────────────────────────────────────────────

impl Add for UnivariatePoly {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        let max_len = self.coeffs.len().max(rhs.coeffs.len());
        let mut coeffs = vec![Rational::zero(); max_len];
        for (i, c) in self.coeffs.into_iter().enumerate() { coeffs[i] = coeffs[i].clone() + c; }
        for (i, c) in rhs.coeffs.into_iter().enumerate() { coeffs[i] = coeffs[i].clone() + c; }
        let mut p = UnivariatePoly { coeffs };
        p.trim();
        p
    }
}

impl Sub for UnivariatePoly {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self { self + (-rhs) }
}

impl Neg for UnivariatePoly {
    type Output = Self;
    fn neg(self) -> Self {
        UnivariatePoly { coeffs: self.coeffs.into_iter().map(|c| -c).collect() }
    }
}

impl Mul for UnivariatePoly {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        if self.is_zero() || rhs.is_zero() { return Self::zero(); }
        let mut coeffs = vec![Rational::zero(); self.coeffs.len() + rhs.coeffs.len() - 1];
        for (i, a) in self.coeffs.iter().enumerate() {
            for (j, b) in rhs.coeffs.iter().enumerate() {
                coeffs[i + j] = coeffs[i + j].clone() + a.clone() * b.clone();
            }
        }
        let mut p = UnivariatePoly { coeffs };
        p.trim();
        p
    }
}

// Reference forms
impl<'b> Add<&'b UnivariatePoly> for &UnivariatePoly {
    type Output = UnivariatePoly;
    fn add(self, rhs: &'b UnivariatePoly) -> UnivariatePoly { self.clone() + rhs.clone() }
}
impl<'b> Sub<&'b UnivariatePoly> for &UnivariatePoly {
    type Output = UnivariatePoly;
    fn sub(self, rhs: &'b UnivariatePoly) -> UnivariatePoly { self.clone() - rhs.clone() }
}
impl<'b> Mul<&'b UnivariatePoly> for &UnivariatePoly {
    type Output = UnivariatePoly;
    fn mul(self, rhs: &'b UnivariatePoly) -> UnivariatePoly { self.clone() * rhs.clone() }
}
impl Neg for &UnivariatePoly {
    type Output = UnivariatePoly;
    fn neg(self) -> UnivariatePoly { -self.clone() }
}

// ── Display ───────────────────────────────────────────────────────────────────

impl fmt::Display for UnivariatePoly {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_zero() { return write!(f, "0"); }
        let mut first = true;
        for (i, c) in self.coeffs.iter().enumerate().rev() {
            if c.is_zero() { continue; }
            if !first { write!(f, " + ")?; }
            first = false;
            match i {
                0 => write!(f, "{}", c)?,
                1 => write!(f, "({})*x", c)?,
                _ => write!(f, "({})*x^{}", c, i)?,
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(n: i64) -> Rational { Rational::from(n) }

    fn poly(coeffs: &[i64]) -> UnivariatePoly {
        UnivariatePoly::from_coeffs(coeffs.iter().map(|&n| r(n)).collect())
    }

    #[test]
    fn division() {
        // x² - 1 = (x - 1)(x + 1)
        let p = poly(&[-1, 0, 1]);
        let d = poly(&[-1, 1]);
        let (q, r) = UnivariatePoly::divrem(&p, &d);
        assert_eq!(q, poly(&[1, 1])); // x + 1
        assert!(r.is_zero());
    }

    #[test]
    fn derivative() {
        // d/dx (x³ - 3x + 2) = 3x² - 3
        let p = poly(&[2, -3, 0, 1]);
        let dp = p.derivative();
        assert_eq!(dp, poly(&[-3, 0, 3]));
    }

    #[test]
    fn sturm_root_count() {
        // x² - 1 has 2 real roots
        let p = poly(&[-1, 0, 1]);
        let seq = p.sturm_sequence();
        let v_neg = UnivariatePoly::variations_at_neg_inf(&seq);
        let v_pos = UnivariatePoly::variations_at_pos_inf(&seq);
        assert_eq!(v_neg as i64 - v_pos as i64, 2);
    }

    #[test]
    fn root_isolation() {
        // Roots of x² - 5 are ±√5 ≈ ±2.236
        let p = poly(&[-5, 0, 1]);
        let intervals = p.real_root_isolate();
        assert_eq!(intervals.len(), 2);
        let roots: Vec<f64> = intervals.iter()
            .map(|(a, b)| p.refine_root_f64(a, b, 1e-10))
            .collect();
        for r in &roots {
            assert!((r * r - 5.0).abs() < 1e-9, "root={} sq={}", r, r * r);
        }
    }

    #[test]
    fn quadratic_roots() {
        // 25y² - 40y - 200 = 0 (from line-circle resultant)
        let p = poly(&[-200, -40, 25]);
        let roots = p.real_roots_f64(1e-12);
        assert_eq!(roots.len(), 2);
        for r in &roots {
            let val = 25.0 * r * r - 40.0 * r - 200.0;
            assert!(val.abs() < 1e-7, "residual={}", val);
        }
    }

    #[test]
    fn root_isolate_with_root_at_midpoint() {
        // x³ - 4x = x(x-2)(x+2): roots at -2, 0, 2. The Cauchy bound is symmetric
        // (±5), so the very first bisection midpoint is exactly 0 — itself a root —
        // which exercises the exact-midpoint split path (no fixed-epsilon nudge).
        let p = poly(&[0, -4, 0, 1]);
        let mut roots = p.real_roots_f64(1e-10);
        assert_eq!(roots.len(), 3, "expected roots -2, 0, 2, got {:?}", roots);
        roots.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((roots[0] + 2.0).abs() < 1e-6, "got {}", roots[0]);
        assert!((roots[1]).abs()       < 1e-6, "got {}", roots[1]);
        assert!((roots[2] - 2.0).abs() < 1e-6, "got {}", roots[2]);
    }

    #[test]
    fn yun_square_free_simple() {
        // x² - 1 = (x-1)(x+1) — already square-free, one factor
        let p = poly(&[-1, 0, 1]);
        let factors = p.yun_square_free_factorization();
        assert_eq!(factors.len(), 1);
        // The single factor should be x²-1 (monic)
        assert_eq!(factors[0], poly(&[-1, 0, 1]));
    }

    #[test]
    fn yun_square_free_double_root() {
        // (x-1)²(x+1) = x³ - x² - x + 1
        // factors: [f1=(x+1), f2=(x-1)]  (f1 appears once, f2 appears twice)
        let p = poly(&[1, -1, -1, 1]);
        let factors = p.yun_square_free_factorization();
        assert_eq!(factors.len(), 2, "Expected 2 factors, got {:?}", factors);
        // f1 * f2² should equal the primitive part of p
        let reconstructed = factors[0].clone()
            * factors[1].clone()
            * factors[1].clone();
        let prim = p.primitive_part();
        assert_eq!(reconstructed, prim);
    }

    #[test]
    fn content_and_primitive() {
        // 2x² + 4x + 6 — primitive part is x² + 2x + 3 (monic)
        let p = UnivariatePoly::from_coeffs(vec![r(6), r(4), r(2)]);
        let prim = p.primitive_part();
        assert_eq!(prim, poly(&[3, 2, 1]));
    }

    #[test]
    fn extended_gcd_identity() {
        // gcd(x²-1, x-1) = (x-1);  verify a·s + b·t = g
        let a = poly(&[-1, 0, 1]); // x²-1
        let b = poly(&[-1, 1]);    // x-1
        let (g, s, t) = UnivariatePoly::extended_gcd(&a, &b);
        // g should be x-1 (monic)
        assert_eq!(g, poly(&[-1, 1]));
        // Verify Bézout identity: a*s + b*t == g
        let check = a.clone() * s + b.clone() * t;
        assert_eq!(check, g);
    }

    #[test]
    fn sturm_cache_matches_direct() {
        let p = poly(&[-200, -40, 25]); // 25y² - 40y - 200
        let cache = p.sturm_cache();
        let seq = p.sturm_sequence();
        let x = Rational::from(0i64);
        assert_eq!(
            cache.variations_at(&x),
            UnivariatePoly::variations_at(&seq, &x)
        );
        assert_eq!(
            cache.count_roots_in(&Rational::from(-10i64), &Rational::from(10i64)),
            2
        );
    }
}
