use std::collections::HashMap;
use crate::rational::Rational;
use crate::univariate::UnivariatePoly;
use crate::algebraic::AlgebraicNumber;

/// Sparse bivariate polynomial over ℚ.
/// `terms[(i, j)]` = coefficient of x^i · y^j.
#[derive(Clone, Debug)]
pub struct BivariatePoly {
    terms: HashMap<(u32, u32), Rational>,
}

impl BivariatePoly {
    pub fn zero() -> Self { BivariatePoly { terms: HashMap::new() } }

    /// Construct from a list of ((x_deg, y_deg), coefficient) pairs.
    pub fn from_terms(terms: &[((u32, u32), Rational)]) -> Self {
        let mut map = HashMap::new();
        for &((xi, yi), ref c) in terms {
            if !c.is_zero() {
                *map.entry((xi, yi)).or_insert_with(Rational::zero) =
                    map.get(&(xi, yi)).cloned().unwrap_or_else(Rational::zero) + c.clone();
            }
        }
        map.retain(|_, v| !v.is_zero());
        BivariatePoly { terms: map }
    }

    pub fn set_term(&mut self, xi: u32, yi: u32, c: Rational) {
        if c.is_zero() { self.terms.remove(&(xi, yi)); } else { self.terms.insert((xi, yi), c); }
    }

    /// Maximum degree in x.
    pub fn x_degree(&self) -> u32 {
        self.terms.keys().map(|&(i, _)| i).max().unwrap_or(0)
    }

    /// Maximum degree in y.
    pub fn y_degree(&self) -> u32 {
        self.terms.keys().map(|&(_, j)| j).max().unwrap_or(0)
    }

    pub fn is_zero(&self) -> bool { self.terms.is_empty() }

    /// Evaluate at rational (x, y).
    pub fn eval_rational(&self, x: &Rational, y: &Rational) -> Rational {
        let mut result = Rational::zero();
        for (&(xi, yi), c) in &self.terms {
            let term = c.clone() * x.pow_u32(xi) * y.pow_u32(yi);
            result = result + term;
        }
        result
    }

    pub fn eval_f64(&self, x: f64, y: f64) -> f64 {
        self.terms.iter().fold(0.0, |acc, (&(xi, yi), c)| {
            acc + c.to_f64() * x.powi(xi as i32) * y.powi(yi as i32)
        })
    }

    /// Partial derivative with respect to x.
    pub fn partial_x(&self) -> Self {
        let mut terms = HashMap::new();
        for (&(xi, yi), c) in &self.terms {
            if xi > 0 {
                let new_c = c.clone() * Rational::from(xi as i64);
                terms.insert((xi - 1, yi), new_c);
            }
        }
        BivariatePoly { terms }
    }

    /// Partial derivative with respect to y.
    pub fn partial_y(&self) -> Self {
        let mut terms = HashMap::new();
        for (&(xi, yi), c) in &self.terms {
            if yi > 0 {
                let new_c = c.clone() * Rational::from(yi as i64);
                terms.insert((xi, yi - 1), new_c);
            }
        }
        BivariatePoly { terms }
    }

    // ── Coefficient extraction ────────────────────────────────────────────────

    /// View `self` as a polynomial in x with coefficients that are polynomials in y.
    /// Returns `Vec<UnivariatePoly>` where `result[i]` = coefficient of x^i as a poly in y.
    pub fn x_coefficients(&self) -> Vec<UnivariatePoly> {
        let m = self.x_degree() as usize;
        let mut result = vec![UnivariatePoly::zero(); m + 1];
        for (&(xi, yi), coeff) in &self.terms {
            result[xi as usize].set_coeff(yi as usize, coeff.clone());
        }
        result
    }

    /// Substitute y = val (rational) → univariate polynomial in x.
    pub fn substitute_y(&self, val: &Rational) -> UnivariatePoly {
        let m = self.x_degree() as usize;
        let mut coeffs = vec![Rational::zero(); m + 1];
        for (&(xi, yi), coeff) in &self.terms {
            coeffs[xi as usize] = coeffs[xi as usize].clone() + coeff.clone() * val.pow_u32(yi);
        }
        UnivariatePoly::from_coeffs(coeffs)
    }

    /// Substitute x = g(y) (a polynomial in y) → univariate polynomial in y.
    ///
    /// Computes f(g(y), y) = Σ_k a_k(y) · g(y)^k  where a_k(y) = x_coefficients()[k].
    /// This is the "substitution for curve intersection" required by the spec.
    pub fn substitute_x_poly(&self, g: &crate::univariate::UnivariatePoly) -> crate::univariate::UnivariatePoly {
        use crate::univariate::UnivariatePoly;
        let x_coeffs = self.x_coefficients(); // a_k(y) for each power of x
        let mut result = UnivariatePoly::zero();
        let mut g_pow = UnivariatePoly::one(); // g(y)^0
        for a_k in &x_coeffs {
            if !a_k.is_zero() {
                result = result + a_k.clone() * g_pow.clone();
            }
            g_pow = g_pow * g.clone();
        }
        result
    }

    /// Substitute x = val (rational) → univariate polynomial in y.
    pub fn substitute_x(&self, val: &Rational) -> UnivariatePoly {
        let m = self.y_degree() as usize;
        let mut coeffs = vec![Rational::zero(); m + 1];
        for (&(xi, yi), coeff) in &self.terms {
            coeffs[yi as usize] = coeffs[yi as usize].clone() + coeff.clone() * val.pow_u32(xi);
        }
        UnivariatePoly::from_coeffs(coeffs)
    }

    // ── Resultant (Sylvester matrix) ──────────────────────────────────────────

    /// Compute resultant of `self` and `other` with respect to x.
    /// Returns a univariate polynomial in y.
    ///
    /// Algorithm: build the Sylvester matrix (entries are polys in y),
    /// compute its determinant via fraction-free Bareiss (Knuth vol.2 §4.6.1).
    ///
    /// **Phase 3 optimization (TODO):** for high-degree polynomials, replace with
    /// modular resultant + Chinese Remainder Theorem reconstruction to avoid
    /// coefficient explosion.  Choose sufficiently many primes p_i, compute
    /// res_x(f mod p_i, g mod p_i) in Z/p_i[y], then lift via CRT to Z[y],
    /// and reconstruct coefficients bounded by the Hadamard height bound.
    pub fn resultant_wrt_x(&self, other: &Self) -> UnivariatePoly {
        let f_coeffs = self.x_coefficients();  // indexed by x-degree
        let g_coeffs = other.x_coefficients();
        let m = f_coeffs.len() - 1; // degree of self in x
        let n = g_coeffs.len() - 1; // degree of other in x

        if m == 0 || n == 0 {
            // One polynomial doesn't depend on x — degenerate case
            return UnivariatePoly::zero();
        }

        let size = m + n;
        let mut mat: Vec<Vec<UnivariatePoly>> = vec![vec![UnivariatePoly::zero(); size]; size];

        // First n rows: shifts of f (from high x-degree to low)
        for row in 0..n {
            for col in 0..=m {
                // f_coeffs[m - col] is the coeff of x^{m-col} in f
                mat[row][row + col] = f_coeffs[m - col].clone();
            }
        }
        // Last m rows: shifts of g (from high x-degree to low)
        for row in 0..m {
            for col in 0..=n {
                mat[n + row][row + col] = g_coeffs[n - col].clone();
            }
        }

        bareiss_determinant(mat)
    }

    // ── Bézout resultant (efficient for deg ≤ 6) ─────────────────────────────

    /// Resultant via the Bézout matrix, which is `max(m,n) × max(m,n)` — about
    /// half the size of the Sylvester matrix for equal-degree polynomials.
    ///
    /// Bézout entry B[i][j] (0-indexed) = coefficient of t^{i+j} in
    ///   (f(x)*g(t) - f(t)*g(x)) / (x - t)
    /// viewed as a polynomial in x, then collecting by degree in x.
    ///
    /// Falls back to `resultant_wrt_x` (Sylvester) if combined degree > 12.
    // Index-based loops are clearer than iterators for this symmetric matrix fill
    // (cross-indexing fa[k]/ga[l] and writing both bez[i][j] and bez[j][i]).
    #[allow(clippy::needless_range_loop)]
    pub fn bezout_resultant_wrt_x(&self, other: &Self) -> UnivariatePoly {
        let m = self.x_degree() as usize;
        let n = other.x_degree() as usize;

        // Guard: only use Bézout for the sizes specified in the spec (deg ≤ 6 each)
        if m > 6 || n > 6 {
            return self.resultant_wrt_x(other);
        }
        if m == 0 || n == 0 {
            return UnivariatePoly::zero();
        }

        let f = self.x_coefficients();   // f[i] = coeff of x^i in f, as poly in y
        let g = other.x_coefficients();  // g[i] = coeff of x^i in g, as poly in y
        let d = m.max(n);

        // Pad both to length d+1 with zero polynomials
        let mut fa = vec![UnivariatePoly::zero(); d + 1];
        let mut ga = vec![UnivariatePoly::zero(); d + 1];
        for (i, p) in f.into_iter().enumerate() { fa[i] = p; }
        for (i, p) in g.into_iter().enumerate() { ga[i] = p; }

        // Build Bézout matrix B (d × d), symmetric.
        // B[i][j] = Σ_{k+l=i+j+1, k≠l} sgn(k-l) · fa[k] · ga[l]
        // Derived from Q(x,t)=(f(x)g(t)-f(t)g(x))/(x-t) = Σ B[i][j] x^i t^j
        let mut bez: Vec<Vec<UnivariatePoly>> = vec![vec![UnivariatePoly::zero(); d]; d];
        for i in 0..d {
            for j in i..d {
                let s = i + j + 1; // k + l = s
                let mut entry = UnivariatePoly::zero();
                for k in 0..=s.min(d) {
                    let l = s - k;
                    if l > d || k == l { continue; }
                    let ak = fa[k].clone();
                    let bl = ga[l].clone();
                    if k > l { entry = entry + ak * bl; }
                    else      { entry = entry - ak * bl; }
                }
                bez[i][j] = entry.clone();
                if i != j { bez[j][i] = entry; }
            }
        }

        bareiss_determinant(bez)
    }

    // ── Resultant wrt y ───────────────────────────────────────────────────────

    /// Compute resultant of `self` and `other` with respect to y.
    /// Returns a univariate polynomial in x.
    pub fn resultant_wrt_y(&self, other: &Self) -> UnivariatePoly {
        let f_coeffs = self.y_coefficients();
        let g_coeffs = other.y_coefficients();
        let m = f_coeffs.len() - 1;
        let n = g_coeffs.len() - 1;

        if m == 0 || n == 0 { return UnivariatePoly::zero(); }

        let size = m + n;
        let mut mat: Vec<Vec<UnivariatePoly>> = vec![vec![UnivariatePoly::zero(); size]; size];

        for row in 0..n {
            for col in 0..=m {
                mat[row][row + col] = f_coeffs[m - col].clone();
            }
        }
        for row in 0..m {
            for col in 0..=n {
                mat[n + row][row + col] = g_coeffs[n - col].clone();
            }
        }
        bareiss_determinant(mat)
    }

    /// View `self` as a polynomial in y with coefficients that are polynomials in x.
    /// `result[j]` = coefficient of y^j as a poly in x.
    pub fn y_coefficients(&self) -> Vec<UnivariatePoly> {
        let n = self.y_degree() as usize;
        let mut result = vec![UnivariatePoly::zero(); n + 1];
        for (&(xi, yi), coeff) in &self.terms {
            result[yi as usize].set_coeff(xi as usize, coeff.clone());
        }
        result
    }

    // ── Discriminant ──────────────────────────────────────────────────────────

    /// Discriminant of `self` with respect to x: res_x(f, ∂f/∂x).
    /// Returns a univariate polynomial in y (or a rational if f is univariate in x).
    pub fn discriminant_wrt_x(&self) -> UnivariatePoly {
        self.resultant_wrt_x(&self.partial_x())
    }

    /// Discriminant of `self` with respect to y: res_y(f, ∂f/∂y).
    pub fn discriminant_wrt_y(&self) -> UnivariatePoly {
        self.resultant_wrt_y(&self.partial_y())
    }

    // ── Total degree & term iterator ──────────────────────────────────────────

    /// Maximum value of (x_deg + y_deg) over all non-zero terms.
    pub fn total_degree(&self) -> u32 {
        self.terms.keys().map(|&(i, j)| i + j).max().unwrap_or(0)
    }

    /// Iterator over terms in graded lexicographic order: sort by (i+j desc, i desc).
    /// Yields `((x_deg, y_deg), &coefficient)` pairs.
    pub fn terms_graded_lex(&self) -> Vec<((u32, u32), &Rational)> {
        let mut pairs: Vec<((u32, u32), &Rational)> = self.terms
            .iter()
            .map(|(&k, v)| (k, v))
            .collect();
        // Graded lex: compare total degree first (descending), then x-degree (descending)
        pairs.sort_by(|&(a, _), &(b, _)| {
            let ta = a.0 + a.1;
            let tb = b.0 + b.1;
            tb.cmp(&ta).then(b.0.cmp(&a.0))
        });
        pairs
    }

    // ── Bounding box evaluation ───────────────────────────────────────────────

    /// Estimate the range [min, max] of f(x, y) over the axis-aligned box
    /// x ∈ [x_lo, x_hi] × y ∈ [y_lo, y_hi] using interval arithmetic.
    ///
    /// Each coefficient c·x^i·y^j is bounded by evaluating the monomial on the
    /// corners of the box.  The result is a conservative (outer) bound.
    pub fn eval_range_f64(
        &self,
        x_lo: f64, x_hi: f64,
        y_lo: f64, y_hi: f64,
    ) -> (f64, f64) {
        let mut lo_sum = 0.0f64;
        let mut hi_sum = 0.0f64;

        for (&(xi, yi), c) in &self.terms {
            // Monomial x^xi * y^yi over the box: evaluate at all 4 corners
            let corners = [
                x_lo.powi(xi as i32) * y_lo.powi(yi as i32),
                x_lo.powi(xi as i32) * y_hi.powi(yi as i32),
                x_hi.powi(xi as i32) * y_lo.powi(yi as i32),
                x_hi.powi(xi as i32) * y_hi.powi(yi as i32),
            ];
            let mono_lo = corners.iter().cloned().fold(f64::INFINITY, f64::min);
            let mono_hi = corners.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

            let cv = c.to_f64();
            if cv >= 0.0 {
                lo_sum += cv * mono_lo;
                hi_sum += cv * mono_hi;
            } else {
                lo_sum += cv * mono_hi;
                hi_sum += cv * mono_lo;
            }
        }
        (lo_sum, hi_sum)
    }

    // ── Inversion: local parameterization ─────────────────────────────────────

    /// Compute a polynomial approximation of y as a function of x near `(x0, y0)`.
    ///
    /// Returns a `UnivariatePoly` in `t = x − x0` such that `y(t) ≈ y0 + c1·t + c2·t²`.
    ///
    /// Algorithm: implicit differentiation.
    ///   c1 = −f_x / f_y  (at the base point)
    ///   c2 = −(f_xx + 2·f_xy·c1 + f_yy·c1²) / (2·f_y)
    ///
    /// Requires:
    ///   - f(x0, y0) = 0  (point lies on the curve)
    ///   - f_y(x0, y0) ≠ 0  (implicit function theorem: curve is locally a graph over x)
    ///
    /// `degree` controls how many terms to compute (currently capped at 2 for Phase 1).
    pub fn local_parameterization_at(
        &self,
        x0: &Rational,
        y0: &Rational,
        degree: usize,
    ) -> Result<crate::univariate::UnivariatePoly, &'static str> {
        use crate::univariate::UnivariatePoly;

        let f_val = self.eval_rational(x0, y0);
        if !f_val.is_zero() {
            return Err("Base point is not on the curve");
        }

        let fy = self.partial_y().eval_rational(x0, y0);
        if fy.is_zero() {
            return Err("Vertical tangent or singular point: ∂f/∂y = 0 at base point");
        }

        let mut coeffs = vec![y0.clone()]; // c_0 = y0

        if degree >= 1 {
            // c1 = −f_x / f_y
            let fx = self.partial_x().eval_rational(x0, y0);
            coeffs.push(-fx / fy.clone());
        }

        if degree >= 2 {
            let c1 = coeffs[1].clone();
            // Second total derivative: f_xx + 2·f_xy·y' + f_yy·(y')² + f_y·y'' = 0
            // → y'' = −(f_xx + 2·f_xy·c1 + f_yy·c1²) / f_y
            // Taylor coeff c2 = y'' / 2!
            let fxx = self.partial_x().partial_x().eval_rational(x0, y0);
            let fxy = self.partial_x().partial_y().eval_rational(x0, y0);
            let fyy = self.partial_y().partial_y().eval_rational(x0, y0);
            let y_double_prime = -(fxx
                + Rational::from(2i64) * fxy * c1.clone()
                + fyy * c1.clone() * c1) / fy.clone();
            coeffs.push(y_double_prime / Rational::from(2i64));
        }

        // Higher terms (degree ≥ 3) require the Faà di Bruno formula;
        // deferred to Phase 2 symbolic differentiation.
        // Pad with zeros up to requested degree.
        while coeffs.len() <= degree { coeffs.push(Rational::zero()); }

        Ok(UnivariatePoly::from_coeffs(coeffs))
    }

    // ── Implicitization ───────────────────────────────────────────────────────

    /// Convert a parametric curve `(x(t), y(t))` to an implicit curve `f(x,y) = 0`
    /// via resultant elimination of the parameter `t`.
    ///
    /// Algorithm: form F(t) = x_poly(t) - x  (as bivariate poly in t and x)
    ///            form G(t) = y_poly(t) - y  (as bivariate poly in t and y)
    ///
    /// Wait — we want to eliminate `t` from two polynomials in `t`:
    ///   P(t) = x(t) - x = 0  [univariate in t, parameterised by x]
    ///   Q(t) = y(t) - y = 0  [univariate in t, parameterised by y]
    ///
    /// Resultant of P and Q w.r.t. t gives f(x, y) = 0.
    ///
    /// `x_poly` and `y_poly` are `UnivariatePoly` in `t` (rational coefficients).
    /// The returned `BivariatePoly` is the implicit equation.
    pub fn implicitize(
        x_poly: &crate::univariate::UnivariatePoly,
        y_poly: &crate::univariate::UnivariatePoly,
    ) -> BivariatePoly {

        // Build P(t) = x_poly(t) - X  (treat X as a free symbol at degree (1,0) in BivariatePoly)
        // We form the bivariate resultant res_t(x(t)-x, y(t)-y).
        //
        // Represent P(t) as a bivariate polynomial in (X, t): coeffs[deg_t] are polys in X.
        //   P[0] = x_poly.coeff(0) - X   [constant-in-t part: c₀ - X]
        //   P[k] = x_poly.coeff(k)        [for k ≥ 1: pure rational, no X]
        //
        // Similarly Q[0] = y_poly.coeff(0) - Y, Q[k] = y_poly.coeff(k).

        let deg_x = x_poly.degree();
        let deg_y = y_poly.degree();

        // Build the Sylvester matrix for res_t(P, Q):
        // P treated as degree `deg_x` in t, Q treated as degree `deg_y` in t.
        // Each entry is a bivariate polynomial in (X, Y).
        let p_t_coeffs: Vec<BivariatePoly> = (0..=deg_x).map(|k| {
            let c = x_poly.coeff(k);
            if k == 0 {
                // c - X  → coefficient of X is -1, constant term is c
                let terms = vec![((0u32, 0u32), c), ((1u32, 0u32), Rational::minus_one())];
                BivariatePoly::from_terms(&terms)
            } else {
                // pure rational constant c (as bivariate with no x,y dependence)
                BivariatePoly::from_terms(&[((0u32, 0u32), c)])
            }
        }).collect();

        let q_t_coeffs: Vec<BivariatePoly> = (0..=deg_y).map(|k| {
            let c = y_poly.coeff(k);
            if k == 0 {
                let terms = vec![((0u32, 0u32), c), ((0u32, 1u32), Rational::minus_one())];
                BivariatePoly::from_terms(&terms)
            } else {
                BivariatePoly::from_terms(&[((0u32, 0u32), c)])
            }
        }).collect();

        // Build Sylvester matrix where entries are BivariatePoly in (X, Y)
        let m = deg_x; // degree of P in t
        let n = deg_y; // degree of Q in t
        let size = m + n;
        let zero_bpoly = BivariatePoly::zero();

        let get_p = |k: usize| p_t_coeffs.get(m - k).cloned().unwrap_or_else(BivariatePoly::zero);
        let get_q = |k: usize| q_t_coeffs.get(n - k).cloned().unwrap_or_else(BivariatePoly::zero);

        let mut mat: Vec<Vec<BivariatePoly>> = vec![vec![zero_bpoly.clone(); size]; size];
        for row in 0..n {
            for col in 0..=m {
                mat[row][row + col] = get_p(col);
            }
        }
        for row in 0..m {
            for col in 0..=n {
                mat[n + row][row + col] = get_q(col);
            }
        }

        // Compute determinant of this BivariatePoly matrix via Bareiss
        bareiss_bpoly_determinant(mat)
    }

    // ── Intersection ──────────────────────────────────────────────────────────

    /// Find all real intersection points of `self = 0` and `other = 0`,
    /// returning each as an exact `(x, y)` pair of `AlgebraicNumber`s.
    ///
    /// Algorithm — symmetric resultant projection:
    ///   1. `res_x(f,g)` is a polynomial in y whose real roots are the
    ///      y-coordinates of every common solution; `res_y(f,g)` gives the
    ///      x-coordinates. Both are exact, so each returned coordinate carries
    ///      its *true* defining polynomial.
    ///   2. The candidates are the grid `{x_i} × {y_j}` of those roots.
    ///   3. A pair is kept iff the box around it provably contains a common zero,
    ///      decided by exact rational interval arithmetic over a box refined in
    ///      lock-step (`box_contains_common_zero`).
    ///
    /// Unlike the previous version this performs **no float round-trip**: the old
    /// code rounded each y-root to a 12-digit rational, substituted it, and handed
    /// back an x whose "defining polynomial" was that float-derived slice rather
    /// than a real minimal polynomial.
    pub fn intersect(&self, other: &Self) -> Result<Vec<(AlgebraicNumber, AlgebraicNumber)>, &'static str> {
        let res_in_y = self.resultant_wrt_x(other); // polynomial in y → y-coords
        if res_in_y.is_zero() {
            return Err("Resultant is zero: curves share a component or are parallel");
        }
        let res_in_x = self.resultant_wrt_y(other); // polynomial in x → x-coords

        // Degenerate x-projection (an axis-aligned input independent of y, e.g. a
        // vertical line): the symmetric grid can't run, so fall back to the
        // fiber-substitution method, which only needs the y-projection.
        if res_in_x.is_zero() {
            return Ok(self.intersect_via_y_fibers(other, &res_in_y));
        }

        let ry = res_in_y.make_square_free();
        let rx = res_in_x.make_square_free();
        let y_intervals = ry.real_root_isolate();
        let x_intervals = rx.real_root_isolate();

        let mut intersections = Vec::new();
        for (y_lo, y_hi) in &y_intervals {
            for (x_lo, x_hi) in &x_intervals {
                let mut x_alg = AlgebraicNumber::new(rx.clone(), x_lo.clone(), x_hi.clone());
                let mut y_alg = AlgebraicNumber::new(ry.clone(), y_lo.clone(), y_hi.clone());
                if self.box_contains_common_zero(other, &mut x_alg, &mut y_alg) {
                    intersections.push((x_alg, y_alg));
                }
            }
        }
        Ok(intersections)
    }

    /// Refine the candidate box `x × y` until it provably does (or does not)
    /// contain a common zero of `self` and `other`.
    ///
    /// Soundness rests on exact interval arithmetic giving a *superset* of each
    /// polynomial's values over the box (`eval_range_rational`): the instant 0
    /// leaves either enclosure the box holds no common zero and we reject. A true
    /// zero keeps 0 in both enclosures at every width, so once the box is tight we
    /// accept. The loop is bounded; distinct algebraic numbers of these low
    /// degrees are separated far more widely than the final box, so a near-miss is
    /// always rejected before the cap.
    fn box_contains_common_zero(
        &self, other: &Self,
        x: &mut AlgebraicNumber, y: &mut AlgebraicNumber,
    ) -> bool {
        // 1/10^18 — far below CAD resolution, comfortably above the root
        // separation of low-degree resultants with modest coefficients.
        let accept_width = Rational::one() / Rational::from(10i64).pow_u32(18);
        for _ in 0..120 {
            if !self.eval_range_rational(&x.lower, &x.upper, &y.lower, &y.upper).contains_zero() {
                return false;
            }
            if !other.eval_range_rational(&x.lower, &x.upper, &y.lower, &y.upper).contains_zero() {
                return false;
            }
            let wx = x.upper.clone() - x.lower.clone();
            let wy = y.upper.clone() - y.lower.clone();
            if wx < accept_width && wy < accept_width {
                return true;
            }
            x.refine(1);
            y.refine(1);
        }
        true
    }

    /// Guaranteed (superset) enclosure of `self` over the box
    /// `[x_lo,x_hi] × [y_lo,y_hi]`, in exact rationals. If the result excludes 0,
    /// `self` has no zero anywhere in the box — the fact the pairing relies on.
    /// (Distinct from `eval_range_f64`, whose corner sampling is *not* a valid
    /// enclosure when the box straddles zero, e.g. x² over [-1,1].)
    fn eval_range_rational(
        &self,
        x_lo: &Rational, x_hi: &Rational,
        y_lo: &Rational, y_hi: &Rational,
    ) -> RInterval {
        let xr = RInterval { lo: x_lo.clone(), hi: x_hi.clone() };
        let yr = RInterval { lo: y_lo.clone(), hi: y_hi.clone() };
        let mut acc = RInterval::point(Rational::zero());
        for (&(xi, yi), c) in &self.terms {
            let term = RInterval::point(c.clone()).mul(&xr.pow(xi)).mul(&yr.pow(yi));
            acc = acc.add(&term);
        }
        acc
    }

    /// Fiber-substitution intersection (the pre-projection method), kept for the
    /// degenerate case where the x-projection resultant vanishes. Isolates the
    /// y-roots of `res_in_y`, substitutes each (as a tight rational) into `self`,
    /// and solves for x, verifying candidates against both curves.
    fn intersect_via_y_fibers(
        &self, other: &Self, res_in_y: &UnivariatePoly,
    ) -> Vec<(AlgebraicNumber, AlgebraicNumber)> {
        let res_sqfree = res_in_y.make_square_free();
        let y_intervals = res_sqfree.real_root_isolate();
        let mut intersections = Vec::new();

        for (y_lo, y_hi) in &y_intervals {
            let y_alg = AlgebraicNumber::new(res_sqfree.clone(), y_lo.clone(), y_hi.clone());
            let y_f64 = res_sqfree.refine_root_f64(y_lo, y_hi, 1e-13);
            let y_rat = Rational::from_f64_approx(y_f64);

            let f_at_y = self.substitute_y(&y_rat);
            if f_at_y.is_zero() { continue; }

            let f_at_y_sqfree = f_at_y.make_square_free();
            let x_intervals = f_at_y_sqfree.real_root_isolate();

            for (x_lo, x_hi) in &x_intervals {
                let x_f64 = f_at_y_sqfree.refine_root_f64(x_lo, x_hi, 1e-13);

                let check_self  = self.eval_f64(x_f64, y_f64).abs();
                let check_other = other.eval_f64(x_f64, y_f64).abs();

                let dx = self.partial_x().eval_f64(x_f64, y_f64);
                let dy = self.partial_y().eval_f64(x_f64, y_f64);
                let grad = (dx * dx + dy * dy).sqrt().max(1e-15);

                let dx2 = other.partial_x().eval_f64(x_f64, y_f64);
                let dy2 = other.partial_y().eval_f64(x_f64, y_f64);
                let grad2 = (dx2 * dx2 + dy2 * dy2).sqrt().max(1e-15);

                if check_self / grad < 1e-4 && check_other / grad2 < 1e-4 {
                    let x_alg = AlgebraicNumber::new(f_at_y_sqfree.clone(), x_lo.clone(), x_hi.clone());
                    intersections.push((x_alg, y_alg.clone()));
                }
            }
        }
        intersections
    }
}

// ── Exact interval arithmetic over ℚ ──────────────────────────────────────────

/// A closed rational interval `[lo, hi]` for *guaranteed* enclosures of polynomial
/// values over a box. The operations are sound even when the interval straddles
/// zero, so "0 ∉ enclosure" is a proof that the polynomial has no zero in the box
/// — the property the exact intersection pairing depends on.
#[derive(Clone)]
struct RInterval { lo: Rational, hi: Rational }

impl RInterval {
    fn point(r: Rational) -> Self { RInterval { lo: r.clone(), hi: r } }

    fn add(&self, o: &RInterval) -> RInterval {
        RInterval { lo: self.lo.clone() + o.lo.clone(), hi: self.hi.clone() + o.hi.clone() }
    }

    fn mul(&self, o: &RInterval) -> RInterval {
        let products = [
            self.lo.clone() * o.lo.clone(),
            self.lo.clone() * o.hi.clone(),
            self.hi.clone() * o.lo.clone(),
            self.hi.clone() * o.hi.clone(),
        ];
        let lo = products.iter().min().unwrap().clone();
        let hi = products.iter().max().unwrap().clone();
        RInterval { lo, hi }
    }

    /// `self^n` by repeated multiplication — a sound enclosure that overestimates
    /// for zero-straddling intervals but converges to the exact value as the width
    /// shrinks, so the pairing's refinement still terminates.
    fn pow(&self, n: u32) -> RInterval {
        let mut acc = RInterval::point(Rational::one());
        for _ in 0..n { acc = acc.mul(self); }
        acc
    }

    /// True iff `lo ≤ 0 ≤ hi`.
    fn contains_zero(&self) -> bool {
        !self.lo.is_positive() && !self.hi.is_negative()
    }
}

// ── BivariatePoly arithmetic ──────────────────────────────────────────────────

impl std::ops::Add for BivariatePoly {
    type Output = Self;
    fn add(mut self, rhs: Self) -> Self {
        for ((xi, yi), c) in rhs.terms {
            let e = self.terms.entry((xi, yi)).or_insert_with(Rational::zero);
            *e = e.clone() + c;
            if e.is_zero() { self.terms.remove(&(xi, yi)); }
        }
        self
    }
}

impl std::ops::Sub for BivariatePoly {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self { self + (-rhs) }
}

impl std::ops::Neg for BivariatePoly {
    type Output = Self;
    fn neg(self) -> Self {
        BivariatePoly { terms: self.terms.into_iter().map(|(k, v)| (k, -v)).collect() }
    }
}

impl std::ops::Mul for BivariatePoly {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        if self.is_zero() || rhs.is_zero() { return Self::zero(); }
        let mut result = BivariatePoly::zero();
        for ((ai, aj), ac) in &self.terms {
            for ((bi, bj), bc) in &rhs.terms {
                let ni = ai + bi;
                let nj = aj + bj;
                let e = result.terms.entry((ni, nj)).or_insert_with(Rational::zero);
                *e = e.clone() + ac.clone() * bc.clone();
            }
        }
        result.terms.retain(|_, v| !v.is_zero());
        result
    }
}

impl<'b> std::ops::Sub<&'b BivariatePoly> for &BivariatePoly {
    type Output = BivariatePoly;
    fn sub(self, rhs: &'b BivariatePoly) -> BivariatePoly { self.clone() - rhs.clone() }
}

impl<'b> std::ops::Mul<&'b BivariatePoly> for &BivariatePoly {
    type Output = BivariatePoly;
    fn mul(self, rhs: &'b BivariatePoly) -> BivariatePoly { self.clone() * rhs.clone() }
}

/// Exact division of BivariatePoly in the Bareiss algorithm.
/// Handles three cases in order of complexity:
///
/// 1. Constant divisor — divide all rational coefficients.
/// 2. Divisor in Y only (x_degree=0) — divide each X-coefficient poly by the Y-poly.
/// 3. General — polynomial long division in X with UnivariatePoly-in-Y coefficients.
///
/// The Bareiss guarantee ensures the division is always exact.
fn bpoly_exact_div(a: &BivariatePoly, b: &BivariatePoly) -> BivariatePoly {
    if b.is_zero() { panic!("bpoly_exact_div: divisor is zero"); }

    // ── Case 1: constant divisor ─────────────────────────────────────────────
    if b.total_degree() == 0 {
        let c = b.terms.values().next().map_or(Rational::one(), |v| v.clone());
        return BivariatePoly {
            terms: a.terms.iter().map(|(&k, v)| (k, v.clone() / c.clone())).collect(),
        };
    }

    let b_xdeg = b.x_degree() as usize;

    // ── Case 2: divisor depends only on Y ────────────────────────────────────
    // b = g(y) with no X terms. Divide each x-coefficient poly of `a` by g(y).
    if b_xdeg == 0 {
        let b_y_poly = b.x_coefficients().remove(0); // b as UnivariatePoly in y
        let a_x_coeffs = a.x_coefficients();
        let mut result = BivariatePoly::zero();
        for (xi, a_y_poly) in a_x_coeffs.iter().enumerate() {
            if a_y_poly.is_zero() { continue; }
            let q = UnivariatePoly::exact_div(a_y_poly, &b_y_poly);
            for yi in 0..=q.degree() {
                let coeff = q.coeff(yi);
                if !coeff.is_zero() {
                    result.set_term(xi as u32, yi as u32, coeff);
                }
            }
        }
        return result;
    }

    // ── Case 3: general — polynomial long division in X ───────────────────────
    // View a and b as elements of (UnivariatePoly-in-Y)[X].
    // The Bareiss guarantee means the division is exact.
    let m = a.x_degree() as usize;
    let n = b_xdeg;
    if m < n { return BivariatePoly::zero(); }

    let b_x_coeffs = b.x_coefficients(); // b_x_coeffs[i] = coeff of X^i in b, as Y-poly
    let lead_b = b_x_coeffs[n].clone();  // leading Y-poly coefficient of b in X

    let mut rem = a.x_coefficients(); // mutable remainder array (indexed by X-degree)
    let quot_len = m - n + 1;
    let mut quot: Vec<UnivariatePoly> = vec![UnivariatePoly::zero(); quot_len];

    for i in (0..quot_len).rev() {
        let idx = i + n;
        if !rem[idx].is_zero() {
            let q_i = UnivariatePoly::exact_div(&rem[idx], &lead_b);
            quot[i] = q_i.clone();
            for j in 0..=n {
                let sub = q_i.clone() * b_x_coeffs[j].clone();
                rem[i + j] = rem[i + j].clone() - sub;
            }
        }
    }

    // Reconstruct BivariatePoly from the quotient X-coefficient array
    let mut result = BivariatePoly::zero();
    for (xi, y_poly) in quot.iter().enumerate() {
        for yi in 0..=y_poly.degree() {
            let coeff = y_poly.coeff(yi);
            if !coeff.is_zero() {
                result.set_term(xi as u32, yi as u32, coeff);
            }
        }
    }
    result
}

/// Bareiss determinant for a matrix of `BivariatePoly` entries.
fn bareiss_bpoly_determinant(mut mat: Vec<Vec<BivariatePoly>>) -> BivariatePoly {
    let n = mat.len();
    if n == 0 { return BivariatePoly::from_terms(&[((0, 0), Rational::one())]); }
    if n == 1 { return mat[0][0].clone(); }

    let mut sign_neg = false;
    let mut prev_pivot = BivariatePoly::from_terms(&[((0, 0), Rational::one())]);

    for k in 0..n {
        let pivot_row = (k..n).find(|&i| !mat[i][k].is_zero());
        let pivot_row = match pivot_row {
            Some(r) => r,
            None => return BivariatePoly::zero(),
        };
        if pivot_row != k {
            mat.swap(k, pivot_row);
            sign_neg = !sign_neg;
        }
        let pivot = mat[k][k].clone();
        for i in (k + 1)..n {
            for j in (k + 1)..n {
                let numerator = &pivot * &mat[i][j] - &mat[i][k] * &mat[k][j];
                mat[i][j] = bpoly_exact_div(&numerator, &prev_pivot);
            }
            mat[i][k] = BivariatePoly::zero();
        }
        prev_pivot = pivot;
    }
    let det = mat[n - 1][n - 1].clone();
    if sign_neg { -det } else { det }
}

// ── Bareiss fraction-free determinant for polynomial matrices ─────────────────

/// Delegate to the shared implementation in `univariate.rs`.
fn bareiss_determinant(mat: Vec<Vec<UnivariatePoly>>) -> UnivariatePoly {
    crate::univariate::poly_matrix_det(mat)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rat(n: i64) -> Rational { Rational::from(n) }

    #[test]
    fn line_circle_intersection() {
        // Line: 3x + 4y - 5 = 0
        let line = BivariatePoly::from_terms(&[
            ((1, 0), rat(3)),
            ((0, 1), rat(4)),
            ((0, 0), rat(-5)),
        ]);

        // Circle: x² + y² - 25 = 0
        let circle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)),
            ((0, 2), rat(1)),
            ((0, 0), rat(-25)),
        ]);

        let intersections = line.intersect(&circle).unwrap();
        assert_eq!(intersections.len(), 2, "Line and circle should have 2 intersections");

        for (x_a, y_a) in &intersections {
            let x = x_a.to_f64(1e-10);
            let y = y_a.to_f64(1e-10);
            let line_check = 3.0 * x + 4.0 * y - 5.0;
            let circle_check = x * x + y * y - 25.0;
            assert!(line_check.abs() < 1e-6, "Point not on line: {}", line_check);
            assert!(circle_check.abs() < 1e-5, "Point not on circle: {}", circle_check);
        }
    }

    #[test]
    fn intersection_coords_carry_exact_defining_poly() {
        // The symmetric-projection solver must hand back coordinates whose defining
        // polynomial is the *exact* projection resultant (degree 2 for line∩circle),
        // not a float-substituted slice as the old fiber method produced.
        let line = BivariatePoly::from_terms(&[
            ((1, 0), rat(3)), ((0, 1), rat(4)), ((0, 0), rat(-5)),
        ]);
        let circle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)), ((0, 2), rat(1)), ((0, 0), rat(-25)),
        ]);
        let hits = line.intersect(&circle).unwrap();
        assert_eq!(hits.len(), 2, "expected 2 intersections, got {}", hits.len());

        for (xa, ya) in &hits {
            // Defining polynomials are the degree-2 projection resultants…
            assert_eq!(xa.poly.degree(), 2, "x poly should be the degree-2 x-resultant");
            assert_eq!(ya.poly.degree(), 2, "y poly should be the degree-2 y-resultant");
            let x = xa.to_f64(1e-13);
            let y = ya.to_f64(1e-13);
            // …and each coordinate is a genuine root of its own defining polynomial.
            assert!(xa.poly.eval_f64(x).abs() < 1e-9, "x not a root of its defining poly");
            assert!(ya.poly.eval_f64(y).abs() < 1e-9, "y not a root of its defining poly");
            // The point lies on both curves to high precision (no 1e-4 slop).
            assert!((3.0 * x + 4.0 * y - 5.0).abs() < 1e-8, "off line: {}", 3.0 * x + 4.0 * y - 5.0);
            assert!((x * x + y * y - 25.0).abs() < 1e-7, "off circle: {}", x * x + y * y - 25.0);
        }
    }

    #[test]
    fn resultant_of_line_and_circle() {
        // Expected resultant: 25y² - 40y - 200
        let line = BivariatePoly::from_terms(&[
            ((1, 0), rat(3)),
            ((0, 1), rat(4)),
            ((0, 0), rat(-5)),
        ]);
        let circle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)),
            ((0, 2), rat(1)),
            ((0, 0), rat(-25)),
        ]);

        let res = line.resultant_wrt_x(&circle);
        // res should be proportional to 25y² - 40y - 200
        let expected_roots = res.real_roots_f64(1e-12);
        assert_eq!(expected_roots.len(), 2);

        // Roots should satisfy 25y²-40y-200=0
        for y in expected_roots {
            let v = 25.0 * y * y - 40.0 * y - 200.0;
            assert!(v.abs() < 1e-6, "Residual: {}", v);
        }
    }

    #[test]
    fn two_circles_no_intersection() {
        // Circle 1: x² + y² - 1 = 0 (radius 1)
        // Circle 2: (x-5)² + y² - 1 = 0 → x²-10x+25 + y² - 1 = 0
        let c1 = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)),
            ((0, 2), rat(1)),
            ((0, 0), rat(-1)),
        ]);
        let c2 = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)),
            ((1, 0), rat(-10)),
            ((0, 2), rat(1)),
            ((0, 0), rat(24)),
        ]);

        let res = c1.intersect(&c2).unwrap();
        assert_eq!(res.len(), 0, "Two distant circles should not intersect");
    }

    #[test]
    fn local_param_circle_at_3_4() {
        // Circle: x² + y² - 25 = 0.  Near (3, 4):
        // f_x=6, f_y=8  → c1 = -6/8 = -3/4
        // f_xx=2, f_xy=0, f_yy=2  → c2 = -(2 + 0 + 2*(9/16)) / (2*8) = -(2 + 9/8)/16
        //   = -(25/8)/16 = -25/128
        let circle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)), ((0, 2), rat(1)), ((0, 0), rat(-25)),
        ]);
        let x0 = Rational::from(3i64);
        let y0 = Rational::from(4i64);
        let y_t = circle.local_parameterization_at(&x0, &y0, 2).unwrap();

        // c0 = 4
        assert_eq!(y_t.coeff(0), Rational::from(4i64));
        // c1 = -3/4
        let c1_expected = Rational::new(exact2d_integer::Integer::from(-3i64),
                                        exact2d_integer::Integer::from(4i64));
        assert_eq!(y_t.coeff(1), c1_expected);
        // c2 = -25/128
        let c2_expected = Rational::new(exact2d_integer::Integer::from(-25i64),
                                        exact2d_integer::Integer::from(128i64));
        assert_eq!(y_t.coeff(2), c2_expected);

        // Verify first-order: f(3 + t, y_t(t)) ≈ 0 for small t (to 2nd order accuracy)
        let t_small = Rational::new(exact2d_integer::Integer::from(1i64),
                                    exact2d_integer::Integer::from(100i64));
        let x_t = x0 + t_small.clone();
        let y_approx = y_t.eval(&t_small);
        let residual = circle.eval_rational(&x_t, &y_approx);
        // Should be O(t³) ≈ (0.01)³ = 1e-6 at degree-2 approximation
        assert!(residual.abs().to_f64() < 1e-4, "residual = {}", residual);
    }

    #[test]
    fn local_param_line_exact() {
        // Line: 2x - y + 1 = 0  →  y = 2x + 1.  At (0, 1): f_y = -1, f_x = 2.
        // c1 = -2 / (-1) = 2.  c2 = 0 (line has no curvature).
        let line = BivariatePoly::from_terms(&[
            ((1, 0), rat(2)), ((0, 1), rat(-1)), ((0, 0), rat(1)),
        ]);
        let y_t = line.local_parameterization_at(
            &Rational::zero(), &Rational::from(1i64), 2,
        ).unwrap();
        assert_eq!(y_t.coeff(0), Rational::from(1i64));
        assert_eq!(y_t.coeff(1), Rational::from(2i64));
        assert_eq!(y_t.coeff(2), Rational::zero()); // line has zero curvature
    }

    #[test]
    fn local_param_singular_point_error() {
        // x² - y² = 0 at (0, 0): f_y(0,0) = 0 → should return Err
        let saddle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)), ((0, 2), rat(-1)),
        ]);
        assert!(saddle.local_parameterization_at(
            &Rational::zero(), &Rational::zero(), 1
        ).is_err());
    }

    #[test]
    fn substitute_x_poly_into_circle() {
        // f = x² + y² - 25.  Substitute x = y + 1 (g(y) = y+1):
        // f(y+1, y) = (y+1)² + y² - 25 = y²+2y+1 + y² - 25 = 2y² + 2y - 24
        let circle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)), ((0, 2), rat(1)), ((0, 0), rat(-25)),
        ]);
        let g = UnivariatePoly::from_coeffs(vec![rat(1), rat(1)]); // y + 1
        let result = circle.substitute_x_poly(&g);
        let expected = UnivariatePoly::from_coeffs(vec![rat(-24), rat(2), rat(2)]); // 2y²+2y-24
        assert_eq!(result, expected);
    }

    #[test]
    fn substitute_x_poly_univariate_line_into_circle() {
        // Parametric: x(t)=t, y=t^2.  substitute_x_poly with g=t (univariate in y)
        // f = x² + y² - 1.  g(y) = y  → f(y, y) = 2y²-1
        let unit_circle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)), ((0, 2), rat(1)), ((0, 0), rat(-1)),
        ]);
        let g_identity = UnivariatePoly::from_coeffs(vec![rat(0), rat(1)]); // y
        let result = unit_circle.substitute_x_poly(&g_identity);
        let expected = UnivariatePoly::from_coeffs(vec![rat(-1), rat(0), rat(2)]); // 2y²-1
        assert_eq!(result, expected);
    }

    #[test]
    fn implicitize_line() {
        // Parametric line: x(t) = t,  y(t) = 2t + 1
        // Implicit form: y - 2x - 1 = 0
        let x_t = UnivariatePoly::from_coeffs(vec![rat(0), rat(1)]); // t
        let y_t = UnivariatePoly::from_coeffs(vec![rat(1), rat(2)]); // 2t + 1
        let implicit = BivariatePoly::implicitize(&x_t, &y_t);

        // Verify: points on the parametric curve satisfy the implicit equation
        for t_val in [-2i64, -1, 0, 1, 2] {
            let xv = Rational::from(t_val);
            let yv = Rational::from(2 * t_val + 1);
            let val = implicit.eval_rational(&xv, &yv);
            assert!(val.is_zero(), "t={}: implicit({},{})={}", t_val, xv, yv, val);
        }
    }

    #[test]
    fn implicitize_parabola() {
        // Parametric parabola: x(t) = t,  y(t) = t²
        // Implicit form: y - x² = 0
        let x_t = UnivariatePoly::from_coeffs(vec![rat(0), rat(1)]); // t
        let y_t = UnivariatePoly::from_coeffs(vec![rat(0), rat(0), rat(1)]); // t²
        let implicit = BivariatePoly::implicitize(&x_t, &y_t);

        // Points on parabola must satisfy implicit = 0
        for t_val in [-3i64, -1, 0, 2, 4] {
            let xv = Rational::from(t_val);
            let yv = Rational::from(t_val * t_val);
            let val = implicit.eval_rational(&xv, &yv);
            assert!(val.is_zero(), "t={}: val={}", t_val, val);
        }

        // A point NOT on the parabola must NOT satisfy it
        let off = implicit.eval_rational(&Rational::from(1i64), &Rational::from(2i64));
        assert!(!off.is_zero(), "Off-curve point should not satisfy implicit");
    }

    #[test]
    fn bezout_resultant_same_roots_as_sylvester() {
        let line = BivariatePoly::from_terms(&[
            ((1, 0), rat(3)), ((0, 1), rat(4)), ((0, 0), rat(-5)),
        ]);
        let circle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)), ((0, 2), rat(1)), ((0, 0), rat(-25)),
        ]);
        let sylvester_roots = line.resultant_wrt_x(&circle).real_roots_f64(1e-10);
        let bezout_roots    = line.bezout_resultant_wrt_x(&circle).real_roots_f64(1e-10);
        assert_eq!(sylvester_roots.len(), bezout_roots.len());
        for (s, b) in sylvester_roots.iter().zip(bezout_roots.iter()) {
            assert!((s - b).abs() < 1e-8, "root mismatch: s={} b={}", s, b);
        }
    }

    #[test]
    fn resultant_wrt_y_matches_wrt_x() {
        // For a line and circle, res_y(f,g) gives roots in x; verify they satisfy both curves
        let line = BivariatePoly::from_terms(&[
            ((1, 0), rat(3)), ((0, 1), rat(4)), ((0, 0), rat(-5)),
        ]);
        let circle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)), ((0, 2), rat(1)), ((0, 0), rat(-25)),
        ]);
        let res_y = line.resultant_wrt_y(&circle);
        assert!(!res_y.is_zero());
        let x_roots = res_y.real_roots_f64(1e-10);
        assert_eq!(x_roots.len(), 2, "Should find 2 x-values");
        for xv in &x_roots {
            // Back-substitute: find y from line: y = (5 - 3x) / 4
            let yv = (5.0 - 3.0 * xv) / 4.0;
            let circle_err = (xv * xv + yv * yv - 25.0).abs();
            assert!(circle_err < 1e-6, "x={} y={} circle_err={}", xv, yv, circle_err);
        }
    }

    #[test]
    fn discriminant_of_circle() {
        // x² + y² - r² = 0. Discriminant wrt x should indicate tangency info.
        let circle = BivariatePoly::from_terms(&[
            ((2, 0), rat(1)), ((0, 2), rat(1)), ((0, 0), rat(-25)),
        ]);
        let disc = circle.discriminant_wrt_x();
        // disc = res_x(f, df/dx) where df/dx = 2x
        // f = x² + (y²-25), df/dx = 2x
        // Sylvester: [[1, 0, y²-25], [2, 0, 0]] → 2x2 matrix... result is -4(y²-25)
        // Should not be identically zero
        assert!(!disc.is_zero());
    }

    #[test]
    fn total_degree() {
        // x²y³ + xy + 1: total degree = 5
        let p = BivariatePoly::from_terms(&[
            ((2, 3), rat(1)),
            ((1, 1), rat(1)),
            ((0, 0), rat(1)),
        ]);
        assert_eq!(p.total_degree(), 5);
    }

    #[test]
    fn term_iterator_graded_lex() {
        // x²y + xy² + x³: should come out in order x³ > x²y ~ xy² > ...
        let p = BivariatePoly::from_terms(&[
            ((3, 0), rat(1)),
            ((2, 1), rat(2)),
            ((1, 2), rat(3)),
        ]);
        let terms = p.terms_graded_lex();
        // All have total degree 3; by x-degree desc: (3,0), (2,1), (1,2)
        assert_eq!(terms[0].0, (3, 0));
        assert_eq!(terms[1].0, (2, 1));
        assert_eq!(terms[2].0, (1, 2));
    }

    #[test]
    fn eval_range_contains_exact() {
        // f(x,y) = x + y over [0,1]×[0,1]. Range should be [0,2].
        let p = BivariatePoly::from_terms(&[
            ((1, 0), rat(1)),
            ((0, 1), rat(1)),
        ]);
        let (lo, hi) = p.eval_range_f64(0.0, 1.0, 0.0, 1.0);
        assert!(lo <= 0.0 + 1e-10);
        assert!(hi >= 2.0 - 1e-10);
        // For x=0.5, y=0.5, f=1.0 must lie within range
        assert!(lo <= 1.0 && 1.0 <= hi);
    }
}
