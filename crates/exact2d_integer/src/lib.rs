use num_bigint::{BigInt, Sign};
use num_integer::Integer as NumIntegerTrait;
use num_traits::{Zero, One, Signed, ToPrimitive};
use std::fmt;
use std::ops::{Add, Sub, Mul, Div, Rem, Neg};
use serde::{Serialize, Deserialize};

/// Arbitrary-precision integer. Wraps `BigInt` with a clean API.
/// Phase 1 uses the `num-bigint` backend; a SIMD-optimized inline-storage
/// variant will replace it in Phase 3.
#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
pub struct Integer(BigInt);

impl Integer {
    pub fn zero() -> Self { Integer(BigInt::zero()) }
    pub fn one() -> Self { Integer(BigInt::one()) }
    pub fn minus_one() -> Self { Integer(BigInt::from(-1i64)) }

    /// Parse an arbitrary-precision integer from a decimal string. Returns `None`
    /// if the string is not a valid integer. Used by file-format (de)serialization.
    pub fn from_dec_str(s: &str) -> Option<Integer> {
        s.trim().parse::<BigInt>().ok().map(Integer)
    }

    pub fn is_zero(&self) -> bool { self.0.is_zero() }
    pub fn is_positive(&self) -> bool { self.0.is_positive() }
    pub fn is_negative(&self) -> bool { self.0.is_negative() }

    pub fn abs(&self) -> Self {
        Integer(if self.0.is_negative() { -self.0.clone() } else { self.0.clone() })
    }

    pub fn signum(&self) -> i32 {
        match self.0.sign() {
            Sign::Minus => -1,
            Sign::NoSign => 0,
            Sign::Plus => 1,
        }
    }

    pub fn gcd(a: &Self, b: &Self) -> Self {
        Integer(a.0.gcd(&b.0))
    }

    pub fn lcm(a: &Self, b: &Self) -> Self {
        Integer(a.0.lcm(&b.0))
    }

    pub fn to_f64(&self) -> f64 {
        self.0.to_f64().unwrap_or(if self.0.is_positive() { f64::INFINITY } else { f64::NEG_INFINITY })
    }

    /// Number of bits in the magnitude.
    pub fn bits(&self) -> u64 {
        self.0.bits()
    }

    /// Returns `Some(s)` if `self` is a perfect square and `s² = self`, else `None`.
    /// Only defined for non-negative integers.
    pub fn integer_sqrt_exact(&self) -> Option<Integer> {
        if self.is_negative() { return None; }
        if self.is_zero() { return Some(Integer::zero()); }
        // Newton's method for integer square root
        let mut x = {
            // Initial guess: 2^(bits/2 + 1)
            let bits = self.0.bits();
            let shift = (bits / 2) + 1;
            Integer(num_bigint::BigInt::from(1u64) << shift as usize)
        };
        loop {
            let x1 = (x.clone() + self.clone() / x.clone()) / Integer::from(2i64);
            if x1 >= x { break; }
            x = x1;
        }
        if x.clone() * x.clone() == *self {
            Some(x)
        } else {
            None
        }
    }

    pub fn pow_u32(&self, exp: u32) -> Self {
        use num_traits::Pow;
        Integer(self.0.clone().pow(exp))
    }
}

// ── From conversions ────────────────────────────────────────────────────────

impl From<i64> for Integer { fn from(n: i64) -> Self { Integer(BigInt::from(n)) } }
impl From<i32> for Integer { fn from(n: i32) -> Self { Integer(BigInt::from(n)) } }
impl From<u64> for Integer { fn from(n: u64) -> Self { Integer(BigInt::from(n)) } }
impl From<u32> for Integer { fn from(n: u32) -> Self { Integer(BigInt::from(n)) } }
impl From<usize> for Integer { fn from(n: usize) -> Self { Integer(BigInt::from(n)) } }

// ── Arithmetic operators (owned) ─────────────────────────────────────────────

impl Add for Integer {
    type Output = Self;
    fn add(self, rhs: Self) -> Self { Integer(self.0 + rhs.0) }
}
impl Sub for Integer {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self { Integer(self.0 - rhs.0) }
}
impl Mul for Integer {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self { Integer(self.0 * rhs.0) }
}
impl Div for Integer {
    type Output = Self;
    fn div(self, rhs: Self) -> Self { Integer(self.0 / rhs.0) }
}
impl Rem for Integer {
    type Output = Self;
    fn rem(self, rhs: Self) -> Self { Integer(self.0 % rhs.0) }
}
impl Neg for Integer {
    type Output = Self;
    fn neg(self) -> Self { Integer(-self.0) }
}

// ── Arithmetic operators (refs) ───────────────────────────────────────────────

impl<'b> Add<&'b Integer> for &Integer {
    type Output = Integer;
    fn add(self, rhs: &'b Integer) -> Integer { Integer(&self.0 + &rhs.0) }
}
impl<'b> Sub<&'b Integer> for &Integer {
    type Output = Integer;
    fn sub(self, rhs: &'b Integer) -> Integer { Integer(&self.0 - &rhs.0) }
}
impl<'b> Mul<&'b Integer> for &Integer {
    type Output = Integer;
    fn mul(self, rhs: &'b Integer) -> Integer { Integer(&self.0 * &rhs.0) }
}
impl Neg for &Integer {
    type Output = Integer;
    fn neg(self) -> Integer { Integer(-self.0.clone()) }
}

// ── Display ──────────────────────────────────────────────────────────────────

impl fmt::Display for Integer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result { write!(f, "{}", self.0) }
}
impl fmt::Debug for Integer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result { write!(f, "Integer({})", self.0) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_arithmetic() {
        let a = Integer::from(7i64);
        let b = Integer::from(3i64);
        assert_eq!(a.clone() + b.clone(), Integer::from(10i64));
        assert_eq!(a.clone() - b.clone(), Integer::from(4i64));
        assert_eq!(a.clone() * b.clone(), Integer::from(21i64));
        assert_eq!(a.clone() / b.clone(), Integer::from(2i64));
        assert_eq!(a.clone() % b.clone(), Integer::from(1i64));
    }

    #[test]
    fn gcd() {
        assert_eq!(Integer::gcd(&Integer::from(12i64), &Integer::from(8i64)), Integer::from(4i64));
        assert_eq!(Integer::gcd(&Integer::from(0i64), &Integer::from(5i64)), Integer::from(5i64));
    }

    #[test]
    fn large_integers() {
        // 2^128 should not overflow
        let two = Integer::from(2i64);
        let large = two.pow_u32(128);
        assert!(large.bits() > 100);
    }
}
