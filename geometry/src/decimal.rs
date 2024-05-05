use std::{
    cmp::Ordering,
    fmt,
    iter::{Product, Sum},
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Rem, RemAssign, Sub, SubAssign},
};

use approx::{AbsDiffEq, UlpsEq};
use nalgebra::{ComplexField, Field, RealField, SimdValue, Vector3};
use num_traits::{pow::Pow, Bounded, FromPrimitive, Num, Signed, ToPrimitive};
use rust_decimal::{
    prelude::{One, Zero},
    Decimal, MathematicalOps,
};
use rust_decimal_macros::dec;
use simba::scalar::{SubsetOf, SupersetOf};

#[derive(PartialEq, PartialOrd, Eq, Ord, Hash, Clone, Copy, Default)]
pub struct Dec(Decimal);

impl fmt::Debug for Dec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Dec: {}", self.round_dp(5))
    }
}

pub const EPS: Dec = Dec(dec!(1e-8));
pub const STABILITY_ROUNDING: u32 = 14;
pub const NORMAL_DOT_ROUNDING: u32 = 4;
//pub const STABILITY_ROUNDING_F: u32 = 15;

impl SubsetOf<Dec> for Dec {
    fn to_superset(&self) -> Dec {
        *self
    }

    fn from_superset_unchecked(element: &Dec) -> Self {
        *element
    }

    fn is_in_subset(_element: &Dec) -> bool {
        true
    }
}

impl Pow<usize> for Dec {
    type Output = Dec;

    fn pow(self, rhs: usize) -> Self::Output {
        Dec(self.0.powu(rhs as u64))
    }
}

impl Pow<u64> for Dec {
    type Output = Dec;

    fn pow(self, rhs: u64) -> Self::Output {
        Dec(self.0.powu(rhs))
    }
}

impl Pow<u16> for Dec {
    type Output = Dec;

    fn pow(self, rhs: u16) -> Self::Output {
        Dec(self.0.powu(rhs as u64))
    }
}

impl Pow<i64> for Dec {
    type Output = Dec;

    fn pow(self, rhs: i64) -> Self::Output {
        Dec(self.0.powi(rhs))
    }
}

impl Mul<Vector3<Dec>> for Dec {
    type Output = Vector3<Dec>;

    fn mul(self, rhs: Vector3<Dec>) -> Self::Output {
        self * rhs
    }
}

impl ComplexField for Dec {
    type RealField = Self;

    #[doc = r" Builds a pure-real complex number from the given value."]
    fn from_real(_re: Self::RealField) -> Self {
        todo!()
    }

    #[doc = r" The real part of this complex number."]
    fn real(self) -> Self::RealField {
        self
    }

    #[doc = r" The imaginary part of this complex number."]
    fn imaginary(self) -> Self::RealField {
        Self::zero()
    }

    #[doc = r" The modulus of this complex number."]
    fn modulus(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" The squared modulus of this complex number."]
    fn modulus_squared(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" The argument of this complex number."]
    fn argument(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" The sum of the absolute value of this complex number's real and imaginary part."]
    fn norm1(self) -> Self::RealField {
        todo!()
    }

    #[doc = r" Multiplies this complex number by `factor`."]
    fn scale(self, _factor: Self::RealField) -> Self {
        todo!()
    }

    #[doc = r" Divides this complex number by `factor`."]
    fn unscale(self, factor: Self::RealField) -> Self {
        self / factor
    }

    fn floor(self) -> Self {
        todo!()
    }

    fn ceil(self) -> Self {
        todo!()
    }

    fn round(self) -> Self {
        todo!()
    }

    fn trunc(self) -> Self {
        todo!()
    }

    fn fract(self) -> Self {
        todo!()
    }

    fn mul_add(self, _a: Self, _b: Self) -> Self {
        todo!()
    }

    #[doc = r" The absolute value of this complex number: `self / self.signum()`."]
    #[doc = r""]
    #[doc = r" This is equivalent to `self.modulus()`."]
    fn abs(self) -> Self::RealField {
        Self(self.0.abs())
    }

    #[doc = r" Computes (self.conjugate() * self + other.conjugate() * other).sqrt()"]
    fn hypot(self, _other: Self) -> Self::RealField {
        todo!()
    }

    fn recip(self) -> Self {
        todo!()
    }

    fn conjugate(self) -> Self {
        self
    }

    fn sin(self) -> Self {
        Self(self.0.sin())
    }

    fn cos(self) -> Self {
        Self(self.0.cos())
    }

    fn sin_cos(self) -> (Self, Self) {
        (Self(self.0.sin()), Self(self.0.cos()))
    }

    fn tan(self) -> Self {
        todo!()
    }

    fn asin(self) -> Self {
        todo!()
    }

    fn acos(self) -> Self {
        let inner: f64 = self.into();
        Self(Decimal::from_f64(inner.acos()).expect("conversion must be ok"))
    }

    fn atan(self) -> Self {
        todo!()
    }

    fn sinh(self) -> Self {
        todo!()
    }

    fn cosh(self) -> Self {
        todo!()
    }

    fn tanh(self) -> Self {
        todo!()
    }

    fn asinh(self) -> Self {
        todo!()
    }

    fn acosh(self) -> Self {
        todo!()
    }

    fn atanh(self) -> Self {
        todo!()
    }

    fn log(self, _base: Self::RealField) -> Self {
        todo!()
    }

    fn log2(self) -> Self {
        todo!()
    }

    fn log10(self) -> Self {
        todo!()
    }

    fn ln(self) -> Self {
        todo!()
    }

    fn ln_1p(self) -> Self {
        todo!()
    }

    fn sqrt(self) -> Self {
        // Self(self.0.powd(dec!(0.5)))
        Self(self.0.sqrt().expect("aaa"))
    }

    fn exp(self) -> Self {
        Self(self.0.exp())
    }

    fn exp2(self) -> Self {
        todo!()
    }

    fn exp_m1(self) -> Self {
        todo!()
    }

    fn powi(self, _n: i32) -> Self {
        todo!()
    }

    fn powf(self, _n: Self::RealField) -> Self {
        todo!()
    }

    fn powc(self, _n: Self) -> Self {
        todo!()
    }

    fn cbrt(self) -> Self {
        todo!()
    }

    fn is_finite(&self) -> bool {
        todo!()
    }

    fn try_sqrt(self) -> Option<Self> {
        todo!()
    }
}

impl RealField for Dec {
    fn is_sign_positive(&self) -> bool {
        self.0.is_sign_positive()
    }

    fn is_sign_negative(&self) -> bool {
        self.0.is_sign_negative()
    }

    fn copysign(self, _sign: Self) -> Self {
        (self.0 / self.0.abs()).into()
    }

    fn max(self, other: Self) -> Self {
        self.0.max(other.0).into()
    }

    fn min(self, other: Self) -> Self {
        self.0.min(other.0).into()
    }

    fn clamp(self, min: Self, max: Self) -> Self {
        self.0.clamp(min.0, max.0).into()
    }

    fn atan2(self, _other: Self) -> Self {
        todo!()
    }

    fn min_value() -> Option<Self> {
        Some(<Self as Bounded>::min_value())
    }

    fn max_value() -> Option<Self> {
        Some(<Self as Bounded>::max_value())
    }

    fn pi() -> Self {
        todo!()
    }

    fn two_pi() -> Self {
        todo!()
    }

    fn frac_pi_2() -> Self {
        todo!()
    }

    fn frac_pi_3() -> Self {
        todo!()
    }

    fn frac_pi_4() -> Self {
        todo!()
    }

    fn frac_pi_6() -> Self {
        todo!()
    }

    fn frac_pi_8() -> Self {
        todo!()
    }

    fn frac_1_pi() -> Self {
        todo!()
    }

    fn frac_2_pi() -> Self {
        todo!()
    }

    fn frac_2_sqrt_pi() -> Self {
        todo!()
    }

    fn e() -> Self {
        todo!()
    }

    fn log2_e() -> Self {
        todo!()
    }

    fn log10_e() -> Self {
        todo!()
    }

    fn ln_2() -> Self {
        todo!()
    }

    fn ln_10() -> Self {
        todo!()
    }
}
impl Signed for Dec {
    fn abs(&self) -> Self {
        self.0.abs().into()
    }

    fn abs_sub(&self, other: &Self) -> Self {
        (*self - *other).abs()
    }

    fn signum(&self) -> Self {
        *self / self.abs()
    }

    fn is_positive(&self) -> bool {
        self.0.is_sign_positive()
    }

    fn is_negative(&self) -> bool {
        self.0.is_sign_negative()
    }
}
impl UlpsEq for Dec {
    fn default_max_ulps() -> u32 {
        Self::EPSILON.into()
    }

    fn ulps_eq(&self, other: &Self, _epsilon: Self::Epsilon, _max_ulps: u32) -> bool {
        self.0 == other.0
    }
}
impl AbsDiffEq for Dec {
    type Epsilon = Self;

    fn default_epsilon() -> Self::Epsilon {
        Self::EPSILON
    }

    fn abs_diff_eq(&self, other: &Self, _epsilon: Self::Epsilon) -> bool {
        self.0 == other.0
    }
}

impl approx::RelativeEq for Dec {
    fn default_max_relative() -> Self::Epsilon {
        Dec::EPSILON
    }

    fn relative_eq(
        &self,
        other: &Self,
        _epsilon: Self::Epsilon,
        _max_relative: Self::Epsilon,
    ) -> bool {
        self.0 == other.0
    }
}

impl FromPrimitive for Dec {
    fn from_i64(n: i64) -> Option<Self> {
        Decimal::from_i64(n).map(Self)
    }

    fn from_u64(n: u64) -> Option<Self> {
        Decimal::from_u64(n).map(Self)
    }
}

impl Field for Dec {}

impl SimdValue for Dec {
    type Element = Dec;

    type SimdBool = bool;

    fn lanes() -> usize {
        1
    }

    fn splat(val: Self::Element) -> Self {
        val
    }

    fn extract(&self, _: usize) -> Self::Element {
        *self
    }

    unsafe fn extract_unchecked(&self, _i: usize) -> Self::Element {
        *self
    }

    fn replace(&mut self, _i: usize, val: Self::Element) {
        *self = val
    }

    unsafe fn replace_unchecked(&mut self, _i: usize, val: Self::Element) {
        *self = val
    }

    fn select(self, cond: Self::SimdBool, other: Self) -> Self {
        if cond {
            self
        } else {
            other
        }
    }
}

impl Num for Dec {
    type FromStrRadixErr = <Decimal as Num>::FromStrRadixErr;

    fn from_str_radix(str: &str, radix: u32) -> Result<Self, Self::FromStrRadixErr> {
        let d = Decimal::from_str_radix(str, radix)?;
        Ok(Self(d))
    }
}
impl SupersetOf<f64> for Dec {
    fn is_in_subset(&self) -> bool {
        true
    }

    fn to_subset_unchecked(&self) -> f64 {
        self.0.to_f64().unwrap()
    }

    fn from_subset(element: &f64) -> Self {
        Dec::from(*element)
    }
}

impl Sum for Dec {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        let mut d = Self(Decimal::zero());
        for i in iter {
            d += i
        }
        d
    }
}
impl Product for Dec {
    fn product<I: Iterator<Item = Self>>(iter: I) -> Self {
        let mut d = Self(Decimal::one());
        for i in iter {
            d *= i
        }
        d
    }
}

impl From<f32> for Dec {
    fn from(value: f32) -> Self {
        Self(Decimal::from_f32_retain(value).unwrap_or_else(|| {
            println!("WARNING: Cannot convert float f32 to decimal `{value}`, setting 0");

            Decimal::zero()
        }))
    }
}
impl From<Dec> for i128 {
    fn from(value: Dec) -> Self {
        value.0.to_i128().unwrap_or_else(|| {
            println!("WARNING: Cannot convert float f32 to decimal `{value}`, setting 0");
            0
        })
    }
}
impl From<Dec> for u32 {
    fn from(value: Dec) -> Self {
        value.0.to_u32().unwrap_or_else(|| {
            println!("WARNING: Cannot convert float u32 to decimal `{value}`, setting 0");
            0
        })
    }
}

impl From<Dec> for f64 {
    fn from(value: Dec) -> Self {
        value.0.to_f64().unwrap_or_else(|| {
            println!("WARNING: Cannot convert float f32 to decimal `{value}`, setting 0");
            0.0
        })
    }
}

impl From<Dec> for f32 {
    fn from(value: Dec) -> Self {
        value.0.to_f32().unwrap_or_else(|| {
            println!("WARNING: Cannot convert float f32 to decimal `{value}`, setting 0");
            0.0
        })
    }
}
impl From<f64> for Dec {
    fn from(value: f64) -> Self {
        Self(Decimal::from_f64_retain(value).unwrap_or_else(|| {
            println!("WARNING: Cannot convert float f64 to decimal `{value}`, setting 0");

            Decimal::zero()
        }))
    }
}

impl From<i128> for Dec {
    fn from(value: i128) -> Self {
        Self(Decimal::from_i128(value).unwrap_or_else(|| {
            println!("WARNING: Cannot convert integer i128 to decimal `{value}`, setting 0");

            Decimal::zero()
        }))
    }
}
impl From<i64> for Dec {
    fn from(value: i64) -> Self {
        Self(Decimal::from_i64(value).unwrap_or_else(|| {
            println!("WARNING: Cannot convert integer i64 to decimal `{value}`, setting 0");

            Decimal::zero()
        }))
    }
}
impl From<i32> for Dec {
    fn from(value: i32) -> Self {
        Self(Decimal::from_i32(value).unwrap_or_else(|| {
            println!("WARNING: Cannot convert integer i32 to decimal `{value}`, setting 0");

            Decimal::zero()
        }))
    }
}
impl From<u32> for Dec {
    fn from(value: u32) -> Self {
        Self(Decimal::from_u32(value).unwrap_or_else(|| {
            println!("WARNING: Cannot convert integer u32 to decimal `{value}`, setting 0");

            Decimal::zero()
        }))
    }
}

impl From<u16> for Dec {
    fn from(value: u16) -> Self {
        Self(Decimal::from_u16(value).unwrap_or_else(|| {
            println!("WARNING: Cannot convert integer u32 to decimal `{value}`, setting 0");

            Decimal::zero()
        }))
    }
}

impl From<usize> for Dec {
    fn from(value: usize) -> Self {
        Self(Decimal::from_usize(value).unwrap_or_else(|| {
            println!("WARNING: Cannot convert integer usize to decimal `{value}`, setting 0");

            Decimal::zero()
        }))
    }
}

/*
impl<RHS> Pow<RHS> for Dec
where
    Decimal: Pow<RHS, Output = Decimal>,
{
    type Output = Self;

    fn pow(self, rhs: RHS) -> Self::Output {
        Self(self.0.pow(rhs))
    }
}
*/

impl Zero for Dec {
    fn zero() -> Self {
        Self(Decimal::zero())
    }

    fn is_zero(&self) -> bool {
        self.0.is_zero()
    }
}

impl One for Dec {
    fn one() -> Self {
        Self(Decimal::one())
    }
}

impl Dec {
    pub const EPSILON: Self = Self(Decimal::from_parts(1, 0, 0, false, 28));

    pub fn round(&self) -> Self {
        Self(self.0.round())
    }

    pub fn round_dp(&self, dp: u32) -> Self {
        Self(self.0.round_dp(dp))
    }

    pub fn total_cmp(&self, other: &Self) -> Ordering {
        self.0.cmp(&other.0)
    }
}

impl fmt::Display for Dec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

impl DivAssign for Dec {
    fn div_assign(&mut self, rhs: Self) {
        if rhs == Self::zero() {
            dbg!("QQQQQQQ");
        }
        self.0 /= rhs.0;
    }
}

impl AddAssign for Dec {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}
impl SubAssign for Dec {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl MulAssign for Dec {
    fn mul_assign(&mut self, rhs: Self) {
        self.0 *= rhs.0;
    }
}

impl RemAssign for Dec {
    fn rem_assign(&mut self, rhs: Self) {
        self.0 %= rhs.0;
    }
}

impl Neg for Dec {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}
impl Rem for Dec {
    type Output = Self;

    fn rem(self, rhs: Self) -> Self::Output {
        Dec(self.0 % rhs.0)
    }
}
impl Div for Dec {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        if rhs == Self::zero() {
            panic!("div by zero {self} / {rhs}")
        }
        Dec(self.0 / rhs.0)
    }
}
impl Div<i64> for Dec {
    type Output = Self;

    fn div(self, rhs: i64) -> Self::Output {
        if rhs == 0 {
            dbg!("IIII");
        }
        self / Dec::from(rhs)
    }
}

impl Add for Dec {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let d = self.0 + rhs.0;

        Dec(d)
    }
}
impl Add<i64> for Dec {
    type Output = Self;

    fn add(self, rhs: i64) -> Self::Output {
        dbg!("add i64");
        self + Dec::from(rhs)
    }
}
impl Sub for Dec {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Dec(self.0 - rhs.0)
    }
}
impl Sub<i64> for Dec {
    type Output = Self;

    fn sub(self, rhs: i64) -> Self::Output {
        self - Dec::from(rhs)
    }
}

impl Mul for Dec {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Dec(self.0 * rhs.0)
    }
}

impl Mul<i64> for Dec {
    type Output = Self;

    fn mul(self, rhs: i64) -> Self::Output {
        self * Dec::from(rhs)
    }
}

impl Mul<Dec> for i64 {
    type Output = Dec;

    fn mul(self, rhs: Dec) -> Self::Output {
        Dec::from(self) * rhs
    }
}

impl Mul<Dec> for f64 {
    type Output = Dec;

    fn mul(self, rhs: Dec) -> Self::Output {
        (Dec::from(self) * rhs).round_dp(8)
    }
}

impl Mul<Dec> for f32 {
    type Output = Dec;

    fn mul(self, rhs: Dec) -> Self::Output {
        (Dec::from(self) * rhs).round_dp(10)
    }
}

impl Mul<f64> for Dec {
    type Output = Dec;

    fn mul(self, rhs: f64) -> Self::Output {
        (Dec::from(rhs) * self).round_dp(8)
    }
}

impl Mul<f32> for Dec {
    type Output = Dec;

    fn mul(self, rhs: f32) -> Self::Output {
        (Dec::from(rhs) * self).round_dp(10)
    }
}

impl From<Decimal> for Dec {
    fn from(value: Decimal) -> Self {
        Dec(value)
    }
}

impl From<Dec> for Decimal {
    fn from(value: Dec) -> Self {
        value.0
    }
}

impl Bounded for Dec {
    fn min_value() -> Self {
        Decimal::MIN.into()
    }

    fn max_value() -> Self {
        Decimal::MAX.into()
    }
}
