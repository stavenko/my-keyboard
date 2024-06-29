use std::ops::Add;

use geometry::decimal::Dec;
use num_traits::Zero;
use rust_decimal::Decimal;

#[derive(Default)]
pub struct Angle(Dec);

impl Add for Angle {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Zero for Angle {
    fn zero() -> Self {
        Self(Dec::zero())
    }

    fn is_zero(&self) -> bool {
        self.0.is_zero()
    }
}

impl Angle {
    pub fn from_deg(deg: impl Into<Dec>) -> Self {
        Self(deg.into() * Dec::from(Decimal::PI) / Dec::from(180))
    }

    pub fn deg(&self) -> Dec {
        self.0 / Dec::from(Decimal::PI) * Dec::from(180)
    }

    pub fn rad(&self) -> Dec {
        self.0
        /*
        match self {
            Self::Rad(r) => *r,
            Self::Deg(d) => *d * Dec::from(Decimal::PI) / Dec::from(180),
        }
         */
    }
}
