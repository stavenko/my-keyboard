use core::fmt;
use std::{
    ops::{Mul, Sub},
    process::Output,
};

use nalgebra::{Dim, Matrix, Storage};
use num_traits::Zero;

use crate::{decimal::Dec, parametric_iterator::ParametricIterator};

use super::{hyper_point::Tensor, length::Length, line::GetT};

#[derive(Clone, PartialEq)]
pub struct HyperLine<const ORDER: usize, T>(pub(super) [T; ORDER]);

impl<const O: usize> fmt::Debug for HyperLine<O, Dec> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for i in 0..O {
            write!(f, " HL {i}: {} ", self.0[i])?;
        }
        Ok(())
    }
}

impl<const O: usize, R, C, S> fmt::Debug for HyperLine<O, Matrix<Dec, R, C, S>>
where
    R: Dim,
    C: Dim,
    S: Storage<Dec, R, C>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for i in 0..O {
            write!(f, "\n HL {i}: ")?;
            for item in self.0[i].iter() {
                write!(f, "{} ", item.round_dp(4))?;
            }
        }
        Ok(())
    }
}
impl<T> HyperLine<2, T> {
    pub fn new_2(a: T, b: T) -> Self {
        Self([a, b])
    }

    /*
        fn get_t<const O: usize, T>(&self, t: <T as Length>::Scalar) -> _
        where
            T: Zero + Clone + Copy + std::iter::Sum,
            T: Sub<T, Output = T>,
            T: Length,
            T: Mul<<T as Length>::Scalar, Output = T>,
            <T as Length>::Scalar: Div<<T as Length>::Scalar, Output = <T as Length>::Scalar>
                + Sub<<T as Length>::Scalar, Output = <T as Length>::Scalar>
                + Zero
                + One
                + Copy
                + Debug
                + From<u16>
                + Pow<u16, Output = <T as Length>::Scalar>
                + AddAssign<<T as Length>::Scalar>,
        {
            todo!()
        }
    */
}

impl<T> HyperLine<4, T> {
    pub fn new_4(a: T, b: T, c: T, d: T) -> Self {
        Self([a, b, c, d])
    }
}

impl<const O: usize, T> Length for HyperLine<O, T>
where
    T: Tensor + Mul<<T as Tensor>::Scalar, Output = T>,
    <T as Tensor>::Scalar: From<u16>,
    <T as Sub<T>>::Output: Length<Scalar = <T as Tensor>::Scalar>,
    T: Length<Scalar = <T as Tensor>::Scalar>,
{
    fn length(&self) -> Self::Scalar {
        if O == 2 {
            let distance = self.0[0] - self.0[1];
            distance.length()
        } else {
            let mut total = Self::Scalar::zero();
            for (t, tt) in ParametricIterator::<Self::Scalar>::new(10) {
                let t0 = self.get_t(t);
                let t1 = self.get_t(tt);
                total += (t0 - t1).length();
            }
            total
        }
    }

    type Scalar = <<T as Sub<T>>::Output as Length>::Scalar;
}
