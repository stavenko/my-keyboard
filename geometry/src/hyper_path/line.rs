use std::fmt;
use std::ops::{Add, Mul, Sub};

use itertools::Itertools;
use num_traits::{One, Pow, Zero};

use super::hyper_line::HyperLine;
use super::hyper_point::Tensor;

pub trait GetT {
    type Value;
    type Scalar;
    fn get_t(&self, t: Self::Scalar) -> Self::Value;
}

impl<const O: usize, T> GetT for HyperLine<O, T>
where
    T: Tensor + Mul<T::Scalar, Output = T>,
    T::Scalar: From<u16>,
{
    type Value = T;
    type Scalar = T::Scalar;

    fn get_t(&self, t: Self::Scalar) -> Self::Value
where {
        let ws = (0..O).map(|i| bernstein::<O, _>(i, t));
        let v: Vec<T> = ws.zip(self.0).map(|(w, t)| t * w).collect_vec();
        v.into_iter().fold(T::zero(), |a, t| a + t)
    }
}

pub(crate) fn bernstein<const O: usize, F>(item: usize, t: F) -> F
where
    F: One + Sub<F, Output = F> + Pow<u16, Output = F> + From<u16> + Copy + fmt::Debug,
{
    let opt = O - 1;
    let factor = (fact(opt) / (fact(item) * fact(opt - item))) as u16;
    let ot = F::one() - t;
    let o_item = opt - item;

    t.pow(item as u16) * ot.pow(o_item as u16) * factor.into()
}

const fn fact(i: usize) -> usize {
    match i {
        0 => 1,
        1 => 1,
        2 => 2,
        x => x * fact(x - 1),
    }
}

#[cfg(test)]
mod tests {
    use itertools::Itertools;
    use num_traits::{One, Zero};

    use crate::decimal::Dec;

    #[test]
    fn bernstein() {
        let t = Dec::zero();
        let ws = (0..4).map(|i| super::bernstein::<4, _>(i, t)).collect_vec();
        assert_eq!(ws, vec![Dec::one(), Dec::zero(), Dec::zero(), Dec::zero(),]);

        let t = Dec::one();
        let ws = (0..4).map(|i| super::bernstein::<4, _>(i, t)).collect_vec();
        assert_eq!(ws, vec![Dec::zero(), Dec::zero(), Dec::zero(), Dec::one(),]);
    }
}
