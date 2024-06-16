use core::fmt;
use std::{
    fmt::{Debug, Display},
    ops::{Add, Div, Mul, Sub},
};

use nalgebra::Vector3;
use num_traits::{One, Pow, Zero};

use crate::parametric_iterator::ParametricIterator;

use super::{
    hyper_path::IsLinear,
    hyper_point::{Point, SuperPoint, Tensor},
    length::Length,
    line::GetT,
    split_hyper_line::SplitHyperLine,
};

#[derive(Clone, PartialEq)]
pub struct HyperLine<T>(pub(super) Vec<T>);

impl<T: fmt::Debug> fmt::Debug for HyperLine<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for i in 0..self.0.len() {
            write!(f, "\nHL {i}: {:?} ", self.0[i])?;
        }
        Ok(())
    }
}

impl<T> HyperLine<T>
where
    T: Clone,
{
    pub fn new_2(a: T, b: T) -> Self {
        Self([a, b].to_vec())
    }

    pub fn new_4(a: T, b: T, c: T, d: T) -> Self {
        Self([a, b, c, d].to_vec())
    }
}

pub trait ShiftInPlane {
    type Scalar;
    fn shift_in_plane(self, normal: Vector3<Self::Scalar>, amount: Self::Scalar) -> Self;
}

impl<T> ShiftInPlane for HyperLine<T>
where
    T: Tensor + Mul<T::Scalar, Output = T> + Point<Vector = Vector3<T::Scalar>>,
    T::Scalar: From<u16>
        + One
        + Zero
        + Div<Output = T::Scalar>
        + Pow<u16, Output = T::Scalar>
        + Sub
        + Debug
        + Display
        + Copy
        + nalgebra::Field
        + nalgebra::ComplexField
        + nalgebra::Scalar,
{
    type Scalar = T::Scalar;
    fn shift_in_plane(mut self, normal: Vector3<T::Scalar>, amount: T::Scalar) -> Self {
        let delta = T::Scalar::one() / T::Scalar::from(65535);
        let half = T::Scalar::one() / T::Scalar::from(2);
        let b = self.get_t(T::Scalar::zero());
        let bb = self.get_t(delta);
        let e = self.get_t(T::Scalar::one() - delta);
        let ee = self.get_t(T::Scalar::one());
        let dir_b = (bb.point() - b.point()).normalize();
        let dir_e = (ee.point() - e.point()).normalize();
        let ext_b = dir_b.cross(&normal);
        let ext_e = dir_e.cross(&normal);

        if self.0.len() % 2 == 0 {
            let center = self.0.len() / 2;
            for (ix, p) in self.0.iter_mut().enumerate() {
                if ix < center {
                    p.set_point(p.point() + ext_b * amount);
                } else {
                    p.set_point(p.point() + ext_e * amount);
                }
            }
        } else {
            let center = self.0.len() / 2;
            for (ix, p) in self.0.iter_mut().enumerate() {
                if ix < center {
                    p.set_point(p.point() + ext_b * amount)
                } else if ix == center {
                    p.set_point(p.point() + (ext_b.lerp(&ext_e, half)) * amount)
                } else {
                    p.set_point(p.point() + ext_e * amount)
                }
            }
        }

        self
    }
}

impl<T> Length for HyperLine<T>
where
    T: Tensor + Mul<<T as Tensor>::Scalar, Output = T>,
    <T as Tensor>::Scalar: From<u16>,
    <T as Sub<T>>::Output: Length<Scalar = <T as Tensor>::Scalar>,
    T: Length<Scalar = <T as Tensor>::Scalar>,
{
    fn length(&self) -> Self::Scalar {
        if self.0.len() == 2 {
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

    type Scalar = <T as Tensor>::Scalar;
}

impl<S, T> SplitHyperLine<S> for HyperLine<T>
where
    T: Sub<T, Output = T>,
    T: Add<T, Output = T>,
    T: Mul<S, Output = T>,
    T: Copy,
    S: Copy,
{
    fn split_hyper_line(&self, t: S) -> (Self, Self)
    where
        Self: Sized,
    {
        match self.0.len() {
            2 => {
                let v = self.0[1] - self.0[0];
                let c = self.0[0] + v * t;

                (
                    HyperLine::new_2(self.0[0], c),
                    HyperLine::new_2(c, self.0[1]),
                )
            }
            4 => {
                let [a, b, c, d] = [self.0[0], self.0[1], self.0[2], self.0[3]];
                let e = lerp(&a, &b, t);
                let f = lerp(&b, &c, t);
                let g = lerp(&c, &d, t);
                let h = lerp(&e, &f, t);
                let j = lerp(&f, &g, t);
                let k = lerp(&h, &j, t);
                (Self::new_4(a, e, h, k), Self::new_4(k, j, g, d))
            }
            n => {
                unimplemented!("Hyper line for {n}");
            }
        }
    }
}

impl<S> HyperLine<SuperPoint<S>> {
    pub fn to_points(self) -> HyperLine<Vector3<S>> {
        HyperLine(self.0.into_iter().map(|sp| sp.point).collect())
    }
}

impl<T> IsLinear for HyperLine<T> {
    fn is_linear(&self) -> bool {
        self.0.len() == 2
    }
}

fn lerp<T, S>(one: &T, other: &T, t: S) -> T
where
    T: Sub<T, Output = T> + Add<T, Output = T> + Mul<S, Output = T> + Copy,
{
    let d = *other - *one;
    *one + d * t
}
