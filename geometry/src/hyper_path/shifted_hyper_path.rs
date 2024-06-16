use core::fmt::{Debug, Display};
use std::{
    marker::PhantomData,
    ops::{Div, Mul, Sub},
};

use nalgebra::{Vector3};
use num_traits::{One, Pow, Zero};

use super::{
    hyper_line::{HyperLine, ShiftInPlane},
    hyper_path::HyperPath,
    hyper_point::{Point, SideDir, Tensor},
    length::Length,
};

pub struct ShiftedSideHyperPath<S, T, Hp: HyperPath<T>> {
    root: Hp,
    shift: S,
    normal: Vector3<S>,
    _t: PhantomData<T>,
    _s: PhantomData<S>,
}

impl<S, T, Hp> HyperPath<T> for ShiftedSideHyperPath<S, T, Hp>
where
    Self: Length<Scalar = S>,
    Hp: HyperPath<T> + Length<Scalar = S>,
    T: Tensor<Scalar = S>
        + Mul<S, Output = T>
        + Point<Vector = Vector3<S>>
        + SideDir
        + Length<Scalar = S>
        + Copy,
    S: nalgebra::Scalar
        + nalgebra::Field
        + nalgebra::ComplexField
        + Copy
        + From<u16>
        + One
        + Zero
        + Div<Output = S>
        + Pow<u16, Output = S>
        + Sub
        + Debug
        + Display,
{
    fn push_back(self, h: HyperLine<T>) -> Self {
        Self::new(self.root.push_back(h), self.shift, self.normal)
    }

    fn extend(self, h: impl IntoIterator<Item = HyperLine<T>>) -> Self {
        Self::new(self.root.extend(h), self.shift, self.normal)
    }

    fn push_front(self, h: HyperLine<T>) -> Self {
        Self::new(self.root.push_front(h), self.shift, self.normal)
    }

    fn head_tail(self) -> (HyperLine<T>, Self) {
        let (t, h) = self.root.head_tail();
        (
            t.shift_in_plane(self.normal, self.shift),
            Self::new(h, self.shift, self.normal),
        )
    }

    fn len(&self) -> usize {
        self.root.len()
    }

    fn connect_ends(&mut self) {
        todo!()
    }

    fn connect_ends_circular(&mut self) {
        todo!()
    }

    fn map<F, R>(self, map: F) -> super::hyper_path::Root<R>
    where
        F: Fn(HyperLine<T>) -> HyperLine<R>,
    {
        todo!()
    }
}

impl<S, T, Hp> Length for ShiftedSideHyperPath<S, T, Hp>
where
    Hp: HyperPath<T>,
    Hp: Length<Scalar = S>,
{
    type Scalar = S;

    fn length(&self) -> Self::Scalar {
        self.root.length()
    }
}

impl<S, T, Hp: HyperPath<T>> ShiftedSideHyperPath<S, T, Hp>
where
    T: SideDir,
{
    pub fn new(hp: Hp, shift: S, normal: Vector3<S>) -> Self {
        Self {
            root: hp,
            shift,
            normal,
            _t: PhantomData,
            _s: PhantomData,
        }
    }
}
