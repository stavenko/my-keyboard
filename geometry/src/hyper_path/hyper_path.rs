use std::{
    collections::VecDeque,
    ops::{Mul, Sub},
};

use num_traits::{One, Zero};

use super::{hyper_line::HyperLine, hyper_point::Tensor, length::Length};

pub trait IsLinear {
    fn is_linear(&self) -> bool;
}

#[derive(Clone)]
pub struct Root<Tensor> {
    items: VecDeque<HyperLine<Tensor>>,
}

impl<T> Default for Root<T> {
    fn default() -> Self {
        Self {
            items: Default::default(),
        }
    }
}

impl<Tensor> Root<Tensor> {
    pub fn new() -> Self {
        Self {
            items: VecDeque::new(),
        }
    }
}

#[allow(clippy::len_without_is_empty)]
pub trait HyperPath<T>: Sized + Length {
    fn push_back(self, h: HyperLine<T>) -> Self;

    fn connect_ends(&mut self);

    fn connect_ends_circular(&mut self);

    fn extend(self, h: impl IntoIterator<Item = HyperLine<T>>) -> Self;

    fn map<F, R>(self, map: F) -> Root<R>
    where
        F: Fn(HyperLine<T>) -> HyperLine<R>;

    fn push_front(self, h: HyperLine<T>) -> Self;

    fn head_tail(self) -> (HyperLine<T>, Self);

    fn len(&self) -> usize;
}

impl<T> Length for Root<T>
where
    T: Tensor,
    T: Length<Scalar = <T as Tensor>::Scalar>,
    <T as Length>::Scalar:
        Zero + From<u16> + nalgebra::Scalar + nalgebra::ComplexField + nalgebra::RealField,
    T: Mul<T, Output = T> + Mul<<T as Tensor>::Scalar, Output = T> + Sub<T, Output = T>,
{
    type Scalar = <T as Tensor>::Scalar;

    fn length(&self) -> Self::Scalar {
        self.items
            .iter()
            .map(|i| i.length())
            .fold(Self::Scalar::zero(), |a, f| a + f)
    }
}

impl<T> HyperPath<T> for Root<T>
where
    T: Tensor,
    T: Length<Scalar = <T as Tensor>::Scalar>,
    <T as Length>::Scalar:
        Zero + One + From<u16> + nalgebra::Scalar + nalgebra::ComplexField + nalgebra::RealField,
    T: Mul<T, Output = T> + Mul<<T as Tensor>::Scalar, Output = T> + Sub<T, Output = T>,
{
    fn push_back(mut self, h: HyperLine<T>) -> Self {
        self.items.push_back(h);
        self
    }

    fn map<F, R>(self, map: F) -> Root<R>
    where
        F: Fn(HyperLine<T>) -> HyperLine<R>,
    {
        Root {
            items: self.items.into_iter().map(map).collect(),
        }
    }

    fn extend(mut self, h: impl IntoIterator<Item = HyperLine<T>>) -> Self {
        self.items.extend(h);
        self
    }

    fn push_front(mut self, h: HyperLine<T>) -> Self {
        self.items.push_front(h);
        self
    }

    fn head_tail(mut self) -> (HyperLine<T>, Self) {
        let head = self.items.pop_front().expect("Unreachable");
        (head, self)
    }

    fn len(&self) -> usize {
        self.items.len()
    }

    fn connect_ends(&mut self) {
        let l = self.items.len() - 1;
        let half = <T as Tensor>::Scalar::one() / <T as Tensor>::Scalar::from(2);
        for cur in 0..l {
            let next = cur + 1;
            let f = *self.items[cur].0.last().expect("ok");
            let l = *self.items[next].0.first().expect("ok");
            let d = l - f;
            let d = d * half;
            let m = d + f;
            if let Some(last) = self.items[cur].0.last_mut() {
                *last = m;
            }
            if let Some(first) = self.items[next].0.first_mut() {
                *first = m;
            }
        }
    }

    fn connect_ends_circular(&mut self) {
        let l = self.items.len();
        let half = <T as Tensor>::Scalar::one() / <T as Tensor>::Scalar::from(2);
        for cur in 0..l {
            let next = (cur + 1) % l;
            let f = *self.items[cur].0.last().expect("ok");
            let l = *self.items[next].0.first().expect("ok");
            let d = l - f;
            let d = d * half;
            let m = d + f;
            if let Some(last) = self.items[cur].0.last_mut() {
                *last = m;
            }
            if let Some(first) = self.items[next].0.first_mut() {
                *first = m;
            }
        }
    }
}
