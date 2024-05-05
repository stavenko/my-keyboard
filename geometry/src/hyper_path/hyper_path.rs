use std::{
    cell::RefCell,
    ops::{Add, Div},
    rc::Rc,
};

use nalgebra::Scalar;
use num_traits::Zero;

use super::{length::Length, split_hyper_line::SplitHyperLine};

pub struct HyperPath<F, O, Scalar> {
    lengths: Rc<RefCell<Vec<Scalar>>>,
    first: F,
    other: Option<O>,
    len: usize,
}

/*
pub fn zip_one<A, D, T, F>(a: A, d: D, with: impl )
where
    A: HeadTail,
    D: HeadTail,
    F:Fn(B, E) -> T) -> (T, Option<(C, F)>
{
    let (b, c) = a.head_tail();
    let (e, f) = d.head_tail();
    let tail = c.and_then(|c| f.map(|f| (c, f)));
    let head = with(b, e);
    (head, tail)
}
*/

pub trait HeadTail {
    type Head;
    type Tail;

    fn head_tail(self) -> (Self::Head, Option<Self::Tail>);
}

pub trait Zip {
    fn zip<T, A, B>(self, with: impl Fn(A, B) -> T) -> Vec<T>;
}

impl<A, Scalar> HyperPath<A, (), Scalar>
where
    A: Length<Scalar = Scalar>,
{
    pub fn new(item: A) -> Self {
        let length = item.length();
        Self {
            lengths: Rc::new(RefCell::new(vec![length])),
            first: item,
            other: None,
            len: 1,
        }
    }
}

impl<H, T, Scalar> HeadTail for HyperPath<H, T, Scalar> {
    fn head_tail(self) -> (H, Option<T>) {
        (self.first, self.other)
    }

    type Head = H;

    type Tail = T;
}

impl HeadTail for () {
    fn head_tail(self) -> (Self::Head, Option<Self::Tail>) {
        unreachable!("Head tail fo$ ()")
    }

    type Head = ();

    type Tail = ();
}

impl<A, B, Scalar> HyperPath<A, B, Scalar> {
    pub fn len(&self) -> usize {
        self.len
    }
}

impl<A, B, Scalar> HyperPath<A, B, Scalar>
where
    Scalar: Div<Scalar, Output = Scalar> + Add<Scalar, Output = Scalar> + Zero + Copy,
{
    pub fn push<D>(self, item: D) -> HyperPath<D, Self, Scalar>
    where
        D: Length<Scalar = Scalar>,
    {
        let length = item.length();

        let lengths = self.lengths.clone();
        {
            let mut lengths_ref = lengths.borrow_mut();
            lengths_ref.push(length);
        }
        HyperPath {
            lengths,
            first: item,
            len: self.len + 1,
            other: Some(self),
        }
    }

    /*
    pub fn length(&self) -> Scalar {
        let ls = self.lengths.borrow();
        ls.iter().fold(Scalar::zero(), |acc, l| acc + *l)
    }

    pub fn length_params(&self) -> Vec<Scalar> {
        let l = self.length();
        let ls = self.lengths.borrow();
        ls.iter().map(|ol| *ol / l).collect()
    }
    */

    pub fn head(&self) -> &A {
        &self.first
    }
}

impl<A, B, C, Scalar> HyperPath<A, HyperPath<B, C, Scalar>, Scalar> {
    pub fn tail(self) -> Option<HyperPath<B, C, Scalar>> {
        self.other
    }
}

impl<A, B, C, Scalar> HyperPath<A, HyperPath<B, C, Scalar>, Scalar>
where
    A: SplitHyperLine<Scalar>,
    A: Length<Scalar = Scalar>,
{
    pub fn split(
        mut self,
        t: Scalar,
    ) -> HyperPath<A, HyperPath<A, HyperPath<B, C, Scalar>, Scalar>, Scalar> {
        let (aa, ab) = self.first.split_hyper_line(t);
        self.first = ab;
        let lengths = self.lengths.clone();
        {
            let mut lengths_ref = lengths.borrow_mut();
            *lengths_ref.last_mut().expect("Must be at least non-zero") = self.first.length();
            lengths_ref.push(aa.length());
        }
        let len = self.len + 1;
        HyperPath {
            lengths,
            first: aa,
            other: Some(self),
            len,
        }
    }
}

impl<A, B, Scalar> Length for HyperPath<A, B, Scalar>
where
    A: Length,
    B: Length<Scalar = A::Scalar>,
    A::Scalar: Add<Output = A::Scalar>,
    A::Scalar: Zero,
{
    type Scalar = A::Scalar;
    fn length(&self) -> Self::Scalar {
        self.first.length()
            + self
                .other
                .as_ref()
                .map(|b| b.length())
                .unwrap_or(Zero::zero())
    }
}

impl<A, Scalar> Length for HyperPath<A, (), Scalar>
where
    A: Length,
{
    type Scalar = A::Scalar;

    fn length(&self) -> Self::Scalar {
        self.first.length()
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector1, Vector2, Vector3};

    use crate::{decimal::Dec, hyper_path::hyper_line::HyperLine};

    use super::{HeadTail, HyperPath};

    #[test]
    fn test_norm_length() {
        let a: Vector3<Dec> = Vector3::new(1.into(), 1.into(), 1.into());

        let norm = a.norm();
        let length = a.magnitude();
        assert_eq!(norm, length);
    }
    /*

    #[test]
    fn simple_builder() {
        let l1 = HyperLine::new_2(Vector1::new(Dec::from(2)), Vector1::new(Dec::from(3)));

        let hp = HyperPath::new(l1);
        let l2 = HyperLine::new_4(
            Vector2::new(Dec::from(2f32), 1.0.into()),
            Vector2::new(3.0.into(), 1.0.into()),
            Vector2::new(9.0.into(), 1.0.into()),
            Vector2::new(100.0.into(), 1.0.into()),
        );
        let hp2 = hp.push(l2.clone());
        let l3 = HyperLine::new_2(
            Vector3::new(0.0f64.into(), 0.0.into(), 0.5.into()),
            Vector3::new(0.0f64.into(), 1.0.into(), 0.5.into()),
        );
        let hp3 = hp2.push(l3.clone());

        let (f2, hp) = hp3.head_tail();
        assert_eq!(f2, l3);

        let (i4, hp) = hp.unwrap().head_tail();
        assert_eq!(i4, l2);

        //let (i2, hp) = hp.unwrap().head_tail();
    }
    */
}
