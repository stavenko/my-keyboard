use core::fmt;
use std::{
    fmt::Debug,
    fmt::Display,
    marker::PhantomData,
    ops::{Add, Div, Mul, Sub},
};

use nalgebra::{RealField, Scalar, Vector3};
use num_traits::{One, Pow, Zero};

use crate::{
    decimal::Dec,
    geometry::Geometry,
    hyper_path::{
        hyper_path::HyperPath,
        hyper_point::{Point, SideDir, Tensor},
        length::Length,
        split_hyper_line::SplitHyperLine,
    },
    indexes::geo_index::index::GeoIndex,
};

use super::simple_dynamic_surface::SimpleSurface;

pub struct DynamicSurface<S, T, L1, L2>(
    pub(super) L1,
    pub(super) L2,
    PhantomData<S>,
    PhantomData<T>,
);

impl<S, T, L1, L2> DynamicSurface<S, T, L1, L2>
where
//L1: HyperPath<T>,
//L2: HyperPath<T>,
{
    pub fn new(l: L1, p: L2) -> Self {
        Self(l, p, Default::default(), Default::default())
    }
}

impl<S, T, L1, L2> Geometry for DynamicSurface<S, T, L1, L2>
where
    L1: HyperPath<T> + Length<Scalar = S>,
    L2: HyperPath<T> + Length<Scalar = S>,
    S: nalgebra::Scalar
        + nalgebra::Field
        + nalgebra::ComplexField<RealField = S>
        + Copy
        + From<u16>
        + One
        + Zero
        + Div<Output = S>
        + Pow<u16, Output = S>
        + Sub
        + PartialOrd<S>
        + Debug
        + Into<Dec>
        + Display,

    T: Tensor<Scalar = S>
        + Length<Scalar = S>
        + Mul<<T as Tensor>::Scalar, Output = T>
        + Point<Vector = <T as SideDir>::Vector>
        + SideDir<Vector = Vector3<S>>
        + Sub<T, Output = T>
        + fmt::Debug
        + Copy
        + 'static,
    <T as SideDir>::Vector: Add<<T as Point>::Vector, Output = <T as Point>::Vector>,
    <T as Sub<T>>::Output: Length<Scalar = <T as Tensor>::Scalar>,
{
    fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
        //println!("lengths: {}, {}", self.0.len(), self.1.len());
        if self.0.len() == self.1.len() && self.0.len() == 1 {
            let (f, _) = self.0.head_tail();
            let (s, _) = self.1.head_tail();
            SimpleSurface::new(f, s).polygonize(index, complexity)?;
        } else if self.0.len() == self.1.len() {
            let (f, ft) = self.0.head_tail();
            let (s, st) = self.1.head_tail();
            //println!("simple {f:?} {s:?}");
            SimpleSurface::new(f, s).polygonize(index, complexity)?;
            DynamicSurface::new(ft, st).polygonize(index, complexity)?;
        } else if self.0.len() != self.1.len() {
            //println!("wtf1");

            let len_1 = self.0.length();
            let len_2 = self.1.length();
            //println!(">> {}, {}", self.0.len(), self.1.len());
            //println!(">> {len_1}, {len_2}");

            let (f, mut f_tail) = self.0.head_tail();
            let (s, mut s_tail) = self.1.head_tail();
            //println!("     --{}--    -- {} ---", f.length(), s.length());
            let ft_on_second = f.length() / len_2;
            let ft_on_first = f.length() / len_1;
            let st_on_second = s.length() / len_2;
            let st_on_first = s.length() / len_1;
            //println!(" first is {ft_on_first} for first line and {ft_on_second} of second line");
            //println!(" second is {st_on_first} for first line and {st_on_second} of second line");

            if f.length().is_zero() || s.length().is_zero() {
                println!("What do we gonna do with zero length");
                println!("\n{f:?}\n{s:?}");

                panic!();
            }

            let ps = f.length() / s.length();
            let pf = s.length() / f.length();

            let is_same = (ps - pf).abs() < (S::one() / S::from(65535));
            //println!("is same {is_same}");

            if s_tail.len() == 0 {
                let p = S::from(1) / S::from((f_tail.len() + 1) as u16);
                //println!("split {p} because second tail is empty");
                let (ss, s) = s.split_hyper_line(p);
                SimpleSurface::new(f, ss).polygonize(index, complexity)?;
                s_tail = s_tail.push_front(s);
            } else if f_tail.len() == 0 {
                let p = S::from(1) / S::from((s_tail.len() + 1) as u16);
                //println!("split {p} because first tail is empty");
                let (ff, f) = f.split_hyper_line(p);
                SimpleSurface::new(ff, s).polygonize(index, complexity)?;
                f_tail = f_tail.push_front(f);
            } else if ft_on_second < st_on_second && !is_same {
                // let p = f.length() / s.length();
                //println!("split second at {ps} because: {ft_on_second} {st_on_second} ");
                let (ss, s) = s.split_hyper_line(ps);
                SimpleSurface::new(f, ss).polygonize(index, complexity)?;

                s_tail = s_tail.push_front(s);
            } else if st_on_first < ft_on_first && !is_same {
                //println!("split first at {pf}  because: {st_on_first} {ft_on_first}");

                let (ff, f) = f.split_hyper_line(pf);
                SimpleSurface::new(ff, s).polygonize(index, complexity)?;

                f_tail = f_tail.push_front(f);
            } else {
                //println!("Same");
                SimpleSurface::new(f, s).polygonize(index, complexity)?;
            }
            DynamicSurface::new(f_tail, s_tail).polygonize(index, complexity)?;
        }
        Ok(())
    }
}
