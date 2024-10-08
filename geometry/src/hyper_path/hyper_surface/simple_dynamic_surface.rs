use std::{
    fmt::{Debug, Display},
    marker::PhantomData,
    ops::{Add, Div, Mul},
};

use crate::{
    decimal::Dec,
    geometry::Geometry,
    hyper_path::{
        hyper_line::HyperLine,
        hyper_path::IsLinear,
        hyper_point::{Point, SideDir, Tensor},
        line::GetT,
    },
    indexes::geo_index::mesh::MeshRefMut,
    parametric_iterator::ParametricIterator,
};
use nalgebra::Vector3;
use num_traits::Pow;

use super::primitive_dynamic_surface::PrimitiveSurface;

pub struct SimpleSurface<S, T>(pub HyperLine<T>, pub HyperLine<T>, PhantomData<S>);

impl<S, T> SimpleSurface<S, T> {
    pub fn new(l: HyperLine<T>, s: HyperLine<T>) -> Self {
        Self(l, s, PhantomData)
    }
}

impl<S, T> Geometry for SimpleSurface<S, T>
where
    S: Debug
        + Display
        + Div<Output = S>
        + From<u16>
        + Copy
        + Pow<u16, Output = S>
        + nalgebra::Field
        + nalgebra::Scalar
        + Into<Dec>,
    T: Tensor<Scalar = S>,
    T: Mul<S, Output = T>,
    T: SideDir<Vector = Vector3<S>> + Point<Vector = <T as SideDir>::Vector> + 'static,
    HyperLine<T>: IsLinear,
    <T as SideDir>::Vector: Add<<T as Point>::Vector, Output = <T as Point>::Vector>,
{
    fn polygonize(self, mesh: &mut MeshRefMut, complexity: usize) -> anyhow::Result<()> {
        if self.0.is_linear() && self.1.is_linear() {
            let l1 = self.get_line_at(S::zero());
            let l2 = self.get_line_at(S::one());
            PrimitiveSurface(l1, l2).polygonize(mesh, complexity)?;
        } else {
            for (t, tt) in ParametricIterator::<S>::new(complexity) {
                let l1 = self.get_line_at(t);
                let l2 = self.get_line_at(tt);
                PrimitiveSurface(l1, l2).polygonize(mesh, complexity)?;
            }
        }
        Ok(())
    }
}

impl<S, T> SimpleSurface<S, T>
where
    S: Debug
        + Display
        + Copy
        + From<u16>
        + Pow<u16, Output = S>
        + nalgebra::Field
        + nalgebra::Scalar,
    T: SideDir<Vector = Vector3<S>> + Point<Vector = <T as SideDir>::Vector>,
    T: Tensor<Scalar = S>,
    T: Mul<S, Output = T>,
    <T as SideDir>::Vector: Add<<T as Point>::Vector, Output = <T as Point>::Vector> + Debug,
{
    fn get_line_at(&self, t: S) -> impl GetT<Value = Vector3<S>, Scalar = S> + Debug + IsLinear {
        let f = self.0.get_t(t);
        let s = self.1.get_t(t);
        let a = f.point();
        let b = f.side_dir() + f.point();
        let c = s.side_dir() + s.point();
        let d = s.point();
        HyperLine::new_4(a, b, c, d)
    }
}
