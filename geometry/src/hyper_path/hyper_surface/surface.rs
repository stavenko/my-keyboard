use std::{ops::Div, process::Output};

use nalgebra::{Vector, Vector3};

use crate::{
    geometry::Geometry,
    hyper_path::{hyper_path::HyperPath, line::GetT},
    indexes::geo_index::index::GeoIndex,
};

use super::simple_surface::{GetLineAt, SimpleSurface};

pub struct Surface<A, B>(pub(super) A, pub(super) B);

impl<A, B> Geometry for Surface<A, B>
where
    A: HyperPath,
    B: HyperPath,
    A::Head: GetT,
    B::Head: GetT<Scalar = <A::Head as GetT>::Scalar>,
    <A::Head as GetT>::Scalar:
        Div<<A::Head as GetT>::Scalar, Output = <A::Head as GetT>::Scalar> + From<u16> + Copy,
    SimpleSurface<A::Head, B::Head>: GetLineAt<Scalar = <A::Head as GetT>::Scalar>,
    <SimpleSurface<A::Head, B::Head> as GetLineAt>::Line:
        GetT<Value = Vector3<<A::Head as GetT>::Scalar>, Scalar = <A::Head as GetT>::Scalar>,
{
    fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
        if self.0.len() == self.1.len() && self.0.len() == 1 {
            let (f, _) = self.0.head_tail();
            let (s, _) = self.1.head_tail();
            SimpleSurface(f, s).polygonize(index, complexity)?;
        } else if self.0.len() == self.1.len() {
            //HyperSurfaceSameSize(self.0, self.1).polygonize(index, complexity)?;
        } else if self.0.len() > self.1.len() {
            //HyperSurfaceDifferentSize(self.0, self.1).polygonize(index, complexity)?;
        } else {
            //HyperSurfaceDifferentSize(self.1, self.0).polygonize(index, complexity)?;
        }
        Ok(())
    }
}
