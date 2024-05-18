use std::ops::Div;

use nalgebra::Vector3;

use crate::{
    geometry::Geometry,
    hyper_path::{hyper_path::HyperPath, line::GetT},
    indexes::geo_index::index::GeoIndex,
};

use super::{
    simple_surface::{GetLineAt, SimpleSurface},
    surface::Surface,
};

pub struct SurfaceOfSameSize<A, B>(A, B);

impl<A, B> Geometry for SurfaceOfSameSize<A, B>
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

    A::Tail: HyperPath,
    B::Tail: HyperPath,

    <A::Tail as HyperPath>::Head: GetT,
    <B::Tail as HyperPath>::Head: GetT<Scalar = <<A::Tail as HyperPath>::Head as GetT>::Scalar>,
    <<A::Tail as HyperPath>::Head as GetT>::Scalar:
        Div<Output = <<A::Tail as HyperPath>::Head as GetT>::Scalar>,
{
    fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
        let (f, rest_f) = self.0.head_tail();
        let (s, rest_s) = self.1.head_tail();

        SimpleSurface(f, s).polygonize(index, complexity)?;
        if let Some((f, s)) = rest_f.and_then(|f| rest_s.map(|s| (f, s))) {
            Surface(f, s).polygonize(index, complexity)?;
        }

        Ok(())
    }
}
