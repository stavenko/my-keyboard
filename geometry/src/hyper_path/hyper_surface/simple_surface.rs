use std::ops::{Add, Div};

use nalgebra::Vector3;

use crate::{
    geometry::Geometry,
    hyper_path::{
        hyper_line::HyperLine,
        hyper_point::{Point, SideDir},
        line::GetT,
    },
    parametric_iterator::ParametricIterator,
};

use super::primitive_surface::PrimitiveSurface;

pub trait GetLineAt {
    type Line;
    type Scalar;
    fn get_line_at(&self, t: Self::Scalar) -> Self::Line;
}

pub struct SimpleSurface<A, B>(pub(super) A, pub(super) B);

impl<A, B> Geometry for SimpleSurface<A, B>
where
    Self: GetLineAt,
    <Self as GetLineAt>::Scalar:
        Div<<Self as GetLineAt>::Scalar, Output = <Self as GetLineAt>::Scalar> + From<u16> + Copy,
    <Self as GetLineAt>::Line:
        GetT<Scalar = <Self as GetLineAt>::Scalar, Value = Vector3<<Self as GetLineAt>::Scalar>>,
    <<Self as GetLineAt>::Line as GetT>::Value: Copy,
{
    fn polygonize(
        self,
        index: &mut crate::indexes::geo_index::index::GeoIndex,
        complexity: usize,
    ) -> anyhow::Result<()> {
        for (t, tt) in ParametricIterator::<<Self as GetLineAt>::Scalar>::new(complexity) {
            let l1 = self.get_line_at(t);
            let l2 = self.get_line_at(tt);
            PrimitiveSurface(l1, l2).polygonize(index, complexity)?
        }
        Ok(())
    }
}

impl<A, B> GetLineAt for SimpleSurface<A, B>
where
    A: GetT,
    B: GetT<Scalar = A::Scalar, Value = A::Value>,
    A::Scalar: Copy,
    A::Value: SideDir + Point,
    <A::Value as SideDir>::Vector:
        Add<<A::Value as Point>::Vector, Output = <A::Value as Point>::Vector>,
{
    type Line = HyperLine<4, <A::Value as Point>::Vector>;
    type Scalar = A::Scalar;

    fn get_line_at(&self, t: Self::Scalar) -> Self::Line {
        let f = self.0.get_t(t);
        let s = self.1.get_t(t);
        let a = f.point();
        let b = f.side_dir() + f.point();
        let c = s.side_dir() + s.point();
        let d = s.point();
        HyperLine::new_4(a, b, c, d)
    }
}
