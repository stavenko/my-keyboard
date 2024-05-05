use std::ops::Div;

use nalgebra::Vector3;

use crate::{geometry::Geometry, parametric_iterator::ParametricIterator};

use super::line::GetT;

pub struct PrimitiveSurface<A, B>(pub(super) A, pub(super) B)
where
    A: GetT<Value = Vector3<<A as GetT>::Scalar>>,
    B: GetT<Scalar = A::Scalar, Value = A::Value>;

impl<A, B> Geometry for PrimitiveSurface<A, B>
where
    A: GetT<Value = Vector3<<A as GetT>::Scalar>>,
    B: GetT<Scalar = A::Scalar, Value = A::Value>,
    A::Value: Copy,
    A::Scalar: Div<A::Scalar, Output = A::Scalar> + From<u16> + Copy,
{
    fn polygonize(
        self,
        _index: &mut crate::indexes::geo_index::index::GeoIndex,
        complexity: usize,
    ) -> anyhow::Result<()> {
        for (t, tt) in ParametricIterator::<A::Scalar>::new(complexity) {
            let _a = self.0.get_t(t);
            let _b = self.0.get_t(tt);
            let _c = self.1.get_t(tt);
            let _d = self.1.get_t(t);
            println!("Created polygons - just need correct type ");
            //let p1 = Polygon::new(vec![a.into(), b.into(), c.into()])?;
            //let p2 = Polygon::new(vec![a.into(), c.into(), d.into()])?;
            //index.save_polygon(&p1);
            //index.save_polygon(&p2);
        }
        Ok(())
    }
}
