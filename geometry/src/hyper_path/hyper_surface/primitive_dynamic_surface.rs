use std::ops::Div;

use nalgebra::Vector3;
use num_traits::{One, Zero};

use crate::{
    decimal::Dec,
    geometry::Geometry,
    hyper_path::{hyper_path::IsLinear, line::GetT},
    indexes::geo_index::mesh::MeshRefMut,
    parametric_iterator::ParametricIterator,
};

pub struct PrimitiveSurface<L1, L2>(pub L1, pub L2)
where
    L1: GetT<Value = Vector3<<L1 as GetT>::Scalar>> + IsLinear,
    L2: GetT<Scalar = L1::Scalar, Value = L1::Value> + IsLinear,

    L1::Scalar: Div<Output = L1::Scalar>
        + From<u16>
        + Copy
        + Into<Dec>
        + nalgebra::Scalar
        + nalgebra::Field;

impl<L1, L2> Geometry for PrimitiveSurface<L1, L2>
where
    L1: GetT<Value = Vector3<<L1 as GetT>::Scalar>> + IsLinear,
    L2: GetT<Scalar = L1::Scalar, Value = L1::Value> + IsLinear,

    L1::Scalar: Div<Output = L1::Scalar>
        + From<u16>
        + Copy
        + Into<Dec>
        + One
        + Zero
        + nalgebra::Scalar
        + nalgebra::Field,
{
    fn polygonize(self, mesh: &mut MeshRefMut, complexity: usize) -> anyhow::Result<()> {
        if self.0.is_linear() && self.1.is_linear() {
            let t = L1::Scalar::zero();
            let tt = L1::Scalar::one();
            let a = self.0.get_t(t);
            let b = self.0.get_t(tt);
            let c = self.1.get_t(tt);
            let d = self.1.get_t(t);
            mesh.add_polygon(&[a, b, c]).ok();
            mesh.add_polygon(&[a, c, d]).ok();
        } else {
            for (t, tt) in ParametricIterator::<L1::Scalar>::new(complexity) {
                let a = self.0.get_t(t);
                let b = self.0.get_t(tt);
                let c = self.1.get_t(tt);
                let d = self.1.get_t(t);
                mesh.add_polygon(&[a, b, c]).ok();
                mesh.add_polygon(&[a, c, d]).ok();
            }
        }
        Ok(())
    }
}
