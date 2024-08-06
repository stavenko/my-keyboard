use itertools::Itertools;
use nalgebra::{ClosedAdd, SimdRealField, Vector3};
use num_traits::Zero;

use crate::{
    decimal::Dec, geometry::GeometryDyn, indexes::geo_index::index, origin::BaseOrigin,
    parametric_iterator::ParametricIterator,
};

pub struct Plane<F> {
    origin: BaseOrigin<F>,
    width: F,
    height: F,
    resolution: usize,
}

impl<F> Plane<F>
where
    F: Zero + Copy,
    F: nalgebra::Scalar + nalgebra::SimdRealField + ClosedAdd + nalgebra::SimdRealField,
    F::Element: SimdRealField,
    F: From<usize>,
    F: From<u16>,
{
    pub fn centered(
        origin: BaseOrigin<F>,
        width: impl Into<F>,
        height: impl Into<F>,
        resolution: usize,
    ) -> Self {
        Self {
            origin,
            width: width.into(),
            height: height.into(),
            resolution,
        }
    }

    pub fn render(&self) -> Vec<Vec<Vector3<F>>> {
        let wf = self.origin.center
            - self.origin.x() * self.width / F::from(2u16)
            - self.origin.y() * self.height / F::from(2u16);

        ParametricIterator::<F>::new(self.resolution)
            .flat_map(|(s, ss)| {
                ParametricIterator::<F>::new(self.resolution).map(move |(t, tt)| {
                    let ws: Vector3<F> = self.origin.x() * self.width * s;
                    let wss: Vector3<F> = self.origin.x() * self.width * ss;

                    let ht: Vector3<F> = self.origin.y() * self.width * t;
                    let htt: Vector3<F> = self.origin.y() * self.width * tt;
                    let a = wf + ws + ht;
                    let b = wf + wss + ht;
                    let c = wf + wss + htt;
                    let d = wf + ws + htt;
                    dbg!(a, b, c, d);
                    vec![a, b, c, d]
                })
            })
            .collect_vec()
    }
}

impl<F> GeometryDyn for Plane<F>
where
    F: Into<Dec>
        + nalgebra::Scalar
        + nalgebra::Field
        + nalgebra::RealField
        + Copy
        + From<usize>
        + From<u16>,
{
    fn polygonize(&self, index: &mut index::GeoIndex, _complexity: usize) -> anyhow::Result<()> {
        for p in self.render() {
            index.save_as_polygon(&p, None)?;
        }

        Ok(())
    }
}
