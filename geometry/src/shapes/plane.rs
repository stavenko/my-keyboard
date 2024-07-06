use itertools::Itertools;
use nalgebra::{ClosedAdd, SimdRealField, Vector3};
use num_traits::Zero;

use crate::{
    origin::{self, BaseOrigin},
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
    pub fn centered(origin: BaseOrigin<F>, width: F, height: F, resolution: usize) -> Self {
        Self {
            origin,
            width,
            height,
            resolution,
        }
    }

    pub fn render(self) -> Vec<Vec<Vector3<F>>> {
        let wf = self.origin.center - self.origin.x() * self.width / F::from(2u16);
        let hf = self.origin.center - self.origin.y() * self.height / F::from(2u16);
        let wt = wf + self.origin.x() * self.width;
        let ht = hf + self.origin.y() * self.height;

        ParametricIterator::new(self.resolution)
            .flat_map(|(s, ss)| {
                ParametricIterator::new(self.resolution).map(move |(t, tt)| {
                    let a = wf.lerp(&wt, s);
                    let b = wf.lerp(&wt, ss);
                    let c = hf.lerp(&ht, tt);
                    let d = hf.lerp(&ht, t);
                    vec![a, b, c, d]
                })
            })
            .collect_vec()
    }
}
