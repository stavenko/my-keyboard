use std::{
    marker::PhantomData,
    ops::{Div, Mul},
};

use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::Zero;

use crate::{
    decimal::Dec,
    geometry::Geometry,
    hyper_path::{
        hyper_path::{HyperPath, IsLinear},
        hyper_point::{Point, Tensor},
        length::Length,
        line::GetT,
    },
    parametric_iterator::ParametricIterator,
};

pub struct PolygonFromLineInPlane<T, S, L1>(L1, bool, PhantomData<S>, PhantomData<T>);

impl<T, S, L1> PolygonFromLineInPlane<T, S, L1> {
    pub fn new(line: L1, inversed: bool) -> Self {
        Self(line, inversed, Default::default(), Default::default())
    }
}

impl<L1, S, T> Geometry for PolygonFromLineInPlane<T, S, L1>
where
    L1: HyperPath<T> + Length<Scalar = S>,
    T: Tensor<Scalar = S> + Mul<S, Output = T> + Point<Vector = Vector3<S>>,
    S: Zero
        + From<u16>
        + Div<S, Output = S>
        + Into<Dec>
        + Copy
        + nalgebra::Scalar
        + nalgebra::Field,
{
    fn polygonize(
        self,
        index: &mut crate::indexes::geo_index::index::GeoIndex,
        complexity: usize,
    ) -> anyhow::Result<()> {
        let Self(mut line1, inversed, _, _) = self;
        let mut vertices = Vec::new();
        loop {
            if line1.len() == 0 {
                break;
            }
            let (f, fs) = line1.head_tail();
            if f.is_linear() {
                let v = f.get_t(S::zero());
                vertices.push(v);
            } else {
                vertices.extend(
                    ParametricIterator::<L1::Scalar>::new(complexity).map(|(t, _)| f.get_t(t)),
                );
            }
            line1 = fs;
        }
        if inversed {
            index.save_as_polygon(
                &vertices.into_iter().map(|t| t.point()).rev().collect_vec(),
                None,
            )?;
        } else {
            index.save_as_polygon(&vertices.into_iter().map(|t| t.point()).collect_vec(), None)?;
        }
        Ok(())
    }
}
