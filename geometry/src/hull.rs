use std::marker::PhantomData;

use crate::{geometry::Geometry, planar::polygon::Polygon};

use self::hull_between_bounded_surfaces::HullBetweenSurfaces;

use super::surface::{
    topology::{Four, Three, Topology},
    GetBoundingPath, Surface,
};

pub mod hull_between_bounded_surfaces;

pub struct Hull<T: Topology> {
    pub sides: Vec<Vec<Polygon>>,
    pub inner: Vec<Polygon>,
    pub outer: Vec<Polygon>,
    ph: PhantomData<T>,
}

impl Hull<Four> {
    pub fn drop_left(&mut self) {
        if let Some(v) = self.sides.get_mut(2) {
            v.clear();
        }
    }

    pub fn drop_right(&mut self) {
        if let Some(v) = self.sides.get_mut(3) {
            v.clear();
        }
    }
}

impl<A, B> TryFrom<(A, B)> for Hull<Four>
where
    A: Surface<Four>,
    B: Surface<Four>,
    A: Geometry,
    B: Geometry,
    A: GetBoundingPath<0>,
    A: GetBoundingPath<1>,
    A: GetBoundingPath<2>,
    A: GetBoundingPath<3>,
    B: GetBoundingPath<0>,
    B: GetBoundingPath<1>,
    B: GetBoundingPath<2>,
    B: GetBoundingPath<3>,
{
    type Error = anyhow::Error;

    fn try_from(value: (A, B)) -> Result<Self, Self::Error> {
        let inner = Geometry::polygonize(&value.0)?;
        let outer = Geometry::polygonize(&value.1)?;
        let inner = inner.into_iter().map(|f| f.flip()).collect();
        let h = HullBetweenSurfaces::new(value.0, value.1);
        let sides = h.polygonize_sides()?.to_vec();

        Ok(Self {
            inner,
            outer,
            sides,
            ph: PhantomData,
        })
    }
}

impl<A, B> TryFrom<(A, B)> for Hull<Three>
where
    A: Surface<Three>,
    B: Surface<Three>,
    A: Geometry,
    B: Geometry,
    A: GetBoundingPath<0>,
    A: GetBoundingPath<1>,
    A: GetBoundingPath<2>,
    B: GetBoundingPath<0>,
    B: GetBoundingPath<1>,
    B: GetBoundingPath<2>,
{
    type Error = anyhow::Error;

    fn try_from(value: (A, B)) -> Result<Self, Self::Error> {
        let outer = Geometry::polygonize(&value.1)?;
        let inner = Geometry::polygonize(&value.0)?;
        let inner = inner.into_iter().map(|f| f.flip()).collect();
        let h = HullBetweenSurfaces::new(value.1, value.0);
        let sides = h.polygonize_sides()?.to_vec();

        Ok(Self {
            inner,
            outer,
            sides,
            ph: PhantomData,
        })
    }
}
impl<T: Topology> Geometry for Hull<T> {
    fn polygonize(&self) -> anyhow::Result<Vec<Polygon>> {
        Ok(self
            .sides
            .clone()
            .into_iter()
            .flatten()
            .chain(self.inner.clone())
            .chain(self.outer.clone())
            .collect::<Vec<_>>())
    }
}
