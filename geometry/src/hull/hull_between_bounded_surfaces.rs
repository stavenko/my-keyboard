use std::marker::PhantomData;

use crate::{
    geometry::Geometry,
    planar::polygon::Polygon,
    surface::{
        topology::{Four, Three, Topology},
        GetBoundingPath, Surface, SurfaceBetweenTwoPaths,
    },
};

/// Generic hull between two surfaces.
/// Expected, that surfaces have following properties:
/// 1. They have no common points
/// 2. Have same orientation: from one point of view, one can see, that left of one surface is the
///    left of another, and same for top, bottom, right.
///
pub struct HullBetweenSurfaces<T, A, B>
where
    A: Surface<T>,
    B: Surface<T>,
    T: Topology,
{
    upper: A,
    lower: B,
    _p: PhantomData<T>,
}

impl<A, B> HullBetweenSurfaces<Four, A, B>
where
    A: Surface<Four>,
    B: Surface<Four>,
    A: GetBoundingPath<0>,
    B: GetBoundingPath<0>,
    A: GetBoundingPath<1>,
    B: GetBoundingPath<1>,
    A: GetBoundingPath<2>,
    B: GetBoundingPath<2>,
    A: GetBoundingPath<3>,
    B: GetBoundingPath<3>,
{
    pub fn polygonize_sides(&self) -> anyhow::Result<[Vec<Polygon>; 4]> {
        Ok([
            Geometry::polygonize(&self.get_side_surface_0())?,
            Geometry::polygonize(&self.get_side_surface_1())?
                .into_iter()
                .map(|f| f.flip())
                .collect(),
            Geometry::polygonize(&self.get_side_surface_2())?
                .into_iter()
                .map(|f| f.flip())
                .collect(),
            Geometry::polygonize(&self.get_side_surface_3())?,
        ])
    }
}

impl<A, B> HullBetweenSurfaces<Three, A, B>
where
    A: Surface<Three>,
    B: Surface<Three>,
    A: GetBoundingPath<0>,
    B: GetBoundingPath<0>,
    A: GetBoundingPath<1>,
    B: GetBoundingPath<1>,
    A: GetBoundingPath<2>,
    B: GetBoundingPath<2>,
{
    pub fn polygonize_sides(&self) -> anyhow::Result<[Vec<Polygon>; 3]> {
        Ok([
            Geometry::polygonize(&self.get_side_surface_0())?
                .into_iter()
                .map(|f| f.flip())
                .collect(),
            Geometry::polygonize(&self.get_side_surface_1())?,
            Geometry::polygonize(&self.get_side_surface_2())?
                .into_iter()
                .map(|f| f.flip())
                .collect(),
        ])
    }
}

impl<T, A, B> HullBetweenSurfaces<T, A, B>
where
    A: Surface<T>,
    B: Surface<T>,
    T: Topology,
    A: GetBoundingPath<0>,
    B: GetBoundingPath<0>,
{
    fn get_side_surface_0(&self) -> impl Geometry + '_ {
        let p1 = <A as GetBoundingPath<0>>::get_bounding_path(&self.upper);
        let p2 = <B as GetBoundingPath<0>>::get_bounding_path(&self.lower);
        SurfaceBetweenTwoPaths::<_, _, Four>::new(p1, p2)
    }
}

impl<T, A, B> HullBetweenSurfaces<T, A, B>
where
    A: Surface<T>,
    B: Surface<T>,
    T: Topology,
    A: GetBoundingPath<1>,
    B: GetBoundingPath<1>,
{
    fn get_side_surface_1(&self) -> impl Geometry + '_ {
        let p1 = <A as GetBoundingPath<1>>::get_bounding_path(&self.upper);
        let p2 = <B as GetBoundingPath<1>>::get_bounding_path(&self.lower);
        SurfaceBetweenTwoPaths::<_, _, Four>::new(p1, p2)
    }
}

impl<T, A, B> HullBetweenSurfaces<T, A, B>
where
    A: Surface<T>,
    B: Surface<T>,
    T: Topology,
    A: GetBoundingPath<2>,
    B: GetBoundingPath<2>,
{
    fn get_side_surface_2(&self) -> impl Geometry + '_ {
        let p1 = <A as GetBoundingPath<2>>::get_bounding_path(&self.upper);
        let p2 = <B as GetBoundingPath<2>>::get_bounding_path(&self.lower);
        SurfaceBetweenTwoPaths::<_, _, Four>::new(p1, p2)
    }
}
impl<T, A, B> HullBetweenSurfaces<T, A, B>
where
    A: Surface<T>,
    B: Surface<T>,
    T: Topology,
    A: GetBoundingPath<3>,
    B: GetBoundingPath<3>,
{
    fn get_side_surface_3(&self) -> impl Geometry + '_ {
        let p1 = <A as GetBoundingPath<3>>::get_bounding_path(&self.upper);
        let p2 = <B as GetBoundingPath<3>>::get_bounding_path(&self.lower);
        SurfaceBetweenTwoPaths::<_, _, Four>::new(p1, p2)
    }
}

impl<T, A, B> HullBetweenSurfaces<T, A, B>
where
    A: Surface<T>,
    B: Surface<T>,
    T: Topology,
{
    pub fn new(upper: A, lower: B) -> Self {
        Self {
            upper,
            lower,
            _p: PhantomData,
        }
    }
}

impl<A, B> Geometry for HullBetweenSurfaces<Three, A, B>
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
    fn polygonize(&self) -> anyhow::Result<Vec<Polygon>> {
        let upper = Geometry::polygonize(&self.upper)?;
        let lower = Geometry::polygonize(&self.lower)?
            .into_iter()
            .map(|f| f.flip())
            .collect();
        let sides = self.polygonize_sides()?;

        Ok([[upper, lower].as_slice(), &sides]
            .concat() //, right, left, top, bottom]
            .into_iter()
            .flatten()
            .collect())
    }
}

impl<A, B> Geometry for HullBetweenSurfaces<Four, A, B>
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
    fn polygonize(&self) -> anyhow::Result<Vec<Polygon>> {
        let upper = Geometry::polygonize(&self.upper)?;
        let lower = Geometry::polygonize(&self.lower)?;
        let upper = upper.into_iter().map(|f| f.flip()).collect();
        let [top, bottom, right, left] = self.polygonize_sides()?;

        Ok([[upper, lower].as_slice(), &[top, bottom, left, right]]
            .concat() // right, left, top, bottom]
            .into_iter()
            .flatten()
            .collect())
    }
}
