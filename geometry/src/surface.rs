use std::marker::PhantomData;

use anyhow::anyhow;
use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::{One, Zero};

use crate::{geometry::Geometry, planar::polygon::Polygon};

use self::topology::{Four, Topology};

use super::{
    decimal::Dec,
    path::{bezier::BezierEdge, segment::EdgeSegment, Path},
};

pub mod topology;
pub mod tri_bezier;

pub trait Surface<T: Topology> {
    fn get_point(&self, coords: T::ParametricCoords) -> anyhow::Result<Vector3<Dec>>;
    fn get_curve_at_param(&self, param: Dec) -> impl Path;
}

pub trait GetBoundingPath<const NUM: usize> {
    fn get_bounding_path(&self) -> impl Path;
}

pub struct SurfaceBetweenTwoPaths<P: Path, S: Path, T: Topology>(P, S, PhantomData<T>);

impl<P: Path, S: Path, T: Topology> SurfaceBetweenTwoPaths<P, S, T> {
    pub fn new(p: P, s: S) -> Self {
        Self(p, s, PhantomData)
    }
}
pub struct SurfaceBetweenTwoEdgePaths<P: Path, S: Path, T: Topology>(P, S, PhantomData<T>);

impl<P: Path, S: Path> Geometry for SurfaceBetweenTwoEdgePaths<P, S, Four> {
    fn polygonize(&self) -> anyhow::Result<Vec<Polygon>> {
        let faces = Four::parametric_face_iterator()
            .map(|pf| match pf.map(|p| self.get_point(p)) {
                [Ok(a), Ok(b), Ok(c)] => Polygon::new(vec![a, b, c]),
                _ => Err(anyhow!("failed to create faces")),
            })
            .try_collect::<_, Vec<_>, _>()?;
        Ok(faces)
    }
}

impl<P: Path, S: Path> Geometry for SurfaceBetweenTwoPaths<P, S, Four> {
    fn polygonize(&self) -> anyhow::Result<Vec<Polygon>> {
        let faces = Four::parametric_face_iterator_s()
            .map(|pf| match pf.map(|p| self.get_point(p)) {
                [Ok(a), Ok(b), Ok(c)] => Polygon::new(vec![a, b, c]),
                _err => Err(anyhow!("failed to create faces")),
            })
            .try_collect::<_, Vec<_>, _>()?;
        Ok(faces)
    }
}

impl<P: Path, S: Path> Surface<Four> for SurfaceBetweenTwoPaths<P, S, Four> {
    fn get_point(
        &self,
        coords: <Four as Topology>::ParametricCoords,
    ) -> anyhow::Result<Vector3<Dec>> {
        let p = self.get_curve_at_param(coords.y);
        Ok(p.get_t(coords.x))
    }

    fn get_curve_at_param(&self, s: Dec) -> impl Path {
        let from = self.0.get_t(s);
        let to = self.1.get_t(s);
        let edge_from = (from - to).normalize();
        let edge_to = -edge_from;
        EdgeSegment {
            from,
            to,
            edge_from,
            edge_to,
        }
    }
}
impl<P: Path, S: Path> Surface<Four> for SurfaceBetweenTwoEdgePaths<P, S, Four> {
    fn get_point(
        &self,
        coords: <Four as Topology>::ParametricCoords,
    ) -> anyhow::Result<Vector3<Dec>> {
        let p = self.get_curve_at_param(coords.y);
        Ok(p.get_t(coords.x))
    }

    fn get_curve_at_param(&self, s: Dec) -> impl Path {
        let left = self.0.get_t(s);
        let left_w = self.0.get_edge_dir(s) + left;
        let right = self.1.get_t(s);
        let right_w = self.1.get_edge_dir(s) + right;
        let b = self.0.get_t(Dec::zero());
        let e = self.0.get_t(Dec::one());
        let dir_left = (e - b).normalize();
        let b = self.1.get_t(Dec::zero());
        let e = self.1.get_t(Dec::one());
        let dir_right = (e - b).normalize();
        let dir1 = dir_left.lerp(&dir_right, Dec::one() / 3).normalize();
        let dir2 = dir_left.lerp(&dir_right, 2 * Dec::one() / 3).normalize();

        BezierEdge::new(
            [left, left_w, right_w, right],
            [dir_left, dir1, dir2, dir_right],
        )
    }
}

impl<P: Path + Clone, S: Path, T: Topology> GetBoundingPath<0>
    for SurfaceBetweenTwoEdgePaths<P, S, T>
{
    fn get_bounding_path(&self) -> impl Path {
        self.0.clone()
    }
}
impl<P: Path, S: Path + Clone, T: Topology> GetBoundingPath<1>
    for SurfaceBetweenTwoEdgePaths<P, S, T>
{
    fn get_bounding_path(&self) -> impl Path {
        self.1.clone()
    }
}
impl<P: Path, S: Path> GetBoundingPath<2> for SurfaceBetweenTwoEdgePaths<P, S, Four> {
    fn get_bounding_path(&self) -> impl Path {
        self.get_curve_at_param(Dec::zero())
    }
}
impl<P: Path, S: Path> GetBoundingPath<3> for SurfaceBetweenTwoEdgePaths<P, S, Four> {
    fn get_bounding_path(&self) -> impl Path {
        self.get_curve_at_param(Dec::one())
    }
}

impl<P: Path, S: Path, T: Topology> SurfaceBetweenTwoEdgePaths<P, S, T> {
    pub fn new(p: P, s: S) -> Self {
        Self(p, s, PhantomData)
    }

    pub fn get_curve_with_edge_force(&self, s: Dec, edge_force: Dec) -> BezierEdge {
        let left = self.0.get_t(s);
        let left_w = self.0.get_edge_dir(s) + left;
        let right = self.1.get_t(s);
        let right_w = self.1.get_edge_dir(s) + right;
        let b = self.0.get_t(Dec::zero());
        let e = self.0.get_t(Dec::one());
        let dir_left = (b - e).normalize();
        let b = self.1.get_t(Dec::zero());
        let e = self.1.get_t(Dec::one());
        let dir_right = (b - e).normalize();
        let dir1 = dir_left.lerp(&dir_right, Dec::one() / 3).normalize() * edge_force;
        let dir2 = dir_left.lerp(&dir_right, 2 * Dec::one() / 3).normalize() * edge_force;

        BezierEdge::new(
            [left, left_w, right_w, right],
            [dir_left * edge_force, dir1, dir2, dir_right * edge_force],
        )
    }
}
pub trait InverseSurface {
    type Inverted;
    fn inverse_surface(self) -> Self::Inverted;
}

/*
   pub mod bounded_surface;
   pub mod surface_between_two_bezier;

   pub trait GetBoundingPath<const D: usize> {
   type Path: Path;
   fn get_bounding_path(&self) -> Self::Path;
   }

   pub struct Four;

   impl Topology for Four {
   type ParametricCoords = Vector2<Dec>;

   type ParametricDims = Vector2<usize>;

   fn parametric_coords_iterator() -> impl Iterator<Item = Self::ParametricCoords> {
   iter::empty()
   }

   fn parametric_coords_dimensions() -> Self::ParametricDims {
   Vector2::new(10, 10)
   }
   }

   pub trait Topology {
   type ParametricCoords;
   type ParametricDims;

   fn parametric_coords_iterator() -> impl Iterator<Item = Self::ParametricCoords>;
   fn parametric_coords_dimensions() -> Self::ParametricDims;
   }

   pub trait GenericBoundedSurface<T: Topology> {
   fn get_point(&self, param: T::ParametricCoords) -> anyhow::Result<Vector3<Dec>>;
   }


   pub trait Surface4<PL, PR, PT, PB>:
   GenericBoundedSurface<Four>
   + GetBoundingPath<0, Path = PL>
   + GetBoundingPath<1, Path = PR>
   + GetBoundingPath<2, Path = PT>
   + GetBoundingPath<3, Path = PB>
   where
   PL: Path,
   PR: Path,
   PT: Path,
   PB: Path,
   {
   fn left_path(&self) -> PL {
   <Self as GetBoundingPath<0>>::get_bounding_path(self)
   }
   fn right_path(&self) -> PR {
   <Self as GetBoundingPath<1>>::get_bounding_path(self)
   }
   fn top_path(&self) -> PT {
   <Self as GetBoundingPath<2>>::get_bounding_path(self)
   }
   fn bottom_path(&self) -> PB {
   <Self as GetBoundingPath<3>>::get_bounding_path(self)
   }
   }

   impl<A, B, C, D, T> Surface4<A, B, C, D> for T
   where
T: GenericBoundedSurface<Four>,
    A: Path,
    B: Path,
    C: Path,
    D: Path,
    T: GetBoundingPath<0, Path = A>,
    T: GetBoundingPath<1, Path = B>,
    T: GetBoundingPath<2, Path = C>,
    T: GetBoundingPath<3, Path = D>,
{
}

impl<T, const D: usize> Geometry<D> for T
where
    T: GenericBoundedSurface<Four>,
{
    fn polygonize(&self) -> anyhow::Result<Vec<crate::Face>> {
        let maybe_faces = |prev: <Four as Topology>::ParametricCoords,
                           next: <Four as Topology>::ParametricCoords|
         -> anyhow::Result<Vec<[Vector3<Dec>; 3]>> {
            let a = self.get_point(prev)?;
            let b = self.get_point(Vector2::new(prev.x, next.y))?;
            let c = self.get_point(Vector2::new(next.x, prev.y))?;
            let d = self.get_point(next)?;

            Ok(vec![[a, b, c], [b, d, c]])
        };

        let dims = Four::parametric_coords_dimensions();
        Ok(Segments::new(dims.x)
            .flat_map(|(t, tt)| {
                Segments::new(dims.y)
                    .filter_map(move |(s, ss)| {
                        maybe_faces(Vector2::new(t, s), Vector2::new(tt, ss)).ok()
                    })
                    .flatten()
            })
            .collect())
    }
}
*/
