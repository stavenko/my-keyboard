use core::fmt;

use anyhow::anyhow;
use itertools::{Either, Itertools};
use stl_io::Triangle;

use super::{
    bsp::bsp3::Bsp3,
    hull::Hull,
    primitives::polygon::{Basis2d, Polygon},
    surface::topology::Topology,
};

pub struct Side {
    basis: Basis2d,
    polygons: Vec<Polygon>,
}
impl Side {
    fn triangles(&self) -> Vec<Triangle> {
        todo!()
    }
}

pub struct Mesh {
    sides: Vec<Side>,
}

impl Mesh {
    pub fn boolean_union(self, other: Self) -> Either<Self, (Self, Self)> {
        todo!("implement boolean_union")
    }
    pub fn boolean_diff(self, other: Self) -> Option<Self> {
        todo!("implement boolean_diff")
    }
    pub fn boolean_intersecion(self, other: Self) -> Option<Self> {
        todo!("implement boolean_intersecion")
    }
}

pub struct TriIter {
    inner: <Vec<Triangle> as IntoIterator>::IntoIter,
    size: usize,
}

impl ExactSizeIterator for TriIter {}

impl Iterator for TriIter {
    type Item = Triangle;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next()
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.size, Some(self.size))
    }
}

impl IntoIterator for Mesh {
    type Item = Triangle;

    type IntoIter = TriIter;

    fn into_iter(self) -> Self::IntoIter {
        let triangles = self
            .sides
            .into_iter()
            .flat_map(|side| side.triangles())
            .collect_vec();
        let size = triangles.len();
        TriIter {
            inner: triangles.into_iter(),
            size,
        }
    }
}

impl<T> TryFrom<Hull<T>> for Mesh
where
    T: Topology,
{
    type Error = anyhow::Error;

    fn try_from(value: Hull<T>) -> Result<Self, Self::Error> {
        let faces = value
            .sides
            .into_iter()
            .flatten()
            .chain(value.outer)
            .chain(value.inner);

        dbg!("buildmesh");
        let bsp = Bsp3::build(faces).ok_or(anyhow!("failed to build bsp tree"))?;
        dbg!("ok convert to mesh");
        todo!("mesh creation");

        //  Ok(Self(bsp))
    }
}
