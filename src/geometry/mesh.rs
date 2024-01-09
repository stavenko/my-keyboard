use core::fmt;

use anyhow::anyhow;

use super::{
    bsp::Bsp,
    hull::Hull,
    primitives::{polygon::Polygon, Face},
    surface::topology::Topology,
    Geometry,
};

pub struct Mesh(Bsp);
// pub struct SimpleMesh(Vec<Face>);

/*
impl IntoIterator for SimpleMesh {
    type Item = Face;

    type IntoIter = <Vec<Face> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

impl SimpleMesh {
    pub(crate) fn join(self, mm: SimpleMesh) -> SimpleMesh {
        self
    }
}
*/

impl Mesh {
    pub(crate) fn join(self, b: Mesh) -> Mesh {
        dbg!("join");
        let mut a = self.0.clip_by(&b.0);
        let b = b.0.clip_by(&a);
        let b = b.invert();
        let b = b.clip_by(&a);
        let b = b.invert();
        a.merge(b);
        Self(a)
        //self
    }
}
pub struct TriIter {
    inner: Box<dyn Iterator<Item = Face>>,
    size: usize,
}
impl ExactSizeIterator for TriIter {}

impl Iterator for TriIter {
    type Item = Face;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next()
    }
    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.size, Some(self.size))
    }
}

impl IntoIterator for Mesh {
    type Item = Face;

    type IntoIter = TriIter;

    fn into_iter(self) -> Self::IntoIter {
        let size = self.0.calculate_triangles();
        TriIter {
            inner: Box::new(self.0.into_iter().flat_map(|p| p.triangles())),
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
        let bsp = Bsp::build(faces).ok_or(anyhow!("failed to build bsp tree"))?;
        dbg!("ok convert to mesh");

        Ok(Self(bsp))
    }
}
