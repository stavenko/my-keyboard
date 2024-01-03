use anyhow::anyhow;
use itertools::Itertools;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use scad::{ScadElement, ScadObject};

use self::{primitives::Face, spatial_index::Index};

pub(crate) mod bezier;
pub(crate) mod hull;
pub(crate) mod path;
pub(crate) mod primitives;
mod spatial_index;
pub(crate) mod stiching;
pub(crate) mod surface;

pub trait Geometry {
    fn polygonize(&self) -> anyhow::Result<Vec<Face>>;
}

impl<T> Geometry for Vec<T>
where
    T: Geometry,
{
    fn polygonize(&self) -> anyhow::Result<Vec<Face>> {
        Ok(self
            .iter()
            .filter_map(|item| item.polygonize().ok())
            .flatten()
            .collect())
    }
}

#[derive(Default)]
pub struct FaceCollection {
    faces: Vec<Face>,
}

impl FaceCollection {
    pub fn from_geometry(item: impl Geometry) -> anyhow::Result<Self> {
        let fc = FaceCollection::default();
        Ok(fc.update(item.polygonize()?))
    }

    pub fn update(mut self, mut faces: Vec<Face>) -> Self {
        self.faces.append(&mut faces);
        self
    }

    pub fn join(&mut self, geometry: impl Geometry) -> anyhow::Result<()> {
        self.faces.extend(geometry.polygonize()?);
        Ok(())
    }
    pub fn quantize(face: Face) -> Face {
        let factor = 1e5;
        face.map(|v| (v * factor).map(|c| c.round()) / factor)
    }

    pub fn make_scad(self) -> anyhow::Result<ScadObject> {
        let mut spatial_index = Index::allocate_default();
        let faces = self
            .faces
            .into_iter()
            .map(Self::quantize)
            .collect::<Vec<_>>();
        for f in &faces {
            for p in f.iter() {
                spatial_index.insert(*p, 0);
            }
        }
        let spatial_index = spatial_index.rebalance();
        let faces = faces
            .into_par_iter()
            .map(|face| {
                face.into_iter()
                    .map(|point| {
                        spatial_index
                            .get_point_index(&point)
                            .map(|ix| ix as i32)
                            .ok_or(anyhow!(
                                "no such point in index - we lost it somehow {point}"
                            ))
                    })
                    .try_collect::<_, Vec<i32>, _>()
            })
            .collect::<Vec<_>>();
        let faces = faces.into_iter().try_collect()?;

        Ok(ScadObject::new(ScadElement::Polyhedron(
            spatial_index.linearize(),
            faces,
        )))
    }
}
