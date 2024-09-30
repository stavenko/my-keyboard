use crate::indexes::geo_index::mesh::MeshRefMut;

pub trait Geometry {
    fn polygonize(self, mesh: &mut MeshRefMut, complexity: usize) -> anyhow::Result<()>;
}

pub trait GeometryDyn {
    fn polygonize(&self, mesh: MeshRefMut, complexity: usize) -> anyhow::Result<()>;
}
