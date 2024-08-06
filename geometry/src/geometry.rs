use crate::indexes::geo_index::index::GeoIndex;

pub trait Geometry {
    fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()>;
}

pub trait GeometryDyn {
    fn polygonize(&self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()>;
}
