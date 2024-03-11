use crate::planar::polygon::Polygon;

pub trait Geometry {
    fn polygonize(&self) -> anyhow::Result<Vec<Polygon>>;
}

impl<T> Geometry for Vec<T>
where
    T: Geometry,
{
    fn polygonize(&self) -> anyhow::Result<Vec<Polygon>> {
        Ok(self
            .iter()
            .filter_map(|item| item.polygonize().ok())
            .flatten()
            .collect())
    }
}
