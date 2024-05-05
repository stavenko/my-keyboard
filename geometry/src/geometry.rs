use tap::TapFallible;

use crate::{indexes::geo_index::index::GeoIndex, planar::polygon::Polygon};

pub trait Geometry {
    fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()>;
}

/*
impl<T> Geometry for Vec<T>
where
    T: Geometry,
{
    fn polygonize(&self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
        self.iter().for_each(|item| {
            item.polygonize(index, complexity)
                .tap_err(|err| println!("Error polygonizing {err}"))
        });
        Ok(())
    }
}
*/
