use anyhow::anyhow;
use geometry::indexes::geo_index::index::GeoIndex;

pub struct ChokHotswap;

impl ChokHotswap {
    pub fn top_mesh(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        Err(anyhow!(">>"))
    }

    pub fn bottom_mesh(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        Err(anyhow!(">>"))
    }
}
