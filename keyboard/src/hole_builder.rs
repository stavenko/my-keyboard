use std::rc::Rc;

use anyhow::anyhow;
use geometry::geometry::GeometryDyn;

use crate::hole::Hole;

#[derive(Default)]
pub struct HoleBuilder {
    shape: Option<Rc<dyn GeometryDyn>>,
}

impl HoleBuilder {
    pub fn shape(mut self, shape: impl GeometryDyn + 'static) -> Self {
        self.shape = Some(Rc::new(shape));
        self
    }

    pub fn build(self) -> anyhow::Result<Hole> {
        let shape = self.shape.ok_or(anyhow!("Shape is not provided"))?;
        Ok(Hole { shape })
    }
}
