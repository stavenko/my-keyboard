use std::rc::Rc;

use geometry::geometry::GeometryDyn;

use crate::hole_builder::HoleBuilder;

pub struct Hole {
    pub(crate) shape: Rc<dyn GeometryDyn>,
}

impl Hole {
    pub fn build() -> HoleBuilder {
        HoleBuilder::default()
    }
}
