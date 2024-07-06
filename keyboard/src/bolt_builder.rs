use geometry::decimal::Dec;

use crate::{bolt::Nut, Bolt};

#[derive(Default)]
pub struct BoltBuilder {
    nut: Option<Nut>,
    diameter: Option<Dec>,
    height: Option<Dec>,
    head_diameter: Option<Dec>,
    head_height: Option<Dec>,
}

pub struct NutBuilder {}

impl BoltBuilder {
    pub fn m2(self) -> Self {
        self.diameter(Dec::from(2)).nut(Nut::m2_hex())
    }

    pub fn height(mut self, height: Dec) -> Self {
        self.height = Some(height);
        self
    }

    pub fn diameter(mut self, diameter: Dec) -> Self {
        self.diameter = Some(diameter);
        self
    }

    pub fn head_diameter(mut self, head_diameter: Dec) -> Self {
        self.head_diameter = Some(head_diameter);
        self
    }

    pub fn head_height(mut self, head_height: Dec) -> Self {
        self.head_height = Some(head_height);
        self
    }

    pub fn nut(mut self, nut: Nut) -> Self {
        self.nut = Some(nut);
        self
    }

    pub fn no_nut(mut self) -> Self {
        self.nut = None;
        self
    }
    pub fn build(self) -> Bolt {
        Bolt {
            head_diameter: self.head_diameter.expect("No head diameter"),
            diameter: self.diameter.expect("Bolt diameter not specified"),
            head_height: self.head_height.expect("Head head not specified"),
            height: self.height.expect("Bolt height is not specified"),
            nut: self.nut,
        }
    }
}
