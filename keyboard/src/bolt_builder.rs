use geometry::decimal::Dec;

use crate::{bolt::Nut, Bolt};

#[derive(Default)]
pub struct BoltBuilder {
    nut: Option<Nut>,
    diameter: Option<Dec>,
    thread_inner_diameter: Option<Dec>,
    height: Option<Dec>,
    head_diameter: Option<Dec>,
    head_height: Option<Dec>,
}

pub struct NutBuilder {}

impl BoltBuilder {
    pub fn m2(self) -> Self {
        self.diameter(Dec::from(2)).nut(Nut::m2_hex())
    }

    pub fn m1_no_nut(self) -> Self {
        self.diameter(Dec::from(1)).no_nut()
    }

    pub fn height(mut self, height: impl Into<Dec>) -> Self {
        self.height = Some(height.into());
        self
    }

    pub fn diameter(mut self, diameter: Dec) -> Self {
        self.diameter = Some(diameter);
        self
    }

    pub fn thread_inner_diameter(mut self, thread_inner_diameter: impl Into<Dec>) -> Self {
        self.thread_inner_diameter = Some(thread_inner_diameter.into());
        self
    }

    pub fn head_diameter(mut self, head_diameter: Dec) -> Self {
        self.head_diameter = Some(head_diameter);
        self
    }

    pub fn head_height(mut self, head_height: impl Into<Dec>) -> Self {
        self.head_height = Some(head_height.into());
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
            thread_inner_diameter: self.thread_inner_diameter,
            nut: self.nut,
        }
    }
}
