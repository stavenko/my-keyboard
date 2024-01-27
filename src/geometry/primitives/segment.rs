use nalgebra::Vector3;

use super::{decimal::Dec, line::Line};

#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    from: Vector3<Dec>,
    to: Vector3<Dec>,
}

impl Segment {
    pub fn join(self, mut other: Self) -> anyhow::Result<Self> {
        todo!()
    }

    pub fn new(from: Vector3<Dec>, to: Vector3<Dec>) -> Self {
        Self { from, to }
    }

    pub fn get_line(&self, with_normal: Vector3<Dec>) -> Line {
        Line {
            normal: with_normal,
            origin: self.from,
            dir: self.to - self.from,
        }
    }

    pub(crate) fn flip(mut self) -> Self {
        Self {
            from: self.to,
            to: self.from,
        }
    }
}
