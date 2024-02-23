use nalgebra::Vector3;
use rust_decimal_macros::dec;

use super::{
    decimal::Dec,
    segment::{Segment},
};

#[derive(Clone, Debug)]
pub struct Line {
    pub origin: Vector3<Dec>,
    pub dir: Vector3<Dec>,
    pub normal: Vector3<Dec>,
}
#[derive(Default, Debug)]
pub struct SplitResult {
    pub front: Vec<Segment>,
    pub back: Vec<Segment>,
    pub coplanar_back: Vec<Segment>,
    pub coplanar_front: Vec<Segment>,
}

impl SplitResult {
    fn front(mut self, face: Segment) -> Self {
        self.front.push(face);
        self
    }
    fn back(mut self, face: Segment) -> Self {
        self.back.push(face);
        self
    }
    fn coplanar_back(mut self, face: Segment) -> Self {
        self.coplanar_back.push(face);
        self
    }
    fn coplanar_front(mut self, face: Segment) -> Self {
        self.coplanar_front.push(face);
        self
    }
}

#[derive(PartialEq)]
enum Location {
    Front,
    Back,
    Coplanar,
}

impl Line {
    fn split(&self, _s: &Segment) -> (Segment, Segment) {
        todo!()
    }

    fn location(&self, v: &Vector3<Dec>) -> Location {
        let eps: Dec = dec!(1e-9).into();
        let i = (v - self.origin).dot(&self.normal);
        if i < -eps {
            Location::Back
        } else if i > eps {
            Location::Front
        } else {
            Location::Coplanar
        }
    }
    pub fn split_segment(&self, _segment: Segment) -> SplitResult {
        todo!();
    }

    pub(crate) fn flip(mut self) -> Self {
        self.normal = -self.normal;
        self
    }
}
