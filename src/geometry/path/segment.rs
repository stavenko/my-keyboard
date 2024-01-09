use crate::geometry::primitives::decimal::Dec;
use nalgebra::Vector3;

use super::{Path, PathInverse};

#[derive(Clone, Debug)]
pub struct EdgeSegment {
    pub from: Vector3<Dec>,
    pub to: Vector3<Dec>,
    pub edge_from: Vector3<Dec>,
    pub edge_to: Vector3<Dec>,
}

impl PathInverse for EdgeSegment {
    fn inverse(self) -> Self {
        Self {
            from: self.to,
            to: self.from,
            edge_from: self.edge_to,
            edge_to: self.edge_from,
        }
    }
}

impl Path for EdgeSegment {
    fn get_edge_dir(&self, t: Dec) -> Vector3<Dec> {
        self.edge_from.lerp(&self.edge_to, t)
    }
    fn get_t(&self, t: Dec) -> Vector3<Dec> {
        self.from.lerp(&self.to, t)
    }

    fn len(&self) -> Dec {
        let d = self.to - self.from;
        d.magnitude()
    }
    fn segments_hint(&self) -> usize {
        1
    }

    fn get_tangent(&self, t: Dec) -> Vector3<Dec> {
        todo!()
    }

    fn first(&self) -> Vector3<Dec> {
        self.from
    }

    fn last(&self) -> Vector3<Dec> {
        self.from
    }
}
