use nalgebra::Vector3;

use super::{Path, PathInverse};

#[derive(Clone, Debug)]
pub struct EdgeSegment {
    pub from: Vector3<f32>,
    pub to: Vector3<f32>,
    pub edge_from: Vector3<f32>,
    pub edge_to: Vector3<f32>,
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
    fn get_edge_dir(&self, t: f32) -> Vector3<f32> {
        self.edge_from.lerp(&self.edge_to, t)
    }
    fn get_t(&self, t: f32) -> Vector3<f32> {
        self.from.lerp(&self.to, t)
    }

    fn len(&self) -> f32 {
        let d = self.to - self.from;
        d.magnitude()
    }

    fn get_tangent(&self, t: f32) -> nalgebra::UnitVector3<f32> {
        todo!()
    }

    fn first(&self) -> Vector3<f32> {
        self.from
    }

    fn last(&self) -> Vector3<f32> {
        self.from
    }
}
