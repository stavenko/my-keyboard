use std::fmt;

use crate::decimal::Dec;
use nalgebra::Vector3;

use super::{Path, PathInverse};

#[derive(Clone)]
pub struct EdgeSegment {
    pub from: Vector3<Dec>,
    pub to: Vector3<Dec>,
    pub edge_from: Vector3<Dec>,
    pub edge_to: Vector3<Dec>,
}

impl fmt::Debug for EdgeSegment {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "\n{}… {}… {}… -> {}… {}… {}…\n ⇧{}, {}, {}, ⇩{}, {}, {}",
            self.from.x.round_dp(4),
            self.from.y.round_dp(4),
            self.from.z.round_dp(4),
            self.to.x.round_dp(4),
            self.to.y.round_dp(4),
            self.to.z.round_dp(4),
            self.edge_from.x.round_dp(4),
            self.edge_from.y.round_dp(4),
            self.edge_from.z.round_dp(4),
            self.edge_to.x.round_dp(4),
            self.edge_to.y.round_dp(4),
            self.edge_to.z.round_dp(4)
        )
    }
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

    fn get_tangent(&self, _t: Dec) -> Vector3<Dec> {
        todo!()
    }

    fn first(&self) -> Vector3<Dec> {
        self.from
    }

    fn last(&self) -> Vector3<Dec> {
        self.from
    }
}
