use nalgebra::Vector3;
use rstar::{Point, RTreeObject, AABB};

use crate::{decimal::Dec, indexes::aabb::Aabb};

use super::{face::FaceId, poly::PolyId};

// use super::poly::PolyId;

impl From<Aabb> for AABB<RtreePt> {
    fn from(value: Aabb) -> Self {
        let p1 = value.min;
        let p2 = value.max;
        AABB::from_corners(p1.into(), p2.into())
    }
}

impl From<Vector3<Dec>> for RtreePt {
    fn from(value: Vector3<Dec>) -> Self {
        Self([value.x, value.y, value.z])
    }
}

impl Point for RtreePt {
    type Scalar = Dec;

    const DIMENSIONS: usize = 3;

    fn generate(mut generator: impl FnMut(usize) -> Self::Scalar) -> Self {
        RtreePt([generator(0), generator(1), generator(2)])
    }

    fn nth(&self, index: usize) -> Self::Scalar {
        self.0[index]
    }

    fn nth_mut(&mut self, index: usize) -> &mut Self::Scalar {
        &mut self.0[index]
    }
}

impl RTreeObject for PolyRtreeRecord {
    type Envelope = AABB<RtreePt>;

    fn envelope(&self) -> Self::Envelope {
        self.1.into()
    }
}

impl RTreeObject for FaceRtreeRecord {
    type Envelope = AABB<RtreePt>;

    fn envelope(&self) -> Self::Envelope {
        self.1.into()
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct RtreePt([Dec; 3]);

#[derive(Debug, PartialEq)]
pub struct PolyRtreeRecord(pub(super) PolyId, pub(super) Aabb);

#[derive(Debug, PartialEq)]
pub struct FaceRtreeRecord(pub(super) FaceId, pub(super) Aabb);
