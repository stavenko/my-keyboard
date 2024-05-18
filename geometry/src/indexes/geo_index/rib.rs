use std::{fmt, ops::Deref};

use nalgebra::Vector3;

use crate::{decimal::Dec, indexes::vertex_index::PtId};

use super::{index::GeoIndex, seg::SegmentDir};

#[derive(PartialEq, Eq, Hash, Clone, Copy, Ord, PartialOrd)]
pub struct RibId(pub(super) usize);

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Rib(pub(super) PtId, pub(super) PtId);

impl Rib {
    pub fn build(from: PtId, to: PtId) -> (Self, SegmentDir) {
        if from > to {
            (Rib(to, from), SegmentDir::Rev)
        } else {
            (Rib(from, to), SegmentDir::Fow)
        }
    }
}

pub struct RibRef<'a> {
    pub(crate) index: &'a GeoIndex,
    pub(super) rib_id: RibId,
}

impl<'a> RibRef<'a> {
    pub(crate) fn from(&self) -> Vector3<Dec> {
        self.index
            .vertices
            .get_point(self.index.ribs[&self.rib_id].0)
    }

    pub(crate) fn to(&self) -> Vector3<Dec> {
        self.index
            .vertices
            .get_point(self.index.ribs[&self.rib_id].1)
    }

    pub(crate) fn dir(&self) -> Vector3<Dec> {
        self.to() - self.from()
    }

    pub(crate) fn has(&self, pt_id: PtId) -> bool {
        let rib = self.index.ribs[&self.rib_id];
        rib.0 == pt_id || rib.1 == pt_id
    }

    pub(crate) fn from_pt(&self) -> PtId {
        self.index.ribs[&self.rib_id].0
    }

    pub(crate) fn to_pt(&self) -> PtId {
        self.index.ribs[&self.rib_id].1
    }
}

impl<'a> Deref for RibRef<'a> {
    type Target = RibId;

    fn deref(&self) -> &Self::Target {
        &self.rib_id
    }
}

impl fmt::Debug for RibId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "RibId:{}", self.0)
    }
}
