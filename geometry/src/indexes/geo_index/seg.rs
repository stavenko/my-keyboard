use std::collections::HashMap;

use nalgebra::Vector3;
use uuid::Uuid;

use crate::{decimal::Dec, indexes::vertex_index::PtId};

use super::{
    index::GeoIndex,
    rib::{Rib, RibId, RibRef},
};

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct SegId(Uuid);

#[derive(Debug, PartialEq, Clone, Copy)]
pub(super) enum SegmentDir {
    Fow,
    Rev,
}

impl SegmentDir {
    fn flip(&self) -> SegmentDir {
        match self {
            SegmentDir::Fow => Self::Rev,
            SegmentDir::Rev => Self::Fow,
        }
    }
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Seg {
    pub(super) rib_id: RibId,
    pub(super) dir: SegmentDir,
}

#[derive(Debug, Clone, Copy)]
pub struct SegRef<'i> {
    pub(super) rib_id: RibId,
    pub(super) dir: SegmentDir,
    pub(super) index: &'i GeoIndex,
}

impl<'a> SegRef<'a> {
    pub(crate) fn from(&self) -> Vector3<Dec> {
        let rib = self.index.ribs[&self.rib_id];
        self.index.vertices.get_point(self.from_pt())
    }

    pub(crate) fn to(&self) -> Vector3<Dec> {
        let rib = self.index.ribs[&self.rib_id];
        self.index.vertices.get_point(self.to_pt())
    }

    pub(crate) fn dir(&self) -> Vector3<Dec> {
        self.to() - self.from()
    }

    pub(crate) fn has(&self, v: PtId) -> bool {
        self.to_pt() == v || self.from_pt() == v
    }

    pub(crate) fn to_pt(&self) -> PtId {
        let rib = self.index.ribs[&self.rib_id];
        match self.dir {
            SegmentDir::Fow => rib.1,
            SegmentDir::Rev => rib.0,
        }
    }
    pub(crate) fn from_pt(&self) -> PtId {
        let rib = self.index.ribs[&self.rib_id];
        match self.dir {
            SegmentDir::Fow => rib.0,
            SegmentDir::Rev => rib.1,
        }
    }

    pub(crate) fn rib(&self) -> super::rib::RibRef<'a> {
        RibRef {
            index: &self.index,
            rib_id: self.rib_id,
        }
    }
}

impl Default for SegId {
    fn default() -> Self {
        Self(Uuid::new_v4())
    }
}

impl Seg {
    pub(super) fn flip(&self) -> Seg {
        Self {
            rib_id: self.rib_id,
            dir: self.dir.flip(),
        }
    }

    pub(super) fn to(&self, ribs: &HashMap<RibId, Rib>) -> PtId {
        let rib = ribs[&self.rib_id];
        match self.dir {
            SegmentDir::Fow => rib.1,
            SegmentDir::Rev => rib.0,
        }
    }
    pub(super) fn from(&self, ribs: &HashMap<RibId, Rib>) -> PtId {
        let rib = ribs[&self.rib_id];
        match self.dir {
            SegmentDir::Fow => rib.0,
            SegmentDir::Rev => rib.1,
        }
    }
}
