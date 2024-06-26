use std::{
    collections::HashMap,
    fmt::{self, Debug},
};

use nalgebra::{Matrix2, Vector2, Vector3};
use uuid::Uuid;

use crate::{decimal::Dec, indexes::vertex_index::PtId};

use super::{
    index::GeoIndex,
    rib::{Rib, RibId, RibRef},
};

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct SegId(Uuid);

#[derive(Debug, PartialEq, Clone, Copy)]
pub(crate) enum SegmentDir {
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

#[derive(Copy, Clone)]
pub struct SegRef<'i> {
    pub(super) rib_id: RibId,
    pub(super) dir: SegmentDir,
    pub(super) index: &'i GeoIndex,
}

#[derive(Copy, Clone)]
pub struct SegmentRef<'i> {
    pub(super) to: PtId,
    pub(super) from: PtId,
    pub(super) index: &'i GeoIndex,
}

impl<'a> fmt::Debug for SegRef<'a> {
    fn fmt(&self, fo: &mut fmt::Formatter<'_>) -> fmt::Result {
        let f = self.from();
        let t = self.to();
        write!(fo, "{} {} {} -> {} {} {}", f.x, f.y, f.z, t.x, t.y, t.z)
    }
}
impl<'a> SegmentRef<'a> {
    pub fn from(&self) -> Vector3<Dec> {
        self.index.vertices.get_point(self.from_pt())
    }

    pub fn to(&self) -> Vector3<Dec> {
        self.index.vertices.get_point(self.to_pt())
    }

    pub(crate) fn dir(&self) -> Vector3<Dec> {
        self.to() - self.from()
    }

    pub(crate) fn has(&self, v: PtId) -> bool {
        self.to_pt() == v || self.from_pt() == v
    }

    pub(crate) fn to_pt(&self) -> PtId {
        self.to
    }

    pub(crate) fn from_pt(&self) -> PtId {
        self.from
    }

    pub(crate) fn flip(self) -> Self {
        Self {
            to: self.from,
            from: self.to,
            index: self.index,
        }
    }

    pub(crate) fn get_intersection_params_seg_ref(&self, to: &SegRef<'_>) -> Option<(Dec, Dec)> {
        let segment_dir = to.dir().normalize();
        let self_dir = self.dir().normalize();
        let q = self.from() - to.from();

        let dot = self_dir.dot(&segment_dir);

        let m = Matrix2::new(Dec::from(1), -dot, dot, -Dec::from(1));
        let b = -Vector2::new(q.dot(&self_dir), q.dot(&segment_dir));

        if let Some(mi) = m.try_inverse() {
            let st = mi * b;
            Some((st.x / self.dir().magnitude(), st.y / to.dir().magnitude()))
        } else {
            None
        }
    }
}

impl<'a> SegRef<'a> {
    pub fn from(&self) -> Vector3<Dec> {
        self.index.vertices.get_point(self.from_pt())
    }

    pub fn to(&self) -> Vector3<Dec> {
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
            index: self.index,
            rib_id: self.rib_id,
        }
    }

    pub(crate) fn flip(self) -> Self {
        Self {
            rib_id: self.rib_id,
            dir: self.dir.flip(),
            index: self.index,
        }
    }

    pub(crate) fn seg(self) -> Seg {
        Seg {
            rib_id: self.rib_id,
            dir: self.dir,
        }
    }

    pub(crate) fn get_intersection_params_seg_ref(&self, to: &SegRef<'_>) -> Option<(Dec, Dec)> {
        let segment_dir = to.dir().normalize();
        let self_dir = self.dir().normalize();
        let q = self.from() - to.from();

        let dot = self_dir.dot(&segment_dir);

        let m = Matrix2::new(Dec::from(1), -dot, dot, -Dec::from(1));
        let b = -Vector2::new(q.dot(&self_dir), q.dot(&segment_dir));

        if let Some(mi) = m.try_inverse() {
            let st = mi * b;
            Some((st.x / self.dir().magnitude(), st.y / to.dir().magnitude()))
        } else {
            None
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
    pub(crate) fn to_ref<'a>(self, index: &'a GeoIndex) -> SegRef<'a> {
        SegRef {
            rib_id: self.rib_id,
            dir: self.dir,
            index,
        }
    }
}
