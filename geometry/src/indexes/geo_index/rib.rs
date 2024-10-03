use std::{fmt, ops::Deref};

use nalgebra::Vector3;
use num_traits::One;

use crate::{decimal::Dec, indexes::vertex_index::PtId};

use super::{
    geo_object::{GeoObject, UnRef},
    index::GeoIndex,
    seg::SegmentDir,
};

#[derive(PartialEq, Eq, Hash, Clone, Copy, Ord, PartialOrd)]
pub struct RibId(pub(super) usize);

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Rib(pub(super) PtId, pub(super) PtId);

impl Rib {
    pub(crate) fn build(from: PtId, to: PtId) -> (Self, SegmentDir) {
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

impl fmt::Display for RibRef<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:?} {:?} -> {:?}",
            self.rib_id,
            self.from_pt(),
            self.to_pt(),
        )
    }
}

#[allow(dead_code)]
pub struct RibRefMut<'a> {
    pub(crate) index: &'a mut GeoIndex,
    pub(super) rib_id: RibId,
}

impl<'a> RibRef<'a> {
    pub(crate) fn from(&self) -> Vector3<Dec> {
        self.index
            .vertices
            .get_point(self.index.ribs[&self.rib_id].0)
    }

    pub(crate) fn middle(&self) -> Vector3<Dec> {
        self.from().lerp(&self.to(), Dec::one() / 2)
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

    #[allow(clippy::wrong_self_convention)]
    pub(crate) fn from_pt(&self) -> PtId {
        self.index.ribs[&self.rib_id].0
    }

    pub(crate) fn to_pt(&self) -> PtId {
        self.index.ribs[&self.rib_id].1
    }

    pub(crate) fn magnitude(&self) -> Dec {
        (self.from() - self.to()).magnitude()
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

impl PartialEq<usize> for RibId {
    fn eq(&self, other: &usize) -> bool {
        self.0 == *other
    }
}

impl<'a> UnRef<'a> for RibRef<'a> {
    type Obj = RibId;

    fn un_ref(self) -> Self::Obj {
        self.rib_id
    }
}

impl<'a> UnRef<'a> for RibRefMut<'a> {
    type Obj = RibId;

    fn un_ref(self) -> Self::Obj {
        self.rib_id
    }
}

impl<'a> GeoObject<'a> for RibId {
    type Ref = RibRef<'a>;

    type MutRef = RibRefMut<'a>;

    fn make_ref(&self, index: &'a GeoIndex) -> Self::Ref {
        RibRef {
            index,
            rib_id: *self,
        }
    }

    fn make_mut_ref(&self, index: &'a mut GeoIndex) -> Self::MutRef {
        RibRefMut {
            index,
            rib_id: *self,
        }
    }
}
