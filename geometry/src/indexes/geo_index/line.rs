use nalgebra::Vector3;

use crate::{decimal::Dec, indexes::vertex_index::PtId};

use super::index::GeoIndex;

#[derive(Clone, Copy, Debug)]
pub struct Line {
    pub(crate) origin: PtId,
    pub(crate) dir: Vector3<Dec>,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Ord, PartialOrd, Debug)]
pub struct LineId(pub(super) usize);

pub struct LineRef<'a> {
    pub(crate) line_id: LineId,
    pub(crate) index: &'a GeoIndex,
}
