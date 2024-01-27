use nalgebra::UnitVector3;
use nalgebra::Vector3;

use self::bezier::BezierEdge;
use self::segment::EdgeSegment;

use super::primitives::decimal::Dec;

pub(crate) mod bezier;
// pub mod polypath;
pub(crate) mod segment;

#[derive(Clone, Debug)]
pub enum SomePath {
    S(EdgeSegment),
    B(BezierEdge),
}
impl From<EdgeSegment> for SomePath {
    fn from(value: EdgeSegment) -> Self {
        Self::S(value)
    }
}
impl From<BezierEdge> for SomePath {
    fn from(value: BezierEdge) -> Self {
        Self::B(value)
    }
}

pub trait Path: std::fmt::Debug {
    fn get_t(&self, t: Dec) -> Vector3<Dec>;
    fn len(&self) -> Dec;
    fn get_tangent(&self, t: Dec) -> Vector3<Dec>;
    fn first(&self) -> Vector3<Dec>;
    fn last(&self) -> Vector3<Dec>;
    fn get_edge_dir(&self, t: Dec) -> Vector3<Dec>;
    fn segments_hint(&self) -> usize {
        20
    }
}

impl Path for SomePath {
    fn get_t(&self, t: Dec) -> Vector3<Dec> {
        match self {
            SomePath::S(p) => p.get_t(t),
            SomePath::B(p) => p.get_t(t),
        }
    }

    fn len(&self) -> Dec {
        match self {
            SomePath::S(p) => p.len(),
            SomePath::B(p) => p.len(),
        }
    }

    fn get_tangent(&self, t: Dec) -> Vector3<Dec> {
        match self {
            SomePath::S(p) => p.get_tangent(t),
            SomePath::B(p) => p.get_tangent(t),
        }
    }

    fn first(&self) -> Vector3<Dec> {
        match self {
            SomePath::S(p) => p.first(),
            SomePath::B(p) => p.first(),
        }
    }

    fn last(&self) -> Vector3<Dec> {
        match self {
            SomePath::S(p) => p.last(),
            SomePath::B(p) => p.last(),
        }
    }

    fn get_edge_dir(&self, t: Dec) -> Vector3<Dec> {
        match self {
            SomePath::S(p) => p.get_edge_dir(t),
            SomePath::B(p) => p.get_edge_dir(t),
        }
    }
    fn segments_hint(&self) -> usize {
        match self {
            SomePath::S(p) => p.segments_hint(),
            SomePath::B(p) => p.segments_hint(),
        }
    }
}

pub trait PathInverse: Path {
    fn inverse(self) -> Self;
}

pub trait Translate {
    fn translate(self) -> Self;
}
