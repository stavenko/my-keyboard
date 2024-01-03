use nalgebra::UnitVector3;
use nalgebra::Vector3;

use self::bezier::BezierEdge;
use self::segment::EdgeSegment;

pub(crate) mod bezier;
pub mod polypath;
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
    fn get_t(&self, t: f32) -> Vector3<f32>;
    fn len(&self) -> f32;
    fn get_tangent(&self, t: f32) -> UnitVector3<f32>;
    fn first(&self) -> Vector3<f32>;
    fn last(&self) -> Vector3<f32>;
    fn get_edge_dir(&self, t: f32) -> Vector3<f32>;
}

impl Path for SomePath {
    fn get_t(&self, t: f32) -> Vector3<f32> {
        match self {
            SomePath::S(p) => p.get_t(t),
            SomePath::B(p) => p.get_t(t),
        }
    }

    fn len(&self) -> f32 {
        match self {
            SomePath::S(p) => p.len(),
            SomePath::B(p) => p.len(),
        }
    }

    fn get_tangent(&self, t: f32) -> UnitVector3<f32> {
        match self {
            SomePath::S(p) => p.get_tangent(t),
            SomePath::B(p) => p.get_tangent(t),
        }
    }

    fn first(&self) -> Vector3<f32> {
        match self {
            SomePath::S(p) => p.first(),
            SomePath::B(p) => p.first(),
        }
    }

    fn last(&self) -> Vector3<f32> {
        match self {
            SomePath::S(p) => p.last(),
            SomePath::B(p) => p.last(),
        }
    }

    fn get_edge_dir(&self, t: f32) -> Vector3<f32> {
        match self {
            SomePath::S(p) => p.get_edge_dir(t),
            SomePath::B(p) => p.get_edge_dir(t),
        }
    }
}

pub trait PathInverse: Path {
    fn inverse(self) -> Self;
}

pub trait Translate {
    fn translate(self) -> Self;
}
