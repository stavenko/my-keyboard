use nalgebra::Vector3;

use crate::decimal::Dec;

pub trait DirectionPerpendicular {
    fn direction_perpendicular(&self) -> Vector3<Dec>;
}
