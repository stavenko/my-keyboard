use nalgebra::Vector3;

use crate::decimal::Dec;

#[derive(Debug, Clone, Default, Copy)]
#[allow(dead_code)]

pub struct Sphere {
    pub center: Vector3<Dec>,
    pub radius: Dec,
}
