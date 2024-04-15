use nalgebra::Vector3;

use crate::decimal::Dec;

#[derive(Debug, Clone, Default, Copy)]
#[allow(dead_code)]

pub struct Sphere {
    center: Vector3<Dec>,
    radius: Dec,
}
