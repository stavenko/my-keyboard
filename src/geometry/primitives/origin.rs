use nalgebra::{UnitQuaternion, Vector3};
use num_traits::Zero;
use stl_io::Vector;

use super::decimal::Dec;

#[derive(Clone, Debug)]
pub(crate) struct Origin {
    pub center: Vector3<Dec>,
    pub rotation: UnitQuaternion<Dec>,
}

impl Origin {
    pub(crate) fn new() -> Self {
        Self {
            center: Vector3::new(Dec::zero(), Dec::zero(), Dec::zero()),
            rotation: UnitQuaternion::new(Vector3::zero()),
        }
    }

    pub(crate) fn project(&self, v: Vector3<Dec>) -> Vector3<Dec> {
        let v = v - self.center;
        self.center + self.x() * v.dot(&self.x()) + self.y() * v.dot(&self.y())
    }
    pub(crate) fn project_unit(&self, v: Vector3<Dec>) -> Vector3<Dec> {
        (self.x() * v.dot(&self.x()) + self.y() * v.dot(&self.y())).normalize()
    }

    pub(crate) fn offset_z(mut self, amount: Dec) -> Self {
        self.center = self.z() * amount + self.center;
        self
    }
    pub(crate) fn offset_x(mut self, amount: Dec) -> Self {
        self.center = self.x() * amount + self.center;
        self
    }
    pub(crate) fn offset_y(mut self, amount: Dec) -> Self {
        self.center = self.y() * amount + self.center;
        self
    }

    fn offset(mut self, axis: Vector3<Dec>) -> Self {
        self.center = axis + self.center;
        self
    }

    fn rotate(mut self, quat: UnitQuaternion<Dec>) -> Self {
        self.rotation *= quat;
        self
    }

    pub(crate) fn rotate_axisangle(mut self, axisangle: Vector3<Dec>) -> Self {
        let quat = UnitQuaternion::from_scaled_axis(axisangle);
        self.rotation *= quat;
        self
    }

    pub(crate) fn left(&self) -> Vector3<Dec> {
        -self.x()
    }

    pub(crate) fn right(&self) -> Vector3<Dec> {
        self.x()
    }

    pub(crate) fn top(&self) -> Vector3<Dec> {
        self.y()
    }

    pub(crate) fn x(&self) -> Vector3<Dec> {
        self.rotation * Vector3::x()
    }
    pub(crate) fn y(&self) -> Vector3<Dec> {
        self.rotation * Vector3::y()
    }

    pub(crate) fn z(&self) -> Vector3<Dec> {
        self.rotation * Vector3::z()
    }
    pub(crate) fn add(&mut self, origin: &Origin) {
        self.center = origin.rotation * (self.center + origin.center);
        self.rotation *= origin.rotation;
    }
}
