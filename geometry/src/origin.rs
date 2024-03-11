use nalgebra::{UnitQuaternion, Vector3};
use num_traits::Zero;

use super::decimal::Dec;

#[derive(Clone, Debug)]
pub struct Origin {
    pub center: Vector3<Dec>,
    pub rotation: UnitQuaternion<Dec>,
}

impl Default for Origin {
    fn default() -> Self {
        Self::new()
    }
}

impl Origin {
    pub fn new() -> Self {
        Self {
            center: Vector3::new(Dec::zero(), Dec::zero(), Dec::zero()),
            rotation: UnitQuaternion::new(Vector3::zero()),
        }
    }

    pub fn project(&self, v: Vector3<Dec>) -> Vector3<Dec> {
        let v = v - self.center;
        self.center + self.x() * v.dot(&self.x()) + self.y() * v.dot(&self.y())
    }
    pub fn project_unit(&self, v: Vector3<Dec>) -> Vector3<Dec> {
        (self.x() * v.dot(&self.x()) + self.y() * v.dot(&self.y())).normalize()
    }

    pub fn offset_z(mut self, amount: Dec) -> Self {
        self.center = self.z() * amount + self.center;
        self
    }
    pub fn offset_x(mut self, amount: Dec) -> Self {
        self.center = self.x() * amount + self.center;
        self
    }
    pub fn offset_y(mut self, amount: Dec) -> Self {
        self.center = self.y() * amount + self.center;
        self
    }

    pub fn offset(mut self, axis: Vector3<Dec>) -> Self {
        self.center = axis + self.center;
        self
    }

    pub fn rotate(mut self, quat: UnitQuaternion<Dec>) -> Self {
        self.rotation *= quat;
        self
    }

    pub fn rotate_axisangle(mut self, axisangle: Vector3<Dec>) -> Self {
        let quat = UnitQuaternion::from_scaled_axis(axisangle);
        self.rotation *= quat;
        self
    }

    pub fn left(&self) -> Vector3<Dec> {
        -self.x()
    }

    pub fn right(&self) -> Vector3<Dec> {
        self.x()
    }

    pub fn top(&self) -> Vector3<Dec> {
        self.y()
    }

    pub fn x(&self) -> Vector3<Dec> {
        self.rotation * Vector3::x()
    }
    pub fn y(&self) -> Vector3<Dec> {
        self.rotation * Vector3::y()
    }

    pub fn z(&self) -> Vector3<Dec> {
        self.rotation * Vector3::z()
    }
    pub fn add(&mut self, origin: &Origin) {
        self.center = origin.rotation * (self.center + origin.center);
        self.rotation *= origin.rotation;
    }
}
