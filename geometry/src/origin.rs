use nalgebra::{ClosedAdd, Matrix4, SimdRealField, UnitQuaternion, Vector3};
use num_traits::Zero;

use super::decimal::Dec;

#[derive(Clone, Debug)]
pub struct BaseOrigin<F> {
    pub center: Vector3<F>,
    pub rotation: UnitQuaternion<F>,
}

pub type Origin = BaseOrigin<Dec>;

impl<F> Default for BaseOrigin<F>
where
    F: Zero + Copy,
    F: nalgebra::Scalar + nalgebra::SimdRealField + ClosedAdd + nalgebra::SimdRealField,
    F::Element: SimdRealField,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<F> BaseOrigin<F>
where
    F: Zero + Copy,
    F: nalgebra::Scalar + nalgebra::SimdRealField + ClosedAdd + nalgebra::SimdRealField,
    F::Element: SimdRealField,
{
    pub fn new() -> Self {
        Self {
            center: Vector3::new(F::zero(), F::zero(), F::zero()),
            rotation: UnitQuaternion::new(Vector3::zero()),
        }
    }

    pub fn project(&self, v: Vector3<F>) -> Vector3<F> {
        let v = v - self.center;
        self.center + self.x() * v.dot(&self.x()) + self.y() * v.dot(&self.y())
    }

    pub fn project_unit(&self, v: Vector3<F>) -> Vector3<F> {
        (self.x() * v.dot(&self.x()) + self.y() * v.dot(&self.y())).normalize()
    }

    pub fn offset_z(mut self, amount: impl Into<F>) -> Self {
        self.center = self.z() * amount.into() + self.center;
        self
    }

    pub fn offset_x(mut self, amount: impl Into<F>) -> Self {
        self.center = self.x() * amount.into() + self.center;
        self
    }

    pub fn offset_y(mut self, amount: impl Into<F>) -> Self {
        self.center = self.y() * amount.into() + self.center;
        self
    }

    pub fn offset(mut self, axis: Vector3<F>) -> Self {
        self.center = axis + self.center;
        self
    }

    pub fn rotate(mut self, quat: UnitQuaternion<F>) -> Self {
        self.rotation *= quat;
        self
    }

    pub fn rotate_axisangle(mut self, axisangle: Vector3<F>) -> Self {
        let quat = UnitQuaternion::from_scaled_axis(axisangle);
        self.rotation *= quat;
        self
    }

    pub fn left(&self) -> Vector3<F> {
        -self.x()
    }

    pub fn right(&self) -> Vector3<F> {
        self.x()
    }

    pub fn top(&self) -> Vector3<F> {
        self.y()
    }

    pub fn x(&self) -> Vector3<F> {
        self.rotation * Vector3::x()
    }

    pub fn y(&self) -> Vector3<F> {
        self.rotation * Vector3::y()
    }

    pub fn z(&self) -> Vector3<F> {
        self.rotation * Vector3::z()
    }

    pub fn get_matrix(&self) -> Matrix4<F> {
        let m = self.rotation.to_rotation_matrix().to_homogeneous();
        m * Matrix4::new_translation(&self.center)
    }

    pub fn apply(&mut self, origin: &BaseOrigin<F>) {
        self.center = origin.rotation * (self.center) + origin.center;
        self.rotation = origin.rotation * self.rotation;
    }
}

/*
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

    pub fn offset_z(mut self, amount: impl Into<Dec>) -> Self {
        self.center = self.z() * amount.into() + self.center;
        self
    }

    pub fn offset_x(mut self, amount: impl Into<Dec>) -> Self {
        self.center = self.x() * amount.into() + self.center;
        self
    }

    pub fn offset_y(mut self, amount: impl Into<Dec>) -> Self {
        self.center = self.y() * amount.into() + self.center;
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

    pub fn get_matrix(&self) -> Matrix4<Dec> {
        let m = self.rotation.to_rotation_matrix().to_homogeneous();
        m * Matrix4::new_translation(&self.center)
    }

    pub fn apply(&mut self, origin: &Origin) {
        self.center = origin.rotation * (self.center) + origin.center;
        self.rotation = origin.rotation * self.rotation;
    }
}
*/
