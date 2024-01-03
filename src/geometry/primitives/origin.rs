use nalgebra::{UnitQuaternion, Vector3};

#[derive(Clone, Debug)]
pub(crate) struct Origin {
    pub center: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
}

impl Origin {
    pub(crate) fn new() -> Self {
        Self {
            center: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::new(Vector3::new(0., 0., 0.)),
        }
    }

    pub(crate) fn project(&self, v: Vector3<f32>) -> Vector3<f32> {
        let v = v - self.center;
        self.center + v.dot(&self.x()) * self.x() + v.dot(&self.y()) * self.y()
    }
    pub(crate) fn project_unit(&self, v: Vector3<f32>) -> Vector3<f32> {
        (v.dot(&self.x()) * self.x() + v.dot(&self.y()) * self.y()).normalize()
    }
    pub(crate) fn offset_z(mut self, amount: f32) -> Self {
        self.center = self.z() * amount + self.center;
        self
    }
    pub(crate) fn offset_x(mut self, amount: f32) -> Self {
        self.center = self.x() * amount + self.center;
        self
    }
    pub(crate) fn offset_y(mut self, amount: f32) -> Self {
        self.center = self.y() * amount + self.center;
        self
    }

    fn offset(mut self, axis: Vector3<f32>) -> Self {
        self.center = axis + self.center;
        self
    }

    fn rotate(mut self, quat: UnitQuaternion<f32>) -> Self {
        self.rotation *= quat;
        self
    }

    pub(crate) fn rotate_axisangle(mut self, axisangle: Vector3<f32>) -> Self {
        let quat = UnitQuaternion::from_scaled_axis(axisangle);
        self.rotation *= quat;
        self
    }

    pub(crate) fn left(&self) -> Vector3<f32> {
        -self.x()
    }

    pub(crate) fn right(&self) -> Vector3<f32> {
        self.x()
    }

    pub(crate) fn top(&self) -> Vector3<f32> {
        self.y()
    }

    fn bottom(&self) -> Vector3<f32> {
        -self.y()
    }

    pub(crate) fn x(&self) -> Vector3<f32> {
        self.rotation * Vector3::x()
    }
    pub(crate) fn y(&self) -> Vector3<f32> {
        self.rotation * Vector3::y()
    }

    pub(crate) fn z(&self) -> Vector3<f32> {
        self.rotation * Vector3::z()
    }
    pub(crate) fn add(&mut self, origin: &Origin) {
        self.center = origin.rotation * self.center + origin.center;
        self.rotation = self.rotation * origin.rotation;
    }
}
