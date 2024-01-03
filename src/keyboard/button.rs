use nalgebra::Vector3;
use scad::{scad, Difference, Polyhedron, ScadObject};

use crate::geometry::primitives::origin::Origin;

#[derive(Clone)]
struct ButtonMount {
    width: f32,
    height: f32,
    lock_height: f32,
    padding: f32,
}

impl ButtonMount {
    fn chok() -> Self {
        Self {
            width: 13.8,
            height: 13.8,
            lock_height: 1.2,
            padding: 0.7,
        }
    }
}

#[derive(Clone)]
pub struct Button {
    pub origin: Origin,
    mount: ButtonMount,
}

impl Button {
    pub(crate) fn chok(origin: Origin) -> Self {
        Self {
            origin,
            mount: ButtonMount::chok(),
        }
    }

    pub(crate) fn inner_left_bottom(&self, thickness: f32) -> Vector3<f32> {
        let left = self.origin.left() * (self.mount.padding + self.mount.width / 2.0);
        let top = self.origin.y() * (self.mount.padding + self.mount.height / 2.0);
        let up = self.origin.z() * thickness / 2.0;
        left - top - up + self.origin.center
    }

    pub(crate) fn inner_left_top(&self, thickness: f32) -> Vector3<f32> {
        let left = self.origin.left() * (self.mount.padding + self.mount.width / 2.0);
        let top = self.origin.top() * (self.mount.padding + self.mount.height / 2.0);
        let up = self.origin.z() * thickness / 2.0;
        left + top - up + self.origin.center
    }

    pub(crate) fn outer_left_bottom(&self, thickness: f32) -> Vector3<f32> {
        let left = self.origin.left() * (self.mount.padding + self.mount.width / 2.0);
        let top = self.origin.top() * (self.mount.padding + self.mount.height / 2.0);
        let up = self.origin.z() * thickness / 2.0;
        left - top + up + self.origin.center
    }

    pub(crate) fn outer_left_top(&self, thickness: f32) -> Vector3<f32> {
        let left = self.origin.left() * (self.mount.padding + self.mount.width / 2.0);
        let top = self.origin.top() * (self.mount.padding + self.mount.height / 2.0);
        let up = self.origin.z() * thickness / 2.0;
        left + top + up + self.origin.center
    }

    pub(crate) fn inner_right_bottom(&self, thickness: f32) -> Vector3<f32> {
        let right = self.origin.right() * (self.mount.padding + self.mount.width / 2.0);
        let top = self.origin.top() * (self.mount.padding + self.mount.height / 2.0);
        let up = self.origin.z() * thickness / 2.0;
        right - top - up + self.origin.center
    }

    pub(crate) fn inner_right_top(&self, thickness: f32) -> Vector3<f32> {
        let right = self.origin.right() * (self.mount.padding + self.mount.width / 2.0);
        let top = self.origin.top() * (self.mount.padding + self.mount.height / 2.0);
        let up = self.origin.z() * thickness / 2.0;
        right + top - up + self.origin.center
    }

    pub(crate) fn outer_right_bottom(&self, thickness: f32) -> Vector3<f32> {
        let right = self.origin.right() * (self.mount.padding + self.mount.width / 2.0);
        let top = self.origin.top() * (self.mount.padding + self.mount.height / 2.0);
        let up = self.origin.z() * thickness / 2.0;
        right - top + up + self.origin.center
    }

    pub(crate) fn outer_right_top(&self, thickness: f32) -> Vector3<f32> {
        let right = self.origin.right() * (self.mount.padding + self.mount.width / 2.0);
        let top = self.origin.top() * (self.mount.padding + self.mount.height / 2.0);
        let up = self.origin.z() * thickness / 2.0;
        right + top + up + self.origin.center
    }
}

impl Button {
    /// Gives button mount point - box in the plain
    pub(crate) fn scad(&self, thickness: f32) -> Result<ScadObject, anyhow::Error> {
        let width = self.mount.width;
        let height = self.mount.height;
        let depth = self.mount.lock_height;
        let p = self.mount.padding * 2.0;
        let surrounds = create_simple_box(self.origin.clone(), width + p, height + p, depth)?;
        let hole = create_simple_box(self.origin.clone(), width, height, depth + 1.0)?;

        let obj = scad!(Difference; {
            surrounds,
            hole,
        });

        Ok(obj)
    }
}
pub fn create_simple_box(
    origin: Origin,
    width: f32,
    height: f32,
    depth: f32,
) -> anyhow::Result<ScadObject> {
    let y_dir = origin.y();
    let x_dir = origin.x();
    let z_dir = origin.z();
    if x_dir.magnitude() < f32::EPSILON {
        Err(anyhow::Error::msg("Button orientation conflict, row direction and plane normal are collinear. Cannot calculate corrent `left-right` orientation"))?;
    }

    let dx = x_dir * width / 2.;
    let dy = y_dir * height / 2.;
    let dz = z_dir * depth / 2.;
    let o = origin.center;

    let points = vec![
        -dx + dy + dz,
        dx + dy + dz,
        dx - dy + dz,
        -dx - dy + dz,
        -dx + dy - dz,
        dx + dy - dz,
        dx - dy - dz,
        -dx - dy - dz,
    ]
    .into_iter()
    .map(|p| p + o)
    .collect();

    let faces = vec![
        //  1
        vec![0, 1, 2, 3],
        //  2
        vec![1, 0, 4, 5],
        //  3
        vec![7, 6, 5, 4],
        //  4
        vec![3, 2, 6, 7],
        //  5
        vec![0, 3, 7, 4],
        //  6
        vec![2, 1, 5, 6],
    ];

    let polyhedron = Polyhedron(points, faces);
    Ok(ScadObject::new(polyhedron))
}
