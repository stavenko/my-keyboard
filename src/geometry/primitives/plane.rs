use nalgebra::{Rotation3, Vector3};
use scad::{Polyhedron, ScadObject};

#[derive(Clone)]
pub struct Plane {
    origin: Vector3<f32>,
    normal: Vector3<f32>,
    dir: Vector3<f32>,
}

impl Plane {
    fn offset_normal(mut self, diff: f32) -> Self {
        self.origin += self.normal * diff;
        self
    }
    fn xy0() -> Self {
        Self {
            origin: Vector3::<f32>::new(0.0, 0.0, 0.0),
            normal: Vector3::<f32>::new(0.0, 0.0, 1.0),
            dir: Vector3::<f32>::new(0.0, 1.0, 0.0),
        }
    }
    fn yz0() -> Self {
        Self {
            origin: Vector3::<f32>::new(0.0, 0.0, 0.0),
            normal: Vector3::<f32>::new(1.0, 0.0, 0.0),
            dir: Vector3::<f32>::new(0.0, 1.0, 0.0),
        }
    }
    fn xz0() -> Self {
        Self {
            origin: Vector3::<f32>::new(0.0, 0.0, 0.0),
            normal: Vector3::<f32>::new(0.0, 1.0, 0.0),
            dir: Vector3::<f32>::new(1.0, 0.0, 0.0),
        }
    }
    fn xy(origin: Vector3<f32>) -> Self {
        Self {
            origin,
            normal: Vector3::<f32>::new(0.0, 0.0, 1.0),
            dir: Vector3::<f32>::new(0.0, 1.0, 0.0),
        }
    }
    fn xz(origin: Vector3<f32>) -> Self {
        Self {
            origin,
            normal: Vector3::<f32>::new(0.0, 1.0, 0.0),
            dir: Vector3::<f32>::new(1.0, 0.0, 0.0),
        }
    }
    fn yz(origin: Vector3<f32>) -> Self {
        Self {
            origin,
            normal: Vector3::<f32>::new(1.0, 0.0, 0.0),
            dir: Vector3::<f32>::new(0.0, 1.0, 0.0),
        }
    }

    fn rotate(mut self, axis: Vector3<f32>, angle: f32) -> Self {
        let rotation = Rotation3::new(axis * angle);
        self.normal = rotation * self.normal;
        self.dir = rotation * self.dir;
        self
    }

    fn create_simple_box(&self, width: f32, height: f32, depth: f32) -> anyhow::Result<ScadObject> {
        let y_dir = self.dir;
        let x_dir = self.normal.cross(&self.dir);
        let z_dir = self.normal;
        if x_dir.magnitude() < f32::EPSILON {
            Err(anyhow::Error::msg("Button orientation conflict, row direction and plane normal are collinear. Cannot calculate corrent `left-right` orientation"))?;
        }

        let dx = x_dir * width / 2.;
        let dy = y_dir * height / 2.;
        let dz = z_dir * depth / 2.;
        let o = self.origin;

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

    fn right(&self) -> Vector3<f32> {
        self.normal.cross(&self.dir)
    }

    fn left(&self) -> Vector3<f32> {
        -self.right()
    }
}
