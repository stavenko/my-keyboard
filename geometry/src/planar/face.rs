use nalgebra::Vector3;
use num_traits::Zero;
use stl_io::{Triangle, Vector};

use crate::decimal::Dec;

use super::plane::Plane;

pub struct TriangleWrap(Triangle);

#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    pub vertices: [Vector3<Dec>; 3],
    pub normal: Vector3<Dec>,
}

impl Face {
    pub fn new_with_normal(vertices: [Vector3<Dec>; 3], normal: Vector3<Dec>) -> Self {
        Face { vertices, normal }
    }
    pub fn get_plane(&self) -> Plane {
        let [u, _v, _w] = &self.vertices;
        let origin = *u;
        Plane::new_from_normal_and_point(self.normal, origin)
    }
    pub fn get_normal(&self) -> Vector3<Dec> {
        self.normal
    }

    pub fn new(vertices: [Vector3<Dec>; 3]) -> Self {
        let [u, v, w] = &vertices;
        let _origin = *u;
        let a = (v - u);
        let b = (w - u);

        let cross = a.cross(&b);

        if dbg!(cross.magnitude_squared().is_zero()) {
            dbg!(a.dot(&b));
            panic!("aaa {u} , {v} , {w} ({a} x {b} , {cross}");
        }
        let normal = cross.normalize();
        Self { vertices, normal }
    }
}

impl From<Face> for TriangleWrap {
    fn from(value: Face) -> Self {
        let normal = value.get_normal();
        let normal = Vector::new([normal.x.into(), normal.y.into(), normal.z.into()]);
        let vertices = value
            .vertices
            .map(|na_vec| Vector::new([na_vec.x.into(), na_vec.y.into(), na_vec.z.into()]));
        TriangleWrap(Triangle { normal, vertices })
    }
}
