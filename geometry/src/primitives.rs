use std::{borrow::Borrow, slice::Iter};

use nalgebra::Vector3;
use num_traits::Zero;
use stl_io::{Triangle, Vector};

use crate::{decimal::Dec, planar::plane::Plane};

#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    pub vertices: [Vector3<Dec>; 3],
    pub normal: Vector3<Dec>,
}

impl Face {
    pub fn new_with_normal(vertices: [Vector3<Dec>; 3], normal: Vector3<Dec>) -> Self {
        Face { vertices, normal }
    }
}
pub struct TriangleWrap(Triangle);

impl Borrow<Triangle> for TriangleWrap {
    fn borrow(&self) -> &Triangle {
        &self.0
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

impl IntoIterator for Face {
    type Item = Vector3<Dec>;

    type IntoIter = <[Vector3<Dec>; 3] as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.vertices.into_iter()
    }
}

impl Face {
    pub fn iter(&self) -> Iter<'_, Vector3<Dec>> {
        self.vertices.iter()
    }

    pub fn new(vertices: [Vector3<Dec>; 3]) -> Self {
        let [u, v, w] = &vertices;
        let _origin = *u;
        let a = v - u;
        let b = w - u;

        let cross = &a.cross(&b);

        if cross.magnitude() == Dec::zero() {
            panic!("aaa {u} x {v} x {w}");
        }
        let normal = cross.normalize();
        Self { vertices, normal }
    }

    pub fn get_plane(&self) -> Plane {
        let [u, _v, _w] = &self.vertices;
        let origin = *u;
        Plane::new_from_normal_and_point(self.normal, origin)
    }

    fn get_normal(&self) -> Vector3<Dec> {
        self.normal
    }
}

impl From<[Vector3<Dec>; 3]> for Face {
    fn from(value: [Vector3<Dec>; 3]) -> Self {
        Face::new(value)
    }
}

pub type LineIx = [usize; 2];

#[derive(Clone)]
pub struct PointInPlane<T> {
    pub point: Vector3<T>,
    pub normal: Vector3<T>,
    pub dir: Option<Vector3<T>>,
}

pub struct IndexIterator<const D: usize>(usize);

impl<const D: usize> Iterator for IndexIterator<D> {
    type Item = (usize, usize);

    fn next(&mut self) -> Option<Self::Item> {
        if self.0 < D - 1 {
            self.0 += 1;
            Some((self.0 - 1, self.0))
        } else {
            None
        }
    }
}

impl<const D: usize> Default for IndexIterator<D> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const D: usize> IndexIterator<D> {
    pub fn new() -> Self {
        Self(0)
    }
}

pub struct Segments {
    segments: usize,
    current_segment: usize,
}

impl Segments {
    pub(crate) fn new(segments: usize) -> Self {
        Self {
            segments,
            current_segment: 0,
        }
    }
}

impl Iterator for Segments {
    type Item = (Dec, Dec);

    fn next(&mut self) -> Option<Self::Item> {
        let first = self.current_segment;
        let next = first + 1;
        self.current_segment += 1;
        if next > self.segments {
            None
        } else {
            let first = Dec::from(first) / Dec::from(self.segments);
            let next = Dec::from(next) / Dec::from(self.segments);
            Some((first, next))
        }
    }
}
