use std::{borrow::Borrow, slice::Iter};

use nalgebra::{ComplexField, Vector2, Vector3};
use num_traits::{Pow, Zero};
use stl_io::{Triangle, Vector};

use self::{decimal::Dec, plane::Plane};

pub mod decimal;
pub mod origin;
pub mod plane;
pub mod polygon;

#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    vertices: [Vector3<Dec>; 3],
    normal: Vector3<Dec>,
}

impl Face {
    fn new_with_normal(vertices: [Vector3<Dec>; 3], normal: Vector3<Dec>) -> Self {
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
    /*
    pub fn map(&self, f: impl Fn(Vector3<Dec>) -> Vector3<Dec>) -> Self {
        Self(self.0.map(f))
    }
    */

    pub fn iter(&self) -> Iter<'_, Vector3<Dec>> {
        self.vertices.iter()
    }

    pub fn new(vertices: [Vector3<Dec>; 3]) -> Self {
        let [u, v, w] = &vertices;
        let origin = *u;
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
        let [u, v, w] = &self.vertices;
        let origin = *u;
        Plane {
            origin,
            normal: self.normal,
        }
    }

    pub(crate) fn flip(self) -> Face {
        let [a, b, c] = self.vertices;
        let normal = -self.normal;
        Self {
            vertices: [a, c, b],
            normal,
        }
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

pub type Line = [Vector2<Dec>; 2];
pub type LineIx = [usize; 2];

#[derive(Clone)]
pub struct PointInPlane<T> {
    pub point: Vector3<T>,
    pub normal: Vector3<T>,
    pub dir: Option<Vector3<T>>,
}

pub struct Segments {
    segments: usize,
    current_segment: usize,
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

impl<const D: usize> IndexIterator<D> {
    pub fn new() -> Self {
        Self(0)
    }
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
#[derive(Debug, PartialEq)]
pub struct Segment<T> {
    pub from: T,
    pub to: T,
}
#[cfg(test)]
mod tests {
    use super::Segments;

    #[test]
    fn simple_segments() {
        let s = Segments::new(2).collect::<Vec<_>>();
        assert_eq!(s, vec!((0.into(), 0.5.into()), (0.5.into(), 1.0.into())));
        let s = Segments::new(3).collect::<Vec<_>>();
        assert_eq!(
            s,
            vec![
                (0.0.into(), 0.33333334.into()),
                (0.33333334.into(), 0.6666667.into()),
                (0.6666667.into(), 1.0.into())
            ]
        );
    }
}
