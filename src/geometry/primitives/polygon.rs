use core::panic;

use anyhow::anyhow;
use nalgebra::{ComplexField, Vector3};
use num_traits::Zero;
use rust_decimal_macros::dec;

use super::{decimal::Dec, plane::Plane, Face};

#[derive(Clone, Debug, PartialEq)]
pub struct Polygon {
    pub vertices: Vec<Vector3<Dec>>,
    normal: Vector3<Dec>,
}

impl Polygon {
    pub fn triangles(self) -> Vec<Face> {
        let mut result = Vec::new();
        for i in 1..(self.vertices.len() - 1) {
            let v0 = self.vertices[0];
            let v1 = self.vertices[i];
            let v2 = self.vertices[i + 1];

            result.push(Face::new_with_normal([v0, v1, v2], self.normal));
        }

        result
    }
    fn find_point_position(&self, p: &Vector3<Dec>) -> Option<usize> {
        self.vertices.iter().position(|v| v == p)
    }
    pub fn join(self, mut other: Self) -> anyhow::Result<Self> {
        let p = other.vertices[0];
        let d = (p - self.vertices[0]).dot(&self.normal);
        if self.normal.dot(&other.normal) > Zero::zero() && d.abs() < dec!(1e-6).into() {
            let normal = self.normal;
            let mut i = other.vertices.iter().enumerate();
            if let Some((my_pos, other_pos)) = loop {
                if let Some((pos_other, f)) = i.next() {
                    let my_pos = self.find_point_position(f);
                    if my_pos.is_some() {
                        break my_pos.map(|my_pos| (my_pos, pos_other));
                    }
                } else {
                    break None;
                }
            } {
                let (before, after) = self.vertices.split_at(my_pos + 1);
                other.vertices.rotate_left(other_pos);
                let middle: &[Vector3<Dec>] =
                    &other.vertices.as_slice()[1..other.vertices.len() - 1];

                let vertices = [before, &middle, after].concat();
                return Ok(Self { vertices, normal });
            }
        }
        Err(anyhow!("not pretty"))
    }
    pub fn new(vertices: Vec<Vector3<Dec>>) -> anyhow::Result<Self> {
        let u = vertices.get(0).ok_or(anyhow!("not a single point"))?;
        let v = vertices.get(1).ok_or(anyhow!("only one point"))?;
        let w = vertices.get(2).ok_or(anyhow!("need 3 points at least"))?;

        let origin = *u;
        let a = v - u;
        let b = w - u;

        let cross = &a.cross(&b);

        if cross.magnitude() == Dec::zero() {
            Err(anyhow!("points to close - cannot calculate normal"))
        } else {
            let normal = cross.normalize();
            Ok(Self { vertices, normal })
        }
    }
    pub fn new_with_normal(vertices: Vec<Vector3<Dec>>, normal: Vector3<Dec>) -> Self {
        Self { vertices, normal }
    }

    pub fn get_plane(&self) -> anyhow::Result<Plane> {
        let u = self.vertices.get(0).ok_or(anyhow!("not a single point"))?;
        let origin = *u;

        Ok(Plane {
            origin,
            normal: self.normal,
        })
    }

    pub(crate) fn flip(mut self) -> Self {
        let normal = -self.normal;
        self.vertices.reverse();
        Self {
            vertices: self.vertices,
            normal,
        }
    }

    pub fn get_normal(&self) -> Vector3<Dec> {
        self.normal
    }

    pub(crate) fn calculate_triangles(&self) -> usize {
        self.vertices.len() - 2
    }
}
