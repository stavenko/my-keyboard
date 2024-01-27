use std::collections::VecDeque;

use anyhow::anyhow;
use itertools::{Either, Itertools};
use nalgebra::{ComplexField, Vector2, Vector3};
use num_traits::Zero;
use rayon::current_num_threads;
use rust_decimal_macros::dec;

use crate::geometry::path::segment;

use super::{decimal::Dec, plane::Plane, segment2d::Segment2D, Face};

#[derive(Clone, Debug, PartialEq)]
pub struct Polygon {
    pub vertices: Vec<Vector3<Dec>>,
    basis: Basis2d,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Basis2d {
    pub origin: Vector3<Dec>,
    pub x: Vector3<Dec>,
    pub y: Vector3<Dec>,
    pub z: Vector3<Dec>,
}

impl IntoIterator for Polygon {
    type Item = Segment2D;

    type IntoIter = <Vec<Segment2D> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        let mut vv = self.vertices.iter().peekable();
        let mut segments = Vec::new();
        let first: Vector3<Dec> = self.vertices[0];
        while let Some(v) = vv.next() {
            let prev = self.project(v);

            let next: Vector3<Dec> = if let Some(p) = vv.peek() { **p } else { first };
            let next = self.project(&next);
            segments.push(Segment2D::new(prev, next));
        }
        segments.into_iter()
    }
}

impl Polygon {
    pub fn get_basis_2d(&self) -> Basis2d {
        self.basis.clone()
    }

    pub fn calculate_basis_2d(vertices: &[Vector3<Dec>]) -> anyhow::Result<Basis2d> {
        let u = vertices.first().ok_or(anyhow!("not a single point"))?;
        let v = vertices.get(1).ok_or(anyhow!("only one point"))?;
        let w = vertices.get(2).ok_or(anyhow!("need 3 points at least"))?;

        let a = v - u;
        let b = w - u;

        let cross = &a.cross(&b);
        if cross.magnitude() == dec!(0).into() {
            Err(anyhow::anyhow!("Normal to this polyton calculated as 0"))?;
        }
        let normal = cross.normalize();
        let sum: Vector3<Dec> = vertices.iter().copied().fold(Vector3::zero(), |a, b| a + b);
        let center = sum / Dec::from(vertices.len());
        let plane_x = (u - center).normalize();
        let plane_y = plane_x.cross(&normal).normalize();
        Ok(Basis2d {
            origin: center,
            x: plane_x,
            y: plane_y,
            z: normal,
        })
    }

    fn restore_from_2d(
        points: Vec<Vector2<Dec>>,
        Basis2d { origin, x, y, .. }: Basis2d,
    ) -> anyhow::Result<Self> {
        Self::new(
            points
                .into_iter()
                .map(|p| x * p.x + y * p.y + origin)
                .collect(),
        )
    }

    fn join_segments(segments: Vec<Segment2D>) -> Vec<Segment2D> {
        let mut result = Vec::new();
        let mut segments: VecDeque<Segment2D> = segments.into();
        while let Some(b) = segments.pop_front() {
            if let Some(n) = segments.pop_front() {
                match b.join(n) {
                    Either::Left(s) => {
                        segments.push_front(s);
                    }
                    Either::Right((b, n)) => {
                        result.push(b);
                        segments.push_front(n);
                    }
                }
            } else {
                result.push(b);
                break;
            }
        }
        result
    }

    pub fn from_segments(
        mut segments: Vec<Segment2D>,
        basis: Basis2d,
    ) -> (anyhow::Result<Self>, Vec<Segment2D>) {
        let mut ordered_segments: Vec<Segment2D> = Vec::new();

        let mut tails = Vec::new();
        loop {
            let mut taken = 0;
            for current_segment in segments {
                if let Some(last) = ordered_segments.last() {
                    if current_segment.from == last.to {
                        ordered_segments.push(current_segment);
                        taken += 1;
                    } else {
                        tails.push(current_segment)
                    }
                } else {
                    ordered_segments.push(current_segment);
                    taken += 1;
                }
            }
            if tails.is_empty() {
                break;
            }
            if taken == 0 {
                break;
            }
            segments = tails;
            tails = Vec::new()
        }
        let vertices = Self::join_segments(ordered_segments)
            .into_iter()
            .map(|s| s.from)
            .collect_vec();

        let poly = Self::restore_from_2d(vertices, basis);
        (poly, tails)
    }

    fn project(&self, point: &Vector3<Dec>) -> Vector2<Dec> {
        let Basis2d { origin, x, y, .. } = self.get_basis_2d();
        let x = (point - origin).dot(&x);
        let y = (point - origin).dot(&y);
        Vector2::new(x, y)
    }
    pub fn triangles(self) -> Vec<Face> {
        let mut result = Vec::new();
        for i in 1..(self.vertices.len() - 1) {
            let v0 = self.vertices[0];
            let v1 = self.vertices[i];
            let v2 = self.vertices[i + 1];

            result.push(Face::new_with_normal([v0, v1, v2], self.basis.z));
        }

        result
    }

    fn find_point_position(&self, p: &Vector3<Dec>) -> Option<usize> {
        self.vertices.iter().position(|v| v == p)
    }

    pub fn join(self, mut other: Self) -> anyhow::Result<Self> {
        let mut basis = self.basis.clone();
        let p = other.vertices[0];
        let d = (p - self.vertices[0]).dot(&basis.z);
        if basis.z.dot(&other.basis.z) > Zero::zero() && d.abs() < dec!(1e-6).into() {
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
                let center: Vector3<Dec> =
                    vertices.iter().copied().fold(Vector3::zero(), |a, b| a + b)
                        / Dec::from(vertices.len());
                basis.origin = center;
                return Ok(Self { vertices, basis });
            }
        }
        Err(anyhow!("not pretty"))
    }

    pub fn new(vertices: Vec<Vector3<Dec>>) -> anyhow::Result<Self> {
        let basis = Self::calculate_basis_2d(&vertices)?;
        Ok(Self { vertices, basis })
    }

    pub fn new_with_basis(vertices: Vec<Vector3<Dec>>, basis: Basis2d) -> Self {
        Self { vertices, basis }
    }

    pub fn get_plane(&self) -> anyhow::Result<Plane> {
        Ok(Plane {
            origin: self.basis.origin,
            normal: self.basis.z,
        })
    }

    pub(crate) fn flip(mut self) -> Self {
        let mut basis = self.basis;
        basis.z = -basis.z;

        self.vertices.reverse();
        Self {
            vertices: self.vertices,
            basis,
        }
    }

    pub fn get_normal(&self) -> Vector3<Dec> {
        self.basis.z
    }

    pub(crate) fn calculate_triangles(&self) -> usize {
        self.vertices.len() - 2
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use rust_decimal_macros::dec;

    use crate::geometry::primitives::{polygon::Polygon, segment2d::Segment2D};

    #[test]
    fn check2d_polygon_assemble() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly = Polygon::new(points.to_vec()).unwrap();
        let segments: Vec<Segment2D> = poly.clone().into_iter().collect();
        let mut ps = Vec::new();
        for s in segments.iter() {
            ps.push(s.from);
        }
        let b = poly.get_basis_2d();
        let pp = Polygon::restore_from_2d(ps, b).unwrap();
        assert_eq!(pp, poly);
    }
}
