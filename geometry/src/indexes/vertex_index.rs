use core::fmt;

use itertools::Itertools;
use nalgebra::Vector3;

use crate::decimal::Dec;

use super::{
    aabb::Aabb,
    octree::{Node, Octree},
    sphere::Sphere,
};

#[derive(Debug)]
pub struct VertexIndex {
    octree: Octree<usize>,
    points: Vec<Vector3<Dec>>,
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
pub struct PtId(usize);

impl PartialEq<usize> for PtId {
    fn eq(&self, other: &usize) -> bool {
        self.0 == *other
    }
}

impl fmt::Display for PtId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl VertexIndex {
    pub fn new(aabb: Aabb) -> Self {
        Self {
            octree: Octree::<usize>::new_with_aabb(Vec::new(), aabb),
            points: Vec::new(),
        }
    }

    pub fn get_or_insert_point(&mut self, vertex: Vector3<Dec>, separation_distance: Dec) -> PtId {
        if let Some(n) = self.find_closest(vertex, separation_distance) {
            n
        } else {
            self.points.push(vertex);
            let id = self.points.len() - 1;
            let node = Node {
                data: id,
                point: vertex,
            };
            self.octree.insert(node);
            PtId(id)
        }
    }

    pub fn get_point(&self, ix: PtId) -> Vector3<Dec> {
        self.points[ix.0]
    }

    pub fn print_all(&self) {
        for (i, vertex) in self.points.iter().enumerate() {
            println!("v:{i} {} {} {} ", vertex.x, vertex.y, vertex.z);
        }
    }

    pub fn get_vertex_array(&self) -> Vec<[f64; 3]> {
        self.points
            .clone()
            .into_iter()
            .map(|vec| [vec.x.into(), vec.y.into(), vec.z.into()])
            .collect_vec()
    }

    pub fn find_closest(&self, center: Vector3<Dec>, distance: Dec) -> Option<PtId> {
        let mut points = self.octree.query_within_sphere(Sphere {
            center,
            radius: distance,
        });
        points.sort_by_key(|node| (node.point - center).magnitude_squared());
        points.first().map(|node| PtId(node.data))
    }

    pub(crate) fn set_initial_bb(&mut self, aabb: super::aabb::Aabb) {
        if self.octree.is_empty() {
            self.octree.set_aabb(aabb);
        }
    }
}

impl From<usize> for PtId {
    fn from(value: usize) -> Self {
        Self(value)
    }
}

impl From<PtId> for usize {
    fn from(value: PtId) -> Self {
        value.0
    }
}
