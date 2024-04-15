use nalgebra::Vector3;

use crate::decimal::Dec;

use super::octree::{Node, Octree};

#[derive(Default, Debug)]
pub struct VertexIndex {
    octree: Octree<usize>,
    points: Vec<Vector3<Dec>>,
}

pub const MAX_DIGITS: usize = 10;

#[derive(Debug, PartialEq, Eq, PartialOrd, Hash, Clone, Copy)]
pub struct PtId(usize);

impl VertexIndex {
    fn vertex(mut v: Vector3<Dec>, quantification: usize) -> Vector3<Dec> {
        v.x = v.x.round_dp(quantification as u32);
        v.y = v.y.round_dp(quantification as u32);
        v.z = v.z.round_dp(quantification as u32);
        v
    }
    pub fn get_vertex_index(&mut self, vertex: Vector3<Dec>) -> PtId {
        let vertex = Self::vertex(vertex, MAX_DIGITS);
        if let Some(n) = self.octree.find(vertex) {
            //println!("@HAS PT {}", n.data);
            PtId(n.data)
        } else {
            self.points.push(vertex);
            let id = self.points.len() - 1;
            //println!("@NEW PT {id}");
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
}

impl From<usize> for PtId {
    fn from(value: usize) -> Self {
        Self(value)
    }
}
