use bezier_nd::Bezier;
use nalgebra::Vector3;

use crate::geometry::{
    path::{bezier::BezierEdge, segment::EdgeSegment, Path},
    primitives, Geometry,
};

use super::{
    topology::{Three, Topology},
    GetBoundingPath, Surface,
};

pub struct TriBezier {
    vertices: [Vector3<f32>; 10],
}

impl GetBoundingPath<0> for TriBezier {
    fn get_bounding_path(&self) -> impl Path {
        BezierEdge::new_simple(self.vertices[0..4].try_into().expect("four"))
    }
}
impl GetBoundingPath<1> for TriBezier {
    fn get_bounding_path(&self) -> impl Path {
        let c = [
            self.vertices[0],
            self.vertices[4],
            self.vertices[7],
            self.vertices[9],
        ];
        BezierEdge::new_simple(c)
    }
}
impl GetBoundingPath<2> for TriBezier {
    fn get_bounding_path(&self) -> impl Path {
        let c = [
            self.vertices[3],
            self.vertices[6],
            self.vertices[8],
            self.vertices[9],
        ];
        BezierEdge::new_simple(c)
    }
}

fn fact(n: u8) -> f32 {
    match n {
        0 => 1.0,
        1 => 1.0,
        2 => 2.0,
        3 => 6.0,
        _ => unreachable!("impossibly"),
    }
}

impl TriBezier {
    pub fn new(vertices: [Vector3<f32>; 10]) -> Self {
        Self { vertices }
    }

    /// ----------------------------
    ///      *
    ///     / \
    ///   1/   \2
    ///   /     \
    ///  /       \
    /// *---0-----*
    pub fn new_from_bezier(curves: [BezierEdge; 3], ext: f32) -> Self {
        let mids = curves.clone().map(|path| path.get_t(0.5));
        let u = mids[0] - mids[1];
        let v = mids[2] - mids[0];
        let n = u.cross(&v).normalize();
        let c = mids.into_iter().sum::<Vector3<f32>>() / 3.0;
        let central_point = c + n * ext;
        let l0: &[Vector3<f32>] = &curves[0].base;
        let l1: &[Vector3<f32>] = &[curves[1].base[1], central_point, curves[2].base[1]];
        let l2: &[Vector3<f32>] = &[curves[1].base[2], curves[2].base[2]];
        let l3: &[Vector3<f32>] = &[curves[1].base[3]];

        let vertices = [l0, l1, l2, l3]
            .concat()
            .try_into()
            .expect("10 points - for sure");
        Self { vertices }
    }

    pub fn get_vertex(&self, ix: [u8; 3]) -> Vector3<f32> {
        match ix {
            [3, 0, 0] => self.vertices[0],
            [2, 1, 0] => self.vertices[1],
            [1, 2, 0] => self.vertices[2],
            [0, 3, 0] => self.vertices[3],

            [2, 0, 1] => self.vertices[4],
            [1, 1, 1] => self.vertices[5],
            [0, 2, 1] => self.vertices[6],

            [1, 0, 2] => self.vertices[7],
            [0, 1, 2] => self.vertices[8],

            [0, 0, 3] => self.vertices[9],
            x => unreachable!("check your code, dude {:?}", x),
        }
    }
    pub fn get_ix(&self, ix: usize) -> [u8; 3] {
        match ix {
            0 => [3, 0, 0],
            1 => [2, 1, 0],
            2 => [1, 2, 0],
            3 => [0, 3, 0],

            4 => [2, 0, 1],
            5 => [1, 1, 1],
            6 => [0, 2, 1],

            7 => [1, 0, 2],
            8 => [0, 1, 2],

            9 => [0, 0, 3],

            x => unreachable!("check your code, dude {:?}", x),
        }
    }

    pub fn bernstein(ix: [u8; 3], t: [f32; 3]) -> f32 {
        let pow: f32 = ix
            .into_iter()
            .zip(t)
            .map(|(p, t)| t.powi(p as i32))
            .product();

        let n = ix.iter().sum();
        let ff = fact(n) / (fact(ix[0]) * fact(ix[1]) * fact(ix[2]));
        pow * ff
    }

    pub fn get_weight(&self, t: [f32; 3], ix: usize) -> f32 {
        let ix = self.get_ix(ix);
        let k = Self::bernstein(ix, t);

        k
    }

    pub fn get_point(&self, t: [f32; 3]) -> Vector3<f32> {
        self.vertices
            .iter()
            .enumerate()
            .map(|(ix, v)| self.get_weight(t, ix) * v)
            .sum()
    }
}

fn uvw<const D: usize>(i: usize, j: usize) -> [f32; 3] {
    let u = i as f32 / D as f32;
    let v = (1.0 - u) * (1.0 - (j as f32 / D as f32));
    let w = 1.0 - u - v;
    [v, w, u]
}

impl Geometry for TriBezier {
    fn polygonize(&self) -> anyhow::Result<Vec<primitives::Face>> {
        let mut faces = Vec::new();
        for i in 1..=10 {
            for j in 1..=10 {
                let ninj = uvw::<10>(i - 1, j - 1);
                let nij = uvw::<10>(i - 1, j);
                let inj = uvw::<10>(i, j - 1);
                let ij = uvw::<10>(i, j);
                let a = self.get_point(ninj);
                let b = self.get_point(nij);
                let c = self.get_point(inj);
                let d = self.get_point(ij);

                faces.push([a, b, c]);
                faces.push([b, d, c]);
            }
        }
        dbg!("poly");
        Ok(faces)
    }
}

impl Surface<Three> for TriBezier {
    fn get_point(
        &self,
        coords: <Three as Topology>::ParametricCoords,
    ) -> anyhow::Result<Vector3<f32>> {
        dbg!("get_p");
        Ok(Self::get_point(self, [coords.x, coords.y, coords.z]))
    }

    fn get_curve_at_param(&self, _param: f32) -> impl Path {
        dbg!("GCAP");
        EdgeSegment {
            from: Vector3::zeros(),
            to: Vector3::zeros(),
            edge_from: Vector3::zeros(),
            edge_to: Vector3::zeros(),
        }
    }
}
