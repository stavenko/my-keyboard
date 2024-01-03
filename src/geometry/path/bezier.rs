use nalgebra::{UnitVector3, Vector3};

use crate::geometry::{path::Path, primitives::Segments};

use super::PathInverse;

#[derive(Clone, Debug)]
pub struct BezierEdge {
    pub base: [Vector3<f32>; 4],
    edge_force: [Vector3<f32>; 4],
    len_cache: Option<f32>,
    quality: usize,
}

impl PathInverse for BezierEdge {
    fn inverse(self) -> Self {
        Self {
            base: [self.base[3], self.base[2], self.base[1], self.base[0]],
            edge_force: [
                self.edge_force[3],
                self.edge_force[2],
                self.edge_force[1],
                self.edge_force[0],
            ],
            len_cache: self.len_cache,
            quality: self.quality,
        }
    }
}

impl BezierEdge {
    pub fn new(base: [Vector3<f32>; 4], edge_force: [Vector3<f32>; 4]) -> Self {
        let mut b = BezierEdge {
            base,
            edge_force,
            len_cache: None,
            quality: 20,
        };

        b.len_cache = Some(b.len());
        b
    }

    pub fn new_simple(base: [Vector3<f32>; 4]) -> Self {
        let zeros = Vector3::zeros();
        let mut b = BezierEdge {
            base,
            edge_force: [zeros, zeros, zeros, zeros],
            len_cache: None,
            quality: 20,
        };
        b.len_cache = Some(b.len());
        b
    }
}

impl Path for BezierEdge {
    fn get_edge_dir(&self, t: f32) -> Vector3<f32> {
        let ot = 1.0 - t;
        let weights = [
            ot.powi(3),
            3. * ot.powi(2) * t,
            3. * ot * t.powi(2),
            t.powi(3),
        ];

        self.edge_force
            .into_iter()
            .zip(weights)
            .map(|(v, w)| v * w)
            .sum()
    }
    fn get_t(&self, t: f32) -> Vector3<f32> {
        let ot = 1.0 - t;
        let weights = [
            ot.powi(3),
            3. * ot.powi(2) * t,
            3. * ot * t.powi(2),
            t.powi(3),
        ];

        self.base.into_iter().zip(weights).map(|(v, w)| v * w).sum()
    }

    fn len(&self) -> f32 {
        if let Some(l) = self.len_cache {
            l
        } else {
            let l = Segments::new(40)
                .map(|(f, l)| self.get_t(f) - self.get_t(l))
                .map(|line| line.magnitude())
                .sum();
            l
        }
    }

    fn get_tangent(&self, t: f32) -> nalgebra::UnitVector3<f32> {
        let dt = 0.001; // TODO something with t == 1.0
        let t1 = t + dt;
        let v = self.get_t(t1) - self.get_t(t);
        UnitVector3::new_normalize(v)
    }

    fn first(&self) -> Vector3<f32> {
        self.base[0]
    }

    fn last(&self) -> Vector3<f32> {
        self.base[3]
    }
}
