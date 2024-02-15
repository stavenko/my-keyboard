use nalgebra::{UnitVector3, Vector3};
use rust_decimal::prelude::One;

use crate::{
    path::Path,
    primitives::{decimal::Dec, Segments},
};

use super::PathInverse;
use num_traits::Pow;

#[derive(Clone, Debug)]
pub struct BezierEdge {
    pub base: [Vector3<Dec>; 4],
    edge_force: [Vector3<Dec>; 4],
    len_cache: Option<Dec>,
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
    pub fn new(base: [Vector3<Dec>; 4], edge_force: [Vector3<Dec>; 4]) -> Self {
        let mut b = BezierEdge {
            base,
            edge_force,
            len_cache: None,
            quality: 20,
        };

        b.len_cache = Some(b.len());
        b
    }

    pub fn new_simple(base: [Vector3<Dec>; 4]) -> Self {
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
    fn get_edge_dir(&self, t: Dec) -> Vector3<Dec> {
        let ot = Dec::one() - t;
        let weights = [
            ot.pow(3i64),
            Dec::from(3) * ot.pow(2i64) * t,
            Dec::from(3) * ot * t.pow(2i64),
            t.pow(3i64),
        ];

        self.edge_force
            .into_iter()
            .zip(weights)
            .map(|(v, w)| v * w)
            .sum()
    }
    fn get_t(&self, t: Dec) -> Vector3<Dec> {
        let ot = Dec::one() - t;
        let weights = [
            ot.pow(3i64),
            Dec::from(3) * ot.pow(2i64) * t,
            Dec::from(3) * ot * t.pow(2i64),
            t.pow(3i64),
        ];

        self.base.into_iter().zip(weights).map(|(v, w)| v * w).sum()
    }

    fn len(&self) -> Dec {
        if let Some(l) = self.len_cache {
            l
        } else {
            Segments::new(40)
                .map(|(f, l)| self.get_t(f) - self.get_t(l))
                .map(|line| line.magnitude())
                .sum()
        }
    }

    fn get_tangent(&self, t: Dec) -> Vector3<Dec> {
        let dt = Dec::from(0.0001); // TODO something with t == 1.0
        let t1 = t + dt;
        let v = self.get_t(t1) - self.get_t(t);
        v.normalize()
    }

    fn first(&self) -> Vector3<Dec> {
        self.base[0]
    }

    fn last(&self) -> Vector3<Dec> {
        self.base[3]
    }
}
