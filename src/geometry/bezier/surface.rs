use nalgebra::Vector3;

use crate::geometry::{path::bezier::BezierEdge, primitives::Segments, Geometry};

use super::bernstein;

pub struct BezierSurface<const D: usize>(pub [[Vector3<f32>; D]; D]);

impl BezierSurface<4> {
    fn new(input: [[Vector3<f32>; 4]; 4]) -> Self {
        Self(input)
    }
}

impl<const D: usize> Geometry<D> for BezierSurface<4> {
    fn polygonize(&self) -> anyhow::Result<Vec<crate::geometry::primitives::Face>> {
        Ok(Segments::new(D)
            .flat_map(|(t, tt)| {
                Segments::new(D).flat_map(move |(s, ss)| {
                    let a = self.get_point(t, s);
                    let b = self.get_point(t, ss);
                    let c = self.get_point(tt, s);
                    let d = self.get_point(tt, ss);
                    vec![[a, b, c], [b, d, c]]
                })
            })
            .collect())
    }
}

impl BezierSurface<4> {
    pub(crate) fn top_curve(&self) -> BezierEdge {
        BezierEdge::new_simple(self.0[0])
    }
    pub(crate) fn bottom_curve(&self) -> BezierEdge {
        BezierEdge::new_simple(self.0[3])
    }
    pub(crate) fn left_curve(&self) -> BezierEdge {
        BezierEdge::new_simple([self.0[0][0], self.0[1][0], self.0[2][0], self.0[3][0]])
    }
    pub(crate) fn right_curve(&self) -> BezierEdge {
        BezierEdge::new_simple([self.0[0][3], self.0[1][3], self.0[2][3], self.0[3][3]])
    }

    fn get_point(&self, t: f32, s: f32) -> Vector3<f32> {
        let weights = [
            [
                weight::<4>(0, 0, t, s),
                weight::<4>(1, 0, t, s),
                weight::<4>(2, 0, t, s),
                weight::<4>(3, 0, t, s),
            ],
            [
                weight::<4>(0, 1, t, s),
                weight::<4>(1, 1, t, s),
                weight::<4>(2, 1, t, s),
                weight::<4>(3, 1, t, s),
            ],
            [
                weight::<4>(0, 2, t, s),
                weight::<4>(1, 2, t, s),
                weight::<4>(2, 2, t, s),
                weight::<4>(3, 2, t, s),
            ],
            [
                weight::<4>(0, 3, t, s),
                weight::<4>(1, 3, t, s),
                weight::<4>(2, 3, t, s),
                weight::<4>(3, 3, t, s),
            ],
        ];

        self.0
            .into_iter()
            .zip(weights)
            .map(|(row, row_w)| {
                row.into_iter()
                    .zip(row_w)
                    .map(|(v, w)| v * w)
                    .sum::<Vector3<f32>>()
            })
            .sum()
    }
}

fn weight<const D: usize>(i: usize, j: usize, t: f32, s: f32) -> f32 {
    bernstein::<D>(i, t) * bernstein::<D>(j, s)
}
