use nalgebra::{Vector2, Vector3};
use std::iter;

use crate::geometry::primitives::{decimal::Dec, Segments};

pub trait Topology {
    const DIMS: usize;
    const SIDES: usize;
    type ParametricCoords;

    fn parametric_face_iterator() -> impl Iterator<Item = [Self::ParametricCoords; 3]>;
}

pub struct Four;

impl Four {
    pub fn parametric_face_iterator_t() -> impl Iterator<Item = [Vector2<Dec>; 3]> {
        Segments::new(10).flat_map(|(t, tt)| {
            Segments::new(1).flat_map(move |(s, ss)| {
                let a = Vector2::new(t, s);
                let b = Vector2::new(t, ss);
                let c = Vector2::new(tt, s);
                let d = Vector2::new(tt, ss);
                vec![[a, b, c], [b, d, c]]
            })
        })
    }
    pub fn parametric_face_iterator_s() -> impl Iterator<Item = [Vector2<Dec>; 3]> {
        Segments::new(1).flat_map(|(t, tt)| {
            Segments::new(10).flat_map(move |(s, ss)| {
                let a = Vector2::new(t, s);
                let b = Vector2::new(t, ss);
                let c = Vector2::new(tt, s);
                let d = Vector2::new(tt, ss);
                vec![[a, b, c], [b, d, c]]
            })
        })
    }
}
pub struct Three;

impl Topology for Three {
    const DIMS: usize = 1;

    const SIDES: usize = 3;

    type ParametricCoords = Vector3<Dec>;

    fn parametric_face_iterator() -> impl Iterator<Item = [Self::ParametricCoords; 3]> {
        iter::empty()
    }
}

impl Topology for Four {
    const DIMS: usize = 2;

    const SIDES: usize = 4;

    type ParametricCoords = Vector2<Dec>;

    fn parametric_face_iterator() -> impl Iterator<Item = [Self::ParametricCoords; 3]> {
        Segments::new(10).flat_map(|(t, tt)| {
            Segments::new(10).flat_map(move |(s, ss)| {
                let a = Vector2::new(t, s);
                let b = Vector2::new(t, ss);
                let c = Vector2::new(tt, s);
                let d = Vector2::new(tt, ss);
                vec![[a, b, c], [b, d, c]]
            })
        })
    }
}
