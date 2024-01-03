use std::{rc::Rc, sync::Arc};

use super::{bezier::BezierEdge, segment::EdgeSegment, Path};

#[derive(Debug, Clone)]
pub struct PolyPath {
    parts: Vec<Rc<dyn Path>>,
}

fn wrap<'a, T: Path + 'a>(p: T) -> Rc<dyn Path + 'a> {
    Rc::new(p)
}

impl<T> From<Vec<T>> for PolyPath
where
    T: Path + 'static,
{
    fn from(value: Vec<T>) -> Self {
        Self {
            parts: value.into_iter().map(wrap).collect(),
        }
    }
}

impl From<EdgeSegment> for PolyPath {
    fn from(value: EdgeSegment) -> Self {
        Self {
            parts: vec![wrap(value)],
        }
    }
}
impl From<BezierEdge> for PolyPath {
    fn from(value: BezierEdge) -> Self {
        Self {
            parts: vec![wrap(value)],
        }
    }
}

impl PolyPath {
    pub fn join<T: Path + 'static>(&mut self, t: T) {
        self.parts.push(wrap(t))
    }

    fn get_ix_t(&self, mut t: f32) -> (usize, f32) {
        let self_len = self.len();
        for (ix, item) in self.parts.iter().enumerate() {
            let l = item.len() / self_len;
            if t <= l {
                return (ix, t / l);
            } else {
                t -= l;
            }
        }
        unreachable!("t is bigger that 1.0 ({t})- unreachable");
    }
}

impl Path for PolyPath {
    fn get_t(&self, t: f32) -> nalgebra::Vector3<f32> {
        let (ix, t) = self.get_ix_t(t);
        self.parts
            .get(ix)
            .expect("You made it clear to respect bounds")
            .get_t(t)
    }

    fn len(&self) -> f32 {
        self.parts.iter().map(|p| p.len()).sum()
    }

    fn get_tangent(&self, t: f32) -> nalgebra::UnitVector3<f32> {
        let (ix, t) = self.get_ix_t(t);
        self.parts
            .get(ix)
            .expect("You made it clear to respect bounds")
            .get_tangent(t)
    }

    fn first(&self) -> nalgebra::Vector3<f32> {
        self.get_t(0.0)
    }

    fn last(&self) -> nalgebra::Vector3<f32> {
        self.get_t(1.0)
    }

    fn get_edge_dir(&self, t: f32) -> nalgebra::Vector3<f32> {
        let (ix, t) = self.get_ix_t(t);
        self.parts
            .get(ix)
            .expect("You made it clear to respect bounds")
            .get_edge_dir(t)
    }
}

/*
trait PathSpecificT<const N: usize>: Path {
    fn path_specific_t(&self, t: f32) -> [Option<f32>; N];
}

impl<S, T> PathSpecificT<2> for PolyPath<(S, T)>
where
    S: Path,
    T: Path,
{
    fn path_specific_t(&self, t: f32) -> [Option<f32>; 2] {
        let len = self.len::<10>();
        let f = self.0 .0.len::<10>() / len;
        let l = 1.0 - f;
        if t > f {
            let rest_t = t - f;
            let rest_t = rest_t / l;
            [None, Some(rest_t)]
        } else {
            [Some(t / f), None]
        }
    }
}
impl<S, T, U> PathSpecificT<3> for PolyPath<(S, T, U)>
where
    S: Path,
    T: Path,
    U: Path,
{
    fn path_specific_t(&self, t: f32) -> [Option<f32>; 2] {
        let len = self.len::<10>();
        let a = self.0 .0.len::<10>() / len;
        let b = self.0 .1.len::<10>() / len;
        let c = 1.0 - a - b;
        match (t > a && t <= b, t > b && t <= c) {
            (true, false) => [Some(t / a), None, None],
            (false, true) => [None, Some((t - a) / b), None],
            (false, false) => [None, None, Some((t - a - b) / c)],
            _ => unreachable!("Cannot be that"),
        }
        if t > a {
        } else {
            [Some(t / f), None]
        }
    }
}
impl<S, T> Path for PolyPath<(S, T)>
where
    S: Path,
    T: Path,
{
    fn get_t(&self, t: f32) -> nalgebra::Vector3<f32> {
        match self.path_specific_t(t) {
            [Some(t), None] => self.0 .0.get_t(t),
            [None, Some(t)] => self.0 .1.get_t(t),
            _ => unreachable!("Incorrect combination"),
        }
    }

    fn len<const D: usize>(&self) -> f32 {
        self.0 .0.len::<D>() + self.0 .1.len::<D>()
    }

    fn get_tangent(&self, t: f32) -> nalgebra::UnitVector3<f32> {
        match self.path_specific_t(t) {
            [Some(t), None] => self.0 .0.get_tangent(t),
            [None, Some(t)] => self.0 .1.get_tangent(t),
            _ => unreachable!("Incorrect combination"),
        }
    }

    fn first(&self) -> nalgebra::Vector3<f32> {
        self.0 .0.first()
    }

    fn last(&self) -> nalgebra::Vector3<f32> {
        self.0 .1.first()
    }

    fn inverse(self) -> Self {
        todo!("inverse has problems during impl");
    }

    fn get_edge_dir(&self, t: f32) -> nalgebra::Vector3<f32> {
        match self.path_specific_t(t) {
            [Some(t), None] => self.0 .0.get_edge_dir(t),
            [None, Some(t)] => self.0 .1.get_edge_dir(t),
            _ => unreachable!("Incorrect combination"),
        }
    }
}
*/
