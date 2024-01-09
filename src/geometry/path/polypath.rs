use std::{rc::Rc, sync::Arc};

use nalgebra::{Vector2, Vector3};
use num_traits::{One, Zero};

use crate::geometry::{
    primitives::{decimal::Dec, origin::Origin, Segments},
    Geometry2D,
};

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
    fn get_plane(&self) -> (Vector3<Dec>, Vector3<Dec>) {
        let a = self.get_t(Dec::from(0.333));
        let b = self.get_t(Dec::from(0.5));
        let c = self.get_t(Dec::from(0.6667));
        let u = a - b;
        let v = a - c;
        let c = a + b + c / Dec::from(3.0);
        let n = u.cross(&v).normalize();
        (c, n)
    }

    fn get_ix_t(&self, mut t: Dec) -> (usize, Dec) {
        let self_len = self.len();
        for (ix, item) in self.parts.iter().enumerate() {
            let l = item.len() / self_len;
            if t <= l {
                return (ix, t / l);
            } else {
                t -= l;
            }
        }
        (self.parts.len() - 1, Dec::one())
    }
}

impl Path for PolyPath {
    fn get_t(&self, t: Dec) -> Vector3<Dec> {
        let (ix, t) = self.get_ix_t(t);
        self.parts
            .get(ix)
            .expect("You made it clear to respect bounds")
            .get_t(t)
    }

    fn len(&self) -> Dec {
        self.parts.iter().map(|p| p.len()).sum()
    }

    fn get_tangent(&self, t: Dec) -> Vector3<Dec> {
        let (ix, t) = self.get_ix_t(t);
        self.parts
            .get(ix)
            .expect("You made it clear to respect bounds")
            .get_tangent(t)
    }

    fn first(&self) -> nalgebra::Vector3<Dec> {
        self.get_t(Dec::zero())
    }

    fn last(&self) -> nalgebra::Vector3<Dec> {
        self.get_t(Dec::one())
    }

    fn get_edge_dir(&self, t: Dec) -> nalgebra::Vector3<Dec> {
        let (ix, t) = self.get_ix_t(t);
        self.parts
            .get(ix)
            .expect("You made it clear to respect bounds")
            .get_edge_dir(t)
    }
}
impl Geometry2D for PolyPath {
    fn lines(&self) -> anyhow::Result<Vec<crate::geometry::primitives::Line>> {
        Ok(self
            .parts
            .iter()
            .flat_map(|part| {
                Segments::new(part.segments_hint()).map(|(t, t1)| {
                    let p1 = part.get_t(t);
                    let p2 = part.get_t(t1);
                    let p1 = Vector2::new(p1.x, p1.y);
                    let p2 = Vector2::new(p2.x, p2.y);
                    [p1, p2]
                })
            })
            .collect::<Vec<_>>())
    }
}
