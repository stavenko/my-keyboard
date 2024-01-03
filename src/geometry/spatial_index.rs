use std::{
    collections::{HashMap, HashSet},
    hash::{Hash, Hasher},
};

use anyhow::Ok;
use itertools::{Either, Itertools};
use nalgebra::{Vector, Vector3};

#[derive(Debug)]
pub enum IndexContents {
    Empty,
    Quadrants([Box<Index>; 8]),
    Single(Vector3<f32>),
}

#[derive(PartialEq, Debug)]
pub struct VecWrap(Vector3<f32>);

impl Eq for VecWrap {}

impl Hash for VecWrap {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let item = format!("{:.5} {:.5} {:.5}", self.0.x, self.0.y, self.0.z);

        dbg!(&item);
        item.hash(state);
    }
}

impl IndexContents {}

#[derive(Debug)]
pub struct Index {
    middle: Vector3<f32>,
    contents: IndexContents,
    points: usize,
}
impl Default for Index {
    fn default() -> Self {
        Index::empty()
    }
}

impl Index {
    fn is_empty(&self) -> bool {
        matches!(self.contents, IndexContents::Empty)
    }

    fn empty() -> Self {
        Self {
            middle: Vector3::zeros(),
            contents: IndexContents::Empty,
            points: 0,
        }
    }
    fn single(v: Vector3<f32>) -> Self {
        Self {
            middle: v,
            contents: IndexContents::Single(v),
            points: 1,
        }
    }

    pub fn insert(&mut self, p: Vector3<f32>, deep: usize) {
        match &mut self.contents {
            IndexContents::Empty => {
                self.contents = IndexContents::Single(p);
            }
            IndexContents::Single(v) => {
                let diff = *v - p;
                if diff.magnitude() > f32::EPSILON * 2.0 {
                    let quadrants = Self::sort(vec![*v, p], &self.middle)
                        .map(|points| Box::new(Index::new(points)));
                    self.contents = IndexContents::Quadrants(quadrants);
                }
            }
            IndexContents::Quadrants(quadrants) => {
                let ix = Self::index(&self.middle, &p);

                quadrants[ix].insert(p, deep + 1);
            }
        }

        self.points += 1;
    }

    pub fn rebalance(self) -> Self {
        if let IndexContents::Quadrants(_) = &self.contents {
            let items = self.linearize();
            Self::new(items)
        } else {
            self
        }
    }

    pub fn rebalance_mut(&mut self) {
        if let IndexContents::Quadrants(_) = &self.contents {
            let items = self.get_vec();
            *self = Self::new(items);
        }
    }

    pub fn get_vec(&self) -> Vec<Vector3<f32>> {
        match self.contents {
            IndexContents::Empty => Vec::new(),
            IndexContents::Single(v) => vec![v],
            IndexContents::Quadrants(ref qs) => qs.iter().flat_map(|q| q.get_vec()).collect(),
        }
    }
    pub fn linearize(self) -> Vec<Vector3<f32>> {
        match self.contents {
            IndexContents::Empty => Vec::new(),
            IndexContents::Single(v) => vec![v],
            IndexContents::Quadrants(qs) => qs.into_iter().flat_map(|q| q.linearize()).collect(),
        }
    }

    fn get_length(&self) -> usize {
        match &self.contents {
            IndexContents::Empty => 0,
            IndexContents::Quadrants(q) => q.iter().map(|i| i.get_length()).sum(),
            IndexContents::Single(_) => 1,
        }
    }

    pub fn get_point_index(&self, p: &Vector3<f32>) -> Option<usize> {
        match &self.contents {
            IndexContents::Single(v) if (p - v).magnitude() < 1e-4 => Some(0),
            IndexContents::Quadrants(qs) => {
                let ix = Self::index(&self.middle, p);
                let len_before: usize = qs.iter().take(ix).map(|q| q.get_length()).sum();
                //dbg!(ix, self.middle);
                qs[ix].get_point_index(p).map(|p| p + len_before)
            }
            _ => {
                // dbg!("@@@@", self.middle);
                None
            }
        }
    }

    fn index(middle: &Vector3<f32>, p: &Vector3<f32>) -> usize {
        #[allow(clippy::let_and_return)]
        let ix = if p.x > middle.x { 1 << 2 } else { 0 }
            + if p.y > middle.y { 1 << 1 } else { 0 }
            + if p.z > middle.z { 1 } else { 0 };
        ix
    }

    pub fn allocate(min: Vector3<f32>, max: Vector3<f32>) -> Self {
        Self {
            middle: min.lerp(&max, 0.5),
            contents: IndexContents::Empty,
            points: 0,
        }
    }

    pub fn allocate_default() -> Self {
        Self {
            middle: Vector3::new(0.0, 0.0, 0.0),
            contents: IndexContents::Empty,
            points: 0,
        }
    }

    fn sort(points: Vec<Vector3<f32>>, middle: &Vector3<f32>) -> [Vec<Vector3<f32>>; 8] {
        let mut ars: [Vec<Vector3<f32>>; 8] = Default::default();

        for p in points {
            let ix = Self::index(middle, &p);
            ars[ix].push(p);
        }
        ars
    }

    pub fn new(mut points: Vec<Vector3<f32>>) -> Self {
        let sum: Vector3<f32> = points.iter().sum();
        let avg = sum / points.len() as f32;

        let b = points.len();
        let _c = points.clone();
        //points.retain(|p| *p != avg);
        let a = points.len();
        if b == 2 && a != b {
            println!("≈≈≈ {} {} {}", avg.x, avg.y, avg.z);
            for p in _c {
                if !points.contains(&p) {
                    println!("⇧⇧⇧ {} {} {}", p.x, p.y, p.z);
                }
            }
        }

        // dbg!(&points);
        /*
        if points.len() == 2 && points.iter().filter(|p| *p == avg) {
            let d = points.get(0).unwrap() - points.get(1).unwrap();
            dbg!(d.magnitude(), f32::EPSILON);
            dbg!(d.magnitude() < f32::EPSILON);
            dbg!(avg, &points);
        }
        let points = points
            .into_iter()
            .fold(HashMap::new(), |mut hm, item| {
                let key = format!("{:.5} {:.5} {:.5}", item.x, item.y, item.z);
                hm.insert(key, item);
                hm
            })
            .into_iter()
            .map(|v| v.1)
            .collect_vec();
        */

        if points.is_empty() {
            Index::empty()
        } else if points.len() == 1 {
            let point = points
                .first()
                .expect("I have checked. We certain that we have something");
            Index::single(*point)
        } else {
            let points_amount = points.len();
            let quadrants = Self::sort(points, &avg).map(|points| Box::new(Index::new(points)));
            Index {
                points: points_amount,
                middle: avg,
                contents: IndexContents::Quadrants(quadrants),
            }
        }
    }
}
#[cfg(test)]
mod test {

    use assert_matches::assert_matches;
    use nalgebra::Vector3;

    use crate::geometry::spatial_index::Index;

    use super::IndexContents;

    #[test]
    fn ix_is_correct() {
        let items = vec![
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(1.0, 0.0, 1.0),
            Vector3::new(0.0, -10.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 1.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ];
        let ix = Index::new(items.clone());
        let t = items
            .iter()
            .map(|v| ix.get_point_index(v).unwrap())
            .collect::<Vec<_>>();
        let arr = ix.linearize();

        let tt = items
            .iter()
            .map(|point| {
                arr.iter()
                    .position(|v| (v - point).magnitude() < f32::EPSILON)
                    .unwrap()
            })
            .collect::<Vec<_>>();
        assert!(t == tt);
    }
    #[test]
    fn balance_is_ok() {
        let items = vec![
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(1.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 1.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.0, 0.0, 0.0),
        ];
        let mut ix = Index::allocate(
            Vector3::new(-500., -500., -500.),
            Vector3::new(-499., -499., -499.),
        );
        for t in &items {
            ix.insert(*t, 0);
        }

        let qs = assert_matches!(&ix.contents, IndexContents::Quadrants(qs) => qs);
        assert_eq!(
            qs.iter().map(|i| i.is_empty()).collect::<Vec<_>>(),
            [true, true, true, true, true, true, true, false]
        );
        let ix = ix.rebalance();
        let qs = assert_matches!(ix.contents, IndexContents::Quadrants(qs) => qs);
        assert_eq!(
            qs.map(|i| i.is_empty()),
            [false, false, false, false, false, false, false, false]
        );
    }
}
