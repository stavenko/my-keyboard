use nalgebra::Vector3;

use super::decimal::Dec;

#[derive(Debug)]
pub enum IndexContents {
    Empty,
    Quadrants([Box<Index>; 8]),
    Single(Vector3<Dec>),
}

impl IndexContents {}

#[derive(Debug)]
pub struct Index {
    middle: Vector3<Dec>,
    contents: IndexContents,
    points: usize,
}
impl Default for Index {
    fn default() -> Self {
        Index::empty()
    }
}

impl Index {
    pub fn is_empty(&self) -> bool {
        matches!(self.contents, IndexContents::Empty)
    }

    fn empty() -> Self {
        Self {
            middle: Vector3::zeros(),
            contents: IndexContents::Empty,
            points: 0,
        }
    }
    fn single(v: Vector3<Dec>) -> Self {
        Self {
            middle: v,
            contents: IndexContents::Single(v),
            points: 1,
        }
    }

    pub fn insert(&mut self, p: Vector3<Dec>) {
        match &mut self.contents {
            IndexContents::Empty => {
                self.contents = IndexContents::Single(p);
            }
            IndexContents::Single(v) => {
                let diff = *v - p;
                if diff.magnitude() > Dec::EPSILON {
                    let quadrants = Self::sort(vec![*v, p], &self.middle)
                        .map(|points| Box::new(Index::new(points)));
                    self.contents = IndexContents::Quadrants(quadrants);
                }
            }
            IndexContents::Quadrants(quadrants) => {
                let ix = Self::index(&self.middle, &p);

                quadrants[ix].insert(p);
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

    pub fn get_vec(&self) -> Vec<Vector3<Dec>> {
        match self.contents {
            IndexContents::Empty => Vec::new(),
            IndexContents::Single(v) => vec![v],
            IndexContents::Quadrants(ref qs) => qs.iter().flat_map(|q| q.get_vec()).collect(),
        }
    }

    pub fn linearize(self) -> Vec<Vector3<Dec>> {
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

    pub fn get_point_index(&self, p: &Vector3<Dec>) -> Option<usize> {
        match &self.contents {
            IndexContents::Single(v) if (p - v).magnitude() < Dec::EPSILON => Some(0),
            IndexContents::Quadrants(qs) => {
                let ix = Self::index(&self.middle, p);
                let len_before: usize = qs.iter().take(ix).map(|q| q.get_length()).sum();
                qs[ix].get_point_index(p).map(|p| p + len_before)
            }
            _ => None,
        }
    }

    fn index(middle: &Vector3<Dec>, p: &Vector3<Dec>) -> usize {
        #[allow(clippy::let_and_return)]
        let ix = if p.x > middle.x { 1 << 2 } else { 0 }
            + if p.y > middle.y { 1 << 1 } else { 0 }
            + if p.z > middle.z { 1 } else { 0 };
        ix
    }

    pub fn allocate(min: Vector3<Dec>, max: Vector3<Dec>) -> Self {
        Self {
            middle: min.lerp(&max, Dec::from(0.5)),
            contents: IndexContents::Empty,
            points: 0,
        }
    }

    pub fn allocate_default() -> Self {
        Self {
            middle: Vector3::zeros(),
            contents: IndexContents::Empty,
            points: 0,
        }
    }

    fn sort(points: Vec<Vector3<Dec>>, middle: &Vector3<Dec>) -> [Vec<Vector3<Dec>>; 8] {
        let mut ars: [Vec<Vector3<Dec>>; 8] = Default::default();

        for p in points {
            let ix = Self::index(middle, &p);
            ars[ix].push(p);
        }
        ars
    }

    pub fn new(points: Vec<Vector3<Dec>>) -> Self {
        let sum: Vector3<Dec> = points.iter().sum();
        let avg: Vector3<Dec> = sum / Dec::from(points.len());

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

    use crate::{decimal::Dec, spatial_index::Index};

    use super::IndexContents;

    #[test]
    fn ix_is_correct() {
        let items = vec![
            Vector3::new(Dec::from(1.0), Dec::from(1.0), Dec::from(1.0)),
            Vector3::new(Dec::from(1.0), Dec::from(1.0), Dec::from(0.0)),
            Vector3::new(Dec::from(1.0), Dec::from(0.0), Dec::from(1.0)),
            Vector3::new(Dec::from(1.0), Dec::from(0.0), Dec::from(0.0)),
            Vector3::new(Dec::from(0.0), Dec::from(1.0), Dec::from(1.0)),
            Vector3::new(Dec::from(0.0), Dec::from(1.0), Dec::from(0.0)),
            Vector3::new(Dec::from(0.0), Dec::from(0.0), Dec::from(1.0)),
            Vector3::new(Dec::from(0.0), Dec::from(0.0), Dec::from(0.0)),
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
                    .position(|v| (v - point).magnitude() < Dec::EPSILON)
                    .unwrap()
            })
            .collect::<Vec<_>>();
        assert!(t == tt);
    }
    #[test]
    #[ignore = "reason"]
    fn balance_is_ok() {
        let items = vec![
            Vector3::new(Dec::from(1.0), Dec::from(1.0), Dec::from(1.0)),
            Vector3::new(Dec::from(1.0), Dec::from(1.0), Dec::from(0.0)),
            Vector3::new(Dec::from(1.0), Dec::from(0.0), Dec::from(1.0)),
            Vector3::new(Dec::from(1.0), Dec::from(0.0), Dec::from(0.0)),
            Vector3::new(Dec::from(0.0), Dec::from(1.0), Dec::from(1.0)),
            Vector3::new(Dec::from(0.0), Dec::from(1.0), Dec::from(0.0)),
            Vector3::new(Dec::from(0.0), Dec::from(0.0), Dec::from(1.0)),
            Vector3::new(Dec::from(0.0), Dec::from(0.0), Dec::from(0.0)),
        ];
        let mut ix = Index::allocate(
            Vector3::new(Dec::from(-500), Dec::from(-500), Dec::from(-500)),
            Vector3::new(Dec::from(-499), Dec::from(-499), Dec::from(-499)),
        );
        for t in &items {
            ix.insert(*t);
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
