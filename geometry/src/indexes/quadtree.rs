use nalgebra::{ComplexField, Vector2};

use crate::decimal::{Dec, STABILITY_ROUNDING};

#[derive(Debug)]
pub enum QuadtreeContent {
    Empty,
    Quadrants([Box<Quadtree>; 4]),
    Single(Vector2<Dec>),
}

#[derive(Debug)]
pub struct Quadtree {
    middle: Vector2<Dec>,
    contents: QuadtreeContent,
}
impl Default for Quadtree {
    fn default() -> Self {
        Quadtree::empty()
    }
}

impl Quadtree {
    pub fn is_empty(&self) -> bool {
        matches!(self.contents, QuadtreeContent::Empty)
    }

    fn empty() -> Self {
        Self {
            middle: Vector2::zeros(),
            contents: QuadtreeContent::Empty,
        }
    }
    fn single(v: Vector2<Dec>) -> Self {
        Self {
            middle: v,
            contents: QuadtreeContent::Single(v),
        }
    }

    pub fn insert(&mut self, p: Vector2<Dec>) {
        match &mut self.contents {
            QuadtreeContent::Empty => {
                self.contents = QuadtreeContent::Single(p);
            }
            QuadtreeContent::Single(v) => {
                let diff = *v - p;
                if diff
                    .magnitude_squared()
                    .round_dp(STABILITY_ROUNDING - 2)
                    .sqrt()
                    > Dec::EPSILON
                {
                    let quadrants = Self::sort(vec![*v, p], &self.middle)
                        .map(|points| Box::new(Quadtree::new(points)));
                    self.contents = QuadtreeContent::Quadrants(quadrants);
                }
            }
            QuadtreeContent::Quadrants(quadrants) => {
                let ix = Self::index(&self.middle, &p);

                quadrants[ix].insert(p);
            }
        }
    }

    pub fn rebalance(self) -> Self {
        if let QuadtreeContent::Quadrants(_) = &self.contents {
            let items = self.linearize();
            Self::new(items)
        } else {
            self
        }
    }

    pub fn rebalance_mut(&mut self) {
        if let QuadtreeContent::Quadrants(_) = &self.contents {
            let items = self.get_vec();
            *self = Self::new(items);
        }
    }

    pub fn get_vec(&self) -> Vec<Vector2<Dec>> {
        match self.contents {
            QuadtreeContent::Empty => Vec::new(),
            QuadtreeContent::Single(v) => vec![v],
            QuadtreeContent::Quadrants(ref qs) => qs.iter().flat_map(|q| q.get_vec()).collect(),
        }
    }
    pub fn linearize(self) -> Vec<Vector2<Dec>> {
        match self.contents {
            QuadtreeContent::Empty => Vec::new(),
            QuadtreeContent::Single(v) => vec![v],
            QuadtreeContent::Quadrants(qs) => qs.into_iter().flat_map(|q| q.linearize()).collect(),
        }
    }

    fn get_length(&self) -> usize {
        match &self.contents {
            QuadtreeContent::Empty => 0,
            QuadtreeContent::Quadrants(q) => q.iter().map(|i| i.get_length()).sum(),
            QuadtreeContent::Single(_) => 1,
        }
    }

    pub fn get_point_index(&self, p: &Vector2<Dec>) -> Option<usize> {
        match &self.contents {
            QuadtreeContent::Single(v) if (p - v).magnitude() < Dec::EPSILON => Some(0),
            QuadtreeContent::Quadrants(qs) => {
                let ix = Self::index(&self.middle, p);
                let len_before: usize = qs.iter().take(ix).map(|q| q.get_length()).sum();
                qs[ix].get_point_index(p).map(|p| p + len_before)
            }
            _ => None,
        }
    }

    fn index(middle: &Vector2<Dec>, p: &Vector2<Dec>) -> usize {
        #[allow(clippy::let_and_return)]
        let ix = if p.x > middle.x { 1 << 1 } else { 0 } + if p.y > middle.y { 1 } else { 0 };
        ix
    }

    pub fn allocate(min: Vector2<Dec>, max: Vector2<Dec>) -> Self {
        Self {
            middle: min.lerp(&max, Dec::from(0.5)),
            contents: QuadtreeContent::Empty,
        }
    }

    pub fn allocate_default() -> Self {
        Self {
            middle: Vector2::zeros(),
            contents: QuadtreeContent::Empty,
        }
    }

    fn sort(points: Vec<Vector2<Dec>>, middle: &Vector2<Dec>) -> [Vec<Vector2<Dec>>; 4] {
        let mut ars: [Vec<Vector2<Dec>>; 4] = Default::default();

        for p in points {
            let ix = Self::index(middle, &p);
            ars[ix].push(p);
        }
        ars
    }

    pub fn new(points: Vec<Vector2<Dec>>) -> Self {
        if points.is_empty() {
            Quadtree::empty()
        } else if points.len() == 1 {
            let point = points
                .first()
                .expect("I have checked. We certain that we have something");
            Quadtree::single(*point)
        } else {
            let sum: Vector2<Dec> = points.iter().sum();
            let avg = sum / Dec::from(points.len());

            let quadrants = Self::sort(points, &avg).map(|points| Box::new(Quadtree::new(points)));
            Quadtree {
                middle: avg,
                contents: QuadtreeContent::Quadrants(quadrants),
            }
        }
    }
}
