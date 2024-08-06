use crate::{indexes::aabb::Aabb, primitives_relation};
use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::{Bounded, Zero};
use rust_decimal_macros::dec;

use crate::{decimal::Dec, primitives_relation::relation::Relation};

use super::sphere::Sphere;

const MAX_NODES: usize = 3;

pub enum BoundRelation {
    Intersects,
    Unrelated,
}

pub enum BoundNodeRelation {
    Inside,
    Outside,
}

impl<T: Clone> Relation<Node<T>> for Sphere {
    type Relate = BoundNodeRelation;

    fn relate(&self, node: &Node<T>) -> Self::Relate {
        let m = (node.point - self.center).magnitude();
        if m < self.radius {
            BoundNodeRelation::Inside
        } else {
            BoundNodeRelation::Outside
        }
    }
}

#[derive(Debug, Clone)]
pub struct Node<T: Clone> {
    pub data: T,
    pub point: Vector3<Dec>,
}

#[derive(Debug)]
pub enum OctreeContent<T: Clone> {
    Empty,
    Quadrants([Box<Octree<T>>; 8]),
    Container(Vec<Node<T>>),
}

#[derive(Debug)]
pub struct Octree<T: Clone> {
    aabb: Aabb,
    contents: OctreeContent<T>,
}

impl<T: Clone> Octree<T> {
    pub fn query_within_sphere(&self, bound: Sphere) -> Vec<Node<T>> {
        match self.contents {
            OctreeContent::Empty => Vec::new(),
            OctreeContent::Container(ref items) => items
                .iter()
                .filter(|i| matches!(bound.relate(i), BoundNodeRelation::Inside))
                .cloned()
                .collect_vec(),
            OctreeContent::Quadrants(ref qs) => qs
                .iter()
                .filter(|q| matches!(q.aabb.relate(&bound), BoundRelation::Intersects))
                .flat_map(|q| q.query_within_sphere(bound))
                .collect(),
        }
    }

    pub fn query_within_aabb(&self, bound: Aabb) -> Vec<Node<T>> {
        match self.contents {
            OctreeContent::Empty => Vec::new(),
            OctreeContent::Container(ref items) => items
                .iter()
                .filter(|i| matches!(bound.relate(*i), BoundNodeRelation::Inside))
                .cloned()
                .collect_vec(),
            OctreeContent::Quadrants(ref qs) => qs
                .iter()
                .filter(|q| matches!(q.aabb.relate(&bound), BoundRelation::Intersects))
                .flat_map(|q| q.query_within_aabb(bound))
                .collect(),
        }
    }

    pub fn is_empty(&self) -> bool {
        matches!(self.contents, OctreeContent::Empty)
    }

    fn empty(aabb: Aabb) -> Self {
        Self {
            aabb,
            contents: OctreeContent::Empty,
        }
    }

    fn container(v: Vec<Node<T>>, aabb: Aabb) -> Self {
        Self {
            aabb,
            contents: OctreeContent::Container(v),
        }
    }

    pub fn insert(&mut self, node: Node<T>) {
        let new_aabb = self.insert_recursive(node);
        if new_aabb != self.aabb {}
    }

    fn insert_recursive(&mut self, node: Node<T>) -> Aabb {
        match &mut self.contents {
            OctreeContent::Empty => {
                self.contents = OctreeContent::Container(vec![node]);
                self.aabb
            }

            OctreeContent::Container(ref mut v) => {
                if let BoundNodeRelation::Outside = self.aabb.relate(&node) {
                    panic!(
                        "Inserting point failed - not inside bounds {:?} <== {} {} {} ",
                        self.aabb, node.point.x, node.point.y, node.point.z
                    );
                }
                v.push(node);
                if v.len() > MAX_NODES {
                    let quadrants = Self::sort(v, &self.aabb)
                        .map(|(points, aabb)| Box::new(Octree::new_with_aabb(points, aabb)));
                    self.contents = OctreeContent::Quadrants(quadrants);
                }
                self.aabb
            }

            OctreeContent::Quadrants(quadrants) => {
                let ix = Self::index(&self.aabb, &node.point);

                quadrants[ix].insert_recursive(node)
            }
        }
    }

    /*
        pub fn rebalance(&mut self) {
            if let OctreeContent::Quadrants(_) = &self.contents {
                let items = self.get_vec();
                let new_tree = Self::new(items);
                *self = new_tree;
            }
        }

        pub fn rebalance_mut(&mut self) {
            if let OctreeContent::Quadrants(_) = &self.contents {
                let items = self.get_vec();
                *self = Self::new(items);
            }
        }

    */
    pub fn get_vec(&self) -> Vec<Node<T>> {
        match &self.contents {
            OctreeContent::Empty => Vec::new(),
            OctreeContent::Container(v) => v.to_owned(),
            OctreeContent::Quadrants(ref qs) => qs.iter().flat_map(|q| q.get_vec()).collect(),
        }
    }
    /*

    pub fn linearize(self) -> Vec<Vector3<Dec>> {
        match self.contents {
            OctreeContent::Empty => Vec::new(),
            OctreeContent::Container(v) => vec![v],
            OctreeContent::Quadrants(qs) => qs.into_iter().flat_map(|q| q.linearize()).collect(),
        }
    }

    pub fn get_point_index(&self, p: &Vector3<Dec>) -> Option<usize> {
        match &self.contents {
            OctreeContent::Container(v) if (p - v).magnitude() < Dec::EPSILON => Some(0),
            OctreeContent::Quadrants(qs) => {
                let ix = Self::index(&self.middle, p);
                let len_before: usize = qs.iter().take(ix).map(|q| q.get_length()).sum();
                qs[ix].get_point_index(p).map(|p| p + len_before)
            }
            _ => None,
        }
    }
    */

    fn index(aabb: &Aabb, p: &Vector3<Dec>) -> usize {
        let middle = aabb.min.lerp(&aabb.max, dec!(0.5).into());

        #[allow(clippy::let_and_return)]
        let ix = if p.x > middle.x { 1 << 2 } else { 0 }
            + if p.y > middle.y { 1 << 1 } else { 0 }
            + if p.z > middle.z { 1 } else { 0 };
        ix
    }

    pub fn allocate<const Q: usize>(min: Vector3<Dec>, max: Vector3<Dec>) -> Self {
        Self {
            aabb: Aabb { min, max },
            contents: OctreeContent::Empty,
        }
    }

    pub fn allocate_default<const Q: usize>() -> Self {
        Self {
            aabb: Aabb::default(),
            contents: OctreeContent::Empty,
        }
    }

    fn sort(points: &Vec<Node<T>>, aabb: &Aabb) -> [(Vec<Node<T>>, Aabb); 8] {
        let mut octets = aabb.split_by_octs().map(|aabb| (Vec::new(), aabb));

        for p in points {
            let ix = Self::index(aabb, &p.point);
            octets[ix].0.push(p.clone());
        }
        octets
    }

    fn len(&self) -> usize {
        match &self.contents {
            OctreeContent::Empty => 0,
            OctreeContent::Quadrants(v) => v.iter().map(|q| q.len()).sum(),
            OctreeContent::Container(qs) => qs.len(),
        }
    }

    pub fn new_with_aabb(nodes: Vec<Node<T>>, aabb: Aabb) -> Self {
        if nodes.is_empty() {
            Octree::empty(aabb)
        } else if nodes.len() <= MAX_NODES {
            Octree::container(nodes, aabb)
        } else {
            let quadrants = Self::sort(&nodes, &aabb)
                .map(|(nodes, aabb)| Box::new(Octree::new_with_aabb(nodes, aabb)));

            Octree {
                aabb,
                contents: OctreeContent::Quadrants(quadrants),
            }
        }
    }

    /*
    pub fn new_(nodes: Vec<Node<T>>) -> Self {
        let mut min: Vector3<Dec> = Vector3::new(
            Bounded::max_value(),
            Bounded::max_value(),
            Bounded::max_value(),
        );
        let mut max: Vector3<Dec> = Vector3::new(
            Bounded::min_value(),
            Bounded::min_value(),
            Bounded::min_value(),
        );
        for p in nodes.iter().map(|n| n.point) {
            min.x = Ord::min(min.x, p.x);
            min.y = Ord::min(min.y, p.y);
            min.z = Ord::min(min.z, p.z);

            max.x = Ord::max(max.x, p.x);
            max.y = Ord::max(max.y, p.y);
            max.z = Ord::max(max.z, p.z);
        }

        let aabb = Aabb { min, max };
        if nodes.is_empty() {
            Octree::empty()
        } else if nodes.len() <= MAX_NODES {
            Octree::container(nodes, aabb)
        } else {
            let quadrants = Self::sort(&nodes, &aabb)
                .map(|(nodes, aabb)| Box::new(Octree::new_with_aabb(nodes, aabb)));
            Octree {
                aabb,
                contents: OctreeContent::Quadrants(quadrants),
            }
        }
    }
    */

    pub(crate) fn set_aabb(&mut self, aabb: Aabb) {
        self.aabb = aabb
    }
}
#[cfg(test)]
mod test {
    use nalgebra::Vector3;
    use rust_decimal_macros::dec;

    use crate::{
        decimal::Dec,
        indexes::{aabb::Aabb, sphere::Sphere},
    };

    use super::Octree;

    #[test]
    fn insert_works() {
        let mut i: Octree<usize> = Octree::empty(Aabb::from_points(&[
            Vector3::zeros(),
            Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
        ]));

        let pt: Vector3<Dec> = Vector3::new(
            dec!(21.923418701854968902836034998).into(),
            dec!(18.503840233424920130914685065).into(),
            dec!(0.0).into(),
        );

        assert!(i
            .query_within_sphere(Sphere {
                center: Vector3::zeros(),
                radius: Dec::from(1),
            })
            .is_empty());
        i.insert(super::Node { data: 0, point: pt });

        assert_eq!(
            i.query_within_sphere(Sphere {
                center: pt,
                radius: Dec::from(0.001),
            })
            .len(),
            1
        );
    }
}
