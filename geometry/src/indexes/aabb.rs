use nalgebra::Vector3;
use num_traits::{Bounded, Pow, Zero};
use rust_decimal_macros::dec;

use crate::{decimal::Dec, primitives_relation::relation::Relation};

use super::{
    octree::{BoundNodeRelation, BoundRelation, Node},
    sphere::Sphere,
};

#[derive(Debug, Clone, Default, Copy, PartialEq)]
pub struct Aabb {
    pub(crate) min: Vector3<Dec>,
    pub(crate) max: Vector3<Dec>,
}

impl Aabb {
    pub fn split_x(&self) -> [Self; 2] {
        let middle = self.min.lerp(&self.max, dec!(0.5).into());

        [
            Aabb::from_points(&[self.min, Vector3::new(middle.x, self.max.y, self.max.z)]),
            Aabb::from_points(&[self.max, Vector3::new(middle.x, self.min.y, self.min.z)]),
        ]
    }

    pub fn split_y(&self) -> [Self; 2] {
        let middle = self.min.lerp(&self.max, dec!(0.5).into());

        [
            Aabb::from_points(&[self.min, Vector3::new(self.max.x, middle.y, self.max.z)]),
            Aabb::from_points(&[self.max, Vector3::new(self.min.x, middle.y, self.min.z)]),
        ]
    }
    pub fn split_z(&self) -> [Self; 2] {
        let middle = self.min.lerp(&self.max, dec!(0.5).into());

        [
            Aabb::from_points(&[self.min, Vector3::new(self.max.x, self.max.y, middle.z)]),
            Aabb::from_points(&[self.max, Vector3::new(self.min.x, self.min.y, middle.z)]),
        ]
    }

    pub fn split_by_octs(&self) -> [Self; 8] {
        let x = self.split_x();
        let [ny, py] = x.map(|x| x.split_y());
        let [[[nxnynz, nxnypz], [nxpynz, nxpypz]], [[pxnynz, pxnypz], [pxpynz, pxpypz]]] =
            [ny.map(|y| y.split_z()), py.map(|y| y.split_z())];

        [
            nxnynz, nxnypz, nxpynz, nxpypz, pxnynz, pxnypz, pxpynz, pxpypz,
        ]
        //     x   y   z
        //result[0 + 0 + 0] = nxnynz;
        //result[0 + 0 + 1] = nxnypz;

        //result[0 + 2 + 0] = nxpynz;
        //result[0 + 2 + 1] = nxpypz;

        //result[4 + 0 + 0] = pxnynz;
        //result[4 + 0 + 1] = pxnypz;

        //result[4 + 2 + 0] = pxpynz;
        //result[4 + 2 + 1] = pxpypz;
    }
    pub fn from_points(points: &[Vector3<Dec>]) -> Self {
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
        for p in points.iter() {
            min.x = Ord::min(min.x, p.x);
            min.y = Ord::min(min.y, p.y);
            min.z = Ord::min(min.z, p.z);

            max.x = Ord::max(max.x, p.x);
            max.y = Ord::max(max.y, p.y);
            max.z = Ord::max(max.z, p.z);
        }
        let d = Dec::from(dec!(0.000_1));
        let min = min - Vector3::new(d, d, d);
        let max = max + Vector3::new(d, d, d);

        Aabb { min, max }
    }
    pub fn add(&mut self, pt: Vector3<Dec>) {
        self.min.x = self.min.x.min(pt.x);
        self.min.y = self.min.y.min(pt.y);
        self.min.z = self.min.x.min(pt.z);
    }

    #[allow(unused)]
    pub(crate) fn merge(mut self, aabb: Aabb) -> Aabb {
        self.min.x = self.min.x.min(aabb.min.x);
        self.min.y = self.min.y.min(aabb.min.y);
        self.min.z = self.min.x.min(aabb.min.z);

        self.max.x = self.max.x.max(aabb.max.x);
        self.max.y = self.max.y.max(aabb.max.y);
        self.max.z = self.max.x.max(aabb.max.z);

        self
    }
}

impl Relation<Sphere> for Aabb {
    type Relate = BoundRelation;

    fn relate(&self, sphere: &Sphere) -> Self::Relate {
        let center = [sphere.center.x, sphere.center.y, sphere.center.z];
        let min_b = [self.min.x, self.min.y, self.min.z];
        let max_b = [self.max.x, self.max.y, self.max.z];

        let min_dist: Dec = center
            .iter()
            .zip(min_b)
            .map(|(&c, m)| {
                if c < m {
                    (c - m).pow(2i64)
                } else {
                    Dec::zero()
                }
            })
            .sum();

        let max_dist: Dec = center
            .iter()
            .zip(max_b)
            .map(|(&c, m)| {
                if c > m {
                    (c - m).pow(2i64)
                } else {
                    Dec::zero()
                }
            })
            .sum();

        //dbg!(max_dist, min_dist, sphere.radius);
        if (max_dist + min_dist) < sphere.radius.pow(2i64) {
            BoundRelation::Intersects
        } else {
            BoundRelation::Unrelated
        }
    }
}

impl<T: Clone> Relation<Node<T>> for Aabb {
    type Relate = BoundNodeRelation;

    fn relate(&self, to: &Node<T>) -> Self::Relate {
        if to.point.x >= self.min.x
            && to.point.x <= self.max.x
            && to.point.y >= self.min.y
            && to.point.y <= self.max.y
            && to.point.z >= self.min.z
            && to.point.z <= self.max.z
        {
            BoundNodeRelation::Inside
        } else {
            BoundNodeRelation::Outside
        }
    }
}

impl Relation<Aabb> for Aabb {
    type Relate = BoundRelation;

    fn relate(&self, _to: &Aabb) -> Self::Relate {
        todo!("implement aabb <> aabb");
    }
}
