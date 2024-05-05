use nalgebra::Vector3;
use num_traits::Bounded;
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

    fn relate(&self, _to: &Sphere) -> Self::Relate {
        todo!("implement aabb <> sphere");
    }
}

impl<T: Clone> Relation<Node<T>> for Aabb {
    type Relate = BoundNodeRelation;

    fn relate(&self, _to: &Node<T>) -> Self::Relate {
        todo!("implement aabb <> node");
    }
}

impl Relation<Aabb> for Aabb {
    type Relate = BoundRelation;

    fn relate(&self, _to: &Aabb) -> Self::Relate {
        todo!("implement aabb <> aabb");
    }
}
