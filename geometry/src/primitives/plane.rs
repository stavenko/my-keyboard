use std::fmt;

use itertools::Itertools;
use nalgebra::{ComplexField, Vector3};
use num_traits::{One, Zero};
use rust_decimal_macros::dec;

use crate::bsp::Reversable;

use super::{
    cutter::{Location, SplitResult, Splitter},
    decimal::{Dec, STABILITY_ROUNDING},
    polygon::Polygon,
};

#[derive(Clone, PartialEq, Eq, PartialOrd, Hash)]
pub struct Plane {
    normal: Vector3<Dec>,
    d: Dec,
}

impl fmt::Debug for Plane {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}x  {}y {}z {}",
            self.normal.x.round_dp(4),
            self.normal.y.round_dp(4),
            self.normal.z.round_dp(4),
            self.d.round_dp(4)
        )
    }
}

impl Reversable for Plane {
    fn flip(mut self) -> Self {
        self.normal = -self.normal;
        self.d = -self.d;
        self
    }
}

impl Splitter<Polygon> for Plane {
    fn split(&self, item: Polygon) -> super::cutter::SplitResult<Polygon> {
        let locations = item.vertices.iter().map(|v| self.location(v)).collect_vec();

        let has_back = locations.iter().any(|f| *f == Location::Back);
        let has_front = locations.iter().any(|f| *f == Location::Front);
        match (has_back, has_front) {
            (true, true) => {
                let (fronts, backs) = self.split_fb(&item);
                SplitResult::default().front(fronts).back(backs)
            }
            (false, true) => SplitResult::default().front(item),
            (true, false) => SplitResult::default().back(item),
            (false, false) => {
                let n = item.get_normal();
                if self.normal.dot(&n) > Dec::zero() {
                    SplitResult::default().coplanar_front(item)
                } else {
                    SplitResult::default().coplanar_back(item)
                }
            }
        }
    }

    fn from_item(item: &Polygon) -> Self {
        item.get_plane()
    }

    fn locate(&self, item: Polygon) -> Location {
        todo!()
    }
}
/*
#[derive(Default, Debug)]
pub struct SplitResult {
    pub front: Vec<Polygon>,
    pub back: Vec<Polygon>,
    pub coplanar_back: Vec<Polygon>,
    pub coplanar_front: Vec<Polygon>,
}

impl SplitResult {
    fn front(mut self, face: Polygon) -> Self {
        self.front.push(face);
        self
    }
    pub fn fronts(mut self, mut segment: Vec<Polygon>) -> Self {
        self.front.append(&mut segment);
        self
    }
    fn back(mut self, face: Polygon) -> Self {
        self.back.push(face);
        self
    }
    pub fn backs(mut self, mut segment: Vec<Polygon>) -> Self {
        self.back.append(&mut segment);
        self
    }
    fn coplanar_back(mut self, face: Polygon) -> Self {
        self.coplanar_back.push(face);
        self
    }
    pub fn coplanar_backs(mut self, mut segment: Vec<Polygon>) -> Self {
        self.coplanar_back.append(&mut segment);
        self
    }
    fn coplanar_front(mut self, face: Polygon) -> Self {
        self.coplanar_front.push(face);
        self
    }
    pub fn coplanar_fronts(mut self, mut segment: Vec<Polygon>) -> Self {
        self.coplanar_front.append(&mut segment);
        self
    }
}

#[derive(PartialEq)]
enum Location {
    Front,
    Back,
    Coplanar,
}
*/

impl Plane {
    pub fn is_same_or_opposite(&self, other: &Self) -> bool {
        if self
            .normal
            .dot(&other.normal)
            .abs()
            .round_dp(STABILITY_ROUNDING)
            == Dec::one()
        {
            let m = self.normal * self.d;
            let o = other.normal * other.d;
            (m - o).magnitude_squared().round_dp(STABILITY_ROUNDING) == Dec::zero()
        } else {
            false
        }
    }

    pub fn new_from_normal_and_point(normal: Vector3<Dec>, point: Vector3<Dec>) -> Self {
        let d = normal.dot(&point);

        Self { normal, d }
    }

    fn point_on_plane(&self) -> Vector3<Dec> {
        self.normal * self.d
    }

    fn split_fb(&self, polygon: &Polygon) -> (Polygon, Polygon) {
        let mut front = Vec::new();
        let mut back = Vec::new();
        let len = polygon.vertices.len();
        for (i, current) in polygon.vertices.iter().enumerate() {
            let j = (i + 1) % len;
            let next = polygon.vertices[j];
            let location_current = self.location(current);
            let location_next = self.location(&next);
            match location_current {
                Location::Back => {
                    back.push(*current);
                }
                Location::Front => {
                    front.push(*current);
                }
                Location::Coplanar => {
                    front.push(*current);
                    back.push(*current);
                }
            }
            match (location_next, location_current) {
                (Location::Front, Location::Back) => {
                    let d = (next - self.point_on_plane()).dot(&self.normal);
                    let t = d / (next - current).dot(&self.normal);
                    assert!(t >= Dec::zero());
                    assert!(t <= Dec::one());
                    let u = next.lerp(current, t);
                    front.push(u);
                    back.push(u);
                }
                (Location::Back, Location::Front) => {
                    let d = (current - self.point_on_plane()).dot(&self.normal);
                    let t = d / (current - next).dot(&self.normal);
                    assert!(t >= Dec::zero());
                    assert!(t <= Dec::one());
                    let u = current.lerp(&next, t);
                    front.push(u);
                    back.push(u);
                }
                _ => {}
            }
        }
        (
            Polygon::new_with_plane(front, polygon.get_plane().to_owned()).unwrap(),
            Polygon::new_with_plane(back, polygon.get_plane().to_owned()).unwrap(),
        )
    }

    /*
    fn split_fcb(
        f: Vector3<Dec>,
        c: Vector3<Dec>,
        b: Vector3<Dec>,
        normal: Vector3<Dec>,
        o: Vector3<Dec>,
        n: Vector3<Dec>,
    ) -> (Face, Face) {
        let d = (f - o).dot(&n);

        let t = d / (f - b).dot(&n);

        if t < Dec::zero() || t > Dec::one() {
            panic!("out of t {t}");
        }
        let u = f.lerp(&b, t);
        (
            Face::new_with_normal([f, c, u], normal),
            Face::new_with_normal([c, b, u], normal),
        )
    }

    fn split_bcf(
        b: Vector3<Dec>,
        c: Vector3<Dec>,
        f: Vector3<Dec>,
        normal: Vector3<Dec>,
        o: Vector3<Dec>,
        n: Vector3<Dec>,
    ) -> (Face, Face) {
        let d = (f - o).dot(&n);

        let t = d / (f - b).dot(&n);
        if t < Dec::zero() || t > Dec::one() {
            panic!("out of t {t}");
        }
        let u = f.lerp(&b, t);
        (
            Face::new_with_normal([b, c, u], normal),
            Face::new_with_normal([c, f, u], normal),
        )
    }
    */

    fn location(&self, v: &Vector3<Dec>) -> Location {
        let eps: Dec = dec!(1e-9).into();
        let i = (v - self.point_on_plane()).dot(&self.normal);
        if i < -eps {
            Location::Back
        } else if i > eps {
            Location::Front
        } else {
            Location::Coplanar
        }
    }

    pub fn normal(&self) -> Vector3<Dec> {
        self.normal
    }

    pub fn d(&self) -> Dec {
        self.d
    }
}
#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use rust_decimal_macros::dec;

    use crate::primitives::{cutter::Splitter, decimal::Dec, polygon::Polygon};

    use super::Plane;

    #[test]
    fn plane_is_good() {
        let normal1 = Vector3::x() * Dec::from(dec!(.4))
            + Vector3::y() * Dec::from(dec!(-0.3))
            + Vector3::z() * Dec::from(dec!(1.7));
        let normal1 = normal1.normalize();

        let origin = Vector3::x() * Dec::from(dec!(2.4))
            + Vector3::y() * Dec::from(dec!(-17.3))
            + Vector3::z() * Dec::from(dec!(10.7));
        let pl = Plane::new_from_normal_and_point(normal1, origin);
        let new_origin = pl.point_on_plane();

        let pl_new = Plane::new_from_normal_and_point(normal1, new_origin);

        assert_eq!(pl, pl_new);
    }
    #[test]
    fn split_polygon_fb() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly = Polygon::new(points.to_vec()).unwrap();
        let plane = Plane::new_from_normal_and_point(Vector3::y(), Vector3::zeros());

        let result = plane.split(poly);
        assert_eq!(result.front.len(), 1);
        assert_eq!(result.back.len(), 1);

        let pts_front = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let pts_back = [
            Vector3::new(dec!(1).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(0).into(), dec!(0).into()),
        ];
        assert_eq!(result.front[0].vertices, pts_front);
        assert_eq!(result.back[0].vertices, pts_back);
    }
}
