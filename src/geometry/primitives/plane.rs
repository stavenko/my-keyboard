use itertools::{Itertools, PutBack};
use nalgebra::{ComplexField, Vector3};
use num_traits::{One, Zero};
use rust_decimal_macros::dec;

use super::{
    decimal::Dec,
    polygon::{self, Polygon},
    Face,
};

#[derive(Clone, Debug)]
pub struct Plane {
    pub origin: Vector3<Dec>,
    pub normal: Vector3<Dec>,
}
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
    fn back(mut self, face: Polygon) -> Self {
        self.back.push(face);
        self
    }
    fn coplanar_back(mut self, face: Polygon) -> Self {
        self.coplanar_back.push(face);
        self
    }
    fn coplanar_front(mut self, face: Polygon) -> Self {
        self.coplanar_front.push(face);
        self
    }
}

#[derive(PartialEq)]
enum Location {
    Front,
    Back,
    Coplanar,
}

impl Plane {
    fn split(&self, polygon: &Polygon) -> (Polygon, Polygon) {
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
                    let d = (next - self.origin).dot(&self.normal);
                    let t = d / (next - current).dot(&self.normal);
                    assert!(t > Dec::zero());
                    let u = next.lerp(current, t);
                    front.push(u);
                    back.push(u);
                }
                (Location::Back, Location::Front) => {
                    let d = (current - self.origin).dot(&self.normal);
                    let t = d / (current - next).dot(&self.normal);
                    assert!(t > Dec::zero());
                    let u = current.lerp(&next, t);
                    front.push(u);
                    back.push(u);
                }
                _ => {}
            }
        }
        (
            Polygon::new_with_normal(front, polygon.get_normal()),
            Polygon::new_with_normal(back, polygon.get_normal()),
        )
    }
    fn split_fbb(
        f: Vector3<Dec>,
        b1: Vector3<Dec>,
        b2: Vector3<Dec>,
        normal: Vector3<Dec>,
        o: Vector3<Dec>,
        n: Vector3<Dec>,
    ) -> (Face, [Face; 2]) {
        let d = (f - o).dot(&n);

        let t = d / (f - b1).dot(&n);
        let u = f.lerp(&b1, t);
        let s = d / (f - b2).dot(&n);
        let v = f.lerp(&b2, s);
        if t < Dec::zero() || s < Dec::zero() || s > Dec::one() || t > Dec::one() {
            dbg!(t);
            panic!("out of t {t} or s {s}");
        }

        (
            Face::new_with_normal([f, u, v], normal),
            [
                Face::new_with_normal([u, b1, b2], normal),
                Face::new_with_normal([u, b2, v], normal),
            ],
        )
    }

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

    fn location(&self, v: &Vector3<Dec>) -> Location {
        let eps: Dec = dec!(1e-9).into();
        let i = (v - self.origin).dot(&self.normal);
        if i < -eps {
            Location::Back
        } else if i > eps {
            Location::Front
        } else {
            Location::Coplanar
        }
    }
    pub fn split_polygon(&self, face: Polygon) -> SplitResult {
        let eps: Dec = dec!(1e-9).into();
        let locations = face
            .vertices
            .iter()
            .map(|v| (v - self.origin).dot(&self.normal))
            .map(|i| {
                if i < -eps {
                    Location::Back
                } else if i > eps {
                    Location::Front
                } else {
                    Location::Coplanar
                }
            })
            .collect_vec();

        let has_back = locations.iter().any(|f| *f == Location::Back);
        let has_front = locations.iter().any(|f| *f == Location::Front);
        match (has_back, has_front) {
            (true, true) => {
                let (fronts, backs) = self.split(&face);
                SplitResult::default().front(fronts).back(backs)
            }
            (false, true) => SplitResult::default().front(face),
            (true, false) => SplitResult::default().back(face),
            (false, false) => {
                let n = face.get_normal();
                if self.normal.dot(&n) > Dec::zero() {
                    SplitResult::default().coplanar_front(face)
                } else {
                    SplitResult::default().coplanar_back(face)
                }
            }
        }
    }

    pub(crate) fn flip(mut self) -> Plane {
        self.normal = -self.normal;
        self
    }
}
