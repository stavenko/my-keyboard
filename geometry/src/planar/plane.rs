use std::fmt;

use nalgebra::Vector3;
use num_traits::{One, Zero};

use crate::{
    decimal::{Dec, STABILITY_ROUNDING},
    reversable::Reversable,
};

#[derive(Clone, Eq, PartialOrd)]
pub struct Plane {
    normal: Vector3<Dec>,
    d: Dec,
}

impl PartialEq for Plane {
    fn eq(&self, other: &Self) -> bool {
        (self.d - other.d).round_dp(STABILITY_ROUNDING).is_zero()
            && self
                .normal
                .dot(&other.normal)
                .round_dp(STABILITY_ROUNDING)
                .is_one()
    }
}

impl fmt::Debug for Plane {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}x  {}y {}z {}",
            self.normal.x.round_dp(STABILITY_ROUNDING),
            self.normal.y.round_dp(STABILITY_ROUNDING),
            self.normal.z.round_dp(STABILITY_ROUNDING),
            self.d.round_dp(STABILITY_ROUNDING)
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
impl Plane {
    pub fn new(a: Dec, b: Dec, c: Dec, d: Dec) -> Self {
        Self {
            normal: Vector3::new(a, b, c).normalize(),
            d,
        }
    }
    pub fn new_from_normal_and_point(normal: Vector3<Dec>, point: Vector3<Dec>) -> Self {
        let d = normal.dot(&point);

        Self { normal, d }
    }

    pub fn normal(&self) -> Vector3<Dec> {
        self.normal
    }

    pub fn d(&self) -> Dec {
        self.d
    }
    pub fn point_on_plane(&self) -> Vector3<Dec> {
        self.normal * self.d
    }
}

/*
impl Splitter<Polygon, Vector3<Dec>> for Plane {
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

    fn locate(&self, item: &Polygon) -> ItemLocation {
        let mut in_plane = 0;
        let mut fronts = 0;
        let mut backs = 0;
        for v in &item.vertices {
            let loc = self.locate_vertex(v);

            match loc {
                VertexLocation::Front => {
                    fronts += 1;
                }
                VertexLocation::Back => {
                    backs += 1;
                }
                VertexLocation::On => {
                    in_plane += 1;
                }
            }
        }

        match (in_plane, fronts, backs) {
            (l, _, _) if l == item.vertices.len() => {
                if self.normal().dot(&item.get_normal()).is_positive() {
                    ItemLocation::Co
                } else {
                    ItemLocation::UnCo
                }
            }
            (0, _, 0) => ItemLocation::Front,
            (0, 0, _) => ItemLocation::Back,
            (_, _, 0) => ItemLocation::Front,
            (_, 0, _) => ItemLocation::Back,
            (_, _, _) => ItemLocation::Split,
        }
    }

    fn locate_vertex(&self, vertex: &Vector3<Dec>) -> crate::cutter::VertexLocation {
        let distance = (self.normal.dot(vertex) - self.d).round_dp(STABILITY_ROUNDING - 2);

        if distance == Zero::zero() {
            VertexLocation::On
        } else if distance.is_positive() {
            VertexLocation::Front
        } else {
            VertexLocation::Back
        }
    }
}
#[derive(Default, Debug)]
pub struct SplitResult {
    pub front: Vec<Polygon>,
    pub back: Vec<Polygon>,
    pub coplanar_back: Vec<Polygon>,
    pub coplanar_front: Vec<Polygon>,;

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

impl Plane {
    pub fn new(a: Dec, b: Dec, c: Dec, d: Dec) -> Self {
        Self {
            normal: Vector3::new(a, b, c).normalize(),
            d,
        }
    }

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

    pub fn point_on_plane(&self) -> Vector3<Dec> {
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

    pub fn is_point_on_plane(&self, vertex: &Vector3<Dec>) -> bool {
        let distance = (self.normal.dot(vertex) - self.d).round_dp(STABILITY_ROUNDING - 2);

        distance.is_zero()
    }

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
    pub fn point_on_plane(&self) -> Vector3<Dec> {
        self.normal * self.d
    }
}

impl Intersects<Polygon> for Plane {
    type Out = Ray;

    fn intersects(&self, other: &Polygon) -> Option<Self::Out> {
        self.intersects(&other.get_plane())
    }
}

impl Intersects<Plane> for Plane {
    type Out = Ray;

    fn intersects(&self, other: &Plane) -> Option<Self::Out> {
        let dir = self.normal.cross(&other.normal);

        if dir
            .magnitude_squared()
            .round_dp(STABILITY_ROUNDING)
            .is_zero()
        {
            None
        } else {
            let x = dir.x;
            let y = dir.y;
            let z = dir.z;
            if x.abs() > y.abs() && x.abs() > z.abs() {
                let mat =
                    Matrix2::new(self.normal.y, self.normal.z, other.normal.y, other.normal.z);
                let inv_mat = mat.try_inverse().unwrap();
                let ds = Vector2::new(self.d, other.d);
                let r = inv_mat * ds;

                let mut origin = Vector3::zeros();
                origin.y = r.x;
                origin.z = r.y;

                Some(Ray { origin, dir })
            } else if y.abs() > x.abs() && y.abs() > z.abs() {
                let mat =
                    Matrix2::new(self.normal.x, self.normal.z, other.normal.x, other.normal.z);
                let inv_mat = mat.try_inverse().unwrap();
                let ds = Vector2::new(self.d, other.d);
                let r = inv_mat * ds;

                let mut origin = Vector3::zeros();
                origin.x = r.x;
                origin.z = r.y;

                Some(Ray { origin, dir })
            } else if z.abs() > x.abs() && z.abs() > y.abs() {
                let mat =
                    Matrix2::new(self.normal.x, self.normal.y, other.normal.x, other.normal.y);
                let inv_mat = mat.try_inverse().unwrap();
                let ds = Vector2::new(self.d, other.d);
                let r = inv_mat * ds;

                let mut origin = Vector3::zeros();
                origin.x = r.x;
                origin.y = r.y;

                Some(Ray { origin, dir })
            } else {
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{ComplexField, Matrix2, Vector2, Vector3};
    use num_traits::Zero;
    use rust_decimal_macros::dec;
    use stl_io::Vector;

    use crate::{
        cutter::Splitter,
        decimal::{Dec, STABILITY_ROUNDING},
        origin,
        polygon::Polygon,
    };

    use super::Plane;

    #[test]
    fn test_plane_intersection() {
        //let plane1 = Plane::new(2.into(), 2.into(), 3.into(), 1.into());
        //let plane2 = Plane::new(2.into(), 5.into(), (-1).into(), 1.into());

        let plane1 = Plane::new(2.into(), 3.into(), 2.into(), 1.into());
        let plane2 = Plane::new(5.into(), 2.into(), (-1).into(), 1.into());

        let ray_dir = plane2.normal.cross(&plane1.normal);
        dbg!(ray_dir);
        assert!(!ray_dir
            .magnitude_squared()
            .round_dp(STABILITY_ROUNDING)
            .is_zero());

        let x = ray_dir.x;
        let y = ray_dir.y;
        let z = ray_dir.z;
        if x > y && x > z {
            let mat = Matrix2::new(
                plane1.normal.y,
                plane1.normal.z,
                plane2.normal.y,
                plane2.normal.z,
            );
            let inv_mat = mat.try_inverse().unwrap();
            let ds = Vector2::new(plane1.d, plane2.d);
            let r = inv_mat * ds;

            dbg!(r);
            let mut origin = Vector3::zeros();
            origin.y = r.x;
            origin.z = r.y;

            assert!(
                dbg!((origin.dot(&plane1.normal) - plane1.d).round_dp(STABILITY_ROUNDING))
                    .is_zero()
            );

            assert!(
                dbg!((origin.dot(&plane2.normal) - plane2.d).round_dp(STABILITY_ROUNDING))
                    .is_zero()
            );

            let t: Dec = 1050.into();
            let p: Vector3<Dec> = origin + ray_dir * t;

            assert!(
                dbg!((p.dot(&plane1.normal) - plane1.d).round_dp(STABILITY_ROUNDING - 3)).is_zero()
            );

            assert!(
                dbg!((p.dot(&plane2.normal) - plane2.d).round_dp(STABILITY_ROUNDING - 3)).is_zero()
            );
            let in_between = (plane1.normal + plane2.normal) / Dec::from(2);

            let p: Vector3<Dec> = origin + in_between * t;

            assert!(
                !dbg!((p.dot(&plane1.normal) - plane1.d).round_dp(STABILITY_ROUNDING - 3))
                    .is_zero()
            );

            assert!(
                !dbg!((p.dot(&plane2.normal) - plane2.d).round_dp(STABILITY_ROUNDING - 3))
                    .is_zero()
            );
        } else if y > x && y > z {
            let mat = Matrix2::new(
                plane1.normal.x,
                plane1.normal.z,
                plane2.normal.x,
                plane2.normal.z,
            );
            let inv_mat = mat.try_inverse().unwrap();
            let ds = Vector2::new(plane1.d, plane2.d);
            let r = inv_mat * ds;

            dbg!(r);
            let mut origin = Vector3::zeros();
            origin.x = r.x;
            origin.z = r.y;

            assert!(
                dbg!((origin.dot(&plane1.normal) - plane1.d).round_dp(STABILITY_ROUNDING))
                    .is_zero()
            );

            assert!(
                dbg!((origin.dot(&plane2.normal) - plane2.d).round_dp(STABILITY_ROUNDING))
                    .is_zero()
            );

            let t: Dec = 1050.into();
            let p: Vector3<Dec> = origin + ray_dir * t;

            assert!(
                dbg!((p.dot(&plane1.normal) - plane1.d).round_dp(STABILITY_ROUNDING - 3)).is_zero()
            );

            assert!(
                dbg!((p.dot(&plane2.normal) - plane2.d).round_dp(STABILITY_ROUNDING - 3)).is_zero()
            );
            let in_between = (plane1.normal + plane2.normal) / Dec::from(2);

            let p: Vector3<Dec> = origin + in_between * t;

            assert!(
                !dbg!((p.dot(&plane1.normal) - plane1.d).round_dp(STABILITY_ROUNDING - 3))
                    .is_zero()
            );

            assert!(
                !dbg!((p.dot(&plane2.normal) - plane2.d).round_dp(STABILITY_ROUNDING - 3))
                    .is_zero()
            );
        } else if z > x && z > y {
            let mat = Matrix2::new(
                plane1.normal.x,
                plane1.normal.y,
                plane2.normal.x,
                plane2.normal.y,
            );
            let inv_mat = mat.try_inverse().unwrap();
            let ds = Vector2::new(plane1.d, plane2.d);
            let r = inv_mat * ds;

            dbg!(r);
            let mut origin = Vector3::zeros();
            origin.x = r.x;
            origin.y = r.y;

            assert!(
                dbg!((origin.dot(&plane1.normal) - plane1.d).round_dp(STABILITY_ROUNDING))
                    .is_zero()
            );

            assert!(
                dbg!((origin.dot(&plane2.normal) - plane2.d).round_dp(STABILITY_ROUNDING))
                    .is_zero()
            );

            let t: Dec = 1050.into();
            let p: Vector3<Dec> = origin + ray_dir * t;

            assert!(
                dbg!((p.dot(&plane1.normal) - plane1.d).round_dp(STABILITY_ROUNDING - 3)).is_zero()
            );

            assert!(
                dbg!((p.dot(&plane2.normal) - plane2.d).round_dp(STABILITY_ROUNDING - 3)).is_zero()
            );
            let in_between = (plane1.normal + plane2.normal) / Dec::from(2);

            let p: Vector3<Dec> = origin + in_between * t;

            assert!(
                !dbg!((p.dot(&plane1.normal) - plane1.d).round_dp(STABILITY_ROUNDING - 3))
                    .is_zero()
            );

            assert!(
                !dbg!((p.dot(&plane2.normal) - plane2.d).round_dp(STABILITY_ROUNDING - 3))
                    .is_zero()
            );
        }
    }

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
*/
