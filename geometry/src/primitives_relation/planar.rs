use itertools::Itertools;
use nalgebra::{ComplexField, Matrix2, Vector2, Vector3};
use num_traits::{One, Zero};

use crate::{
    decimal::{Dec, STABILITY_ROUNDING},
    linear::{line::Line, segment::Segment},
    planar::{plane::Plane, polygon::Polygon},
};

use super::{point_planar::PointPlanarRelation, relation::Relation};

#[derive(Debug)]
pub enum PlanarRelation {
    Coplanar,
    Intersect(Line),
    Opposite,
    Parallel,
}

#[allow(unused)]
pub enum PolygonRelation {
    Coplanar,
    Opposite,
    CommonVertex(Vector3<Dec>),
    CommonEdge(Segment),
    Split(Polygon, Polygon), // TODO: Replace to vectors
    NonIntersecting,
}

impl Relation<Plane> for Plane {
    type Relate = PlanarRelation;

    fn relate(&self, to: &Plane) -> Self::Relate {
        let dir = self.normal().cross(&to.normal());

        if dir
            .magnitude_squared()
            .round_dp(STABILITY_ROUNDING)
            .is_zero()
        {
            if self
                .normal()
                .dot(&to.normal())
                .round_dp(STABILITY_ROUNDING - 3)
                .is_one()
            {
                if (self.d() - to.d())
                    .round_dp(STABILITY_ROUNDING - 3)
                    .is_zero()
                {
                    PlanarRelation::Coplanar
                } else {
                    PlanarRelation::Parallel
                }
            } else if (self.d() + to.d()).round_dp(STABILITY_ROUNDING).is_zero() {
                PlanarRelation::Opposite
            } else {
                PlanarRelation::Parallel
            }
        } else {
            let dir_len = dir.magnitude_squared().round_dp(STABILITY_ROUNDING).sqrt();
            let dir = dir / dir_len;
            let x = dir.x.abs();
            let y = dir.y.abs();
            let z = dir.z.abs();

            #[allow(clippy::nonminimal_bool)]
            if (x > y && x > z) || (x == y && x > z) {
                let mat = Matrix2::new(
                    self.normal().y,
                    self.normal().z,
                    to.normal().y,
                    to.normal().z,
                );
                let inv_mat = mat.try_inverse().unwrap();
                let ds = Vector2::new(self.d(), to.d());
                let r = inv_mat * ds;

                let mut origin = Vector3::zeros();
                origin.y = r.x;
                origin.z = r.y;

                PlanarRelation::Intersect(Line { origin, dir })
            } else if y > x && y > z || (y == z && y > x) {
                let mat = Matrix2::new(
                    self.normal().x,
                    self.normal().z,
                    to.normal().x,
                    to.normal().z,
                );
                let inv_mat = mat.try_inverse().unwrap();
                let ds = Vector2::new(self.d(), to.d());
                let r = inv_mat * ds;

                let mut origin = Vector3::zeros();
                origin.x = r.x;
                origin.z = r.y;

                PlanarRelation::Intersect(Line { origin, dir })
            } else if (z > x && z > y) || (x == z && x > y) {
                let mat = Matrix2::new(
                    self.normal().x,
                    self.normal().y,
                    to.normal().x,
                    to.normal().y,
                );
                let inv_mat = mat.try_inverse().unwrap();
                let ds = Vector2::new(self.d(), to.d());
                let r = inv_mat * ds;

                let mut origin = Vector3::zeros();
                origin.x = r.x;
                origin.y = r.y;

                PlanarRelation::Intersect(Line { origin, dir })
            } else {
                unreachable!();
            }
        }
    }
}

impl Plane {
    pub fn split(&self, item: &Polygon) -> (Option<Polygon>, Option<Polygon>) {
        let relations = item.vertices.iter().map(|v| self.relate(v)).collect_vec();

        let has_back = relations
            .iter()
            .any(|f| *f == PointPlanarRelation::OpposeToNormal);
        let has_front = relations
            .iter()
            .any(|f| *f == PointPlanarRelation::WithNormal);
        match (has_back, has_front) {
            (true, true) => {
                let (f, b) = self.split_fb(item);
                (Some(f), Some(b))
            }
            (false, true) => (Some(item.clone()), None),
            (true, false) => (None, Some(item.clone())),
            (false, false) => {
                unreachable!("ensure, that no coplanars here")
            }
        }
    }
    fn split_fb(&self, polygon: &Polygon) -> (Polygon, Polygon) {
        let mut front = Vec::new();
        let mut back = Vec::new();
        let len = polygon.vertices.len();
        for (i, current) in polygon.vertices.iter().enumerate() {
            let j = (i + 1) % len;
            let next = polygon.vertices[j];
            let relation_current = self.relate(current);
            let relation_next = self.relate(&next);
            match relation_current {
                PointPlanarRelation::OpposeToNormal => {
                    back.push(*current);
                }
                PointPlanarRelation::WithNormal => {
                    front.push(*current);
                }
                PointPlanarRelation::In => {
                    front.push(*current);
                    back.push(*current);
                }
            }
            match (&relation_current, &relation_next) {
                (PointPlanarRelation::WithNormal, PointPlanarRelation::OpposeToNormal) => {
                    let d = (next - self.point_on_plane())
                        .dot(&self.normal())
                        .round_dp(STABILITY_ROUNDING);
                    let t = (d / (next - current).dot(&self.normal())).round_dp(STABILITY_ROUNDING);
                    assert!(t >= Dec::zero());
                    assert!(t <= Dec::one());
                    let u = next.lerp(current, t);
                    front.push(u);
                    back.push(u);
                }
                (PointPlanarRelation::OpposeToNormal, PointPlanarRelation::WithNormal) => {
                    let d = (current - self.point_on_plane())
                        .dot(&self.normal())
                        .round_dp(STABILITY_ROUNDING);
                    let t = (d / (current - next).dot(&self.normal())).round_dp(STABILITY_ROUNDING);
                    assert!(t >= Dec::zero());
                    assert!(t <= Dec::one());
                    let u = current.lerp(&next, t);
                    front.push(u);
                    back.push(u);
                }
                (PointPlanarRelation::WithNormal, PointPlanarRelation::WithNormal) => {}
                (PointPlanarRelation::OpposeToNormal, PointPlanarRelation::OpposeToNormal) => {}
                (PointPlanarRelation::OpposeToNormal, PointPlanarRelation::In) => {}
                (PointPlanarRelation::WithNormal, PointPlanarRelation::In) => {}
                (PointPlanarRelation::In, PointPlanarRelation::WithNormal) => {}
                (PointPlanarRelation::In, PointPlanarRelation::OpposeToNormal) => {}
                a => {
                    dbg!(a);
                    unreachable!()
                }
            }
        }
        (
            Polygon::new_with_plane(front, polygon.get_plane().to_owned()).unwrap(),
            Polygon::new_with_plane(back, polygon.get_plane().to_owned()).unwrap(),
        )
    }
}
