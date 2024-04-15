use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::{Signed, Zero};

use crate::{
    decimal::{Dec, NORMAL_DOT_ROUNDING, STABILITY_ROUNDING},
    edge::Edge,
    linear::{line::Line, ray::Ray, segment::Segment},
    planar::{plane::Plane, polygon::Polygon},
};

use super::{
    linear::{LinearIntersection, LinearRelation},
    relation::Relation,
};

#[derive(PartialEq, Debug)]
pub enum PointPlanarRelation {
    In,
    WithNormal,
    OpposeToNormal,
    //Edge(Segment),
    //Vertex,
}

#[derive(PartialEq, Debug)]
pub enum PointPolygonRelation {
    In,
    WithNormal,
    OpposeToNormal,
    InPlane,
    Edge(Segment),
    Vertex,
}

#[derive(PartialEq, Debug)]
pub enum PointEdgeRelation {
    In,
    WithNormal,
    OpposeToNormal,
    InPlane,
    Edge(Segment),
    Vertex,
}

impl Relation<Vector3<Dec>> for Plane {
    type Relate = PointPlanarRelation;

    fn relate(&self, to: &Vector3<Dec>) -> Self::Relate {
        let distance = (self.normal().dot(to) - self.d()).round_dp(NORMAL_DOT_ROUNDING);

        if distance.is_zero() {
            PointPlanarRelation::In
        } else if distance.is_positive() {
            PointPlanarRelation::WithNormal
        } else {
            PointPlanarRelation::OpposeToNormal
        }
    }
}

impl Relation<Vector3<Dec>> for Polygon {
    type Relate = PointPolygonRelation;

    fn relate(&self, to: &Vector3<Dec>) -> Self::Relate {
        match self.get_plane().relate(to) {
            PointPlanarRelation::In => {
                let ray = {
                    let Line { dir, .. } = self.get_segments()[0].clone().into();
                    Ray { dir, origin: *to }
                };

                let mut edges_crossed = 0;
                let mut vertices = Vec::new();
                for segment in self.get_segments() {
                    match segment.relate(to) {
                        super::linear_point::PointOnLine::On => {
                            return PointPolygonRelation::Edge(segment);
                        }
                        super::linear_point::PointOnLine::Origin => {
                            return PointPolygonRelation::Vertex
                        }
                        super::linear_point::PointOnLine::Outside => {}
                    }

                    match ray.relate(&segment) {
                        LinearRelation::Intersect(LinearIntersection::Origin(v)) => {
                            vertices.push(v);
                        }
                        LinearRelation::Intersect(LinearIntersection::In(_)) => {
                            edges_crossed += 1;
                        }
                        _ => {}
                    }
                }

                let mut all_segments = self.get_segments();
                for v in vertices {
                    if let Some(p1) = all_segments.iter().position(|s| {
                        (s.from - v)
                            .magnitude_squared()
                            .round_dp(STABILITY_ROUNDING)
                            .is_zero()
                    }) {
                        let s1 = all_segments.swap_remove(p1);

                        if let Some(p2) = all_segments.iter().position(|s| {
                            (s.to - v)
                                .magnitude_squared()
                                .round_dp(STABILITY_ROUNDING - 2)
                                .is_zero()
                        }) {
                            let s2 = all_segments.swap_remove(p2);
                            let p1 = s1.to - ray.origin;
                            let p2 = s2.from - ray.origin;

                            let c1 = ray.dir.cross(&p1);
                            let c2 = ray.dir.cross(&p2);

                            if c1.dot(&c2).is_negative() {
                                edges_crossed += 1;
                            }
                        }
                    }
                }

                if edges_crossed % 2 == 1 {
                    PointPolygonRelation::In
                } else {
                    PointPolygonRelation::InPlane
                }
            }
            PointPlanarRelation::WithNormal => PointPolygonRelation::WithNormal,
            PointPlanarRelation::OpposeToNormal => PointPolygonRelation::OpposeToNormal,
        }
    }
}

impl Relation<Vector3<Dec>> for Edge {
    type Relate = PointEdgeRelation;

    fn relate(&self, to: &Vector3<Dec>) -> Self::Relate {
        //println!("~~~~~~~~~~~~~edge to pt ~~~~~~");
        match self.get_plane().relate(to) {
            PointPlanarRelation::In => {
                let ray = {
                    let Line { dir, .. } = self.get_segments().next().unwrap().into();
                    Ray { dir, origin: *to }
                };

                let mut edges_crossed = 0;
                let mut vertices = Vec::new();
                for segment in self.get_segments() {
                    match segment.relate(to) {
                        super::linear_point::PointOnLine::On => {
                            return PointEdgeRelation::Edge(segment);
                        }
                        super::linear_point::PointOnLine::Origin => {
                            return PointEdgeRelation::Vertex
                        }
                        super::linear_point::PointOnLine::Outside => {}
                    }

                    match ray.relate(&segment) {
                        LinearRelation::Intersect(LinearIntersection::Origin(v)) => {
                            vertices.push(v);
                        }
                        LinearRelation::Intersect(LinearIntersection::In(_)) => {
                            // println!("-----------------=/=------------");
                            edges_crossed += 1;
                        }
                        _ => {}
                    }
                }

                let mut all_segments = self.get_segments().collect_vec();
                for v in vertices {
                    if let Some(p1) = all_segments.iter().position(|s| {
                        (s.from - v)
                            .magnitude_squared()
                            .round_dp(STABILITY_ROUNDING)
                            .is_zero()
                    }) {
                        let s1 = all_segments.swap_remove(p1);

                        if let Some(p2) = all_segments.iter().position(|s| {
                            (s.to - v)
                                .magnitude_squared()
                                .round_dp(STABILITY_ROUNDING - 2)
                                .is_zero()
                        }) {
                            let s2 = all_segments.swap_remove(p2);
                            let p1 = s1.to - v;
                            let p2 = s2.from - v;
                            let p1n = p1 - ray.dir * (ray.dir.dot(&p1));

                            {
                                //let p1n = p1n.normalize();
                                //println!("{}", p1n.dot(&ray.dir));
                                //println!("{}", p1n.dot(&p2).round_dp(6).is_negative());
                            }

                            //let c1 = ray.dir.cross(&p1);
                            //let c2 = ray.dir.cross(&p2);

                            if p1n.dot(&p2).round_dp(6).is_negative() {
                                //println!("------------------/-------------");
                                edges_crossed += 1;
                            }
                        }
                    }
                }

                if edges_crossed % 2 == 1 {
                    PointEdgeRelation::In
                } else {
                    PointEdgeRelation::InPlane
                }
            }
            PointPlanarRelation::WithNormal => PointEdgeRelation::WithNormal,
            PointPlanarRelation::OpposeToNormal => PointEdgeRelation::OpposeToNormal,
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use rust_decimal_macros::dec;

    use crate::{
        linear::ray::Ray,
        planar::polygon::Polygon,
        primitives_relation::{linear_planar::LinearPolygonRelation, relation::Relation},
    };

    #[test]
    fn ray_crosses_poly() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(1).into()),
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(-1).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(-1).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(1).into()),
        ];
        let poly = Polygon::new(points.to_vec()).unwrap();

        let pt0 = Vector3::zeros();
        let ray = Ray {
            origin: pt0,
            dir: Vector3::x(),
        };
        assert_eq!(
            ray.relate(&poly),
            LinearPolygonRelation::IntersectPlaneInside(Vector3::x())
        );
    }
}
