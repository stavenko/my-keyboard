use std::ops::Neg;

use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::{Signed, Zero};

use crate::{
    decimal::{Dec, STABILITY_ROUNDING},
    indexes::{
        geo_index::{
            poly::PolyRef,
            rib::{RibId, RibRef},
        },
        vertex_index::PtId,
    },
    linear::{line::Line, ray::Ray},
    planar::plane::Plane,
};

use super::{
    linear::{LinearRefIntersection, LinearRefRelation},
    linear_point::PointOnLine,
    point_planar::PointPlanarRelation,
    relation::Relation,
};

#[derive(Debug, PartialEq)]
pub enum LinearPlanarRelation {
    Parallell,
    SamePlane,
    Intersect(Vector3<Dec>),
    NonIntersecting,
}

#[derive(Debug, PartialEq)]
pub enum LinearPolygonRefRelation {
    Parallell,
    NonIntersecting,

    IntersectRib(RibId, Vector3<Dec>),
    IntersectPlaneInside(Vector3<Dec>),
    IntersectVertex(Vector3<Dec>),
    IntersectInPlane {
        vertices: Vec<PtId>,
        ribs: Vec<(RibId, Vector3<Dec>)>,
        common_ribs: Vec<RibId>,
    },
}

impl Relation<Plane> for Line {
    type Relate = LinearPlanarRelation;

    fn relate(&self, to: &Plane) -> Self::Relate {
        let dot = self.dir.dot(&to.normal()).round_dp(STABILITY_ROUNDING);
        if dot.is_zero() {
            match to.relate(&self.origin) {
                PointPlanarRelation::In => LinearPlanarRelation::SamePlane,
                PointPlanarRelation::WithNormal => LinearPlanarRelation::Parallell,
                PointPlanarRelation::OpposeToNormal => LinearPlanarRelation::Parallell,
            }
        } else {
            let t = (to.normal().dot(&self.origin) - to.d()).neg() / dot;
            let p = self.dir * t + self.origin;
            LinearPlanarRelation::Intersect(p)
        }
    }
}

impl Relation<Plane> for Ray {
    type Relate = LinearPlanarRelation;

    fn relate(&self, to: &Plane) -> Self::Relate {
        let dot = self.dir.dot(&to.normal()).round_dp(STABILITY_ROUNDING);
        if dot.is_zero() {
            match to.relate(&self.origin) {
                PointPlanarRelation::In => LinearPlanarRelation::SamePlane,
                PointPlanarRelation::OpposeToNormal => LinearPlanarRelation::Parallell,
                PointPlanarRelation::WithNormal => LinearPlanarRelation::Parallell,
            }
        } else {
            let t = (to.normal().dot(&self.origin) - to.d()).neg() / dot;
            if t.is_positive() {
                let p = self.dir * t + self.origin;

                LinearPlanarRelation::Intersect(p)
            } else {
                LinearPlanarRelation::NonIntersecting
            }
        }
    }
}

impl<'a> Relation<PolyRef<'a>> for Line {
    type Relate = LinearPolygonRefRelation;

    fn relate(&self, to: &PolyRef<'a>) -> Self::Relate {
        let plane = to.get_plane();
        match self.relate(&plane) {
            LinearPlanarRelation::Intersect(point) => {
                for segment in to.get_segments() {
                    match segment.relate(&point) {
                        PointOnLine::On => {
                            return LinearPolygonRefRelation::IntersectRib(*segment.rib(), point)
                        }
                        PointOnLine::Origin => {
                            return LinearPolygonRefRelation::IntersectVertex(point)
                        }
                        PointOnLine::Outside => {}
                    }
                }
                LinearPolygonRefRelation::IntersectPlaneInside(point)
            }

            LinearPlanarRelation::SamePlane => {
                let mut common_ribs: Vec<RibRef<'a>> = Vec::new();
                let mut ribs: Vec<(RibId, Vector3<Dec>)> = Vec::new();
                let mut vertices: Vec<PtId> = Vec::new();
                for segment in to.get_segments() {
                    match self.relate(&segment) {
                        LinearRefRelation::Colinear => {
                            for px in vertices
                                .iter()
                                .enumerate()
                                .filter(|(_, &v)| segment.has(v))
                                .map(|(ix, _)| ix)
                                .rev()
                                .collect_vec()
                            {
                                vertices.swap_remove(px);
                            }
                            common_ribs.push(segment.rib());
                        }

                        LinearRefRelation::Opposite => {
                            for px in vertices
                                .iter()
                                .enumerate()
                                .filter(|(_, &v)| segment.has(v))
                                .map(|(ix, _)| ix)
                                .rev()
                                .collect_vec()
                            {
                                vertices.swap_remove(px);
                            }
                            common_ribs.push(segment.rib());
                        }
                        LinearRefRelation::Intersect(LinearRefIntersection::Zero) => {
                            let v = segment.from_pt();
                            if !common_ribs.iter().any(|s| s.has(v))
                                && !vertices.iter().any(|&x| x == v)
                            {
                                vertices.push(v);
                            }
                        }
                        LinearRefRelation::Intersect(LinearRefIntersection::One) => {
                            let v = segment.to_pt();
                            if !common_ribs.iter().any(|s| s.has(v))
                                && !vertices.iter().any(|&x| x == v)
                            {
                                vertices.push(v);
                            }
                        }
                        LinearRefRelation::Intersect(LinearRefIntersection::In(v, w)) => {
                            dbg!("!!!!!!!!!!!!!!!!!!!");
                            dbg!(v);
                            dbg!(w);

                            let v = self.origin + self.dir * v;
                            ribs.push((*segment.rib(), v))
                        }
                        _ => {
                            //dbg!(x);
                        }
                    }
                }

                LinearPolygonRefRelation::IntersectInPlane {
                    common_ribs: common_ribs.into_iter().map(|r| *r).collect_vec(),
                    ribs,
                    vertices,
                }
            }
            // Ray in plane parallel to polygon
            LinearPlanarRelation::Parallell => LinearPolygonRefRelation::Parallell,
            // Ray looks away from polygon plane
            LinearPlanarRelation::NonIntersecting => LinearPolygonRefRelation::NonIntersecting,
        }
    }
}
