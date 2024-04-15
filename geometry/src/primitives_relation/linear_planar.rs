use std::ops::Neg;

use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::{Signed, Zero};

use crate::{
    decimal::{Dec, STABILITY_ROUNDING},
    edge::Edge,
    linear::{line::Line, ray::Ray, segment::Segment},
    planar::{plane::Plane, polygon::Polygon},
};

use super::{
    linear::{LinearIntersection, LinearRelation},
    linear_point::PointOnLine,
    point_planar::{PointEdgeRelation, PointPlanarRelation, PointPolygonRelation},
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
pub enum LinearPolygonRelation {
    Parallell,
    NonIntersecting,

    IntersectEdge(Segment, Vector3<Dec>),
    IntersectPlaneInside(Vector3<Dec>),
    IntersectVertex(Vector3<Dec>),
    IntersectInPlane {
        vertices: Vec<Vector3<Dec>>,
        edges: Vec<(Segment, Vector3<Dec>)>,
        common_edges: Vec<Segment>,
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

impl Relation<Polygon> for Ray {
    type Relate = LinearPolygonRelation;

    fn relate(&self, to: &Polygon) -> Self::Relate {
        let plane = to.get_plane();
        match self.relate(&plane) {
            LinearPlanarRelation::Intersect(point) => {
                for segment in to.get_segments() {
                    match segment.relate(&point) {
                        PointOnLine::On => {
                            return LinearPolygonRelation::IntersectEdge(segment.clone(), point)
                        }
                        PointOnLine::Origin => {
                            return LinearPolygonRelation::IntersectVertex(point)
                        }
                        PointOnLine::Outside => {}
                    }
                }

                if let PointPolygonRelation::In = to.relate(&point) {
                    LinearPolygonRelation::IntersectPlaneInside(point)
                } else {
                    LinearPolygonRelation::NonIntersecting
                }
            }

            LinearPlanarRelation::SamePlane => {
                let mut common_edges: Vec<Segment> = Vec::new();
                let mut edges: Vec<(Segment, Vector3<Dec>)> = Vec::new();
                let mut vertices: Vec<Vector3<Dec>> = Vec::new();
                for segment in to.get_segments() {
                    match self.relate(&segment) {
                        LinearRelation::Colinear => {
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
                            common_edges.push(segment.clone());
                        }

                        LinearRelation::Opposite => {
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
                            common_edges.push(segment.clone());
                        }
                        LinearRelation::Intersect(LinearIntersection::Origin(v)) => {
                            if !common_edges.iter().any(|s| s.has(v)) {
                                vertices.push(v);
                            }
                        }
                        LinearRelation::Intersect(LinearIntersection::In(v)) => {
                            edges.push((segment.clone(), v))
                        }
                        _ => {}
                    }
                }

                LinearPolygonRelation::IntersectInPlane {
                    common_edges,
                    edges,
                    vertices,
                }
            }
            // Ray in plane parallel to polygon
            LinearPlanarRelation::Parallell => LinearPolygonRelation::Parallell,
            // Ray looks away from polygon plane
            LinearPlanarRelation::NonIntersecting => LinearPolygonRelation::NonIntersecting,
        }
    }
}

impl Relation<Polygon> for Line {
    type Relate = LinearPolygonRelation;

    fn relate(&self, to: &Polygon) -> Self::Relate {
        let plane = to.get_plane();
        match self.relate(&plane) {
            LinearPlanarRelation::Intersect(point) => {
                for segment in to.get_segments() {
                    match segment.relate(&point) {
                        PointOnLine::On => {
                            return LinearPolygonRelation::IntersectEdge(segment.clone(), point)
                        }
                        PointOnLine::Origin => {
                            return LinearPolygonRelation::IntersectVertex(point)
                        }
                        PointOnLine::Outside => {}
                    }
                }
                LinearPolygonRelation::IntersectPlaneInside(point)
            }

            LinearPlanarRelation::SamePlane => {
                let mut common_edges: Vec<Segment> = Vec::new();
                let mut edges: Vec<(Segment, Vector3<Dec>)> = Vec::new();
                let mut vertices: Vec<Vector3<Dec>> = Vec::new();
                for segment in to.get_segments() {
                    match self.relate(&segment) {
                        LinearRelation::Colinear => {
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
                            common_edges.push(segment.clone());
                        }

                        LinearRelation::Opposite => {
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
                            common_edges.push(segment.clone());
                        }
                        LinearRelation::Intersect(LinearIntersection::Origin(v)) => {
                            if !common_edges.iter().any(|s| s.has(v))
                                && !vertices
                                    .iter()
                                    .any(|x| (x - v).magnitude_squared().round_dp(7).is_zero())
                            {
                                vertices.push(v);
                            }
                        }
                        LinearRelation::Intersect(LinearIntersection::In(v)) => {
                            edges.push((segment.clone(), v))
                        }
                        _ => {
                            //dbg!(x);
                        }
                    }
                }

                LinearPolygonRelation::IntersectInPlane {
                    common_edges,
                    edges,
                    vertices,
                }
            }
            // Ray in plane parallel to polygon
            LinearPlanarRelation::Parallell => LinearPolygonRelation::Parallell,
            // Ray looks away from polygon plane
            LinearPlanarRelation::NonIntersecting => LinearPolygonRelation::NonIntersecting,
        }
    }
}

impl Relation<Edge> for Ray {
    type Relate = LinearPolygonRelation;

    fn relate(&self, to: &Edge) -> Self::Relate {
        let plane = to.get_plane();
        match self.relate(&plane) {
            LinearPlanarRelation::Intersect(point) => {
                for segment in to.get_segments() {
                    match segment.relate(&point) {
                        PointOnLine::On => {
                            return LinearPolygonRelation::IntersectEdge(segment.clone(), point);
                        }
                        PointOnLine::Origin => {
                            return LinearPolygonRelation::IntersectVertex(point)
                        }
                        PointOnLine::Outside => {}
                    }
                }

                // This simple fact notifies us that, point in polygon
                // is almost as point in `edge`, when we use ray intersection parity method.
                // Edge holds all needed segments, thats why this method works
                if let PointEdgeRelation::In = to.relate(&point) {
                    // dbg!(&to.bound);
                    LinearPolygonRelation::IntersectPlaneInside(point)
                } else {
                    LinearPolygonRelation::NonIntersecting
                }
            }

            LinearPlanarRelation::SamePlane => {
                let mut common_edges: Vec<Segment> = Vec::new();
                let mut edges: Vec<(Segment, Vector3<Dec>)> = Vec::new();
                let mut vertices: Vec<Vector3<Dec>> = Vec::new();
                for segment in to.get_segments() {
                    match self.relate(&segment) {
                        LinearRelation::Colinear => {
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
                            common_edges.push(segment.clone());
                        }

                        LinearRelation::Opposite => {
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
                            common_edges.push(segment.clone());
                        }
                        LinearRelation::Intersect(LinearIntersection::Origin(v)) => {
                            if !common_edges.iter().any(|s| s.has(v)) {
                                vertices.push(v);
                            }
                        }
                        LinearRelation::Intersect(LinearIntersection::In(v)) => {
                            edges.push((segment.clone(), v))
                        }
                        _ => {}
                    }
                }

                LinearPolygonRelation::IntersectInPlane {
                    common_edges,
                    edges,
                    vertices,
                }
            }
            // Ray in plane parallel to polygon
            LinearPlanarRelation::Parallell => LinearPolygonRelation::Parallell,
            // Ray looks away from polygon plane
            LinearPlanarRelation::NonIntersecting => LinearPolygonRelation::NonIntersecting,
        }
    }
}
