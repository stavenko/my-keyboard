use crate::{
    indexes::quadtree::Quadtree,
    planar::{face::Face, plane::Plane},
};
use core::fmt;

use itertools::Itertools;
use nalgebra::{Vector2, Vector3};
use stl_io::{Triangle, Vector};
use tap::TapFallible;

use crate::{
    decimal::Dec, linear::segment::Segment, planar::polygon::Polygon, polygon_basis::PolygonBasis,
};

#[derive(Clone, PartialEq)]
pub struct Edge {
    pub bound: Polygon,
    pub holes: Vec<Polygon>,
}

impl fmt::Debug for Edge {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Edge")
            .field("plane", &self.bound.get_plane())
            .field("holes", &self.holes.len())
            .finish()
    }
}

impl Edge {
    pub fn get_segments(&self) -> impl Iterator<Item = Segment> + '_ {
        self.bound
            .get_segments()
            .into_iter()
            .chain(self.holes.iter().flat_map(|p| p.get_segments()))
    }

    pub(crate) fn polygons_ref(&self) -> impl Iterator<Item = &Polygon> {
        [&self.bound].into_iter().chain(self.holes.iter())
    }

    pub(crate) fn polygons_owned(self) -> impl Iterator<Item = Polygon> {
        [self.bound].into_iter().chain(self.holes)
    }
    /*

    pub fn join_polygons_on_side(mut self) -> Option<Self> {
        let segments = self
            .polygons
            .iter()
            .flat_map(|p| p.get_segments())
            .collect_vec();

        let segments = Segment::remove_opposites(segments);
        if !segments.is_empty() {
            match Polygon::from_segments(segments) {
                Ok(polygons) => {
                    self.polygons = polygons;
                    Some(self)
                }
                Err(err) => {
                    eprintln!("ERROR: {}", err);
                    Some(self)
                }
            }
        } else {
            None
        }
    }
    */

    fn get_common_basis(&self) -> PolygonBasis {
        // we got plane, so we at least have one point
        //
        let plane = self.bound.get_plane();
        let u = plane.normal() * plane.d();
        let Some(v) = self.bound.vertices.iter().find(|v| **v != u) else {
            panic!(
                "Cannot find point in main polygon, that is not origin of polygons plane. And I need this point",
            );
        };

        let x = (v - u).normalize();
        let y = plane.normal().cross(&x).normalize();
        PolygonBasis { center: u, x, y }
    }

    pub fn triangles(&self) -> anyhow::Result<Vec<Triangle>> {
        let mut index = Quadtree::default();
        let mut contours = Vec::new();
        let basis = self.get_common_basis();

        self.polygons_ref().for_each(|poly| {
            let segments = poly.get_segments_2d(&basis);

            for s in &segments {
                index.insert(s.from);
                index.insert(s.to);
            }
            contours.push(segments);
        });
        let contours: Vec<Vec<usize>> = contours
            .into_iter()
            .map(|c| {
                c.into_iter()
                    .filter_map(|s| index.get_point_index(&s.from))
                    .collect_vec()
            })
            .map(|mut c| {
                if let Some(first) = c.first() {
                    c.push(*first);
                }
                c
            })
            .collect_vec();

        let tup_array: Vec<_> = index
            .linearize()
            .into_iter()
            .map(|v| (v.x.round_dp(9).into(), v.y.round_dp(9).into()))
            .collect_vec();

        let mut t = cdt::Triangulation::new_from_contours(&tup_array, &contours).tap_err(|e| {
            panic!("{}", e);
        })?;
        while !t.done() {
            t.step().tap_err(|e| {
                panic!("{}", e);
            })?;
        }

        let result = t
            .triangles()
            .map(|(a, b, c)| {
                let a: Vector3<Dec> =
                    basis.unproject(&Vector2::new(tup_array[a].0.into(), tup_array[a].1.into()));

                let b: Vector3<Dec> =
                    basis.unproject(&Vector2::new(tup_array[b].0.into(), tup_array[b].1.into()));
                let c: Vector3<Dec> =
                    basis.unproject(&Vector2::new(tup_array[c].0.into(), tup_array[c].1.into()));

                let face = Face::new([a, b, c]);

                Triangle {
                    normal: Vector::new([
                        face.normal.x.into(),
                        face.normal.y.into(),
                        face.normal.z.into(),
                    ]),
                    vertices: face
                        .vertices
                        .map(|na| Vector::new([na.x.into(), na.y.into(), na.z.into()])),
                }
            })
            .collect::<Vec<_>>();
        Ok(result)
    }

    pub(crate) fn from_polygon(p: Polygon) -> Self {
        Self {
            bound: p,
            holes: Vec::new(),
        }
    }

    pub(crate) fn get_plane(&self) -> Plane {
        self.bound.get_plane()
    }
}
