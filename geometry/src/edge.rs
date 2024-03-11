use crate::planar::face::Face;
use crate::spatial_index_2d::Index;
use core::fmt;

use itertools::Itertools;
use nalgebra::{Vector2, Vector3};
use stl_io::{Triangle, Vector};
use tap::TapFallible;

use crate::{
    decimal::Dec,
    linear::segment::Segment,
    planar::{plane::Plane, polygon::Polygon},
    polygon_basis::PolygonBasis,
};

#[derive(Clone)]
pub struct Edge {
    pub plane: Plane,
    pub polygons: Vec<Polygon>,
}

impl fmt::Debug for Edge {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Edge")
            .field("plane", &self.plane)
            .field("polygons_len", &self.polygons.len())
            .finish()
    }
}

impl Edge {
    pub fn get_segments(&self) -> Vec<Segment> {
        self.polygons
            .iter()
            .flat_map(|p| p.get_segments())
            .collect()
    }

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

    fn get_common_basis(&self) -> PolygonBasis {
        // we got plane, so we at least have one point
        //
        let u = self.plane.normal() * self.plane.d();
        let Some(v) = self
            .polygons
            .iter()
            .flat_map(|p| p.vertices.iter())
            .find(|v| **v != u)
        else {
            panic!(
                "With {} polygons we cannot find point, that differs from one vertex on plane",
                self.polygons.len()
            );
        };

        let x = (v - u).normalize();
        let y = self.plane.normal().cross(&x).normalize();
        PolygonBasis { center: u, x, y }
    }

    pub fn triangles(&self) -> anyhow::Result<Vec<Triangle>> {
        let mut index = Index::default();
        let mut contours = Vec::new();
        let basis = self.get_common_basis();
        self.polygons.iter().for_each(|poly| {
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
            plane: p.get_plane(),
            polygons: vec![p],
        }
    }
}
