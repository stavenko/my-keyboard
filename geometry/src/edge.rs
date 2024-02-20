use core::fmt;

use itertools::{Either, Itertools};
use nalgebra::{Vector2, Vector3};
use stl_io::{Triangle, Vector};
use tap::TapFallible;

use crate::{
    primitives::{
        basis,
        decimal::Dec,
        plane::Plane,
        polygon::Polygon,
        polygon_basis::PolygonBasis,
        segment2d::{self, Segment2D},
        Face,
    },
    spatial_index_2d::Index,
};

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
    pub fn join_polygons_on_side(mut self) -> Self {
        let basis = self.get_common_basis();
        let segments_2d = self
            .polygons
            .iter()
            .flat_map(|p| p.get_segments_with_basis(&basis))
            .collect_vec();

        let segments_2d = Segment2D::remove_opposites(segments_2d);
        match Polygon::from_segments(segments_2d, &basis) {
            Ok(polygons) => {
                self.polygons = polygons;
                self
            }
            Err(err) => {
                eprintln!("ERRROR: {}", err);
                self
            }
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
            let segments = poly.get_segments_with_basis(&basis);

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
            .map(|v| (v.x.into(), v.y.into()))
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
}
#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::{
        bsp::Bsp,
        primitives::{
            basis::Basis, decimal::Dec, line2d::Line2D, polygon::Polygon, segment2d::Segment2D,
        },
    };

    use super::Edge;

    #[test]
    fn edge_creation() {
        let h = Dec::from(2);
        let w = Dec::from(2);
        let d = Dec::from(2);
        let ww: Vector3<Dec> = Vector3::x() * (w / 2);
        let hh: Vector3<Dec> = Vector3::y() * (h / 2);
        let dd: Vector3<Dec> = Vector3::z() * (d / 2);
        let top_basis = Basis::new(
            Vector3::x(),
            -Vector3::y(),
            -Vector3::z(),
            Vector3::zeros() + hh,
        )
        .unwrap();

        let top = Polygon::new(vec![
            top_basis.center() - ww + dd,
            top_basis.center() - ww - dd,
            top_basis.center() + ww - dd,
            top_basis.center() + ww + dd,
        ])
        .unwrap();

        let segments = dbg!(top.get_segments(&top_basis.get_polygon_basis()));
        let bsp = Bsp::<Line2D, Segment2D>::build(segments).unwrap();
    }
}
