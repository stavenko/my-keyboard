use itertools::Itertools;
use nalgebra::{Vector2, Vector3};
use stl_io::{Triangle, Vector};
use tap::TapFallible;

use crate::{
    primitives::{decimal::Dec, plane::Plane, polygon::Polygon, polygon_basis::PolygonBasis, Face},
    spatial_index_2d::Index,
};

#[derive(Debug)]
pub struct Edge {
    pub plane: Plane,
    // pub polygon_basis: PolygonBasis,
    pub polygons: Vec<Polygon>,
}

impl Edge {
    fn get_common_basis(&self) -> PolygonBasis {
        todo!();
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
