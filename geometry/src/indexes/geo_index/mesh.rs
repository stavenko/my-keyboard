use std::{
    collections::{HashMap, HashSet},
    ops::Deref,
};

use itertools::Itertools;
use nalgebra::Vector3;
use stl_io::Triangle;
use tap::TapFallible;

use crate::{
    decimal::Dec,
    indexes::{geo_index::poly::Side, vertex_index::PtId},
    planar::polygon::Polygon,
};

use super::{
    index::GeoIndex,
    poly::{PolyId, PolyRef},
    rib::{RibId, RibRef},
    tri_iter::TriIter,
};

#[derive(Debug, PartialEq, Default, Clone)]
pub struct Mesh(pub(super) Vec<PolyId>);

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct MeshId(pub usize);

impl PartialEq<usize> for MeshId {
    fn eq(&self, other: &usize) -> bool {
        self.0 == *other
    }
}

#[derive(Debug, Clone)]
pub struct MeshRef<'a> {
    pub(super) geo_index: &'a GeoIndex,
    pub(super) mesh_id: MeshId,
}

impl<'a> Deref for MeshRef<'a> {
    type Target = MeshId;

    fn deref(&self) -> &Self::Target {
        &self.mesh_id
    }
}

impl<'a> MeshRef<'a> {
    pub(crate) fn polygons(&self) -> impl Iterator<Item = PolyRef<'a>> + 'a {
        self.geo_index.polygons(&self.mesh_id)
    }

    pub(crate) fn get_mesh_polygons_for_rib(&self, rib: RibId) -> Vec<PolyId> {
        let mesh_polygons = self
            .geo_index
            .get_mesh_polygon_ids(self.mesh_id)
            .collect::<HashSet<_>>();

        let rib_polygons = self
            .geo_index
            .rib_to_poly
            .get(&rib)
            .into_iter()
            .flatten()
            .copied()
            .collect::<HashSet<_>>();

        rib_polygons.intersection(&mesh_polygons).copied().collect()
    }

    pub(crate) fn get_poly_ref(&self, poly1: PolyId) -> PolyRef<'a> {
        PolyRef {
            poly_id: poly1,
            index: self.geo_index,
        }
    }

    pub(crate) fn get_rib_ref(&self, rib: RibId) -> RibRef<'a> {
        RibRef {
            index: self.geo_index,
            rib_id: rib,
        }
    }

    pub(crate) fn center(&self) -> Vector3<Dec> {
        let points = self.geo_index.get_mesh_vertices(self.mesh_id);
        let sum: Vector3<Dec> = points
            .iter()
            .map(|p| self.geo_index.vertices.get_point(*p))
            .sum();
        sum / Dec::from(points.len())
    }
}

#[derive(Debug)]
pub struct MeshRefMut<'a> {
    pub(super) geo_index: &'a mut GeoIndex,
    pub(super) mesh_id: MeshId,
}

impl<'a> IntoIterator for MeshRef<'a> {
    type Item = Triangle;

    type IntoIter = TriIter;

    fn into_iter(self) -> Self::IntoIter {
        let triangles = self
            .geo_index
            .polygons(&self.mesh_id)
            .filter_map(|polygon| {
                polygon
                    .triangles()
                    .tap_err(|e| {
                        dbg!(e);
                    })
                    .ok()
            })
            .flatten()
            .collect_vec();

        let size = dbg!(triangles.len());

        TriIter {
            inner: triangles.into_iter(),
            size,
        }
    }
}

impl<'a> MeshRefMut<'a> {
    pub(crate) fn polygons(&'a self) -> impl Iterator<Item = PolyRef<'a>> + 'a {
        self.geo_index.polygons(&self.mesh_id)
    }

    /*
    pub fn boolean_union(self, tool: MeshId) -> Vec<MeshRef<'a>> {
        todo!();
    }

    pub fn boolean_diff_rust(self, tool: MeshId) -> anyhow::Result<Vec<MeshRef<'a>>> {
        let common_chains = self.geo_index.collect_common_chains(self.mesh_id, tool);
        let visited = HashMap::new();

        for chain in common_chains {
            for seg in chain {
                let polygons = self.geo_index.rib_to_poly[&seg.rib_id]
                    .iter()
                    .map(|p| (*p, self.geo_index.get_mesh_for_polygon(*p)))
                    .collect::<HashMap<_, _>>();

                let planes = polygons
                    .iter()
                    .map(|(poly_id, mesh_id)| {
                        (mesh_id, self.geo_index.polygons[poly_id].plane.clone())
                    })
                    .collect::<HashMap<_, _>>();

                for (polygon_id, mesh_id) in polygons.iter() {
                    let points = self.geo_index.get_polygon_points(*polygon_id);
                    let rib_points = [
                        self.geo_index.ribs[&seg.rib_id].0,
                        self.geo_index.ribs[&seg.rib_id].1,
                    ]
                    .into_iter()
                    .collect::<HashSet<_>>();
                    let plane = planes[mesh_id];
                    points
                        .difference(&rib_points)
                        .map(|pt| {
                            let v = self.geo_index.vertices.get_point(pt);
                            plane.normal().dot(&v) - plane.d()
                        })
                        .s
                }
            }
        }

        Err(anyhow::anyhow!("Implementing"))
    }
    */

    pub fn remove(&mut self) {
        let polygon_ids = self.polygons().map(|p| p.poly_id).collect_vec();
        self.geo_index.drop_polygons(polygon_ids.into_iter());
        self.geo_index.remove_mesh(self.mesh_id);
    }
}
