use std::{
    collections::{BTreeMap, HashSet, VecDeque},
    ops::Deref,
};

use itertools::Itertools;
use nalgebra::Vector3;
use stl_io::Triangle;
use tap::TapFallible;

use crate::{
    decimal::Dec,
};

use super::{
    index::{GeoIndex, PolygonRelation},
    poly::{PolyId, PolyRef},
    rib::{RibId, RibRef},
    tri_iter::TriIter,
};

#[derive(Debug, PartialEq, Default, Clone)]
pub struct Mesh(pub(super) Vec<PolyId>);

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct MeshId(pub(super) usize);

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
            .get_mesh_polygon_ids(&self.mesh_id)
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

    pub fn boolean_union(self, tool: MeshId) -> Vec<MeshRef<'a>> {
        todo!();
    }

    fn mark_polygons(&self, mesh: MeshId, visited: &mut BTreeMap<PolyId, PolygonRelation>) {
        let mut polygons: VecDeque<PolyId> = self.geo_index.meshes[&mesh].0.clone().into();
        while let Some(polygon_id) = polygons.pop_front() {
            if visited.contains_key(&polygon_id) {
                continue;
            }
            let mut is_found = false;
            for r in self.geo_index.polygons[&polygon_id]
                .segments
                .iter()
                .map(|s| s.rib_id)
            {
                if let Some(p) = self.geo_index.rib_to_poly[&r].iter().find(|pp| {
                    self.geo_index.get_mesh_for_polygon(**pp) == self.mesh_id && **pp != polygon_id
                }) {
                    if visited.contains_key(p) {
                        visited.insert(polygon_id, visited[&p]);
                        is_found = true;
                        break;
                    }
                }
            }
            if !is_found {
                // println!("push back poly {polygon_id:?}");
                polygons.push_back(polygon_id);
            }
        }
    }

    pub fn boolean_diff(self, tool: MeshId) -> anyhow::Result<()> {
        let common_chains = self.geo_index.collect_common_chains(self.mesh_id, tool);
        let mut visited = BTreeMap::new();

        for chain in common_chains {
            visited.extend(self.geo_index.mark_polygons_around_common_chain(
                chain,
                self.mesh_id,
                tool,
            ));
        }

        self.mark_polygons(self.mesh_id, &mut visited);
        self.mark_polygons(tool, &mut visited);

        for (poly_id, relation) in visited.into_iter() {
            match relation {
                PolygonRelation::SrcPolygonFrontOfTool => {}
                PolygonRelation::SrcPolygonBackOfTool => {
                    self.geo_index.remove_polygon(poly_id);
                }
                PolygonRelation::ToolPolygonBackOfSrc => {
                    self.geo_index.flip_polygon(poly_id);
                    self.geo_index.move_polygon(poly_id, self.mesh_id);
                }
                PolygonRelation::ToolPolygonFrontOfSrc => {
                    self.geo_index.remove_polygon(poly_id);
                }
            }
        }
        if self.geo_index.meshes[&tool].0.is_empty() {
            self.geo_index.remove_mesh(tool);
        }

        Ok(())
    }

    pub fn remove(&mut self) {
        let polygon_ids = self.polygons().map(|p| p.poly_id).collect_vec();
        self.geo_index.drop_polygons(polygon_ids.into_iter());
        self.geo_index.remove_mesh(self.mesh_id);
    }
}
