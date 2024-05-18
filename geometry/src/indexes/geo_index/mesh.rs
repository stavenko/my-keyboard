use std::{collections::HashSet, iter, ops::Deref};

use itertools::Itertools;
use nalgebra::Vector3;
use stl_io::Triangle;
use tap::TapFallible;
use uuid::Uuid;

use crate::{
    decimal::Dec,
    indexes::geo_index::poly::Side,
    measure_time::print_time,
    primitives_relation::{point_volume::PointVolumeRelation, relation::Relation},
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
pub struct MeshId(pub(super) usize);

#[derive(Debug)]
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
            index: &self.geo_index,
        }
    }

    pub(crate) fn get_rib_ref(&self, rib: RibId) -> RibRef<'a> {
        RibRef {
            index: &self.geo_index,
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
    pub fn boolean_union(self, tool: MeshId) -> Vec<MeshRef<'a>> {
        println!("==== start boolean union ====");
        //let tool_mesh_id = tool;
        let mut checkpoint = std::time::Instant::now();
        let tot = checkpoint;
        // let tool_mesh_id = self.geo_index.save_mesh(tool.sides());
        let mut total_splitted_polygons = 0;

        while let Some((poly, tool)) = self.geo_index.find_first_intersection(&tool) {
            let plane = self.geo_index.calculate_polygon_plane(tool);
            self.geo_index.split_poly_by_plane(poly, &plane);
            total_splitted_polygons += 1;
        }
        //checkpoint = print_time("find intersection in my", checkpoint);

        while let Some((poly, tool)) = self.geo_index.find_first_intersection(&self.mesh_id) {
            let plane = self.geo_index.calculate_polygon_plane(tool);
            self.geo_index.split_poly_by_plane(poly, &plane);
            total_splitted_polygons += 1;
            /*
            self.geo_index.wtf_wtf_wtf();
            if total_splitted_polygons == 5 {
                return vec![self.geo_index.get_mesh(tool_mesh_id)];
            }
            */
        }

        //panic!("DEB");
        //checkpoint = print_time("find intersection in tool", checkpoint);
        println!("Total splitted: {total_splitted_polygons}");

        /*
                let my_vertices = self.geo_index.get_mesh_vertices(self.mesh_id);
                let my_vertices_inside_tool = my_vertices.iter().any(|v| {
                    matches!(
                        self.geo_index.is_point_inside_mesh(v, tool),
                        PointVolumeRelation::In
                    )
                });
                let tool_vertices = self.geo_index.get_mesh_vertices(tool);

                let tool_vertices_inside_me = my_vertices.iter().any(|v| {
                    matches!(
                        self.geo_index.is_point_inside_mesh(v, tool),
                        PointVolumeRelation::In
                    )
                });

        */
        let to_drop_inside_tool = self.geo_index.find_insides(self.mesh_id, tool);
        //checkpoint = print_time("find dropped polygons #1", checkpoint);
        let to_drop_inside_self = self.geo_index.find_insides(tool, self.mesh_id);
        //checkpoint = print_time("find dropped polygons #2", checkpoint);
        let total_polygons_to_remove = to_drop_inside_self
            .into_iter()
            .chain(to_drop_inside_tool)
            .collect_vec();
        println!(
            "Found polygons to remove {}",
            total_polygons_to_remove.len()
        );

        self.geo_index
            .drop_polygons(total_polygons_to_remove.into_iter());
        //checkpoint = print_time("drop polygons", checkpoint);

        if self.geo_index.has_common_ribs() {
            self.geo_index.remove_opposites_and_collapse_sames();
            checkpoint = print_time("drop opposites", checkpoint);
        }

        if self.geo_index.has_common_ribs() {
            self.geo_index.inspect_common_ribs();
            panic!("Something is wrong");
        }

        let meshes = self.geo_index.collect_meshes();
        print_time("collect", checkpoint);
        print_time("total", tot);

        meshes
    }

    pub fn boolean_diff(self, tool: MeshId) -> Result<Vec<MeshRef<'a>>, (PolyId, PolyId)> {
        // insert tool polygons in index
        //let tool_mesh_id = self.geo_index.save_mesh(tool.sides());

        let mut ps_cutted = HashSet::new();
        while let Some((poly, tool)) = self.geo_index.find_first_intersection(&tool) {
            // dbg!("aaa");
            let plane = self.geo_index.calculate_polygon_plane(tool);
            ps_cutted.insert(poly);
            match self.geo_index.split_poly_by_plane(poly, &plane) {
                Ok(ps) => ps_cutted.extend(ps),
                Err(_) => {
                    return Err((poly, tool));
                }
            };
        }

        while let Some((poly, tool)) = self.geo_index.find_first_intersection(&self.mesh_id) {
            let plane = self.geo_index.calculate_polygon_plane(tool);

            match self.geo_index.split_poly_by_plane(poly, &plane) {
                Ok(ps) => ps_cutted.extend(ps),
                Err(_) => {
                    return Err((poly, tool));
                }
            };
        }
        dbg!("@");

        let ps1 = self
            .geo_index
            .mark_based_on_cutted_polygons(self.mesh_id, tool, Side::Front);

        let ps = self
            .geo_index
            .mark_based_on_cutted_polygons(tool, self.mesh_id, Side::Back);

        let lll = dbg!(ps_cutted.len());
        //self.geo_index.drop_polygons(ps1.into_iter().chain(ps));
        let around_ribs = ps1.into_iter().chain(ps).collect::<HashSet<_>>();
        dbg!(&around_ribs);
        let ccc = ps_cutted
            .difference(&around_ribs)
            .filter(|p| self.geo_index.polygons.contains_key(p))
            .collect_vec();
        let lll = ccc.len();

        dbg!(lll);
        for p in ccc.iter() {
            for r in self.geo_index.get_polygon_ribs(p) {
                if self.geo_index.rib_to_poly[&r].len() == 3 {
                    panic!("WTF!");
                }
            }

            self.geo_index.flip_polygon(**p);
            println!("{:?}", p);
        }

        //self.geo_index.drop_polygons(ps.into_iter());
        /*
        let to_drop_inside_tool = self.geo_index.find_insides(self.mesh_id, tool);
        let to_drop_outside_self = self.geo_index.find_outsides(tool, self.mesh_id);

        self.geo_index
            .drop_polygons(to_drop_outside_self.into_iter().chain(to_drop_inside_tool));

        */
        self.geo_index.remove_opposites_and_collapse_sames();

        self.geo_index.inverse_mesh(tool);

        //self.geo_index.join_mesh_bounds_in_same_plane();
        Ok(self.geo_index.collect_meshes())
    }
}
