/*
use crate::{planar::polygon::Polygon, volume::mesh::Mesh};

use super::relation::Relation;

pub enum MeshPolygonRelation {
    Inside,
    Outside,
    IsEdge(Vec<Polygon>),
}

impl Relation<Polygon> for Mesh {
    type Relate = MeshPolygonRelation;

    fn relate(&self, to: &Polygon) -> Self::Relate {
        let polygon_plane = to.get_plane();
        let mut coplanars = Vec::new();
        for p in self.polygons() {
            let my_polygon_plane = p.get_plane();
            match my_polygon_plane.relate(&polygon_plane) {
                super::planar::PlanarRelation::Coplanar => {
                    // return MeshPolygonRelation::IsEdge(p.clone());
                    coplanars.push(p.clone());
                }
                super::planar::PlanarRelation::Opposite => {
                    //return MeshPolygonRelation::IsEdge(p.clone());
                    coplanars.push(p.clone());
                }
                super::planar::PlanarRelation::Intersect(_line) => {
                    dbg!("Polygon planes intersect, but we already splitted staff");
                }
                super::planar::PlanarRelation::Parallel => {
                    dbg!("parallel");
                }
            }
        }
        if !coplanars.is_empty() {
            if to.vertices.iter().all(|v| self.is_point_inside(*v)) {
                MeshPolygonRelation::Inside
            } else {
                MeshPolygonRelation::Outside
            }
        } else {
            MeshPolygonRelation::IsEdge(coplanars)
        }
    }
}
*/
