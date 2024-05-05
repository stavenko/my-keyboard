use itertools::Itertools;
use nalgebra::{Vector, Vector3};
use num_traits::{Signed, Zero};

use crate::{
    decimal::Dec,
    indexes::geo_index::{mesh::MeshRef, poly::PolyId},
    linear::ray::Ray,
    planar::plane::Plane,
};

use super::{
    linear_planar::LinearPolygonRefRelation, point_planar::PointPolygonRefRelation,
    relation::Relation,
};

#[derive(Debug, Clone, PartialEq)]
pub enum PointVolumeRelation {
    In,
    Out,
    Edge,
    Vertex,
    Side,
}

impl<'a> Relation<Vector3<Dec>> for MeshRef<'a> {
    type Relate = PointVolumeRelation;

    fn relate(&self, to: &Vector3<Dec>) -> Self::Relate {
        let pt = self.center();
        let ray = Ray {
            origin: *to,
            dir: (pt - to).normalize(),
        };

        let mut parity = 0;
        let mut intersected_ribs = Vec::new();
        let mut intersected_vertices = Vec::new();
        //dbg!(self.polygons().count());
        for polygon in self.polygons() {
            match polygon.relate(to) {
                PointPolygonRefRelation::In => return PointVolumeRelation::Side,
                PointPolygonRefRelation::Rib(_) => return PointVolumeRelation::Edge,
                PointPolygonRefRelation::Vertex => return PointVolumeRelation::Vertex,
                PointPolygonRefRelation::WithNormal => {}
                PointPolygonRefRelation::OpposeToNormal => {}
                PointPolygonRefRelation::InPlane => {}
            }

            match ray.relate(&polygon) {
                LinearPolygonRefRelation::IntersectPlaneInside(pt) => {
                    /*
                    println!(
                        "ok {} {} {} ",
                        p.x.round_dp(6),
                        p.y.round_dp(6),
                        p.z.round_dp(6)
                    );
                    */
                    //println!("p ");
                    parity += 1;
                }
                LinearPolygonRefRelation::IntersectRib(s, p) => {
                    intersected_ribs.push((s, p));
                }
                LinearPolygonRefRelation::IntersectVertex(v) => {
                    intersected_vertices.push(v);
                }
                LinearPolygonRefRelation::IntersectInPlane { .. } => {
                    // this similar to problem, when ray from point is collateral with on of
                    // polygon segments
                }
                LinearPolygonRefRelation::NonIntersecting => {}
                LinearPolygonRefRelation::Parallell => {}
            }
        }

        for vertex in intersected_vertices {
            todo!("Fuck me");
        }

        for (rib, point) in intersected_ribs {
            let [poly1, poly2] = <Vec<PolyId> as TryInto<[PolyId; 2]>>::try_into(
                self.get_mesh_polygons_for_rib(rib),
            )
            .expect("Only two polygons for rib supported");

            let p1 = self.get_poly_ref(poly1).middle();
            let p2 = self.get_poly_ref(poly2).middle();

            let rib_dir = self.get_rib_ref(rib).dir().normalize();

            let plane = Plane::new_from_normal_and_point(rib_dir, point);

            let p1 = p1 - point;
            let p2 = p2 - point;

            let first_polygon_direction_in_plane_perpendecular_to_rib =
                p1 - plane.normal() * plane.normal().dot(&p1);
            let second_polygon_direction_in_plane_perpendecular_to_rib =
                p2 - plane.normal() * plane.normal().dot(&p2);

            // Ray dir in plane, perpendecular to rib
            let ray_dir_plane =
                (ray.dir - plane.normal() * plane.normal().dot(&ray.dir)).normalize();

            if is_ray_between_rays(
                plane.normal(),
                ray_dir_plane,
                first_polygon_direction_in_plane_perpendecular_to_rib,
                second_polygon_direction_in_plane_perpendecular_to_rib,
            ) {
                parity += 1;
            }
        }

        if parity % 2 == 1 {
            PointVolumeRelation::In
        } else {
            PointVolumeRelation::Out
        }
    }
}

pub fn is_ray_between_rays(
    plane_normal: Vector3<Dec>,
    tested_ray: Vector3<Dec>,
    ray_one: Vector3<Dec>,
    ray_two: Vector3<Dec>,
) -> bool {
    let ray_perpendicular = tested_ray.cross(&plane_normal).normalize();
    let one_dot = ray_perpendicular.dot(&ray_one).round_dp(9);
    let other_dot = ray_perpendicular.dot(&ray_two).round_dp(9);
    //dbg!(one_dot);
    //dbg!(other_dot);
    let is_ray_on_some_ray = one_dot.is_zero() || other_dot.is_zero();

    (one_dot * other_dot).is_negative() && !is_ray_on_some_ray
}
