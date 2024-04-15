use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::{Signed, Zero};

use crate::{
    decimal::{Dec, STABILITY_ROUNDING},
    edge::Edge,
    linear::{ray::Ray, segment::Segment},
    planar::plane::Plane,
    primitives_relation::linear_planar::LinearPolygonRelation,
    volume::mesh::Mesh,
};

use super::{point_planar::PointEdgeRelation, relation::Relation};

#[derive(Debug, Clone, PartialEq)]
pub enum PointVolumeRelation {
    In,
    Out,
    Edge(Segment),
    Vertex,
    Side(Edge),
}

impl Relation<Vector3<Dec>> for Mesh {
    type Relate = PointVolumeRelation;

    fn relate(&self, to: &Vector3<Dec>) -> Self::Relate {
        let pt = self.sides().next().expect("we have a side").bound.middle();
        let ray = Ray {
            origin: *to,
            dir: (pt - to).normalize(),
        };

        let mut parity = 0;
        let mut checked_edges: Vec<Segment> = Vec::new();
        for edge in self.sides() {
            match edge.relate(to) {
                PointEdgeRelation::In => return PointVolumeRelation::Side(edge.into_owned()),
                PointEdgeRelation::Edge(segment) => return PointVolumeRelation::Edge(segment),
                PointEdgeRelation::Vertex => return PointVolumeRelation::Vertex,
                PointEdgeRelation::WithNormal => {}
                PointEdgeRelation::OpposeToNormal => {}
                PointEdgeRelation::InPlane => {}
            }

            match ray.relate(edge.as_ref()) {
                LinearPolygonRelation::IntersectPlaneInside(_) => {
                    /*
                    println!(
                        "ok {} {} {} ",
                        p.x.round_dp(6),
                        p.y.round_dp(6),
                        p.z.round_dp(6)
                    );
                    */
                    parity += 1;
                }
                LinearPolygonRelation::IntersectEdge(s, p) => {
                    let flipped = s.to_owned().flip();
                    if !checked_edges
                        .iter()
                        .any(|checked_segment| *checked_segment == flipped)
                    {
                        checked_edges.push(s.clone());
                        let mut polygons = self.polygons().collect_vec();
                        // This plane i a plane perpendicular to edge in point of intersection.
                        // Project ray, and and faces vectors on this plane to remove problems with
                        // non-strict crossing
                        let plane = Plane::new_from_normal_and_point(s.dir().normalize(), p);

                        if let Some(ix1) = polygons
                            .iter()
                            .position(|p| p.get_segments().iter().any(|ps| *ps == s))
                        {
                            let poly1 = polygons.swap_remove(ix1);
                            let si = s.flip();
                            if let Some(ix2) = polygons
                                .iter()
                                .position(|p| p.get_segments().iter().any(|ps| *ps == si))
                            {
                                let poly2 = polygons.swap_remove(ix2);
                                let p1 = poly1
                                    .vertices
                                    .iter()
                                    .map(Clone::clone)
                                    .fold(Vector3::zeros(), |a, b| a + b)
                                    / Dec::from(poly1.vertices.len());
                                let p2 = poly2
                                    .vertices
                                    .iter()
                                    .map(Clone::clone)
                                    .fold(Vector3::zeros(), |a, b| a + b)
                                    / Dec::from(poly2.vertices.len());
                                //println!("p1 {} {} {}", p1.x, p1.y, p1.z);
                                //println!("p2 {} {} {}", p2.x, p2.y, p2.z);
                                let p1 = p1 - p;
                                let p2 = p2 - p;
                                let p1_plane = p1 - plane.normal() * plane.normal().dot(&p1);
                                let p2_plane = p2 - plane.normal() * plane.normal().dot(&p2);
                                let ray_dir_plane = (ray.dir
                                    - plane.normal() * plane.normal().dot(&ray.dir))
                                .normalize();

                                let p1p: Vector3<Dec> =
                                    ray_dir_plane * ray_dir_plane.dot(&p1_plane);
                                let p1n: Vector3<Dec> = (p1_plane - p1p).normalize();
                                //dbg!(&p1p);
                                //dbg!(&p1n);
                                //dbg!(p1n + p1p);
                                // dbg!(p1n.dot(&ray_dir_plane));
                                // println!("wtf");
                                if p1n.dot(&p2_plane).is_negative() {
                                    parity += 1;
                                }

                                //panic!("dbg");
                                //TODO Beware: when this ray hits segment, and both polygons on this
                                //segment at 180 degree, This will fail
                                /*
                                if c1.dot(&c2).is_negative() {
                                    dbg!(c1.dot(&c2));
                                    parity += 1;
                                }
                                */
                            }
                        }
                    }
                }
                LinearPolygonRelation::IntersectVertex(v) => {
                    let mut dirs = Vec::new();
                    for p in self.polygons() {
                        for s in p.get_segments() {
                            if (s.from - v)
                                .magnitude_squared()
                                .round_dp(STABILITY_ROUNDING - 2)
                                .is_zero()
                            {
                                dirs.push(s.to - v);
                            }
                        }
                    }
                    let total_dir = dirs.iter().fold(Vector3::zeros(), |a, v| a + v);
                    let total_dir = &total_dir.normalize();
                    let dir_dir = ray.dir * ray.dir.dot(total_dir);
                    let norm_dir = total_dir - dir_dir;
                    let norm_dir = norm_dir.normalize();

                    // println!("possible problem #2");
                    if dirs.iter().all(|d| d.dot(&norm_dir).is_negative()) {
                        parity += 1;
                    }

                    //todo!("implement vertex intersection {v}")
                }
                LinearPolygonRelation::IntersectInPlane { .. } => {
                    // this similar to problem, when ray from point is collateral with on of
                    // polygon segments
                }
                LinearPolygonRelation::NonIntersecting => {}
                LinearPolygonRelation::Parallell => {}
            }
        }

        if parity % 2 == 1 {
            // dbg!(parity);
            PointVolumeRelation::In
        } else {
            PointVolumeRelation::Out
        }
    }
}
#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use num_traits::{One, Zero};

    use crate::{
        basis::Basis,
        decimal::Dec,
        planar::polygon::Polygon,
        primitives_relation::{point_volume::PointVolumeRelation, relation::Relation},
        shapes,
    };

    #[test]
    fn is_point_inside_mesh() {
        let zz = Vector3::z();
        let yy = Vector3::y();
        let xx = yy.cross(&zz).normalize();

        let _zero_basis = Basis::new(xx, yy, zz, Vector3::zeros()).unwrap();

        let box_mesh = shapes::rect(
            _zero_basis.clone(),
            Dec::one() * 2,
            Dec::one() * 2,
            Dec::one() * 2,
        );

        let pt0 = Vector3::zeros();
        assert_eq!(box_mesh.relate(&pt0), PointVolumeRelation::In);
    }

    #[test]
    fn is_polygon_inside() {
        let zz = Vector3::z();
        let yy = Vector3::y();
        let xx = yy.cross(&zz).normalize();

        let _zero_basis = Basis::new(xx, yy, zz, Vector3::zeros()).unwrap();

        let box_mesh = shapes::rect(
            _zero_basis.clone(),
            Dec::one() * 4,
            Dec::one() * 4,
            Dec::one() * 4,
        );

        let poly = Polygon::new(vec![
            Vector3::new(Dec::one() * 2, Dec::one() * 2, Dec::zero()),
            Vector3::new(Dec::one() * 2, Dec::one() * -2, Dec::zero()),
            Vector3::new(Dec::one() * -2, Dec::one() * -2, Dec::zero()),
            Vector3::new(Dec::one() * -2, Dec::one() * 2, Dec::zero()),
        ])
        .unwrap();

        assert_eq!(box_mesh.drop_full_insides(vec![poly]), Vec::new());

        let poly = Polygon::new(vec![
            Vector3::new(Dec::one() * 2, Dec::zero(), Dec::zero()),
            Vector3::new(Dec::zero(), Dec::one() * 2, Dec::zero()),
            Vector3::new(Dec::one() * -2, Dec::zero(), Dec::zero()),
            Vector3::new(Dec::zero(), Dec::one() * -2, Dec::zero()),
        ])
        .unwrap();

        assert_eq!(box_mesh.drop_full_insides(vec![poly]), Vec::new());

        let poly = Polygon::new(vec![
            Vector3::new(Dec::one() * 2, Dec::one() * 2, Dec::one() * 2),
            Vector3::new(Dec::one() * 2, Dec::one() * -2, Dec::zero()),
            Vector3::new(Dec::one() * -2, Dec::one() * -2, Dec::one() * -2),
            Vector3::new(Dec::one() * -2, Dec::one() * 2, Dec::zero()),
        ])
        .unwrap();

        assert_eq!(box_mesh.drop_full_insides(vec![poly]), Vec::new());
    }
}
