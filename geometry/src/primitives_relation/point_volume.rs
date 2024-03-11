use nalgebra::Vector3;
use num_traits::{Signed, Zero};

use crate::{
    decimal::{Dec, STABILITY_ROUNDING},
    linear::{ray::Ray, segment::Segment},
    planar::polygon::Polygon,
    primitives_relation::linear_planar::LinearPolygonRelation,
    volume::mesh::Mesh,
};

use super::{point_planar::PointPolygonRelation, relation::Relation};

#[derive(Debug, PartialEq)]
pub enum PointVolumeRelation {
    In,
    Out,
    Edge(Segment),
    Vertex,
    Side(Polygon),
}

impl Relation<Vector3<Dec>> for Mesh {
    type Relate = PointVolumeRelation;

    fn relate(&self, to: &Vector3<Dec>) -> Self::Relate {
        let ray = Ray {
            origin: *to,
            dir: Vector3::y(),
        };

        let mut parity = 0;
        for p in self.polygons() {
            match p.relate(to) {
                PointPolygonRelation::In => return PointVolumeRelation::Side(p),
                PointPolygonRelation::Edge(segment) => return PointVolumeRelation::Edge(segment),
                PointPolygonRelation::Vertex => return PointVolumeRelation::Vertex,
                PointPolygonRelation::WithNormal => {}
                PointPolygonRelation::OpposeToNormal => {}
                PointPolygonRelation::InPlane => {}
            }

            match ray.relate(&p) {
                LinearPolygonRelation::IntersectPlane(_) => {
                    parity += 1;
                }
                LinearPolygonRelation::IntersectEdge(s) => {
                    let mut polygons = self.polygons();

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
                            let p1 = p1 - ray.origin;
                            let p2 = p2 - ray.origin;
                            let c1: Vector3<Dec> = ray.dir.cross(&p1);
                            let c2 = ray.dir.cross(&p2);

                            if c1.dot(&c2).is_negative() {
                                parity += 1;
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
