use nalgebra::{Vector2, Vector3};
use num_traits::Bounded;

use crate::{
    decimal::Dec,
    polygon_basis::{self, PolygonBasis},
};

use super::geo_index::poly::PolyRef;

#[derive(Debug)]
pub struct PolygonOrientedBb {
    polygon_basis: PolygonBasis,
    min: Vector2<Dec>,
    max: Vector2<Dec>,
}

impl PolygonOrientedBb {
    pub fn create_from_poly(poly_ref: PolyRef<'_>) -> Self {
        let seg1 = poly_ref
            .get_segments()
            .next()
            .expect("polygon must at least have 3 segments");

        let plane = poly_ref.get_plane();

        let polygon_basis = PolygonBasis {
            center: plane.point_on_plane(),
            x: seg1.dir().normalize(),
            y: plane.normal().cross(&seg1.dir().normalize()).normalize(),
        };
        let mut min = Vector2::new(Dec::max_value(), Dec::max_value());
        let mut max = Vector2::new(Dec::min_value(), Dec::min_value());
        for pt in poly_ref.points() {
            let v = poly_ref.get_vertex(pt);
            let v = polygon_basis.project_on_plane_z(&v);
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);

            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
        }

        Self {
            polygon_basis,
            min,
            max,
        }
    }

    pub fn is_point_inside(&self, v: Vector3<Dec>) -> bool {
        let v = self.polygon_basis.project_on_plane_z(&v);
        v.x > self.min.x && v.x < self.max.x && v.y > self.min.y && v.y < self.max.y
    }
}
