pub mod dynamic_surface;
pub mod polygon_from_line_in_plane;
pub mod primitive_dynamic_surface;
pub mod simple_dynamic_surface;

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use num_traits::Zero;

    use crate::{
        decimal::Dec,
        geometry::Geometry,
        hyper_path::{
            hyper_line::HyperLine,
            hyper_path::{HyperPath, Root},
            hyper_point::HyperPointT,
        },
        indexes::{
            aabb::Aabb,
            geo_index::{geo_object::GeoObject, index::GeoIndex},
        },
    };

    use super::dynamic_surface::DynamicSurface;

    #[test]
    fn join_1_1() {
        let l1 = HyperLine::new_2(
            HyperPointT {
                normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                point: Vector3::zeros(),
            },
            HyperPointT {
                normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                point: Vector3::x(),
            },
        );

        let hp = Root::new().push_back(l1);
        let l2 = HyperLine::new_2(
            HyperPointT {
                normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                point: Vector3::z(),
            },
            HyperPointT {
                normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                point: Vector3::x() + Vector3::z(),
            },
        );
        let hp2 = Root::new().push_back(l2);
        let hs = DynamicSurface::new(hp, hp2);
        let mut ix = GeoIndex::new(Aabb::from_points(&[
            Vector3::new(Dec::from(-50), Dec::from(-50), Dec::from(-50)),
            Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
        ]));
        let mesh = ix.new_mesh();
        let mut mm = mesh.make_mut_ref(&mut ix);
        hs.polygonize(&mut mm, 10).unwrap();
    }

    #[test]
    fn join_2_2() {
        let hp = Root::new()
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::zeros(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x(),
                },
            ))
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x(),
                },
            ));
        let hp2 = Root::new()
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::z(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::z(),
                },
            ))
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::z(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x() + Vector3::z(),
                },
            ));
        let hs = DynamicSurface::new(hp, hp2);
        let mut ix = GeoIndex::new(Aabb::from_points(&[
            Vector3::new(Dec::from(-50), Dec::from(-50), Dec::from(-50)),
            Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
        ]));
        let mesh = ix.new_mesh();
        let mut mm = mesh.make_mut_ref(&mut ix);
        hs.polygonize(&mut mm, 10).unwrap();
    }
    #[test]
    fn join_3_2() {
        let hp = Root::new()
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::zeros(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x(),
                },
            ))
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x(),
                },
            ));
        let hp2 = Root::new()
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::z(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::z(),
                },
            ))
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::z() + Vector3::x(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x() + Vector3::z(),
                },
            ))
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x() + Vector3::z(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x() + Vector3::x() + Vector3::z(),
                },
            ));
        let hs = DynamicSurface::new(hp2, hp);
        let mut ix = GeoIndex::new(Aabb::from_points(&[
            Vector3::new(Dec::from(-50), Dec::from(-50), Dec::from(-50)),
            Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
        ]));
        let mesh = ix.new_mesh();
        let mut mm = mesh.make_mut_ref(&mut ix);
        hs.polygonize(&mut mm, 10).unwrap();
    }

    #[test]
    fn join_2_3() {
        let hp = Root::new()
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::zeros(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x(),
                },
            ))
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x(),
                },
            ));
        let hp2 = Root::new()
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::z(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::z(),
                },
            ))
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::z() + Vector3::x(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x() + Vector3::z(),
                },
            ))
            .push_back(HyperLine::new_2(
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x() + Vector3::z(),
                },
                HyperPointT {
                    normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                    dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                    point: Vector3::x() + Vector3::x() + Vector3::x() + Vector3::z(),
                },
            ));
        let hs = DynamicSurface::new(hp, hp2);
        let mut ix = GeoIndex::new(Aabb::from_points(&[
            Vector3::new(Dec::from(-50), Dec::from(-50), Dec::from(-50)),
            Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
        ]));
        let mesh = ix.new_mesh();
        let mut mm = mesh.make_mut_ref(&mut ix);
        hs.polygonize(&mut mm, 10).unwrap();
    }
}
