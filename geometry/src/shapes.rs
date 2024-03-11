use crate::{basis::Basis, decimal::Dec, edge::Edge, planar::polygon::Polygon, volume::mesh::Mesh};
use nalgebra::{ComplexField, Vector3};
use num_traits::Zero;
use rust_decimal::Decimal;

pub fn rect(b: Basis, w: Dec, h: Dec, d: Dec) -> Mesh {
    let ww: Vector3<Dec> = b.x() * (w / 2);
    let hh: Vector3<Dec> = b.y() * (h / 2);
    let dd: Vector3<Dec> = b.z() * (d / 2);

    let top_basis = Basis::new(b.x(), b.z(), -b.y(), b.center() + hh).unwrap();
    let bottom_basis = Basis::new(b.x(), -b.z(), b.y(), b.center() - hh).unwrap();
    let far_basis = Basis::new(b.x(), -b.y(), -b.z(), b.center() + dd).unwrap();
    let near_basis = Basis::new(b.x(), b.y(), b.z(), b.center() - dd).unwrap();
    let left_basis = Basis::new(-b.z(), b.y(), b.x(), b.center() - ww).unwrap();
    let right_basis = Basis::new(b.z(), b.y(), -b.x(), b.center() + ww).unwrap();

    let top = Polygon::new(vec![
        top_basis.center() + ww + dd,
        top_basis.center() - ww + dd,
        top_basis.center() - ww - dd,
        top_basis.center() + ww - dd,
    ])
    .unwrap();
    let bottom = Polygon::new(vec![
        bottom_basis.center() + ww + dd,
        bottom_basis.center() + ww - dd,
        bottom_basis.center() - ww - dd,
        bottom_basis.center() - ww + dd,
    ])
    .unwrap();
    let left = Polygon::new(vec![
        left_basis.center() - hh + dd,
        left_basis.center() - hh - dd,
        left_basis.center() + hh - dd,
        left_basis.center() + hh + dd,
    ])
    .unwrap();
    let right = Polygon::new(vec![
        right_basis.center() + hh + dd,
        right_basis.center() + hh - dd,
        right_basis.center() - hh - dd,
        right_basis.center() - hh + dd,
    ])
    .unwrap();
    let near = Polygon::new(vec![
        near_basis.center() + hh + ww,
        near_basis.center() + hh - ww,
        near_basis.center() - hh - ww,
        near_basis.center() - hh + ww,
    ])
    .unwrap();
    let far = Polygon::new(vec![
        far_basis.center() - hh + ww,
        far_basis.center() - hh - ww,
        far_basis.center() + hh - ww,
        far_basis.center() + hh + ww,
    ])
    .unwrap();
    Mesh {
        sides: vec![
            Edge {
                plane: top_basis.xy(),
                polygons: vec![top],
            },
            Edge {
                plane: bottom_basis.xy(),
                polygons: vec![bottom],
            },
            Edge {
                plane: left_basis.xy(),
                polygons: vec![left],
            },
            Edge {
                plane: right_basis.xy(),
                polygons: vec![right],
            },
            Edge {
                plane: near_basis.xy(),
                polygons: vec![near],
            },
            Edge {
                plane: far_basis.xy(),
                polygons: vec![far],
            },
        ],
    }
}

pub fn cylinder(b: Basis, h: Dec, r: Dec, s: usize) -> Mesh {
    let hh: Vector3<Dec> = b.y() * (h / 2);

    let top_basis = Basis::new(b.x(), -b.z(), b.y(), b.center() + hh).unwrap();
    let mut bb = top_basis.get_polygon_basis();

    let bottom_basis = Basis::new(b.x(), b.z(), -b.y(), b.center() - hh).unwrap();
    bb.center = bottom_basis.center();

    let mut top = Vec::new();
    let mut bottom = Vec::new();
    let mut wall = Vec::new();
    let from = Dec::zero();
    for (p, n) in (0..s).zip(1..=s) {
        let ap = Dec::from(p) / Dec::from(s) * Dec::from(Decimal::TWO_PI) - from;
        let an = Dec::from(n) / Dec::from(s) * Dec::from(Decimal::TWO_PI) - from;

        let apt = top_basis.center() + top_basis.x() * ap.cos() * r + top_basis.y() * ap.sin() * r;
        let ant = top_basis.center() + top_basis.x() * an.cos() * r + top_basis.y() * an.sin() * r;

        let apb = apt - (hh * Dec::from(2));
        let anb = ant - (hh * Dec::from(2));

        let bottom_point = bottom_basis.center()
            + bottom_basis.x() * ap.cos() * r
            + bottom_basis.y() * ap.sin() * r;

        let u = apt;
        let v = ant;
        let w = anb;
        let a = v - u;
        let b = w - u;
        let z = a.cross(&b).normalize();
        let y = a.normalize();
        let x = z.cross(&y).normalize();

        let basis = Basis::new(x, z, y, (apt + apb + anb + ant) / Dec::from(4)).unwrap();
        wall.push(Edge {
            polygons: vec![Polygon::new(vec![apt, ant, anb, apb]).unwrap()],
            plane: basis.xy(),
        });
        top.push(apt);
        bottom.push(bottom_point);
    }
    wall.push(Edge {
        polygons: vec![Polygon::new(top).unwrap()],
        plane: top_basis.xy(),
    });

    wall.push(Edge {
        polygons: vec![Polygon::new(bottom).unwrap()],
        plane: bottom_basis.xy(),
    });

    Mesh { sides: wall }
}
