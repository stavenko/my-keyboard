use nalgebra::Vector3;

use crate::{basis::Basis, decimal::Dec};

pub fn centered(b: Basis, w: Dec, h: Dec, d: Dec) -> Vec<Vec<Vector3<Dec>>> {
    let ww: Vector3<Dec> = b.x() * (w / 2);
    let hh: Vector3<Dec> = b.y() * (h / 2);
    let dd: Vector3<Dec> = b.z() * (d / 2);

    let top_basis = Basis::new(b.x(), b.z(), -b.y(), b.center() + hh).unwrap();
    let bottom_basis = Basis::new(b.x(), -b.z(), b.y(), b.center() - hh).unwrap();
    let far_basis = Basis::new(b.x(), -b.y(), -b.z(), b.center() + dd).unwrap();
    let near_basis = Basis::new(b.x(), b.y(), b.z(), b.center() - dd).unwrap();
    let left_basis = Basis::new(-b.z(), b.y(), b.x(), b.center() - ww).unwrap();
    let right_basis = Basis::new(b.z(), b.y(), -b.x(), b.center() + ww).unwrap();

    let top = vec![
        top_basis.center() + ww + dd,
        top_basis.center() - ww + dd,
        top_basis.center() - ww - dd,
        top_basis.center() + ww - dd,
    ];
    let bottom = vec![
        bottom_basis.center() + ww + dd,
        bottom_basis.center() + ww - dd,
        bottom_basis.center() - ww - dd,
        bottom_basis.center() - ww + dd,
    ];
    let left = vec![
        left_basis.center() - hh + dd,
        left_basis.center() - hh - dd,
        left_basis.center() + hh - dd,
        left_basis.center() + hh + dd,
    ];
    let right = vec![
        right_basis.center() + hh + dd,
        right_basis.center() + hh - dd,
        right_basis.center() - hh - dd,
        right_basis.center() - hh + dd,
    ];
    let near = vec![
        near_basis.center() + hh + ww,
        near_basis.center() + hh - ww,
        near_basis.center() - hh - ww,
        near_basis.center() - hh + ww,
    ];
    let far = vec![
        far_basis.center() - hh + ww,
        far_basis.center() - hh - ww,
        far_basis.center() + hh - ww,
        far_basis.center() + hh + ww,
    ];
    vec![top, bottom, right, left, near, far]
}
