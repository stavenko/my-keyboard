use nalgebra::Vector3;

use crate::{decimal::Dec, origin::Origin};

pub struct Rect {
    width: Dec,
    height: Dec,
    depth: Dec,
    basis: Origin,
}

impl Rect {
    pub fn centered(b: Origin, w: Dec, h: Dec, d: Dec) -> Self {
        Self {
            width: w,
            height: h,
            depth: d,
            basis: b,
        }
    }

    pub fn render(self) -> Vec<Vec<Vector3<Dec>>> {
        let ww: Vector3<Dec> = self.basis.x() * (self.width / 2);
        let hh: Vector3<Dec> = self.basis.y() * (self.height / 2);
        let dd: Vector3<Dec> = self.basis.z() * (self.depth / 2);

        let top = vec![
            self.basis.center + hh + ww + dd,
            self.basis.center + hh - ww + dd,
            self.basis.center + hh - ww - dd,
            self.basis.center + hh + ww - dd,
        ];
        let bottom = vec![
            self.basis.center - hh + ww + dd,
            self.basis.center - hh + ww - dd,
            self.basis.center - hh - ww - dd,
            self.basis.center - hh - ww + dd,
        ];
        let left = vec![
            self.basis.center - ww - hh + dd,
            self.basis.center - ww - hh - dd,
            self.basis.center - ww + hh - dd,
            self.basis.center - ww + hh + dd,
        ];
        let right = vec![
            self.basis.center + ww + hh + dd,
            self.basis.center + ww + hh - dd,
            self.basis.center + ww - hh - dd,
            self.basis.center + ww - hh + dd,
        ];
        let near = vec![
            self.basis.center - dd + hh + ww,
            self.basis.center - dd + hh - ww,
            self.basis.center - dd - hh - ww,
            self.basis.center - dd - hh + ww,
        ];
        let far = vec![
            self.basis.center + dd - hh + ww,
            self.basis.center + dd - hh - ww,
            self.basis.center + dd + hh - ww,
            self.basis.center + dd + hh + ww,
        ];
        vec![top, bottom, right, left, near, far]
    }
}
