use nalgebra::Vector3;
use num_traits::One;

use crate::{decimal::Dec, geometry::GeometryDyn, indexes::geo_index::index, origin::Origin};

pub struct Rect {
    width: Dec,
    height: Dec,
    depth: Dec,
    basis: Origin,
}

pub enum Align {
    Neg,
    Pos,
    Middle,
}

pub struct RectBuilder {
    width: Dec,
    height: Dec,
    depth: Dec,
    origin: Origin,
    align_z: Align,
    align_y: Align,
    align_x: Align,
}

impl RectBuilder {
    pub fn width(mut self, width: impl Into<Dec>) -> Self {
        self.width = width.into();
        self
    }

    pub fn origin(mut self, origin: Origin) -> Self {
        self.origin = origin;
        self
    }

    pub fn height(mut self, height: impl Into<Dec>) -> Self {
        self.height = height.into();
        self
    }

    pub fn depth(mut self, depth: impl Into<Dec>) -> Self {
        self.depth = depth.into();
        self
    }

    pub fn align_x(mut self, x: Align) -> Self {
        self.align_x = x;
        self
    }

    pub fn align_y(mut self, x: Align) -> Self {
        self.align_y = x;
        self
    }

    pub fn align_z(mut self, x: Align) -> Self {
        self.align_z = x;
        self
    }

    pub fn build(self) -> Rect {
        let b = match self.align_x {
            Align::Neg => self.origin.offset_x(self.width / 2),
            Align::Pos => self.origin.offset_x(-self.width / 2),
            Align::Middle => self.origin,
        };

        let b = match self.align_y {
            Align::Neg => b.offset_y(self.height / 2),
            Align::Pos => b.offset_y(-self.height / 2),
            Align::Middle => b,
        };

        let b = match self.align_z {
            Align::Neg => b.offset_z(self.depth / 2),
            Align::Pos => b.offset_z(-self.depth / 2),
            Align::Middle => b,
        };

        Rect {
            width: self.width,
            height: self.height,
            depth: self.depth,
            basis: b,
        }
    }
}

impl Default for RectBuilder {
    fn default() -> Self {
        Self {
            width: Dec::one(),
            height: Dec::one(),
            depth: Dec::one(),
            origin: Default::default(),
            align_z: Align::Middle,
            align_y: Align::Middle,
            align_x: Align::Middle,
        }
    }
}

impl Rect {
    pub fn build() -> RectBuilder {
        RectBuilder::default()
    }

    pub fn centered(b: Origin, w: Dec, h: Dec, d: Dec) -> Self {
        RectBuilder::default()
            .origin(b)
            .width(w)
            .height(h)
            .depth(d)
            .build()
    }

    pub fn with_top_at(b: Origin, w: Dec, h: Dec, d: Dec) -> Self {
        RectBuilder::default()
            .origin(b)
            .align_z(Align::Pos)
            .width(w)
            .height(h)
            .depth(d)
            .build()
    }

    pub fn with_bottom_at(b: Origin, w: Dec, h: Dec, d: Dec) -> Self {
        Self {
            width: w,
            height: h,
            depth: d,
            basis: b.offset_z(d / 2),
        }
    }

    pub fn render(&self) -> Vec<Vec<Vector3<Dec>>> {
        let ww: Vector3<Dec> = self.basis.x() * (self.width / 2);
        let hh: Vector3<Dec> = self.basis.y() * (self.height / 2);
        let dd: Vector3<Dec> = self.basis.z() * (self.depth / 2);

        let top = vec![
            self.basis.center + hh + ww - dd,
            self.basis.center + hh - ww - dd,
            self.basis.center + hh - ww + dd,
            self.basis.center + hh + ww + dd,
        ];
        let bottom = vec![
            self.basis.center - hh - ww + dd,
            self.basis.center - hh - ww - dd,
            self.basis.center - hh + ww - dd,
            self.basis.center - hh + ww + dd,
        ];
        let left = vec![
            self.basis.center - ww + hh + dd,
            self.basis.center - ww + hh - dd,
            self.basis.center - ww - hh - dd,
            self.basis.center - ww - hh + dd,
        ];
        let right = vec![
            self.basis.center + ww - hh + dd,
            self.basis.center + ww - hh - dd,
            self.basis.center + ww + hh - dd,
            self.basis.center + ww + hh + dd,
        ];
        let near = vec![
            self.basis.center - dd - hh + ww,
            self.basis.center - dd - hh - ww,
            self.basis.center - dd + hh - ww,
            self.basis.center - dd + hh + ww,
        ];
        let far = vec![
            self.basis.center + dd + hh + ww,
            self.basis.center + dd + hh - ww,
            self.basis.center + dd - hh - ww,
            self.basis.center + dd - hh + ww,
        ];
        vec![top, bottom, right, left, near, far]
    }
}

impl GeometryDyn for Rect {
    fn polygonize(&self, index: &mut index::GeoIndex, _complexity: usize) -> anyhow::Result<()> {
        for p in self.render() {
            index.save_as_polygon(&p, None)?;
        }

        Ok(())
    }
}
