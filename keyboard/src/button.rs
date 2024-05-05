use std::borrow::Cow;

use geometry::{basis::Basis, decimal::Dec, origin::Origin, shapes};
use nalgebra::Vector3;
use rust_decimal_macros::dec;

#[derive(Clone, Debug)]
#[allow(unused)]
pub(crate) struct ButtonMount {
    pub(crate) width: Dec,
    pub(crate) height: Dec,
    pub(crate) lock_height: Dec,
    pub(crate) padding: Dec,
}

#[derive(Clone, Debug, Copy)]
pub enum ButtonMountKind {
    Chok,
    Cherry,
    Placeholder,
}

impl ButtonMountKind {
    pub(crate) fn button_width(&self) -> Dec {
        match self {
            ButtonMountKind::Chok => todo!("Width choc"),
            ButtonMountKind::Cherry => todo!("Width cherry"),
            ButtonMountKind::Placeholder => {
                let ps = self.params();
                ps.width + ps.padding
            }
        }
    }

    pub(crate) fn button_height(&self) -> Dec {
        match self {
            ButtonMountKind::Chok => todo!("Width choc"),
            ButtonMountKind::Cherry => todo!("Width cherry"),
            ButtonMountKind::Placeholder => {
                let ps = self.params();
                ps.height + ps.padding
            }
        }
    }
}

impl ButtonMountKind {
    pub(crate) fn params(&self) -> ButtonMount {
        match self {
            ButtonMountKind::Chok => ButtonMount {
                width: dec!(13.8).into(),
                height: dec!(13.8).into(),
                lock_height: dec!(1.2).into(),
                padding: dec!(0.7).into(),
            },
            ButtonMountKind::Placeholder => ButtonMount {
                width: dec!(13.8).into(),
                height: dec!(13.8).into(),
                lock_height: dec!(1.2).into(),
                padding: dec!(0.7).into(),
            },
            ButtonMountKind::Cherry => todo!(),
        }
    }
}

impl ButtonMount {
    fn chok() -> Self {
        Self {
            width: dec!(13.8).into(),
            height: dec!(13.8).into(),
            lock_height: dec!(1.2).into(),
            padding: dec!(0.7).into(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Button {
    pub origin: Origin,
    kind: ButtonMountKind,
}

impl Button {
    pub(crate) fn chok(origin: Origin) -> Self {
        Self {
            origin,
            kind: ButtonMountKind::Chok,
        }
    }

    pub(crate) fn new(origin: Origin, kind: ButtonMountKind) -> Self {
        Self { origin, kind }
    }

    pub(crate) fn inner_left_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let ps = self.kind.params();
        let left = self.origin.left() * (ps.padding + ps.width / Dec::from(dec!(2)));
        let top = self.origin.y() * (ps.padding + ps.height / Dec::from(dec!(2)));
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        left - top - up + self.origin.center
    }

    pub(crate) fn inner_left_top(&self, thickness: Dec) -> Vector3<Dec> {
        let ps = self.kind.params();
        let left = self.origin.left() * (ps.padding + ps.width / Dec::from(dec!(2)));
        let top = self.origin.top() * (ps.padding + ps.height / Dec::from(dec!(2)));
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        left + top - up + self.origin.center
    }

    pub(crate) fn outer_left_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let ps = self.kind.params();
        let left = self.origin.left() * (ps.padding + ps.width / Dec::from(dec!(2)));
        let top = self.origin.top() * (ps.padding + ps.height / Dec::from(dec!(2)));
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        left - top + up + self.origin.center
    }

    pub(crate) fn outer_left_top(&self, thickness: Dec) -> Vector3<Dec> {
        let ps = self.kind.params();
        let left = self.origin.left() * (ps.padding + ps.width / Dec::from(dec!(2)));
        let top = self.origin.top() * (ps.padding + ps.height / Dec::from(dec!(2)));
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        left + top + up + self.origin.center
    }

    pub(crate) fn inner_right_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let ps = self.kind.params();
        let right = self.origin.right() * (ps.padding + ps.width / Dec::from(dec!(2)));
        let top = self.origin.top() * (ps.padding + ps.height / Dec::from(dec!(2)));
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        right - top - up + self.origin.center
    }

    pub(crate) fn inner_right_top(&self, thickness: Dec) -> Vector3<Dec> {
        let ps = self.kind.params();
        let right = self.origin.right() * (ps.padding + ps.width / Dec::from(dec!(2)));
        let top = self.origin.top() * (ps.padding + ps.height / Dec::from(dec!(2)));
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        right + top - up + self.origin.center
    }

    pub(crate) fn outer_right_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let ps = self.kind.params();
        let right = self.origin.right() * (ps.padding + ps.width / Dec::from(dec!(2)));
        let top = self.origin.top() * (ps.padding + ps.height / Dec::from(dec!(2)));
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        right - top + up + self.origin.center
    }

    pub(crate) fn outer_right_top(&self, thickness: Dec) -> Vector3<Dec> {
        let ps = self.kind.params();
        let right = self.origin.right() * (ps.padding + ps.width / Dec::from(dec!(2)));
        let top = self.origin.top() * (ps.padding + ps.height / Dec::from(dec!(2)));
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        right + top + up + self.origin.center
    }

    pub(crate) fn mesh(
        &self,
        index: &mut geometry::indexes::geo_index::index::GeoIndex,
        thickness: Dec,
    ) {
        match self.kind {
            ButtonMountKind::Placeholder => {
                let w = self.kind.button_width();
                let h = self.kind.button_height();
                let basis = Basis {
                    center: self.origin.center,
                    x: self.origin.x(),
                    y: self.origin.y(),
                    z: self.origin.z(),
                };
                let ps = shapes::rect(basis, w, h, thickness);
                index.save_mesh(ps.into_iter().map(Cow::Owned));
            }
            _ => todo!("Implement mesh for chok and cherry"),
        }
    }
}
