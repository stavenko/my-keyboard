use nalgebra::Vector3;
use rust_decimal_macros::dec;

use geometry::primitives::{decimal::Dec, origin::Origin};

#[derive(Clone, Debug)]
struct ButtonMount {
    width: Dec,
    height: Dec,
    lock_height: Dec,
    padding: Dec,
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
    mount: ButtonMount,
}

impl Button {
    pub(crate) fn chok(origin: Origin) -> Self {
        Self {
            origin,
            mount: ButtonMount::chok(),
        }
    }

    pub(crate) fn inner_left_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let left = self.origin.left() * (self.mount.padding + self.mount.width / Dec::from(2.0));
        let top = self.origin.y() * (self.mount.padding + self.mount.height / Dec::from(2.0));
        let up = self.origin.z() * thickness / Dec::from(2.0);
        left - top - up + self.origin.center
    }

    pub(crate) fn inner_left_top(&self, thickness: Dec) -> Vector3<Dec> {
        let left = self.origin.left() * (self.mount.padding + self.mount.width / Dec::from(2.0));
        let top = self.origin.top() * (self.mount.padding + self.mount.height / Dec::from(2.0));
        let up = self.origin.z() * thickness / Dec::from(2.0);
        left + top - up + self.origin.center
    }

    pub(crate) fn outer_left_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let left = self.origin.left() * (self.mount.padding + self.mount.width / Dec::from(2.0));
        let top = self.origin.top() * (self.mount.padding + self.mount.height / Dec::from(2.0));
        let up = self.origin.z() * thickness / Dec::from(2.0);
        left - top + up + self.origin.center
    }

    pub(crate) fn outer_left_top(&self, thickness: Dec) -> Vector3<Dec> {
        let left = self.origin.left() * (self.mount.padding + self.mount.width / Dec::from(2.0));
        let top = self.origin.top() * (self.mount.padding + self.mount.height / Dec::from(2.0));
        let up = self.origin.z() * thickness / Dec::from(2.0);
        left + top + up + self.origin.center
    }

    pub(crate) fn inner_right_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let right = self.origin.right() * (self.mount.padding + self.mount.width / Dec::from(2.0));
        let top = self.origin.top() * (self.mount.padding + self.mount.height / Dec::from(2.0));
        let up = self.origin.z() * thickness / Dec::from(2.0);
        right - top - up + self.origin.center
    }

    pub(crate) fn inner_right_top(&self, thickness: Dec) -> Vector3<Dec> {
        let right = self.origin.right() * (self.mount.padding + self.mount.width / Dec::from(2.0));
        let top = self.origin.top() * (self.mount.padding + self.mount.height / Dec::from(2.0));
        let up = self.origin.z() * thickness / Dec::from(2.0);
        right + top - up + self.origin.center
    }

    pub(crate) fn outer_right_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let right = self.origin.right() * (self.mount.padding + self.mount.width / Dec::from(2.0));
        let top = self.origin.top() * (self.mount.padding + self.mount.height / Dec::from(2.0));
        let up = self.origin.z() * thickness / Dec::from(2.0);
        right - top + up + self.origin.center
    }

    pub(crate) fn outer_right_top(&self, thickness: Dec) -> Vector3<Dec> {
        let right = self.origin.right() * (self.mount.padding + self.mount.width / Dec::from(2.0));
        let top = self.origin.top() * (self.mount.padding + self.mount.height / Dec::from(2.0));
        let up = self.origin.z() * thickness / Dec::from(2.0);
        right + top + up + self.origin.center
    }
}
