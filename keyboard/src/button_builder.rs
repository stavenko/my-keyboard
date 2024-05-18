use geometry::{decimal::Dec, origin::Origin};
use nalgebra::Vector3;
use num_traits::{One, Zero};

use crate::{button::Button, Angle, ButtonMountKind};

pub struct ButtonBuilder {
    incline: Angle,
    additional_padding: Dec,
    depth: Dec,
    kind: ButtonMountKind,

    pub(crate) outer_right_top_edge: Vector3<Dec>,
    pub(crate) outer_right_bottom_edge: Vector3<Dec>,
    pub(crate) outer_left_top_edge: Vector3<Dec>,
    pub(crate) outer_left_bottom_edge: Vector3<Dec>,

    pub(crate) inner_right_top_edge: Vector3<Dec>,
    pub(crate) inner_right_bottom_edge: Vector3<Dec>,
    pub(crate) inner_left_top_edge: Vector3<Dec>,
    pub(crate) inner_left_bottom_edge: Vector3<Dec>,
}
impl Default for ButtonBuilder {
    fn default() -> Self {
        Self {
            incline: Default::default(),
            additional_padding: Dec::zero(),
            depth: Default::default(),
            kind: ButtonMountKind::Placeholder,
            outer_right_top_edge: Vector3::new(One::one(), One::one(), One::one()),
            outer_right_bottom_edge: Vector3::new(One::one(), One::one(), One::one()),
            outer_left_top_edge: Vector3::new(One::one(), One::one(), One::one()),
            outer_left_bottom_edge: Vector3::new(One::one(), One::one(), One::one()),
            inner_right_top_edge: Vector3::new(One::one(), One::one(), One::one()),
            inner_right_bottom_edge: Vector3::new(One::one(), One::one(), One::one()),
            inner_left_top_edge: Vector3::new(One::one(), One::one(), One::one()),
            inner_left_bottom_edge: Vector3::new(One::one(), One::one(), One::one()),
        }
    }
}

impl ButtonBuilder {
    pub fn chok() -> Self {
        Self {
            kind: ButtonMountKind::Chok,
            ..Default::default()
        }
    }

    pub fn placeholder() -> Self {
        Self {
            kind: ButtonMountKind::Placeholder,
            ..Default::default()
        }
    }

    pub fn additional_padding(mut self, padding: Dec) -> Self {
        self.additional_padding = padding;
        self
    }

    pub fn incline(mut self, incline: Angle) -> Self {
        self.incline = incline;
        self
    }

    pub fn depth(mut self, depth: Dec) -> Self {
        self.depth = depth;
        self
    }

    pub fn outer_left_top_edge(mut self, v: Vector3<Dec>) -> Self {
        self.outer_left_top_edge = v;
        self
    }

    pub fn outer_left_bottom_edge(mut self, v: Vector3<Dec>) -> Self {
        self.outer_left_bottom_edge = v;
        self
    }

    pub fn outer_right_top_edge(mut self, v: Vector3<Dec>) -> Self {
        self.outer_right_top_edge = v;
        self
    }

    pub fn outer_right_bottom_edge(mut self, v: Vector3<Dec>) -> Self {
        self.outer_right_bottom_edge = v;
        self
    }

    pub fn inner_left_top_edge(mut self, v: Vector3<Dec>) -> Self {
        self.inner_left_top_edge = v;
        self
    }

    pub fn inner_left_bottom_edge(mut self, v: Vector3<Dec>) -> Self {
        self.inner_left_bottom_edge = v;
        self
    }

    pub fn inner_right_top_edge(mut self, v: Vector3<Dec>) -> Self {
        self.inner_right_top_edge = v;
        self
    }

    pub fn inner_right_bottom_edge(mut self, v: Vector3<Dec>) -> Self {
        self.inner_right_bottom_edge = v;
        self
    }

    pub fn build(self) -> Button {
        let o = Origin::new()
            .offset_y(self.additional_padding)
            .offset_z(-self.depth);
        let x = o.x();
        let o = o.rotate_axisangle(x * self.incline.rad());
        let Self {
            outer_right_top_edge,
            outer_right_bottom_edge,
            outer_left_top_edge,
            outer_left_bottom_edge,
            inner_right_top_edge,
            inner_right_bottom_edge,
            inner_left_top_edge,
            inner_left_bottom_edge,
            ..
        } = self;
        Button {
            origin: o,
            kind: self.kind,
            outer_right_top_edge,
            outer_right_bottom_edge,
            outer_left_top_edge,
            outer_left_bottom_edge,
            inner_right_top_edge,
            inner_right_bottom_edge,
            inner_left_top_edge,
            inner_left_bottom_edge,
        }
    }
}
