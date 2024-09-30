use geometry::{
    decimal::Dec,
    indexes::geo_index::{
        geo_object::GeoObject,
        index::GeoIndex,
        mesh::{MeshId, MeshRefMut},
    },
    origin::Origin,
};
use nalgebra::Vector3;
use rust_decimal_macros::dec;

use crate::{button_builder::ButtonBuilder, chok_hotswap::ChokHotswap};

#[derive(Clone, Debug, Default)]
#[allow(unused)]
pub(crate) struct ButtonMount {
    pub(crate) width: Dec,
    pub(crate) height: Dec,
    pub(crate) lock_width: Dec,
    pub(crate) lock_height: Dec,
    pub(crate) lock_depth: Dec,
    pub(crate) lock_inner_padding: Dec,
    pub(crate) around_button_padding: Dec,
}

#[derive(Clone, Debug, Copy)]
pub enum ButtonMountKind {
    Chok,
    Cherry,
    Placeholder,
    ChokHotswapCustom,
}

impl ButtonMountKind {
    pub(crate) fn button_width(&self) -> Dec {
        match self {
            ButtonMountKind::Chok => {
                let ps = self.params();
                ps.width + ps.around_button_padding
            }
            ButtonMountKind::Cherry => todo!("Width cherry"),
            ButtonMountKind::Placeholder => {
                let ps = self.params();
                ps.width
            }
            ButtonMountKind::ChokHotswapCustom => {
                let a = ChokHotswap::new();
                a.width()
            }
        }
    }

    pub(crate) fn button_height(&self) -> Dec {
        match self {
            ButtonMountKind::Chok => {
                let ps = self.params();
                ps.height + ps.around_button_padding
            }
            ButtonMountKind::Cherry => todo!("Width cherry"),
            ButtonMountKind::Placeholder => {
                let ps = self.params();
                ps.height
            }
            ButtonMountKind::ChokHotswapCustom => {
                let a = ChokHotswap::new();
                a.height()
            }
        }
    }
}

impl ButtonMountKind {
    pub(crate) fn params(&self) -> ButtonMount {
        match self {
            ButtonMountKind::Chok => ButtonMount {
                width: dec!(18).into(),
                height: dec!(18).into(),
                lock_width: dec!(13.8).into(),
                lock_height: dec!(13.8).into(),
                lock_depth: dec!(1.2).into(),
                lock_inner_padding: 1.into(),
                around_button_padding: 1.into(),
            },
            ButtonMountKind::Placeholder => ButtonMount {
                width: dec!(18).into(),
                height: dec!(18).into(),
                ..Default::default()
            },
            ButtonMountKind::Cherry => todo!(),
            ButtonMountKind::ChokHotswapCustom => unreachable!(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Button {
    pub origin: Origin,
    pub(super) kind: ButtonMountKind,

    pub(crate) outer_right_top_edge: Vector3<Dec>,
    pub(crate) outer_right_bottom_edge: Vector3<Dec>,
    pub(crate) outer_left_top_edge: Vector3<Dec>,
    pub(crate) outer_left_bottom_edge: Vector3<Dec>,

    pub(crate) inner_right_top_edge: Vector3<Dec>,
    pub(crate) inner_right_bottom_edge: Vector3<Dec>,
    pub(crate) inner_left_top_edge: Vector3<Dec>,
    pub(crate) inner_left_bottom_edge: Vector3<Dec>,
}

impl Button {
    pub fn chok() -> ButtonBuilder {
        ButtonBuilder::chok()
    }
    pub fn chok_hotswap_custom() -> ButtonBuilder {
        ButtonBuilder::chok_hotswap_custom()
    }

    pub fn placeholder() -> ButtonBuilder {
        ButtonBuilder::placeholder()
    }

    pub(crate) fn inner_left_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let w = self.kind.button_width();
        let h = self.kind.button_height();
        let left = self.origin.left() * w / Dec::from(2);
        let top = self.origin.top() * h / Dec::from(2);
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        left - top - up + self.origin.center
    }

    pub(crate) fn inner_left_top(&self, thickness: Dec) -> Vector3<Dec> {
        let w = self.kind.button_width();
        let h = self.kind.button_height();
        let left = self.origin.left() * w / Dec::from(2);
        let top = self.origin.top() * h / Dec::from(2);
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        left + top - up + self.origin.center
    }

    pub(crate) fn outer_left_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let w = self.kind.button_width();
        let h = self.kind.button_height();
        let left = self.origin.left() * w / Dec::from(2);
        let top = self.origin.top() * h / Dec::from(2);
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        left - top + up + self.origin.center
    }

    pub(crate) fn outer_left_top(&self, thickness: Dec) -> Vector3<Dec> {
        let w = self.kind.button_width();
        let h = self.kind.button_height();
        let left = self.origin.left() * w / Dec::from(2);
        let top = self.origin.top() * h / Dec::from(2);
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        left + top + up + self.origin.center
    }

    pub(crate) fn inner_right_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let w = self.kind.button_width();
        let h = self.kind.button_height();
        let right = self.origin.right() * w / Dec::from(2);
        let top = self.origin.top() * h / Dec::from(2);
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        right - top - up + self.origin.center
    }

    pub(crate) fn inner_right_top(&self, thickness: Dec) -> Vector3<Dec> {
        let w = self.kind.button_width();
        let h = self.kind.button_height();
        let right = self.origin.right() * w / Dec::from(2);
        let top = self.origin.top() * h / Dec::from(2);
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        right + top - up + self.origin.center
    }

    pub(crate) fn outer_right_bottom(&self, thickness: Dec) -> Vector3<Dec> {
        let w = self.kind.button_width();
        let h = self.kind.button_height();
        let right = self.origin.right() * w / Dec::from(2);
        let top = self.origin.top() * h / Dec::from(2);
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        right - top + up + self.origin.center
    }

    pub(crate) fn outer_right_top(&self, thickness: Dec) -> Vector3<Dec> {
        let w = self.kind.button_width();
        let h = self.kind.button_height();
        let right = self.origin.right() * w / Dec::from(2);
        let top = self.origin.top() * h / Dec::from(2);
        let up = self.origin.z() * thickness / Dec::from(dec!(2));
        right + top + up + self.origin.center
    }

    pub fn pt(&self, v: Vector3<Dec>) -> Vector3<Dec> {
        self.origin.center + self.origin.x() * v.x + self.origin.y() * v.y + self.origin.z() * v.z
    }

    pub(crate) fn mesh(&self, index: &mut GeoIndex, thickness: Dec) -> anyhow::Result<MeshId> {
        match self.kind {
            ButtonMountKind::Placeholder => {
                let mesh_id = index.new_mesh();
                let top = [
                    self.outer_left_top(thickness),
                    self.outer_left_bottom(thickness),
                    self.outer_right_bottom(thickness),
                    self.outer_right_top(thickness),
                ];
                let bottom = [
                    self.inner_right_top(thickness),
                    self.inner_right_bottom(thickness),
                    self.inner_left_bottom(thickness),
                    self.inner_left_top(thickness),
                ];

                let mut mesh = mesh_id.make_mut_ref(index);
                mesh.add_polygon(&top)?;
                mesh.add_polygon(&bottom)?;
                Ok(mesh_id)
            }
            ButtonMountKind::Chok => {
                let mesh_id = index.new_mesh();
                let mut mesh = mesh_id.make_mut_ref(index);
                let ps = self.kind.params();
                let outer_btn_width = ps.width + ps.around_button_padding;
                let outer_btn_height = ps.height + ps.around_button_padding;

                #[rustfmt::skip]
                let top_pl1 = [
                    self.pt(Vector3::new( ps.lock_width / 2, ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( ps.lock_width / 2, -ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( outer_btn_width / 2, -outer_btn_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( outer_btn_width / 2, outer_btn_height / 2, thickness / 2,)),

                ];

                #[rustfmt::skip]
                let top_pl2 = [
                    self.pt(Vector3::new( ps.lock_width / 2, -ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( -ps.lock_width / 2, -ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( -outer_btn_width / 2, -outer_btn_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( outer_btn_width / 2, -outer_btn_height / 2, thickness / 2,)),

                ];

                #[rustfmt::skip]
                let top_pl3 = [
                    self.pt(Vector3::new( -ps.lock_width / 2, -ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( -ps.lock_width / 2, ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( -outer_btn_width / 2, outer_btn_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( -outer_btn_width / 2, -outer_btn_height / 2, thickness / 2,)),
                ];

                #[rustfmt::skip]
                let top_pl4 = [
                    self.pt(Vector3::new( -ps.lock_width / 2, ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( ps.lock_width / 2, ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( outer_btn_width / 2, outer_btn_height / 2, thickness / 2,)),

                    self.pt(Vector3::new( -outer_btn_width / 2, outer_btn_height / 2, thickness / 2,)),

                ];

                #[rustfmt::skip]
                let tr = [
                    self.pt(Vector3::new( ps.lock_width / 2, ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( ps.lock_width / 2, ps.lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new( ps.lock_width / 2, -ps.lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new( ps.lock_width / 2, -ps.lock_height / 2, thickness / 2 )),
                ];

                #[rustfmt::skip]
                let tl = [
                    self.pt(Vector3::new( -ps.lock_width / 2, -ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( -ps.lock_width / 2, -ps.lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new( -ps.lock_width / 2, ps.lock_height / 2, thickness / 2 - ps.lock_depth,)),

                    self.pt(Vector3::new( -ps.lock_width / 2, ps.lock_height / 2, thickness / 2,)),
                ];

                #[rustfmt::skip]
                let tb = [
                    self.pt(Vector3::new( ps.lock_width / 2, -ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( ps.lock_width / 2, -ps.lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new( -ps.lock_width / 2, -ps.lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new( -ps.lock_width / 2, -ps.lock_height / 2, thickness / 2,)),
                ];

                #[rustfmt::skip]
                let mut tt = [
                    self.pt(Vector3::new( ps.lock_width / 2, ps.lock_height / 2, thickness / 2,)),
                    self.pt(Vector3::new( ps.lock_width / 2, ps.lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new( -ps.lock_width / 2, ps.lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new( -ps.lock_width / 2, ps.lock_height / 2, thickness / 2,)),
                ];
                tt.reverse();

                mesh.add_polygon(&top_pl1)?;
                mesh.add_polygon(&top_pl2)?;
                mesh.add_polygon(&top_pl3)?;
                mesh.add_polygon(&top_pl4)?;
                mesh.add_polygon(&tr)?;
                mesh.add_polygon(&tl)?;
                mesh.add_polygon(&tb)?;
                mesh.add_polygon(&tt)?;

                let inner_btn_width = ps.width + ps.around_button_padding;
                let inner_btn_height = ps.height + ps.around_button_padding;

                let inner_lock_width = ps.lock_width + ps.lock_inner_padding;
                let inner_lock_height = ps.lock_height + ps.lock_inner_padding;

                #[rustfmt::skip]
                let bot_pl1 = [
                    self.pt(Vector3::new(inner_btn_width  / 2, inner_btn_height  / 2, -thickness / 2)),
                    self.pt(Vector3::new(inner_btn_width  / 2, -inner_btn_height / 2, -thickness / 2)),
                    self.pt(Vector3::new(inner_lock_width / 2, -inner_lock_height/ 2, -thickness / 2,)),
                    self.pt(Vector3::new(inner_lock_width / 2, inner_lock_height / 2, -thickness / 2,)),
                ];

                #[rustfmt::skip]
                let bot_pl2 = [
                    self.pt(Vector3::new(inner_btn_width / 2, -inner_btn_height / 2, -thickness / 2)),
                    self.pt(Vector3::new(-inner_btn_width / 2, -inner_btn_height / 2, -thickness / 2)),
                    self.pt(Vector3::new( -inner_lock_width / 2, -inner_lock_height / 2, -thickness / 2,)),
                    self.pt(Vector3::new( inner_lock_width / 2, -inner_lock_height / 2, -thickness / 2,)),
                ];

                #[rustfmt::skip]
                let bot_pl3 = [

                    self.pt(Vector3::new(-inner_btn_width / 2,  -inner_btn_height / 2, -thickness / 2)),
                    self.pt(Vector3::new(-inner_btn_width / 2,   inner_btn_height / 2, -thickness / 2)),
                    self.pt(Vector3::new(-inner_lock_width / 2,  inner_lock_height / 2, -thickness / 2,)),
                    self.pt(Vector3::new(-inner_lock_width / 2, -inner_lock_height / 2, -thickness / 2,)),

                ];

                #[rustfmt::skip]
                let bot_pl4 = [

                    self.pt(Vector3::new(-inner_btn_width / 2, inner_btn_height / 2, -thickness / 2)),
                    self.pt(Vector3::new(inner_btn_width / 2, inner_btn_height / 2, -thickness / 2)),
                    self.pt(Vector3::new( inner_lock_width / 2, inner_lock_height / 2, -thickness / 2,)),
                    self.pt(Vector3::new( -inner_lock_width / 2, inner_lock_height / 2, -thickness / 2,)),

                ];

                #[rustfmt::skip]
                let mut br = [
                    self.pt(Vector3::new( inner_lock_width / 2, inner_lock_height / 2, -thickness / 2,)),
                    self.pt(Vector3::new( inner_lock_width / 2, inner_lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new( inner_lock_width / 2, -inner_lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new( inner_lock_width / 2, -inner_lock_height / 2, -thickness / 2 )),
                ];
                br.reverse();
                #[rustfmt::skip]
                let bl = [
                    self.pt(Vector3::new(-inner_lock_width / 2, inner_lock_height / 2, -thickness / 2,)),
                    self.pt(Vector3::new(-inner_lock_width / 2, inner_lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new(-inner_lock_width / 2, -inner_lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new(-inner_lock_width / 2, -inner_lock_height / 2, -thickness / 2 )),
                ];
                #[rustfmt::skip]
                let mut bb = [
                    self.pt(Vector3::new(inner_lock_width / 2, -inner_lock_height / 2, -thickness / 2,)),
                    self.pt(Vector3::new(inner_lock_width / 2, -inner_lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new(-inner_lock_width / 2, -inner_lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new(-inner_lock_width / 2, -inner_lock_height / 2, -thickness / 2 )),
                ];
                bb.reverse();

                #[rustfmt::skip]
                let bt = [
                    self.pt(Vector3::new(inner_lock_width / 2, inner_lock_height / 2, -thickness / 2,)),
                    self.pt(Vector3::new(inner_lock_width / 2, inner_lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new(-inner_lock_width / 2, inner_lock_height / 2, thickness / 2 - ps.lock_depth,)),
                    self.pt(Vector3::new(-inner_lock_width / 2, inner_lock_height / 2, -thickness / 2 )),
                ];
                #[rustfmt::skip]
                let bot2_pl = [
                    self.pt(Vector3::new(inner_lock_width / 2, inner_lock_height / 2, thickness / 2 - ps.lock_depth)),
                    self.pt(Vector3::new(inner_lock_width / 2, -inner_lock_height / 2, thickness / 2 - ps.lock_depth)),
                    self.pt(Vector3::new(-inner_lock_width / 2, -inner_lock_height / 2, thickness / 2 - ps.lock_depth)),
                    self.pt(Vector3::new(-inner_lock_width / 2, inner_lock_height / 2, thickness / 2 - ps.lock_depth)),

                    self.pt(Vector3::new(inner_lock_width / 2, inner_lock_height / 2, thickness / 2 - ps.lock_depth)),

                    self.pt(Vector3::new(ps.lock_width / 2, ps.lock_height / 2, thickness / 2 - ps.lock_depth)),
                    self.pt(Vector3::new(-ps.lock_width / 2, ps.lock_height / 2, thickness / 2 - ps.lock_depth)),
                    self.pt(Vector3::new(-ps.lock_width / 2, -ps.lock_height / 2, thickness / 2 - ps.lock_depth)),
                    self.pt(Vector3::new(ps.lock_width / 2, -ps.lock_height / 2, thickness / 2 - ps.lock_depth)),
                    self.pt(Vector3::new(ps.lock_width / 2, ps.lock_height / 2, thickness / 2 - ps.lock_depth)),
                ];

                mesh.add_polygon(&bot_pl1)?;
                mesh.add_polygon(&bot_pl2)?;
                mesh.add_polygon(&bot_pl3)?;
                mesh.add_polygon(&bot_pl4)?;

                mesh.add_polygon(&bot2_pl)?;
                mesh.add_polygon(&br)?;
                mesh.add_polygon(&bl)?;
                mesh.add_polygon(&bb)?;
                mesh.add_polygon(&bt)?;
                Ok(mesh_id)
            }

            ButtonMountKind::ChokHotswapCustom => {
                let mount = ChokHotswap::new();
                println!("!!!");

                mount.outer_mount(self.origin.clone(), index)
                //Ok(index.new_mesh())
            }
            _ => todo!("Implement mesh for chok and cherry"),
        }
    }
}
