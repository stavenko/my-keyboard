use std::{
    collections::{HashMap, HashSet},
    rc::Rc,
};

use anyhow::anyhow;
use geometry::{
    decimal::Dec,
    geometry::{Geometry, GeometryDyn},
    hyper_path::{
        hyper_line::{HyperLine, ShiftInPlane},
        hyper_path::{HyperPath, Root},
        hyper_point::{SideDir, SuperPoint},
        hyper_surface::{
            dynamic_surface::DynamicSurface, polygon_from_line_in_plane::PolygonFromLineInPlane,
            primitive_dynamic_surface::PrimitiveSurface,
        },
    },
    indexes::geo_index::{
        geo_object::GeoObject,
        index::{GeoIndex, PolygonFilter},
        mesh::{MeshId, MeshRefMut},
    },
};
use itertools::Itertools;
use nalgebra::Vector3;
use rust_decimal_macros::dec;

use crate::{
    button_collections::ButtonsCollection, keyboard_builder::KeyboardBuilder,
    next_and_peek::NextAndPeekBlank,
};

#[derive(PartialEq, Eq, Hash, Clone, Copy)]
pub enum KeyboardMesh {
    ButtonsHull,
    Bottom,
    PcbMount,
}

#[derive(PartialEq, Eq, Clone, Copy)]
#[allow(unused)]
pub enum MaterialAddition {
    InnerSurface,
    OuterSurface,
    Both,
}

#[allow(clippy::type_complexity)]
pub struct RightKeyboardConfig {
    pub(crate) main_buttons: ButtonsCollection,
    pub(crate) thumb_buttons: ButtonsCollection,
    pub(crate) table_outline: Root<SuperPoint<Dec>>,
    pub(crate) main_plane_thickness: Dec,
    pub(crate) bottom_thickness: Dec,
    pub(crate) additional_material:
        HashMap<KeyboardMesh, Vec<(MaterialAddition, Rc<dyn GeometryDyn>)>>,

    pub(crate) holes: HashMap<KeyboardMesh, Vec<Rc<dyn GeometryDyn>>>,
}

impl RightKeyboardConfig {
    pub fn build() -> KeyboardBuilder {
        KeyboardBuilder::default()
    }

    fn right_line_inner(&self) -> impl Iterator<Item = SuperPoint<Dec>> + '_ {
        self.main_buttons
            .right_line_inner(self.main_plane_thickness)
    }

    fn right_line_outer(&self) -> impl Iterator<Item = SuperPoint<Dec>> + '_ {
        self.main_buttons
            .right_line_outer(self.main_plane_thickness)
    }

    fn top_thumb_main_connection_inner(&self) -> HyperLine<SuperPoint<Dec>> {
        let main_point_mutual = self
            .main_buttons
            .left_line_inner(self.main_plane_thickness)
            .last()
            .expect("all good");
        let main_point_side_dir = self
            .main_buttons
            .top_line_inner(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_mutual = self
            .thumb_buttons
            .right_line_inner(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_dir = self
            .thumb_buttons
            .top_line_inner(self.main_plane_thickness)
            .next_back()
            .expect("all good");

        let a = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point,
        };

        let b = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point + thumb_point_mutual.side_dir(),
        };

        let c = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point + main_point_mutual.side_dir(),
        };

        let d = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point,
        };

        HyperLine::new_4(a, b, c, d)
    }

    fn top_thumb_main_connection_outer(&self) -> HyperLine<SuperPoint<Dec>> {
        let main_point_mutual = self
            .main_buttons
            .left_line_outer(self.main_plane_thickness)
            .last()
            .expect("all good");
        let main_point_side_dir = self
            .main_buttons
            .top_line_outer(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_mutual = self
            .thumb_buttons
            .right_line_outer(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_dir = self
            .thumb_buttons
            .top_line_outer(self.main_plane_thickness)
            .next_back()
            .expect("all good");

        let a = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point,
        };

        let b = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point + thumb_point_mutual.side_dir(),
        };

        let c = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point + main_point_mutual.side_dir(),
        };

        let d = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point,
        };

        HyperLine::new_4(a, b, c, d)
    }

    fn bottom_main_thumb_connection_inner(&self) -> HyperLine<SuperPoint<Dec>> {
        let main_point_mutual = self
            .main_buttons
            .left_line_inner(self.main_plane_thickness)
            .next()
            .expect("all good");
        let main_point_side_dir = self
            .main_buttons
            .bottom_line_inner(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_mutual = self
            .thumb_buttons
            .right_line_inner(self.main_plane_thickness)
            .next_back()
            .expect("all good");
        let thumb_point_dir = self
            .thumb_buttons
            .bottom_line_inner(self.main_plane_thickness)
            .next_back()
            .expect("all good");

        let a = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point,
        };
        let b = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point + main_point_mutual.side_dir(),
        };

        let c = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point + thumb_point_mutual.side_dir(),
        };

        let d = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point,
        };

        HyperLine::new_4(a, b, c, d)
    }

    fn bottom_main_thumb_connection_outer(&self) -> HyperLine<SuperPoint<Dec>> {
        let main_point_mutual = self
            .main_buttons
            .left_line_outer(self.main_plane_thickness)
            .next()
            .expect("all good");
        let main_point_side_dir = self
            .main_buttons
            .bottom_line_outer(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_mutual = self
            .thumb_buttons
            .right_line_outer(self.main_plane_thickness)
            .next_back()
            .expect("all good");
        let thumb_point_dir = self
            .thumb_buttons
            .bottom_line_outer(self.main_plane_thickness)
            .next_back()
            .expect("all good");

        let a = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point,
        };
        let b = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point + main_point_mutual.side_dir(),
        };

        let c = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point + thumb_point_mutual.side_dir(),
        };

        let d = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point,
        };

        HyperLine::new_4(a, b, c, d)
    }

    fn line_1_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.right_line_inner()
            .chain(
                self.main_buttons
                    .bottom_line_inner(self.main_plane_thickness)
                    .rev(),
            )
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    fn line_1_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.right_line_outer()
            .chain(
                self.main_buttons
                    .bottom_line_outer(self.main_plane_thickness)
                    .rev(),
            )
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    fn line_2_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        vec![(self.bottom_main_thumb_connection_inner())].into_iter()
    }

    fn line_2_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        vec![(self.bottom_main_thumb_connection_outer())].into_iter()
    }

    fn line_3_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.thumb_buttons
            .bottom_line_inner(self.main_plane_thickness)
            .rev()
            .chain(
                self.thumb_buttons
                    .left_line_inner(self.main_plane_thickness),
            )
            .chain(self.thumb_buttons.top_line_inner(self.main_plane_thickness))
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    fn line_3_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.thumb_buttons
            .bottom_line_outer(self.main_plane_thickness)
            .rev()
            .chain(
                self.thumb_buttons
                    .left_line_outer(self.main_plane_thickness),
            )
            .chain(self.thumb_buttons.top_line_outer(self.main_plane_thickness))
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    fn line_4_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        vec![(self.top_thumb_main_connection_inner())].into_iter()
    }

    fn line_4_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        vec![(self.top_thumb_main_connection_outer())].into_iter()
    }

    fn line_5_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.main_buttons
            .top_line_inner(self.main_plane_thickness)
            .chain(
                self.main_buttons
                    .right_line_inner(self.main_plane_thickness)
                    .take(1),
            )
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    fn line_5_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.main_buttons
            .top_line_outer(self.main_plane_thickness)
            .chain(
                self.main_buttons
                    .right_line_outer(self.main_plane_thickness)
                    .take(1),
            )
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    fn line_around_buttons_inner(&self) -> Root<SuperPoint<Dec>> {
        self.line_1_inner()
            .chain(self.line_2_inner())
            .chain(self.line_3_inner())
            .chain(self.line_4_inner())
            .chain(self.line_5_inner())
            .fold(Root::new(), |hp, l| hp.push_back(l))
    }

    fn line_around_buttons_outer(&self) -> Root<SuperPoint<Dec>> {
        self.line_1_outer()
            .chain(self.line_2_outer())
            .chain(self.line_3_outer())
            .chain(self.line_4_outer())
            .chain(self.line_5_outer())
            .fold(Root::new(), |hp, l| hp.push_back(l))
    }

    pub(crate) fn inner_wall_surface(&self, mut mesh: MeshRefMut) -> anyhow::Result<()> {
        let outline = self.table_outline.clone();
        let around_buttons = self.line_around_buttons_inner();
        if outline.len() != around_buttons.len() {
            println!(
                "WARNING, OUTLINE AND BUTTONS HAVE DIFFERRENT SIZE {} <> {}",
                outline.len(),
                around_buttons.len()
            );
        }

        DynamicSurface::new(around_buttons, outline).polygonize(&mut mesh, 8)?;
        Ok(())
    }

    pub(crate) fn outer_wall_surface(&self, mut mesh: MeshRefMut) -> anyhow::Result<()> {
        let mut outline = self
            .table_outline
            .clone()
            .map(|l| l.shift_in_plane(Vector3::z(), -self.main_plane_thickness));
        outline.connect_ends_circular();
        let around_buttons = self.line_around_buttons_outer();
        if outline.len() != around_buttons.len() {
            println!(
                "WARNING, OUTLINE AND BUTTONS HAVE DIFFERRENT SIZE {} <> {}",
                outline.len(),
                around_buttons.len()
            );
        }

        DynamicSurface::new(outline, around_buttons).polygonize(&mut mesh, 8)?;
        Ok(())
    }

    pub(crate) fn fill_between_collections(&self, mesh: &mut MeshRefMut) -> anyhow::Result<()> {
        let right_line_inner = self
            .thumb_buttons
            .right_line_inner(self.main_plane_thickness)
            .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
            .fold(Root::new(), |hp, l| hp.push_back(l));

        let left_line_inner = self
            .main_buttons
            .left_line_inner(self.main_plane_thickness)
            .rev()
            .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
            .fold(Root::new(), |hp, l| hp.push_back(l));

        let right_line_outer = self
            .thumb_buttons
            .right_line_outer(self.main_plane_thickness)
            .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
            .fold(Root::new(), |hp, l| hp.push_back(l));

        let left_line_outer = self
            .main_buttons
            .left_line_outer(self.main_plane_thickness)
            .rev()
            .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
            .fold(Root::new(), |hp, l| hp.push_back(l));

        DynamicSurface::new(right_line_inner, left_line_inner).polygonize(mesh, 8)?;
        DynamicSurface::new(left_line_outer, right_line_outer).polygonize(mesh, 8)?;
        Ok(())
    }

    pub(crate) fn fill_between_buttons(&self, mut mesh: MeshRefMut) -> anyhow::Result<()> {
        self.main_buttons
            .fill_columns(&mut mesh, self.main_plane_thickness)?;
        self.thumb_buttons
            .fill_columns(&mut mesh, self.main_plane_thickness)?;
        self.main_buttons
            .fill_between_columns_inner(&mut mesh, self.main_plane_thickness)?;
        self.main_buttons
            .fill_between_columns_outer(&mut mesh, self.main_plane_thickness)?;
        self.thumb_buttons
            .fill_between_columns_inner(&mut mesh, self.main_plane_thickness)?;

        self.thumb_buttons
            .fill_between_columns_outer(&mut mesh, self.main_plane_thickness)?;

        self.fill_between_collections(&mut mesh)?;

        Ok(())
    }

    pub(crate) fn buttons(&self, index: &mut GeoIndex) -> anyhow::Result<Vec<MeshId>> {
        let meshes = self
            .main_buttons
            .buttons()
            .chain(self.thumb_buttons.buttons())
            .filter_map(|b| b.mesh(index, self.main_plane_thickness).ok())
            .collect();

        Ok(meshes)
    }

    pub(crate) fn inner_outer_surface_table_connection(
        &self,
        mut mesh: MeshRefMut,
    ) -> anyhow::Result<()> {
        let mut outline = self.table_outline.clone();
        let mut shifted_outline = outline
            .clone()
            .map(|l| l.shift_in_plane(Vector3::z(), -self.main_plane_thickness));
        shifted_outline.connect_ends_circular();

        loop {
            let (f, fs) = outline.head_tail();
            let (s, ss) = shifted_outline.head_tail();
            outline = fs;
            shifted_outline = ss;

            PrimitiveSurface(s.to_points(), f.to_points()).polygonize(&mut mesh, 8)?;
            if outline.len() == 0 {
                break;
            }
        }
        Ok(())
    }

    fn connect_two_lines(
        &self,
        mut mesh: MeshRefMut,
        mut line_one: Root<SuperPoint<Dec>>,
        mut line_two: Root<SuperPoint<Dec>>,
    ) -> anyhow::Result<()> {
        loop {
            let (f, fs) = line_one.head_tail();
            let (s, ss) = line_two.head_tail();
            line_one = fs;
            line_two = ss;

            PrimitiveSurface(s.to_points(), f.to_points()).polygonize(&mut mesh, 8)?;
            if line_one.len() == 0 {
                break;
            }
        }

        Ok(())
    }

    pub fn bottom_pad(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let plate_border = index.new_mesh();

        let mut inner_outline_upper = self
            .table_outline
            .clone()
            .map(|l| l.shift_in_plane(Vector3::z(), Dec::from(dec!(0.3))));

        inner_outline_upper.connect_ends_circular();

        let mut outer_outline_upper = self
            .table_outline
            .clone()
            .map(|l| l.shift_in_plane(Vector3::z(), -self.main_plane_thickness));

        outer_outline_upper.connect_ends_circular();

        let outer_outline_lower = outer_outline_upper.clone().map(|l| {
            l.map(|mut t| {
                t.point -= Vector3::z() * self.bottom_thickness;
                t
            })
        });

        self.connect_two_lines(
            plate_border.make_mut_ref(index),
            outer_outline_upper.clone(),
            inner_outline_upper.clone(),
        )?;
        self.connect_two_lines(
            plate_border.make_mut_ref(index),
            outer_outline_lower.clone(),
            outer_outline_upper,
        )?;

        let inner_inside_extension_line = inner_outline_upper.clone().map(|l| {
            l.map(|mut t| {
                t.point += Vector3::z() * Dec::from(dec!(1.0));
                t
            })
        });

        self.connect_two_lines(
            plate_border.make_mut_ref(index),
            inner_outline_upper,
            inner_inside_extension_line.clone(),
        )?;

        let bottom_plane = index.new_mesh();
        let poly = PolygonFromLineInPlane::new(outer_outline_lower, false);
        poly.polygonize(&mut bottom_plane.make_mut_ref(index), 8)?;

        let top_plane = index.new_mesh();
        let poly = PolygonFromLineInPlane::new(inner_inside_extension_line, true);
        poly.polygonize(&mut top_plane.make_mut_ref(index), 8)?;

        let inner_border = index.new_mesh();
        self.inner_wall_surface(inner_border.make_mut_ref(index))?;

        let additional_material =
            self.add_material(KeyboardMesh::Bottom, top_plane, bottom_plane, index)?;

        let to_delete = [
            index.select_polygons(inner_border, plate_border, PolygonFilter::Front),
            index.select_polygons(plate_border, inner_border, PolygonFilter::Back),
        ]
        .concat();

        let to_flip =
            [index.select_polygons(inner_border, plate_border, PolygonFilter::Back)].concat();

        for p in to_delete {
            p.make_mut_ref(index).remove();
        }

        for p in to_flip {
            p.make_mut_ref(index).flip();
        }

        index.move_all_polygons(bottom_plane, plate_border);
        index.move_all_polygons(top_plane, plate_border);
        index.move_all_polygons(inner_border, plate_border);
        for mat in additional_material {
            index.move_all_polygons(mat, plate_border);
        }

        self.apply_holes(KeyboardMesh::Bottom, plate_border, index)?;
        Ok(plate_border)
    }

    pub fn pcb_mount(&self, _index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        Err(anyhow!("not implemented"))
    }

    fn add_material(
        &self,
        to: KeyboardMesh,
        inner_wall_surface: MeshId,
        outer_wall_surface: MeshId,
        index: &mut GeoIndex,
    ) -> anyhow::Result<Vec<MeshId>> {
        let material_polygons = self
            .additional_material
            .iter()
            .flat_map(|mat| {
                if to == *mat.0 {
                    (*mat.1).clone()
                } else {
                    Vec::new()
                }
            })
            .filter_map(|(addition, material)| {
                let bolt_head_material = index.new_mesh();
                material
                    .polygonize(bolt_head_material.make_mut_ref(index), 0)
                    .ok()?;

                match addition {
                    MaterialAddition::InnerSurface => {
                        let bolt_head_below_outer = index
                            .select_polygons(
                                bolt_head_material,
                                outer_wall_surface,
                                PolygonFilter::Back,
                            )
                            .into_iter()
                            .collect::<HashSet<_>>();

                        let bolt_head_above_inner = index
                            .select_polygons(
                                bolt_head_material,
                                inner_wall_surface,
                                PolygonFilter::Back,
                            )
                            .into_iter()
                            .collect::<HashSet<_>>();

                        let between_walls = bolt_head_above_inner
                            .intersection(&bolt_head_below_outer)
                            .copied()
                            .collect_vec();

                        let inner_wall_surface_inside_material = index.select_polygons(
                            inner_wall_surface,
                            bolt_head_material,
                            PolygonFilter::Back,
                        );

                        let to_remove = [
                            inner_wall_surface_inside_material,
                            index.select_polygons(
                                bolt_head_material,
                                outer_wall_surface,
                                PolygonFilter::Front,
                            ),
                            between_walls,
                        ]
                        .concat();

                        for p in to_remove {
                            p.make_mut_ref(index).remove();
                        }

                        Some(bolt_head_material)
                    }
                    MaterialAddition::OuterSurface => todo!(),
                    MaterialAddition::Both => todo!(),
                }
            })
            .collect_vec();
        Ok(material_polygons)
    }

    fn apply_holes(
        &self,
        holes: KeyboardMesh,
        to_mesh: MeshId,
        index: &mut GeoIndex,
    ) -> anyhow::Result<()> {
        for hole in self.holes.get(&holes).into_iter().flatten() {
            let hole_mesh = index.new_mesh();
            hole.polygonize(hole_mesh.make_mut_ref(index), 0)?;

            let to_remove = [
                index.select_polygons(hole_mesh, to_mesh, PolygonFilter::Front),
                index.select_polygons(to_mesh, hole_mesh, PolygonFilter::Back),
            ]
            .concat();
            let to_flip = [index.select_polygons(hole_mesh, to_mesh, PolygonFilter::Back)].concat();
            for p in to_remove {
                p.make_mut_ref(index).remove();
            }

            for p in to_flip {
                p.make_mut_ref(index).flip();
            }
            index.move_all_polygons(hole_mesh, to_mesh);
        }
        Ok(())
    }

    pub fn buttons_hull(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let inner_wall_surface = index.new_mesh();
        let outer_wall_surface = index.new_mesh();
        let buttons = index.new_mesh();
        let buttons_filling = index.new_mesh();
        let table_bottom_surface = index.new_mesh();

        println!("inner");
        self.inner_wall_surface(inner_wall_surface.make_mut_ref(index))?;

        println!("outer");
        self.outer_wall_surface(outer_wall_surface.make_mut_ref(index))?;

        println!("buttons");
        for button_item in self.buttons(index)? {
            index.move_all_polygons(button_item, buttons);
        }

        println!("fill between");
        self.fill_between_buttons(buttons_filling.make_mut_ref(index))?;

        println!("fill between surfaces");
        self.inner_outer_surface_table_connection(table_bottom_surface.make_mut_ref(index))?;

        let hull = inner_wall_surface;

        println!("bolt fills");
        let addition_material_polygons =
            self.add_material(KeyboardMesh::ButtonsHull, hull, outer_wall_surface, index)?;
        index.move_all_polygons(outer_wall_surface, hull);
        index.move_all_polygons(buttons, hull);
        index.move_all_polygons(buttons_filling, hull);
        index.move_all_polygons(table_bottom_surface, hull);
        for mesh_id in addition_material_polygons {
            index.move_all_polygons(mesh_id, hull);
        }

        println!("bolt holes");
        self.apply_holes(KeyboardMesh::ButtonsHull, hull, index)?;
        Ok(hull)
    }
}
