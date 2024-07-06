use std::collections::HashSet;

use anyhow::anyhow;
use geometry::{
    decimal::Dec,
    geometry::Geometry,
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
        index::{self, GeoIndex, PolygonFilter},
        mesh::MeshId,
        poly::PolyId,
    },
};
use itertools::Itertools;
use nalgebra::Vector3;
use rust_decimal_macros::dec;

use crate::{
    bolt_point::BoltPoint, button_collections::ButtonsCollection,
    keyboard_builder::KeyboardBuilder, next_and_peek::NextAndPeekBlank,
};

pub struct RightKeyboardConfig {
    pub(crate) main_buttons: ButtonsCollection,
    pub(crate) thumb_buttons: ButtonsCollection,
    pub(crate) table_outline: Root<SuperPoint<Dec>>,
    pub(crate) main_plane_thickness: Dec,
    pub(crate) bottom_thickness: Dec,
    pub(crate) bolt_points: Vec<BoltPoint>,
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

    pub(crate) fn inner_wall_surface(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        let outline = self.table_outline.clone();
        let around_buttons = self.line_around_buttons_inner();
        if outline.len() != around_buttons.len() {
            println!(
                "WARNING, OUTLINE AND BUTTONS HAVE DIFFERRENT SIZE {} <> {}",
                outline.len(),
                around_buttons.len()
            );
        }

        DynamicSurface::new(around_buttons, outline).polygonize(index, 8)?;
        Ok(())
    }

    pub(crate) fn outer_wall_surface(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
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

        DynamicSurface::new(outline, around_buttons).polygonize(index, 8)?;
        Ok(())
    }

    pub(crate) fn fill_between_collections(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
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

        DynamicSurface::new(right_line_inner, left_line_inner).polygonize(index, 8)?;
        DynamicSurface::new(left_line_outer, right_line_outer).polygonize(index, 8)?;
        Ok(())
    }

    pub(crate) fn fill_between_buttons(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        self.main_buttons
            .fill_columns(index, self.main_plane_thickness)?;
        self.thumb_buttons
            .fill_columns(index, self.main_plane_thickness)?;
        self.main_buttons
            .fill_between_columns_inner(index, self.main_plane_thickness)?;
        self.main_buttons
            .fill_between_columns_outer(index, self.main_plane_thickness)?;
        self.thumb_buttons
            .fill_between_columns_inner(index, self.main_plane_thickness)?;

        self.thumb_buttons
            .fill_between_columns_outer(index, self.main_plane_thickness)?;

        self.fill_between_collections(index)?;

        Ok(())
    }

    pub(crate) fn buttons(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        for b in self
            .main_buttons
            .buttons()
            .chain(self.thumb_buttons.buttons())
        {
            b.mesh(index, self.main_plane_thickness)?;
        }

        Ok(())
    }

    pub(crate) fn inner_outer_surface_table_connection(
        &self,
        index: &mut GeoIndex,
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

            PrimitiveSurface(s.to_points(), f.to_points()).polygonize(index, 8)?;
            if outline.len() == 0 {
                break;
            }
        }
        Ok(())
    }

    fn connect_two_lines(
        &self,
        index: &mut GeoIndex,
        mut line_one: Root<SuperPoint<Dec>>,
        mut line_two: Root<SuperPoint<Dec>>,
    ) -> anyhow::Result<()> {
        loop {
            let (f, fs) = line_one.head_tail();
            let (s, ss) = line_two.head_tail();
            line_one = fs;
            line_two = ss;

            PrimitiveSurface(s.to_points(), f.to_points()).polygonize(index, 8)?;
            if line_one.len() == 0 {
                break;
            }
        }

        Ok(())
    }

    pub fn bottom_pad(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        index.create_new_mesh_and_set_as_default();
        let plate_border = index.get_current_default_mesh();

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
            index,
            outer_outline_upper.clone(),
            inner_outline_upper.clone(),
        )?;
        self.connect_two_lines(index, outer_outline_lower.clone(), outer_outline_upper)?;

        let inner_inside_extension_line = inner_outline_upper.clone().map(|l| {
            l.map(|mut t| {
                t.point += Vector3::z() * Dec::from(dec!(1.0));
                t
            })
        });

        self.connect_two_lines(
            index,
            inner_outline_upper,
            inner_inside_extension_line.clone(),
        )?;

        index.create_new_mesh_and_set_as_default();
        let poly = PolygonFromLineInPlane::new(outer_outline_lower, false);
        poly.polygonize(index, 8)?;

        let bottom_plane = index.get_current_default_mesh();

        index.create_new_mesh_and_set_as_default();
        let poly = PolygonFromLineInPlane::new(inner_inside_extension_line, true);
        poly.polygonize(index, 8)?;

        let top_plane = index.get_current_default_mesh();

        index.create_new_mesh_and_set_as_default();
        self.inner_wall_surface(index)?;
        let inner_border = index.get_current_default_mesh();

        let bolt_polygons: Vec<MeshId> = self
            .bolt_points
            .iter()
            .filter_map(|bolt_point| {
                index.create_new_mesh_and_set_as_default();
                let bolt_thread_material = bolt_point.get_bolt_thread_material(index).ok()?;

                index.create_new_mesh_and_set_as_default();
                let nut_well = bolt_point.get_bolt_nut_well(index).ok()?;

                index.create_new_mesh_and_set_as_default();
                let thread_hole = bolt_point.get_bolt_thread_hole(index).ok()?;

                let top_inside_mat =
                    index.select_polygons(top_plane, bolt_thread_material, PolygonFilter::Back);

                let nut_related_removals = if let Some(nut_well) = nut_well {
                    let top_outside_hole =
                        index.select_polygons(top_plane, nut_well, PolygonFilter::Front);

                    let top_plane_between_mat_and_nut = top_inside_mat
                        .into_iter()
                        .collect::<HashSet<_>>()
                        .intersection(&top_outside_hole.into_iter().collect::<HashSet<_>>())
                        .copied()
                        .collect_vec();

                    [
                        index.select_polygons(top_plane, nut_well, PolygonFilter::Back),
                        index.select_polygons(bottom_plane, nut_well, PolygonFilter::Back),
                        index.select_polygons(nut_well, bottom_plane, PolygonFilter::Front),
                        index.select_polygons(thread_hole, nut_well, PolygonFilter::Back),
                        index.select_polygons(nut_well, thread_hole, PolygonFilter::Back),
                        top_plane_between_mat_and_nut,
                    ]
                    .concat()
                } else {
                    Vec::new()
                };

                let to_remove = [
                    nut_related_removals,
                    index.select_polygons(bolt_thread_material, bottom_plane, PolygonFilter::Front),
                    index.select_polygons(top_plane, bolt_thread_material, PolygonFilter::Back),
                    index.select_polygons(thread_hole, bolt_thread_material, PolygonFilter::Front),
                    index.select_polygons(bolt_thread_material, thread_hole, PolygonFilter::Back),
                    index.select_polygons(top_plane, thread_hole, PolygonFilter::Back),
                    index.select_polygons(bottom_plane, thread_hole, PolygonFilter::Back),
                ]
                .concat();

                let to_flip = [
                    index.get_mesh_polygon_ids(thread_hole).collect_vec(),
                    nut_well
                        .into_iter()
                        .flat_map(|m| index.get_mesh_polygon_ids(m))
                        .collect_vec(),
                ]
                .concat();

                for p in to_remove {
                    index.remove_polygon(p);
                }
                for p in to_flip {
                    index.flip_polygon(p);
                }

                Some(
                    [
                        vec![bolt_thread_material],
                        vec![thread_hole],
                        nut_well.into_iter().collect(),
                    ]
                    .concat(),
                )
            })
            .flatten()
            .collect_vec();

        let to_delete = [
            index.select_polygons(inner_border, plate_border, PolygonFilter::Front),
            index.select_polygons(plate_border, inner_border, PolygonFilter::Back),
        ]
        .concat();

        let to_flip =
            [index.select_polygons(inner_border, plate_border, PolygonFilter::Back)].concat();

        for p in to_delete {
            index.remove_polygon(p);
        }

        for p in to_flip {
            index.flip_polygon(p);
        }

        index.move_all_polygons(bottom_plane, plate_border);
        index.move_all_polygons(top_plane, plate_border);
        index.move_all_polygons(inner_border, plate_border);
        for mesh_id in bolt_polygons {
            index.move_all_polygons(mesh_id, plate_border);
        }

        Ok(plate_border)
    }

    pub fn pcb_mount(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        Err(anyhow!("not implemented"))
    }

    fn usb_connection(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        Err(anyhow!("not implemented"))
    }

    pub fn buttons_hull(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        self.inner_wall_surface(index)?;
        let inner_wall_surface = index.get_current_default_mesh();

        index.create_new_mesh_and_set_as_default();
        self.outer_wall_surface(index)?;
        let outer_wall_surface = index.get_current_default_mesh();

        index.create_new_mesh_and_set_as_default();
        self.buttons(index)?;
        let buttons = index.get_current_default_mesh();

        index.create_new_mesh_and_set_as_default();
        self.fill_between_buttons(index)?;
        let buttons_filling = index.get_current_default_mesh();

        index.create_new_mesh_and_set_as_default();
        self.inner_outer_surface_table_connection(index)?;
        let table_bottom_surface = index.get_current_default_mesh();

        index.create_new_mesh_and_set_as_default();
        let usb_connection = self.usb_connection(index)?;

        let bolt_polygons = self
            .bolt_points
            .iter()
            .filter_map(|bolt_point| {
                index.create_new_mesh_and_set_as_default();
                let bolt_head_hole = bolt_point.get_bolt_head_hole_bounds(index).ok()?;
                println!("{bolt_head_hole:?}");

                index.create_new_mesh_and_set_as_default();
                let bolt_head_material = bolt_point.get_bolt_head_material_bounds(index).ok()?;

                index.create_new_mesh_and_set_as_default();
                let thread_hole = bolt_point.get_bolt_thread_head_hole_bounds(index).ok()?;

                let inner_wall_outside_hole =
                    index.select_polygons(inner_wall_surface, bolt_head_hole, PolygonFilter::Front);

                let inner_wall_inside_material = index.select_polygons(
                    inner_wall_surface,
                    bolt_head_material,
                    PolygonFilter::Back,
                );

                let inner_wall_between_material_and_hole = inner_wall_outside_hole
                    .into_iter()
                    .collect::<HashSet<_>>()
                    .intersection(
                        &inner_wall_inside_material
                            .into_iter()
                            .collect::<HashSet<_>>(),
                    )
                    .copied()
                    .collect_vec();

                /*
                let outer_wall_outside_hole =
                    index.select_polygons(outer_wall_surface, bolt_head_hole, PolygonFilter::Front);

                let outer_wall_inside_material = index.select_polygons(
                    outer_wall_surface,
                    bolt_head_material,
                    PolygonFilter::Back,
                );
                let outer_wall_between_material_and_hole = outer_wall_outside_hole
                    .into_iter()
                    .collect::<HashSet<_>>()
                    .intersection(
                        &outer_wall_inside_material
                            .into_iter()
                            .collect::<HashSet<_>>(),
                    )
                    .copied()
                    .collect_vec();

                */

                let to_remove = [
                    index.select_polygons(
                        bolt_head_material,
                        outer_wall_surface,
                        PolygonFilter::Front,
                    ),
                    index.select_polygons(bolt_head_hole, outer_wall_surface, PolygonFilter::Front),
                    index.select_polygons(outer_wall_surface, bolt_head_hole, PolygonFilter::Back),
                    index.select_polygons(inner_wall_surface, bolt_head_hole, PolygonFilter::Back),
                    index.select_polygons(bolt_head_material, thread_hole, PolygonFilter::Back),
                    index.select_polygons(bolt_head_hole, thread_hole, PolygonFilter::Back),
                    index.select_polygons(thread_hole, bolt_head_material, PolygonFilter::Front),
                    index.select_polygons(thread_hole, bolt_head_hole, PolygonFilter::Back),
                    index.select_polygons(inner_wall_surface, thread_hole, PolygonFilter::Back),
                    inner_wall_between_material_and_hole,
                    // outer_wall_between_material_and_hole,
                ]
                .concat();

                let to_flip = [
                    //index.get_polygon_with_root_parent(PolyId(2639)),
                    //index.get_polygon_with_root_parent(PolyId(1677)),
                    //index.select_polygons(bolt_head_hole, outer_wall_surface, PolygonFilter::Back),
                    //index.select_polygons(inner_wall_surface, bolt_head_hole, PolygonFilter::Back),
                    index.get_mesh_polygon_ids(thread_hole).collect_vec(),
                    index.get_mesh_polygon_ids(bolt_head_hole).collect_vec(),
                ]
                .concat()
                .into_iter()
                .sorted()
                .dedup();

                for p in to_remove {
                    index.remove_polygon(p);
                }
                for p in to_flip {
                    index.flip_polygon(p);
                }

                Some([bolt_head_material, bolt_head_hole, thread_hole])
                //Some([])
            })
            .flatten()
            .collect_vec();

        index.move_all_polygons(outer_wall_surface, inner_wall_surface);
        index.move_all_polygons(buttons, inner_wall_surface);
        index.move_all_polygons(buttons_filling, inner_wall_surface);
        index.move_all_polygons(table_bottom_surface, inner_wall_surface);
        for mesh_id in bolt_polygons {
            index.move_all_polygons(mesh_id, inner_wall_surface);
        }

        Ok(inner_wall_surface)
    }
}
