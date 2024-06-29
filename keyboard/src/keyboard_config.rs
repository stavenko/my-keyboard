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
            dynamic_surface::DynamicSurface, primitive_dynamic_surface::PrimitiveSurface,
        },
    },
    indexes::geo_index::{
        index::{GeoIndex, PolygonFilter},
        mesh::MeshId,
        poly::PolyId,
    },
};
use itertools::Itertools;
use nalgebra::Vector3;

use crate::{
    bolt_point::BoltPoint, button, button_collections::ButtonsCollection,
    keyboard_builder::KeyboardBuilder, next_and_peek::NextAndPeekBlank,
};

pub struct RightKeyboardConfig {
    pub(crate) main_buttons: ButtonsCollection,
    pub(crate) thumb_buttons: ButtonsCollection,
    pub(crate) table_outline: Root<SuperPoint<Dec>>,
    pub(crate) main_plane_thickness: Dec,
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

    fn left_line_inner(&self) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.thumb_buttons
            .left_line_inner(self.main_plane_thickness)
    }

    fn top_line_inner(&self) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.thumb_buttons
            .top_line_inner(self.main_plane_thickness)
            .chain(self.main_buttons.top_line_inner(self.main_plane_thickness))
    }

    fn bottom_line_inner(&self) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.thumb_buttons
            .bottom_line_inner(self.main_plane_thickness)
            .chain(
                self.main_buttons
                    .bottom_line_inner(self.main_plane_thickness),
            )
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

    pub fn bottom_pad(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        println!("inner");
        index.create_new_mesh_and_set_as_default();
        self.inner_wall_surface(index)?;
        let outer_wall_surface = index.get_current_default_mesh();
        // 0. we can use separate index...
        // take inner surface.
        // map its-points on some smaller value "make it smaller by K"
        // cut it by two planes.
        // take planes and surface as new mesh

        // or ...
        // take outline, shrink it.
        // create inner polygon,
        // copy outline, raise it on z, and shrink it even more
        // create side polygons and top polygon
        // and then cut excess...

        Err(anyhow!("not implemented"))
    }

    pub fn pcb_mount(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        Err(anyhow!("not implemented"))
    }

    pub fn usb_connection(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        Err(anyhow!("not implemented"))
    }

    pub fn buttons_hull(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        println!("inner");
        self.inner_wall_surface(index)?;
        let inner_wall_surface = index.get_current_default_mesh();
        println!("in wall surface: {inner_wall_surface:?}");

        println!("outer");
        index.create_new_mesh_and_set_as_default();
        self.outer_wall_surface(index)?;
        let outer_wall_surface = index.get_current_default_mesh();
        println!("out wall surface: {outer_wall_surface:?}");

        println!("buttons");
        index.create_new_mesh_and_set_as_default();
        self.buttons(index)?;
        let buttons = index.get_current_default_mesh();

        println!("filling");
        index.create_new_mesh_and_set_as_default();
        self.fill_between_buttons(index)?;
        let buttons_filling = index.get_current_default_mesh();

        println!("table_bottom_surface");
        index.create_new_mesh_and_set_as_default();
        self.inner_outer_surface_table_connection(index)?;
        let table_bottom_surface = index.get_current_default_mesh();

        let bolt_polygons = self
            .bolt_points
            .iter()
            .filter_map(|bolt_point| {
                index.create_new_mesh_and_set_as_default();
                let bolt_head_hole = bolt_point.get_bolt_head_hole_bounds(index).ok()?;
                println!("bolt head hole mesh {:?}", bolt_head_hole);

                index.create_new_mesh_and_set_as_default();
                let bolt_head_material = bolt_point.get_bolt_head_material_bounds(index).ok()?;
                println!("bolt head material mesh {:?}", bolt_head_material);

                index.create_new_mesh_and_set_as_default();
                let thread_hole = bolt_point.get_bolt_thread_head_hole_bounds(index).ok()?;
                println!("thread hole mesh {:?}", thread_hole);

                let inner_wall_cut_fh =
                    index.select_polygons(inner_wall_surface, bolt_head_hole, PolygonFilter::Front);

                let inner_wall_cut_bm = index.select_polygons(
                    inner_wall_surface,
                    bolt_head_material,
                    PolygonFilter::Back,
                );
                let more = inner_wall_cut_fh
                    .into_iter()
                    .collect::<HashSet<_>>()
                    .intersection(&inner_wall_cut_bm.into_iter().collect::<HashSet<_>>())
                    .copied()
                    .collect_vec();
                let __upper_betweens = {
                    let inner_wall_cut_fh = index.select_polygons(
                        outer_wall_surface,
                        bolt_head_hole,
                        PolygonFilter::Front,
                    );

                    let inner_wall_cut_bm = index.select_polygons(
                        outer_wall_surface,
                        bolt_head_material,
                        PolygonFilter::Back,
                    );

                    inner_wall_cut_fh
                        .into_iter()
                        .collect::<HashSet<_>>()
                        .intersection(&inner_wall_cut_bm.into_iter().collect::<HashSet<_>>())
                        .copied()
                        .collect_vec()
                };
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
                    more,
                ]
                .concat();
                let to_flip = [
                    index.select_polygons(bolt_head_hole, outer_wall_surface, PolygonFilter::Back),
                    index.select_polygons(bolt_head_hole, inner_wall_surface, PolygonFilter::Front),
                    index.get_mesh_polygon_ids(thread_hole).collect_vec(),
                ]
                .concat();

                for p in to_remove {
                    index.remove_polygon(p);
                }
                for p in to_flip {
                    index.flip_polygon(p);
                }

                Some([bolt_head_material, bolt_head_hole, thread_hole])
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

    pub fn base_mesh(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        self.outline_base_mesh(index)?;

        let mesh_id = index.get_current_default_mesh();

        for bolt_point in &self.bolt_points {}
        Ok(mesh_id)
    }

    fn outline_base_mesh(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        todo!()
    }
}
