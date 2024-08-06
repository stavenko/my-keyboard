use std::{collections::HashSet, primitive};

use geometry::{
    decimal::Dec,
    geometry::GeometryDyn,
    hyper_path::length,
    indexes::geo_index::{
        index::{self, GeoIndex, PolygonFilter},
        mesh::MeshId,
        poly::PolyId,
    },
    origin::Origin,
    shapes::{Align, Cylinder, Rect},
};
use itertools::Itertools;
use num_traits::{One, Zero};
use rust_decimal_macros::dec;

use crate::{button::ButtonMount, ButtonMountKind};

pub struct ChokHotswap {
    depth: Dec,
    pcb_thickness: Dec,
    main_hole_radius: Dec,
    side_hole_radius: Dec,
    side_hole_distance: Dec,
    near_pin_distance: [Dec; 2],
    far_pin_distance: [Dec; 2],
    pcb_pin_radius: Dec,
    lock_to_bed_distance: Dec,
    total_switch_below_surface: Dec,
    mount_width: Dec,
    mount_height: Dec,
    mount_lock_width: Dec,
    mount_lock_height: Dec,
    mount_lock_depth: Dec,
    hotswap_thickness: Dec,
    bottom_mesh_button_holes_depth: Dec,
    hotswap_rect_height: Dec,
    hotswap_rect_width: Dec,
    hotswap_chamfer_radius: Dec,
}

impl ChokHotswap {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            mount_width: Dec::from(16),
            mount_height: Dec::from(16),
            mount_lock_width: Dec::from(13.8),
            mount_lock_height: Dec::from(13.8),
            mount_lock_depth: Dec::from(dec!(1.2)),

            depth: Dec::zero(),
            pcb_thickness: dec!(1.5).into(),
            main_hole_radius: dec!(1.7).into(),
            side_hole_radius: dec!(0.95).into(),
            side_hole_distance: dec!(5.5).into(),
            near_pin_distance: [-Dec::from(5), dec!(-3.8).into()],
            far_pin_distance: [Dec::zero(), dec!(-5.9).into()],
            pcb_pin_radius: 1.5.into(),
            lock_to_bed_distance: 1.3.into(),
            total_switch_below_surface: 2.2.into(),
            hotswap_thickness: dec!(2).into(),
            bottom_mesh_button_holes_depth: dec!(0.7).into(),
            hotswap_rect_height: dec!(4.6).into(),
            hotswap_rect_width: dec!(5).into(),
            hotswap_chamfer_radius: dec!(1).into(),
        }
    }

    fn lock(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        //let mount = ButtonMountKind::Chok.params();
        let zero = Origin::new();
        let lock = index.create_new_mesh_and_set_as_default();

        let rect = Rect::with_bottom_at(
            zero.clone().offset_z(self.lock_to_bed_distance),
            self.mount_width,
            self.mount_height,
            self.mount_lock_depth,
        );

        for p in rect.render() {
            index.save_as_polygon(&p, None)?;
        }

        let lock_min = Rect::with_bottom_at(
            zero.clone(),
            self.mount_lock_width,
            self.mount_lock_height,
            self.mount_lock_depth + self.lock_to_bed_distance + Dec::one(),
        );

        let holes = vec![lock_min]
            .into_iter()
            .map(|h| -> Box<dyn GeometryDyn> { Box::new(h) })
            .collect_vec();

        self.apply_holes(&holes.iter().map(|b| b.as_ref()).collect_vec(), lock, index)?;

        Ok(lock)
    }

    fn glue(
        &self,
        glue: &dyn GeometryDyn,
        one_mesh: MeshId,
        other_mesh: MeshId,
        index: &mut GeoIndex,
    ) -> anyhow::Result<MeshId> {
        let glue_material = index.create_new_mesh_and_set_as_default();
        glue.polygonize(index, 0)?;

        let to_delete = [
            index.select_polygons(glue_material, one_mesh, PolygonFilter::Back),
            index.select_polygons(glue_material, other_mesh, PolygonFilter::Back),
            index.select_polygons(one_mesh, glue_material, PolygonFilter::Back),
            index.select_polygons(other_mesh, glue_material, PolygonFilter::Back),
        ]
        .concat();

        for p in to_delete {
            index.remove_polygon(p);
        }

        index.move_all_polygons(other_mesh, one_mesh);
        index.move_all_polygons(glue_material, one_mesh);

        Ok(one_mesh)
    }

    fn add_material(
        &self,
        glue: &dyn GeometryDyn,
        one_mesh: MeshId,
        index: &mut GeoIndex,
    ) -> anyhow::Result<()> {
        let glue_material = index.create_new_mesh_and_set_as_default();
        glue.polygonize(index, 0)?;
        println!("Glue: {glue_material:?}");

        let to_delete = [
            index.select_polygons(glue_material, one_mesh, PolygonFilter::Back),
            index.select_polygons(one_mesh, glue_material, PolygonFilter::Back),
        ]
        .concat();

        for p in to_delete {
            index.remove_polygon(p);
            println!("Glue removing: {glue_material:?} >> {p:?}");
        }

        index.move_all_polygons(glue_material, one_mesh);

        Ok(())
    }

    fn glue_lock_to_bed(
        &self,
        lock: MeshId,
        bed: MeshId,
        index: &mut GeoIndex,
    ) -> anyhow::Result<MeshId> {
        let zero = Origin::new();
        let glue_size_x = self.mount_width;
        let glue_size_y = (self.mount_width - self.mount_lock_width) / 2;
        let glue_size_z = self.total_switch_below_surface + self.pcb_thickness;
        let single = self.glue(
            &Rect::with_bottom_at(
                zero.offset_z(-self.pcb_thickness)
                    .offset_y(self.mount_lock_height / 2)
                    .offset_y(glue_size_y / 2),
                glue_size_x,
                glue_size_y,
                glue_size_z,
            ),
            lock,
            bed,
            index,
        )?;

        Ok(single)
    }

    fn hotswap_hole(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let zero = Origin::new().offset_z(-self.pcb_thickness);
        let mesh = index.create_new_mesh_and_set_as_default();

        let pin1_origin = zero
            .clone()
            .offset_x(self.near_pin_distance[0])
            .offset_z(Dec::from(dec!(0.1)))
            .offset_y(self.near_pin_distance[1]);

        let pin1 = Rect::build()
            .height(self.hotswap_rect_height)
            .width(self.hotswap_rect_width + Dec::from(dec!(0.4)))
            .depth(self.hotswap_thickness + Dec::from(dec!(0.1)))
            .origin(pin1_origin)
            .align_z(Align::Pos)
            .build();

        let pin2_origin = zero
            .clone()
            .offset_x(self.far_pin_distance[0])
            .offset_z(Dec::from(dec!(0.1)))
            .offset_y(self.far_pin_distance[1]);

        let pin2 = Rect::build()
            .height(self.hotswap_rect_height)
            .width(self.hotswap_rect_width)
            .depth(self.hotswap_thickness + Dec::from(dec!(0.1)))
            .origin(pin2_origin)
            .align_z(Align::Pos)
            .build();

        for p in pin1.render() {
            index.save_as_polygon(&p, None)?;
        }

        self.add_material(&pin2, mesh, index)?;

        let mat_up_pos = zero
            .clone()
            .offset_z(Dec::from(dec!(0.1)))
            .offset_x(self.hotswap_rect_width / 2)
            .offset_x(Dec::from(2) / 10)
            .offset_x(self.near_pin_distance[0])
            .offset_y(self.far_pin_distance[1])
            .offset_y(self.hotswap_rect_height / 2);

        let mat_up = Rect::build()
            .height(self.hotswap_chamfer_radius)
            .width(self.hotswap_chamfer_radius)
            .depth(self.hotswap_thickness + Dec::from(dec!(0.1)))
            .origin(mat_up_pos)
            .align_z(Align::Pos)
            .align_x(Align::Neg)
            .align_y(Align::Neg)
            .build();

        println!("src: {mesh:?}, ");
        self.add_material(&mat_up, mesh, index)?;

        Ok(mesh)
    }

    #[allow(clippy::vec_init_then_push)]
    pub fn bottom_mesh(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        /*
        let zero = Origin::new().offset_z(-self.pcb_thickness);
        let bed_point = zero
            .clone()
            .offset_x(-self.mount_width / 2)
            .offset_y(-self.mount_height / 2);

        let material = Rect::build()
            .origin(bed_point)
            .width(self.mount_width / 2 + self.main_hole_radius * 3 / 2)
            .height(self.mount_height * 3 / 4)
            .depth(self.hotswap_thickness + Dec::one())
            .align_z(Align::Pos)
            .align_y(Align::Neg)
            .align_x(Align::Neg)
            .build();

        let hotswap_bottom = index.create_new_mesh_and_set_as_default();

        for p in material.render() {
            index.save_as_polygon(&p, None)?;
        }

        let main_hole = Cylinder::with_bottom_at(
            zero.clone().offset_z(-self.bottom_mesh_button_holes_depth),
            self.pcb_thickness + Dec::one(),
            self.main_hole_radius,
        )
        .top_cap(false)
        .steps(32);

        let left = Cylinder::with_bottom_at(
            zero.clone()
                .offset_z(-self.bottom_mesh_button_holes_depth)
                .offset_x(-self.side_hole_distance),
            self.pcb_thickness + Dec::one(),
            self.side_hole_radius,
        )
        .steps(32)
        .top_cap(false);

        let mut holes: Vec<Box<dyn GeometryDyn>> = Vec::new();
        holes.push(Box::new(main_hole));
        holes.push(Box::new(left));

        self.apply_holes(
            &holes.iter().map(|b| b.as_ref()).collect_vec(),
            hotswap_bottom,
            index,
        )?;
        */

        let hw_hole = self.hotswap_hole(index)?;

        for p in [28, 32, 10]
            .into_iter()
            .map(PolyId)
            .flat_map(|p| index.get_polygon_with_root_parent(p))
            .collect_vec()
        {
            println!("FLIP {p:?}");
            index.flip_polygon(p);
        }

        for p in [20, 19]
            .into_iter()
            .map(PolyId)
            .flat_map(|p| index.get_polygon_with_root_parent(p))
            .collect_vec()
        {
            println!("REMOVE FOR CLEAR VIEW {p:?}");
            index.remove_polygon(p);
        }

        Ok(())
    }

    pub fn top_mesh(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        let lock = self.lock(index)?;
        let bed = self.bed(index)?;

        let part = self.glue_lock_to_bed(lock, bed, index)?;
        let zero = Origin::new();
        let glue_size_x = self.mount_width / 3;
        let glue_size_y = (self.mount_width - self.mount_lock_width) / 2;
        let glue_size_z = self.total_switch_below_surface + self.pcb_thickness;

        self.add_material(
            &Rect::with_bottom_at(
                zero.clone()
                    .offset_z(-self.pcb_thickness)
                    .offset_x(self.mount_width / 3)
                    .offset_y(-self.mount_lock_height / 2)
                    .offset_y(-glue_size_y / 2),
                glue_size_x,
                glue_size_y,
                glue_size_z,
            ),
            part,
            index,
        )?;

        self.add_material(
            &Rect::with_bottom_at(
                zero.clone()
                    .offset_z(-self.pcb_thickness)
                    .offset_x(-self.mount_width / 3)
                    .offset_y(-self.mount_lock_height / 2)
                    .offset_y(-glue_size_y / 2),
                glue_size_x,
                glue_size_y,
                glue_size_z,
            ),
            part,
            index,
        )?;

        let glue_size_y = Dec::from(dec!(2.5));
        let glue_size_x = (self.mount_width - self.mount_lock_width) / 2;

        self.add_material(
            &Rect::with_bottom_at(
                zero.clone()
                    .offset_z(-self.pcb_thickness)
                    .offset_x(-self.mount_lock_width / 2)
                    .offset_x(-glue_size_x / 2),
                glue_size_x,
                glue_size_y,
                glue_size_z,
            ),
            part,
            index,
        )?;
        self.add_material(
            &Rect::with_bottom_at(
                zero.offset_z(-self.pcb_thickness)
                    .offset_x(self.mount_lock_width / 2)
                    .offset_x(glue_size_x / 2),
                glue_size_x,
                glue_size_y,
                glue_size_z,
            ),
            part,
            index,
        )?;

        /*
        for p in [/*987 ,*/ 993]
            .into_iter()
            .flat_map(|p| index.get_polygon_with_root_parent(PolyId(p)))
            .collect::<HashSet<_>>()
        {
            index.flip_polygon(p);
        }
            */
        Ok(())
    }

    fn bed(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let zero = Origin::new();

        let rect = Rect::with_top_at(
            zero.clone(),
            self.mount_lock_width + Dec::from(dec!(0.7)),
            self.mount_height,
            self.pcb_thickness,
        );
        for p in rect.render() {
            index.save_as_polygon(&p, None)?;
        }
        let main_mesh = index.get_current_default_mesh();

        let main_hole = Cylinder::with_top_at(
            zero.clone().offset_z(dec!(0.5)),
            self.pcb_thickness + Dec::one(),
            self.main_hole_radius,
        )
        .top_cap(false)
        .bottom_cap(false)
        .steps(32);

        let right = Cylinder::with_top_at(
            zero.clone()
                .offset_z(dec!(0.5))
                .offset_x(self.side_hole_distance),
            self.pcb_thickness + Dec::one(),
            self.side_hole_radius,
        )
        .steps(32)
        .top_cap(false)
        .bottom_cap(false);

        let left = Cylinder::with_top_at(
            zero.clone()
                .offset_z(dec!(0.5))
                .offset_x(-self.side_hole_distance),
            self.pcb_thickness + Dec::one(),
            self.side_hole_radius,
        )
        .steps(32)
        .top_cap(false)
        .bottom_cap(false);

        let pin1 = Cylinder::with_top_at(
            zero.clone()
                .offset_z(dec!(0.5))
                .offset_x(self.near_pin_distance[0])
                .offset_y(self.near_pin_distance[1]),
            self.pcb_thickness + Dec::one(),
            self.pcb_pin_radius,
        )
        .steps(32)
        .top_cap(false)
        .bottom_cap(false);

        let pin2 = Cylinder::with_top_at(
            zero.clone()
                .offset_z(dec!(0.5))
                .offset_x(self.far_pin_distance[0])
                .offset_y(self.far_pin_distance[1]),
            self.pcb_thickness + Dec::one(),
            self.pcb_pin_radius,
        )
        .steps(32)
        .top_cap(false)
        .bottom_cap(false);

        let holes = vec![main_hole, left, right, pin1, pin2]
            .into_iter()
            .map(|h| -> Box<dyn GeometryDyn> { Box::new(h) })
            .collect_vec();

        self.apply_holes(
            &holes.iter().map(|b| b.as_ref()).collect_vec(),
            main_mesh,
            index,
        )?;

        Ok(main_mesh)
    }

    fn apply_holes(
        &self,
        holes: &[&dyn GeometryDyn],
        in_mesh: MeshId,
        index: &mut GeoIndex,
    ) -> anyhow::Result<()> {
        for hole in holes {
            let new_hole_id = index.create_new_mesh_and_set_as_default();
            println!("hole {new_hole_id:?} ");
            hole.polygonize(index, 0)?;

            println!("not getting here");
            let to_delete = [
                index.select_polygons(new_hole_id, in_mesh, PolygonFilter::Front),
                index.select_polygons(in_mesh, new_hole_id, PolygonFilter::Back),
            ]
            .concat();
            for p in to_delete {
                index.remove_polygon(p);
            }

            for p in index
                .get_mesh_polygon_ids(new_hole_id)
                .collect::<HashSet<_>>()
            {
                index.flip_polygon(p);
            }
            index.move_all_polygons(new_hole_id, in_mesh);
        }
        Ok(())
    }
}
