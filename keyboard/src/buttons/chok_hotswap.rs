use nalgebra::{ComplexField, Vector, Vector3};
use std::{collections::HashSet, primitive};

use geometry::{
    decimal::Dec,
    geometry::GeometryDyn,
    hyper_path::length,
    indexes::geo_index::{
        face::FaceId,
        geo_object::{GeoObject, UnRef},
        index::{self, GeoIndex, PolygonFilter},
        mesh::{self, MeshId},
        poly::{PolyId, UnrefPoly},
    },
    origin::Origin,
    shapes::{Align, Cylinder, Plane, Rect},
};
use itertools::Itertools;
use num_traits::{One, Zero};
use rust_decimal_macros::dec;

pub struct ChokHotswap {
    depth: Dec,
    pcb_thickness: Dec,
    main_hole_radius: Dec,
    side_hole_radius: Dec,
    side_hole_distance: Dec,
    near_pin_distance: [Dec; 2],
    far_pin_distance: [Dec; 2],
    pcb_pin_diameter: Dec,
    lock_to_bed_distance: Dec,
    total_switch_below_surface: Dec,
    mount_width: Dec,
    mount_height: Dec,
    outer_mount_width: Dec,
    outer_mount_height: Dec,
    mount_lock_width: Dec,
    mount_lock_height: Dec,
    mount_lock_depth: Dec,
    hotswap_thickness: Dec,
    bottom_mesh_button_holes_depth: Dec,
    hotswap_rect_height: Dec,
    hotswap_rect_width_far: Dec,
    hotswap_rect_width_near: Dec,
    hotswap_chamfer_radius: Dec,
    bottom_mesh_screw_head_diameter: Dec,
    bottom_mesh_screw_thread_diameter: Dec,
    hs_ear_width: Dec,
    hs_ear_depth: Dec,
}

impl ChokHotswap {
    pub fn height(&self) -> Dec {
        self.outer_mount_height
    }
    pub fn width(&self) -> Dec {
        self.outer_mount_width
    }
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            mount_width: Dec::from(16),
            mount_height: Dec::from(16),
            outer_mount_width: Dec::from(20.4),
            outer_mount_height: Dec::from(20.4),
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
            pcb_pin_diameter: 3.2.into(),
            lock_to_bed_distance: 1.3.into(),
            total_switch_below_surface: 2.2.into(),
            hotswap_thickness: dec!(1.8).into(),
            bottom_mesh_button_holes_depth: dec!(0.7).into(),
            hotswap_rect_height: dec!(4.9).into(),
            hotswap_rect_width_near: dec!(5.1).into(),
            hotswap_rect_width_far: dec!(5.1).into(),
            hotswap_chamfer_radius: dec!(1.5).into(),
            bottom_mesh_screw_head_diameter: dec!(1.7).into(),
            bottom_mesh_screw_thread_diameter: Dec::from(dec!(1)),
            hs_ear_width: dec!(2.2).into(),
            hs_ear_depth: dec!(1.7).into(),
        }
    }

    fn lock(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let zero = Origin::new();
        let lock = index.new_mesh();

        let rect = Rect::with_bottom_at(
            zero.clone().offset_z(self.lock_to_bed_distance),
            self.mount_width,
            self.mount_height,
            self.mount_lock_depth,
        );

        for p in rect.render() {
            lock.make_mut_ref(index).add_polygon(&p)?;
        }

        let lock_min = Rect::with_bottom_at(
            zero.clone(),
            self.mount_lock_width,
            self.mount_lock_height,
            self.mount_lock_depth + self.lock_to_bed_distance + Dec::one(),
        );

        self.add_hole(&lock_min, lock, index)?;

        Ok(lock)
    }

    fn holes_deletions(
        &self,
        mesh_src: MeshId,
        mesh_tool: MeshId,
        index: &GeoIndex,
    ) -> Vec<UnrefPoly> {
        let shared_src = mesh_src
            .make_ref(index)
            .shared_with(mesh_tool.make_ref(index));
        let shared_tool = mesh_tool
            .make_ref(index)
            .shared_with(mesh_src.make_ref(index));

        [
            index.select_polygons(mesh_tool, mesh_src, PolygonFilter::Front),
            index.select_polygons(mesh_src, mesh_tool, PolygonFilter::Back),
            shared_tool
                .into_iter()
                .flat_map(|hole| {
                    if let Some(other) = shared_src.iter().find(|other_poly| {
                        other_poly.make_ref(index).face_id() == hole.make_ref(index).face_id()
                    }) {
                        vec![hole, *other]
                    } else {
                        vec![hole]
                    }
                })
                .collect_vec(),
        ]
        .concat()
    }

    fn addition_deletions(
        &self,
        mesh_one: MeshId,
        mesh_two: MeshId,
        index: &GeoIndex,
    ) -> Vec<UnrefPoly> {
        let shared_one = mesh_one
            .make_ref(index)
            .shared_with(mesh_two.make_ref(index));
        let shared_two = mesh_two
            .make_ref(index)
            .shared_with(mesh_one.make_ref(index));

        [
            index.select_polygons(mesh_one, mesh_two, PolygonFilter::Back),
            index.select_polygons(mesh_two, mesh_one, PolygonFilter::Back),
            shared_two
                .into_iter()
                .flat_map(|glue_poly| {
                    if let Some(other) = shared_one
                        .iter()
                        .find(|other_poly| {
                            other_poly.make_ref(index).face_id()
                                == glue_poly.make_ref(index).face_id()
                        })
                        .and_then(|other| {
                            if other.make_ref(index).dir() != glue_poly.make_ref(index).dir() {
                                Some(other)
                            } else {
                                None
                            }
                        })
                    {
                        vec![glue_poly, *other]
                    } else {
                        vec![glue_poly]
                    }
                })
                .collect(),
        ]
        .concat()
    }

    fn glue(
        &self,
        glue: &dyn GeometryDyn,
        one_mesh: MeshId,
        other_mesh: MeshId,
        index: &mut GeoIndex,
    ) -> anyhow::Result<MeshId> {
        let glue_material = index.new_mesh();
        glue.polygonize(glue_material.make_mut_ref(index), 0)?;

        let to_delete = [
            self.addition_deletions(one_mesh, glue_material, index),
            self.addition_deletions(other_mesh, glue_material, index),
        ]
        .concat();

        for p in to_delete {
            p.make_mut_ref(index).remove();
        }

        index.move_all_polygons(other_mesh, one_mesh);
        index.move_all_polygons(glue_material, one_mesh);

        Ok(one_mesh)
    }

    fn add_material(
        &self,
        glue: &dyn GeometryDyn,
        to_mesh: MeshId,
        index: &mut GeoIndex,
    ) -> anyhow::Result<()> {
        let glue_mesh = index.new_mesh();
        println!("glue: {glue_mesh:?}->{to_mesh:?}");
        glue.polygonize(glue_mesh.make_mut_ref(index), 0)?;

        let to_delete = self.addition_deletions(to_mesh, glue_mesh, index);

        for p in to_delete {
            if glue_mesh == 9
                && to_mesh == 7
                && ([355, 356].contains(&p.make_ref(index).face_id().0))
            {
                println!("SKIP: {:?}", p.make_ref(index).face_id())
            } else {
                p.make_mut_ref(index).remove();
            }
        }

        index.move_all_polygons(glue_mesh, to_mesh);
        glue_mesh.make_mut_ref(index).remove();

        Ok(())
    }

    fn add_mesh_no_del(
        &self,
        glue: &dyn GeometryDyn,
        index: &mut GeoIndex,
    ) -> anyhow::Result<MeshId> {
        let glue_mesh = index.new_mesh();
        glue.polygonize(glue_mesh.make_mut_ref(index), 0)?;

        Ok(glue_mesh)
    }

    fn add_hole(
        &self,
        hole: &dyn GeometryDyn,
        in_mesh: MeshId,
        index: &mut GeoIndex,
    ) -> anyhow::Result<()> {
        let hole_mesh = index.new_mesh();
        println!("hole mesh: {hole_mesh:?} < {in_mesh:?}");
        hole.polygonize(hole_mesh.make_mut_ref(index), 0)?;

        let to_delete = self.holes_deletions(in_mesh, hole_mesh, index);

        for p in to_delete {
            /*
            if hole_mesh == 13 {
                let face_id = p.make_ref(index).face_id();
                println!(
                    "to-delete: {:?} {:?} {:?}",
                    face_id,
                    p.make_ref(index).poly_id(),
                    p.make_ref(index).mesh_id()
                );
            }
            */
            p.make_mut_ref(index).remove();
        }

        for p in hole_mesh.make_ref(index).all_polygons() {
            p.make_mut_ref(index).flip();
        }

        index.move_all_polygons(hole_mesh, in_mesh);
        hole_mesh.make_mut_ref(index).remove();

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
        let hole = index.new_mesh();

        let pin2 = Rect::build()
            .height(self.hotswap_rect_height)
            .width(self.hotswap_rect_width_far)
            .depth(self.hotswap_thickness /* + Dec::from(dec!(0.1))*/)
            .origin(
                zero.clone()
                    .offset_x(self.far_pin_distance[0])
                    .offset_y(self.far_pin_distance[1]),
            )
            .align_z(Align::Pos)
            .build();

        for p in pin2.render() {
            hole.make_mut_ref(index).add_polygon(&p)?;
        }

        self.add_material(
            &Rect::build()
                .height(self.hotswap_rect_height)
                .width(self.hotswap_rect_width_near)
                .depth(self.hotswap_thickness)
                .origin(
                    zero.clone()
                        .offset_x(self.near_pin_distance[0])
                        .offset_y(self.near_pin_distance[1]),
                )
                .align_z(Align::Pos)
                .build(),
            hole,
            index,
        )?;

        self.add_material(
            &Rect::build()
                .height(self.hotswap_chamfer_radius)
                .width(self.hotswap_chamfer_radius)
                .depth(self.hotswap_thickness /*+ Dec::from(dec!(0.1))*/)
                .origin(
                    zero.clone()
                        .offset_x(self.near_pin_distance[0])
                        .offset_x(self.hotswap_rect_width_near / 2)
                        .offset_y(self.far_pin_distance[1])
                        .offset_y(self.hotswap_rect_height / 2)
                        .clone(),
                )
                .align_z(Align::Pos)
                .align_x(Align::Neg)
                .align_y(Align::Neg)
                .build(),
            hole,
            index,
        )?;

        self.add_material(
            &Rect::build()
                .height(self.hotswap_chamfer_radius)
                .width(self.hotswap_chamfer_radius)
                .depth(self.hotswap_thickness)
                .origin(
                    zero.clone()
                        .offset_x(self.far_pin_distance[0])
                        .offset_x(-self.hotswap_rect_width_far / 2)
                        .offset_y(self.near_pin_distance[1])
                        .offset_y(-self.hotswap_rect_height / 2)
                        .clone(),
                )
                .align_z(Align::Pos)
                .align_x(Align::Pos)
                .align_y(Align::Pos)
                .build(),
            hole,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_x(self.near_pin_distance[0])
                    .offset_x(self.hotswap_rect_width_near / 2)
                    .offset_y(self.far_pin_distance[1])
                    .offset_y(self.hotswap_rect_height / 2)
                    .offset_x(self.hotswap_chamfer_radius)
                    .offset_y(self.hotswap_chamfer_radius),
                self.hotswap_thickness,
                self.hotswap_chamfer_radius,
            )
            .bottom_cap(false)
            .top_cap(false)
            .steps(32),
            hole,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_x(self.far_pin_distance[0])
                    .offset_x(-self.hotswap_rect_width_far / 2)
                    .offset_y(self.near_pin_distance[1])
                    .offset_y(-self.hotswap_rect_height / 2)
                    .offset_x(-self.hotswap_chamfer_radius)
                    .offset_y(-self.hotswap_chamfer_radius),
                self.hotswap_thickness,
                self.hotswap_chamfer_radius,
            )
            .bottom_cap(false)
            .top_cap(false)
            .steps(32),
            hole,
            index,
        )?;

        self.add_material(
            &Rect::build()
                .origin(
                    zero.clone()
                        .offset_x(self.near_pin_distance[0] - self.hotswap_rect_width_near / 2)
                        .offset_y(self.near_pin_distance[1]),
                )
                .align_z(Align::Pos)
                .align_x(Align::Pos)
                .width(4)
                .height(self.hs_ear_width)
                .depth(self.hs_ear_depth)
                .build(),
            hole,
            index,
        )?;

        self.add_material(
            &Rect::build()
                .origin(
                    zero.clone()
                        .offset_x(self.far_pin_distance[0] + self.hotswap_rect_width_far / 2)
                        .offset_y(self.far_pin_distance[1]),
                )
                .align_z(Align::Pos)
                .align_x(Align::Neg)
                .width(4)
                .height(self.hs_ear_width)
                .depth(self.hs_ear_depth)
                .build(),
            hole,
            index,
        )?;
        Ok(hole)
    }

    #[allow(clippy::vec_init_then_push)]
    pub fn bottom_mesh(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        let zero = Origin::new().offset_z(-self.pcb_thickness);
        let bed_point = zero
            .clone()
            .offset_x(-self.mount_width / 2)
            .offset_y(-self.mount_height / 2);

        let x_pin_distance = dbg!(self.near_pin_distance[0] - self.far_pin_distance[0]).abs();
        let wh_width = self.hotswap_rect_width_near + x_pin_distance;
        let wh_width_with_ears = Dec::from(dec!(1)) + wh_width;

        let material = Rect::build()
            .origin(bed_point.clone())
            .width(wh_width_with_ears)
            .height(self.mount_height * 3 / 4)
            .depth(self.hotswap_thickness + Dec::one())
            .align_z(Align::Pos)
            .align_y(Align::Neg)
            .align_x(Align::Neg)
            .build();

        let hotswap_bottom = index.new_mesh();

        for p in material.render() {
            hotswap_bottom.make_mut_ref(index).add_polygon(&p)?;
        }

        self.add_material(
            &Rect::build()
                .origin(bed_point)
                .width(5)
                .height(0.2)
                .depth(self.hotswap_thickness + Dec::one())
                .align_z(Align::Pos)
                .align_x(Align::Neg)
                .align_y(Align::Pos)
                .build(),
            hotswap_bottom,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_bottom_at(
                zero.clone().offset_z(-self.bottom_mesh_button_holes_depth),
                self.pcb_thickness + (Dec::one() * 3) / 2,
                self.main_hole_radius,
            )
            .top_cap(false)
            .steps(32),
            hotswap_bottom,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_bottom_at(
                zero.clone()
                    .offset_z(-self.bottom_mesh_button_holes_depth)
                    .offset_x(-self.side_hole_distance),
                self.pcb_thickness + (Dec::one() * 3) / 2,
                self.side_hole_radius,
            )
            .steps(32)
            .top_cap(false),
            hotswap_bottom,
            index,
        )?;

        /*
        let mut holes: Vec<Box<dyn GeometryDyn>> = Vec::new();
        holes.push(Box::new(main_hole));
        holes.push(Box::new(left));


        self.apply_holes(
            &holes.iter().map(|b| b.as_ref()).collect_vec(),
            hotswap_bottom,
            index,
        )?;
        */

        let first_bolt_center = zero
            .clone()
            .offset_x(self.screw_coords_1()[0])
            .offset_y(self.screw_coords_1()[1]);

        let first_bolt_body = Cylinder::with_top_at(
            first_bolt_center,
            dec!(5),
            self.bottom_mesh_screw_thread_diameter / 2,
        );

        self.add_hole(&first_bolt_body, hotswap_bottom, index)?;

        let second_bolt_center = zero
            .clone()
            .offset_x(self.screw_coords_2()[0])
            .offset_y(self.screw_coords_2()[1]);

        self.add_hole(
            &Cylinder::with_top_at(
                second_bolt_center,
                dec!(5),
                self.bottom_mesh_screw_thread_diameter / 2,
            ),
            hotswap_bottom,
            index,
        )?;

        let hw_hole = self.hotswap_hole(index)?;

        let hw_hole_polies = hw_hole
            .make_ref(index)
            .into_polygons()
            .into_iter()
            .sorted_by_key(|p| p.make_ref(index).face_id().0)
            .collect_vec();

        for p in [352]
            .into_iter()
            .map(FaceId)
            .flat_map(|f| index.get_face_with_root_parent(f))
            .flat_map(|f| {
                hw_hole_polies
                    .iter()
                    .copied()
                    .filter(|p| p.make_ref(index).face_id() == f)
                    .collect_vec()
            })
            .collect::<HashSet<_>>()
            .into_iter()
            .sorted_by_key(|f| f.make_ref(index).face_id().0)
        {
            println!(
                "SELECTED IN HOLE: {:?}[{:?}] ({:?})",
                p.make_ref(index).face_id(),
                p.make_ref(index).poly_id(),
                p.make_ref(index).mesh_id(),
            );
            // p.make_mut_ref(index).flip();
        }

        self.treat_as_hole_in(hw_hole, hotswap_bottom, index);

        let result_polies = hotswap_bottom
            .make_ref(index)
            .into_polygons()
            .into_iter()
            .sorted_by_key(|p| p.make_ref(index).face_id().0)
            .collect_vec();

        for p in [352, 355 /*, 356 */]
            .into_iter()
            .map(FaceId)
            .flat_map(|f| index.get_face_with_root_parent(f))
            .flat_map(|f| {
                result_polies
                    .iter()
                    .copied()
                    .filter(|p| p.make_ref(index).face_id() == f)
                    .collect_vec()
            })
            .collect::<HashSet<_>>()
            .into_iter()
            .sorted_by_key(|f| f.make_ref(index).face_id().0)
        {
            println!(
                "SELECTED: {:?}[{:?}] ({:?})",
                p.make_ref(index).face_id(),
                p.make_ref(index).poly_id(),
                p.make_ref(index).mesh_id(),
            );
            //p.make_mut_ref(index).flip();
            for s in p.make_ref(index).segments() {
                println!("F: {:?}-> {:?} [{:?}]", s.from_pt(), s.to_pt(), s.rib_id())
            }

            if p.make_ref(index).face_id().0 == 352 {
                //p.make_mut_ref(index).remove();
                println!(
                    "REMOVED: {:?}[{:?}] ({:?})",
                    p.make_ref(index).face_id(),
                    p.make_ref(index).poly_id(),
                    p.make_ref(index).mesh_id(),
                );
            }
        }

        Ok(())
    }

    pub fn treat_as_hole_in(&self, hole_mesh: MeshId, in_mesh: MeshId, index: &mut GeoIndex) {
        let del = self.holes_deletions(in_mesh, hole_mesh, index);

        for p in del {
            let face_id = p.make_ref(index).face_id();
            println!(
                "TREAT AS HOLE REMOVE: {:?} {:?} {:?}",
                face_id,
                p.make_ref(index).poly_id(),
                p.make_ref(index).mesh_id()
            );

            if ![/*2304,2306,2309,2300, 2299,*/ 2303, 2295].contains(&face_id.0) {
                p.make_mut_ref(index).remove();
            }
        }

        for p in hole_mesh.make_ref(index).into_polygons() {
            let face_id = p.make_ref(index).face_id();
            println!(
                "TREAT AS HOLE FLIP: {:?} {:?} {:?}",
                face_id,
                p.make_ref(index).poly_id(),
                p.make_ref(index).mesh_id()
            );
            p.make_mut_ref(index).flip();
        }
        index.move_all_polygons(hole_mesh, in_mesh);
        hole_mesh.make_mut_ref(index).remove();
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
                zero.clone()
                    .offset_z(-self.pcb_thickness)
                    .offset_x(self.mount_lock_width / 2)
                    .offset_x(glue_size_x / 2),
                glue_size_x,
                glue_size_y,
                glue_size_z,
            ),
            part,
            index,
        )?;

        { /*
                     let h = Dec::from(2.2);
                     let hh = h - self.bottom_mesh_screw_head_diameter / 2;

                     // Material at top of the lock (bottom of key)
                     self.add_material(
                         &Rect::build()
                             .origin(
                                 zero.clone()
                                     .offset_z(self.lock_to_bed_distance + self.mount_lock_depth)
                                     .offset_x(-self.mount_width / 2)
                                     .offset_y(-self.mount_height / 2),
                             )
                             .width(5)
                             .height(h)
                             .depth(self.mount_lock_depth)
                             .align_z(Align::Pos)
                             .align_x(Align::Neg)
                             .align_y(Align::Pos)
                             .build(),
                         part,
                         index,
                     )?;

                     // Bolt head
                     self.add_hole(
                         &Cylinder::with_top_at(
                             zero.clone()
                                 .offset_z(self.lock_to_bed_distance + self.mount_lock_depth)
                                 .offset_x(-self.mount_width / 2)
                                 .offset_x(Dec::from(5) / 2)
                                 .offset_y(-hh)
                                 .offset_y(-self.mount_height / 2),
                             0.5,
                             self.bottom_mesh_screw_head_diameter / 2,
                         ),
                         part,
                         index,
                     )?;

                     // Bolt thread
                     self.add_hole(
                         &Cylinder::with_top_at(
                             zero.clone()
                                 .offset_z(self.lock_to_bed_distance + self.mount_lock_depth)
                                 .offset_x(-self.mount_width / 2)
                                 .offset_x(Dec::from(5) / 2)
                                 .offset_y(-hh)
                                 .offset_y(-self.mount_height / 2),
                             5,
                             self.bottom_mesh_screw_thread_diameter / 2 * 9 / 10,
                         ),
                         part,
                         index,
                     )?;

             */
        }
        { //
             /*
             // Material at top of the lock (top of key)
                 self.add_material(
                     &Rect::build()
                         .origin(
                             zero.clone()
                                 .offset_z(self.lock_to_bed_distance + self.mount_lock_depth)
                                 .offset_x(self.mount_width / 2)
                                 .offset_y(self.mount_height / 2),
                         )
                         .width(5)
                         .height(h)
                         .depth(self.mount_lock_depth)
                         .align_z(Align::Pos)
                         .align_x(Align::Pos)
                         .align_y(Align::Neg)
                         .build(),
                     part,
                     index,
                 )?;

                 // HEAD
                 self.add_hole(
                     &Cylinder::with_top_at(
                         zero.clone()
                             .offset_z(self.lock_to_bed_distance + self.mount_lock_depth)
                             .offset_x(self.mount_width / 2)
                             .offset_x(-Dec::from(5) / 2)
                             .offset_y(hh)
                             .offset_y(self.mount_height / 2),
                         0.5,
                         self.bottom_mesh_screw_head_diameter / 2,
                     ),
                     part,
                     index,
                 )?;

                 // THREAD
                 self.add_hole(
                     &Cylinder::with_top_at(
                         zero.clone()
                             .offset_z(self.lock_to_bed_distance + self.mount_lock_depth)
                             .offset_x(self.mount_width / 2)
                             .offset_x(-Dec::from(5) / 2)
                             .offset_y(hh)
                             .offset_y(self.mount_height / 2),
                         5,
                         self.bottom_mesh_screw_thread_diameter / 2 * 9 / 10,
                     ),
                     part,
                     index,
                 )?;
                 */
        }

        // litter matter near bolt
        self.add_material(
            &Rect::build()
                .origin(
                    zero.clone()
                        .offset_z(self.lock_to_bed_distance + self.mount_lock_depth)
                        .offset_x(-self.mount_width / 2)
                        .offset_y(-self.mount_height / 2),
                )
                .width(5)
                .height(0.5)
                .depth(self.pcb_thickness + self.lock_to_bed_distance + self.mount_lock_depth)
                .align_z(Align::Pos)
                .align_x(Align::Neg)
                .align_y(Align::Pos)
                .build(),
            part,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_x(self.screw_coords_1()[0])
                    .offset_y(self.screw_coords_1()[1])
                    .offset_z(self.mount_lock_depth)
                    .offset_z(self.lock_to_bed_distance)
                    .clone(),
                self.mount_lock_depth + self.lock_to_bed_distance + Dec::from(dec!(0.5)),
                self.bottom_mesh_screw_head_diameter / 2,
            ),
            part,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_x(self.screw_coords_1()[0])
                    .offset_y(self.screw_coords_1()[1])
                    .offset_z(self.mount_lock_depth)
                    .offset_z(self.lock_to_bed_distance)
                    .offset_z(-self.mount_lock_depth)
                    .offset_z(-self.lock_to_bed_distance),
                10,
                self.bottom_mesh_screw_thread_diameter / 2 * 11 / 10,
            ),
            part,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_x(self.screw_coords_2()[0])
                    .offset_y(self.screw_coords_2()[1])
                    .offset_z(self.mount_lock_depth)
                    .offset_z(self.lock_to_bed_distance)
                    .clone(),
                self.mount_lock_depth + self.lock_to_bed_distance + Dec::from(dec!(0.5)),
                self.bottom_mesh_screw_head_diameter / 2,
            ),
            part,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_x(self.screw_coords_2()[0])
                    .offset_y(self.screw_coords_2()[1])
                    .offset_z(self.mount_lock_depth)
                    .offset_z(self.lock_to_bed_distance)
                    .offset_z(-self.mount_lock_depth)
                    .offset_z(-self.lock_to_bed_distance),
                10,
                self.bottom_mesh_screw_thread_diameter / 2 * 11 / 10,
            ),
            part,
            index,
        )?;

        Ok(())
    }

    fn screw_coords_1(&self) -> [Dec; 2] {
        [Dec::from(-dec!(5.0)), Dec::from(-dec!(7.3))]
    }

    fn screw_coords_2(&self) -> [Dec; 2] {
        [Dec::from(dec!(1.5)), Dec::from(dec!(2.5))]
    }

    fn bed(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let bed = index.new_mesh();
        let zero = Origin::new();

        let rect = Rect::with_top_at(
            zero.clone(),
            self.mount_lock_width + Dec::from(dec!(0.7)),
            self.mount_height,
            self.pcb_thickness,
        );

        for p in rect.render() {
            bed.make_mut_ref(index).add_polygon(&p)?;
        }

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone().offset_z(dec!(0.5)),
                self.pcb_thickness + Dec::one(),
                self.main_hole_radius,
            )
            .steps(32)
            .top_cap(false)
            .bottom_cap(false),
            bed,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_z(dec!(0.5))
                    .offset_x(self.side_hole_distance),
                self.pcb_thickness + Dec::one(),
                self.side_hole_radius,
            )
            .steps(32)
            .top_cap(false)
            .bottom_cap(false),
            bed,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_z(dec!(0.5))
                    .offset_x(-self.side_hole_distance),
                self.pcb_thickness + Dec::one(),
                self.side_hole_radius,
            )
            .steps(32)
            .top_cap(false)
            .bottom_cap(false),
            bed,
            index,
        )?;

        for p in bed
            .make_ref(index)
            .into_polygons()
            .into_iter()
            .sorted_by_key(|p| p.make_ref(index).face_id().0)
        {
            let face_id = p.make_ref(index).face_id();
            if face_id.0 == 333 {
                println!("flip face {:?}", face_id);

                p.make_mut_ref(index).flip();
            }
        }

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_z(dec!(0.5))
                    .offset_x(self.near_pin_distance[0])
                    .offset_y(self.near_pin_distance[1]),
                self.pcb_thickness + Dec::one(),
                self.pcb_pin_diameter / 2,
            )
            .steps(32)
            .top_cap(false)
            .bottom_cap(false),
            bed,
            index,
        )?;

        self.add_hole(
            &Cylinder::with_top_at(
                zero.clone()
                    .offset_z(dec!(0.5))
                    .offset_x(self.far_pin_distance[0])
                    .offset_y(self.far_pin_distance[1]),
                self.pcb_thickness + Dec::one(),
                self.pcb_pin_diameter / 2,
            )
            .steps(32)
            .top_cap(false)
            .bottom_cap(false),
            bed,
            index,
        )?;

        Ok(bed)
    }

    pub fn outer_mount(&self, center: Origin, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let upper_plane = index.new_mesh();
        let lower_plane = index.new_mesh();
        let total_depth = self.pcb_thickness + self.lock_to_bed_distance + self.mount_lock_depth;
        let top = center.offset_z(total_depth / 2);
        let bottom = top
            .clone()
            .offset_z(-total_depth)
            .rotate_axisangle(Vector3::x() * Dec::pi());

        Plane::centered(
            top.clone(),
            self.outer_mount_width,
            self.outer_mount_height,
            1,
        )
        .polygonize(upper_plane.make_mut_ref(index), 0)?;

        Plane::centered(
            bottom.clone(),
            self.outer_mount_width,
            self.outer_mount_height,
            1,
        )
        .polygonize(lower_plane.make_mut_ref(index), 0)?;
        println!("{lower_plane:?}");
        for f in lower_plane.make_ref(index).into_polygons() {
            println!("Plane polyes: {f:?}: {:?}", f.make_ref(index).face_id());
        }

        let rect_hole = index.new_mesh();
        Rect::build()
            .align_z(Align::Pos)
            .origin(top.clone())
            .width(self.mount_width)
            .height(self.mount_height)
            .depth(total_depth * 10001 / 10000)
            .build()
            .polygonize(rect_hole.make_mut_ref(index), 0)?;

        let mount = upper_plane;

        let remove = [
            rect_hole
                .make_ref(index)
                .shared_with(upper_plane.make_ref(index)),
            rect_hole
                .make_ref(index)
                .shared_with(lower_plane.make_ref(index)),
            lower_plane
                .make_ref(index)
                .shared_with(rect_hole.make_ref(index)),
            upper_plane
                .make_ref(index)
                .shared_with(rect_hole.make_ref(index)),
        ]
        .concat();

        for r in remove {
            println!(
                "REMOVE SHARED: {:?} {:?} {:?}",
                r.make_ref(index).face_id(),
                r.make_ref(index).poly_id(),
                r.make_ref(index).mesh_id()
            );
            r.make_mut_ref(index).remove();
        }
        for p in rect_hole.make_ref(index).into_polygons() {
            p.make_mut_ref(index).flip();
        }
        index.move_all_polygons(lower_plane, mount);
        index.move_all_polygons(rect_hole, mount);

        self.add_material(
            &Rect::build()
                .origin(
                    bottom
                        .offset_z(Dec::one() / 100)
                        .offset_y(-self.mount_width / 2),
                )
                .align_z(Align::Neg)
                .width(self.mount_height * 105 / 100)
                .height(2)
                .depth(0.7)
                .build(),
            mount,
            index,
        )?;
        let h = (self.outer_mount_height - self.mount_height) / 2;
        let hh = h - self.bottom_mesh_screw_head_diameter / 2;

        let one_hole_mesh = index.new_mesh();
        println!("~~~~~~~~~~~~~~~AFTER{one_hole_mesh:?}~~~~~~~~~~~~~~~~~~~~~~~");

        self.add_hole(
            &Rect::build()
                .origin(
                    top.clone()
                        .offset_x(-self.mount_width / 2)
                        .offset_y(-self.mount_height / 2), //.offset_z(-self.mount_lock_depth),
                )
                .width(5)
                .height(0.5)
                .depth(
                    (self.pcb_thickness + self.lock_to_bed_distance + self.mount_lock_depth)
                        * 10001
                        / 10000,
                )
                .align_z(Align::Pos)
                .align_x(Align::Neg)
                .align_y(Align::Pos)
                .build(),
            mount,
            index,
        )?;
        //.polygonize(one_hole_mesh.make_mut_ref(index), 0)?;

        /*
        //println!("~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~");
        self.add_material(
            &Rect::build()
                .origin(
                    top.clone()
                        .offset_x(-self.mount_width / 2)
                        .offset_y(-self.mount_height / 2),
                )
                .width(5)
                .height(h)
                .depth(self.mount_lock_depth)
                .align_z(Align::Pos)
                .align_x(Align::Neg)
                .align_y(Align::Pos)
                .build(),
            one_hole_mesh,
            index,
        )?;

        self.treat_as_hole_in(one_hole_mesh, mount, index);
        index.move_all_polygons(one_hole_mesh, mount);

        /*
        for p in mount
            .make_ref(index)
            .into_polygons()
            .into_iter()
            .sorted_by_key(|p| p.make_ref(index).face_id().0)
        {
            let face_id = p.make_ref(index).face_id();
            if [2303, 2307, 2308]
                .into_iter()
                .flat_map(|i| index.get_face_with_root_parent(FaceId(i)))
                .any(|f| f == face_id)
            {
                println!(
                    "flip in mesh: {:?} {:?} {:?}",
                    face_id,
                    p.make_ref(index).poly_id(),
                    p.make_ref(index).mesh_id()
                );
                p.make_mut_ref(index).flip();
            }
        }
        */

        // Bolt thread
        self.add_hole(
            &Cylinder::with_top_at(
                top.clone()
                    .offset_x(-self.mount_width / 2)
                    .offset_y(-self.mount_height / 2)
                    .offset_x(Dec::from(5) / 2)
                    .offset_y(-hh),
                5,
                self.bottom_mesh_screw_thread_diameter / 2 * 9 / 10,
            ),
            mount,
            index,
        )?;

        self.add_hole(
            &Rect::build()
                .origin(
                    top.clone()
                        .offset_x(self.mount_width / 2)
                        .offset_y(self.mount_height / 2),
                )
                .width(5)
                .height(h)
                .depth(self.mount_lock_depth)
                .align_z(Align::Pos)
                .align_x(Align::Pos)
                .align_y(Align::Neg)
                .build(),
            mount,
            index,
        )?;

        // Bolt thread
        self.add_hole(
            &Cylinder::with_top_at(
                top.clone()
                    .offset_x(self.mount_width / 2)
                    .offset_y(self.mount_height / 2)
                    .offset_x(-Dec::from(5) / 2)
                    .offset_y(hh),
                5,
                self.bottom_mesh_screw_thread_diameter / 2 * 9 / 10,
            ),
            mount,
            index,
        )?;
        */

        Ok(mount)
    }
}
