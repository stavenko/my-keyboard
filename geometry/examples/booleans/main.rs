use std::{
    fs::{self},
    path::PathBuf,
};

use clap::Parser;
use geometry::{
    decimal::Dec,
    geometry::GeometryDyn,
    indexes::{
        aabb::Aabb,
        geo_index::{geo_object::GeoObject, index::GeoIndex},
    },
    origin::Origin,
    shapes::{Plane, Rect},
};

use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::One;
use rust_decimal_macros::dec;

#[derive(Parser)]
pub struct Command {
    #[arg(long)]
    pub output_path: PathBuf,
}

fn rib_unification_1(file_root: PathBuf) -> anyhow::Result<String> {
    let mut index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-10), Dec::from(-10), Dec::from(-10)),
        Vector3::new(Dec::from(15), Dec::from(14), Dec::from(10)),
    ]))
    .debug_svg_path(file_root.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let zero = Origin::new();

    let mesh_one = index.new_mesh();

    Plane::centered(zero.clone(), Dec::one(), Dec::one(), 1)
        .polygonize(mesh_one.make_mut_ref(&mut index), 0)?;

    let mesh_two = index.new_mesh();

    Plane::centered(
        zero.clone().offset_x(dec!(1.5)),
        Dec::one() * 2,
        Dec::one() * 2,
        1,
    )
    .polygonize(mesh_two.make_mut_ref(&mut index), 0)?;

    Ok(index.scad())
}

fn rib_unification_2(file_root: PathBuf) -> anyhow::Result<String> {
    let mut index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-10), Dec::from(-10), Dec::from(-10)),
        Vector3::new(Dec::from(15), Dec::from(14), Dec::from(10)),
    ]))
    .debug_svg_path(file_root.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let mesh_one = index.new_mesh();
    let zero = Origin::new();

    Plane::centered(
        zero.clone().offset_x(dec!(1.5)),
        Dec::one() * 2,
        Dec::one() * 2,
        1,
    )
    .polygonize(mesh_one.make_mut_ref(&mut index), 0)?;

    let mesh_two = index.new_mesh();
    Plane::centered(zero.clone(), Dec::one(), Dec::one(), 1)
        .polygonize(mesh_two.make_mut_ref(&mut index), 0)?;

    Ok(index.scad())
}

fn cut_planes(file_root: PathBuf) -> anyhow::Result<String> {
    let mut index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-10), Dec::from(-10), Dec::from(-10)),
        Vector3::new(Dec::from(15), Dec::from(14), Dec::from(10)),
    ]))
    .debug_svg_path(file_root.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let zero = Origin::new();
    let rotated = zero
        .clone()
        .rotate_axisangle(Vector3::x() * (Dec::from(dec!(3.14)) / 2));

    let mesh_one = index.new_mesh();
    Plane::centered(zero, Dec::one(), Dec::one(), 1)
        .polygonize(mesh_one.make_mut_ref(&mut index), 0)?;
    let mesh_two = index.new_mesh();
    Plane::centered(rotated, Dec::one() * 2, Dec::one() * 2, 1)
        .polygonize(mesh_two.make_mut_ref(&mut index), 0)?;

    Ok(index.scad())
}

fn overlap_in_center(file_root: PathBuf) -> anyhow::Result<String> {
    let mut index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-10), Dec::from(-10), Dec::from(-10)),
        Vector3::new(Dec::from(15), Dec::from(14), Dec::from(10)),
    ]))
    .debug_svg_path(file_root.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let zero = Origin::new();

    let small_box = index.new_mesh();
    Rect::centered(zero.clone().offset_x(1), Dec::one(), Dec::one(), Dec::one())
        .polygonize(small_box.make_mut_ref(&mut index), 0)?;
    let big_box = index.new_mesh();
    Rect::centered(zero, Dec::one() * 2, Dec::one() * 2, Dec::one() * 2)
        .polygonize(big_box.make_mut_ref(&mut index), 0)?;

    let to_remove = [
        small_box
            .make_ref(&index)
            .front_of(big_box.make_ref(&index)),
        big_box.make_ref(&index).back_of(small_box.make_ref(&index)),
    ]
    .into_iter()
    .flatten()
    .collect_vec();

    for p in to_remove {
        p.make_mut_ref(&mut index).remove();
    }
    Ok(index.scad())
}

fn overlap_touching_edge(file_root: PathBuf) -> anyhow::Result<String> {
    let mut index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-10), Dec::from(-10), Dec::from(-10)),
        Vector3::new(Dec::from(15), Dec::from(14), Dec::from(10)),
    ]))
    .debug_svg_path(file_root.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let zero = Origin::new();

    let smaller_box = Rect::centered(
        zero.clone().offset_x(1).offset_y(Dec::from(1) / 2),
        Dec::one(),
        Dec::one(),
        Dec::one(),
    );

    let bigger_box = Rect::centered(zero, Dec::one() * 2, Dec::one() * 2, Dec::one() * 2);

    let small = index.new_mesh();
    smaller_box.polygonize(small.make_mut_ref(&mut index), 0)?;
    let big = index.new_mesh();
    bigger_box.polygonize(big.make_mut_ref(&mut index), 0)?;

    let remove = [
        index.select_polygons(
            small,
            big,
            geometry::indexes::geo_index::index::PolygonFilter::Front,
        ),
        index.select_polygons(
            big,
            small,
            geometry::indexes::geo_index::index::PolygonFilter::Back,
        ),
        index.select_polygons(
            big,
            small,
            geometry::indexes::geo_index::index::PolygonFilter::Shared,
        ),
        index.select_polygons(
            small,
            big,
            geometry::indexes::geo_index::index::PolygonFilter::Shared,
        ),
    ]
    .concat();

    for poly in remove {
        //println!("remove {poly:?}: {:?}", poly.make_ref(&index).face_id());
        poly.make_mut_ref(&mut index).remove();
    }
    /*
    for poly in small.make_ref(&index).into_polygons() {
        let f = poly.make_ref(&index).face_id();
        let p = poly.make_ref(&index).poly_id();
        println!("p: {:?} / {:?}", p, f);
        if f.0 == 10 {
            poly.make_mut_ref(&mut index).remove();
        }
    }

    for f in index.get_face_with_root_parent(FaceId(5)) {
        println!(">>> {f:?}");
        for p in small.make_ref(&index).all_polygons() {
            println!("  >>> {p:?} {:?}", p.make_ref(&index).face_id());

            if p.make_ref(&index).face_id() == f {
                dbg!(p);
                p.make_mut_ref(&mut index).flip();
            }
        }
    }
    */
    index.move_all_polygons(big, small);
    big.make_mut_ref(&mut index).remove();

    Ok(index.scad())
}

fn overlap_touching_edge_with_opposite_polygons(file_root: PathBuf) -> anyhow::Result<String> {
    let mut index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-10), Dec::from(-10), Dec::from(-10)),
        Vector3::new(Dec::from(15), Dec::from(14), Dec::from(10)),
    ]))
    .debug_svg_path(file_root.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let zero = Origin::new();

    let smaller_box = Rect::build()
        .origin(zero.clone().offset_z(-1))
        .align_z(geometry::shapes::Align::Pos)
        .build();

    let cutter = Plane::centered(zero.clone().offset_z(0.9), 10, 10, 1);
    let bigger_box = Rect::centered(zero, Dec::one() * 2, Dec::one() * 2, Dec::one() * 2);

    //let small = index.get_current_default_mesh();
    //smaller_box.polygonize(&mut index, 0)?;
    let big = index.new_mesh();
    bigger_box.polygonize(big.make_mut_ref(&mut index), 0)?;
    let cut = index.new_mesh();
    cutter.polygonize(cut.make_mut_ref(&mut index), 0)?;
    for p in [
        cut.make_ref(&index).all_polygons(),
        big.make_ref(&index).front_of(cut.make_ref(&index)),
    ]
    .into_iter()
    .flatten()
    {
        p.make_mut_ref(&mut index).remove();
    }

    let smal = index.new_mesh();
    smaller_box
        .polygonize(smal.make_mut_ref(&mut index), 0)
        .unwrap();

    let shared_of_big = big.make_ref(&index).shared_with(smal.make_ref(&index));
    let shared_of_cut = smal.make_ref(&index).shared_with(big.make_ref(&index));
    for p in shared_of_cut {
        p.make_mut_ref(&mut index).remove();
    }
    for p in shared_of_big {
        p.make_mut_ref(&mut index).remove();
    }

    Ok(index.scad())
}
/*

fn complex_cut(file_root: PathBuf) -> anyhow::Result<String> {
    let mut index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-10), Dec::from(-10), Dec::from(-10)),
        Vector3::new(Dec::from(15), Dec::from(14), Dec::from(10)),
    ]))
    .debug_svg_path(file_root.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let zero = Origin::new();

    let platform = Rect::centered(zero.clone(), Dec::one(), Dec::one(), Dec::one() / 5);

    let cutter = Rect::centered(
        zero.clone().offset_x(0.2).offset_y(0.2),
        Dec::one(),
        Dec::one(),
        Dec::one(),
    );

    let platform_mesh = index.get_current_default_mesh();
    platform.polygonize(&mut index, 0)?;
    let cutter_mesh = index.create_new_mesh_and_set_as_default();
    cutter.polygonize(&mut index, 0)?;

    let remove = [
        index.select_polygons(
            cutter_mesh,
            platform_mesh,
            geometry::indexes::geo_index::index::PolygonFilter::Front,
        ),
        index.select_polygons(
            platform_mesh,
            cutter_mesh,
            geometry::indexes::geo_index::index::PolygonFilter::Back,
        ),
    ]
    .concat();

    for (mesh_id, poly) in &remove {
        index.remove_polygon(*poly, *mesh_id);
    }

    for p in index.get_mesh_polygons(cutter_mesh) {
        index.flip_polygon(p);
    }

    let matter = Rect::centered(
        zero.offset_x(-0.95),
        Dec::one(),
        Dec::from(dec!(0.6)),
        Dec::one() / 5,
    );

    index.move_all_polygons(cutter_mesh, platform_mesh);
    let matter_mesh = index.create_new_mesh_and_set_as_default();

    matter.polygonize(&mut index, 0)?;

    let select_polygons_2 = [
        index.select_polygons(
            matter_mesh,
            platform_mesh,
            geometry::indexes::geo_index::index::PolygonFilter::Back,
        ),
        index.select_polygons(
            platform_mesh,
            matter_mesh,
            geometry::indexes::geo_index::index::PolygonFilter::Back,
        ),
    ]
    .concat();

    for (mesh, poly) in select_polygons_2 {
        index.remove_polygon(poly, mesh);
    }
    index.move_all_polygons(matter_mesh, platform_mesh);

    Ok(index.scad())
}

fn apply_holes(
    holes: &[&dyn GeometryDyn],
    in_mesh: MeshId,
    index: &mut GeoIndex,
) -> anyhow::Result<()> {
    for hole in holes {
        let new_hole_id = index.create_new_mesh_and_set_as_default();
        hole.polygonize(index, 0)?;

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

fn some_interesting_mesh(index: &mut GeoIndex) -> anyhow::Result<MeshId> {
    let zero = Origin::new();
    let lock = index.create_new_mesh_and_set_as_default();

    let rect = Rect::with_bottom_at(zero.clone().offset_z(1.3), 18.into(), 18.into(), 1.2.into());

    for p in rect.render() {
        index.save_as_polygon(&p, None)?;
    }

    let lock_min = Rect::with_bottom_at(
        zero.clone(),
        Dec::from(13.8),
        Dec::from(13.8),
        Dec::from(1.2) + Dec::from(1.3) + Dec::one(),
    );

    let holes = vec![lock_min]
        .into_iter()
        .map(|h| -> Box<dyn GeometryDyn> { Box::new(h) })
        .collect_vec();

    apply_holes(&holes.iter().map(|b| b.as_ref()).collect_vec(), lock, index)?;

    Ok(lock)
}

fn some_other_interesting_mesh(index: &mut GeoIndex) -> anyhow::Result<MeshId> {
    let zero = Origin::new();

    let rect = Rect::with_top_at(
        zero.clone(),
        Dec::from(13.8) + Dec::from(dec!(0.7)),
        Dec::from(13.8) + Dec::from(dec!(0.7)),
        Dec::from(1.5),
    );
    for p in rect.render() {
        index.save_as_polygon(&p, None)?;
    }
    let main_mesh = index.get_current_default_mesh();

    let main_hole = Cylinder::with_top_at(
        zero.clone().offset_z(dec!(0.5)),
        Dec::from(1.5) + Dec::one(),
        Dec::from(1.7),
    )
    .top_cap(false)
    .bottom_cap(false)
    .steps(32);

    let right = Cylinder::with_top_at(
        zero.clone().offset_z(dec!(0.5)).offset_x(5.5),
        Dec::from(1.5) + Dec::one(),
        Dec::from(0.95),
    )
    .steps(32)
    .top_cap(false)
    .bottom_cap(false);

    let left = Cylinder::with_top_at(
        zero.clone().offset_z(dec!(0.5)).offset_x(-5.5),
        Dec::from(1.5) + Dec::one(),
        Dec::from(0.95),
    )
    .steps(32)
    .top_cap(false)
    .bottom_cap(false);

    let pin1 = Cylinder::with_top_at(
        zero.clone().offset_z(dec!(0.5)).offset_x(5).offset_y(-3.8),
        Dec::from(1.5) + Dec::one(),
        Dec::from(1.5),
    )
    .steps(32)
    .top_cap(false)
    .bottom_cap(false);

    let pin2 = Cylinder::with_top_at(
        zero.clone().offset_z(dec!(0.5)).offset_y(dec!(-5.9)),
        Dec::from(1.5) + Dec::one(),
        Dec::from(1.5),
    )
    .steps(32)
    .top_cap(false)
    .bottom_cap(false);

    let holes = vec![main_hole, left, right, pin1, pin2 /**/]
        .into_iter()
        .map(|h| -> Box<dyn GeometryDyn> { Box::new(h) })
        .collect_vec();

    apply_holes(
        &holes.iter().map(|b| b.as_ref()).collect_vec(),
        main_mesh,
        index,
    )?;

    Ok(main_mesh)
}

fn glue_mesh_to_mesh(
    one_mesh: MeshId,
    other_mesh: MeshId,
    index: &mut GeoIndex,
) -> anyhow::Result<MeshId> {
    let zero = Origin::new();
    let glue_size_x = Dec::from(18);
    let min = Dec::from(dec!(0.1));
    let mmin = Dec::from(dec!(0.005));
    let glue_size_y = (Dec::from(18) - Dec::from(13.8)) / 2;
    let glue_size_z = Dec::from(2.2) + Dec::from(1.5);

    let glue_mesh = Rect::with_bottom_at(
        zero.offset_z(-Dec::from(1.5))
            .offset_y(Dec::from(13.8) / 2)
            .offset_y(glue_size_y / 2),
        glue_size_x,
        glue_size_y,
        glue_size_z,
    );
    let glue_material = index.create_new_mesh_and_set_as_default();
    dbg!(glue_material, other_mesh);

    glue_mesh.polygonize(index, 0)?;

    /*for pp in [6]
                .into_iter()
                .flat_map(|p| index.get_polygon_with_root_parent(PolyId(p)))
                .collect::<HashSet<_>>()
            {
                println!("pp : {pp:?}");
                index.remove_polygon(pp);
            }

    */

    let to_delete = [
        index.select_polygons(glue_material, one_mesh, PolygonFilter::Back),
        index.select_polygons(glue_material, other_mesh, PolygonFilter::Back),
        index.select_polygons(one_mesh, glue_material, PolygonFilter::Back),
        index.select_polygons(other_mesh, glue_material, PolygonFilter::Back),
    ]
    .concat();

    for p in to_delete {
        println!("REMOVE--- {:?} ", p);

        index.remove_polygon(p);
    }

    for pp in [854]
        .into_iter()
        .flat_map(|p| index.get_polygon_with_root_parent(PolyId(p)))
        .collect::<HashSet<_>>()
    {
        println!("log pp : {pp:?}");
        index.remove_polygon(pp);
    }

    //index.move_all_polygons(other_mesh, one_mesh);
    //index.move_all_polygons(glue_material, one_mesh);

    Ok(one_mesh)
}

fn glueing_two_meshes(file_root: PathBuf) -> anyhow::Result<String> {
    let mut index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-10), Dec::from(-10), Dec::from(-10)),
        Vector3::new(Dec::from(15), Dec::from(14), Dec::from(10)),
    ]))
    .debug_svg_path(file_root.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    index.poly_split_debug(
        PolyId(175),
        PolygonBasis {
            center: Vector3::zeros(),
            x: Vector3::x(),
            y: Vector3::y(),
        },
    );

    let lock = some_interesting_mesh(&mut index)?;
    let bed = some_other_interesting_mesh(&mut index)?;

    let part = glue_mesh_to_mesh(lock, bed, &mut index)?;

    Ok(index.scad())
}
*/

/*

fn smaller_by_bigger(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let zz = Vector3::z();
    let yy = Vector3::y();
    let xx = yy.cross(&zz).normalize();

    let _zero_basis = Basis::new(xx, yy, zz, Vector3::zeros())?;

    let smaller_box = shapes::rect(
        _zero_basis.clone(),
        Dec::one() * 1,
        Dec::one() * 1,
        Dec::one() * 1,
    );

    let bigger_box = index.save_mesh(
        shapes::rect(_zero_basis, Dec::one() * 2, Dec::one() * 2, Dec::one() * 2)
            .into_iter()
            .map(Cow::Owned),
    );

    let smaller_box = index.save_mesh(smaller_box.into_iter().map(Cow::Owned));
    let smaller_box_mut = index.get_mutable_mesh(smaller_box);

    let result = smaller_box_mut.boolean_union(bigger_box).remove(0);

    let filename = "smaller_by_bigger.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.into()])
}

fn two_identical_boxes_with_overlapped_side_and_rotated(
    file_root: PathBuf,
) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let rot = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(Vector3::x()),
        Dec::from(std::f32::consts::FRAC_PI_4),
    );

    let x_basis_two = Basis::new(
        Vector3::x(),
        rot * Vector3::y(),
        rot * Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(0.5)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "one_with_rotated_overlapped_side.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.into()])
}
fn two_identical_boxes_one_with_one_common_side_rotated(
    file_root: PathBuf,
) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let rot = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(Vector3::x()),
        Dec::from(std::f32::consts::FRAC_PI_4),
    );

    let x_basis_two = Basis::new(
        Vector3::x(),
        rot * Vector3::y(),
        rot * Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(1.0)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "one_with_rotated_common_side.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.into()])
}
fn two_identical_boxes_one_with_one_common_side(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(1.0)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "one_with_other_overlap.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.into()])
}

fn two_identical_boxes_one_with_overlap(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(0.4).round_dp(STABILITY_ROUNDING)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "one_with_one_common_side.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn two_identical_boxes_one_shifted_in_plane(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * Dec::from(dec!(0.7)).round_dp(3)
            + Vector3::y() * Dec::from(dec!(0.6)).round_dp(3),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "shifted_in_plane.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn two_identical_boxes_one_shifted_in_space(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * Dec::from(dec!(0.5))
            + Vector3::y() * Dec::from(dec!(0.6))
            + Vector3::z() * Dec::from(dec!(0.9)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "shifted_in_space.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn bigger_box_extended_by_smaller(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(dec!(1.5)),
            Dec::one() * Dec::from(dec!(1.5)),
            Dec::one() * Dec::from(dec!(1.5)),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::z() * Dec::from(dec!(0.9)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(dec!(0.5)),
            Dec::one() * Dec::from(dec!(0.5)),
            Dec::one() * Dec::from(dec!(0.5)),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "extention_by_smaller.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn bigger_box_extended_by_longer(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(dec!(1.5)),
            Dec::one() * Dec::from(dec!(1.5)),
            Dec::one() * Dec::from(dec!(1.5)),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x = (Vector3::x() + Vector3::y()).normalize();
    let z = Vector3::z();
    let y = z.cross(&x).normalize();
    let x_basis_two = Basis::new(x, y, z, Vector3::z() * Dec::from(0.0))?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(0.5),
            Dec::one() * Dec::from(4.5),
            Dec::one() * Dec::from(0.5),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "extention_by_longer.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn smaller_box_cutted_by_bigger(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x = Vector3::x();
    let z = Vector3::z();
    let y = Vector3::y();
    let x_basis_two = Basis::new(x, y, z, Vector3::z() * Dec::from(0.5))?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(2.5),
            Dec::one() * Dec::from(2.5),
            Dec::one() * Dec::from(0.5),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_diff(box_two)
        .unwrap()
        .remove(0);

    let filename = "smaller_cutted_by_bigger.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn smaller_box_cutted_by_longer(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x = Vector3::x();
    let z = Vector3::z();
    let y = Vector3::y();
    let x_basis_two = Basis::new(x, y, z, Vector3::x() * Dec::from(0.1))?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(0.25),
            Dec::one() * Dec::from(0.25),
            Dec::one() * Dec::from(3.0),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_diff(box_two)
        .unwrap()
        .remove(0);

    let filename = "smaller_cutted_by_longer.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn smaller_box_cutted_by_bigger_in_two(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(2),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x = Vector3::x();
    let z = Vector3::z();
    let y = Vector3::y();
    let x_basis_two = Basis::new(x, y, z, Vector3::zeros())?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(2.5),
            Dec::one() * Dec::from(2.5),
            Dec::one() * Dec::from(0.5),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let mut paths = Vec::new();
    let mut results = index
        .get_mutable_mesh(box_one)
        .boolean_diff(box_two)
        .unwrap();

    let filename = "cutted_in_two1.stl";
    let r = results.remove(0);
    let p = file_root.join(filename);
    paths.push(filename.to_owned());
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(p)
        .unwrap();

    stl_io::write_stl(&mut writer, r.into_iter())?;

    let filename = "cutted_in_two2.stl";
    let r = results.remove(0);
    let p = file_root.join(filename);
    paths.push(filename.to_owned());
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(p)
        .unwrap();

    stl_io::write_stl(&mut writer, r.into_iter())?;
    Ok(paths)
}
*/

fn main() -> Result<(), anyhow::Error> {
    let cli = Command::parse();

    fs::create_dir_all(cli.output_path.clone())?;
    let meshes = [
        /*
         */
        rib_unification_1(cli.output_path.clone())?,
        rib_unification_2(cli.output_path.clone())?,
        cut_planes(cli.output_path.clone())?,
        overlap_in_center(cli.output_path.clone())?,
        overlap_touching_edge(cli.output_path.clone())?,
        overlap_touching_edge_with_opposite_polygons(cli.output_path.clone())?,
        /*
        complex_cut(cli.output_path.clone())?,
        glueing_two_meshes(cli.output_path.clone())?,
        smaller_by_bigger(cli.output_path.clone())?,
        two_identical_boxes_one_with_one_common_side(cli.output_path.clone())?,
        two_identical_boxes_one_with_overlap(cli.output_path.clone())?,
        two_identical_boxes_one_with_one_common_side_rotated(cli.output_path.clone())?,
        two_identical_boxes_with_overlapped_side_and_rotated(cli.output_path.clone())?,
        two_identical_boxes_one_shifted_in_plane(cli.output_path.clone())?,
        two_identical_boxes_one_shifted_in_space(cli.output_path.clone())?,
        bigger_box_extended_by_smaller(cli.output_path.clone())?,
        bigger_box_extended_by_longer(cli.output_path.clone())?,
        smaller_box_cutted_by_bigger(cli.output_path.clone())?,
        smaller_box_cutted_by_bigger_in_two(cli.output_path.clone())?,
        smaller_box_cutted_by_longer(cli.output_path.clone())?,
         */
    ];

    let grid: i32 = 4;
    let grid_size = 5.0;
    let mut scad = Vec::new();
    'outer: for w in 0..grid {
        let x = grid_size * (w as f32 - (grid as f32 / 2.0));
        for h in 0..grid {
            let y = grid_size * (h as f32 - (grid as f32 / 2.0));
            let i = h + (w * grid);
            if let Some(mesh) = meshes.get(i as usize) {
                scad.push(format!("translate(v=[{}, {}, 0]) {{ {} }};", x, y, mesh));
            } else {
                break 'outer;
            }
        }
    }

    let file_content = scad.join("\n");

    fs::write(cli.output_path.join("tot.scad"), file_content).unwrap();

    Ok(())
}
