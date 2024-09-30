use nalgebra::Vector3;
use num_traits::{One, Zero};
use rust_decimal_macros::dec;
use std::fs::OpenOptions;

use clap::Parser;

use geometry::{
    decimal::Dec,
    hyper_path::{
        hyper_line::HyperLine,
        hyper_path::{HyperPath, Root},
        hyper_point::SuperPoint,
        split_hyper_line::SplitHyperLine,
    },
    indexes::{aabb::Aabb, geo_index::index::GeoIndex},
    origin::Origin,
    polygon_basis::PolygonBasis,
    shapes::Cylinder,
};
use keyboard::{
    chok_hotswap::{self, ChokHotswap},
    Angle, Bolt, BoltPoint, Button, ButtonsCollection, ButtonsColumn, Hole, KeyboardMesh,
    RightKeyboardConfig,
};

mod cli;

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();

    let m2_10_nut = Bolt::build()
        .m2()
        .head_height(Dec::from(1.2))
        .head_diameter(Dec::from(4.2))
        .height(Dec::from(10))
        .build();

    let m2_10 = Bolt::build()
        .m2()
        .head_height(dec!(1.2))
        .head_diameter(Dec::from(4))
        .height(dec!(10) - dec!(1.2))
        .thread_inner_diameter(dec!(1.2))
        .no_nut()
        .build();

    let m1_8 = Bolt::build()
        .m1_no_nut()
        .head_height(Dec::from(0.8))
        .head_diameter(Dec::from(1.6))
        .height(Dec::from(8))
        .build();

    let keyboard = RightKeyboardConfig::build()
        .wall_thickness(4)
        .bottom_thickness(2)
        /*
        .add_bolt(
            KeyboardMesh::ButtonsHull,
            KeyboardMesh::Bottom,
            BoltPoint::new(m2_10_nut.clone())
                .head_thread_material_gap(2)
                .radial_head_material_extention(dec!(1))
                .origin(
                    Origin::new()
                        .offset_x(-15)
                        .offset_y(dec!(3))
                        .offset_z(dec!(15))
                        .rotate_axisangle(
                            Vector3::new(Dec::one(), Dec::one(), Dec::zero()).normalize()
                                * Angle::from_deg(-75).rad(),
                        )
                        .rotate_axisangle(Vector3::x() * Angle::from_deg(30).rad())
                        .offset_y(5)
                        .offset_z(-10),
                ),
        )
        .add_bolt(
            KeyboardMesh::ButtonsHull,
            KeyboardMesh::Bottom,
            BoltPoint::new(m2_10)
                .thread_hole_radius_plastic_modification(dec!(1.6))
                .radial_head_material_extention(dec!(1))
                .head_thread_material_gap(4)
                .origin(
                    Origin::new()
                        .offset_x(16)
                        .offset_y(-dec!(12.5))
                        .offset_z(dec!(0.4))
                        .rotate_axisangle(Vector3::x() * Angle::from_deg(10).rad())
                        .offset_z(dec!(2.5)),
                ),
        )
        .add_bolt(
            KeyboardMesh::ButtonsHull,
            KeyboardMesh::Bottom,
            BoltPoint::new(m1_8)
                .radial_head_material_extention(dec!(1))
                .thread_hole_radius_plastic_modification(1.8)
                .head_thread_material_gap(4)
                .origin(
                    Origin::new()
                        .offset_x(15)
                        .offset_y(dec!(11.5))
                        .offset_z(dec!(0.4))
                        .rotate_axisangle(Vector3::x() * Angle::from_deg(-15).rad())
                        .offset_y(dec!(-1) + dec!(0.6))
                        .offset_z(2),
                ),
        )
        */
        .main(
            ButtonsCollection::build()
                .column(
                    ButtonsColumn::build()
                        .main_button(
                            Button::chok_hotswap_custom()
                                .outer_left_bottom_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(2),
                                    Dec::one(),
                                ))
                                .build(),
                        )
                        .main_button(
                            Button::chok_hotswap_custom()
                                .outer_right_top_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(10),
                                    Dec::one(),
                                ))
                                .outer_left_top_edge(Vector3::new(
                                    Dec::from(10),
                                    Dec::from(10),
                                    Dec::one(),
                                ))
                                .build(),
                        )
                        .curvature(Angle::from_deg(Dec::from(10)))
                        .padding(Dec::from(2))
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_button(
                            Button::chok_hotswap_custom()
                                .outer_left_top_edge(Vector3::new(
                                    Dec::from(2),
                                    Dec::from(2),
                                    Dec::one(),
                                ))
                                .build(),
                        )
                        .curvature(Angle::from_deg(Dec::from(10)))
                        .padding(Dec::from(2))
                        .build(),
                )
                .padding(25)
                .first_column_angle(Angle::from_deg(30))
                .plane_pitch(Angle::from_deg(-7))
                .height(20)
                .curvature(Angle::from_deg(Dec::from(10)))
                .build(),
        )
        .thumb(
            ButtonsCollection::build()
                .column(
                    ButtonsColumn::build()
                        .main_button(
                            Button::chok_hotswap_custom()
                                .outer_left_top_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(10),
                                    Dec::one(),
                                ))
                                .outer_right_top_edge(Vector3::new(
                                    Dec::from(10),
                                    Dec::from(10),
                                    Dec::one(),
                                ))
                                .outer_right_bottom_edge(Vector3::new(
                                    Dec::from(2),
                                    Dec::from(2),
                                    Dec::one(),
                                ))
                                .build(),
                        )
                        .build(),
                )
                .height(Dec::from(15))
                .padding(Dec::from(18))
                .position_shift_x(Dec::from(-20))
                .position_shift_y(Dec::from(-15))
                .first_column_angle(Angle::from_deg(Dec::from(-85)))
                .curvature(Angle::from_deg(Dec::from(-9)))
                .plane_pitch(Angle::from_deg(Dec::from(25)))
                .plane_yaw(Angle::from_deg(Dec::from(-15)))
                .build(),
        )
        .table_outline(
            Root::new() // right
                .push_back(HyperLine::new_2(
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(1),
                        point: Vector3::new(Dec::from(30), Dec::from(10), Dec::zero()),
                    },
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(1),
                        point: Vector3::new(Dec::from(30), -Dec::from(10), Dec::zero()),
                    },
                ))
                .push_back(HyperLine::new_2(
                    // corner
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(1),
                        point: Vector3::new(Dec::from(30), -Dec::from(10), Dec::zero()),
                    },
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(1),
                        point: Vector3::new(Dec::from(20), -Dec::from(15), Dec::zero()),
                    },
                ))
                .extend(
                    HyperLine::new_2(
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(1),
                            point: Vector3::new(Dec::from(20), -Dec::from(15), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(1),
                            point: Vector3::new(-Dec::from(5), -Dec::from(30), Dec::zero()),
                        },
                    )
                    .split_by_weights([Dec::from(2), Dec::from(dec!(0.5)), Dec::from(2)].to_vec()),
                )
                .push_back(HyperLine::new_2(
                    // corner
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(1),
                        point: Vector3::new(-Dec::from(5), -Dec::from(30), Dec::zero()),
                    },
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(dec!(0.01)),
                        point: Vector3::new(-Dec::from(10), -Dec::from(24), Dec::zero()),
                    },
                ))
                .extend(
                    HyperLine::new_2(
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(dec!(0.01)),
                            point: Vector3::new(-Dec::from(10), -Dec::from(24), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(dec!(0.01)),
                            point: Vector3::new(-Dec::from(12), -Dec::from(10), Dec::zero()),
                        },
                    )
                    .split_by([Dec::from(dec!(0.1)), Dec::from(dec!(0.2))].to_vec()),
                )
                .extend(
                    HyperLine::new_4(
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(dec!(0.01)),
                            point: Vector3::new(-Dec::from(12), -Dec::from(10), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(3),
                            point: Vector3::new(-Dec::from(12), Dec::from(20), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(2),
                            point: Vector3::new(Dec::from(10), Dec::from(20), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(1),
                            point: Vector3::new(Dec::from(30), Dec::from(10), Dec::zero()),
                        },
                    )
                    .split_by_weights(
                        [
                            Dec::from(dec!(1)),
                            Dec::from(dec!(10)),
                            Dec::from(dec!(10)),
                            Dec::from(dec!(10)),
                            Dec::from(dec!(10)),
                            Dec::from(dec!(10)),
                            Dec::from(dec!(10)),
                        ]
                        .to_vec(),
                    ),
                ),
        )
        /*
        .add_main_hole(
            Hole::build()
                .shape(
                    Cylinder::with_top_at(
                        Origin::new()
                            .offset_y(-25)
                            .offset_z(15)
                            .rotate_axisangle(Vector3::x() * Angle::from_deg(67).rad())
                            .rotate_axisangle(Vector3::y() * Angle::from_deg(20).rad())
                            .offset_z(1),
                        5,
                        4,
                    )
                    .top_cap(false)
                    .bottom_cap(false)
                    .steps(16),
                )
                .build()?,
        )
        .add_main_hole(
            Hole::build()
                .shape(
                    Cylinder::with_top_at(
                        Origin::new()
                            .offset_y(-25)
                            .offset_z(15)
                            .rotate_axisangle(Vector3::x() * Angle::from_deg(67).rad())
                            .rotate_axisangle(Vector3::y() * Angle::from_deg(20).rad())
                            //.offset_y(-4)
                            .offset_z(-1),
                        5,
                        5.5,
                    )
                    //.top_cap(false)
                    .bottom_cap(false)
                    .steps(6),
                )
                .build()?,
        )
            */
        .build();

    std::fs::create_dir_all(&cli.output_path)?;
    println!("create main");
    let mut main = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-51), Dec::from(-51), Dec::from(-51)),
        Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
    ]))
    .debug_svg_path(cli.output_path.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let mut chok_hotswap_top = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-15), Dec::from(-15), Dec::from(-15)),
        Vector3::new(Dec::from(15), Dec::from(15), Dec::from(16)),
    ]))
    .debug_svg_path(cli.output_path.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let mut chok_hotswap_bottom = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-15), Dec::from(-15), Dec::from(-10)),
        Vector3::new(Dec::from(15), Dec::from(15), Dec::from(16)),
    ]))
    .debug_svg_path(cli.output_path.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    /*
    let some_basis = PolygonBasis {
        center: Vector3::new(
            dec!(-1.5543950009929657439721960749).into(),
            dec!(-23.402748182863959638629538724).into(),
            dec!(16.084464323260701317707916789).into(),
        ),
        x: Vector3::new(
            dec!(0.5579776416337965548034479667).into(),
            dec!(-0.6844767094290788649763709834).into(),
            dec!(0.4692042046763081230933800416).into(),
        ),
        y: Vector3::new(
            dec!(-0.6450649795271313430775016103).into(),
            dec!(-0.7134427578111101333039070128).into(),
            dec!(-0.2736614761243154445197904833).into(),
        ),
    };

    let mut shifted_basis = some_basis.clone();
    shifted_basis.center += some_basis.x * Dec::from(5);
    */

    //main.face_debug(3389, some_basis);

    //main.face_debug(2295, None);

    let _bottom = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-50), Dec::from(-50), Dec::from(-50)),
        Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
    ]))
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    keyboard.buttons_hull(&mut main).unwrap();
    println!("create bottom");
    //keyboard.bottom_pad(&mut bottom).unwrap();
    let chok = ChokHotswap::new();
    chok.top_mesh(&mut chok_hotswap_top)?;
    chok.bottom_mesh(&mut chok_hotswap_bottom)?;

    //let scad_path_all = cli.output_path.join("main_all.scad");
    let main_all = cli.output_path.join("main.scad");
    let chok_hw_top = cli.output_path.join("chok_hw_top.scad");
    let chok_hw_bottom = cli.output_path.join("chok_hw_bottom.scad");

    let scad = main.scad();
    let button_hull = format!("translate(v=[0, 0, 0]) {{ {scad} }};");
    let scad = chok_hotswap_top.scad();
    let chok_hotswap_top = format!("translate(v=[0, 0, 0]) {{ {scad} }};");
    let scad = chok_hotswap_bottom.scad();
    let chok_hotswap_bottom = format!("translate(v=[0, 0, 0]) {{ {scad} }};");
    std::fs::write(main_all, button_hull)?;
    std::fs::write(chok_hw_top, chok_hotswap_top)?;
    std::fs::write(chok_hw_bottom, chok_hotswap_bottom)?;

    Ok(())
}
