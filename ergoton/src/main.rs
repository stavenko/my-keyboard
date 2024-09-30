use clap::Parser;
use nalgebra::Vector3;
use num_traits::{One, Zero};
use rust_decimal_macros::dec;

use geometry::{
    decimal::Dec,
    hyper_path::{
        hyper_line::HyperLine,
        hyper_path::{HyperPath, Root},
        hyper_point::SuperPoint,
        split_hyper_line::SplitHyperLine,
    },
    indexes::{
        aabb::Aabb,
        geo_index::{face::FaceId, index::GeoIndex},
    },
    origin::Origin,
};
use keyboard::{
    chok_hotswap::ChokHotswap, Angle, Button, ButtonsCollection, ButtonsColumn, RightKeyboardConfig,
};

mod cli;

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();

    let keyboard = RightKeyboardConfig::build()
        .main(
            ButtonsCollection::build()
                .column(
                    ButtonsColumn::build()
                        //.main_buttons(3, ButtonMountKind::Placeholder)
                        .main_button(Button::chok().build())
                        .main_button(Button::placeholder().build())
                        .main_button(
                            Button::placeholder()
                                .outer_left_top_edge(Vector3::new(
                                    Dec::from(5),
                                    Dec::from(8),
                                    Dec::one(),
                                ))
                                .outer_right_top_edge(Vector3::new(
                                    Dec::one(),
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
                        .main_button(Button::placeholder().build())
                        .main_button(Button::placeholder().build())
                        .main_button(
                            Button::placeholder()
                                .outer_left_top_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(8),
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
                        .main_button(Button::placeholder().build())
                        .main_button(Button::placeholder().build())
                        .main_button(Button::placeholder().build())
                        .add_on_top(
                            Button::placeholder()
                                .additional_padding(Dec::from(2))
                                .depth(Dec::from(5))
                                .incline(Angle::from_deg(Dec::from(-30)))
                                .outer_right_top_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(25),
                                    Dec::one(),
                                ))
                                .outer_left_top_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(25),
                                    Dec::one(),
                                ))
                                .inner_right_top_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(10),
                                    Dec::one(),
                                ))
                                .inner_left_top_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(10),
                                    Dec::one(),
                                ))
                                .build(),
                        )
                        .curvature(Angle::from_deg(Dec::from(10)))
                        // .depth(Dec::from(-3))
                        //.incline(Angle::from_deg(Dec::from(4)))
                        .padding(Dec::from(2))
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_button(Button::placeholder().build())
                        .main_button(Button::placeholder().build())
                        .main_button(
                            Button::placeholder()
                                .outer_left_top_edge(Vector3::new(
                                    Dec::one(),
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
                        .main_button(Button::placeholder().build())
                        .main_button(Button::placeholder().build())
                        .main_button(Button::placeholder().build())
                        .curvature(Angle::from_deg(Dec::from(10)))
                        .incline(Angle::from_deg(Dec::from(10)))
                        .addition_column_padding(Dec::from(5))
                        .padding(Dec::from(2))
                        .build(),
                )
                .padding(Dec::from(20))
                .first_column_angle(Angle::from_deg(Dec::from(30)))
                .plane_pitch(Angle::from_deg(Dec::from(-7)))
                .height(Dec::from(30))
                .curvature(Angle::from_deg(Dec::from(10)))
                .build(),
        )
        .thumb(
            ButtonsCollection::build()
                .column(
                    ButtonsColumn::build()
                        .main_button(Button::placeholder().build())
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_button(Button::placeholder().build())
                        .incline(Angle::from_deg(Dec::from(5)))
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_button(
                            Button::placeholder()
                                .outer_right_bottom_edge(Vector3::new(
                                    Dec::from(10),
                                    Dec::from(8),
                                    Dec::one(),
                                ))
                                .outer_right_top_edge(Vector3::new(
                                    Dec::from(10),
                                    Dec::from(8),
                                    Dec::one(),
                                ))
                                .inner_right_bottom_edge(Vector3::new(
                                    Dec::from(5),
                                    Dec::from(8),
                                    Dec::one(),
                                ))
                                .inner_right_top_edge(Vector3::new(
                                    Dec::from(5),
                                    Dec::from(8),
                                    Dec::one(),
                                ))
                                .build(),
                        )
                        .incline(Angle::from_deg(Dec::from(10)))
                        .build(),
                )
                .height(Dec::from(13))
                .padding(Dec::from(22))
                .position_shift_x(Dec::from(-35))
                .position_shift_y(Dec::from(-20))
                .first_column_angle(Angle::from_deg(Dec::from(-85)))
                .curvature(Angle::from_deg(Dec::from(-9)))
                .plane_pitch(Angle::from_deg(Dec::from(25)))
                .plane_yaw(Angle::from_deg(Dec::from(-15)))
                .build(),
        )
        .wall_thickness(Dec::from(4))
        .table_outline(
            Root::new() // right
                .extend(
                    HyperLine::new_2(
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(20),
                            point: Vector3::new(Dec::from(100), Dec::from(30), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(20),
                            point: Vector3::new(Dec::from(100), -Dec::from(25), Dec::zero()),
                        },
                    )
                    .split_by_weights(vec![
                        Dec::from(dec!(1)),
                        dec!(0.5).into(),
                        dec!(1).into(),
                        dec!(0.5).into(),
                        dec!(1).into(),
                    ]),
                )
                .push_back(HyperLine::new_2(
                    // corner
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(20),
                        point: Vector3::new(Dec::from(100), -Dec::from(25), Dec::zero()),
                    },
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(20),
                        point: Vector3::new(Dec::from(90), -Dec::from(30), Dec::zero()),
                    },
                ))
                .extend(
                    HyperLine::new_2(
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(20),
                            point: Vector3::new(Dec::from(90), -Dec::from(30), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(1),
                            point: Vector3::new(-Dec::from(20), -Dec::from(34), Dec::zero()),
                        },
                    )
                    .split_by_weights(
                        [
                            Dec::from(2),
                            Dec::from(dec!(0.5)),
                            Dec::from(2),
                            Dec::from(dec!(0.8)),
                            Dec::from(2),
                            Dec::from(dec!(0.9)),
                            Dec::from(3),
                            Dec::from(dec!(0.5)),
                            Dec::from(2),
                            Dec::from(3),
                            Dec::from(1),
                            Dec::from(dec!(0.5)),
                            Dec::from(1),
                            Dec::from(dec!(0.5)),
                            Dec::from(1),
                        ]
                        .to_vec(),
                    ),
                )
                .push_back(HyperLine::new_2(
                    // corner
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(1),
                        point: Vector3::new(-Dec::from(20), -Dec::from(34), Dec::zero()),
                    },
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(dec!(0.01)),
                        point: Vector3::new(-Dec::from(31), -Dec::from(24), Dec::zero()),
                    },
                ))
                .push_back(HyperLine::new_2(
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(dec!(0.01)),
                        point: Vector3::new(-Dec::from(31), -Dec::from(24), Dec::zero()),
                    },
                    SuperPoint {
                        side_dir: Vector3::z() * Dec::from(dec!(0.01)),
                        point: Vector3::new(-Dec::from(39), -Dec::from(10), Dec::zero()),
                    },
                ))
                .extend(
                    HyperLine::new_4(
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(dec!(0.01)),
                            point: Vector3::new(-Dec::from(39), -Dec::from(10), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(10),
                            point: Vector3::new(-Dec::from(39), Dec::from(30), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(5),
                            point: Vector3::new(Dec::from(80), Dec::from(50), Dec::zero()),
                        },
                        SuperPoint {
                            side_dir: Vector3::z() * Dec::from(20),
                            point: Vector3::new(Dec::from(100), Dec::from(30), Dec::zero()),
                        },
                    )
                    .split_by_weights(
                        [
                            Dec::from(dec!(0.5)),
                            Dec::from(dec!(0.5)),
                            Dec::from(1),
                            Dec::from(1),
                            Dec::from(dec!(0.5)),
                            Dec::from(1),
                            Dec::from(dec!(0.1)),
                            Dec::from(1),
                            Dec::from(dec!(0.5)),
                            Dec::from(1),
                            Dec::from(dec!(0.5)),
                            Dec::from(5),
                            Dec::from(dec!(0.5)),
                            Dec::from(2),
                            Dec::from(dec!(0.5)),
                            Dec::from(3),
                            Dec::from(dec!(0.5)),
                        ]
                        .to_vec(),
                    ),
                ),
        )
        .build();

    let mut buttons_hull = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-100), Dec::from(-100), Dec::from(-100)),
        Vector3::new(Dec::from(105), Dec::from(100), Dec::from(100)),
    ]))
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));
    /*

    let some_basis = PolygonBasis {
        center: Vector3::new(
            //
            dec!(22.82445866667124051206245548).into(),
            dec!(-19.779570592522108815492239509).into(),
            dec!(22.15891213079591064257514847).into(),
        ),
        //x
        x: Vector3::new(
            dec!(-0.8374019817599564937384033586).into(),
            dec!(0.5312522329518458348708607207).into(),
            dec!(0.1285651038508320043182247472).into(),
        ),
        //y
        y: Vector3::new(
            dec!(0.4412129513248363290765072231).into(),
            dec!(0.7958346018056754586727604878).into(),
            dec!(-0.4147028070221246913099825422).into(),
        ),
    };
    */

    buttons_hull.face_debug(23, Some(FaceId(23)));

    keyboard.buttons_hull(&mut buttons_hull).unwrap();
    /*

    let main_button_hull_scad_path = cli.output_path.join("main_button_hull.scad");
    let scad = buttons_hull.scad();

    std::fs::write(main_button_hull_scad_path, scad)?;
    */

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

    let mut chok_hotswap_mount = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-20), Dec::from(-20), Dec::from(-20)),
        Vector3::new(Dec::from(20), Dec::from(20), Dec::from(20)),
    ]))
    .debug_svg_path(cli.output_path.clone())
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    let chok = ChokHotswap::new();
    /*
    let xy = PolygonBasis {
        center: Vector3::zeros(),
        x: Vector3::x(),
        y: Vector3::y(),
    };
    let xz = PolygonBasis {
        center: Vector3::zeros(),
        x: Vector3::x(),
        y: Vector3::z(),
    };

    let yz = PolygonBasis {
        center: Vector3::zeros(),
        x: Vector3::y(),
        y: Vector3::z(),
    };
    */

    //chok_hotswap_top.poly_split_debug(64, xy.clone());
    //chok_hotswap_top.poly_split_debug(67, xy.clone());
    //chok_hotswap_top.poly_split_debug(911, xz.clone());
    //chok_hotswap_top.poly_split_debug(913, xz);

    chok_hotswap_top.face_debug(333, None);
    chok.top_mesh(&mut chok_hotswap_top)?;
    chok.bottom_mesh(&mut chok_hotswap_bottom)?;
    chok.outer_mount(Origin::new(), &mut chok_hotswap_mount)?;

    let scad_path_all = cli.output_path.join("chok_hotswap_all.scad");
    let scad_path_top = cli.output_path.join("chok_hotswap_top.scad");
    let scad_path_bottom = cli.output_path.join("chok_hotswap_bottom.scad");
    let scad_path_mount = cli.output_path.join("chok_hotswap_mount.scad");
    let scad = chok_hotswap_top.scad();
    let top = format!("translate(v=[0, 0, 0]) {{ {scad} }};");
    let scad = chok_hotswap_bottom.scad();
    let bottom = format!("translate(v=[0, 0, 0]) {{ {scad} }};");
    let scad = chok_hotswap_mount.scad();
    let mount = format!("translate(v=[0, 0, 7]) {{ {scad} }};");

    let totals = format!("{top}\n{bottom}\n{mount}");

    std::fs::write(scad_path_all, totals)?;
    std::fs::write(scad_path_top, top)?;
    std::fs::write(scad_path_bottom, bottom)?;
    std::fs::write(scad_path_mount, mount)?;

    Ok(())
}
