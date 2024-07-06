use std::fs::OpenOptions;

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
    indexes::{aabb::Aabb, geo_index::index::GeoIndex},
};
use keyboard::{Angle, Button, ButtonsCollection, ButtonsColumn, RightKeyboardConfig};

mod cli;

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();

    /*
    let _m2_10 = Bolt {
        nut: Some(keyboard::bolt_builder::Nut::hex_with_inner_diameter(4.0)),
        diameter: 2.0,
        height: 10.0,
        head_diameter: 4.0,
        head_height: 1.0,
    };
    */

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
                                    Dec::from(5),
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
                                    Dec::from(15),
                                    Dec::one(),
                                ))
                                .outer_left_top_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(10),
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
                        .addition_column_padding(Dec::from(4))
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
                                    Dec::from(5),
                                    Dec::from(8),
                                    Dec::one(),
                                ))
                                .outer_right_top_edge(Vector3::new(
                                    Dec::from(5),
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
                .height(Dec::from(10))
                .padding(Dec::from(18))
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

    let mut index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-50), Dec::from(-50), Dec::from(-50)),
        Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
    ]));
    keyboard.buttons_hull(&mut index).unwrap();

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(cli.output_path)?;

    stl_io::write_stl(&mut writer, index.into_iter())?;

    Ok(())
}
