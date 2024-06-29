use nalgebra::Vector3;
use num_traits::{One, Zero};
use rust_decimal_macros::dec;
use std::fs::OpenOptions;

use clap::Parser;

use geometry::{
    basis::Basis,
    decimal::Dec,
    hyper_path::{
        hyper_line::HyperLine,
        hyper_path::{HyperPath, Root},
        hyper_point::SuperPoint,
        split_hyper_line::SplitHyperLine,
    },
    indexes::geo_index::index::GeoIndex,
    origin::Origin,
};
use keyboard::{
    Angle, Bolt, BoltPoint, Button, ButtonsCollection, ButtonsColumn, RightKeyboardConfig,
};

mod cli;

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();

    let m2_10 = Bolt::new()
        .m2()
        .head_height(Dec::from(2))
        .head_diameter(Dec::from(4))
        .height(Dec::from(10))
        .build();

    let keyboard = RightKeyboardConfig::build()
        .add_bolt(
            BoltPoint::new(m2_10.clone())
                .head_up_extension(10)
                .thread_material_gap(dec!(0.5))
                .radial_head_extension(dec!(0.7))
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
                        .offset_z(-5),
                ),
        )
        .add_bolt(
            BoltPoint::new(m2_10.clone())
                .head_up_extension(10)
                .thread_material_gap(dec!(0.5))
                .radial_head_extension(dec!(0.7))
                .origin(
                    Origin::new()
                        .offset_x(15)
                        .offset_y(-dec!(11.5))
                        .offset_z(dec!(0.4))
                        .rotate_axisangle(Vector3::x() * Angle::from_deg(10).rad()),
                ),
        )
        .add_bolt(
            BoltPoint::new(m2_10.clone())
                .head_up_extension(10)
                .thread_material_gap(dec!(0.5))
                .radial_head_extension(dec!(0.7))
                .origin(
                    Origin::new()
                        .offset_x(15)
                        .offset_y(dec!(11.5))
                        .offset_z(dec!(0.4))
                        .rotate_axisangle(Vector3::x() * Angle::from_deg(-10).rad()),
                ),
        )
        .main(
            ButtonsCollection::build()
                .column(
                    ButtonsColumn::build()
                        .main_button(
                            Button::chok()
                                .outer_left_bottom_edge(Vector3::new(
                                    Dec::one(),
                                    Dec::from(2),
                                    Dec::one(),
                                ))
                                .build(),
                        )
                        .main_button(
                            Button::chok()
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
                            Button::chok()
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
                .padding(Dec::from(20))
                .first_column_angle(Angle::from_deg(Dec::from(30)))
                .plane_pitch(Angle::from_deg(Dec::from(-7)))
                .height(Dec::from(14))
                .curvature(Angle::from_deg(Dec::from(10)))
                .build(),
        )
        .thumb(
            ButtonsCollection::build()
                .column(
                    ButtonsColumn::build()
                        .main_button(
                            Button::chok()
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
        .wall_thickness(Dec::from(4))
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
        .build();

    let mut index = GeoIndex::new();

    keyboard.buttons_hull(&mut index).unwrap();

    let o = Origin::new()
        .offset_x(Dec::from(20))
        .offset_y(Dec::from(10));

    let x = o.x();
    let o = o.rotate_axisangle(x * Dec::from(14));

    let b = Basis::new(o.x(), o.y(), o.z(), o.center).unwrap();

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(cli.output_path)?;

    stl_io::write_stl(&mut writer, index.into_iter())?;

    Ok(())
}
