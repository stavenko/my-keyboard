use nalgebra::Vector3;
use num_traits::{One, Zero};
use rust_decimal_macros::dec;
use std::{borrow::Cow, collections::HashSet, fs::OpenOptions};

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
    indexes::geo_index::{index::GeoIndex, mesh},
    origin::Origin,
    shapes,
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

    let mut index = GeoIndex::default();

    keyboard.inner_wall_surface(&mut index).unwrap();

    keyboard.outer_wall_surface(&mut index).unwrap();

    keyboard.buttons(&mut index).unwrap();

    keyboard.fill_between_buttons(&mut index).unwrap();

    keyboard
        .inner_outer_surface_table_connection(&mut index)
        .unwrap();

    let o = Origin::new()
        .offset_x(Dec::from(20))
        .offset_y(Dec::from(10));

    let x = o.x();
    let o = o.rotate_axisangle(x * Dec::from(14));

    let b = Basis::new(o.x(), o.y(), o.z(), o.center).unwrap();

    let mesh_id = index.save_mesh(
        shapes::cylinder(b, Dec::from(15), Dec::from(2), 10)
            .into_iter()
            .map(Cow::Owned),
    );

    let mm = *index.collect_meshes()[0];

    {
        let mutm = index.get_mutable_mesh(mm);
        if let Err((p, pp)) = mutm.boolean_diff(mesh_id) {
            dbg!("~~~~", index.load_polygon_ref(p));
            dbg!("~~~~", index.load_polygon_ref(pp));
            index.flip_polygon(p);
            index.flip_polygon(pp);
        }
    }
    let m = index.get_mesh(mm);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(cli.output_path)?;

    stl_io::write_stl(&mut writer, index.into_iter())?;

    Ok(())
}
