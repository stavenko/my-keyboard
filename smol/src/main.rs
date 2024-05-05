use num_traits::{One, Zero};
use std::fs::OpenOptions;

use clap::Parser;

use geometry::{decimal::Dec, indexes::geo_index::index::GeoIndex};
use keyboard::{Angle, ButtonMountKind, ButtonsCollection, ButtonsColumn, RightKeyboardConfig};

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
                        .main_buttons(3, ButtonMountKind::Placeholder)
                        .curvature(Angle::from_deg(Dec::from(10)))
                        .padding(Dec::from(2))
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_buttons(3, ButtonMountKind::Placeholder)
                        .curvature(Angle::from_deg(Dec::from(10)))
                        .padding(Dec::from(2))
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_buttons(3, ButtonMountKind::Placeholder)
                        .add_on_top(
                            ButtonMountKind::Placeholder,
                            Zero::zero(),
                            Zero::zero(),
                            Zero::zero(),
                        )
                        .curvature(Angle::from_deg(Dec::from(10)))
                        // .depth(Dec::from(-3))
                        //.incline(Angle::from_deg(Dec::from(4)))
                        .padding(Dec::from(2))
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_buttons(3, ButtonMountKind::Placeholder)
                        .curvature(Angle::from_deg(Dec::from(10)))
                        .padding(Dec::from(2))
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_buttons(3, ButtonMountKind::Placeholder)
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
                        .main_buttons(1, ButtonMountKind::Placeholder)
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_buttons(1, ButtonMountKind::Placeholder)
                        .incline(Angle::from_deg(Dec::from(5)))
                        .build(),
                )
                .column(
                    ButtonsColumn::build()
                        .main_buttons(1, ButtonMountKind::Placeholder)
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
        .build();

    let mut index = GeoIndex::default();

    keyboard.buttons(&mut index).unwrap();

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(cli.output_path)?;

    stl_io::write_stl(&mut writer, index.into_iter())?;

    Ok(())
}
