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
};
use keyboard::{Angle, Button, ButtonsCollection, ButtonsColumn, RightKeyboardConfig};

mod cli;

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();

    let _keyboard = RightKeyboardConfig::build()
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

    let index = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-50), Dec::from(-50), Dec::from(-50)),
        Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
    ]));

    #[rustfmt::skip]
    let _p1: &[Vector3<Dec>] = &[
    Vector3::new(dec!(20.6180339887).into(), dec!(12.4559581815).into(), dec!(3.8817317944).into()),
    Vector3::new(dec!(20.6180339887).into(), dec!(12.4560890688).into(), dec!(3.8826800212).into()),
    Vector3::new(dec!(20.6180339887).into(), dec!(12.9097762979).into(), dec!(7.1694655225).into()),
    Vector3::new(dec!(21.6180339887).into(), dec!(12.1900579255).into(), dec!(7.2688109267).into()),
    Vector3::new(dec!(21.6180339887).into(), dec!(11.7365480016).into(), dec!(3.9833099320).into()),
    Vector3::new(dec!(20.9079214406).into(), dec!(12.2474101976).into(), dec!(3.9111780219).into()),
    Vector3::new(dec!(20.7607867695).into(), dec!(12.3532603778).into(), dec!(3.8962323560).into()),

    ];

    #[rustfmt::skip]
    let _p2: &[Vector3<Dec>] = &[
    Vector3::new(dec!(18.6975304844).into(), dec!(14.1709823847).into(), dec!(3.5093506678).into() ),
    Vector3::new(dec!(18.8115659456).into(), dec!(13.7573368844).into(), dec!(3.7111817178).into() ),
    Vector3::new(dec!(20.6180339887).into(), dec!(12.4560890688).into(), dec!(3.8826800212).into() ),
    Vector3::new(dec!(20.7607867695).into(), dec!(12.3532603778).into(), dec!(3.8962323560).into() ),
    ];

    //let p1 = index.save_as_polygon(p1).unwrap();
    //let p2 = index.save_as_polygon(p2).unwrap();

    /*
         * [smol/src/main.rs:275:13] index.load_polygon_ref(p) = 20.6180339887 12.4559581815 3.8817317944 -> 20.6180339887 12.4560890688 3.8826800212
    20.6180339887 12.4560890688 3.8826800212 -> 20.6180339887 12.9097762979 7.1694655225
    20.6180339887 12.9097762979 7.1694655225 -> 21.6180339887 12.1900579255 7.2688109267
    21.6180339887 12.1900579255 7.2688109267 -> 21.6180339887 11.7365480016 3.9833099320
    21.6180339887 11.7365480016 3.9833099320 -> 20.9079214406 12.2474101976 3.9111780219
    20.9079214406 12.2474101976 3.9111780219 -> 20.7607867695 12.3532603778 3.8962323560
    20.7607867695 12.3532603778 3.8962323560 -> 20.6180339887 12.4559581815 3.8817317944
    [smol/src/main.rs:276:13] "~~~~" = "~~~~"
    [smol/src/main.rs:276:13] index.load_polygon_ref(pp) =
    18.6975304844 14.1709823847 3.5093506678 -> 18.8115659456 13.7573368844 3.7111817178
    18.8115659456 13.7573368844 3.7111817178 -> 20.6180339887 12.4560890688 3.8826800212
    20.6180339887 12.4560890688 3.8826800212 -> 20.7607867695 12.3532603778 3.8962323560
    20.7607867695 12.3532603778 3.8962323560 -> 18.6975304844 14.1709823847 3.5093506678
    [geometry/src/indexes/geo_index/index.rs:2084:20] triangles.len() = 3102
         * */

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(cli.output_path)?;

    Ok(())
}
