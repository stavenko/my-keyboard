use chok_hotswap::ChokHotswap;
use nalgebra::Vector3;
use rust_decimal_macros::dec;
use std::fs::OpenOptions;

use clap::Parser;

use geometry::{
    decimal::Dec,
    indexes::{aabb::Aabb, geo_index::index::GeoIndex},
};

mod chok_hotswap;
mod cli;

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();
    let chok = ChokHotswap;

    println!("create top");
    let mut top = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-50), Dec::from(-50), Dec::from(-50)),
        Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
    ]))
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));

    chok.top_mesh(&mut top).unwrap();

    let main_button_hull_path = cli.output_path.join("chok_hotswap_top.stl");

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(main_button_hull_path)?;

    println!("create bottom");
    let mut bottom = GeoIndex::new(Aabb::from_points(&[
        Vector3::new(Dec::from(-50), Dec::from(-50), Dec::from(-50)),
        Vector3::new(Dec::from(50), Dec::from(50), Dec::from(50)),
    ]))
    .input_polygon_min_rib_length(dec!(0.05))
    .points_precision(dec!(0.001));
    chok.bottom_mesh(&mut bottom).unwrap();
    let bottom_pad_path = cli.output_path.join("chok_hotswap_bottom.stl");
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(bottom_pad_path)?;
    // TODO: Make scad file

    Ok(())
}
