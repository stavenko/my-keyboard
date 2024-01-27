use std::{fs::OpenOptions, path};

use clap::Parser;
use geometry::primitives::{Face, TriangleWrap};
use nalgebra::Vector3;
use scad::ScadFile;

use crate::keyboard::bolt_builder::{Bolt, BoltBuilder, BoltPlace};

mod cli;
mod geometry;
mod keyboard;
mod utils;

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();

    let m2_10 = Bolt {
        nut: Some(keyboard::bolt_builder::Nut::hex_with_inner_diameter(4.0)),
        diameter: 2.0,
        height: 10.0,
        head_diameter: 4.0,
        head_height: 1.0,
    };

    let bb = BoltBuilder::new().add_bolt(BoltPlace {
        position: Vector3::new(0.5, 35.0, 10.0),
        bolt: m2_10,
        nut_depth: 0.2,
    });
    // let mut scad_file = ScadFile::new();

    let keyboard = keyboard::KeyboardConfig::simple();

    /*
    for button in keyboard.buttons() {
        scad_file.add_object(button.scad(2.0)?);
    }

    scad_file.add_object(keyboard.thumb_right_to_main_left_2()?);


    for s in keyboard
        .between_buttons_in_columns()
        .chain(keyboard.between_columns_main())
        .chain(keyboard.between_columns_thumb())
    {
        scad_file.add_object(s);
    }
    */
    // let base = keyboard.bottom_base()?;
    let wall = keyboard.build_total_wall()?;
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .open(cli.output_path)?;

    stl_io::write_stl(&mut writer, wall.into_iter())?;

    //let base_with_nuts = bb.with_nut(base);
    //let wall = bb.with_head(wall);

    // scad_file.add_object(base_with_nuts);
    // scad_file.add_object(wall_with_bolts);
    //scad_file.add_object(wall);
    //println!("Done");

    /*
    //Create an scad object
    let mut cube = scad!(Translate(vec3(2.0, 2.0, 3.0)); {
        scad!(Cube(vec3(2.0,1.0,4.0)))
    });

    //Create a cylinder with a height of 10 and a diameter of 3 mm
    let cylinder = scad!(Cylinder(10., Diameter(3.)));

    //Add the cylinder to the cubes translation.
    cube.add_child(cylinder);

    //Add the cube object to the file
    scad_file.add_object(cube.clone());
    */

    //Save the scad code to a file
    // scad_file.write_to_file(cli.output_path);

    Ok(())
}
