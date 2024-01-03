use clap::Parser;
use scad::ScadFile;

mod cli;
mod geometry;
mod keyboard;
mod utils;

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();

    let mut scad_file = ScadFile::new();
    scad_file.set_detail(50);
    let keyboard = keyboard::KeyboardConfig::simple();

    for button in keyboard.buttons() {
        println!("--");
        scad_file.add_object(button.scad(2.0)?);
    }

    //scad_file.add_object(keyboard.right_wall()?);
    //scad_file.add_object(keyboard.top_wall()?);
    //scad_file.add_object(keyboard.bottom_wall()?);
    //scad_file.add_object(keyboard.thumb_bottom_wall()?);
    //scad_file.add_object(keyboard.thumb_left_wall()?);
    scad_file.add_object(keyboard.thumb_right_to_main_left_2()?);
    //scad_file.add_object(keyboard.thumb_top_wall()?);
    scad_file.add_object(keyboard.build_total_wall()?);

    println!("Done");

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
    scad_file.write_to_file(cli.output_path);

    Ok(())
}
