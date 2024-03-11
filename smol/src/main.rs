use std::fs::OpenOptions;

use clap::Parser;

use keyboard::bolt_builder::Bolt;

mod cli;

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();

    let _m2_10 = Bolt {
        nut: Some(keyboard::bolt_builder::Nut::hex_with_inner_diameter(4.0)),
        diameter: 2.0,
        height: 10.0,
        head_diameter: 4.0,
        head_height: 1.0,
    };

    let keyboard = keyboard::KeyboardConfig::simple();

    let wall = keyboard.build_total_wall()?;

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(cli.output_path)?;

    stl_io::write_stl(&mut writer, wall.into_iter())?;

    Ok(())
}
