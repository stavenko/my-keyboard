use std::fs::OpenOptions;

use clap::Parser;
use rust_decimal::Decimal;

#[derive(Parser)]
pub struct Command {
    #[arg(long)]
    pub output_path: String,
}

pub fn box(b: Basis2d, w: Decimal, h: Decimal, d: Decimal )  -> Mesh {
    todo!()
}

fn main() -> Result<(), anyhow::Error> {
    let cli = cli::Command::parse();
    println!("hello geometry");
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .open(cli.output_path)?;
    stl_io::write_stl(&mut writer, mesh.into_iter())?;

    Ok(())
}
