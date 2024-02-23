use std::{
    fs::{self, OpenOptions},
    path::PathBuf,
};

use clap::Parser;
use geometry::{
    primitives::{basis::Basis, decimal::Dec},
    shapes,
};
use nalgebra::Vector3;
use num_traits::One;

#[derive(Parser)]
pub struct Command {
    #[arg(long)]
    pub output_path: PathBuf,
}

fn bigger_by_smaller(file_root: PathBuf) -> anyhow::Result<()> {
    let zz = Vector3::z();
    let yy = Vector3::y();
    let xx = yy.cross(&zz);

    let _zero_basis = Basis::new(xx, yy, zz, Vector3::zeros())?;

    let smaller_box = shapes::rect(
        _zero_basis.clone(),
        Dec::one() * 1,
        Dec::one() * 1,
        Dec::one() * 1,
    );

    let bigger_box = shapes::rect(_zero_basis, Dec::one() * 2, Dec::one() * 2, Dec::one() * 2);

    let result = bigger_box.boolean_union(smaller_box);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("bigger_by_smaller.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn two_identical_boxes_one_with_the_other_overlap(file_root: PathBuf) -> anyhow::Result<()> {
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = shapes::rect(
        x_basis_one,
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(0.7)),
    )?;

    let box_two = shapes::rect(
        x_basis_two,
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
    );

    let result = box_one.boolean_union(box_two);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("one_with_other_overlap.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn two_identical_boxes_one_with_the_other(file_root: PathBuf) -> anyhow::Result<()> {
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = shapes::rect(
        x_basis_one,
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(1.0)),
    )?;

    let box_two = shapes::rect(
        x_basis_two,
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
    );

    let result = box_one.boolean_union(box_two);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("one_with_other.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn two_identical_boxes_one_shifted_in_plane(file_root: PathBuf) -> anyhow::Result<()> {
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = shapes::rect(
        x_basis_one,
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * Dec::from(0.9) + Vector3::y() * Dec::from(0.9),
    )?;

    let box_two = shapes::rect(
        x_basis_two,
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
    );

    let result = box_one.boolean_union(box_two);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("shifted_in_plane.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn two_identical_boxes_one_shifted_in_space(file_root: PathBuf) -> anyhow::Result<()> {
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = shapes::rect(
        x_basis_one,
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * Dec::from(0.9)
            + Vector3::y() * Dec::from(0.9)
            + Vector3::z() * Dec::from(0.9),
    )?;

    let box_two = shapes::rect(
        x_basis_two,
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
        Dec::one() * Dec::from(1.),
    );

    let result = box_one.boolean_union(box_two);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("shifted_in_space.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn bigger_box_extended_by_smaller(file_root: PathBuf) -> anyhow::Result<()> {
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = shapes::rect(
        x_basis_one,
        Dec::one() * Dec::from(1.5),
        Dec::one() * Dec::from(1.5),
        Dec::one() * Dec::from(1.5),
    );

    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::z() * Dec::from(0.9),
    )?;

    let box_two = shapes::rect(
        x_basis_two,
        Dec::one() * Dec::from(0.5),
        Dec::one() * Dec::from(0.5),
        Dec::one() * Dec::from(0.5),
    );

    let result = box_one.boolean_union(box_two);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("extention_by_smaller.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn bigger_box_extended_by_longer(file_root: PathBuf) -> anyhow::Result<()> {
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = shapes::rect(
        x_basis_one,
        Dec::one() * Dec::from(1.5),
        Dec::one() * Dec::from(1.5),
        Dec::one() * Dec::from(1.5),
    );

    let x = (Vector3::x() + Vector3::y()).normalize();
    let z = Vector3::z();
    let y = z.cross(&x).normalize();
    let x_basis_two = Basis::new(x, y, z, Vector3::z() * Dec::from(0.0))?;

    let box_two = shapes::rect(
        x_basis_two,
        Dec::one() * Dec::from(0.5),
        Dec::one() * Dec::from(4.5),
        Dec::one() * Dec::from(0.5),
    );

    let result = box_one.boolean_union(box_two);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("extention_by_longer.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn smaller_box_cutted_by_bigger(file_root: PathBuf) -> anyhow::Result<()> {
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = shapes::rect(
        x_basis_one,
        Dec::one() * Dec::from(1),
        Dec::one() * Dec::from(1),
        Dec::one() * Dec::from(1),
    );

    let x = Vector3::x();
    let z = Vector3::z();
    let y = Vector3::y();
    let x_basis_two = Basis::new(x, y, z, Vector3::z() * Dec::from(0.5))?;

    let box_two = shapes::rect(
        x_basis_two,
        Dec::one() * Dec::from(2.5),
        Dec::one() * Dec::from(2.5),
        Dec::one() * Dec::from(0.5),
    );

    let result = box_one.boolean_diff(box_two);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("smaller_cutted_by_bigger.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn smaller_box_cutted_by_longer(file_root: PathBuf) -> anyhow::Result<()> {
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = shapes::rect(
        x_basis_one,
        Dec::one() * Dec::from(1),
        Dec::one() * Dec::from(1),
        Dec::one() * Dec::from(1),
    );

    let x = Vector3::x();
    let z = Vector3::z();
    let y = Vector3::y();
    let x_basis_two = Basis::new(x, y, z, Vector3::x() * Dec::from(0.1))?;

    let box_two = shapes::rect(
        x_basis_two,
        Dec::one() * Dec::from(0.25),
        Dec::one() * Dec::from(0.25),
        Dec::one() * Dec::from(3.0),
    );

    let result = box_one.boolean_diff(box_two);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("smaller_cutted_by_longer.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn smaller_box_cutted_by_bigger_in_two(file_root: PathBuf) -> anyhow::Result<()> {
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = shapes::rect(
        x_basis_one,
        Dec::one() * Dec::from(1),
        Dec::one() * Dec::from(1),
        Dec::one() * Dec::from(2),
    );

    let x = Vector3::x();
    let z = Vector3::z();
    let y = Vector3::y();
    let x_basis_two = Basis::new(x, y, z, Vector3::zeros())?;

    let box_two = shapes::rect(
        x_basis_two,
        Dec::one() * Dec::from(2.5),
        Dec::one() * Dec::from(2.5),
        Dec::one() * Dec::from(0.5),
    );

    let result = box_one.boolean_diff(box_two);

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(file_root.join("cutted_in_two.stl"))
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(())
}

fn main() -> Result<(), anyhow::Error> {
    let cli = Command::parse();

    fs::create_dir_all(cli.output_path.clone())?;
    bigger_by_smaller(cli.output_path.clone())?;
    two_identical_boxes_one_with_the_other_overlap(cli.output_path.clone())?;
    two_identical_boxes_one_with_the_other(cli.output_path.clone())?;
    two_identical_boxes_one_shifted_in_plane(cli.output_path.clone())?;
    two_identical_boxes_one_shifted_in_space(cli.output_path.clone())?;
    bigger_box_extended_by_smaller(cli.output_path.clone())?;
    bigger_box_extended_by_longer(cli.output_path.clone())?;

    smaller_box_cutted_by_bigger(cli.output_path.clone())?;
    smaller_box_cutted_by_bigger_in_two(cli.output_path.clone())?;
    smaller_box_cutted_by_longer(cli.output_path.clone())?;

    Ok(())
}
