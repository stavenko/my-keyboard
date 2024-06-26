/*
use std::{
    borrow::Cow,
    fs::{self, OpenOptions},
    path::PathBuf,
};

use clap::Parser;
use geometry::{
    basis::Basis,
    decimal::{Dec, STABILITY_ROUNDING},
    indexes::geo_index::index::GeoIndex,
    shapes,
};

use nalgebra::{UnitQuaternion, UnitVector3, Vector3};
use num_traits::One;
use rust_decimal_macros::dec;


#[derive(Parser)]
pub struct Command {
    #[arg(long)]
    pub output_path: PathBuf,
}

fn bigger_by_smaller(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();

    let zz = Vector3::z();
    let yy = Vector3::y();
    let xx = yy.cross(&zz).normalize();

    let _zero_basis = Basis::new(xx, yy, zz, Vector3::zeros())?;

    let smaller_box = shapes::rect(
        _zero_basis.clone(),
        Dec::one() * 1,
        Dec::one() * 1,
        Dec::one() * 1,
    );

    let bigger_box = index.save_mesh(
        shapes::rect(_zero_basis, Dec::one() * 2, Dec::one() * 2, Dec::one() * 2)
            .into_iter()
            .map(Cow::Owned),
    );

    let smaller_box = index.save_mesh(smaller_box.into_iter().map(Cow::Owned));
    let bigger_box_mut = index.get_mutable_mesh(bigger_box);

    let result = bigger_box_mut.boolean_union(smaller_box).remove(0);

    let filename = "bigger_by_smaller.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.into()])
}

fn smaller_by_bigger(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let zz = Vector3::z();
    let yy = Vector3::y();
    let xx = yy.cross(&zz).normalize();

    let _zero_basis = Basis::new(xx, yy, zz, Vector3::zeros())?;

    let smaller_box = shapes::rect(
        _zero_basis.clone(),
        Dec::one() * 1,
        Dec::one() * 1,
        Dec::one() * 1,
    );

    let bigger_box = index.save_mesh(
        shapes::rect(_zero_basis, Dec::one() * 2, Dec::one() * 2, Dec::one() * 2)
            .into_iter()
            .map(Cow::Owned),
    );

    let smaller_box = index.save_mesh(smaller_box.into_iter().map(Cow::Owned));
    let smaller_box_mut = index.get_mutable_mesh(smaller_box);

    let result = smaller_box_mut.boolean_union(bigger_box).remove(0);

    let filename = "smaller_by_bigger.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.into()])
}

fn two_identical_boxes_with_overlapped_side_and_rotated(
    file_root: PathBuf,
) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let rot = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(Vector3::x()),
        Dec::from(std::f32::consts::FRAC_PI_4),
    );

    let x_basis_two = Basis::new(
        Vector3::x(),
        rot * Vector3::y(),
        rot * Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(0.5)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "one_with_rotated_overlapped_side.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.into()])
}
fn two_identical_boxes_one_with_one_common_side_rotated(
    file_root: PathBuf,
) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let rot = UnitQuaternion::from_axis_angle(
        &UnitVector3::new_normalize(Vector3::x()),
        Dec::from(std::f32::consts::FRAC_PI_4),
    );

    let x_basis_two = Basis::new(
        Vector3::x(),
        rot * Vector3::y(),
        rot * Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(1.0)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "one_with_rotated_common_side.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.into()])
}
fn two_identical_boxes_one_with_one_common_side(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(1.0)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "one_with_other_overlap.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.into()])
}

fn two_identical_boxes_one_with_overlap(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * (Dec::one() * Dec::from(0.4).round_dp(STABILITY_ROUNDING)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "one_with_one_common_side.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn two_identical_boxes_one_shifted_in_plane(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * Dec::from(dec!(0.7)).round_dp(3)
            + Vector3::y() * Dec::from(dec!(0.6)).round_dp(3),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "shifted_in_plane.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn two_identical_boxes_one_shifted_in_space(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );
    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x() * Dec::from(dec!(0.5))
            + Vector3::y() * Dec::from(dec!(0.6))
            + Vector3::z() * Dec::from(dec!(0.9)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
            Dec::one() * Dec::from(1.),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "shifted_in_space.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn bigger_box_extended_by_smaller(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(dec!(1.5)),
            Dec::one() * Dec::from(dec!(1.5)),
            Dec::one() * Dec::from(dec!(1.5)),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x_basis_two = Basis::new(
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::z() * Dec::from(dec!(0.9)),
    )?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(dec!(0.5)),
            Dec::one() * Dec::from(dec!(0.5)),
            Dec::one() * Dec::from(dec!(0.5)),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "extention_by_smaller.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn bigger_box_extended_by_longer(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(dec!(1.5)),
            Dec::one() * Dec::from(dec!(1.5)),
            Dec::one() * Dec::from(dec!(1.5)),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x = (Vector3::x() + Vector3::y()).normalize();
    let z = Vector3::z();
    let y = z.cross(&x).normalize();
    let x_basis_two = Basis::new(x, y, z, Vector3::z() * Dec::from(0.0))?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(0.5),
            Dec::one() * Dec::from(4.5),
            Dec::one() * Dec::from(0.5),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_union(box_two)
        .remove(0);

    let filename = "extention_by_longer.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn smaller_box_cutted_by_bigger(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x = Vector3::x();
    let z = Vector3::z();
    let y = Vector3::y();
    let x_basis_two = Basis::new(x, y, z, Vector3::z() * Dec::from(0.5))?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(2.5),
            Dec::one() * Dec::from(2.5),
            Dec::one() * Dec::from(0.5),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_diff(box_two)
        .unwrap()
        .remove(0);

    let filename = "smaller_cutted_by_bigger.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn smaller_box_cutted_by_longer(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x = Vector3::x();
    let z = Vector3::z();
    let y = Vector3::y();
    let x_basis_two = Basis::new(x, y, z, Vector3::x() * Dec::from(0.1))?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(0.25),
            Dec::one() * Dec::from(0.25),
            Dec::one() * Dec::from(3.0),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let result = index
        .get_mutable_mesh(box_one)
        .boolean_diff(box_two)
        .unwrap()
        .remove(0);

    let filename = "smaller_cutted_by_longer.stl";
    let path = file_root.join(filename);
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;
    Ok(vec![filename.to_owned()])
}

fn smaller_box_cutted_by_bigger_in_two(file_root: PathBuf) -> anyhow::Result<Vec<String>> {
    let mut index = GeoIndex::new();
    let x_basis_one = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros())?;

    let box_one = index.save_mesh(
        shapes::rect(
            x_basis_one,
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(1),
            Dec::one() * Dec::from(2),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let x = Vector3::x();
    let z = Vector3::z();
    let y = Vector3::y();
    let x_basis_two = Basis::new(x, y, z, Vector3::zeros())?;

    let box_two = index.save_mesh(
        shapes::rect(
            x_basis_two,
            Dec::one() * Dec::from(2.5),
            Dec::one() * Dec::from(2.5),
            Dec::one() * Dec::from(0.5),
        )
        .into_iter()
        .map(Cow::Owned),
    );

    let mut paths = Vec::new();
    let mut results = index
        .get_mutable_mesh(box_one)
        .boolean_diff(box_two)
        .unwrap();

    let filename = "cutted_in_two1.stl";
    let r = results.remove(0);
    let p = file_root.join(filename);
    paths.push(filename.to_owned());
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(p)
        .unwrap();

    stl_io::write_stl(&mut writer, r.into_iter())?;

    let filename = "cutted_in_two2.stl";
    let r = results.remove(0);
    let p = file_root.join(filename);
    paths.push(filename.to_owned());
    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(p)
        .unwrap();

    stl_io::write_stl(&mut writer, r.into_iter())?;
    Ok(paths)
}
*/

fn main() -> Result<(), anyhow::Error> {
    /*
    let cli = Command::parse();

    fs::create_dir_all(cli.output_path.clone())?;
    let paths = vec![
        /*
        bigger_by_smaller(cli.output_path.clone())?,
        smaller_by_bigger(cli.output_path.clone())?,
        two_identical_boxes_one_with_one_common_side(cli.output_path.clone())?,
        two_identical_boxes_one_with_overlap(cli.output_path.clone())?,
        two_identical_boxes_one_with_one_common_side_rotated(cli.output_path.clone())?,
        two_identical_boxes_with_overlapped_side_and_rotated(cli.output_path.clone())?,
        two_identical_boxes_one_shifted_in_plane(cli.output_path.clone())?,
        two_identical_boxes_one_shifted_in_space(cli.output_path.clone())?,
        bigger_box_extended_by_smaller(cli.output_path.clone())?,
        bigger_box_extended_by_longer(cli.output_path.clone())?,
        smaller_box_cutted_by_bigger(cli.output_path.clone())?,
        smaller_box_cutted_by_bigger_in_two(cli.output_path.clone())?,
        smaller_box_cutted_by_longer(cli.output_path.clone())?,
         */
        /*
         */
    ]
    .concat();

    let grid: i32 = 4;
    let grid_size = 3.0;
    let mut scad = Vec::new();
    'outer: for w in 0..grid {
        let x = grid_size * (w as f32 - (grid as f32 / 2.0));
        for h in 0..grid {
            let y = grid_size * (h as f32 - (grid as f32 / 2.0));
            let i = h + (w * grid);
            if let Some(path) = paths.get(i as usize) {
                scad.push(format!(
                    "translate(v=[{}, {}, 0]) {{ import(\"{}\"); }}",
                    x, y, path
                ));
            } else {
                break 'outer;
            }
        }
    }

    let file_content = scad.join("\n");

    fs::write(cli.output_path.join("tot.scad"), file_content).unwrap();

    */
    Ok(())
}
