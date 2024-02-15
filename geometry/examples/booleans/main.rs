use std::fs::OpenOptions;

use clap::Parser;
use geometry::{
    primitives::{basis::Basis, decimal::Dec},
    shapes,
};
use nalgebra::Vector3;
use num_traits::{One, Zero};
use rust_decimal_macros::dec;

#[derive(Parser)]
pub struct Command {
    #[arg(long)]
    pub output_path: String,
}

fn main() -> Result<(), anyhow::Error> {
    /*
    std::panic::set_hook(Box::new(|panic_info| {
        let backtrace = std::backtrace::Backtrace::capture();
        eprintln!("My backtrace: {:#?}", backtrace);
    }));
    */
    let zz = Vector3::new(Dec::one(), Dec::zero(), Dec::one()).normalize();
    let yy = Vector3::y();
    let xx = zz.cross(&yy);

    let _rotated_basis = Basis {
        x: xx,
        y: yy,
        z: -zz,
        center: Vector3::zero(),
    };

    let _rotated_shifted_basis = Basis {
        x: xx,
        y: yy,
        z: -zz,
        center: Vector3::zero() + (xx * Dec::from(dec!(2.2))),
    };

    let _zero_basis = Basis {
        center: Vector3::zeros(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    };

    let _x_basis = Basis {
        center: Vector3::zeros() + Vector3::x() * Dec::from(dec!(1.5)),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    };

    let cli = Command::parse();

    // let cylinder = shapes::(_zero_basis, Dec::one() * 1, Dec::one() * 1, 5);
    let smaller_box = shapes::rect(
        _zero_basis.clone(),
        Dec::one() * 1,
        Dec::one() * 1,
        Dec::one() * 1,
    );

    let bigger_box = shapes::rect(_zero_basis, Dec::one() * 2, Dec::one() * 2, Dec::one() * 2);

    let result = match bigger_box.boolean_union(smaller_box) {
        itertools::Either::Left(joined) => joined,
        itertools::Either::Right(_) => {
            panic!("join didn't happend");
        }
    };

    let mut writer = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(cli.output_path)
        .unwrap();

    stl_io::write_stl(&mut writer, result.into_iter())?;

    Ok(())
}
