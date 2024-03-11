use std::{fmt, ops::Neg};

use itertools::Either;
use nalgebra::{ComplexField, Vector3};
use num_traits::{One, Zero};

use crate::decimal::{Dec, STABILITY_ROUNDING};

use super::{line::Line, ray::Ray};

#[derive(Clone)]
pub struct Segment {
    pub from: Vector3<Dec>,
    pub to: Vector3<Dec>,
}
impl From<Segment> for Line {
    fn from(value: Segment) -> Self {
        Self {
            origin: value.from,
            dir: value.dir().normalize(),
        }
    }
}

impl fmt::Debug for Segment {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} {} {} -> {} {} {}",
            self.from.x.round_dp(4),
            self.from.y.round_dp(4),
            self.from.z.round_dp(4),
            self.to.x.round_dp(4),
            self.to.y.round_dp(4),
            self.to.z.round_dp(4)
        )
    }
}

impl PartialEq for Segment {
    fn eq(&self, other: &Self) -> bool {
        let fd = self.from - other.from;
        let fd = fd.magnitude_squared().round_dp(STABILITY_ROUNDING);
        if fd.is_zero() {
            let td = self.to - other.to;

            let td = td.magnitude_squared().round_dp(STABILITY_ROUNDING);
            td.is_zero()
        } else {
            false
        }
    }
}

impl Segment {
    pub fn has(&self, point: Vector3<Dec>) -> bool {
        let d = self.from - point;
        let q = self.to - point;
        d.magnitude_squared()
            .round_dp(STABILITY_ROUNDING - 5)
            .abs()
            .is_zero()
            || q.magnitude_squared()
                .round_dp(STABILITY_ROUNDING - 5)
                .abs()
                .is_zero()
    }
    pub fn new(from: Vector3<Dec>, to: Vector3<Dec>) -> Self {
        Self { from, to }
    }

    pub(crate) fn flip(self) -> Self {
        Self {
            from: self.to,
            to: self.from,
        }
    }

    pub fn get_ray(&self) -> Ray {
        Ray {
            origin: self.from,
            dir: self.dir().normalize(),
        }
    }

    pub fn get_line(&self) -> Line {
        Line {
            origin: self.from,
            dir: self.dir().normalize(),
        }
    }

    pub(crate) fn dir(&self) -> Vector3<Dec> {
        self.to - self.from
    }

    pub fn remove_opposites(mut segments: Vec<Self>) -> Vec<Self> {
        let mut joined = Vec::new();

        while let Some(left) = segments.pop() {
            let inv = left.clone().flip();
            match segments.iter().position(|segment| *segment == inv) {
                None => joined.push(left),
                Some(ix) => {
                    segments.swap_remove(ix);
                }
            }
        }
        joined
    }

    pub fn join(self, other: Self) -> Either<Self, (Self, Self)> {
        let self_dir_len = self.dir().magnitude_squared().round_dp(STABILITY_ROUNDING);
        let self_dir_len = self_dir_len.sqrt();

        let other_dir_len = other.dir().magnitude_squared().round_dp(STABILITY_ROUNDING);
        let other_dir_len = other_dir_len.sqrt();

        let self_dir_normalized = self.dir() / self_dir_len;
        let other_dir_normalized = other.dir() / other_dir_len;

        let similarity = (self_dir_normalized)
            .dot(&other_dir_normalized)
            .round_dp(STABILITY_ROUNDING - 2);

        if similarity == Dec::one().neg() {
            dbg!(self);
            dbg!(other);
            panic!("segments with different directions");
        }

        if similarity == Dec::one() {
            let other_from = other.from - self.from;
            let other_to = other.to - self.from;
            let tf = other_from.dot(&self_dir_normalized) / self_dir_len;
            let tt = other_to.dot(&self_dir_normalized) / self_dir_len;
            let tf = tf.min(Dec::from(0));
            let tt = tt.max(Dec::from(1));
            Either::Left(Segment {
                from: self.from + self.dir() * tf,
                to: self.from + self.dir() * tt,
            })
        } else {
            Either::Right((self, other))
        }
    }
}
