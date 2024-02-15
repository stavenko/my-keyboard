use std::fmt;

use itertools::Either;
use nalgebra::{ComplexField, Vector3};

use super::{
    decimal::{Dec, EPS},
    line::Line,
};

#[derive(Clone, PartialEq)]
pub struct Segment {
    pub from: Vector3<Dec>,
    pub to: Vector3<Dec>,
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

impl Segment {
    pub fn new(from: Vector3<Dec>, to: Vector3<Dec>) -> Self {
        Self { from, to }
    }

    pub fn get_line(&self, with_normal: Vector3<Dec>) -> Line {
        Line {
            normal: with_normal,
            origin: self.from,
            dir: self.to - self.from,
        }
    }

    pub(crate) fn flip(mut self) -> Self {
        Self {
            from: self.to,
            to: self.from,
        }
    }
    pub(crate) fn dir(&self) -> Vector3<Dec> {
        self.to - self.from
    }

    fn angle_between(&self, other: &Self) -> Dec {
        let n = self.dir().normalize();
        let p = other.dir().normalize();
        n.dot(&p).acos()
    }

    pub(crate) fn join(self, other: Segment) -> Either<Self, (Self, Self)> {
        let angle = self.angle_between(&other);
        if angle.abs() < EPS {
            let dot = self.dir().normalize().dot(&other.dir().normalize());
            if dot < -EPS {
                panic!("segments with different directions");
            }
            let other_from = other.from - self.from;
            let other_to = other.to - self.from;
            let tf = other_from.dot(&self.dir().normalize()) / self.dir().magnitude();
            let tt = other_to.dot(&self.dir().normalize()) / self.dir().magnitude();
            if (tf - 1) > EPS {
                Either::Right((self, other))
            } else {
                let tf = tf.min(Dec::from(0));
                let tt = tt.max(Dec::from(1));
                Either::Left(Segment {
                    from: self.from + self.dir() * tf,
                    to: self.from + self.dir() * tt,
                })
            }
        } else {
            Either::Right((self, other))
        }
    }
}
