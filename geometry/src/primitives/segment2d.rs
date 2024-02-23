use core::fmt;
use std::ops::Neg;

use itertools::Either;
use nalgebra::{ComplexField, Vector2};
use num_traits::{One, Zero};

use crate::{
    bsp::Reversable,
    primitives::decimal::{EPS, STABILITY_ROUNDING},
};

use super::{decimal::Dec, line2d::Line2D};

#[derive(Clone)]
pub struct Segment2D {
    pub from: Vector2<Dec>,
    pub to: Vector2<Dec>,
}

impl PartialEq for Segment2D {
    fn eq(&self, other: &Self) -> bool {
        let fd = self.from - other.from;
        let td = self.to - other.to;

        let fd = fd.magnitude_squared().round_dp(STABILITY_ROUNDING);
        let td = td.magnitude_squared().round_dp(STABILITY_ROUNDING);

        fd == Dec::zero() && td == Dec::zero()
    }
}

impl Reversable for Segment2D {
    fn flip(self) -> Self {
        Self {
            from: self.to,
            to: self.from,
        }
    }
}

impl fmt::Debug for Segment2D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} {} -> {}, {}",
            self.from.x.round_dp(4),
            self.from.y.round_dp(4),
            self.to.x.round_dp(4),
            self.to.y.round_dp(4)
        )
    }
}

impl Segment2D {
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

    pub fn new(from: Vector2<Dec>, to: Vector2<Dec>) -> Self {
        Self { from, to }
    }

    pub fn get_line(&self) -> Line2D {
        Line2D {
            origin: self.from,
            dir: self.to - self.from,
        }
    }

    fn angle_between(&self, other: &Self) -> Dec {
        let n = self.dir().normalize();
        let p = other.dir().normalize();
        n.dot(&p).acos()
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
            panic!("segments with different directions");
        }
        if similarity == Dec::one() {
            let other_from = other.from - self.from;
            let other_to = other.to - self.from;
            let tf = other_from.dot(&self_dir_normalized) / self_dir_len;
            let tt = other_to.dot(&self_dir_normalized) / self_dir_len;
            if (tf - 1) > EPS {
                Either::Right((self, other))
            } else {
                let tf = tf.min(Dec::from(0));
                let tt = tt.max(Dec::from(1));
                Either::Left(Segment2D {
                    from: self.from + self.dir() * tf,
                    to: self.from + self.dir() * tt,
                })
            }
        } else {
            Either::Right((self, other))
        }
    }

    pub(crate) fn dir(&self) -> Vector2<Dec> {
        self.to - self.from
    }
}

#[cfg(test)]
mod tests {
    use itertools::Either;
    use nalgebra::Vector2;
    use rust_decimal_macros::dec;

    use crate::primitives::segment2d::Segment2D;

    #[test]
    fn join_segment_1() {
        let segment1 = Segment2D::new(
            Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.2).into(), dec!(0.2).into()),
        );
        let segment2 = Segment2D::new(
            Vector2::new(dec!(0.2).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.5).into(), dec!(0.2).into()),
        );
        let segment_result = Segment2D::new(
            Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.5).into(), dec!(0.2).into()),
        );
        let joined = segment1.join(segment2);
        assert_eq!(joined, Either::Left(segment_result));
    }
    #[test]
    fn join_segment_2() {
        let segment1 = Segment2D::new(
            Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.2).into(), dec!(0.2).into()),
        );
        let segment2 = Segment2D::new(
            Vector2::new(dec!(-0.1).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.5).into(), dec!(0.2).into()),
        );
        let segment_result = Segment2D::new(
            Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.5).into(), dec!(0.2).into()),
        );
        let joined = segment1.join(segment2);
        assert_eq!(joined, Either::Left(segment_result));
    }

    #[test]
    fn join_segment_3() {
        let segment1 = Segment2D::new(
            Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.2).into(), dec!(0.2).into()),
        );
        let segment2 = Segment2D::new(
            Vector2::new(dec!(-0.5).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.5).into(), dec!(0.2).into()),
        );
        let segment_result = Segment2D::new(
            Vector2::new(dec!(-0.5).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.5).into(), dec!(0.2).into()),
        );
        let joined = segment1.join(segment2);
        assert_eq!(joined, Either::Left(segment_result));
    }

    #[test]
    fn join_segment_4() {
        let segment1 = Segment2D::new(
            Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.2).into(), dec!(0.2).into()),
        );
        let segment2 = Segment2D::new(
            Vector2::new(dec!(-0.1).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.1).into(), dec!(0.2).into()),
        );
        let segment_result = Segment2D::new(
            Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.2).into(), dec!(0.2).into()),
        );
        let joined = segment1.join(segment2);
        assert_eq!(joined, Either::Left(segment_result));
    }

    #[test]
    fn join_segment_5() {
        let segment1 = Segment2D::new(
            Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.2).into(), dec!(0.2).into()),
        );
        let segment2 = Segment2D::new(
            Vector2::new(dec!(0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.5).into(), dec!(0.2).into()),
        );
        let joined = segment1.clone().join(segment2.clone());
        assert_eq!(joined, Either::Right((segment1, segment2)));
    }
    #[test]
    fn join_segment_6() {
        let segment1 = Segment2D::new(
            Vector2::new(dec!(-0.1).into(), dec!(0.0).into()),
            Vector2::new(dec!(0.0).into(), dec!(0.2).into()),
        );
        let segment2 = Segment2D::new(
            Vector2::new(dec!(0.0).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.5).into(), dec!(0.1).into()),
        );
        let joined = segment1.clone().join(segment2.clone());
        assert_eq!(joined, Either::Right((segment1, segment2)));
    }
}
