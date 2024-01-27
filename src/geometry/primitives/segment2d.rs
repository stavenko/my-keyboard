use core::fmt;
use std::fmt::Debug;

use itertools::Either;
use nalgebra::{ComplexField, Vector2};

use crate::geometry::primitives::decimal::EPS;

use super::{decimal::Dec, line::Line, line2d::Line2D};

#[derive(Clone, PartialEq, Eq)]
pub struct Segment2D {
    pub from: Vector2<Dec>,
    pub to: Vector2<Dec>,
}

impl fmt::Debug for Segment2D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} {} -> {}, {}",
            self.from.x, self.from.y, self.to.x, self.to.y
        )
    }
}

impl Segment2D {
    pub fn new(from: Vector2<Dec>, to: Vector2<Dec>) -> Self {
        Self { from, to }
    }

    pub fn get_line(&self) -> Line2D {
        Line2D {
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
    fn kross(v: &Vector2<Dec>, u: &Vector2<Dec>) -> Dec {
        v.x * u.y - v.y * u.x
    }
    fn angle_between(&self, other: &Self) -> Dec {
        let n = self.dir().normalize();
        let p = other.dir().normalize();
        n.dot(&p).acos()
    }

    pub fn join(self, other: Self) -> Either<Self, (Self, Self)> {
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

    use crate::geometry::primitives::segment2d::Segment2D;

    use super::Line2D;

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
