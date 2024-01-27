use nalgebra::{ComplexField, Vector2};
use rust_decimal_macros::dec;

use super::{decimal::Dec, segment2d::Segment2D};

#[derive(Clone, Debug, PartialEq)]
pub struct Line2D {
    pub origin: Vector2<Dec>,
    pub dir: Vector2<Dec>,
}
#[derive(Default, Debug, PartialEq, Eq)]
pub struct SplitResult {
    pub front: Vec<Segment2D>,
    pub back: Vec<Segment2D>,
    pub coplanar_back: Vec<Segment2D>,
    pub coplanar_front: Vec<Segment2D>,
}

impl SplitResult {
    pub fn front(mut self, segment: Segment2D) -> Self {
        self.front.push(segment);
        self
    }
    pub fn fronts(mut self, mut segment: Vec<Segment2D>) -> Self {
        self.front.append(&mut segment);
        self
    }
    pub fn back(mut self, segment: Segment2D) -> Self {
        self.back.push(segment);
        self
    }
    pub fn backs(mut self, mut segment: Vec<Segment2D>) -> Self {
        self.back.append(&mut segment);
        self
    }
    pub fn coplanar_back(mut self, segment: Segment2D) -> Self {
        self.coplanar_back.push(segment);
        self
    }
    pub fn coplanar_backs(mut self, mut segment: Vec<Segment2D>) -> Self {
        self.coplanar_back.append(&mut segment);
        self
    }
    pub fn coplanar_front(mut self, segment: Segment2D) -> Self {
        self.coplanar_front.push(segment);
        self
    }
    pub fn coplanar_fronts(mut self, mut segment: Vec<Segment2D>) -> Self {
        self.coplanar_front.append(&mut segment);
        self
    }
}

#[derive(Debug, PartialEq)]
enum Location {
    Front,
    Back,
    Coplanar,
}

impl Line2D {
    fn kross(v: &Vector2<Dec>, u: &Vector2<Dec>) -> Dec {
        v.x * u.y - v.y * u.x
    }

    pub fn split_segment(&self, segment: Segment2D) -> SplitResult {
        let eps: Dec = dec!(1e-8).into();
        let ft = segment.dir();
        let from = segment.from - self.origin;
        let to = segment.to - self.origin;
        let k_from = Self::kross(&self.dir, &from);
        let k_to = Self::kross(&self.dir, &to);
        let k_to_dir = Self::kross(&self.dir, &ft);
        let loc = |k| {
            if k > eps {
                Location::Back
            } else if k < -eps {
                Location::Front
            } else {
                Location::Coplanar
            }
        };
        match (loc(k_from), loc(k_to)) {
            (Location::Front, Location::Front) => SplitResult::default().front(segment),
            (Location::Back, Location::Back) => SplitResult::default().back(segment),

            (Location::Coplanar, Location::Front) => SplitResult::default().front(segment),
            (Location::Front, Location::Coplanar) => SplitResult::default().front(segment),
            (Location::Coplanar, Location::Back) => SplitResult::default().back(segment),
            (Location::Back, Location::Coplanar) => SplitResult::default().back(segment),
            (Location::Front, Location::Back) => {
                let t = k_to / k_to_dir;
                let p = segment.from + segment.dir() * (Dec::from(dec!(1)) - t);
                SplitResult::default()
                    .front(Segment2D {
                        from: segment.from,
                        to: p,
                    })
                    .back(Segment2D {
                        from: p,
                        to: segment.to,
                    })
            }
            (Location::Back, Location::Front) => {
                let t = k_to / k_to_dir;
                let p = segment.from + segment.dir() * (Dec::from(dec!(1)) - t);
                SplitResult::default()
                    .back(Segment2D {
                        from: segment.from,
                        to: p,
                    })
                    .front(Segment2D {
                        from: p,
                        to: segment.to,
                    })
            }
            (Location::Coplanar, Location::Coplanar) => {
                if self.dir.dot(&ft) > Dec::from(0) {
                    SplitResult::default().coplanar_front(segment)
                } else {
                    SplitResult::default().coplanar_back(segment)
                }
            }
        }
    }

    pub(crate) fn flip(mut self) -> Self {
        self.dir = -self.dir;
        self
    }
}
#[cfg(test)]
mod tests {
    use nalgebra::Vector2;
    use rust_decimal_macros::dec;

    use crate::geometry::primitives::segment2d::Segment2D;

    use super::Line2D;

    #[test]
    fn split_segment() {
        let line = Line2D {
            origin: Vector2::zeros(),
            dir: Vector2::new(dec!(0).into(), dec!(1).into()),
        };
        let segment = Segment2D::new(
            Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.3).into(), dec!(0.2).into()),
        );

        let res = line.split_segment(segment);
        assert_eq!(
            res.front,
            vec!(Segment2D {
                from: Vector2::new(dec!(0.000).into(), dec!(0.2).into()),
                to: Vector2::new(dec!(0.3).into(), dec!(0.2).into())
            })
        );
        assert_eq!(
            res.back,
            vec!(Segment2D {
                from: Vector2::new(dec!(-0.3).into(), dec!(0.2).into()),
                to: Vector2::new(dec!(0).into(), dec!(0.2).into())
            })
        );
        let segment = Segment2D::new(
            Vector2::new(dec!(0.1).into(), dec!(0.2).into()),
            Vector2::new(dec!(-0.5).into(), dec!(0.2).into()),
        );

        let res = line.split_segment(segment);
        assert_eq!(
            res.front,
            vec!(Segment2D {
                from: Vector2::new(dec!(0.1).into(), dec!(0.2).into()),
                to: Vector2::new(dec!(0.).into(), dec!(0.2).into())
            })
        );
        assert_eq!(
            res.back,
            vec!(Segment2D {
                from: Vector2::new(dec!(0).into(), dec!(0.2).into()),
                to: Vector2::new(dec!(-0.5).into(), dec!(0.2).into())
            })
        );

        let segment = Segment2D::new(
            Vector2::new(dec!(-0.1).into(), dec!(0.2).into()),
            Vector2::new(dec!(0.5).into(), dec!(0.2).into()),
        );

        let res = line.split_segment(segment);
        assert_eq!(
            res.front,
            vec!(Segment2D {
                from: Vector2::new(dec!(0.000).into(), dec!(0.2).into()),
                to: Vector2::new(dec!(0.5).into(), dec!(0.2).into())
            })
        );
        assert_eq!(
            res.back,
            vec!(Segment2D {
                from: Vector2::new(dec!(-0.1).into(), dec!(0.2).into()),
                to: Vector2::new(dec!(0).into(), dec!(0.2).into())
            })
        );

        let segment = Segment2D::new(
            Vector2::new(dec!(0.0).into(), dec!(0.0).into()),
            Vector2::new(dec!(0.0).into(), dec!(1.2).into()),
        );

        let res = line.split_segment(segment);
        assert_eq!(
            res.coplanar_front,
            vec!(Segment2D {
                from: Vector2::new(dec!(0.000).into(), dec!(0).into()),
                to: Vector2::new(dec!(0.0).into(), dec!(1.2).into())
            })
        );
        assert_eq!(res.coplanar_back, Vec::new());

        let segment = Segment2D::new(
            Vector2::new(dec!(0.0).into(), dec!(0.0).into()),
            Vector2::new(dec!(0.0).into(), dec!(-1.2).into()),
        );

        let res = line.split_segment(segment);
        assert_eq!(
            res.coplanar_back,
            vec!(Segment2D {
                from: Vector2::new(dec!(0.000).into(), dec!(0).into()),
                to: Vector2::new(dec!(0.0).into(), dec!(-1.2).into())
            })
        );
        assert_eq!(res.coplanar_front, Vec::new());

        let segment = Segment2D::new(
            Vector2::new(dec!(0.0).into(), dec!(0.0).into()),
            Vector2::new(dec!(0.5).into(), dec!(1.2).into()),
        );

        let res = line.split_segment(segment);
        assert_eq!(
            res.front,
            vec!(Segment2D {
                from: Vector2::new(dec!(0.000).into(), dec!(0).into()),
                to: Vector2::new(dec!(0.5).into(), dec!(1.2).into())
            })
        );
        assert_eq!(res.back, Vec::new());

        let segment = Segment2D::new(
            Vector2::new(dec!(-0.5).into(), dec!(1.2).into()),
            Vector2::new(dec!(0.0).into(), dec!(0.0).into()),
        );

        let res = line.split_segment(segment);
        assert_eq!(
            res.back,
            vec!(Segment2D {
                from: Vector2::new(dec!(-0.5).into(), dec!(1.2).into()),
                to: Vector2::new(dec!(0.000).into(), dec!(0).into()),
            })
        );
        assert_eq!(res.front, Vec::new());
    }
}
