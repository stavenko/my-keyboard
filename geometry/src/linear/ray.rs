use core::fmt;

use nalgebra::Vector3;

use crate::decimal::Dec;

#[derive(Clone)]
pub struct Ray {
    pub origin: Vector3<Dec>,
    pub dir: Vector3<Dec>,
}

impl fmt::Debug for Ray {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} {} {} -> {} {} {}",
            self.origin.x.round_dp(4),
            self.origin.y.round_dp(4),
            self.origin.z.round_dp(4),
            self.dir.x.round_dp(4),
            self.dir.y.round_dp(4),
            self.dir.z.round_dp(4)
        )
    }
}

/*
impl Intersects<Plane> for Ray {
    type Out = Vector3<Dec>;

    fn intersects(&self, other: &Plane) -> Option<Self::Out> {
        let dot = self.dir.dot(&other.normal()).round_dp(STABILITY_ROUNDING);
        if dot.is_zero() {
            None
        } else {
            let t = (other.normal().dot(&self.origin) - other.d()).neg() / dot;
            if t.is_positive() {
                Some(self.dir * t + self.origin)
            } else {
                None
            }
        }
    }
}

impl Intersects<Ray> for Ray {
    type Out = Vector2<Dec>;

    fn intersects(&self, other: &Ray) -> Option<Self::Out> {
        let dot = self.dir.dot(&other.dir);
        let q = self.origin - other.origin;
        let m = Matrix2::new(Dec::from(1), -dot, dot, -Dec::from(1));
        let b = -Vector2::new(q.dot(&self.dir), q.dot(&other.dir));

        if let Some(mi) = m.try_inverse() {
            let st = mi * b;
            let p1 = self.origin + self.dir * st.x;
            let p2 = other.origin + other.dir * st.y;
            if (p1 - p2)
                .magnitude_squared()
                .round_dp(STABILITY_ROUNDING)
                .is_zero()
            {
                Some(st)
            } else {
                None
            }
        } else {
            println!("WARN, failed to inverse matrix");
            None
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum SegmentIntersection {
    Inner(Vector3<Dec>),
    End(Vector3<Dec>),
}

impl Intersects<Segment> for Ray {
    type Out = SegmentIntersection;

    fn intersects(&self, other_segment: &Segment) -> Option<Self::Out> {
        let dot = other_segment.dir().dot(&self.dir);
        let q = other_segment.from - self.origin;

        let m = Matrix2::new(
            Dec::from(1),
            -dot,
            dot,
            -(other_segment.dir().magnitude_squared()),
        );

        let b = Vector2::new(q.dot(&self.dir), q.dot(&other_segment.dir()));

        if let Some(mi) = m.try_inverse() {
            let st = mi * b;
            let p1 = self.origin + self.dir * st.x;
            let p2 = other_segment.from + other_segment.dir() * st.y;
            if st.x.is_negative() {
                return None;
            }

            if (p1 - p2)
                .magnitude_squared()
                .round_dp(STABILITY_ROUNDING)
                .is_zero()
            {
                let y = st.y.round_dp(STABILITY_ROUNDING);
                if y.is_one() {
                    return Some(SegmentIntersection::End(other_segment.to));
                }

                if y.is_zero() {
                    return Some(SegmentIntersection::End(other_segment.from));
                }

                if y > Dec::zero() && y < Dec::one() {
                    return Some(SegmentIntersection::Inner(p1));
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;
    use num_traits::{One, Zero};

    use crate::{
        decimal::{Dec, STABILITY_ROUNDING},
        intersects::Intersects,
        plane::Plane,
        ray::SegmentIntersection,
        segment::Segment,
    };

    use super::Ray;

    #[test]
    fn check_ray_plane_intersection() {
        let r1 = Ray {
            origin: Vector3::zeros(),
            dir: Vector3::x(),
        };

        let p2 = Plane::new_from_normal_and_point(Vector3::x(), Vector3::x());

        assert_eq!(r1.intersects(&p2), Some(Vector3::x()));
    }

    #[test]
    fn check_ray_ray_intersection() {
        let r1 = Ray {
            origin: Vector3::zeros(),
            dir: Vector3::x(),
        };

        let r2 = Ray {
            origin: Vector3::y(),
            dir: Vector3::x(),
        };

        assert!(r1.intersects(&r2).is_none());

        let r2 = Ray {
            origin: Vector3::y(),
            dir: -Vector3::x(),
        };

        assert!(r1.intersects(&r2).is_none());

        let r2 = Ray {
            origin: Vector3::y(),
            dir: (Vector3::x() - Vector3::y()).normalize(),
        };

        assert_eq!(
            r1.intersects(&r2).map(|v| v.x.round_dp(STABILITY_ROUNDING)),
            Some(Dec::one())
        );

        let r2 = Ray {
            origin: Vector3::y(),
            dir: (Vector3::x() + Vector3::y()).normalize(),
        };

        assert_eq!(
            r1.intersects(&r2).map(|v| v.x.round_dp(STABILITY_ROUNDING)),
            Some(-Dec::one())
        );
        let r2 = Ray {
            origin: Vector3::y() + Vector3::x(),
            dir: (Vector3::x() + Vector3::y()).normalize(),
        };

        assert_eq!(
            r1.intersects(&r2).map(|v| v.x.round_dp(STABILITY_ROUNDING)),
            Some(Dec::zero())
        );
    }

    #[test]
    fn check_ray_segment_intersection() {
        let r1 = Ray {
            origin: Vector3::zeros(),
            dir: Vector3::x(),
        };

        let r2 = Segment {
            from: Vector3::y(),
            to: Vector3::y() + Vector3::x(),
        };

        assert!(dbg!(r1.intersects(&r2)).is_none());

        let r2 = Segment {
            from: Vector3::y(),
            to: Vector3::y() - Vector3::x(),
        };

        assert!(dbg!(r1.intersects(&r2)).is_none());

        let r2 = Segment {
            from: Vector3::y(),
            to: Vector3::x(),
        };

        assert_eq!(
            r1.intersects(&r2),
            Some(SegmentIntersection::End(Vector3::y()))
        );

        let r2 = Segment {
            from: Vector3::y(),
            to: Vector3::x() * Dec::from(3),
        };

        assert_eq!(
            r1.intersects(&r2),
            Some(SegmentIntersection::End(Vector3::x() * Dec::from(3)))
        );

        let r2 = Segment {
            from: Vector3::z() + Vector3::y() + Vector3::z(),
            to: Vector3::zeros(),
        };

        assert_eq!(
            r1.intersects(&r2),
            Some(SegmentIntersection::End(Vector3::zeros()))
        );
    }
}
*/
