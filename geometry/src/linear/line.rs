use core::fmt;

use nalgebra::{ComplexField, Matrix2, Vector2, Vector3};
use num_traits::Zero;
use rust_decimal_macros::dec;

use crate::{decimal::Dec, indexes::geo_index::seg::SegRef};

#[derive(Clone)]
pub struct Line {
    pub origin: Vector3<Dec>,
    pub dir: Vector3<Dec>,
}

impl Line {
    pub(crate) fn get_intersection_params_seg_ref(&self, to: &SegRef<'_>) -> Option<(Dec, Dec)> {
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;

        let segment_dir = to.dir().normalize();
        let q = self.origin - to.from();

        let dot = self.dir.dot(&segment_dir);

        let m = Matrix2::new(Dec::from(1), -dot, dot, -Dec::from(1));
        let b = -Vector2::new(q.dot(&self.dir), q.dot(&segment_dir));

        if m.determinant().abs() < vertex_pulling_sq {
            return None;
        }
        if let Some(mi) = m.try_inverse() {
            let st = mi * b;
            let p1 = self.dir * st.x + self.origin;
            let p2 = to.dir().normalize() * st.y + to.from();
            let dist = p1 - p2;
            if dist.magnitude_squared() < vertex_pulling_sq {
                Some((st.x, st.y / to.dir().magnitude()))
            } else {
                None
            }
        } else {
            None
        }
    }

    pub(crate) fn distance_to_pt_squared(&self, pt: Vector3<Dec>) -> Dec {
        let v = pt - self.origin;
        if v.magnitude_squared().is_zero() {
            Dec::zero()
        } else {
            let t = v.dot(&self.dir);
            v.dot(&v) - t * t
        }
    }
}

impl fmt::Debug for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} {} {} -> {} {} {}",
            self.origin.x.round_dp(5),
            self.origin.y.round_dp(5),
            self.origin.z.round_dp(5),
            self.dir.x.round_dp(5),
            self.dir.y.round_dp(5),
            self.dir.z.round_dp(5)
        )
    }
}
