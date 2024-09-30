use core::fmt;

use nalgebra::{ComplexField, Matrix2, RealField, Vector2, Vector3};
use num_traits::{Signed, Zero};
use rust_decimal_macros::dec;

use crate::{decimal::Dec, indexes::geo_index::seg::SegRef, planar::plane::Plane};

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

    pub(crate) fn get_intersection_params_seg_ref_3(&self, to: &SegRef<'_>) -> Option<(Dec, Dec)> {
        let segment_dir = to.dir().normalize();
        let q = self.origin - to.from();

        let dot = self.dir.dot(&segment_dir);

        let m = Matrix2::new(Dec::from(1), -dot, dot, -Dec::from(1));
        let b = -Vector2::new(q.dot(&self.dir), q.dot(&segment_dir));

        if let Some(mi) = m.try_inverse() {
            let st = mi * b;
            Some((st.x, st.y / to.dir().magnitude()))
        } else {
            None
        }
    }

    pub(crate) fn get_intersection_params_seg_ref_2(&self, to: &SegRef<'_>) -> Option<(Dec, Dec)> {
        let normalized_seg_dir = to.dir().normalize();
        let common_plane_normal = self.dir.cross(&normalized_seg_dir);
        if common_plane_normal.magnitude_squared() > Dec::zero() {
            let common_plane_normal = common_plane_normal.normalize();
            let line_plane_normal = common_plane_normal.cross(&self.dir).normalize();
            let plane = Plane::new_from_normal_and_point(line_plane_normal, self.origin);

            let from_dist = plane.normal().dot(&to.from()) - plane.d();
            let to_dist = plane.normal().dot(&to.to()) - plane.d();
            if to.rib_id() == 3556 {
                println!("from_dist: {from_dist}/ {to_dist}");
            }

            if from_dist.is_sign_positive() != to_dist.is_sign_positive() {
                let b = from_dist.abs() / (from_dist.abs() + to_dist.abs());

                let p = to.from().lerp(&to.to(), b);
                let a = (p - self.origin).dot(&self.dir);
                //println!("!!!!!{b}, {a} !!!! ");
                return Some((a, b));
            }
            //println!(" >>>> f: {from_dist}, to: {to_dist}");

            None
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
