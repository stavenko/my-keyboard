use std::fmt;

use nalgebra::{ComplexField, Vector3};
use num_traits::{One, Signed, Zero};

use crate::{
    decimal::{Dec, STABILITY_ROUNDING},
    reversable::Reversable,
};

#[derive(Clone, Eq, PartialOrd)]
pub struct Plane {
    normal: Vector3<Dec>,
    d: Dec,
}

impl PartialEq for Plane {
    fn eq(&self, other: &Self) -> bool {
        (self.d - other.d).round_dp(STABILITY_ROUNDING).is_zero()
            && self
                .normal
                .dot(&other.normal)
                .round_dp(STABILITY_ROUNDING)
                .is_one()
    }
}

impl fmt::Debug for Plane {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}x  {}y {}z {}",
            self.normal.x.round_dp(STABILITY_ROUNDING),
            self.normal.y.round_dp(STABILITY_ROUNDING),
            self.normal.z.round_dp(STABILITY_ROUNDING),
            self.d.round_dp(STABILITY_ROUNDING)
        )
    }
}

impl Plane {
    pub fn new(a: Dec, b: Dec, c: Dec, d: Dec) -> Self {
        Self {
            normal: Vector3::new(a, b, c).normalize(),
            d,
        }
    }

    pub fn flip(&mut self) {
        self.normal = -self.normal;
        self.d = -self.d;
    }

    pub fn is_point_on_plane(&self, point: Vector3<Dec>, tolerance: impl Into<Dec>) -> bool {
        let p = self.normal.dot(&point) - self.d;
        p.abs() < tolerance.into()
    }

    pub fn get_intersection_param(&self, from: Vector3<Dec>, to: Vector3<Dec>) -> Option<Dec> {
        let from_p = self.normal.dot(&from) - self.d;
        let to_p = self.normal.dot(&to) - self.d;
        if (from_p * to_p).is_negative() {
            let total = from_p.abs() + to_p.abs();
            Some((from_p / total).abs())
        } else {
            None
        }
    }
    pub fn get_intersection_param2(&self, from: Vector3<Dec>, to: Vector3<Dec>) -> Option<Dec> {
        let from_p = self.normal.dot(&from) - self.d;
        let to_p = self.normal.dot(&to) - self.d;
        let sum = from_p - to_p;
        if sum.is_zero() {
            None
        } else {
            Some(from_p / sum)
        }
    }

    pub fn new_from_normal_and_point(normal: Vector3<Dec>, point: Vector3<Dec>) -> Self {
        let d = normal.dot(&point);

        Self { normal, d }
    }

    pub fn normal(&self) -> Vector3<Dec> {
        self.normal
    }

    pub fn d(&self) -> Dec {
        self.d
    }

    pub fn point_on_plane(&self) -> Vector3<Dec> {
        self.normal * self.d
    }

    pub(crate) fn project_unit(&self, unit: Vector3<Dec>) -> Option<Vector3<Dec>> {
        let dot = self.normal.dot(&unit);
        let ortho = self.normal * dot;
        let in_plane_vec = unit - ortho;

        if in_plane_vec.magnitude() == Dec::zero() {
            return None;
        }
        Some((unit - ortho).normalize())
    }

    pub(crate) fn flipped(mut self) -> Plane {
        self.normal = -self.normal;
        self.d = -self.d;
        self
    }
}
