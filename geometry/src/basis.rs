use crate::planar::plane::Plane;
use core::fmt;

use nalgebra::{Vector2, Vector3};

use crate::decimal::Dec;

use super::polygon_basis::PolygonBasis;

#[derive(Clone)]
pub struct Basis {
    center: Vector3<Dec>,
    x: Vector3<Dec>,
    y: Vector3<Dec>,
    z: Vector3<Dec>,
}

impl fmt::Debug for Basis {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  o {} {} {}",
            self.center.x, self.center.y, self.center.z
        )?;
        writeln!(f, "  x {} {} {}", self.x.x, self.x.y, self.x.z)?;
        writeln!(f, "  y {} {} {}", self.y.x, self.y.y, self.y.z)?;
        writeln!(f, "  y {} {} {}", self.z.x, self.z.y, self.z.z)?;
        Ok(())
    }
}
impl Basis {
    pub fn x(&self) -> Vector3<Dec> {
        self.x
    }
    pub fn y(&self) -> Vector3<Dec> {
        self.y
    }
    pub fn z(&self) -> Vector3<Dec> {
        self.z
    }
    pub fn center(&self) -> Vector3<Dec> {
        self.center
    }

    pub fn new(
        x: Vector3<Dec>,
        y: Vector3<Dec>,
        z: Vector3<Dec>,
        center: Vector3<Dec>,
    ) -> anyhow::Result<Self> {
        let cross = z.cross(&x);
        if cross != y {
            Err(anyhow::anyhow!("None-right basis is not supported"))
        } else {
            Ok(Self { center, x, y, z })
        }
    }
    pub fn project_on_plane_z(&self, point: &Vector3<Dec>) -> Vector2<Dec> {
        let Basis { center, x, y, .. } = self;
        let x = (point - center).dot(x);
        let y = (point - center).dot(y);
        Vector2::new(x, y)
    }

    pub fn unproject(&self, point: &Vector2<Dec>) -> Vector3<Dec> {
        let Basis { center, x, y, .. } = self;
        center + x * point.x + y * point.y
    }

    pub fn get_polygon_basis(&self) -> PolygonBasis {
        PolygonBasis {
            center: self.center,
            x: self.x,
            y: self.y,
        }
    }

    pub fn xy(&self) -> Plane {
        Plane::new_from_normal_and_point(self.z, self.center)
    }
}
