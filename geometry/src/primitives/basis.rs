use core::fmt;

use nalgebra::{Vector2, Vector3};

use super::{decimal::Dec, plane::Plane, polygon_basis::PolygonBasis};

#[derive(Clone)]
pub struct Basis {
    pub center: Vector3<Dec>,
    pub x: Vector3<Dec>,
    pub y: Vector3<Dec>,
    pub z: Vector3<Dec>,
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
