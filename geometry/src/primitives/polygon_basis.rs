use core::fmt;

use nalgebra::{Vector2, Vector3};

use super::decimal::Dec;

#[derive(Clone)]
pub struct PolygonBasis {
    pub center: Vector3<Dec>,
    pub x: Vector3<Dec>,
    pub y: Vector3<Dec>,
}

impl fmt::Debug for PolygonBasis {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "  o {} {} {}",
            self.center.x, self.center.y, self.center.z
        )?;
        writeln!(f, "  x {} {} {}", self.x.x, self.x.y, self.x.z)?;
        writeln!(f, "  y {} {} {}", self.y.x, self.y.y, self.y.z)?;
        Ok(())
    }
}
impl PolygonBasis {
    pub fn project_on_plane_z(&self, point: &Vector3<Dec>) -> Vector2<Dec> {
        let PolygonBasis { center, x, y, .. } = self;
        let x = (point - center).dot(x);
        let y = (point - center).dot(y);
        Vector2::new(x, y)
    }

    pub fn unproject(&self, point: &Vector2<Dec>) -> Vector3<Dec> {
        let PolygonBasis { center, x, y, .. } = self;
        center + x * point.x + y * point.y
    }
}
