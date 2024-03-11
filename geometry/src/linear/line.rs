use core::fmt;

use nalgebra::Vector3;

use crate::decimal::Dec;

#[derive(Clone)]
pub struct Line {
    pub origin: Vector3<Dec>,
    pub dir: Vector3<Dec>,
}

impl fmt::Debug for Line {
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
