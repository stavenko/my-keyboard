use nalgebra::{Vector2, Vector3};

use crate::geometry::path::{segment::EdgeSegment, Path};

use super::{Four, GenericBoundedSurface, GetBoundingPath, InverseSurface, Topology};

pub struct SurfaceBetweenTwoPaths<PL, PR>
where
    PL: Path,
    PR: Path,
{
    pub left: PL,
    pub right: PR,
}

impl<PL, PR> InverseSurface for SurfaceBetweenTwoPaths<PL, PR>
where
    PL: Path,
    PR: Path,
{
    type Inverted = SurfaceBetweenTwoPaths<PR, PL>;

    fn inverse_surface(self) -> Self::Inverted {
        SurfaceBetweenTwoPaths {
            left: self.right,
            right: self.left,
        }
    }
}

impl<PL, PR> GenericBoundedSurface<Four> for SurfaceBetweenTwoPaths<PL, PR>
where
    PL: Path,
    PR: Path,
{
    fn get_point(&self, par: Vector2<f32>) -> anyhow::Result<nalgebra::Vector3<f32>> {
        let left = self.left.get_t(par.x);
        let right = self.right.get_t(par.x);
        Ok(left.lerp(&right, par.y))
    }
}

impl<A, B> GetBoundingPath<0> for SurfaceBetweenTwoPaths<A, B>
where
    A: Path + Clone,
    B: Path,
{
    fn get_bounding_path(&self) -> A {
        self.left.clone()
    }

    type Path = A;
}
impl<A, B> GetBoundingPath<1> for SurfaceBetweenTwoPaths<A, B>
where
    A: Path,
    B: Path + Clone,
{
    fn get_bounding_path(&self) -> B {
        self.right.clone()
    }

    type Path = B;
}

impl<A, B> GetBoundingPath<2> for SurfaceBetweenTwoPaths<A, B>
where
    A: Path,
    B: Path,
{
    fn get_bounding_path(&self) -> EdgeSegment {
        let top1 = self.left.first();
        let top2 = self.right.first();
        EdgeSegment {
            from: top1,
            to: top2,
            edge_from: Vector3::zeros(),
            edge_to: Vector3::zeros(),
        }
    }
    type Path = EdgeSegment;
}
impl<A, B> GetBoundingPath<3> for SurfaceBetweenTwoPaths<A, B>
where
    A: Path,
    B: Path,
{
    fn get_bounding_path(&self) -> EdgeSegment {
        let bottom1 = self.left.last();
        let bottom2 = self.right.last();
        EdgeSegment {
            from: bottom1,
            to: bottom2,
            edge_from: Vector3::zeros(),
            edge_to: Vector3::zeros(),
        }
    }
    type Path = EdgeSegment;
}
