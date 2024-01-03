use nalgebra::{Vector2, Vector3};

use crate::geometry::{
    bezier::bernstein,
    path::{bezier::BezierEdge, Path},
};

use super::{Four, GenericBoundedSurface, GetBoundingPath};

pub struct BoundedSurface<const O: usize, T: Path> {
    bounds: [T; O],
}

impl<T: Path> GenericBoundedSurface<Four> for BoundedSurface<2, T> {
    fn get_point(&self, par: Vector2<f32>) -> anyhow::Result<Vector3<f32>> {
        let p = self.get_curve(par.x);
        Ok(p.get_t(par.y))
    }
}

impl<const O: usize, T: Path + Clone> GetBoundingPath<0> for BoundedSurface<O, T> {
    type Path = T;

    fn get_bounding_path(&self) -> Self::Path {
        self.bounds[0].clone()
    }
}

impl<const O: usize, T: Path + Clone> GetBoundingPath<1> for BoundedSurface<O, T> {
    type Path = T;

    fn get_bounding_path(&self) -> Self::Path {
        self.bounds[1].clone()
    }
}

impl<T: Path> GetBoundingPath<2> for BoundedSurface<2, T> {
    type Path = BezierEdge;

    fn get_bounding_path(&self) -> Self::Path {
        self.get_top()
    }
}

impl<T: Path> GetBoundingPath<3> for BoundedSurface<2, T> {
    type Path = BezierEdge;

    fn get_bounding_path(&self) -> Self::Path {
        self.get_bottom()
    }
}

impl<T: Path> BoundedSurface<2, T> {
    pub fn new(a: T, b: T) -> Self {
        // left, right
        Self { bounds: [a, b] }
    }

    pub fn get_curve(&self, s: f32) -> BezierEdge {
        let left = self.bounds[0].get_t(s);
        let left_w = self.bounds[0].get_edge_dir(s) + left;
        let right = self.bounds[1].get_t(s);
        let right_w = self.bounds[1].get_edge_dir(s) + right;
        let b = self.bounds[0].get_t(0.0);
        let e = self.bounds[0].get_t(1.0);
        let dirl = (e - b).normalize();
        let b = self.bounds[1].get_t(0.0);
        let e = self.bounds[1].get_t(1.0);
        let dirr = (e - b).normalize();
        let dir1 = dirl.lerp(&dirr, 0.333).normalize();
        let dir2 = dirl.lerp(&dirr, 0.666).normalize();

        BezierEdge::new([left, left_w, right_w, right], [dirl, dir1, dir2, dirr])
    }

    pub fn get_curve_with_edge_force(&self, s: f32, edge_force: f32) -> BezierEdge {
        let left = self.bounds[0].get_t(s);
        let left_w = self.bounds[0].get_edge_dir(s) + left;
        let right = self.bounds[1].get_t(s);
        let right_w = self.bounds[1].get_edge_dir(s) + right;
        let b = self.bounds[0].get_t(0.0);
        let e = self.bounds[0].get_t(1.0);
        let dirl = (b - e).normalize();
        let b = self.bounds[1].get_t(0.0);
        let e = self.bounds[1].get_t(1.0);
        let dirr = (b - e).normalize();
        let dir1 = dirl.lerp(&dirr, 0.333).normalize() * edge_force;
        let dir2 = dirl.lerp(&dirr, 0.666).normalize() * edge_force;

        BezierEdge::new(
            [left, left_w, right_w, right],
            [dirl * edge_force, dir1, dir2, dirr * edge_force],
        )
    }

    pub fn get_top(&self) -> BezierEdge {
        self.get_curve(0.0)
    }

    pub fn get_bottom(&self) -> BezierEdge {
        self.get_curve(1.0)
    }
}

impl<T: Path> BoundedSurface<3, T> {
    pub fn new(a: T, b: T, c: T) -> Self {
        Self { bounds: [a, b, c] }
    }
}

impl<T: Path> BoundedSurface<4, T> {
    pub fn new(a: T, b: T, c: T, d: T) -> Self {
        // top, right, bottom, left
        Self {
            bounds: [a, b, c, d],
        }
    }
    pub fn get_point_p(&self, t: f32, s: f32) -> Vector3<f32> {
        let ot = 1.0 - t;
        let os = 1.0 - s;
        let top = self.bounds[0].get_t(s);
        let right = self.bounds[1].get_t(ot);
        let bottom = self.bounds[2].get_t(os);
        let left = self.bounds[3].get_t(t);
        let top_w = self.bounds[0].get_edge_dir(s) + top;
        let right_w = self.bounds[1].get_edge_dir(ot) + right;
        let bottom_w = self.bounds[2].get_edge_dir(os) + bottom;
        let left_w = self.bounds[3].get_edge_dir(t) + left;

        let curves = [
            [top, top_w, bottom_w, bottom],
            [left, left_w, right_w, right],
        ];
        let weights = [
            [
                bernstein::<4>(0, t) * bernstein::<4>(0, s),
                bernstein::<4>(1, t) * bernstein::<4>(1, s),
                bernstein::<4>(2, t) * bernstein::<4>(2, s),
                bernstein::<4>(3, t) * bernstein::<4>(3, s),
            ],
            [
                bernstein::<4>(0, t) * bernstein::<4>(0, s),
                bernstein::<4>(1, t) * bernstein::<4>(1, s),
                bernstein::<4>(2, t) * bernstein::<4>(2, s),
                bernstein::<4>(3, t) * bernstein::<4>(3, s),
            ],
        ];

        curves
            .into_iter()
            .zip(weights)
            .flat_map(|(c, w)| c.into_iter().zip(w))
            .map(|(v, w)| v * w)
            .sum()
    }
}

impl<T: Path> BoundedSurface<5, T> {
    pub fn new(a: T, b: T, c: T, d: T, e: T) -> Self {
        Self {
            bounds: [a, b, c, d, e],
        }
    }
}
