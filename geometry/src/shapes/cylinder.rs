use nalgebra::{ComplexField, Vector3};
use num_traits::Zero;
use rust_decimal::Decimal;

use crate::{
    decimal::Dec, geometry::GeometryDyn, indexes::geo_index::mesh::MeshRefMut, origin::Origin,
};

#[derive(Clone)]
pub struct Cylinder {
    top_basis: Origin,
    steps: usize,
    top_cap: bool,
    bottom_cap: bool,
    radius: Dec,
    height: Dec,
}

impl Cylinder {
    pub fn centered(origin: Origin, height: impl Into<Dec>, radius: impl Into<Dec>) -> Self {
        let radius = radius.into();
        let height = height.into();
        let top_basis = origin.clone().offset_z(height / 2);

        Self {
            top_basis,
            steps: 10,
            top_cap: true,
            bottom_cap: true,
            radius,
            height,
        }
    }

    pub fn top_cap(mut self, top_cap: bool) -> Self {
        self.top_cap = top_cap;
        self
    }

    pub fn bottom_cap(mut self, bottom_cap: bool) -> Self {
        self.bottom_cap = bottom_cap;
        self
    }

    pub fn radius(mut self, radius: Dec) -> Self {
        self.radius = radius;
        self
    }

    pub fn steps(mut self, steps: usize) -> Self {
        self.steps = steps;
        self
    }

    pub fn with_top_at(origin: Origin, height: impl Into<Dec>, radius: impl Into<Dec>) -> Self {
        let radius = radius.into();
        let height = height.into();
        let top_basis = origin.clone();

        Self {
            top_basis,
            steps: 10,
            top_cap: true,
            bottom_cap: true,
            radius,
            height,
        }
    }

    pub fn with_bottom_at(origin: Origin, height: Dec, radius: Dec) -> Self {
        let top_basis = origin.clone().offset_z(height);

        Self {
            top_basis,
            steps: 10,
            top_cap: true,
            bottom_cap: true,
            radius,
            height,
        }
    }

    pub fn render(&self) -> Vec<Vec<Vector3<Dec>>> {
        let up = self.top_basis.z();

        let mut top = Vec::new();
        let mut bottom = Vec::new();
        let mut wall = Vec::new();
        let from = Dec::zero();
        for (prev, next) in (0..self.steps).zip(1..=self.steps) {
            let angle_prev =
                Dec::from(prev) / Dec::from(self.steps) * Dec::from(Decimal::TWO_PI) - from;
            let angle_next =
                Dec::from(next) / Dec::from(self.steps) * Dec::from(Decimal::TWO_PI) - from;

            let top_prev = self.top_basis.center
                + self.top_basis.x() * angle_prev.cos() * self.radius
                + self.top_basis.y() * angle_prev.sin() * self.radius;

            let top_next = self.top_basis.center
                + self.top_basis.x() * angle_next.cos() * self.radius
                + self.top_basis.y() * angle_next.sin() * self.radius;

            let bottom_prev = top_prev - (up * self.height);
            let bottom_next = top_next - (up * self.height);

            wall.push(vec![bottom_prev, bottom_next, top_next, top_prev]);

            top.push(top_prev);
            bottom.push(bottom_prev);
        }

        if self.top_cap {
            wall.push(top);
        }

        if self.bottom_cap {
            bottom.reverse();
            wall.push(bottom);
        }

        wall
    }
}

impl GeometryDyn for Cylinder {
    fn polygonize(&self, mut mesh: MeshRefMut, _complexity: usize) -> anyhow::Result<()> {
        for p in self.render() {
            mesh.add_polygon(&p)?;
        }

        Ok(())
    }
}
