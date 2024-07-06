use geometry::{
    decimal::Dec,
    indexes::geo_index::{index::GeoIndex, mesh::MeshId},
    origin::Origin,
    shapes::Cylinder,
};
use num_traits::Zero;
use rust_decimal_macros::dec;

use crate::bolt::Bolt;

pub struct BoltPoint {
    pub(crate) origin: Origin,
    /// amount of material between head bottom surface and empty space
    /// Amount of material, which make all the rigidness
    pub(crate) thread_material_gap: Dec, //
    pub(crate) bolt: Bolt,

    pub(crate) head_up_extension: Dec,

    pub(crate) thread_down_extension: Dec,

    pub(crate) nut_material_gap: Dec,

    pub(crate) radial_head_extension: Dec,
}

impl BoltPoint {
    pub fn new(bolt: Bolt) -> Self {
        Self {
            origin: Origin::new(),
            thread_material_gap: Zero::zero(),
            bolt,
            head_up_extension: Zero::zero(),
            radial_head_extension: Zero::zero(),
            thread_down_extension: Zero::zero(),
            nut_material_gap: Zero::zero(),
        }
    }

    pub fn origin(mut self, origin: Origin) -> Self {
        self.origin = origin;
        self
    }

    pub fn thread_material_gap(mut self, thread_material_gap: impl Into<Dec>) -> Self {
        self.thread_material_gap = thread_material_gap.into();
        self
    }

    pub fn nut_material_gap(mut self, nut_material_gap: impl Into<Dec>) -> Self {
        self.nut_material_gap = nut_material_gap.into();
        self
    }

    pub fn head_up_extension(mut self, head_up_extension: impl Into<Dec>) -> Self {
        self.head_up_extension = head_up_extension.into();
        self
    }

    pub fn thread_down_extension(mut self, thread_down_extension: impl Into<Dec>) -> Self {
        self.thread_down_extension = thread_down_extension.into();
        self
    }

    pub fn radial_head_extension(mut self, radial_head_extension: impl Into<Dec>) -> Self {
        self.radial_head_extension = radial_head_extension.into();
        self
    }

    pub(crate) fn get_bolt_head_material_bounds(
        &self,
        index: &mut GeoIndex,
    ) -> anyhow::Result<MeshId> {
        let mat_origin = self.origin.clone().offset_z(-self.thread_material_gap);
        let cylinder = Cylinder::with_bottom_at(
            mat_origin,
            self.bolt.head_height + self.head_up_extension,
            (self.bolt.head_diameter / Dec::from(2)) + self.radial_head_extension,
        )
        .top_cap(false);

        for poly in cylinder.render() {
            index.save_as_polygon(&poly, None)?;
        }

        Ok(index.get_current_default_mesh())
    }

    pub(crate) fn get_bolt_head_hole_bounds(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let cylinder = Cylinder::with_bottom_at(
            self.origin.clone(),
            self.bolt.head_height + self.head_up_extension,
            self.bolt.head_diameter / Dec::from(2),
        )
        .top_cap(false);

        for poly in cylinder.render().into_iter() {
            index.save_as_polygon(&poly, None)?;
        }

        Ok(index.get_current_default_mesh())
    }

    pub(crate) fn get_bolt_thread_head_hole_bounds(
        &self,
        index: &mut GeoIndex,
    ) -> anyhow::Result<MeshId> {
        let cylinder = Cylinder::with_top_at(
            self.origin.clone().offset_z(dec!(0.1)),
            self.head_up_extension * (Dec::from(105) / 100),
            self.bolt.diameter / Dec::from(2),
        )
        .top_cap(false)
        .bottom_cap(false);

        for poly in cylinder.render().into_iter()
        //.skip(1).take(1)
        {
            index.save_as_polygon(&poly, None)?;
        }

        Ok(index.get_current_default_mesh())
    }

    pub(crate) fn get_bolt_nut_well(&self, index: &mut GeoIndex) -> anyhow::Result<Option<MeshId>> {
        if let Some(nut) = self.bolt.nut.as_ref() {
            let mat_origin = self
                .origin
                .clone()
                .offset_z(-self.thread_material_gap - self.nut_material_gap);
            let shape = match nut {
                crate::bolt::Nut::Hex { outer_diameter, .. } => Cylinder::with_top_at(
                    mat_origin,
                    self.bolt.height + self.thread_down_extension,
                    *outer_diameter / Dec::from(2),
                )
                .steps(6)
                .bottom_cap(false),
            };
            for poly in shape.render().into_iter() {
                index.save_as_polygon(&poly, None)?;
            }

            Ok(Some(index.get_current_default_mesh()))
        } else {
            Ok(None)
        }
    }

    pub(crate) fn get_bolt_thread_hole(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let mat_origin = self.origin.clone().offset_z(-self.thread_material_gap);
        let cylinder = Cylinder::with_top_at(
            mat_origin.offset_z(dec!(0.1)),
            self.bolt.height,
            self.bolt.diameter / Dec::from(2),
        )
        .top_cap(false);

        for poly in cylinder.render().into_iter() {
            index.save_as_polygon(&poly, None)?;
        }
        Ok(index.get_current_default_mesh())
    }

    pub(crate) fn get_bolt_thread_material(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let diameter = if let Some(nut) = self.bolt.nut.as_ref() {
            match nut {
                crate::bolt::Nut::Hex { outer_diameter, .. } => {
                    *outer_diameter * Dec::from(dec!(1.1))
                }
            }
        } else {
            self.bolt.head_diameter
        };
        let mat_origin = self.origin.clone().offset_z(-self.thread_material_gap);
        let cylinder = Cylinder::with_top_at(
            mat_origin,
            self.bolt.height + self.thread_down_extension,
            (diameter / Dec::from(2)) + self.radial_head_extension,
        )
        .bottom_cap(false);

        for poly in cylinder.render() {
            index.save_as_polygon(&poly, None)?;
        }

        Ok(index.get_current_default_mesh())
    }
}
