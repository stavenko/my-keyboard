use geometry::{
    decimal::{self, Dec},
    geometry::GeometryDyn,
    origin::Origin,
    shapes::Cylinder,
};
use num_traits::{One, Zero};
use rust_decimal_macros::dec;

use crate::bolt::Bolt;

pub struct BoltPoint {
    pub(crate) origin: Origin,
    /// amount of material between head bottom surface and empty space
    /// Amount of material, which make all the rigidness
    pub(crate) head_thread_material_gap: Dec, //
    pub(crate) bolt: Bolt,

    pub(crate) head_up_extension: Dec,

    pub(crate) thread_down_extension: Dec,

    // pub(crate) nut_material_gap: Dec,
    pub(crate) radial_head_material_extention: Dec,

    pub(crate) radial_head_hole_extention: Dec,

    pub(crate) thread_hole_radius_plastic_modification: Dec,
}

// const INNER_THREAD_MULTIPLIER: rust_decimal::Decimal = dec!(1.25);

impl BoltPoint {
    pub fn new(bolt: Bolt) -> Self {
        Self {
            origin: Origin::new(),
            head_thread_material_gap: bolt.head_height,
            bolt,
            head_up_extension: 30.into(),
            radial_head_material_extention: dec!(0.5).into(),
            radial_head_hole_extention: dec!(0.5).into(),
            thread_down_extension: 30.into(),
            thread_hole_radius_plastic_modification: Dec::from(1.5),
        }
    }

    /// Origin of bolt point is a point between two screwed parts.
    /// up direction of this point must contain bolt head
    /// down direction of this poitn must contain bolt thread part.
    pub fn origin(mut self, origin: Origin) -> Self {
        self.origin = origin;
        self
    }

    /// Material amount between bolt head and empty space
    pub fn head_thread_material_gap(mut self, head_thread_material_gap: impl Into<Dec>) -> Self {
        self.head_thread_material_gap = head_thread_material_gap.into();
        self
    }

    /// Increase (or decrease of hole due to 3D printing plastic problems
    /// Allows to make hole bigger and make bold with thread to move easily from
    /// printed hole
    pub fn thread_hole_radius_plastic_modification(
        mut self,
        hole_radius_plastic_modification: impl Into<Dec>,
    ) -> Self {
        self.thread_hole_radius_plastic_modification = hole_radius_plastic_modification.into();
        self
    }

    ///  Space between origin point and position of nut and bolt is tighten
    fn nut_material_gap(&self) -> Dec {
        if let Some(nut) = self.bolt.nut.as_ref() {
            println!(
                "bolt height --nut height: {} - {}",
                self.bolt.height,
                nut.height()
            );
            self.bolt.height - (nut.height() * (Dec::from(2)))
        } else {
            Dec::zero()
        }
    }

    ///
    pub fn head_up_extension(mut self, head_up_extension: impl Into<Dec>) -> Self {
        self.head_up_extension = head_up_extension.into();
        self
    }

    pub fn thread_down_extension(mut self, thread_down_extension: impl Into<Dec>) -> Self {
        self.thread_down_extension = thread_down_extension.into();
        self
    }

    /// Additional radius, which is added to head and to head hole
    pub fn radial_head_material_extention(
        mut self,
        radial_head_material_extention: impl Into<Dec>,
    ) -> Self {
        self.radial_head_material_extention = radial_head_material_extention.into();
        self
    }

    fn head_material_radius(&self) -> Dec {
        self.head_hole_radius() + self.radial_head_material_extention
    }

    fn head_hole_radius(&self) -> Dec {
        (self.bolt.head_diameter / Dec::from(2)) + self.radial_head_hole_extention
    }

    fn bolt_rest_height(&self) -> Dec {
        self.bolt.height - self.head_thread_material_gap
    }

    fn material_radius(&self) -> Dec {
        let tail_radius = if let Some(nut) = self.bolt.nut.as_ref() {
            match nut {
                crate::bolt::Nut::Hex { outer_diameter, .. } => {
                    *outer_diameter * Dec::from(dec!(1.1)) / 2
                }
            }
        } else {
            self.tail_thread_hole_radius() + self.radial_head_material_extention
        };

        self.head_material_radius().max(tail_radius)
    }

    fn head_thread_hole_radius(&self) -> Dec {
        let radius = self.bolt.diameter / 2;

        radius * self.thread_hole_radius_plastic_modification
    }

    fn tail_thread_hole_radius(&self) -> Dec {
        let radius = if self.bolt.nut.is_some() {
            self.bolt.diameter / 2
        } else {
            self.bolt
                .thread_inner_diameter
                .unwrap_or(self.bolt.diameter * Dec::from(dec!(0.8)))
                / 2
        };
        radius * self.thread_hole_radius_plastic_modification
    }

    pub(crate) fn get_head_material(&self) -> impl GeometryDyn {
        Cylinder::with_bottom_at(
            self.origin.clone(),
            self.bolt.head_height + self.head_up_extension,
            self.material_radius(),
        )
        .top_cap(false)
    }

    pub(crate) fn get_tail_material(&self) -> impl GeometryDyn {
        Cylinder::with_top_at(
            self.origin.clone(),
            self.bolt.height + self.thread_down_extension,
            self.material_radius(),
        )
        .bottom_cap(false)
    }

    pub(crate) fn get_head_hole(&self) -> impl GeometryDyn + Sized {
        Cylinder::with_bottom_at(
            self.origin.clone().offset_z(self.head_thread_material_gap),
            self.bolt.head_height + self.head_up_extension,
            self.head_hole_radius(),
        )
        .top_cap(false)
    }

    pub(crate) fn get_head_thread_hole(&self) -> impl GeometryDyn + Sized {
        Cylinder::with_bottom_at(
            self.origin.clone().offset_z(-dec!(0.1)),
            self.head_up_extension,
            dbg!(self.head_thread_hole_radius()),
        )
        .top_cap(false)
        .bottom_cap(false)
    }

    pub(crate) fn get_tail_nut_hole(&self) -> Option<impl GeometryDyn> {
        self.bolt.nut.as_ref().map(|nut| match nut {
            crate::bolt::Nut::Hex { outer_diameter, .. } => Cylinder::with_top_at(
                self.origin.clone().offset_z(-self.nut_material_gap()),
                self.bolt.height + self.thread_down_extension,
                *outer_diameter / Dec::from(2),
            )
            .steps(6)
            .bottom_cap(false),
        })
    }

    pub(crate) fn get_tail_thread_hole(&self) -> impl GeometryDyn {
        let cut_addition = if self.bolt.nut.is_some() {
            Dec::one()
        } else {
            Dec::zero()
        };

        let sw = Dec::from(dec!(0.1));
        Cylinder::with_top_at(
            self.origin.clone().offset_z(sw),
            self.bolt_rest_height() - sw + cut_addition,
            self.tail_thread_hole_radius(),
        )
        .top_cap(false)
    }
}
