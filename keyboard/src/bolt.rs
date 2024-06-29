use geometry::{
    decimal::Dec,
    indexes::geo_index::{index::GeoIndex, mesh::MeshId},
    origin::Origin,
    shapes,
};
use rust_decimal_macros::dec;

use crate::bolt_builder::BoltBuilder;

#[derive(Clone)]
pub struct Bolt {
    pub(crate) head_diameter: Dec,
    pub(crate) diameter: Dec,
    pub(crate) head_height: Dec,
    /// Height of whole bolt without head
    pub(crate) height: Dec,
    pub(crate) nut: Option<Nut>,
}

impl Bolt {
    pub fn new() -> BoltBuilder {
        BoltBuilder::default()
    }
}

#[derive(Clone)]
pub enum Nut {
    Hex { outer_diameter: Dec, height: Dec },
}

impl Nut {
    pub fn m2_hex() -> Self {
        let outer_diameter = dec!(4) / dec!(3) * dec!(4);
        Self::Hex {
            outer_diameter: outer_diameter.into(),
            height: Dec::from(1),
        }
    }
}
