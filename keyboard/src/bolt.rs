use geometry::decimal::Dec;
use rust_decimal_macros::dec;

use crate::bolt_builder::BoltBuilder;

#[derive(Clone)]
pub struct Bolt {
    pub(crate) head_diameter: Dec,
    pub(crate) diameter: Dec,
    pub(crate) head_height: Dec,
    /// Height of whole bolt without head
    pub(crate) height: Dec,
    pub(crate) thread_inner_diameter: Option<Dec>,
    pub(crate) nut: Option<Nut>,
}

impl Bolt {
    pub fn build() -> BoltBuilder {
        BoltBuilder::default()
    }
}

#[derive(Clone)]
pub enum Nut {
    Hex { outer_diameter: Dec, height: Dec },
}

impl Nut {
    pub fn height(&self) -> Dec {
        match self {
            Nut::Hex { height, .. } => *height,
        }
    }
    pub fn m2_hex() -> Self {
        let outer_diameter = dec!(4) / dec!(3) * dec!(4);
        Self::Hex {
            outer_diameter: outer_diameter.into(),
            height: Dec::from(1),
        }
    }
}
