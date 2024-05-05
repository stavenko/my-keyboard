use geometry::{decimal::Dec, indexes::geo_index::index::GeoIndex};

use crate::{button_collections::ButtonsCollection, keyboard_builder::KeyboardBuilder};

pub struct RightKeyboardConfig {
    pub(crate) main_buttons: ButtonsCollection,
    pub(crate) thumb_buttons: ButtonsCollection,
    pub(crate) main_plane_thickness: Dec,
}

impl RightKeyboardConfig {
    pub fn build() -> KeyboardBuilder {
        KeyboardBuilder::default()
    }
    pub fn buttons(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        for b in self
            .main_buttons
            .buttons()
            .chain(self.thumb_buttons.buttons())
        {
            b.mesh(index, self.main_plane_thickness);
        }

        Ok(())
    }
}
