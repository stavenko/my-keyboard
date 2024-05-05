use geometry::decimal::Dec;

use crate::{button_collections::ButtonsCollection, keyboard_config::RightKeyboardConfig};

#[derive(Default)]
pub struct KeyboardBuilder {
    main: Option<ButtonsCollection>,
    thumb: Option<ButtonsCollection>,
    wall_thickness: Dec,
    wall_extension: Dec,
}

impl KeyboardBuilder {
    pub fn build(self) -> RightKeyboardConfig {
        let main_buttons = self.main.unwrap_or(ButtonsCollection::empty());
        let thumb_buttons = self.thumb.unwrap_or(ButtonsCollection::empty());

        RightKeyboardConfig {
            main_buttons,
            thumb_buttons,
            main_plane_thickness: self.wall_thickness,
        }
    }

    pub fn main(mut self, button_collections: ButtonsCollection) -> Self {
        self.main = Some(button_collections);
        self
    }

    pub fn thumb(mut self, button_collections: ButtonsCollection) -> Self {
        self.thumb = Some(button_collections);
        self
    }

    pub fn wall_thickness(mut self, wall_thickness: Dec) -> Self {
        self.wall_thickness = wall_thickness;
        self
    }

    pub fn wall_extension(mut self, wall_extension: Dec) -> Self {
        self.wall_extension = wall_extension;
        self
    }
}
