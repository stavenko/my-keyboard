use geometry::{
    decimal::Dec,
    hyper_path::{
        hyper_path::{HyperPath, Root},
        hyper_point::SuperPoint,
    },
};

use crate::{button_collections::ButtonsCollection, keyboard_config::RightKeyboardConfig};

/*
pub struct Props{
    collection_side: Side,
    collection: Collection,
}
*/

#[derive(Default)]
pub struct KeyboardBuilder {
    main: Option<ButtonsCollection>,
    thumb: Option<ButtonsCollection>,
    table_outline: Option<Root<SuperPoint<Dec>>>,
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
            table_outline: self.table_outline.expect("Must have outline on the table"),
        }
    }

    pub fn table_outline(mut self, hp: Root<SuperPoint<Dec>>) -> Self {
        self.table_outline = Some(hp);
        self
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
