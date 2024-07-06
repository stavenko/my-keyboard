use geometry::{
    decimal::Dec,
    hyper_path::{hyper_path::Root, hyper_point::SuperPoint},
};

use crate::{
    bolt_point::BoltPoint, button_collections::ButtonsCollection,
    keyboard_config::RightKeyboardConfig,
};

#[derive(Default)]
pub struct KeyboardBuilder {
    main: Option<ButtonsCollection>,
    thumb: Option<ButtonsCollection>,
    table_outline: Option<Root<SuperPoint<Dec>>>,
    bolts: Vec<BoltPoint>,
    wall_thickness: Dec,
    bottom_thickness: Dec,
    wall_extension: Dec,
}

impl KeyboardBuilder {
    pub fn build(self) -> RightKeyboardConfig {
        let main_buttons = self.main.unwrap_or(ButtonsCollection::empty());
        let thumb_buttons = self.thumb.unwrap_or(ButtonsCollection::empty());

        RightKeyboardConfig {
            main_buttons,
            thumb_buttons,
            bottom_thickness: self.bottom_thickness,
            main_plane_thickness: self.wall_thickness,
            table_outline: self.table_outline.expect("Must have outline on the table"),
            bolt_points: self.bolts,
        }
    }

    pub fn add_bolt(mut self, bolt_point: BoltPoint) -> Self {
        self.bolts.push(bolt_point);
        self
    }

    pub fn table_outline(mut self, hp: Root<SuperPoint<Dec>>) -> Self {
        self.table_outline = Some(hp);
        self
    }

    pub fn bottom_thickness(mut self, bottom_thickness: impl Into<Dec>) -> Self {
        self.bottom_thickness = bottom_thickness.into();
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

    pub fn wall_thickness(mut self, wall_thickness: impl Into<Dec>) -> Self {
        self.wall_thickness = wall_thickness.into();
        self
    }

    pub fn wall_extension(mut self, wall_extension: impl Into<Dec>) -> Self {
        self.wall_extension = wall_extension.into();
        self
    }
}
