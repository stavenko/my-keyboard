use std::{collections::HashMap, rc::Rc};

use geometry::{
    decimal::Dec,
    geometry::GeometryDyn,
    hyper_path::{hyper_path::Root, hyper_point::SuperPoint},
};

use crate::{
    bolt_point::BoltPoint,
    button_collections::ButtonsCollection,
    hole::Hole,
    keyboard_config::{KeyboardMesh, MaterialAddition, RightKeyboardConfig},
};

#[derive(Default)]
#[allow(clippy::type_complexity)]
pub struct KeyboardBuilder {
    main: Option<ButtonsCollection>,
    thumb: Option<ButtonsCollection>,
    table_outline: Option<Root<SuperPoint<Dec>>>,
    //bolts: Vec<BoltPoint>,
    wall_thickness: Dec,
    bottom_thickness: Dec,
    wall_extension: Dec,
    //bottom_holes: Vec<Hole>,
    //main_holes: Vec<Hole>,
    holes: HashMap<KeyboardMesh, Vec<Rc<dyn GeometryDyn>>>,
    material: HashMap<KeyboardMesh, Vec<(MaterialAddition, Rc<dyn GeometryDyn>)>>,
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
            //bolt_points: self.bolts,
            holes: self.holes.into_iter().collect(),
            additional_material: self.material,
        }
    }

    pub fn add_main_hole(mut self, hole: Hole) -> Self {
        save_index(&mut self.holes, KeyboardMesh::ButtonsHull, hole.shape);
        self
    }

    pub fn add_bottom_hole(mut self, hole: Hole) -> Self {
        save_index(&mut self.holes, KeyboardMesh::Bottom, hole.shape);
        self
    }

    pub fn add_bolt(
        mut self,
        head_on: KeyboardMesh,
        thread_on: KeyboardMesh,
        bolt_point: BoltPoint,
    ) -> Self {
        let head_material = (
            MaterialAddition::InnerSurface,
            rc(bolt_point.get_head_material()),
        );
        let tail_material = (
            MaterialAddition::InnerSurface,
            rc(bolt_point.get_tail_material()),
        );
        save_index(&mut self.material, head_on, head_material);
        save_index(&mut self.material, thread_on, tail_material);

        save_index(&mut self.holes, head_on, rc(bolt_point.get_head_hole()));
        save_index(
            &mut self.holes,
            head_on,
            rc(bolt_point.get_head_thread_hole()),
        );

        if let Some(nut) = bolt_point.get_tail_nut_hole() {
            save_index(&mut self.holes, thread_on, rc(nut));
        }

        save_index(
            &mut self.holes,
            thread_on,
            rc(bolt_point.get_tail_thread_hole()),
        );

        // self.bolts.push(bolt_point);
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

fn rc(t: impl GeometryDyn + 'static) -> Rc<dyn GeometryDyn> {
    Rc::new(t)
}
fn save_index<Ix, Item>(index: &mut HashMap<Ix, Vec<Item>>, ix: Ix, item: Item)
where
    Ix: std::hash::Hash + Eq,
{
    if let Some(items) = index.get_mut(&ix) {
        items.push(item);
    } else {
        index.insert(ix, vec![item]);
    }
}
