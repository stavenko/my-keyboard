use std::iter::empty;

use itertools::Itertools;

use crate::geometry::primitives::origin::Origin;

use super::{button::Button, ButtonsColumn};

pub struct ButtonsCollection {
    thickness: f32,
    pub origin: Origin,
    columns: Vec<ButtonsColumn>,
    first_column_ix: usize,
    last_column_ix: usize,
}

impl ButtonsHull for ButtonsCollection {
    fn buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        Box::new(
            self.columns
                .iter()
                .flat_map(|column| column.buttons())
                .map(|mut b| {
                    b.origin.add(&self.origin);
                    b
                }),
        )
    }

    fn right_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        let last_column_ix = self.last_column_ix;
        if let Some(right_column) = self.columns.get(last_column_ix) {
            Box::new(right_column.buttons().map(|mut b| {
                b.origin.add(&self.origin);
                b
            }))
        } else {
            Box::new(empty())
        }
    }

    fn left_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        let first_column_ix = self.first_column_ix;
        if let Some(right_column) = self.columns.get(first_column_ix) {
            Box::new(right_column.buttons().map(|mut b| {
                b.origin.add(&self.origin);
                b
            }))
        } else {
            Box::new(empty())
        }
    }

    fn top_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        Box::new(self.columns.iter().filter_map(|c| c.top()).map(|mut b| {
            b.origin.add(&self.origin);
            b
        }))
    }

    fn bottom_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        Box::new(self.columns.iter().filter_map(|c| c.bottom()).map(|mut b| {
            b.origin.add(&self.origin);
            b
        }))
    }
}

impl ButtonsCollection {
    pub fn columns(mut self, columns: Vec<ButtonsColumn>) -> Self {
        self.columns = columns;
        self.last_column_ix = self.columns.len() - 1;
        self
    }
    pub fn new(origin: Origin, thickness: f32) -> Self {
        Self {
            origin,
            thickness,
            columns: Vec::new(),
            first_column_ix: 0,
            last_column_ix: 0,
        }
    }
    pub(crate) fn left_column(&self) -> Option<&ButtonsColumn> {
        self.columns.get(0)
    }
    pub(crate) fn right_column(&self) -> Option<&ButtonsColumn> {
        if self.columns.len() > 0 {
            self.columns.get(self.columns.len() - 1)
        } else {
            None
        }
    }
}

pub trait ButtonsHull {
    fn buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;
    fn right_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;

    fn left_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;

    fn top_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;

    fn bottom_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;
}
