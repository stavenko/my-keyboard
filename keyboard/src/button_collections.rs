use std::iter::empty;

use geometry::origin::Origin;

use super::{button::Button, ButtonsColumn};

#[derive(Debug)]
#[allow(unused)]
pub struct ButtonsCollection {
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

    fn columns(&self) -> impl Iterator<Item = ButtonsColumn> + '_ {
        self.columns.iter().map(|c| {
            let mut c = c.to_owned();
            c.origin.add(&self.origin);
            c
        })
    }
}

impl ButtonsCollection {
    pub fn with_columns(mut self, columns: Vec<ButtonsColumn>) -> Self {
        self.columns = columns;
        self.last_column_ix = self.columns.len() - 1;
        self
    }
    pub fn new(origin: Origin) -> Self {
        Self {
            origin,
            columns: Vec::new(),
            first_column_ix: 0,
            last_column_ix: 0,
        }
    }
    pub fn left_column(&self) -> Option<&ButtonsColumn> {
        self.columns.first()
    }
    pub fn right_column(&self) -> Option<&ButtonsColumn> {
        if !self.columns.is_empty() {
            self.columns.last()
        } else {
            None
        }
    }
}

pub trait ButtonsHull {
    fn columns(&self) -> impl Iterator<Item = ButtonsColumn> + '_;

    fn buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;
    fn right_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;

    fn left_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;

    fn top_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;

    fn bottom_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_>;
}
