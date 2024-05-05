use geometry::origin::Origin;

use crate::buttons_column_builder::ButtonsColumnBuilder;

use super::button::Button;

#[derive(Clone, Debug)]
pub struct ButtonsColumn {
    pub(super) buttons: Vec<Button>,
}

impl ButtonsColumn {
    pub fn build() -> ButtonsColumnBuilder {
        ButtonsColumnBuilder::default()
    }

    pub(crate) fn buttons(&self) -> impl Iterator<Item = &Button> {
        self.buttons.iter()
    }

    pub(crate) fn top(&self) -> Option<Button> {
        self.buttons.first().cloned()
    }

    pub(crate) fn bottom(&self) -> Option<Button> {
        self.buttons.last().cloned()
    }
    pub(crate) fn apply_origin(&mut self, origin: &Origin) {
        for b in self.buttons.iter_mut() {
            b.origin.apply(origin);
        }
    }
}
