use geometry::primitives::origin::Origin;

use super::button::Button;

#[derive(Clone, Debug)]
pub(crate) struct ButtonsColumn {
    buttons: Vec<Button>,
    top_button_ix: usize,
    bottom_button_ix: usize,
    pub(crate) origin: Origin,
}

impl ButtonsColumn {
    pub(crate) fn new(origin: Origin) -> Self {
        Self {
            top_button_ix: 0,
            origin,
            bottom_button_ix: 0,
            buttons: Vec::new(),
        }
    }

    pub(crate) fn chocs(mut self, mut buttons: Vec<Button>) -> Self {
        self.buttons = buttons;
        self.bottom_button_ix = self.buttons.len() - 1;
        self
    }

    pub(crate) fn buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        Box::new(self.buttons.iter().cloned().map(|mut b| {
            b.origin.add(&self.origin);
            b
        }))
    }

    pub(crate) fn top(&self) -> Option<Button> {
        self.buttons.get(self.top_button_ix).cloned().map(|mut b| {
            b.origin.add(&self.origin);
            b
        })
    }
    pub(crate) fn bottom(&self) -> Option<Button> {
        self.buttons
            .get(self.bottom_button_ix)
            .cloned()
            .map(|mut b| {
                b.origin.add(&self.origin);
                b
            })
    }
}
