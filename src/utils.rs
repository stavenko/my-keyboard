pub(crate) struct If<const B: bool>;
pub(crate) trait True {}
impl True for If<true> {}
