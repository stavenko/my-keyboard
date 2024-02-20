#[derive(Debug, PartialEq, Eq)]
pub struct SplitResult<Item> {
    pub front: Vec<Item>,
    pub back: Vec<Item>,
    pub coplanar_back: Vec<Item>,
    pub coplanar_front: Vec<Item>,
}

impl<Item> Default for SplitResult<Item> {
    fn default() -> Self {
        Self {
            front: Default::default(),
            back: Default::default(),
            coplanar_back: Default::default(),
            coplanar_front: Default::default(),
        }
    }
}

impl<Item> SplitResult<Item> {
    pub fn front(mut self, segment: Item) -> Self {
        self.front.push(segment);
        self
    }
    pub fn fronts(mut self, mut segment: Vec<Item>) -> Self {
        self.front.append(&mut segment);
        self
    }
    pub fn back(mut self, segment: Item) -> Self {
        self.back.push(segment);
        self
    }
    pub fn backs(mut self, mut segment: Vec<Item>) -> Self {
        self.back.append(&mut segment);
        self
    }
    pub fn coplanar_back(mut self, segment: Item) -> Self {
        self.coplanar_back.push(segment);
        self
    }
    pub fn coplanar_backs(mut self, mut segment: Vec<Item>) -> Self {
        self.coplanar_back.append(&mut segment);
        self
    }
    pub fn coplanar_front(mut self, segment: Item) -> Self {
        self.coplanar_front.push(segment);
        self
    }
    pub fn coplanar_fronts(mut self, mut segment: Vec<Item>) -> Self {
        self.coplanar_front.append(&mut segment);
        self
    }
}

#[derive(Debug, PartialEq)]
pub enum Location {
    Front,
    Back,
    Coplanar,
}

pub trait Splitter<Item> {
    fn split(&self, item: Item) -> SplitResult<Item>;
    fn locate(&self, item: Item) -> Location;
    fn from_item(item: &Item) -> Self;
}
