use std::iter::Peekable;

pub struct NextAndPeek<I, O>
where
    I: Iterator,
{
    inner: Peekable<I>,
    constructor: Box<dyn Fn(&I::Item, &I::Item) -> O>,
}

impl<I, O> Iterator for NextAndPeek<I, O>
where
    I: Iterator,
{
    type Item = O;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(item) = self.inner.next() {
            if let Some(next_item) = self.inner.peek() {
                let item = (self.constructor)(&item, next_item);
                return Some(item);
            }
        }
        None
    }
}
pub trait NextAndPeekBlank: Iterator + Sized {
    fn next_and_peek<O>(
        self,
        constructor: impl Fn(&Self::Item, &Self::Item) -> O + 'static,
    ) -> NextAndPeek<Self, O>;
}

impl<T: Iterator> NextAndPeekBlank for T {
    fn next_and_peek<O>(
        self,
        constructor: impl Fn(&Self::Item, &Self::Item) -> O + 'static,
    ) -> NextAndPeek<Self, O> {
        NextAndPeek {
            inner: self.peekable(),
            constructor: Box::new(constructor),
        }
    }
}
