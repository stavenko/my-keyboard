use std::iter::Peekable;

pub struct WithNext<I, O, F>
where
    I: Iterator,
    F: Fn(I::Item, Option<&I::Item>) -> O,
{
    inner: Peekable<I>,
    mapper: F,
}

impl<I, O, F> Iterator for WithNext<I, O, F>
where
    I: Iterator,
    F: Fn(I::Item, Option<&I::Item>) -> O,
{
    type Item = O;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(item) = self.inner.next() {
            let peeked = self.inner.peek();
            let item = (self.mapper)(item, peeked);
            return Some(item);
        }
        None
    }
}
pub trait WithNextBlank: Iterator + Sized {
    fn with_next<O, F>(self, mapper: F) -> WithNext<Self, O, F>
    where
        F: Fn(Self::Item, Option<&Self::Item>) -> O;
}

impl<T> WithNextBlank for T
where
    T: Iterator,
{
    fn with_next<O, F>(self, mapper: F) -> WithNext<Self, O, F>
    where
        F: Fn(Self::Item, Option<&Self::Item>) -> O,
    {
        WithNext {
            inner: self.peekable(),
            mapper,
        }
    }
}
