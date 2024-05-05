use stl_io::Triangle;

impl Iterator for TriIter {
    type Item = Triangle;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next()
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.size, Some(self.size))
    }
}

pub struct TriIter {
    pub(super) inner: <Vec<Triangle> as IntoIterator>::IntoIter,
    pub(super) size: usize,
}
impl ExactSizeIterator for TriIter {}
