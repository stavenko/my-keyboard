pub trait Relation<To> {
    type Relate;
    fn relate(&self, to: &To) -> Self::Relate;
}
