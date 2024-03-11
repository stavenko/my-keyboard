use nalgebra::Vector2;

pub trait Intersects<T> {
    type Out;
    fn intersects(&self, other: &T) -> Option<Self::Out>;
}

impl Intersects<Segment2D> for Vector2<Dec> {
    type Out = bool;

    fn intersects(&self, other: &Segment2D) -> Option<Self::Out> {
        todo!()
    }
}
