pub trait SplitHyperLine<Scalar> {
    fn split_hyper_line(&self, t: Scalar) -> (Self, Self)
    where
        Self: Sized;
}
