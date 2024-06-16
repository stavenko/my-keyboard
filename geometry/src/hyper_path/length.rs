use std::ops::Neg;

use nalgebra::{Dim, Matrix, Storage};


pub trait Length {
    type Scalar;
    fn length(&self) -> Self::Scalar;
}

impl<T, R, C, S> Length for Matrix<T, R, C, S>
where
    R: Dim,
    C: Dim,
    S: Storage<T, R, C>,
    T: nalgebra::Scalar + nalgebra::ComplexField + nalgebra::RealField,
    T: num_traits::NumOps + num_traits::Num + num_traits::NumAssignOps,
    T: Neg<Output = T>,
{
    type Scalar = T;

    fn length(&self) -> Self::Scalar {
        self.norm()
    }
}
