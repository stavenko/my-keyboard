use std::{
    fmt::{self, Display},
    iter,
    ops::{Add, Div, Mul, Sub},
};

use nalgebra::{Dim, Matrix, Storage, Vector3};
use num_traits::{One, Pow, Zero};

use super::length::Length;

pub trait SideDir {
    type Vector;
    fn side_dir(&self) -> Self::Vector;
}

pub trait Point {
    type Vector;
    fn point(&self) -> Self::Vector;
    fn set_point(&mut self, v: Self::Vector);
}
pub trait Tensor: Zero + Add + Sub + Copy {
    type Scalar: Add
        + Sub
        + Mul
        + Div
        + Zero
        + One
        + nalgebra::Field
        + nalgebra::Scalar
        + fmt::Debug
        + Display
        + Copy
        + Pow<u16, Output = Self::Scalar>;
}

impl<T> Tensor for HyperPointT<T>
where
    T: Add
        + Sub
        + Mul
        + Div
        + Zero
        + One
        + nalgebra::Field
        + nalgebra::Scalar
        + fmt::Debug
        + Display
        + Copy
        + Pow<u16, Output = T>,
{
    type Scalar = T;
}

impl<T> Tensor for SuperPoint<T>
where
    T: Add
        + Sub
        + Mul
        + Div
        + Zero
        + One
        + nalgebra::Field
        + nalgebra::Scalar
        + fmt::Debug
        + Display
        + Copy
        + Pow<u16, Output = T>,
{
    type Scalar = T;
}

#[derive(Clone, Copy)]
pub struct HyperPointT<T> {
    pub normal: Vector3<T>,
    pub dir: Vector3<T>,
    pub point: Vector3<T>,
}

#[derive(Clone, Copy, Debug)]
pub struct SuperPoint<T> {
    pub side_dir: Vector3<T>,
    pub point: Vector3<T>,
}

impl<T, R, C, S> Tensor for Matrix<T, R, C, S>
where
    C: Dim,
    R: Dim,
    S: Storage<T, R, C> + Copy,
    T: nalgebra::Field + nalgebra::Scalar + Display + Copy + Pow<u16, Output = T>,
    nalgebra::DefaultAllocator: nalgebra::allocator::Allocator<T, R, C>,
    nalgebra::Matrix<T, R, C, S>: num_traits::Zero,
{
    type Scalar = T;
}

impl<T> SideDir for SuperPoint<T>
where
    T: nalgebra::Field + nalgebra::Scalar + Display + Copy,
{
    fn side_dir(&self) -> Vector3<T> {
        self.side_dir
    }

    type Vector = Vector3<T>;
}

impl<T> Point for SuperPoint<T>
where
    T: nalgebra::Field + nalgebra::Scalar + Display + Copy,
{
    fn point(&self) -> Self::Vector {
        self.point
    }

    type Vector = Vector3<T>;

    fn set_point(&mut self, v: Self::Vector) {
        self.point = v;
    }
}
impl<T> SideDir for HyperPointT<T>
where
    T: nalgebra::Field + nalgebra::Scalar + Display,
{
    fn side_dir(&self) -> Vector3<T> {
        self.dir.cross(&self.normal)
    }

    type Vector = Vector3<T>;
}

impl<T> Point for HyperPointT<T>
where
    T: nalgebra::Field + nalgebra::Scalar + Display,
{
    fn point(&self) -> Self::Vector {
        self.point.clone()
    }

    type Vector = Vector3<T>;

    fn set_point(&mut self, v: Self::Vector) {
        self.point = v;
    }
}

impl<T> fmt::Debug for HyperPointT<T>
where
    T: nalgebra::Field + nalgebra::Scalar + Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            " HP: {} {} {} ⇧ {} {} {} ⇨ {} {} {}",
            self.point.x,
            self.point.y,
            self.point.z,
            self.normal.x,
            self.normal.y,
            self.normal.z,
            self.dir.x,
            self.dir.y,
            self.dir.z,
        )
    }
}

impl<T> Length for SuperPoint<T>
where
    T: nalgebra::Scalar + Display + nalgebra::Field + nalgebra::ComplexField<RealField = T>,
{
    type Scalar = T;

    fn length(&self) -> Self::Scalar {
        self.point.norm()
    }
}

impl<T> Length for HyperPointT<T>
where
    T: nalgebra::Scalar + Display + nalgebra::Field + nalgebra::ComplexField<RealField = T>,
{
    type Scalar = T;

    fn length(&self) -> Self::Scalar {
        self.point.norm()
    }
}

impl<T> Mul<T> for HyperPointT<T>
where
    T: nalgebra::Scalar + nalgebra::Field + Copy,
{
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            normal: self.normal * rhs,
            dir: self.dir * rhs,
            point: self.point * rhs,
        }
    }
}

impl<T> Mul for HyperPointT<T>
where
    T: nalgebra::Scalar + nalgebra::Field + Copy,
{
    type Output = Self;

    fn mul(self, _rhs: Self) -> Self::Output {
        todo!();
    }
}

impl<T> Mul for SuperPoint<T>
where
    T: nalgebra::Scalar + nalgebra::Field + Copy,
{
    type Output = Self;

    fn mul(self, _rhs: Self) -> Self::Output {
        todo!("Mul for super point");
    }
}

impl<T> Mul<T> for SuperPoint<T>
where
    T: nalgebra::Scalar + nalgebra::Field + Copy,
{
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            side_dir: self.side_dir * rhs,
            point: self.point * rhs,
        }
    }
}

impl<T> Sub<Self> for HyperPointT<T>
where
    T: nalgebra::Scalar + nalgebra::Field,
{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            normal: self.normal - rhs.normal,
            dir: self.dir - rhs.dir,
            point: self.point - rhs.point,
        }
    }
}
impl<T> Add<Self> for HyperPointT<T>
where
    T: nalgebra::Scalar + nalgebra::Field,
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            normal: self.normal + rhs.normal,
            dir: self.dir + rhs.dir,
            point: self.point + rhs.point,
        }
    }
}

impl<T> Add<Self> for SuperPoint<T>
where
    T: nalgebra::Scalar + nalgebra::Field,
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            side_dir: self.side_dir + rhs.side_dir,
            point: self.point + rhs.point,
        }
    }
}
impl<T> Sub<Self> for SuperPoint<T>
where
    T: nalgebra::Scalar + nalgebra::Field,
{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            side_dir: self.side_dir - rhs.side_dir,
            point: self.point - rhs.point,
        }
    }
}

impl<T> Zero for HyperPointT<T>
where
    T: nalgebra::Scalar + nalgebra::Field,
{
    fn zero() -> Self {
        Self {
            normal: Vector3::zeros(),
            dir: Vector3::zeros(),
            point: Vector3::zeros(),
        }
    }

    fn is_zero(&self) -> bool {
        self.point.is_zero() && self.dir.is_zero() && self.normal.is_zero()
    }
}

impl<T> Zero for SuperPoint<T>
where
    T: nalgebra::Scalar + nalgebra::Field,
{
    fn zero() -> Self {
        Self {
            side_dir: Vector3::zeros(),
            point: Vector3::zeros(),
        }
    }

    fn is_zero(&self) -> bool {
        self.point.is_zero() && self.side_dir.is_zero()
    }
}

impl<T> iter::Sum for HyperPointT<T>
where
    T: nalgebra::Scalar + nalgebra::Field,
{
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(HyperPointT::zero(), |acc, i| acc + i)
    }
}
