    pub struct HyperSurfaceDifferentSize<A, B>(A, B);
    pub struct HyperSurfaceManyWithOne<A, B>(A, B);
    pub struct HyperSurfaceManyWithMany<A, B>(A, B);
    pub struct SuperSurface<A, B>(A, B);

    impl<A, B, L1, L2, Scalar> Geometry
        for HyperSurfaceDifferentSize<HyperPath<L1, A, Scalar>, HyperPath<L2, B, Scalar>>
    {
        fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
            if self.0.len() == 1 {
                let (h, _) = self.0.head_tail();
                //HyperSurfaceManyWithOne(self.1, h).polygonize(index, complexity)?;
            } else if self.1.len() == 1 {
                let (h, _) = self.1.head_tail();
                //HyperSurfaceManyWithOne(self.0, h).polygonize(index, complexity)?;
            } else {
                HyperSurfaceManyWithMany(self.0, self.1).polygonize(index, complexity)?;
            }
            Ok(())
        }
    }

    impl<A, L1, L2, Scalar, C> Geometry
        for HyperSurfaceManyWithMany<HyperPath<L1, A, Scalar>, HyperPath<L2, C, Scalar>>
    where
        HyperPath<L1, A, Scalar>: Length<Scalar = Scalar>,
        HyperPath<L2, C, Scalar>: Length<Scalar = Scalar>,
        /*
        HyperPath<A, B, Scalar>: Length<Scalar = Scalar>,
        HyperPath<C, D, Scalar>: Length<Scalar = Scalar>,
        HyperSurface<B, D>: Geometry,
        HyperSurface<HyperPath<A, B, Scalar>, HyperPath<L2, HyperPath<C, D, Scalar>, Scalar>>: Geometry,
        HyperSurface<
            HyperPath<A, B, Scalar>,
            HyperPath<L2, HyperPath<L2, HyperPath<C, D, Scalar>, Scalar>, Scalar>,
        >: Geometry,
        L1: Length<Scalar = Scalar>,
        L2: Length<Scalar = Scalar>,
        A: Length<Scalar = Scalar>,
        C: Length<Scalar = Scalar>,
        L2: SplitHyperLine<<L1 as Length>::Scalar>,
        Scalar: Div<Scalar, Output = Scalar> + Add<Scalar, Output = Scalar> + Zero,
        Scalar: PartialOrd<Scalar> + From<u16> + Copy,
        SimpleSurface<L1, L2>: GetLineAt<Scalar = Scalar>,
        SimpleSurface<A, C>: GetLineAt<Scalar = Scalar>,
        SimpleSurface<A, L2>: GetLineAt<Scalar = Scalar>,
        <SimpleSurface<L1, L2> as GetLineAt>::Line: GetT<Scalar = Scalar, Value = Vector3<Scalar>>,
        <SimpleSurface<A, C> as GetLineAt>::Line: GetT<Scalar = Scalar, Value = Vector3<Scalar>>,
        <SimpleSurface<A, L2> as GetLineAt>::Line: GetT<Scalar = Scalar, Value = Vector3<Scalar>>,
        */
    {
        fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
            let len_1 = self.0.length();
            let len_2 = self.1.length();
            let (f, rest_f) = self.0.head_tail();
            let (s, rest_s) = self.1.head_tail();
            let f_l = f.length() / len_1;
            let s_l = s.length() / len_2;
            let rest_f = rest_f.expect("must be some");
            let rest_s = rest_s.expect("must be some");
            if f_l > s_l {
                SimpleSurface(f, s).polygonize(index, complexity)?;
                HyperSurface(rest_f, rest_s).polygonize(index, complexity)?;
            } else {
                let (ss, s) = s.split_hyper_line(f_l);
                SimpleSurface(f, ss).polygonize(index, complexity)?;
                let rest_s = rest_s.push(s);
                HyperSurface(rest_f, rest_s.push(s)).polygonize(index, complexity)?;
            }
            Ok(())
        }
    }

    impl Geometry for HyperSurface<(), ()> {
        fn polygonize(self, _index: &mut GeoIndex, _complexity: usize) -> anyhow::Result<()> {
            Ok(())
        }
    }

    /*
    impl<L1, L2> HyperSurfaceDifferentSize<L1, L2>
    where
        L1: Length,
        <L1 as Length>::Scalar: Div,
        L2: Length<Scalar = <L1 as Length>::Scalar>,
        <L1 as Length>::Scalar: PartialOrd<<L2 as Length>::Scalar>,
        L1: HeadTail,
        L2: HeadTail,
        L1::Head: Length<Scalar = <L1 as Length>::Scalar>,
        <L1::Head as Length>::Scalar: Div<<L1 as Length>::Scalar> + Div,
        L2::Head: SplitHyperLine<<<L1::Head as Length>::Scalar as Div<<L1 as Length>::Scalar>>::Output>,
        SimpleSurface<L1::Head, L2::Head>: Geometry,
    {
        fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
            let total_len_0 = self.0.length();
            let (f, rest_f) = self.0.head_tail();
            let (s, _) = self.1.head_tail();

            let len_p = f.length() / total_len_0;
            let (s, rest_s) = s.split_hyper_line(len_p);
            SimpleSurface(f, s).polygonize(index, complexity)?;
            if let Some(rest_f) = rest_f {
                HyperSurface(rest_f, HyperPath::new(rest_s)).polygonize_different(index, complexity)?;
            }
            Ok(())
        }
    }


    impl<const C1: usize, const C2: usize, Scalar>
        HyperSurface<HyperLine<C1, HyperPointT<Scalar>>, HyperLine<C2, HyperPointT<Scalar>>>
    {
        fn get_line_at(&self, t: Scalar) -> HyperLine<4, Vector3<Scalar>>
        where
            Scalar: One + Zero,
            Scalar: Sub<Scalar, Output = Scalar>,
            Scalar: Pow<u16, Output = Scalar>,
            Scalar: From<u16>,
            Scalar: Copy,
            Scalar: Debug,
            Scalar: nalgebra::Scalar + nalgebra::Field,
            HyperPointT<Scalar>: Add<HyperPointT<Scalar>, Output = HyperPointT<Scalar>>,
            HyperPointT<Scalar>: iter::Sum,
            HyperPointT<Scalar>: Mul<Scalar, Output = HyperPointT<Scalar>>,
        {
            let f = self.0.get_t(t);
            let s = self.1.get_t(t);
            let a = f.point;
            let b = f.side_dir() + f.point;
            let c = s.side_dir() + s.point;
            let d = s.point;
            HyperLine::new_4(a, b, c, d)
        }
    }

    impl<const C1: usize, const C2: usize, Scalar> Geometry
        for HyperSurface<HyperLine<C1, Vector3<Scalar>>, HyperLine<C2, Vector3<Scalar>>>
    where
        Scalar: One + Zero,
        Scalar: Sub<Scalar, Output = Scalar>,
        Scalar: Pow<u16, Output = Scalar>,
        Scalar: From<u16>,
        Scalar: Copy,
        Scalar: Debug,
        Scalar: nalgebra::Scalar + nalgebra::Field,
        HyperPointT<Scalar>: Add<HyperPointT<Scalar>, Output = HyperPointT<Scalar>>,
        HyperPointT<Scalar>: iter::Sum,
        HyperPointT<Scalar>: Mul<Scalar, Output = HyperPointT<Scalar>>,
        Vector3<Dec>: From<Vector3<Scalar>>,
    {
        fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
            if C1 == 2 && C2 == 2 {
                let t = Scalar::zero();
                let tt = Scalar::one();

                let a = self.0.get_t(t);
                let b = self.0.get_t(tt);
                let c = self.1.get_t(tt);
                let d = self.1.get_t(t);
                let p1 = Polygon::new(vec![a.into(), b.into(), c.into()])?;
                let p2 = Polygon::new(vec![a.into(), c.into(), d.into()])?;
                index.save_polygon(&p1);
                index.save_polygon(&p2);
            } else {
                for (t, tt) in ParametricIterator::<Scalar>::new(complexity) {
                    let a = self.0.get_t(t);
                    let b = self.0.get_t(tt);
                    let c = self.1.get_t(tt);
                    let d = self.1.get_t(t);
                    let p1 = Polygon::new(vec![a.into(), b.into(), c.into()])?;
                    let p2 = Polygon::new(vec![a.into(), c.into(), d.into()])?;
                    index.save_polygon(&p1);
                    index.save_polygon(&p2);
                }
            }

            Ok(())
        }
    }

    impl<const C1: usize, const C2: usize, Scalar> Geometry
        for HyperSurface<HyperLine<C1, HyperPointT<Scalar>>, HyperLine<C2, HyperPointT<Scalar>>>
    where
        Scalar: Zero + One,
        Scalar: Pow<u16, Output = Scalar>,
        Scalar: From<u16>,
        Scalar: Copy,
        Scalar: Debug,
        Scalar: nalgebra::Scalar + nalgebra::Field,
        Vector3<Dec>: From<Vector3<Scalar>>,
    {
        fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
            if C1 == 2 && C2 == 2 {
                let l1 = self.get_line_at(Scalar::zero());
                let l2 = self.get_line_at(Scalar::one());
                HyperSurface(l1, l2).polygonize(index, complexity)?
            } else {
                for (t, tt) in ParametricIterator::<Scalar>::new(complexity) {
                    let l1 = self.get_line_at(t);
                    let l2 = self.get_line_at(tt);
                    HyperSurface(l1, l2).polygonize(index, complexity)?
                }
            }
            Ok(())
        }
    }

    impl<A, B, const C1: usize, const C2: usize, Scalar> Geometry
        for HyperSurface<
            HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>,
            HyperPath<HyperLine<C2, HyperPointT<Scalar>>, B, Scalar>,
        >
    where
        Scalar: nalgebra::Scalar + nalgebra::Field,
        HyperSurface<A, B>: Geometry,
        Vector3<Dec>: From<Vector3<Scalar>>,
        A: Length<Scalar = Scalar>,
        B: Length<Scalar = A::Scalar>,
        HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>: Length<Scalar = A::Scalar>,
        HyperPath<HyperLine<C2, HyperPointT<Scalar>>, B, Scalar>: Length<Scalar = A::Scalar>,
        A::Scalar: Add<Output = A::Scalar>,
        A::Scalar: Zero,
        Scalar: Div<Scalar, Output = Scalar> + Add<Scalar, Output = Scalar> + Zero + Copy + Debug,
        A::Scalar: Div<Scalar, Output = Scalar>,
        HyperPointT<Scalar>: Sub<HyperPointT<Scalar>, Output = HyperPointT<Scalar>>,
        HyperPointT<Scalar>: Mul<<HyperPointT<Scalar> as Length>::Scalar, Output = HyperPointT<Scalar>>,
        HyperPointT<Scalar>: Length<Scalar = Scalar>,
        HyperPointT<Scalar>: iter::Sum,
        HyperPointT<Scalar>: Zero,
        <HyperPointT<Scalar> as Length>::Scalar: Div<
                <HyperPointT<Scalar> as Length>::Scalar,
                Output = <HyperPointT<Scalar> as Length>::Scalar,
            > + Sub<
                <HyperPointT<Scalar> as Length>::Scalar,
                Output = <HyperPointT<Scalar> as Length>::Scalar,
            > + Zero
            + One
            + Copy
            + Debug
            + Display
            + From<u16>
            + Pow<u16, Output = <HyperPointT<Scalar> as Length>::Scalar>
            + AddAssign<<HyperPointT<Scalar> as Length>::Scalar>, //Dec: Div<Scalar, Output = Scalar>,
    {
        fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
            if self.0.len() == self.1.len() {
                self.polygonize_same(index, complexity)
            } else {
                todo!();
                // self.polygonize_different(index, complexity)
            }
        }
    }

    impl<A, const C1: usize, const C2: usize, Scalar> Geometry
        for HyperSurface<
            HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>,
            HyperPath<HyperLine<C2, HyperPointT<Scalar>>, (), Scalar>,
        >
    where
        A: Length,
        A::Scalar: Add<Output = A::Scalar>,
        A::Scalar: Zero,
        A: Length<Scalar = Scalar>,
        HyperLine<C1, HyperPointT<Scalar>>: Length<Scalar = Scalar>,
        HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>: Length<Scalar = A::Scalar>,
        HyperLine<C2, HyperPointT<Scalar>>: SplitHyperLine<Scalar>,

        HyperSurface<A, HyperPath<HyperLine<C2, HyperPointT<Scalar>>, (), Scalar>>: Geometry,
        Scalar: Div<Scalar, Output = Scalar>
            + Pow<u16, Output = Scalar>
            + nalgebra::Scalar
            + nalgebra::Field
            + Add<Scalar, Output = Scalar>
            + Zero
            + One
            + Copy
            + Debug
            + From<u16>
            + nalgebra::ComplexField<RealField = Scalar>,
        Vector3<Dec>: From<Vector3<Scalar>>,
    {
        fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
            let total_len = self.0.length();
            let (f, rest_f) = self.0.head_tail();
            let (s, _) = self.1.head_tail();

            let len_p = f.length() / total_len;
            let (s, rest_s) = s.split_hyper_line(len_p);
            PrimitiveSurface(f, s).polygonize(index, complexity)?;
            if let Some(rest_f) = rest_f {
                HyperSurface(rest_f, HyperPath::new(rest_s)).polygonize(index, complexity)?;
            }
            Ok(())
        }
    }

    impl<A, const C1: usize, const C2: usize, Scalar> Geometry
        for HyperSurface<
            HyperPath<HyperLine<C2, HyperPointT<Scalar>>, (), Scalar>,
            HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>,
        >
    where
        A: Length,
        A::Scalar: Add<Output = A::Scalar>,
        A::Scalar: Zero,
        A: Length<Scalar = Scalar>,
        HyperLine<C1, HyperPointT<Scalar>>: Length<Scalar = Scalar>,
        HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>: Length<Scalar = A::Scalar>,
        HyperLine<C2, HyperPointT<Scalar>>: SplitHyperLine<Scalar>,

        HyperSurface<A, HyperPath<HyperLine<C2, HyperPointT<Scalar>>, (), Scalar>>: Geometry,
        Scalar: Div<Scalar, Output = Scalar>
            + Pow<u16, Output = Scalar>
            + nalgebra::Scalar
            + nalgebra::Field
            + Add<Scalar, Output = Scalar>
            + Zero
            + One
            + Copy
            + Debug
            + From<u16>
            + nalgebra::ComplexField<RealField = Scalar>,
        Vector3<Dec>: From<Vector3<Scalar>>,
    {
        fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
            HyperSurface(self.1, self.0).polygonize(index, complexity)
        }
    }

    impl<const C1: usize, const C2: usize, Scalar> Geometry
        for HyperSurface<
            HyperPath<HyperLine<C2, HyperPointT<Scalar>>, (), Scalar>,
            HyperPath<HyperLine<C1, HyperPointT<Scalar>>, (), Scalar>,
        >
    where
        Scalar: Div<Scalar, Output = Scalar>
            + Pow<u16, Output = Scalar>
            + nalgebra::Scalar
            + nalgebra::Field
            + Add<Scalar, Output = Scalar>
            + Zero
            + One
            + Copy
            + Debug
            + Display
            + From<u16>,
        Vector3<Dec>: From<Vector3<Scalar>>,
    {
        fn polygonize(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()> {
            let (f, _) = self.0.head_tail();
            let (s, _) = self.1.head_tail();

            PrimitiveSurface(f, s).polygonize(index, complexity)?;

            Ok(())
        }
    }

    impl<A, const C1: usize, Scalar> Geometry
        for HyperSurface<(), HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>>
    {
        fn polygonize(self, _index: &mut GeoIndex, _complexity: usize) -> anyhow::Result<()> {
            unreachable!("Cannot get to this condition");
        }
    }

    impl<A, const C1: usize, Scalar> Geometry
        for HyperSurface<HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>, ()>
    {
        fn polygonize(self, _index: &mut GeoIndex, _complexity: usize) -> anyhow::Result<()> {
            unreachable!("Cannot get to this condition");
        }
    }


    impl<A, B, const C1: usize, const C2: usize, Scalar>
        HyperSurface<
            HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>,
            HyperPath<HyperLine<C2, HyperPointT<Scalar>>, B, Scalar>,
        >
    where
        HyperSurface<A, B>: Geometry,
        A: Length<Scalar = Scalar>,
        B: Length<Scalar = A::Scalar>,
        HyperPath<HyperLine<C1, HyperPointT<Scalar>>, A, Scalar>: Length<Scalar = A::Scalar>,
        HyperPath<HyperLine<C2, HyperPointT<Scalar>>, B, Scalar>: Length<Scalar = A::Scalar>,
        A::Scalar: Add<Output = A::Scalar>,
        A::Scalar: Zero,
        Scalar: Div<Scalar, Output = Scalar> + Add<Scalar, Output = Scalar> + Zero + Copy + Debug,
        A::Scalar: Div<Scalar, Output = Scalar>,
        HyperPointT<Scalar>: Sub<HyperPointT<Scalar>, Output = HyperPointT<Scalar>>,
        HyperPointT<Scalar>: Mul<<HyperPointT<Scalar> as Length>::Scalar, Output = HyperPointT<Scalar>>,
        HyperPointT<Scalar>: Length<Scalar = Scalar>,
        HyperPointT<Scalar>: iter::Sum,
        HyperPointT<Scalar>: Zero,
        <HyperPointT<Scalar> as Length>::Scalar: Div<
                <HyperPointT<Scalar> as Length>::Scalar,
                Output = <HyperPointT<Scalar> as Length>::Scalar,
            > + Sub<
                <HyperPointT<Scalar> as Length>::Scalar,
                Output = <HyperPointT<Scalar> as Length>::Scalar,
            > + Zero
            + One
            + Copy
            + Debug
            + From<u16>
            + Pow<u16, Output = <HyperPointT<Scalar> as Length>::Scalar>
            + AddAssign<<HyperPointT<Scalar> as Length>::Scalar>, //Dec: Div<Scalar, Output = Scalar>,
    {
        fn polygonize_different(self, index: &mut GeoIndex, complexity: usize) -> anyhow::Result<()>
    where {
            let total_f_length = self.0.length();
            let total_s_length = self.1.length();
            let (f, rest_f) = self.0.head_tail();
            let (s, rest_s) = self.1.head_tail();

            //dbg!(total_f_length);
            //dbg!(f.length());
            //dbg!(total_s_length);
            //dbg!(s.length());
            //dbg!(f.length() / total_f_length);

            Ok(())
        }
    }
    impl Geometry for HyperSurface<(), ()> {
        fn polygonize(self, _index: &mut GeoIndex, _complexity: usize) -> anyhow::Result<()> {
            Ok(())
        }
    }
    */
}
#[cfg(test)]
mod tests {
    use nalgebra::{Vector3, Vector4};
    use num_traits::Zero;

    use crate::{
        decimal::Dec,
        geometry::Geometry,
        hyper_path::{hyper_line::HyperLine, hyper_path::HyperPath, hyper_point::HyperPointT},
        indexes::geo_index::index::GeoIndex,
    };

    use super::HyperSurface;

    #[test]
    fn join_1_1() {
        let l1 = HyperLine::new_2(
            HyperPointT {
                normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                point: Vector3::zeros(),
            },
            HyperPointT {
                normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                point: Vector3::x(),
            },
        );

        let hp = HyperPath::new().add(l1).build();
        let l2 = HyperLine::new_2(
            HyperPointT {
                normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                point: Vector3::z(),
            },
            HyperPointT {
                normal: Vector3::new(Dec::zero(), Dec::zero(), Dec::from(1)),
                dir: Vector3::new(Dec::from(1), Dec::zero(), Dec::zero()),
                point: Vector3::x() + Vector3::z(),
            },
        );
        let hp2 = HyperPath::new().add(l2).build();
        let hs = HyperSurface(hp, hp2);
        let mut ix = GeoIndex::default();
        hs.polygonize(&mut ix, 10).unwrap();
    }
    /*

    #[test]
    fn join_1_2() {
        let l1 = HyperLine::new_2(
            HyperPointT {
                normal: Vector3::new(f64::zero(), f64::zero(), f64::from(1)),
                dir: Vector3::new(f64::from(1), f64::zero(), f64::zero()),
                point: Vector3::zeros(),
            },
            HyperPointT {
                normal: Vector3::new(f64::zero(), f64::zero(), f64::from(1)),
                dir: Vector3::new(f64::from(1), f64::zero(), f64::zero()),
                point: Vector3::x(),
            },
        );

        let hp = HyperPath::new(l1);
        let l2 = HyperLine::new_2(
            HyperPointT {
                normal: Vector3::new(f64::zero(), f64::zero(), f64::from(1)),
                dir: Vector3::new(f64::from(1), f64::zero(), f64::zero()),
                point: Vector3::z(),
            },
            HyperPointT {
                normal: Vector3::new(f64::zero(), f64::zero(), f64::from(1)),
                dir: Vector3::new(f64::from(1), f64::zero(), f64::zero()),
                point: Vector3::x() + Vector3::z(),
            },
        );
        let hp2 = HyperPath::new(l2);

        let l3 = HyperLine::new_2(
            HyperPointT {
                normal: Vector3::new(f64::zero(), f64::zero(), f64::from(1)),
                dir: Vector3::new(f64::from(1), f64::zero(), f64::zero()),
                point: Vector3::x() + Vector3::z(),
            },
            HyperPointT {
                normal: Vector3::new(f64::zero(), f64::zero(), f64::from(1)),
                dir: Vector3::new(f64::from(1), f64::zero(), f64::zero()),
                point: Vector3::x() * f64::from(2) + Vector3::z(),
            },
        );
        let hp2 = hp2.push(l3);
        let hs = HyperSurface(hp, hp2);
        let mut ix = GeoIndex::default();
        hs.polygonize(&mut ix, 10).unwrap();
        assert!(false);
    }
    */

