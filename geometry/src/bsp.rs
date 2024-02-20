use std::fmt;

use itertools::Itertools;
use nalgebra::Vector3;

use crate::primitives::cutter::{SplitResult, Splitter};

pub trait Reversable {
    fn flip(self) -> Self;
}

#[derive(Debug, Clone, PartialEq)]
pub struct Bsp<Cutter, Item> {
    cutter: Cutter,
    pub front: Option<Box<Self>>,
    pub back: Option<Box<Self>>,
    pub coplanar_front: Vec<Item>,
    pub coplanar_back: Vec<Item>,
}

impl<Cutter, Item> Bsp<Cutter, Item> {
    fn items_amount(&self) -> usize {
        let mut amount = self.coplanar_front.len() + self.coplanar_back.len();
        if let Some(f) = self.front.as_ref().map(|f| f.items_amount()) {
            amount += f;
        }
        if let Some(f) = self.back.as_ref().map(|f| f.items_amount()) {
            amount += f;
        }
        amount
    }
}

impl<Cutter, Item> Bsp<Cutter, Item>
where
    Cutter: Splitter<Item>,
    Cutter: Clone + fmt::Debug,
    Cutter: Reversable,
    Item: Clone + fmt::Debug,
    Item: Reversable,
    Item: PartialEq,
    Item: 'static,
{
    fn new(cutter: Cutter) -> Self {
        Self {
            cutter,
            front: None,
            back: None,
            coplanar_front: Vec::new(),
            coplanar_back: Vec::new(),
        }
    }

    pub fn merge(&mut self, other: impl IntoIterator<Item = Item>) {
        let (front, back, mut coplanar_front, mut coplanar_back) =
            other.into_iter().map(|f| self.cutter.split(f)).fold(
                (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
                |(mut front, mut back, mut coplanar_front, mut coplanar_back), mut split| {
                    front.append(&mut split.front);
                    back.append(&mut split.back);
                    coplanar_front.append(&mut split.coplanar_front);
                    coplanar_back.append(&mut split.coplanar_back);

                    (front, back, coplanar_front, coplanar_back)
                },
            );
        if !front.is_empty() {
            if let Some(tree) = self.front.as_mut() {
                tree.merge(front);
            } else {
                self.front = Self::build(front).map(Box::new);
            }
        }
        if !back.is_empty() {
            if let Some(tree) = self.back.as_mut() {
                tree.merge(back);
            } else {
                self.back = Self::build(back).map(Box::new);
            }
        }
        self.coplanar_back.append(&mut coplanar_back);
        self.coplanar_front.append(&mut coplanar_front);
    }

    pub fn build(polygon: impl IntoIterator<Item = Item>) -> Option<Self> {
        let mut iter = polygon.into_iter();
        let segment = iter.next();
        segment.and_then(|segment| {
            let cutter = Cutter::from_item(&segment);
            let (front, back, coplanar_front, coplanar_back) = iter.map(|f| cutter.split(f)).fold(
                (Vec::new(), Vec::new(), vec![segment], Vec::new()),
                |(mut front, mut back, mut coplanar_front, mut coplanar_back), mut split| {
                    front.append(&mut split.front);
                    back.append(&mut split.back);
                    coplanar_front.append(&mut split.coplanar_front);
                    coplanar_back.append(&mut split.coplanar_back);

                    (front, back, coplanar_front, coplanar_back)
                },
            );
            let front = if front.is_empty() {
                None
            } else {
                Some(Box::new(Self::build(front)?))
            };
            let back = if back.is_empty() {
                None
            } else {
                Some(Box::new(Self::build(back)?))
            };

            Some(Self {
                cutter,
                front,
                back,
                coplanar_front,
                coplanar_back,
            })
        })
    }

    pub(crate) fn invert(mut self) -> Self {
        let coplanar_back = self.coplanar_front.into_iter().map(|f| f.flip()).collect();
        let coplanar_front = self.coplanar_back.into_iter().map(|f| f.flip()).collect();
        self.coplanar_back = coplanar_back;
        self.coplanar_front = coplanar_front;
        self.cutter = self.cutter.flip();
        let back = self.front.take().map(|tree| Box::new(tree.invert()));
        let front = self.back.take().map(|tree| Box::new(tree.invert()));

        self.front = front;
        self.back = back;

        self
    }

    pub fn sort_front_back(&self, items: Vec<Item>) -> (Vec<Item>, Vec<Item>) {
        let (front, back) = items
            .into_iter()
            .map(|segment| {
                //dbg!(&segment);
                self.cutter.split(segment)
            })
            .fold(
                (Vec::new(), Vec::new()),
                |(mut front, mut back), mut split| {
                    front.append(&mut split.front);
                    back.append(&mut split.back);
                    front.extend(split.coplanar_front);
                    back.extend(split.coplanar_back);
                    (front, back)
                },
            );

        let mut split_result = SplitResult::default();

        if let Some(tree) = self.front.as_ref() {
            let (f, b) = tree.sort_front_back(front);
            split_result = split_result.fronts(f).backs(b)
        } else {
            split_result = split_result.fronts(front)
        }
        if let Some(tree) = self.back.as_ref() {
            let (f, b) = tree.sort_front_back(back);
            split_result = split_result.fronts(f).backs(b)
        } else {
            split_result = split_result.backs(back)
        }
        (split_result.front, split_result.back)
    }

    pub fn clip(&self, items: Vec<Item>) -> Vec<Item> {
        let mut items = items
            .into_iter()
            .map(|segment| {
                //dbg!(&segment);
                self.cutter.split(segment)
            })
            .fold(Vec::new(), |(mut clipped), mut split| {
                clipped.append(&mut split.front);
                clipped.append(&mut split.back);
                clipped.extend(split.coplanar_front);
                clipped.extend(split.coplanar_back);
                clipped
            });

        if let Some(tree) = self.front.as_ref() {
            items = tree.clip(items);
            //println!("no_front_tree");
        }

        if let Some(tree) = self.back.as_ref() {
            items = tree.clip(items);
        } else {
            //println!("no_back_tree");
        }
        items
    }

    /*
    fn clip_segments_by_front_back(&self, lines: Vec<Item>) -> (Vec<Item>, Vec<Item>) {
        let (front, back) = lines
            .into_iter()
            .map(|segment| self.cutter.split(segment))
            .fold(
                (Vec::new(), Vec::new()),
                |(mut front, mut back), mut split| {
                    front.append(&mut split.front);
                    back.append(&mut split.back);
                    front.extend(split.coplanar_front);
                    back.extend(split.coplanar_back);
                    (front, back)
                },
            );

        let mut split_result = SplitResult::default();

        if let Some(tree) = self.front.as_ref() {
            let (f, b) = tree.clip_segments_by_front_back(front);
            split_result = split_result.fronts(f).backs(b)
        } else {
            println!("no_front_tree");
            split_result = split_result.fronts(front)
        }
        if let Some(tree) = self.back.as_ref() {
            let (f, b) = tree.clip_segments_by_front_back(back);
            split_result = split_result.fronts(f).backs(b)
        } else {
            println!("no_back_tree");
            split_result = split_result.backs(back)
        }
        (split_result.front, split_result.back)
    }
    */
}

pub struct ItemsBspIterator<I, Item>
where
    I: Iterator<Item = Item>,
{
    len: usize,
    inner: I,
}

impl<I, Item> Iterator for ItemsBspIterator<I, Item>
where
    I: Iterator<Item = Item>,
{
    type Item = Item;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next()
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.len, Some(self.len))
    }
}

impl<Cutter, Item> IntoIterator for Bsp<Cutter, Item>
where
    Item: 'static,
{
    type Item = Item;

    type IntoIter = ItemsBspIterator<Box<dyn Iterator<Item = Item>>, Item>;

    fn into_iter(self) -> Self::IntoIter {
        let len = self.items_amount();
        let mut my: Box<dyn Iterator<Item = Item>> =
            Box::new(self.coplanar_front.into_iter().chain(self.coplanar_back));
        if let Some(fronts) = self.front {
            my = Box::new(my.chain(fronts.into_iter()));
        }
        if let Some(backs) = self.back {
            my = Box::new(my.chain(backs.into_iter()));
        }

        ItemsBspIterator {
            inner: Box::new(my),
            len,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::fmt::Debug;

    use assert_matches::assert_matches;
    use itertools::{Either, Itertools};
    use nalgebra::{Vector2, Vector3};
    use num_traits::{One, Zero};
    use rust_decimal_macros::dec;

    use crate::{
        //bsp::bsp2::{self, Bsp},
        bsp::Bsp,
        mesh::Mesh,
        primitives::{
            basis::Basis,
            cutter::{SplitResult, Splitter},
            decimal::Dec,
            line2d::Line2D,
            plane::Plane,
            polygon::{self, Polygon},
            polygon_basis::PolygonBasis,
            segment2d::Segment2D,
        },
        shapes::rect,
    };

    use super::Reversable;

    /*
    #[test]
    fn create_convex() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly = Polygon::new(points.to_vec()).unwrap();
        let basis = poly.get_basis_2d();
        let bsp_2d = Bsp::<Line2D, Segment2D>::build(poly.clone()).unwrap();
        assert!(bsp_2d.back.is_none());
        let lines = bsp_2d.into_iter().collect_vec();
        let polygons = Polygon::from_segments(lines, basis).unwrap();

        assert_eq!(polygons.len(), 1);
        assert_eq!(polygons[0], poly);
    }

    #[test]
    fn create_nonconvex() {
        let points = [
            Vector3::new(dec!(-1).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(0).into(), dec!(0.5).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(0).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly = Polygon::new(points.to_vec()).unwrap();
        let basis = poly.get_basis_2d();
        let bsp_2d = Bsp::<Line2D, Segment2D>::build(poly.clone()).unwrap();
        assert!(bsp_2d.front.is_some());
        assert!(bsp_2d.back.is_some());
        let lines = bsp_2d.into_iter().collect_vec();
        let mut polygons = Polygon::from_segments(lines, basis).unwrap();

        assert_eq!(polygons.len(), 1);

        assert_eq!(polygons[0].vertices.len(), poly.vertices.len());
        let sum: Dec = dbg!(polygons
            .pop()
            .unwrap()
            .vertices
            .into_iter()
            .zip(poly.vertices)
            .map(|(p, q)| p - q)
            .map(|p| p.magnitude())
            .sum());
        assert_eq!(sum, dec!(0).into());
    }
    */

    #[test]
    fn boolean_diff() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let basis = PolygonBasis {
            center: Vector3::zeros(),
            x: Vector3::x(),
            y: Vector3::y(),
        };
        let poly1 = Polygon::new(points.to_vec()).unwrap();

        let points = [
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(0).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(0).into(), dec!(2).into(), dec!(0).into()),
        ];
        let poly2 = Polygon::new(points.to_vec()).unwrap();

        let polygons = poly1.boolean_union(poly2);

        let polygons = assert_matches!(polygons, Either::Left(v)=>v);
        let vv: Vec<Vector3<Dec>> = vec![
            Vector3::new(dec!(0).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(0).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        dbg!(polygons[0].vertices.len());
        assert_eq!(polygons[0].vertices, vv);
    }

    #[test]
    fn boolean_diff_2() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let basis = PolygonBasis {
            center: Vector3::zeros(),
            x: Vector3::x(),
            y: Vector3::y(),
        };
        let poly = Polygon::new(points.to_vec()).unwrap();
        let bsp_2d_one = Bsp::<Line2D, Segment2D>::build(poly.get_segments(&basis)).unwrap();

        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
        ];
        let poly = Polygon::new(points.to_vec()).unwrap();
        let bsp_2d_two = Bsp::<Line2D, Segment2D>::build(poly.get_segments(&basis)).unwrap();

        let segments = [Segment2D {
            from: Vector2::new(Dec::from(dec!(-0.7)), Dec::from(dec!(-0.7))),
            to: Vector2::new(Dec::from(dec!(-0.2)), Dec::from(dec!(0.6))),
        }];

        let result = diff(bsp_2d_one, bsp_2d_two);
        let lines = dbg!(result.into_iter().collect_vec());
        dbg!(lines.len());
        let polygons = Polygon::from_segments(lines, &basis).unwrap();
        assert_eq!(polygons.len(), 1);
        let vv: Vec<Vector3<Dec>> = vec![
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        assert_eq!(polygons[0].vertices, vv);
    }

    #[test]
    fn boolean_union_2() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let basis = PolygonBasis {
            center: Vector3::zeros(),
            x: Vector3::x(),
            y: Vector3::y(),
        };
        let poly = Polygon::new(points.to_vec()).unwrap();
        let bsp_2d_one = Bsp::<Line2D, Segment2D>::build(poly.get_segments(&basis)).unwrap();

        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
        ];
        let poly = Polygon::new(points.to_vec()).unwrap();
        let bsp_2d_two = Bsp::<Line2D, Segment2D>::build(poly.get_segments(&basis)).unwrap();

        let segments = [Segment2D {
            from: Vector2::new(Dec::from(dec!(-0.7)), Dec::from(dec!(-0.7))),
            to: Vector2::new(Dec::from(dec!(-0.2)), Dec::from(dec!(0.6))),
        }];

        let lines = union(bsp_2d_one, bsp_2d_two).into_iter().collect_vec();
        dbg!(lines.len());
        let polygons = Polygon::from_segments(lines, &basis).unwrap();
        assert_eq!(polygons.len(), 1);
        dbg!(polygons[0].vertices.len());
        let vv: Vec<Vector3<Dec>> = vec![
            Vector3::new(dec!(1.1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(1.1).into(), dec!(-1).into(), dec!(0).into()),
        ];
        assert_eq!(polygons[0].vertices, vv);

        assert_eq!(polygons.len(), 1);
        let vv: Vec<Vector3<Dec>> = vec![
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        dbg!(polygons[0].vertices.len());
        assert_eq!(polygons[0].vertices, vv);
    }
    #[test]
    fn boolean_union_4() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(4).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(4).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let basis = PolygonBasis {
            center: Vector3::zeros(),
            x: Vector3::x(),
            y: Vector3::y(),
        };
        let one = Polygon::new(points.to_vec()).unwrap();

        let points = [
            Vector3::new(dec!(-4).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-4).into(), dec!(-4).into(), dec!(0).into()),
            Vector3::new(dec!(4).into(), dec!(-4).into(), dec!(0).into()),
            Vector3::new(dec!(4).into(), dec!(-1).into(), dec!(0).into()),
        ];
        let two = Polygon::new(points.to_vec()).unwrap();
        let three = one.boolean_union(two);

        let left = assert_matches!(three, Either::Left(l)=>l);
        assert_eq!(left.len(), 1);
        assert_eq!(
            left[0].vertices,
            vec![
                Vector3::new(dec!(4).into(), dec!(-4).into(), dec!(0).into()),
                Vector3::new(dec!(4).into(), dec!(1).into(), dec!(0).into()),
                Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
                Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
                Vector3::new(dec!(-4).into(), dec!(-1).into(), dec!(0).into()),
                Vector3::new(dec!(-4).into(), dec!(-4).into(), dec!(0).into()),
            ]
        );
    }

    #[test]
    fn boolean_union_5() {
        let points = [
            Vector3::new(dec!(4).into(), dec!(-4).into(), dec!(0).into()),
            Vector3::new(dec!(4).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-4).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-4).into(), dec!(-4).into(), dec!(0).into()),
        ];
        let basis = PolygonBasis {
            center: Vector3::zeros(),
            x: Vector3::x(),
            y: Vector3::y(),
        };
        let one = Polygon::new(points.to_vec()).unwrap();

        let points = [
            Vector3::new(dec!(-4).into(), dec!(4).into(), dec!(0).into()),
            Vector3::new(dec!(-4).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(4).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(4).into(), dec!(4).into(), dec!(0).into()),
        ];
        let two = Polygon::new(points.to_vec()).unwrap();

        let three = one.boolean_union(two);

        let left = assert_matches!(three, Either::Left(l)=>l);
        assert_eq!(left.len(), 1);
        assert_eq!(
            left[0].vertices,
            vec![
                Vector3::new(dec!(4).into(), dec!(-4).into(), dec!(0).into()),
                Vector3::new(dec!(4).into(), dec!(1).into(), dec!(0).into()),
                Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
                Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
                Vector3::new(dec!(-4).into(), dec!(-1).into(), dec!(0).into()),
                Vector3::new(dec!(-4).into(), dec!(-4).into(), dec!(0).into()),
            ]
        );
    }

    #[test]
    fn merge_cubes() {
        let z = Basis::new(Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros()).unwrap();
        let cube_smaller = rect(z.clone(), Dec::one(), Dec::one(), Dec::one());

        let cube_bigger = rect(z, Dec::one() * 2, Dec::one() * 2, Dec::one() * 2);

        let result = cube_bigger.boolean_union(cube_smaller);
        let mesh = assert_matches!(result, Either::Left(p) => p);

        assert_eq!(mesh.sides.len(), 6);
    }

    pub fn union<S, I>(right: Bsp<S, I>, other: Bsp<S, I>) -> Vec<I>
    where
        S: Splitter<I> + Clone + Reversable + Debug,
        I: Clone + Debug + Reversable + PartialEq + 'static,
    {
        let total_segments = right.clone().into_iter().chain(other.clone()).collect_vec();
        let total_segments = right.clip(total_segments);
        let total_segments = other.clip(total_segments);

        let (_front1, back1) = other.sort_front_back(total_segments.clone());

        let (_front2, mut back2) = right.sort_front_back(total_segments);

        dbg!(&_front1);
        dbg!(&back1);

        dbg!(&_front2);
        dbg!(&back2);

        let mut joined = Vec::new();
        for s in back1.into_iter() {
            let inv = s.clone().flip();
            if let Some(ix) = back2.iter().position(|item| *item == inv) {
                back2.swap_remove(ix);
            } else {
                joined.push(s);
            }
        }
        joined.extend(back2);

        joined
    }

    pub fn diff<S: Splitter<I> + Clone, I: Clone>(right: Bsp<S, I>, other: Bsp<S, I>) -> Vec<I> {
        todo!()
    }
    /*
        #[test]
        fn cut_polygon_with_cube() {
            let z = PolygonBasis {
                x: Vector3::x(),
                y: Vector3::y(),
                z: Vector3::z(),
                center: Vector3::zeros(),
            };

            let cube_bigger = rect(z.clone(), Dec::one() * 2, Dec::one() * 2, Dec::one() * 2);

            let polygon = polygon::Polygon::new(
                vec![
                    Vector3::new(-Dec::one() * 4, Dec::one() * 4, Dec::zero()),
                    Vector3::new(-Dec::one() * 4, -Dec::one() * 4, Dec::zero()),
                    Vector3::new(Dec::one() * 4, -Dec::one() * 4, Dec::zero()),
                    Vector3::new(Dec::one() * 4, Dec::one() * 4, Dec::zero()),
                ],
                z,
            );

            let bsp = Bsp::<Plane, Polygon>::build(cube_bigger.polygons()).unwrap();
            let (inside, outside) = bsp.clip_segments_by_front_back(vec![polygon]);
            assert_eq!(inside.len(), 1);
            assert_eq!(
                inside[0].vertices,
                vec![
                    Vector3::new(-Dec::one(), -Dec::one(), Dec::zero()),
                    Vector3::new(Dec::one(), -Dec::one(), Dec::zero()),
                    Vector3::new(Dec::one(), Dec::one(), Dec::zero()),
                    Vector3::new(-Dec::one(), Dec::one(), Dec::zero()),
                ]
            );
            assert_eq!(outside.len(), 4);
            assert_eq!(
                outside[0].vertices,
                vec![
                    Vector3::new(Dec::one(), -Dec::one(), Dec::zero()),
                    Vector3::new(Dec::one() * 4, -Dec::one(), Dec::zero()),
                    Vector3::new(Dec::one() * 4, Dec::one(), Dec::zero()),
                    Vector3::new(Dec::one(), Dec::one(), Dec::zero()),
                ]
            );
            assert_eq!(
                outside[1].vertices,
                vec![
                    Vector3::new(-Dec::one() * 4, Dec::one(), Dec::zero()),
                    Vector3::new(-Dec::one() * 4, -Dec::one(), Dec::zero()),
                    Vector3::new(-Dec::one(), -Dec::one(), Dec::zero()),
                    Vector3::new(-Dec::one(), Dec::one(), Dec::zero()),
                ]
            );
            assert_eq!(
                outside[2].vertices,
                vec![
                    Vector3::new(-Dec::one() * 4, -Dec::one(), Dec::zero()),
                    Vector3::new(-Dec::one() * 4, -Dec::one() * 4, Dec::zero()),
                    Vector3::new(Dec::one() * 4, -Dec::one() * 4, Dec::zero()),
                    Vector3::new(Dec::one() * 4, -Dec::one(), Dec::zero()),
                ]
            );
            assert_eq!(
                outside[3].vertices,
                vec![
                    Vector3::new(-Dec::one() * 4, Dec::one() * 4, Dec::zero()),
                    Vector3::new(-Dec::one() * 4, Dec::one(), Dec::zero()),
                    Vector3::new(Dec::one() * 4, Dec::one(), Dec::zero()),
                    Vector3::new(Dec::one() * 4, Dec::one() * 4, Dec::zero()),
                ]
            );

            let ps = Mesh::join_polygons_on_side(outside);
            assert_eq!(ps.len(), 2);
        }
    */
}
