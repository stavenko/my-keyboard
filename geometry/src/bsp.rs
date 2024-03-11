use rayon::prelude::*;
use std::{collections::HashMap, fmt};

use crate::{
    cutter::{ItemLocation, SplitResult, Splitter, VertexLocation},
    intersects::Intersects,
    plane::Plane,
    polygon::Polygon,
};

pub trait Reversable {
    fn flip(self) -> Self;
}

pub trait Vertices {
    type Vertex;
    fn get_vertices(&self) -> Vec<Self::Vertex>;
}

pub type SortResult<Item> = HashMap<ItemLocation, Vec<Item>>;

#[derive(Debug, Clone, PartialEq)]
pub struct Bsp<Cutter, Item> {
    cutter: Cutter,
    pub front: Option<Box<Self>>,
    pub back: Option<Box<Self>>,
    pub coplanar_front: Vec<Item>,
    pub coplanar_back: Vec<Item>,
}

impl<Cutter, Item, Vertex> Bsp<Cutter, Item>
where
    Cutter: Splitter<Item, Vertex>,
    Cutter: Send + Sync + Clone + fmt::Debug,
    Cutter: Reversable,
    Vertex: fmt::Debug,
    Item: Send + Sync + Sized + Clone + fmt::Debug,
    Item: Reversable,
    Item: Vertices<Vertex = Vertex>,
    Item: Intersects<Cutter>,
    <Item as Intersects<Cutter>>::Out: Intersects<Item> + fmt::Debug,
    <<Item as Intersects<Cutter>>::Out as Intersects<Item>>::Out: fmt::Debug,
    Item: PartialEq,
    Item: 'static,
{
    /*
    fn new(cutter: Cutter) -> Self {
        Self {
            cutter,
            front: None,
            back: None,
            coplanar_front: Vec::new(),
            coplanar_back: Vec::new(),
        }
    }

    fn merge(&mut self, other: impl IntoIterator<Item = Item>) {
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
    */

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

    pub fn invert(mut self) -> Self {
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
            .into_par_iter()
            .map(|segment| self.cutter.split(segment))
            .fold(
                || Vec::new(),
                |mut clipped, mut split| {
                    clipped.append(&mut split.front);
                    clipped.append(&mut split.back);
                    clipped.extend(split.coplanar_front);
                    clipped.extend(split.coplanar_back);
                    clipped
                },
            )
            .flatten()
            .collect();

        if let Some(tree) = self.front.as_ref() {
            items = tree.clip(items);
        }

        if let Some(tree) = self.back.as_ref() {
            items = tree.clip(items);
        }
        items
    }
    pub fn locate_vertex(&self, vertex: &Vertex) -> VertexLocation {
        match self.cutter.locate_vertex(vertex) {
            VertexLocation::Front => {
                if let Some(tree) = self.front.as_ref() {
                    tree.locate_vertex(vertex)
                } else {
                    VertexLocation::Front
                }
            }
            VertexLocation::Back => {
                if let Some(tree) = self.back.as_ref() {
                    tree.locate_vertex(vertex)
                } else {
                    VertexLocation::Back
                }
            }
            loc => loc,
        }
    }

    pub fn item_edge_location(&self, item: &Item) -> ItemLocation {
        match self.cutter.locate(item) {
            ItemLocation::Front => {
                if let Some(tree) = self.front.as_ref() {
                    tree.item_edge_location(item)
                } else {
                    ItemLocation::Front
                }
            }

            ItemLocation::Back => {
                if let Some(tree) = self.back.as_ref() {
                    tree.item_edge_location(item)
                } else {
                    ItemLocation::Back
                }
            }

            loc => loc,
        }
    }
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

    #[allow(clippy::useless_conversion)]
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

#[cfg(test)]
mod tests {
    use std::fmt::Debug;

    use assert_matches::assert_matches;
    use itertools::{Either, Itertools};
    use nalgebra::Vector3;
    use num_traits::One;
    use rust_decimal_macros::dec;

    use crate::{
        //bsp::bsp2::{self, Bsp},
        basis::Basis,
        bsp::Bsp,
        cutter::Splitter,
        decimal::Dec,
        intersects::Intersects,
        polygon::Polygon,
        polygon_basis::PolygonBasis,
        shapes::rect,
    };

    use super::{Reversable, Vertices};

    #[test]
    fn boolean_diff() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let _basis = PolygonBasis {
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

    /*
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
        let bsp_2d_one = Bsp::<Line2D, Segment2D>::build(poly.get_segments_2d(&basis)).unwrap();

        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
        ];
        let poly = Polygon::new(points.to_vec()).unwrap();
        let bsp_2d_two = Bsp::<Line2D, Segment2D>::build(poly.get_segments_2d(&basis)).unwrap();

        let lines = union(bsp_2d_one, bsp_2d_two).into_iter().collect_vec();
        dbg!(lines.len());
        let polygons = Polygon::from_segments_2d(lines, &basis).unwrap();
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
    */
    #[test]
    fn boolean_union_4() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(4).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(4).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let _basis = PolygonBasis {
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
        let _basis = PolygonBasis {
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

        assert_eq!(result.sides.len(), 6);
    }
}
