use std::collections::VecDeque;

use anyhow::anyhow;
use itertools::{Either, Itertools};
use stl_io::Triangle;
use tap::TapFallible;

use crate::{
    bsp::Bsp,
    edge::Edge,
    primitives::{plane::Plane, segment::Segment},
};

use super::{hull::Hull, primitives::polygon::Polygon, surface::topology::Topology};

pub struct MeshGraph {
    polygon: Polygon,
    missing: Vec<Segment>,
    adjacent: Vec<MeshGraph>,
}

impl MeshGraph {
    pub fn build_from_vec(polygons: &mut Vec<Polygon>) -> Self {
        let polygon = polygons.pop().expect("non-empty");
        let polygon_segments = polygon.get_segments_3d();
        let mut graph = MeshGraph {
            polygon,
            missing: polygon_segments,
            adjacent: Vec::new(),
        };
        graph.fill_adjacent(polygons);

        graph
    }

    fn fill_adjacent(&mut self, polygons: &mut Vec<Polygon>) {
        while let Some(segment) = self.missing.pop() {
            if let Some(ix) = polygons
                .iter()
                .position(|p| p.has_opposite_segment(&segment))
            {
                let polygon = polygons.swap_remove(ix);
                let missing = polygon.get_segments_3d();
                let mut value = MeshGraph {
                    polygon,
                    missing,
                    adjacent: Vec::new(),
                };
                value.fill_adjacent(polygons);
                self.adjacent.push(value);
            }
        }
    }

    fn collect(self) -> Vec<Polygon> {
        self.adjacent
            .into_iter()
            .flat_map(|f| f.collect())
            .chain(vec![self.polygon])
            .collect()
    }
}

#[derive(Debug)]
pub struct Mesh {
    pub sides: Vec<Edge>,
}

impl Mesh {
    pub fn remove_opposites(mut segments: Vec<Polygon>) -> Vec<Polygon> {
        let mut joined = Vec::new();
        while let Some(left) = segments.pop() {
            let inv = left.clone().flip();
            match segments.iter().position(|segment| *segment == inv) {
                None => joined.push(left),
                Some(ix) => {
                    dbg!("~~~remove ~~~");
                    dbg!(inv);
                    dbg!("~~~remove ~~~");

                    segments.swap_remove(ix);
                }
            }
        }
        joined
    }

    pub fn boolean_union(self, other: Self) -> Either<Self, (Self, Self)> {
        let my = Bsp::<Plane, Polygon>::build(self.polygons());
        let other_bsp = Bsp::<Plane, Polygon>::build(other.polygons());
        match (my, other_bsp) {
            (None, None) => Either::Right((self, other)),
            (None, Some(_)) => Either::Right((self, other)),
            (Some(_), None) => Either::Right((self, other)),
            (Some(my), Some(other_bsp)) => {
                //let mut total_polygons = self.polygons();
                // total_polygons.extend(other.polygons());
                let my_clipped = my.clip(self.polygons());
                let my_clipped = other_bsp.clip(my_clipped);

                let other_clipped = my.clip(other.polygons());
                let other_clipped = other_bsp.clip(other_clipped);

                //let total_polygons = my.clip(total_polygons);
                //let total_polygons = other_bsp.clip(total_polygons);
                let (front_my, back_my) = my.sort_front_back(other_clipped);
                let (front_other, back_other) = other_bsp.sort_front_back(my_clipped);

                dbg!(&front_my, &back_my);
                dbg!(&front_other, &back_other);

                let mut totals = back_my;
                let dups = dbg!(Polygon::polygon_union(front_my, front_other));
                //totals.extend(front_other);
                totals.extend(dups);
                totals.extend(back_other);
                //totals.extend(back_my);
                let polygons = Self::remove_opposites(totals);

                let (result, rest) = Self::from_polygons(polygons);
                dbg!(rest);

                Either::Left(result)
            }
        }
    }
    pub fn boolean_diff(self, other: Self) -> Option<Self> {
        todo!("implement boolean_diff")
    }
    pub fn boolean_intersecion(self, other: Self) -> Option<Self> {
        todo!("implement boolean_intersecion")
    }
    pub fn polygons(&self) -> Vec<Polygon> {
        self.sides.iter().flat_map(|s| s.polygons.clone()).collect()
    }
    pub fn group_by_sides(polygons: Vec<Polygon>) -> Vec<Edge> {
        let planes = polygons
            .into_iter()
            .map(|poly| (poly.get_plane(), poly))
            .fold(
                Vec::<(Plane, Vec<Polygon>)>::new(),
                |mut acc, (plane, poly)| {
                    if let Some((_, ps)) =
                        acc.iter_mut().find(|(p, _)| p.is_same_or_opposite(&plane))
                    {
                        ps.push(poly);
                    } else {
                        acc.push((plane, vec![poly]));
                    }
                    acc
                },
            );
        planes
            .into_iter()
            .map(|(pl, ps)| Edge {
                plane: pl,
                polygons: ps,
            })
            .collect()
    }

    pub fn order_polygons_by_adjacency(mut polygons: Vec<Polygon>) -> (Vec<Polygon>, Vec<Polygon>) {
        let mut result = Vec::new();
        if let Some(first) = polygons.pop() {
            result.push(first);
            let mut current_index = 0;
            while !polygons.is_empty() {
                let polygon = if let Some(poly) = result.get(current_index) {
                    polygons
                        .iter()
                        .position(|p| p.is_adjacent(&poly))
                        .map(|ix| polygons.swap_remove(ix))
                } else {
                    None
                };

                if let Some(polygon) = polygon {
                    result.push(polygon);
                } else if current_index < result.len() {
                    current_index += 1;
                } else {
                    break;
                }
            }
        }
        (result, polygons)
    }

    /*
        pub fn join_polygons_on_side(polygons: Vec<Polygon>) -> Vec<Polygon> {

            if polygons.len() <= 2 {
                return polygons;
            }
            let mut polygons: VecDeque<Polygon> = polygons.into();

            'outer: loop {
                let mut attempts = polygons.len();
                dbg!(polygons.len());
                if let Some(mut poly) = polygons.pop_front() {
                    let b = Polygon::calculate_basis_2d(&poly.vertices).unwrap();
                    'inner: while let Some(item) = polygons.pop_front() {
                        match poly.boolean_union(item.clone()) {
                            Either::Left(different) => {
                                dbg!("ok");
                                let len = different.len();
                                for (ix, p) in different.into_iter().enumerate() {
                                    if ix == 0 {
                                        // Solve polygon recombination problem, when sharing one same
                                        // point
                                        polygons.push_front(p);
                                    } else {
                                        polygons.push_back(p);
                                    }
                                }
                                /*
                                if len == 2 {
                                    panic!("TWO");
                                }
                                */

                                break 'inner;
                            }
                            Either::Right([p, next]) => {
                                dbg!("fail");

                                poly = p;
                                polygons.push_back(next);
                                if attempts == 0 {
                                    polygons.push_front(poly);
                                    break 'outer;
                                }
                                attempts -= 1;
                            }
                        }
                    }
                } else {
                    break;
                }
            }
            dbg!("+++++++++++++done++++++++++++");
            if polygons.len() > 1 {
                dbg!(polygons.len());
                panic!("DEBUG");
            }
            polygons.into()
        }
    */

    pub fn from_polygons(polygons: Vec<Polygon>) -> (Self, Vec<Polygon>) {
        let sides = Self::group_by_sides(polygons);
        let sides = sides
            .into_iter()
            .map(|s| {
                println!("------------------********---------------------");
                let s = s.join_polygons_on_side();
                dbg!(&s.polygons);
                s
            })
            .collect_vec();

        dbg!("merged");
        (Self { sides }, Vec::new())
    }
}

pub struct TriIter {
    inner: <Vec<Triangle> as IntoIterator>::IntoIter,
    size: usize,
}

impl ExactSizeIterator for TriIter {}

impl Iterator for TriIter {
    type Item = Triangle;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next()
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.size, Some(self.size))
    }
}

impl IntoIterator for Mesh {
    type Item = Triangle;

    type IntoIter = TriIter;

    fn into_iter(self) -> Self::IntoIter {
        let triangles = self
            .sides
            .into_iter()
            .filter_map(|edge| {
                edge.triangles()
                    .tap_err(|e| {
                        dbg!(e);
                    })
                    .ok()
            })
            .flatten()
            .collect_vec();
        let size = dbg!(triangles.len());

        TriIter {
            inner: triangles.into_iter(),
            size,
        }
    }
}

impl<T> TryFrom<Hull<T>> for Mesh
where
    T: Topology,
{
    type Error = anyhow::Error;

    fn try_from(value: Hull<T>) -> Result<Self, Self::Error> {
        let faces = value
            .sides
            .into_iter()
            .flatten()
            .chain(value.outer)
            .chain(value.inner);

        let bsp = Bsp::<Plane, Polygon>::build(faces).ok_or(anyhow!("failed to build bsp tree"))?;
        todo!("mesh creation");

        //  Ok(Self(bsp))
    }
}
