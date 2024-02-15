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
    pub fn boolean_union(self, other: Self) -> Either<Self, (Self, Self)> {
        let my = Bsp::<Plane, Polygon>::build(self.polygons());
        let other_bsp = Bsp::<Plane, Polygon>::build(other.polygons());
        match (my, other_bsp) {
            (None, None) => Either::Right((self, other)),
            (None, Some(_)) => Either::Right((self, other)),
            (Some(_), None) => Either::Right((self, other)),
            (Some(my), Some(other)) => {
                let union = other.union(my); //.into_iter().map(|s| s.flip());
                let polygons = union.into_iter().collect_vec();
                let (result, rest) = Self::from_polygons(polygons);
                assert!(rest.is_empty());

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
                    if let Some((_, ps)) = acc.iter_mut().find(|(p, _)| *p == plane) {
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

    pub fn join_polygons_on_side(polygons: Vec<Polygon>) -> Vec<Polygon> {
        let mut result = Vec::new();
        if polygons.len() == 1 {
            return polygons;
        }
        let mut polygons: VecDeque<Polygon> = polygons.into();

        //let mut skipped = 0;
        //let mut current_polys = polygons.len();

        'outer: loop {
            let mut merges = Vec::new();
            let mut leftoffs = Vec::new();
            //dbg!(polygons.len());
            if let Some(mut poly) = polygons.pop_front() {
                'inner: while let Some(item) = polygons.pop_front() {
                    match poly.boolean_union(item) {
                        Either::Left(mut different) => {
                            dbg!("111");
                            merges.append(&mut different);
                            break 'inner;
                        }
                        Either::Right([p, next]) => {
                            poly = p;
                            leftoffs.push(next);
                        }
                    }
                }
            }
            dbg!(polygons.is_empty());
            dbg!(leftoffs.is_empty());
            assert!(!merges.is_empty());

            if leftoffs.is_empty() && polygons.is_empty() {
                result.append(&mut merges);
                break 'outer;
            }

            if polygons.is_empty() {
                polygons.extend(merges);
                polygons.extend(leftoffs);
                continue;
            }
        }
        result
    }

    pub fn from_polygons(mut polygons: Vec<Polygon>) -> (Self, Vec<Polygon>) {
        let (ordered, rest) = Self::order_polygons_by_adjacency(polygons);

        let sides = Self::group_by_sides(ordered);

        let sides = sides
            .into_iter()
            .map(|mut s| {
                let joined = Self::join_polygons_on_side(s.polygons);
                s.polygons = joined;
                s
            })
            .collect_vec();

        (Self { sides }, rest)
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

        dbg!("buildmesh");
        let bsp = Bsp::<Plane, Polygon>::build(faces).ok_or(anyhow!("failed to build bsp tree"))?;
        dbg!("ok convert to mesh");
        todo!("mesh creation");

        //  Ok(Self(bsp))
    }
}
