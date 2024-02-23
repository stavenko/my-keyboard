use anyhow::anyhow;
use itertools::Itertools;
use stl_io::Triangle;
use tap::TapFallible;

use crate::{bsp::Bsp, edge::Edge, primitives::plane::Plane};

use super::{hull::Hull, primitives::polygon::Polygon, surface::topology::Topology};

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
                    segments.swap_remove(ix);
                }
            }
        }
        joined
    }

    pub fn boolean_union(self, tool: Self) -> Self {
        println!("union");
        dbg!(self.polygons().len());
        dbg!(tool.polygons().len());
        let my = Bsp::<Plane, Polygon>::build(self.polygons());
        let tool_bsp = Bsp::<Plane, Polygon>::build(tool.polygons());

        match (my, tool_bsp) {
            (None, None) => self,
            (None, Some(_)) => self,
            (Some(_), None) => self,
            (Some(my), Some(tool_bsp)) => {
                let my_clipped = my.clip(self.polygons());
                let my_clipped = tool_bsp.clip(my_clipped);

                dbg!("my clipped by both");
                let tool_clipped = my.clip(tool.polygons());
                let tool_clipped = tool_bsp.clip(tool_clipped);
                dbg!("tool clipped by both");

                let (front_my, back_my) = my.sort_front_back(tool_clipped);
                let (front_tool, back_tool) = tool_bsp.sort_front_back(my_clipped);
                dbg!("sorted");

                let mut totals = back_my;
                dbg!(totals.len());
                let union = Polygon::polygon_union(front_my, front_tool);
                totals.extend(union);
                totals.extend(back_tool);
                let polygons = Self::remove_opposites(totals);

                let (result, _rest) = Self::from_polygons(polygons);

                println!("union done");
                result
            }
        }
    }

    pub fn boolean_diff(self, tool: Self) -> Self {
        let my = Bsp::<Plane, Polygon>::build(self.polygons());
        let tool_bsp = Bsp::<Plane, Polygon>::build(tool.polygons());
        match (my, tool_bsp) {
            (None, None) => self,
            (None, Some(_)) => self,
            (Some(_), None) => self,
            (Some(my), Some(tool_bsp)) => {
                let my_clipped = my.clip(self.polygons());
                let my_clipped = tool_bsp.clip(my_clipped);

                let tool_clipped = my.clip(tool.polygons());
                let tool_clipped = tool_bsp.clip(tool_clipped);

                let (front_my, _back_my) = my.sort_front_back(tool_clipped);
                let (_front_tool, back_tool) = tool_bsp.sort_front_back(my_clipped);

                let mut totals = back_tool;
                totals.extend(front_my.into_iter().map(|f| f.flip()));
                let polygons = Self::remove_opposites(totals);
                if polygons.is_empty() {
                    self
                } else {
                    let (result, _rest) = Self::from_polygons(polygons);
                    result
                }
            }
        }
    }

    pub fn polygons(&self) -> Vec<Polygon> {
        self.sides.iter().flat_map(|s| s.polygons.clone()).collect()
    }

    pub fn group_by_sides(
        polygons: impl IntoIterator<Item = Polygon>,
    ) -> impl Iterator<Item = Edge> {
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

        planes.into_iter().map(|(pl, ps)| Edge {
            plane: pl,
            polygons: ps,
        })
    }

    pub fn from_polygons(polygons: impl IntoIterator<Item = Polygon>) -> (Self, Vec<Polygon>) {
        let sides = Self::group_by_sides(polygons)
            .map(|s| s.join_polygons_on_side())
            .collect_vec();

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
        let polygons = value
            .sides
            .into_iter()
            .flatten()
            .chain(value.outer)
            .chain(value.inner);
        let (mesh, rest) = Mesh::from_polygons(polygons);
        dbg!(rest);

        Ok(mesh)
    }
}
