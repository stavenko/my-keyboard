use rand::Rng;
use rust_decimal_macros::dec;
use std::{collections::VecDeque, fmt};

use anyhow::anyhow;
use itertools::{Either, Itertools};
use nalgebra::Vector3;
use num_traits::{Signed, Zero};

use crate::{
    decimal::{Dec, STABILITY_ROUNDING},
    linear::{segment::Segment, segment2d::Segment2D},
    polygon_basis::PolygonBasis,
    primitives_relation::{point_planar::PointPolygonRelation, relation::Relation},
    reversable::Reversable,
};

use super::plane::Plane;

#[derive(Clone)]
pub struct Polygon {
    pub vertices: Vec<Vector3<Dec>>,
    plane: Plane,
}

pub enum PointLoc {
    Inside,
    Outside,
    Edge,
    Vertex,
}

impl PartialEq for Polygon {
    fn eq(&self, other: &Self) -> bool {
        if self.vertices.is_empty() && other.vertices.is_empty() {
            true
        } else if self.vertices.len() != other.vertices.len() {
            false
        } else {
            let first = self.vertices.first().unwrap();
            let other_ix = other.vertices.iter().position(|p| {
                let d = (p - first).magnitude_squared().round_dp(STABILITY_ROUNDING);

                d == Dec::zero()
            });

            other_ix.is_some_and(|other_ix| {
                for i in 1..self.vertices.len() {
                    let oix = (other_ix + i) % self.vertices.len();
                    let d = (self.vertices[i] - other.vertices[oix])
                        .magnitude_squared()
                        .round_dp(STABILITY_ROUNDING);
                    if d != Dec::zero() {
                        return false;
                    }
                }
                true
            })
        }
    }
}

impl fmt::Debug for Polygon {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "poly")?;
        for v in &self.vertices {
            writeln!(
                f,
                "  v {} {} {}",
                v.x.round_dp(4),
                v.y.round_dp(4),
                v.z.round_dp(4)
            )?;
        }
        Ok(())
    }
}

impl Reversable for Polygon {
    fn flip(self) -> Self {
        let plane = self.plane.flip();

        Self {
            vertices: self.vertices.into_iter().rev().collect(),
            //basis: self.basis,
            plane,
        }
    }
}

impl Polygon {
    pub fn svg_debug(&self, basis: &PolygonBasis) -> String {
        let mut items = Vec::new();
        let colors = ["red", "green", "blue", "orange", "purple"];
        let mut path = Vec::new();
        for (ix, v) in self.vertices.iter().enumerate() {
            let vv = basis.project_on_plane_z(v);
            if ix <= 2 {
                items.push(format!(
                    "<circle cx=\"{}\" cy=\"{}\" r=\"0.08\" fill=\"{}\"/> ",
                    vv.x.round_dp(4),
                    vv.y.round_dp(4),
                    colors[ix],
                ))
            }
            if ix == 0 {
                path.push(format!("M {} {}", vv.x.round_dp(4), vv.y.round_dp(4)));
            } else {
                path.push(format!("L {} {}", vv.x.round_dp(4), vv.y.round_dp(4)));
            }
        }
        path.push("z".to_string());
        let c = colors[rand::thread_rng().gen_range(0..colors.len())];
        items.push(format!(
            "<path stroke=\"{}\" stroke-width=\"0.06\" d = \"{}\" />",
            c,
            path.join(" ")
        ));
        items.join("\n")
    }

    /*
    pub fn split(&self, other: &Self) -> Option<(Self, Self)> {
        let tool_plane = self.get_plane();
        let my_polygon_plane = other.get_plane();

        match tool_plane.relate(&my_polygon_plane) {
            PlanarRelation::Opposite => None,
            PlanarRelation::Coplanar => None,
            PlanarRelation::Parallel => None,
            PlanarRelation::Intersect(line) => match line.relate(other) {
                LinearPolygonRelation::IntersectInPlane {
                    vertices, edges, ..
                } => match line.relate(self) {
                    LinearPolygonRelation::IntersectInPlane {
                        vertices: tool_vertices,
                        edges: tool_edges,
                        ..
                    } => {
                        let intersecion_len = edges.len() + vertices.len();
                        let tool_intersection = tool_vertices.len() + tool_edges.len();
                        if intersecion_len > 1 && tool_intersection > 1 {
                            let (f, b) = tool_plane.split(other);

                            match (f, b) {
                                (Some(f), Some(b)) => Some((f, b)),
                                _ => None,
                            }
                        } else {
                            None
                        }
                    }
                    _ => None,
                },
                LinearPolygonRelation::Parallell => None,
                LinearPolygonRelation::IntersectRib(_, _) => None,
                LinearPolygonRelation::IntersectPlaneInside(_) => None,
                LinearPolygonRelation::IntersectVertex(_) => None,
                LinearPolygonRelation::NonIntersecting => None,
            },
        }
    }
    */

    fn is_segment_inside(&self, segment: &Segment) -> bool {
        let mut points = 0;
        //let pts = [segment.from, segment.to];
        match self.relate(&segment.from) {
            PointPolygonRelation::In => {
                points += 1;
            }
            PointPolygonRelation::Vertex => {
                let pp = segment.from + segment.dir() * Dec::from(dec!(0.000001));
                if let PointPolygonRelation::In = self.relate(&pp) {
                    points += 1;
                }
            }
            PointPolygonRelation::Edge(_) => {
                let pp = segment.from + segment.dir() * Dec::from(dec!(0.000001));
                if let PointPolygonRelation::In = self.relate(&pp) {
                    points += 1;
                }
            }
            PointPolygonRelation::WithNormal => {
                //unimplemented!("with-normal")
            }
            PointPolygonRelation::OpposeToNormal => {
                //unimplemented!("oppose-to-normal")
            }
            PointPolygonRelation::InPlane => {}
        }

        match self.relate(&segment.to) {
            PointPolygonRelation::In => {
                points += 1;
            }
            PointPolygonRelation::Vertex => {
                let pp = segment.from + segment.dir() * Dec::from(dec!(0.9999999));
                if let PointPolygonRelation::In = self.relate(&pp) {
                    points += 1;
                }
            }
            PointPolygonRelation::Edge(_) => {
                let pp = segment.from + segment.dir() * Dec::from(dec!(0.999999));
                if let PointPolygonRelation::In = self.relate(&pp) {
                    points += 1;
                }
            }
            PointPolygonRelation::WithNormal => {
                //unimplemented!("with-normal")
            }
            PointPolygonRelation::OpposeToNormal => {
                //unimplemented!("oppose-to-normal")
            }
            PointPolygonRelation::InPlane => {}
        }

        points == 2
    }

    pub fn drop_full_insides(&self, tool_polygons: Vec<Segment>) -> Vec<Segment> {
        tool_polygons
            .into_iter()
            .filter(|poly| !self.is_segment_inside(poly))
            .collect()
    }

    pub fn take_full_insides(&self, tool_polygons: Vec<Segment>) -> Vec<Segment> {
        tool_polygons
            .into_iter()
            .filter(|poly| self.is_segment_inside(poly))
            .collect()
    }

    pub fn get_segments_2d(&self, basis: &PolygonBasis) -> Vec<Segment2D> {
        let mut vv = self.vertices.iter().peekable();
        let mut segments = Vec::new();
        let first: Vector3<Dec> = self.vertices[0];
        while let Some(v) = vv.next() {
            let prev = basis.project_on_plane_z(v);

            let next: Vector3<Dec> = if let Some(p) = vv.peek() { **p } else { first };
            let next = basis.project_on_plane_z(&next);
            segments.push(Segment2D::new(prev, next));
        }
        segments
    }

    pub fn get_segments(&self) -> Vec<Segment> {
        let mut vv = self.vertices.iter().peekable();
        let mut segments = Vec::new();
        let first: Vector3<Dec> = self.vertices[0];
        while let Some(&from) = vv.next() {
            let to: Vector3<Dec> = if let Some(p) = vv.peek() { **p } else { first };
            segments.push(Segment { from, to });
        }
        segments
    }

    pub fn calculate_plane(vertices: &[Vector3<Dec>]) -> anyhow::Result<Plane> {
        let u = vertices[0];
        let v = vertices[1];
        let w = vertices[vertices.len() - 1];
        let a = v - u;
        let b = w - u;

        if a.magnitude_squared().is_zero() || b.magnitude_squared().is_zero() {
            return Err(anyhow::anyhow!(
                "Cannot calculate plane of polygon, we got repeated points"
            ));
        }
        let cross = &a.cross(&b);
        //dbg!(cross.magnitude());
        if cross.magnitude().is_zero() {
            return Err(anyhow::anyhow!(
                "Cannot calculate plane of polygon, cross product have zero length"
            ));
        }
        let mut plane = Plane::new_from_normal_and_point(cross.normalize(), u);
        let x = a.normalize();
        let y = b.normalize();

        let mut total_area = Dec::zero();
        for current in 0..vertices.len() {
            let next = (current + 1) % vertices.len();
            let x1 = vertices[current].dot(&x);
            let y1 = vertices[current].dot(&y);
            let x2 = vertices[next].dot(&x);
            let y2 = vertices[next].dot(&y);
            total_area += x1 * y2 - x2 * y1;
        }
        if total_area.is_negative() {
            plane = plane.flip();
        }

        if total_area.is_zero() {
            return Err(anyhow!("Zero area"));
        }

        Ok(plane)
    }

    pub fn calculate_basis_2d(vertices: &[Vector3<Dec>]) -> anyhow::Result<PolygonBasis> {
        let plane = Self::calculate_plane(vertices)?;
        let sum: Vector3<Dec> = vertices.iter().copied().fold(Vector3::zero(), |a, b| a + b);
        let center = sum / Dec::from(vertices.len());
        let v = vertices.first().ok_or(anyhow!("not a single point"))?;
        let plane_x = (v - center).normalize();
        let plane_y = plane.normal().cross(&plane_x).normalize();

        Ok(PolygonBasis {
            center,
            x: plane_x,
            y: plane_y,
        })
    }

    fn join_segments(segments: Vec<Segment>) -> Vec<Segment> {
        if segments.len() <= 2 {
            return segments;
        }

        let mut result = Vec::new();
        let mut segments: VecDeque<Segment> = segments.into();
        while let Some(b) = segments.pop_front() {
            if let Some(n) = segments.pop_front() {
                match b.join(n) {
                    Either::Left(s) => {
                        segments.push_front(s);
                    }
                    Either::Right((b, n)) => {
                        result.push(b);
                        segments.push_front(n);
                    }
                }
            } else {
                result.rotate_left(1);
                if let Some(seg) = result.pop() {
                    match seg.join(b) {
                        Either::Left(s) => {
                            result.push(s);
                        }
                        Either::Right((b, n)) => {
                            result.push(n);
                            result.push(b);
                        }
                    }
                }
                result.rotate_right(1);
                break;
            }
        }
        result
    }

    fn segment_loops(mut segments: VecDeque<Segment>) -> Vec<Vec<Segment>> {
        let mut result = Vec::new();
        let mut new_loop: Vec<Segment> = Vec::new();

        loop {
            if let Some(last) = new_loop.last() {
                if let Some(ix) = segments.iter().position(|s| {
                    let d = (s.from - last.to)
                        .magnitude_squared()
                        .round_dp(STABILITY_ROUNDING);
                    d == Dec::zero()
                }) {
                    let item = segments.remove(ix).expect("we just found it");
                    new_loop.push(item);
                } else {
                    result.push(Self::join_segments(new_loop));
                    new_loop = Vec::new();
                }
            } else if let Some(f) = segments.pop_front() {
                new_loop.push(f);
            } else {
                break;
            }
        }

        result
    }

    pub fn from_segments(segments: Vec<Segment>) -> anyhow::Result<Vec<Self>> {
        Self::segment_loops(segments.into())
            .into_iter()
            .map(|l| Polygon::new(l.into_iter().map(|s| s.from).collect_vec()))
            .try_collect()
    }

    pub fn new(vertices: Vec<Vector3<Dec>>) -> anyhow::Result<Self> {
        let plane = Self::calculate_plane(&vertices)?;
        let this = Self { vertices, plane };
        Ok(this)
    }

    pub fn new_with_plane(vertices: Vec<Vector3<Dec>>, plane: Plane) -> anyhow::Result<Self> {
        Ok(Self { vertices, plane })
    }

    pub fn get_plane(&self) -> Plane {
        self.plane.clone()
    }

    pub fn get_normal(&self) -> Vector3<Dec> {
        self.plane.normal()
    }
}
