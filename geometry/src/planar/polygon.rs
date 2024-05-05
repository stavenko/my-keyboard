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
    primitives_relation::{
        linear::{LinearIntersection, LinearRelation},
        linear_point::PointOnLine,
        planar::PlanarRelation,
        point_planar::PointPolygonRelation,
        relation::Relation,
    },
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

    fn split_my_segments_by(&self, other: &Self) -> Vec<Segment> {
        let mut my_segments: VecDeque<Segment> = self.get_segments().into();
        let mut results = Vec::new();
        'my: while let Some(my_segment) = my_segments.pop_front() {
            for tool in other.get_segments() {
                let tool_line = tool.get_line();

                match tool_line.relate(&my_segment) {
                    LinearRelation::Parallell => {}
                    LinearRelation::Crossed { .. } => {}
                    LinearRelation::Colinear => {}
                    LinearRelation::Opposite => {}
                    LinearRelation::Intersect(LinearIntersection::Origin(_point)) => {}
                    LinearRelation::Intersect(LinearIntersection::In(point)) => {
                        if !matches!(tool.relate(&point), PointOnLine::Outside) {
                            let s1 = Segment {
                                from: my_segment.from,
                                to: point,
                            };
                            let s2 = Segment {
                                from: point,
                                to: my_segment.to,
                            };
                            my_segments.push_front(s2);
                            my_segments.push_front(s1);
                            continue 'my;
                        }
                    }
                    LinearRelation::Independent => {}
                }
            }
            // no cut - push
            results.push(my_segment);
        }
        results
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

        let cross = &a.cross(&b);
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
        Ok(Self { vertices, plane })
    }

    pub fn new_with_plane(vertices: Vec<Vector3<Dec>>, plane: Plane) -> anyhow::Result<Self> {
        Ok(Self { vertices, plane })
    }

    pub fn get_plane(&self) -> Plane {
        self.plane.clone()
    }

    pub(crate) fn flip(mut self) -> Self {
        let plane = self.plane.flip();

        self.vertices.reverse();
        Self {
            vertices: self.vertices,
            plane,
        }
    }

    pub fn get_normal(&self) -> Vector3<Dec> {
        self.plane.normal()
    }
    fn filter_duplicates_and_opposites(mut segments: Vec<Segment>) -> Vec<Segment> {
        let mut totals = Vec::new();
        while let Some(segment) = segments.pop() {
            let flipped = segment.clone().flip();
            if let Some(ix) = totals.iter().position(|p| *p == flipped || *p == segment) {
                if flipped == totals[ix] {
                    totals.swap_remove(ix);
                }
            } else {
                totals.push(segment);
            }
        }
        totals
    }

    pub fn boolean_union(&self, tool: &Self) -> Vec<Self> {
        let my_plane = self.get_plane();
        let tool_plane = tool.get_plane();
        if !matches!(my_plane.relate(&tool_plane), PlanarRelation::Coplanar) {
            panic!("try to join polygons on different planes")
        }

        let my_segments = self.split_my_segments_by(tool);
        dbg!(my_segments.len());
        let tool_segments = tool.split_my_segments_by(self);
        dbg!(tool_segments.len());

        let my_segments = tool.drop_full_insides(my_segments);
        dbg!(my_segments.len());
        let tool_segments = self.drop_full_insides(tool_segments);
        dbg!(tool_segments.len());
        let joined = [my_segments, tool_segments].concat();

        dbg!(joined.len());
        let segments = Self::filter_duplicates_and_opposites(joined);
        dbg!(&segments);

        Self::from_segments(segments).unwrap()
    }

    pub fn boolean_diff(&self, tool: &Self) -> Vec<Self> {
        let my_plane = self.get_plane();
        let tool_plane = tool.get_plane();
        if !matches!(my_plane.relate(&tool_plane), PlanarRelation::Opposite) {
            panic!("try to diff polygons on incorrect planes")
        }
        let my_segments = self.split_my_segments_by(tool);
        let tool_segments = tool.split_my_segments_by(self);

        let my_segments = tool.drop_full_insides(my_segments);
        let tool_segments = self.take_full_insides(tool_segments);
        let joined = [my_segments, tool_segments].concat();

        let segments = Self::filter_duplicates_and_opposites(joined);
        Self::from_segments(segments).unwrap()
    }
    /*
    fn debug_segments(segments: &[Segment2D]) {
        println!(
            "polygon( points = [{}]);",
            segments
                .iter()
                .map(|s| s.from)
                .map(|v| format!("[{}, {}]", v.x, v.y))
                .join(", ")
        );
    }

    pub fn remove_opposites(mut segments: Vec<Segment2D>) -> Vec<Segment2D> {
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

    pub fn boolean_union(self, other: Self) -> Either<Vec<Polygon>, [Polygon; 2]> {
        if self.get_normal().dot(&other.get_normal()) == Dec::from(-1) {
            println!("WARN! flip opposite and diff");
            Either::Right([self, other])
        } else {
            let basis = Self::calculate_basis_2d(&self.vertices).unwrap();
            println!("{}", self.svg_debug(&basis));
            println!("{}", other.svg_debug(&basis));
            let my_segments = self.get_segments_2d(&basis);
            let other_segments = other.get_segments_2d(&basis);

            let my = Bsp::<Line2D, Segment2D>::build(my_segments.clone());
            let other_bsp = Bsp::<Line2D, Segment2D>::build(other_segments.clone());
            match (my, other_bsp) {
                (None, None) => Either::Right([self, other]),
                (None, Some(_)) => Either::Right([self, other]),
                (Some(_), None) => Either::Right([self, other]),
                (Some(my), Some(other_bsp)) => {
                    let mut total_segments = my_segments;
                    total_segments.extend(other_segments);

                    let (front_my, mut back_my) = my.sort_front_back(total_segments);
                    let (front_other, back_other) = other_bsp.sort_front_back(front_my);
                    back_my.extend(back_other);
                    let resulting_segments = Self::remove_opposites(back_my);

                    let polygons = Self::from_segments_2d(resulting_segments, &basis).unwrap();
                    polygons
                        .iter()
                        .for_each(|p| println!("{}", p.svg_debug(&basis)));
                    if polygons.iter().zip(&[&self, &other]).all(|(p, n)| *n == p) {
                        Either::Right([self, other])
                    } else {
                        Either::Left(polygons)
                    }
                }
            }
        }
    }
    */

    pub fn boolean_intersecion(self, _other: Self) -> Option<Self> {
        todo!("implement boolean_intersecion")
    }

    pub(crate) fn middle(&self) -> Vector3<Dec> {
        self.vertices
            .iter()
            .map(ToOwned::to_owned)
            .fold(Vector3::zeros(), |a, b| a + b)
            / Dec::from(self.vertices.len())
    }
}

/*
#[derive(Debug)]
pub enum PolygonRayIntersection {
    Inner(Vector3<Dec>),
    Edges(Vec<(SegmentIntersection, Segment)>),
}

impl Intersects<Polygon> for Ray {
    type Out = PolygonRayIntersection;

    fn intersects(&self, other: &Polygon) -> Option<Self::Out> {
        // First idea is to check, if ray in polygon plane, or not.
        // ray dir must be orthogonal to polygon plane

        let polygon_plane = other.get_plane();
        let is_in_polygon_plane = self
            .dir
            .dot(&other.get_normal())
            .round_dp(STABILITY_ROUNDING)
            .is_zero()
            && polygon_plane.is_point_on_plane(&self.origin);

        if is_in_polygon_plane {
            let spots = other
                .get_segments()
                .into_iter()
                .filter_map(|segment| self.intersects(&segment).map(|pt| (pt, segment.clone())))
                .collect_vec();

            if spots.is_empty() {
                None
            } else {
                Some(PolygonRayIntersection::Edges(spots))
            }
        } else if let Some(origin) = self.intersects(&polygon_plane) {
            // we need to test this point, if it in polygon bounds
            match other.is_point_inside(origin) {
                PointLoc::Inside => Some(PolygonRayIntersection::Inner(origin)),
                _ => None,
            }
        } else {
            None
        }
    }
}

impl Intersects<Plane> for Polygon {
    type Out = Ray;

    fn intersects(&self, other: &Plane) -> Option<Self::Out> {
        let pl = self.get_plane();
        pl.intersects(other)
    }
}

impl Intersects<Polygon> for Polygon {
    type Out = ();

    fn intersects(&self, other: &Polygon) -> Option<Self::Out> {
        let pl = self.get_plane();
        let ray = pl.intersects(other)?;

        match ray.intersects(other) {
            Some(PolygonRayIntersection::Inner(v)) => {
                unreachable!("ray in same plane with polygon");
            }
            Some(PolygonRayIntersection::Edges(edges)) => {}
            None => None,
        }

        /*
        if ray.intersects(self).is_some() && ray.intersects(other).is_some() {
            Some(())
        } else {
            None
        }
        */
    }
}
*/
#[cfg(test)]
mod tests {
    use itertools::Itertools;
    use nalgebra::Vector3;
    use rust_decimal_macros::dec;

    use crate::{decimal::Dec, planar::polygon::Polygon};

    use super::PolygonBasis;

    #[test]
    fn boolean_diff_common_edge() {
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
            Vector3::new(dec!(2).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly2 = Polygon::new(points.to_vec()).unwrap();

        let polygons = poly1.clone().boolean_diff(&poly2.flip());

        assert_eq!(polygons[0], poly1);
    }

    #[test]
    fn boolean_diff_with_hole() {
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
            Vector3::new(dec!(2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
        ];
        let poly2 = Polygon::new(points.to_vec()).unwrap();

        let polygons = poly2.boolean_diff(&poly1.flip());

        assert_eq!(polygons.len(), 2);

        let bigger = [
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(2).into(), dec!(0).into()),
        ];

        let bigger = Polygon::new(bigger.to_vec()).unwrap();

        let smaller = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];

        let smaller = Polygon::new(smaller.into_iter().rev().collect_vec()).unwrap();
        assert_eq!(polygons[1], bigger);
        assert_eq!(polygons[0], smaller);
    }

    #[test]
    fn boolean_diff_total_erase() {
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
            Vector3::new(dec!(2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
        ];
        let poly2 = Polygon::new(points.to_vec()).unwrap();

        let polygons = poly1.boolean_diff(&poly2.flip());

        assert_eq!(polygons.len(), 0);
    }

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

        let polygons = poly1.boolean_diff(&poly2.flip());

        let vv: Vec<Vector3<Dec>> = vec![
            Vector3::new(dec!(0).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(0).into(), dec!(1).into(), dec!(0).into()),
        ];
        let pp = Polygon::new(vv.to_vec()).unwrap();
        assert_eq!(polygons[0], pp);
    }

    #[test]
    fn boolean_union_adjacent() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly1 = Polygon::new(points.to_vec()).unwrap();

        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
        ];
        let poly2 = Polygon::new(points.to_vec()).unwrap();

        let polygons = poly1.boolean_union(&poly2);

        assert_eq!(polygons.len(), 1);

        let vv: Vec<Vector3<Dec>> = vec![
            Vector3::new(dec!(2).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(1).into(), dec!(0).into()),
        ];
        let result = Polygon::new(vv).unwrap();
        assert_eq!(polygons[0], result);
    }

    #[test]
    fn boolean_union_in() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly1 = Polygon::new(points.to_vec()).unwrap();

        let points = [
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(2).into(), dec!(0).into()),
        ];
        let poly2 = Polygon::new(points.to_vec()).unwrap();

        let polygons = poly1.boolean_union(&poly2);

        assert_eq!(polygons.len(), 1);

        let vv: Vec<Vector3<Dec>> = vec![
            Vector3::new(dec!(2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
        ];
        let result = Polygon::new(vv).unwrap();
        assert_eq!(polygons[0], result);
    }

    #[test]
    fn boolean_union_non_overlaping_segments() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly1 = Polygon::new(points.to_vec()).unwrap();

        let points = [
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(0).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(0).into(), dec!(2).into(), dec!(0).into()),
        ];
        let poly2 = Polygon::new(points.to_vec()).unwrap();

        let polygons = poly1.boolean_union(&poly2);

        assert_eq!(polygons.len(), 1);

        let vv: Vec<Vector3<Dec>> = vec![
            Vector3::new(dec!(0).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(0).into(), dec!(1).into(), dec!(0).into()),
        ];
        let result = Polygon::new(vv).unwrap();
        assert_eq!(polygons[0], result);
    }

    #[test]
    fn boolean_union_non_overlaping_polygons() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly1 = Polygon::new(points.to_vec()).unwrap();

        let points = [
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(1.1).into(), dec!(0).into(), dec!(0).into()),
            Vector3::new(dec!(1.1).into(), dec!(2).into(), dec!(0).into()),
        ];
        let poly2 = Polygon::new(points.to_vec()).unwrap();

        let polygons = poly1.clone().boolean_union(&poly2.clone());

        assert_eq!(polygons.len(), 2);

        assert_eq!(polygons[0], poly2);
        assert_eq!(polygons[1], poly1);
    }

    #[test]
    fn boolean_union_touching_with_hole_inside() {
        let points = [
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(2).into(), dec!(0).into()),
        ];
        let poly1 = Polygon::new(points.to_vec()).unwrap();

        let points = [
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly2 = Polygon::new(points.to_vec()).unwrap();

        let polygons = poly1.clone().boolean_union(&poly2.clone());

        assert_eq!(polygons.len(), 2);

        let bigger = [
            Vector3::new(dec!(2).into(), dec!(2).into(), dec!(0).into()),
            Vector3::new(dec!(2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(-2).into(), dec!(0).into()),
            Vector3::new(dec!(-2).into(), dec!(2).into(), dec!(0).into()),
        ];

        let bigger = Polygon::new(bigger.to_vec()).unwrap();

        let smaller = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];

        let smaller = Polygon::new(smaller.into_iter().rev().collect_vec()).unwrap();
        assert_eq!(polygons[1], bigger);
        assert_eq!(polygons[0], smaller);
    }

    /*
    #[test]
    fn splitting_diagonals() {
        let debug_poly_my = Polygon::new(
            [
                Vector3::new(
                    Dec::from(dec!(0.7500)),
                    Dec::from(dec!(-0.7500)),
                    Dec::from(dec!(0.2500)),
                ),
                Vector3::new(
                    Dec::from(dec!(0.7500)),
                    Dec::from(dec!(-0.7500)),
                    Dec::from(dec!(-0.2500)),
                ),
                Vector3::new(
                    Dec::from(dec!(-0.7500)),
                    Dec::from(dec!(-0.7500)),
                    Dec::from(dec!(-0.2500)),
                ),
                Vector3::new(
                    Dec::from(dec!(-0.7500)),
                    Dec::from(dec!(-0.7500)),
                    Dec::from(dec!(0.2500)),
                ),
            ]
            .to_vec(),
        )
        .unwrap();

        let debug_poly_tool = Polygon::new(
            [
                Vector3::new(
                    Dec::from(dec!(1.4142)),
                    Dec::from(dec!(-1.7678)),
                    Dec::from(dec!(0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(1.4142)),
                    Dec::from(dec!(-1.7678)),
                    Dec::from(dec!(-0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(-1.7678)),
                    Dec::from(dec!(1.4142)),
                    Dec::from(dec!(-0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(-1.7678)),
                    Dec::from(dec!(1.4142)),
                    Dec::from(dec!(0.250)),
                ),
            ]
            .to_vec(),
        )
        .unwrap();

        let _r1 = Polygon::new(
            [
                Vector3::new(
                    Dec::from(dec!(0.3964)),
                    Dec::from(dec!(-0.75)),
                    Dec::from(dec!(-0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(-1.7678)),
                    Dec::from(dec!(1.4142)),
                    Dec::from(dec!(-0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(-1.7678)),
                    Dec::from(dec!(1.4142)),
                    Dec::from(dec!(0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(0.3964)),
                    Dec::from(dec!(-0.75)),
                    Dec::from(dec!(0.250)),
                ),
            ]
            .to_vec(),
        )
        .unwrap();
        let _r2 = Polygon::new(
            [
                Vector3::new(
                    Dec::from(dec!(1.4142)),
                    Dec::from(dec!(-1.7678)),
                    Dec::from(dec!(0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(1.4142)),
                    Dec::from(dec!(-1.7678)),
                    Dec::from(dec!(-0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(0.3964)),
                    Dec::from(dec!(-0.75)),
                    Dec::from(dec!(-0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(0.3964)),
                    Dec::from(dec!(-0.75)),
                    Dec::from(dec!(0.250)),
                ),
            ]
            .to_vec(),
        )
        .unwrap();

        let (_, _) = debug_poly_tool.split(&debug_poly_my).unwrap();
        // assert_eq!(f, r1);
        //assert_eq!(b, r2);
    }
    */

    #[test]
    fn boolean_union_adjacent_but_not_fully() {
        //v
        //v
        let p1 = Polygon::new(
            [
                Vector3::new(
                    Dec::from(dec!(-0.3964466094067262377995778185)),
                    Dec::from(dec!(0.7499999999999999999999999995)),
                    Dec::from(dec!(-0.2500000000000000000000000000)),
                ),
                Vector3::new(
                    Dec::from(dec!(-1.4142135623730950488016887242)),
                    Dec::from(dec!(1.7677669529663688110021109052)),
                    Dec::from(dec!(-0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(-1.7677669529663688110021109052)),
                    Dec::from(dec!(1.4142135623730950488016887242)),
                    Dec::from(dec!(-0.250)),
                ),
                Vector3::new(
                    Dec::from(dec!(-1.1035533905932737622004221802)),
                    Dec::from(dec!(0.7499999999999999999999999992)),
                    Dec::from(dec!(-0.2500000000000000000000000000)),
                ),
            ]
            .to_vec(),
        )
        .unwrap();
        let p2 = Polygon::new(
            [
                Vector3::new(
                    Dec::from(dec!(-1.1035533905932737622004221802)),
                    Dec::from(dec!(0.7499999999999999999999999992)),
                    Dec::from(dec!(-0.2500000000000000000000000000)),
                ),
                Vector3::new(
                    Dec::from(dec!(-0.7500000000000000000000000002)),
                    Dec::from(dec!(0.3964466094067262377995778192)),
                    Dec::from(dec!(-0.2500000000000000000000000000)),
                ),
                Vector3::new(
                    Dec::from(dec!(-0.7500000000000000000000000001)),
                    Dec::from(dec!(0.7499999999999999999999999993)),
                    Dec::from(dec!(-0.2500000000000000000000000000)),
                ),
            ]
            .to_vec(),
        )
        .unwrap();
        let result = p1.boolean_union(&p2);
        assert_eq!(result.len(), 1);
    }
}
