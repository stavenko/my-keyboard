use rand::Rng;
use std::{collections::VecDeque, fmt};

use anyhow::anyhow;
use itertools::{Either, Itertools};
use nalgebra::{Vector2, Vector3};
use num_traits::{Signed, Zero};

use crate::bsp::{Bsp, Reversable};

use super::{
    decimal::{Dec, STABILITY_ROUNDING},
    line2d::Line2D,
    plane::Plane,
    polygon_basis::PolygonBasis,
    segment::Segment,
    segment2d::Segment2D,
    Face,
};

#[derive(Clone)]
pub struct Polygon {
    pub vertices: Vec<Vector3<Dec>>,
    plane: Plane,
}

impl PartialEq for Polygon {
    fn eq(&self, other: &Self) -> bool {
        if self.vertices.is_empty() && other.vertices.is_empty() {
            true
        } else if self.vertices.len() != other.vertices.len() {
            false
        } else {
            let first = self.vertices.first().expect("Duude");
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
        write!(f, "poly\n")?;
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

    pub fn polygon_union(one: Vec<Polygon>, other: Vec<Polygon>) -> Vec<Polygon> {
        dbg!(one.len());

        let r: Vec<Polygon> = one.into_iter().filter(|p| other.contains(p)).collect();
        dbg!(r.len());
        r
    }

    pub fn get_segments_with_basis(&self, basis: &PolygonBasis) -> Vec<Segment2D> {
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

    pub fn get_segments(&self, basis: &PolygonBasis) -> Vec<Segment2D> {
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

    pub fn vertices_to_segments_3d(vertices: &[Vector3<Dec>]) -> Vec<Segment> {
        let mut vv = vertices.iter().peekable();
        let mut segments = Vec::new();
        let first: Vector3<Dec> = vertices[0];
        while let Some(prev) = vv.next() {
            let next: Vector3<Dec> = if let Some(p) = vv.peek() { **p } else { first };
            segments.push(Segment::new(*prev, next));
        }
        segments
    }

    pub fn get_segments_3d(&self) -> Vec<Segment> {
        Self::vertices_to_segments_3d(&self.vertices)
    }

    pub fn calculate_plane(vertices: &[Vector3<Dec>]) -> anyhow::Result<Plane> {
        let v = vertices.first().ok_or(anyhow!("not a single point"))?;
        let w = vertices.get(1).ok_or(anyhow!("only one point"))?;

        let sum: Vector3<Dec> = vertices.iter().copied().fold(Vector3::zero(), |a, b| a + b);
        let center = sum / Dec::from(vertices.len());
        let u = center;
        let a = v - u;
        let b = w - u;

        let cross = &a.cross(&b);
        let mut plane = Plane::new_from_normal_and_point(cross.normalize(), center);
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
            dbg!("FLOP");
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

    fn restore_from_2d(
        points: Vec<Vector2<Dec>>,
        PolygonBasis { center, x, y }: &PolygonBasis,
    ) -> anyhow::Result<Self> {
        Self::new(
            points
                .into_iter()
                .map(|p| x * p.x + y * p.y + center)
                .collect(),
        )
    }

    fn join_segments(segments: Vec<Segment>) -> Vec<Segment> {
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
                // possibly, the last could be join with the first;
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

    fn join_segments_2d(segments: Vec<Segment2D>) -> Vec<Segment2D> {
        if segments.len() <= 2 {
            return segments;
        }

        let mut result = Vec::new();
        let mut segments: VecDeque<Segment2D> = segments.into();
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

    fn ordered_segments_2d(mut segments: Vec<Segment2D>) -> (Vec<Segment2D>, Vec<Segment2D>) {
        let mut ordered_segments: Vec<Segment2D> = Vec::new();
        let mut tails = Vec::new();
        loop {
            let mut taken = 0;
            for current_segment in segments {
                if let Some(last) = ordered_segments.last() {
                    if current_segment.from == last.to {
                        ordered_segments.push(current_segment);
                        taken += 1;
                    } else {
                        tails.push(current_segment)
                    }
                } else {
                    ordered_segments.push(current_segment);
                    taken += 1;
                }
            }
            if tails.is_empty() {
                break;
            }
            if taken == 0 {
                break;
            }
            segments = tails;
            tails = Vec::new()
        }

        (ordered_segments, tails)
    }

    fn ordered_segments(mut segments: Vec<Segment>) -> (Vec<Segment>, Vec<Segment>) {
        let mut ordered_segments: Vec<Segment> = Vec::new();
        let mut tails = Vec::new();
        loop {
            let mut taken = 0;
            for current_segment in segments {
                if let Some(last) = ordered_segments.last() {
                    if current_segment.from == last.to {
                        ordered_segments.push(current_segment);
                        taken += 1;
                    } else {
                        tails.push(current_segment)
                    }
                } else {
                    ordered_segments.push(current_segment);
                    taken += 1;
                }
            }
            if tails.is_empty() {
                break;
            }
            if taken == 0 {
                break;
            }
            segments = tails;
            tails = Vec::new()
        }

        (ordered_segments, tails)
    }

    pub fn segment_loops(mut segments: VecDeque<Segment2D>) -> Vec<Vec<Segment2D>> {
        let mut result = Vec::new();
        let mut new_loop: Vec<Segment2D> = Vec::new();
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
                    result.push(Self::join_segments_2d(new_loop));
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

    pub fn from_segments(
        segments: Vec<Segment2D>,
        basis: &PolygonBasis,
    ) -> anyhow::Result<Vec<Self>> {
        Self::segment_loops(segments.into())
            .into_iter()
            .map(|l| Polygon::restore_from_2d(l.into_iter().map(|s| s.from).collect_vec(), basis))
            .try_collect()
    }

    pub fn from_segments_3d_with_basis(
        segments: Vec<Segment>,
        plane: Plane,
    ) -> (Self, Vec<Segment>) {
        let (ordered_segments, tails) = Self::ordered_segments(segments);

        let vertices = Self::join_segments(ordered_segments)
            .into_iter()
            .map(|s| s.from)
            .collect_vec();

        let poly = Self { vertices, plane };

        (poly, tails)
    }

    fn project(&self, point: &Vector3<Dec>, basis: PolygonBasis) -> Vector2<Dec> {
        let PolygonBasis { center, x, y, .. } = basis;
        let x = (point - center).dot(&x);
        let y = (point - center).dot(&y);
        Vector2::new(x, y)
    }

    pub fn triangles(self) -> Vec<Face> {
        let mut result = Vec::new();
        for i in 1..(self.vertices.len() - 1) {
            let v0 = self.vertices[0];
            let v1 = self.vertices[i];
            let v2 = self.vertices[i + 1];

            result.push(Face::new_with_normal([v0, v1, v2], self.plane.normal()));
        }

        result
    }

    pub fn new(vertices: Vec<Vector3<Dec>>) -> anyhow::Result<Self> {
        let plane = Self::calculate_plane(&vertices)?;
        Ok(Self { vertices, plane })
    }

    pub fn new_with_plane(vertices: Vec<Vector3<Dec>>, plane: Plane) -> anyhow::Result<Self> {
        Ok(Self { vertices, plane })
    }

    /*
        pub fn new_with_basis(vertices: Vec<Vector3<Dec>>, basis: PolygonBasis) -> Self {
            let plane = Self::calculate_plane(&vertices).unwrap();
            let segments = Self::vertices_to_segments_3d(&vertices);

            Self::from_segments_3d_with_basis(segments, basis, plane).0
        }

    */
    pub fn get_plane(&self) -> Plane {
        self.plane.clone()
    }

    pub(crate) fn flip(mut self) -> Self {
        //let basis = self.basis;
        let plane = self.plane.flip();

        self.vertices.reverse();
        Self {
            vertices: self.vertices,
            //basis,
            plane,
        }
    }

    pub fn get_normal(&self) -> Vector3<Dec> {
        self.plane.normal()
    }

    pub(crate) fn calculate_triangles(&self) -> usize {
        self.vertices.len() - 2
    }

    pub(crate) fn has_opposite_segment(&self, segment: &Segment) -> bool {
        if let Some(index) = self.vertices.iter().position(|v| *v == segment.to) {
            //dbg!(index);
            let next = (index + 1) % self.vertices.len();
            let prev = (index + self.vertices.len() - 1) % self.vertices.len();
            if self.vertices[next] == segment.from {
                return true;
                //} else {
                //dbg!(self.vertices[next] == segment.from);
                //dbg!(self.vertices[prev] == segment.from);
            }
        }

        false
    }

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
                    dbg!("~~~~~~ remove ~~~~~~");
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
            println!("~~~~~~~~~start~~~~~~~~~~~~");
            println!("{}", self.svg_debug(&basis));
            println!("{}", other.svg_debug(&basis));
            let my_segments = self.get_segments_with_basis(&basis);
            let other_segments = other.get_segments_with_basis(&basis);

            //dbg!(&my_segments);
            //dbg!(&other_segments);

            let my = Bsp::<Line2D, Segment2D>::build(my_segments.clone());
            let other_bsp = Bsp::<Line2D, Segment2D>::build(other_segments.clone());
            match (my, other_bsp) {
                (None, None) => Either::Right([self, other]),
                (None, Some(_)) => Either::Right([self, other]),
                (Some(_), None) => Either::Right([self, other]),
                (Some(my), Some(other_bsp)) => {
                    let mut total_segments = my_segments;
                    total_segments.extend(other_segments);

                    //let  = my.clip(other_segments);
                    //let my_clipped = other_bsp.clip(my_segments);

                    let (front_my, mut back_my) = my.sort_front_back(total_segments);
                    dbg!(&front_my, &back_my);
                    let (front_other, back_other) = other_bsp.sort_front_back(front_my);
                    dbg!(&front_other, &back_other);
                    // Front is outside for positive
                    // And negative polygons are not supported:
                    // It is supported, when we have positive polygons with opposite normals
                    back_my.extend(back_other);
                    let resulting_segments = Self::remove_opposites(back_my);
                    // dbg!(&resulting_segments);

                    let polygons = Self::from_segments(resulting_segments, &basis).unwrap();
                    dbg!("collected");
                    polygons
                        .iter()
                        .for_each(|p| println!("{}", p.svg_debug(&basis)));
                    println!("~~~~~~~~~end~~~~~~~~~~~~");
                    if polygons.iter().zip(&[&self, &other]).all(|(p, n)| *n == p) {
                        Either::Right([self, other])
                    } else {
                        Either::Left(polygons)
                    }
                }
            }
        }
    }

    pub fn boolean_diff(self, other: Self) -> Option<Self> {
        todo!("implement boolean_diff")
    }

    pub fn boolean_intersecion(self, other: Self) -> Option<Self> {
        todo!("implement boolean_intersecion")
    }

    pub(crate) fn is_adjacent(&self, poly: &Polygon) -> bool {
        for s in self.get_segments_3d() {
            //dbg!("~~~~~~~~~~");
            //dbg!(&s);
            if poly.has_opposite_segment(&s) {
                return true;
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector2, Vector3};
    use num_traits::One;
    use rust_decimal_macros::dec;

    use crate::primitives::{basis, decimal::Dec, polygon::Polygon, segment2d::Segment2D};

    use super::PolygonBasis;

    #[test]
    fn check2d_polygon_assemble() {
        let points = [
            Vector3::new(dec!(1).into(), dec!(1).into(), dec!(0).into()),
            Vector3::new(dec!(1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(-1).into(), dec!(0).into()),
            Vector3::new(dec!(-1).into(), dec!(1).into(), dec!(0).into()),
        ];
        let poly = Polygon::new(points.to_vec()).unwrap();
        let basis = Polygon::calculate_basis_2d(&poly.vertices).unwrap();
        let segments: Vec<Segment2D> = poly.get_segments(&basis);
        let mut ps = Vec::new();
        for s in segments.iter() {
            ps.push(s.from);
        }
        let pp = Polygon::restore_from_2d(ps, &basis).unwrap();
        assert_eq!(pp, poly);
    }

    #[test]
    fn from_segments_1() {
        let z = PolygonBasis {
            x: Vector3::x(),
            y: Vector3::y(),
            center: Vector3::zeros(),
        };
        /*
        -4 4 -> -4.000, 1.000,
        -4.000 1.000 -> -1.00000000, 1.000,
        -1.00000000 4 -> -4, 4,
        -1.00000000 1.000 -> 4.000, 1.000,
        4.000 1.000 -> 4, 4,
        4 4 -> -1.00000000, 4,
        -4.00000 1.0000000000000000000000000001 -> -4, -4,
        -4 -4 -> 4, -4,
        4 -4 -> 4.00000, -1.00000,
        4.00000 -1.00000 -> -4.00000, -1.00000,
        -4.00000 -1.00000 -> -1.00000000, -1.00000000,
        -1.00000000 -1.00000000 -> -1.000000, 1.000000,

        -1.000000 1.000000 -> -4.00000, 1.0000000000000000000000000001,
        */
        let segments = [
            Segment2D {
                from: Vector2::new(Dec::one() * -4, Dec::one() * 4),
                to: Vector2::new(Dec::one() * -4, Dec::one() * 1),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * -4, Dec::one() * 1),
                to: Vector2::new(Dec::one() * -1, Dec::one() * 1),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * -1, Dec::one() * 4),
                to: Vector2::new(Dec::one() * -4, Dec::one() * 4),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * -1, Dec::one() * 1),
                to: Vector2::new(Dec::one() * 4, Dec::one() * 1),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * 4, Dec::one() * 1),
                to: Vector2::new(Dec::one() * 4, Dec::one() * 4),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * 4, Dec::one() * 4),
                to: Vector2::new(Dec::one() * -1, Dec::one() * 4),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * -4, Dec::one() * 1),
                to: Vector2::new(Dec::one() * -4, Dec::one() * -4),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * -4, Dec::one() * -4),
                to: Vector2::new(Dec::one() * 4, Dec::one() * -4),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * 4, Dec::one() * -4),
                to: Vector2::new(Dec::one() * 4, Dec::one() * -1),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * 4, Dec::one() * -1),
                to: Vector2::new(Dec::one() * -4, Dec::one() * -1),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * -4, Dec::one() * -1),
                to: Vector2::new(Dec::one() * -1, Dec::one() * -1),
            },
            Segment2D {
                from: Vector2::new(Dec::one() * -1, Dec::one() * -1),
                to: Vector2::new(Dec::one() * -1, Dec::one() * 1),
            },
            // ++++
            Segment2D {
                from: Vector2::new(Dec::one() * -1, Dec::one() * 1),
                to: Vector2::new(Dec::one() * -4, Dec::one() * 1),
            },
        ];

        let polygons = Polygon::from_segments(segments.to_vec(), &z).unwrap();
        assert_eq!(polygons.len(), 2);
    }
}
