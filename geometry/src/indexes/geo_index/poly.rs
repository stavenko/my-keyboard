use std::{collections::HashSet, fmt};

use itertools::Itertools;
use nalgebra::{Vector2, Vector3};
use num_traits::Zero;
use rand::Rng;
use rstar::{Point, RTreeObject, AABB};
use stl_io::{Triangle, Vector};
use tap::TapFallible;

use crate::{
    decimal::Dec,
    indexes::{aabb::Aabb, vertex_index::PtId},
    linear::segment2d::Segment2D,
    planar::plane::Plane,
    polygon_basis::PolygonBasis,
    primitives::Face,
};

use super::{
    index::GeoIndex,
    rib::RibRef,
    seg::{Seg, SegRef},
};

#[derive(Debug, PartialEq)]
pub enum Side {
    Front,
    Back,
}

#[derive(Debug)]
pub struct PolygonSplitMeta {
    pub by_plane: Plane,
    pub other: PolyId,
    pub side: Side,
}

impl From<Aabb> for AABB<RtreePt> {
    fn from(value: Aabb) -> Self {
        let p1 = value.min;
        let p2 = value.max;
        AABB::from_corners(p1.into(), p2.into())
    }
}

impl From<Vector3<Dec>> for RtreePt {
    fn from(value: Vector3<Dec>) -> Self {
        Self([value.x, value.y, value.z])
    }
}

impl Point for RtreePt {
    type Scalar = Dec;

    const DIMENSIONS: usize = 3;

    fn generate(mut generator: impl FnMut(usize) -> Self::Scalar) -> Self {
        RtreePt([generator(0), generator(1), generator(2)])
    }

    fn nth(&self, index: usize) -> Self::Scalar {
        self.0[index]
    }

    fn nth_mut(&mut self, index: usize) -> &mut Self::Scalar {
        &mut self.0[index]
    }
}

impl RTreeObject for PolyRtreeRecord {
    type Envelope = AABB<RtreePt>;

    fn envelope(&self) -> Self::Envelope {
        self.1.into()
    }
}
#[derive(Clone, PartialEq, Debug)]
pub struct RtreePt([Dec; 3]);
#[derive(Debug, PartialEq)]
pub struct PolyRtreeRecord(pub(super) PolyId, pub(super) Aabb);

impl<'a> PolyRef<'a> {
    pub(crate) fn get_plane(&self) -> Plane {
        self.index.polygons[&self.poly_id].plane.to_owned()
    }

    pub(crate) fn segments_iter(&self) -> impl Iterator<Item = SegRef<'a>> + 'a {
        self.index
            .polygons
            .get(&self.poly_id)
            .into_iter()
            .flat_map(|p| &p.segments)
            .map(|&Seg { rib_id, dir }| SegRef {
                rib_id,
                dir,
                index: self.index,
            })
    }
    pub fn svg_debug(&self, vertices: Vec<Vector2<Dec>>) -> String {
        let mut items = Vec::new();
        let colors = ["red", "green", "blue", "orange", "purple"];
        let mut path = Vec::new();
        for (ix, vv) in vertices.iter().enumerate() {
            //let vv = basis.project_on_plane_z(v);
            if ix <= 2 {
                items.push(format!(
                    "<circle cx=\"{}\" cy=\"{}\" r=\"0.08\" fill=\"{}\"/> ",
                    vv.x.round_dp(9),
                    vv.y.round_dp(9),
                    colors[ix],
                ))
            }
            if ix == 0 {
                path.push(format!("M {} {}", vv.x.round_dp(9), vv.y.round_dp(9)));
            } else {
                path.push(format!("L {} {}", vv.x.round_dp(9), vv.y.round_dp(9)));
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

    pub(crate) fn triangles(&self) -> anyhow::Result<Vec<Triangle>> {
        /*
        if self.get_segments().count() == 3 {
            let vs = self.index.get_polygon_vertices(self.poly_id);
            let triangle = Triangle {
                normal: Vector::new([
                    self.get_plane().normal().x.into(),
                    self.get_plane().normal().y.into(),
                    self.get_plane().normal().x.into(),
                ]),
                vertices: vs
                    .into_iter()
                    .map(|v| Vector::new([v.x.into(), v.y.into(), v.z.into()]))
                    .collect_vec()
                    .try_into()
                    .expect("ok"),
            };

            return Ok(vec![triangle]);
        }
        */
        let basis = self.calculate_polygon_basis();
        let mut index = Vec::new();

        let mut dbg_2d_poly1 = Vec::new();
        let mut dbg_2d_poly2 = Vec::new();
        let mut contour: Vec<usize> = self
            .segments_2d_iter(&basis)
            .map(|s| {
                index.push(s.from);
                dbg_2d_poly2.push(s.from);
                index.len() - 1
            })
            .collect();

        if let Some(first) = contour.first() {
            contour.push(*first);
        }

        for p in &contour {
            let f = index[*p];
            dbg_2d_poly1.push(f);
        }

        let tup_array: Vec<_> = index
            .iter()
            .map(|v| (v.x.round_dp(9).into(), v.y.round_dp(9).into()))
            .collect();

        let c_len = contour.len();
        if c_len == 3 {
            dbg!(self.poly_id);
        }
        let contours = vec![contour];
        let mut t = cdt::Triangulation::new_from_contours(&tup_array, &contours).tap_err(|e| {
            panic!("{}", e);
        })?;
        while !t.done() {
            t.step().tap_err(|e| {
                dbg!(self.poly_id);
                let vertices = self
                    .segments_2d_iter(&basis)
                    .map(|s| s.from * Dec::from(10))
                    .collect_vec();
                let svg = self.svg_debug(vertices);
                let pts = self.points();
                for pt in &pts {
                    let v = self.index.vertices.get_point(*pt);
                    println!("{}, {}, {}", v.x, v.y, v.z);
                }
                let vvvs: Vec<_> = index.iter().map(|v| (v.x, v.y)).collect();
                for (x, y) in vvvs {
                    println!("{}, {}", x, y);
                }

                panic!("~~~\n\n{svg}+++++\n\n{pts:?}\n\n{e}");
            })?;
        }

        let res: Vec<Vector2<Dec>> = t
            .triangles()
            .flat_map(|(a, b, c)| [a, b, c])
            .map(|a| Vector2::new(tup_array[a].0.into(), tup_array[a].1.into()))
            .collect();

        if c_len > 14 {
            dbg!(&basis);
            println!(
                "--------------------SVG------------------\n{}\n{}\n{}\n\n~~~~~~~~~~~~~",
                self.svg_debug(dbg_2d_poly1),
                self.svg_debug(dbg_2d_poly2),
                self.svg_debug(res),
            );
        }

        let result = t
            .triangles()
            .map(|(a, b, c)| {
                let a: Vector3<Dec> =
                    basis.unproject(&Vector2::new(tup_array[a].0.into(), tup_array[a].1.into()));

                let b: Vector3<Dec> =
                    basis.unproject(&Vector2::new(tup_array[b].0.into(), tup_array[b].1.into()));
                let c: Vector3<Dec> =
                    basis.unproject(&Vector2::new(tup_array[c].0.into(), tup_array[c].1.into()));

                let face = Face::new([a, b, c]);

                Triangle {
                    normal: Vector::new([
                        face.normal.x.into(),
                        face.normal.y.into(),
                        face.normal.z.into(),
                    ]),
                    vertices: face
                        .vertices
                        .map(|na| Vector::new([na.x.into(), na.y.into(), na.z.into()])),
                }
            })
            .collect::<Vec<_>>();
        Ok(result)
    }

    pub(crate) fn calculate_polygon_basis(&self) -> PolygonBasis {
        let plane = self.get_plane();
        let vertices = self.index.get_polygon_vertices(self.poly_id);
        let sum: Vector3<Dec> = vertices.iter().copied().fold(Vector3::zero(), |a, b| a + b);
        let center = sum / Dec::from(vertices.len());
        let v = vertices
            .into_iter()
            .max_by(|a, b| {
                let aa = (a - center).magnitude_squared();
                let bb = (b - center).magnitude_squared();
                aa.cmp(&bb)
            })
            .expect("Cannot calculate max distance from center");

        let distance = (v - center).magnitude();

        let plane_x = (v - center) / distance;
        let plane_y = plane.normal().cross(&plane_x).normalize();

        PolygonBasis {
            center,
            x: plane_x,
            y: plane_y,
        }
    }

    pub(crate) fn segments_2d_iter<'s>(
        &'s self,
        basis: &'s PolygonBasis,
    ) -> impl Iterator<Item = Segment2D> + 's
    where
        's: 'a,
    {
        self.segments_iter().map(|s| Segment2D {
            from: basis.project_on_plane_z(&s.from()),
            to: basis.project_on_plane_z(&s.to()),
        })
    }

    pub(crate) fn middle(&self) -> Vector3<Dec> {
        let pts = self.index.get_polygon_vertices(self.poly_id);
        pts.iter()
            .map(Clone::clone)
            .fold(Vector3::zeros(), |a, b| a + b)
            / Dec::from(pts.len())
    }

    pub(crate) fn ribs(&self) -> impl Iterator<Item = RibRef<'_>> {
        self.index
            .get_polygon_ribs(&self.poly_id)
            .into_iter()
            .map(|rib_id| RibRef {
                rib_id,
                index: self.index,
            })
    }

    pub(crate) fn get_vertex(&self, v: crate::indexes::vertex_index::PtId) -> Vector3<Dec> {
        self.index.vertices.get_point(v)
    }

    pub fn get_segments<'b>(&'b self) -> impl Iterator<Item = SegRef<'a>> + 'a {
        self.index
            .polygons
            .get(&self.poly_id)
            .into_iter()
            .flat_map(|p| &p.segments)
            .map(|s| SegRef {
                rib_id: s.rib_id,
                dir: s.dir,
                index: self.index,
            })
    }

    pub(crate) fn points(&self) -> HashSet<PtId> {
        self.index.get_polygon_points(self.poly_id)
    }

    pub(crate) fn is_vertex_in_polygon_bounds(&self, point: Vector3<Dec>) -> bool {
        self.index
            .polygon_bounding_boxes
            .get(&self.poly_id)
            .is_some_and(|bb| bb.is_point_inside(point))
    }

    pub(crate) fn has_chain(&self, chain: &[Seg]) -> bool {
        let mut segs = self.index.polygons[&self.poly_id].segments.clone();
        if let Some(ix) = segs.iter().position(|seg| seg == chain.first().unwrap()) {
            segs.rotate_left(ix);
            for i in 0..chain.len() {
                if segs[i] != chain[i] {
                    return false;
                }
            }
            return true;
        }
        false
    }
}

pub struct PolyRef<'a> {
    pub(super) poly_id: PolyId,
    pub(super) index: &'a GeoIndex,
}

impl<'a> fmt::Debug for PolyRef<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.index.print_debug_polygon(self.poly_id))
    }
}

#[derive(Debug, Clone)]
pub struct Poly {
    pub(super) segments: Vec<Seg>,
    pub(super) plane: Plane,
    pub(super) aabb: Aabb,
}

impl PartialEq for Poly {
    fn eq(&self, other: &Self) -> bool {
        let len_is_same = self.segments.len() == other.segments.len();
        if self.segments.is_empty() && len_is_same {
            return true;
        }
        if let Some(other_ix) = other.segments.iter().position(|s| *s == self.segments[0]) {
            let mut checker = other.segments.clone();
            checker.rotate_left(other_ix);

            checker == self.segments
        } else {
            false
        }
    }
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, PartialOrd, Ord)]
pub struct PolyId(pub(super) usize);

impl PartialEq<usize> for PolyId {
    fn eq(&self, other: &usize) -> bool {
        self.0 == *other
    }
}

impl From<usize> for PolyId {
    fn from(value: usize) -> Self {
        Self(value)
    }
}

impl fmt::Debug for PolyId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PolyId:{}", self.0)
    }
}
