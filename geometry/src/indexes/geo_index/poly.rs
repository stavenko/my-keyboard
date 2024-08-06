use std::{
    collections::{HashMap, HashSet},
    fmt::{self, format},
    path::Path,
};

use itertools::Itertools;
use nalgebra::{Vector2, Vector3};
use num_traits::{Bounded, Zero};
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

    pub fn svg_debug_fill(&self, basis: &PolygonBasis, fill: &str) -> String {
        let mut items = Vec::new();
        let mut points = Vec::new();
        let colors = ["red", "green", "blue", "orange", "purple"];
        let mut path = Vec::new();

        let mut aabb = Vec::new();
        let mut min_distance_betnween_points = <Dec as Bounded>::max_value();

        for seg in self.segments_iter() {
            let pt = seg.from_pt();
            let to_pt = seg.to_pt();
            let v = self.index.vertices.get_point(pt);
            let to = self.index.vertices.get_point(to_pt);
            let v2 = basis.project_on_plane_z(&v) * Dec::from(1000);
            let to_v2 = basis.project_on_plane_z(&to) * Dec::from(1000);
            let d = (to_v2 - v2).magnitude();
            min_distance_betnween_points = min_distance_betnween_points.min(d);
            //let vv = basis.project_on_plane_z(v);
            let v3 = Vector3::new(v2.x, v2.y, Dec::zero());
            aabb.push(v3);
        }

        let aabb = Aabb::from_points(&aabb);
        let mut width = aabb.max.x - aabb.min.x;
        let mut height = aabb.max.y - aabb.min.y;
        let circle_size: Dec = min_distance_betnween_points * 4;
        let top = aabb.min.y - (circle_size / Dec::from(2));
        let left = aabb.min.x - (circle_size / Dec::from(2));
        width += circle_size * 2;
        height += circle_size * 2;
        let aspect = width / height;
        let img_width = Dec::from(800);
        let img_height = img_width / aspect;
        let font = (circle_size * Dec::from(0.7)).round_dp(1);
        dbg!(circle_size);

        items.push(format!("<svg viewBox=\" {left} {top} {width} {height}\" xmlns=\"http://www.w3.org/2000/svg\" width=\"{img_width}\" height=\"{img_height}\">"));
        items.push(format!(
            "<style> text{{ font: italic {font}pt sans-serif; }} </style>"
        ));
        for (ix, pt) in self.segments_iter().map(|s| s.from_pt()).enumerate() {
            let v = self.index.vertices.get_point(pt);
            let v2 = basis.project_on_plane_z(&v) * Dec::from(1000);
            points.push(format!(
                "<circle cx=\"{}\" cy=\"{}\" r=\"{circle_size}\" fill=\"{}\"/> <text x=\"{}\" y=\"{}\" text-anchor=\"middle\" >{pt} </text>
                ",
                v2.x.round_dp(9),
                v2.y.round_dp(9),
                colors[ix % colors.len()],
                v2.x.round_dp(9),
                v2.y.round_dp(9),
            ));

            if ix == 0 {
                path.push(format!("M {} {}", v2.x.round_dp(9), v2.y.round_dp(9)));
            } else {
                path.push(format!("L {} {}", v2.x.round_dp(9), v2.y.round_dp(9)));
            }
        }
        path.push("z".to_string());
        let c = colors[rand::thread_rng().gen_range(0..colors.len())];

        items.push(format!(
            "<path stroke=\"{}\" fill=\"{fill}\" stroke-width=\"0.0\" d = \"{}\" />",
            c,
            path.join(" ")
        ));
        items.extend(points);
        items.push("</svg>".to_string());
        items.join("\n")
    }

    pub(crate) fn serialized_polygon_pt(&self) -> String {
        let mut collect_vec = self
            .segments_iter()
            .map(|seg| format!("{}", seg.from_pt()))
            .collect_vec();
        collect_vec.reverse();
        collect_vec.join(", ")
    }

    pub(crate) fn triangles(&self) -> anyhow::Result<Vec<Triangle>> {
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

        let contours = vec![contour];

        let mut t = cdt::Triangulation::new_from_contours(&tup_array, &contours).tap_err(|e| {
            panic!("{}", e);
        })?;

        while !t.done() {
            t.step().tap_err(|e| {
                println!("basis {basis:?}");
                let mut parents = self
                    .index
                    .polygon_splits
                    .iter()
                    .flat_map(|(parent, children)| {
                        children.clone().into_iter().map(|child| (child, *parent))
                    })
                    .collect::<HashMap<_, _>>();

                let mut chain = vec![self.poly_id];
                while let Some(parent) = parents.remove(chain.last().expect("ok")) {
                    chain.push(parent);
                }
                chain.reverse();
                println!(
                    "chain: {}",
                    chain.into_iter().map(|p| format!("{p:?}")).join(" > ")
                );

                panic!("{e}");
            })?;
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
pub struct PolyId(pub usize);

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
