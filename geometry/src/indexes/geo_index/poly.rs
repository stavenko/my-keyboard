use std::{collections::HashSet, fmt};

use nalgebra::{Vector2, Vector3};
use num_traits::Zero;
use rstar::{Point, RTreeObject, AABB};
use stl_io::{Triangle, Vector};
use tap::TapFallible;
use uuid::Uuid;

use crate::{
    decimal::Dec,
    indexes::{aabb::Aabb, quadtree::Quadtree, vertex_index::PtId},
    linear::{segment::Segment, segment2d::Segment2D},
    planar::plane::Plane,
    polygon_basis::PolygonBasis,
    primitives::Face,
};

use super::{
    index::GeoIndex,
    rib::RibRef,
    seg::{Seg, SegRef},
};

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

    pub(crate) fn triangles(&self) -> anyhow::Result<Vec<Triangle>> {
        let mut index = Quadtree::default();
        let basis = self.calculate_polygon_basis();

        for s in self.segments_2d_iter(&basis) {
            index.insert(s.from);
            index.insert(s.to);
        }
        let mut contour: Vec<usize> = self
            .segments_2d_iter(&basis)
            .filter_map(|s| index.get_point_index(&s.from))
            .collect();

        if let Some(first) = contour.first() {
            contour.push(*first);
        }
        let tup_array: Vec<_> = index
            .linearize()
            .into_iter()
            .map(|v| (v.x.round_dp(9).into(), v.y.round_dp(9).into()))
            .collect();

        let contours = vec![contour];
        let mut t = cdt::Triangulation::new_from_contours(&tup_array, &contours).tap_err(|e| {
            panic!("{}", e);
        })?;
        while !t.done() {
            t.step().tap_err(|e| {
                panic!("{}", e);
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

    fn calculate_polygon_basis(&self) -> PolygonBasis {
        let plane = self.get_plane();
        let vertices = self.index.get_polygon_vertices(self.poly_id);
        let sum: Vector3<Dec> = vertices.iter().copied().fold(Vector3::zero(), |a, b| a + b);
        let center = sum / Dec::from(vertices.len());
        let v = vertices.first().expect("we have point");
        let plane_x = (v - center).normalize();
        let plane_y = plane.normal().cross(&plane_x).normalize();

        PolygonBasis {
            center,
            x: plane_x,
            y: plane_y,
        }
    }

    fn segments_2d_iter<'s>(
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

    pub(crate) fn get_segments(&self) -> impl Iterator<Item = SegRef<'a>> + '_ {
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
}

pub struct PolyRef<'a> {
    pub(super) poly_id: PolyId,
    pub(super) index: &'a GeoIndex,
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

#[derive(PartialEq, Eq, Hash, Clone, Copy)]
pub struct PolyId(pub(super) usize);

impl fmt::Debug for PolyId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PolyId:{}", self.0)
    }
}
