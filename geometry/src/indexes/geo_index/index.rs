use std::collections::BTreeMap;
use std::ops::Div;
use std::path::PathBuf;
use std::time::SystemTime;
use std::{
    collections::{HashSet, VecDeque},
    fmt::Debug,
    hash::Hash,
};

use anyhow::anyhow;
use itertools::{Either, Itertools};
use nalgebra::{ComplexField, Vector3};
use num_traits::{One, Signed, Zero};
use rstar::RTree;
use rust_decimal_macros::dec;

use crate::linear::line::Line;
use crate::planar::plane::Plane;
use crate::polygon_basis::PolygonBasis;
use crate::{
    decimal::Dec,
    indexes::{
        aabb::Aabb,
        geo_index::{poly::PolyRef, seg::SegRef},
        vertex_index::{PtId, VertexIndex},
    },
    primitives_relation::{planar::PlanarRelation, relation::Relation},
};

use super::face::{Face, FaceId, FaceRef, FaceToFaceRelation};
use super::geo_object::GeoObject;
use super::mesh::Mesh;
use super::poly::{Poly, PolyId, PolyRefMut, UnrefPoly};
use super::poly_rtree::FaceRtreeRecord;
use super::rib::RibRef;
use super::seg::SegmentRef;
use super::{
    mesh::{MeshId, MeshRef, MeshRefMut},
    rib::{Rib, RibId},
    seg::{Seg, SegmentDir},
};

#[derive(Debug)]
pub struct GeoIndex {
    pub(crate) vertices: VertexIndex,
    pub(super) face_index: RTree<FaceRtreeRecord>,
    pub(super) ribs: BTreeMap<RibId, Rib>,
    pub(super) faces: BTreeMap<FaceId, Face>,
    pub(super) meshes: BTreeMap<MeshId, Mesh>,
    pub(super) pt_to_ribs: BTreeMap<PtId, Vec<RibId>>,
    pub(super) rib_to_face: BTreeMap<RibId, Vec<FaceId>>,
    pub(super) partially_split_faces: BTreeMap<FaceId, Vec<RibId>>,
    pub(super) face_splits: BTreeMap<FaceId, Vec<FaceId>>,
    pub(super) rib_parent: BTreeMap<RibId, RibId>,
    pub(super) deleted_faces: BTreeMap<FaceId, Face>,
    pub(super) split_ribs: BTreeMap<RibId, Vec<RibId>>,
    //face_to_mesh: BTreeMap<FaceId, Vec<MeshId>>,
    face_split_debug: BTreeMap<FaceId, Option<FaceId>>,
    input_polygon_min_rib_length: Dec,
    points_precision: Dec,
    rib_counter: usize,
    face_counter: usize,
    mesh_counter: usize,
    current_color: usize,
    debug_path: PathBuf,
}

impl GeoIndex {
    pub fn new(aabb: Aabb) -> Self {
        let vertices = VertexIndex::new(aabb);

        Self {
            meshes: BTreeMap::default(),
            vertices,
            face_index: Default::default(),
            ribs: Default::default(),
            faces: Default::default(),
            pt_to_ribs: Default::default(),
            rib_to_face: Default::default(),
            partially_split_faces: Default::default(),
            face_splits: Default::default(),
            split_ribs: Default::default(),
            rib_parent: Default::default(),
            deleted_faces: Default::default(),
            input_polygon_min_rib_length: Default::default(),
            points_precision: Default::default(),
            rib_counter: Default::default(),
            face_counter: Default::default(),
            mesh_counter: Default::default(),
            face_split_debug: BTreeMap::new(),

            current_color: 0,
            debug_path: "/tmp/".into(),
            //default_mesh,
        }
    }

    pub fn face_debug(&mut self, face_id: impl Into<FaceId>, with_basis_of: Option<FaceId>) {
        let face_id = face_id.into();

        self.face_split_debug.insert(face_id, with_basis_of);
    }

    pub fn debug_svg_path(mut self, debug_path: PathBuf) -> Self {
        self.debug_path = debug_path;
        self
    }

    pub fn input_polygon_min_rib_length(
        mut self,
        input_polygon_min_rib_length: impl Into<Dec>,
    ) -> Self {
        self.input_polygon_min_rib_length = input_polygon_min_rib_length.into();
        self
    }

    pub fn points_precision(mut self, points_precision: impl Into<Dec>) -> Self {
        self.points_precision = points_precision.into();
        self
    }

    fn get_next_rib_id(&mut self) -> RibId {
        self.rib_counter += 1;
        RibId(self.rib_counter)
    }

    fn get_next_face_id(&mut self) -> FaceId {
        self.face_counter += 1;
        FaceId(self.face_counter)
    }

    fn get_next_mesh_id(&mut self) -> MeshId {
        self.mesh_counter += 1;
        MeshId(self.mesh_counter)
    }

    pub fn get_mutable_mesh(&mut self, mesh_id: MeshId) -> MeshRefMut {
        MeshRefMut {
            geo_index: self,
            mesh_id,
        }
    }

    fn collect_seg_chains(&self, mut ribs: Vec<RibId>) -> Vec<Vec<Seg>> {
        let mut result = Vec::new();
        if ribs.is_empty() {
            return Vec::new();
        }

        loop {
            let rib_id = ribs.pop().expect("non-empty ribs");

            let mut chain = VecDeque::new();
            chain.push_back(SegRef {
                rib_id,
                dir: SegmentDir::Fow,
                index: self,
            });

            'inner: loop {
                if let Some(from_ix) = ribs.iter().position(|rib_id| {
                    let rib = &self.ribs[rib_id];
                    let to = chain.back().unwrap().to_pt();

                    to == rib.0 || to == rib.1
                }) {
                    let to = chain.back().unwrap().to_pt();
                    let new_rib = ribs.swap_remove(from_ix);
                    let r = self.ribs[&new_rib];
                    chain.push_back(SegRef {
                        rib_id: new_rib,
                        dir: if r.0 == to {
                            SegmentDir::Fow
                        } else {
                            SegmentDir::Rev
                        },
                        index: self,
                    });
                } else if let Some(to_ix) = ribs.iter().position(|rib_id| {
                    let rib = &self.ribs[rib_id];
                    let from = chain.front().unwrap().from_pt();

                    from == rib.0 || from == rib.1
                }) {
                    let from = chain.front().unwrap().from_pt();
                    let new_rib = ribs.swap_remove(to_ix);
                    let r = self.ribs[&new_rib];
                    chain.push_front(SegRef {
                        rib_id: new_rib,
                        dir: if r.0 == from {
                            SegmentDir::Rev
                        } else {
                            SegmentDir::Fow
                        },
                        index: self,
                    });
                } else {
                    result.push(chain.into_iter().map(|c| c.seg()).collect_vec());
                    break 'inner;
                }
            }

            if ribs.is_empty() {
                break;
            }
        }
        result
    }

    fn split_segment_loop<'b>(
        &self,
        //segment_loop: &'b mut [Seg],
        segment_loop: &'b mut [SegRef<'b>],
        (from, to): (PtId, PtId),
    ) -> (&'b [SegRef<'b>], &'b [SegRef<'b>]) {
        let from = segment_loop
            .iter()
            //.map(|s| self.load_segref(s))
            .position(|sr| sr.from_pt() == from)
            .unwrap();

        segment_loop.rotate_left(from);

        let to = segment_loop
            .iter()
            //.map(|s| self.load_segref(s))
            .position(|sr| sr.from_pt() == to)
            .unwrap();

        let (fronts, backs) = segment_loop.split_at(to);
        (fronts, backs)
    }

    fn has_debug_req(&self, face_id: FaceId) -> Option<PolygonBasis> {
        self.face_split_debug.get(&face_id).and_then(|basis_face| {
            basis_face
                .and_then(|bf| {
                    if self.faces.contains_key(&bf) {
                        Some(bf.make_ref(self).calculate_2d_basis())
                    } else {
                        None
                    }
                })
                .or_else(|| {
                    if self.faces.contains_key(&face_id) {
                        Some(face_id.make_ref(self).calculate_2d_basis())
                    } else {
                        None
                    }
                })
        })
    }

    pub fn split_face_by_chain(&mut self, chain: Vec<Seg>, face_id: FaceId) -> [FaceId; 2] {
        let face_ref = self.load_face_ref(face_id);
        let chain = chain.into_iter().map(|s| s.to_ref(self)).collect_vec();
        let chain_last = chain.last().unwrap().to_pt();
        let chain_first = chain.first().unwrap().from_pt();
        let ribs_to_index = chain.iter().map(|s| s.rib_id).collect_vec();
        let reversed_chain = chain
            .clone()
            .into_iter()
            .map(|sr| sr.flip())
            .rev()
            .collect_vec();

        let mut segments = face_ref.segments(SegmentDir::Fow).collect_vec();

        let (fronts, backs) = self.split_segment_loop(&mut segments, (chain_last, chain_first));

        let fronts = [fronts, &chain.into_iter().collect_vec()].concat();
        let backs = [backs, &reversed_chain.into_iter().collect_vec()].concat();

        if fronts.len() < 3 || backs.len() < 3 {
            panic!("Less than 3 segments per polygon is not possible {face_id:?}");
        }

        let front_aabb = self.calculate_aabb_from_segments(fronts.clone().into_iter());
        let back_aabb = self.calculate_aabb_from_segments(backs.clone().into_iter());

        let face_one = Face::create(
            fronts.clone().into_iter().map(|sr| sr.seg()).collect(),
            face_ref.plane().to_owned(),
            front_aabb,
        );

        let face_two = Face::create(
            backs.into_iter().map(|sr| sr.seg()).collect(),
            face_ref.plane().to_owned(),
            back_aabb,
        );

        let face_one = self.insert_face(face_one);
        let face_two = self.insert_face(face_two);
        let new_ids = [face_one, face_two].into_iter().map(|a| a.0).collect_vec();
        self.replace_faces_in_meshes(face_id, &new_ids);
        if let Some(basis) = self.has_debug_req(face_id) {
            self.debug_svg_face("P-", face_id, &basis, &[]);
        }
        self.remove_face(face_id);
        [face_one, face_two]
            .into_iter()
            .for_each(|(child_face_id, _is_created)| {
                if let Some(basis) = self.has_debug_req(face_id) {
                    self.debug_svg_face(&format!("from-{face_id:?}-"), child_face_id, &basis, &[]);
                }
                for r in &ribs_to_index {
                    Self::save_index(&mut self.rib_to_face, *r, child_face_id);
                }
                self.unify_faces_ribs(child_face_id);
                self.create_common_ribs_for_adjacent_faces(child_face_id);

                Self::save_index(&mut self.face_splits, face_id, child_face_id);
            });

        new_ids.try_into().expect("ok")
    }

    pub fn find_first_bridge_point(
        &self,
        chain: &[SegRef<'_>],
        face_id: FaceId,
    ) -> Option<(PtId, PtId)> {
        let chain_pts = chain.iter().map(|s| s.from_pt()).collect_vec();
        let chain_center = chain_pts
            .iter()
            .fold(Vector3::zeros(), |a, p| a + self.vertices.get_point(*p))
            / Dec::from(chain.len());
        let points = chain_pts.len();
        let segs = [
            chain,
            &face_id
                .make_ref(self)
                .segments(SegmentDir::Fow)
                .collect_vec(),
        ]
        .concat();
        for ix in 0..points {
            let prev_ix = (points + ix - 1) % points;
            let next_ix = (ix + 1) % points;
            let origin = self.vertices.get_point(chain_pts[ix]);
            let main = self.vertices.get_point(chain_pts[next_ix]);
            let limit = self.vertices.get_point(chain_pts[prev_ix]);
            let main_dir = (main - origin).normalize();
            let limit_dir = (limit - origin).normalize();
            let face_normal = face_id.make_ref(self).plane().normal();
            let best_dir = (origin - chain_center).normalize();
            if let Some(p) = face_id
                .make_ref(self)
                .segments(SegmentDir::Fow)
                .map(|seg| seg.from_pt())
                .filter(|p| {
                    let test = self.vertices.get_point(*p);
                    let test_dir = test - origin;
                    let is_vec_dir_between_two_other_dirs = self.is_vec_dir_between_two_other_dirs(
                        face_normal,
                        main_dir,
                        limit_dir,
                        test_dir,
                    );
                    let is_bridge = self.is_bridge(&segs, (chain_pts[ix], *p));
                    if face_id.0 == 2 {
                        println!("is_bridge {is_bridge}, is between: {is_vec_dir_between_two_other_dirs}");
                    }
                    is_vec_dir_between_two_other_dirs && is_bridge
                })
                .max_by_key(|p| {
                    let test = self.vertices.get_point(*p);
                    let test_dir = (test - origin).normalize();
                    let test_dist = (test - origin).magnitude();
                    //println!("Maxing F: {p:?} [{}] {test_dist} {} ",test_dir.dot(&best_dir)/test_dist, test_dir.dot(&best_dir));
                    // Looks like heuristics, but works for now
                    test_dir.dot(&best_dir) / test_dist
                })
            {
                return Some((chain_pts[ix], p));
            }
        }
        None
    }

    pub fn find_opposing_bridge_point(
        &self,
        chain_pt: PtId,
        chain: &[SegRef<'_>],
        face_id: FaceId,
    ) -> Option<(PtId, PtId)> {
        let chain_pts = chain.iter().map(|s| s.from_pt()).collect_vec();
        let chain_center = chain_pts
            .iter()
            .fold(Vector3::zeros(), |a, p| a + self.vertices.get_point(*p))
            / Dec::from(chain.len());
        let testing_pts_ix = chain_pts
            .iter()
            .enumerate()
            .sorted_by_key(|(_, pt)| {
                let v = self.vertices.get_point(**pt);
                let vv = self.vertices.get_point(chain_pt);
                let first_point_dir = (vv - chain_center).normalize();
                let test_point_dir = (v - chain_center).normalize();

                test_point_dir.dot(&first_point_dir)
            })
            .collect::<Vec<_>>();
        let segs = [
            chain,
            &face_id
                .make_ref(self)
                .segments(SegmentDir::Fow)
                .collect_vec(),
        ]
        .concat();

        let points = chain_pts.len();
        for ix in testing_pts_ix.into_iter().map(|f| f.0) {
            if chain_pts[ix] == chain_pt {
                continue;
            }
            let prev_ix = (points + ix - 1) % points;
            let next_ix = (ix + 1) % points;
            let origin = self.vertices.get_point(chain_pts[ix]);
            let main = self.vertices.get_point(chain_pts[next_ix]);
            let limit = self.vertices.get_point(chain_pts[prev_ix]);
            let main_dir = (main - origin).normalize();
            let limit_dir = (limit - origin).normalize();
            let face_normal = face_id.make_ref(self).plane().normal();
            let best_dir = (origin - chain_center).normalize();
            if let Some(p) = face_id
                .make_ref(self)
                .segments(SegmentDir::Fow)
                .map(|seg| seg.from_pt())
                .filter(|p| {
                    let test = self.vertices.get_point(*p);
                    let test_dir = (test - origin).normalize();

                    let is_ok = self.is_vec_dir_between_two_other_dirs(
                        face_normal,
                        main_dir,
                        limit_dir,
                        test_dir,
                    );

                    is_ok && self.is_bridge(&segs, (chain_pts[ix], *p))
                })
                .max_by_key(|p| {
                    let test = self.vertices.get_point(*p);
                    let test_dir = (test - origin).normalize();
                    let test_dist = (test - origin).magnitude();
                    // Looks like heuristics, but works for now
                    test_dir.dot(&best_dir) / test_dist
                })
            {
                return Some((chain_pts[ix], p));
            }
        }
        None
    }

    pub fn split_face_by_closed_chain(
        &mut self,
        face_id: FaceId,
        mut chain: Vec<Seg>,
    ) -> Vec<FaceId> {
        let face_ref = self.load_face_ref(face_id);
        let pb = face_ref.calculate_2d_basis();
        let area = |s: SegRef| {
            s.from().dot(&pb.x) * s.to().dot(&pb.y) - s.to().dot(&pb.x) * s.from().dot(&pb.y)
        };

        let poly_area: Dec = face_ref.segments(SegmentDir::Fow).map(area).sum();
        let loop_area: Dec = chain.iter().map(|s| self.load_segref(s)).map(area).sum();
        if (loop_area * poly_area).is_positive() {
            chain = chain.into_iter().map(|s| s.flip()).rev().collect_vec();
        }

        let chain = chain
            .into_iter()
            .map(|s| self.load_segref(&s))
            .collect_vec();

        /*
        for s in &chain {
        }
        */

        if let Some(first) = self.find_first_bridge_point(&chain, face_id) {
            if let Some(opposite) = self.find_opposing_bridge_point(first.0, &chain, face_id) {
                let chain_segs = chain.into_iter().map(|s| s.seg()).collect_vec();
                let (rib, dir) = Rib::build(first.0, first.1);
                let (rib_id, _) = self.insert_rib(rib);
                let seg_first = Seg { rib_id, dir };
                let seg_first_rev = seg_first.flip();
                let (rib, dir) = Rib::build(opposite.0, opposite.1);
                let (rib_id, _) = self.insert_rib(rib);
                let seg_opposite = Seg { rib_id, dir };
                let seg_opposite_rev = seg_opposite.flip();

                let mut chain_srs = chain_segs
                    .clone()
                    .into_iter()
                    .map(|s| self.load_segref(&s))
                    .collect_vec();
                let (chain_front, chain_back) =
                    self.split_segment_loop(&mut chain_srs, (first.0, opposite.0));

                let mut polygon_segments = self
                    .load_face_ref(face_id)
                    .segments(SegmentDir::Fow)
                    .collect_vec();

                let (poly_front, poly_back) =
                    self.split_segment_loop(&mut polygon_segments, (opposite.1, first.1));

                let one = [
                    chain_front.iter().map(|sr| sr.seg()).collect_vec(),
                    vec![seg_opposite],
                    poly_front.iter().map(|sr| sr.seg()).collect_vec(),
                    vec![seg_first_rev],
                ]
                .concat();
                let two = [
                    chain_back.iter().map(|sr| sr.seg()).collect_vec(),
                    vec![seg_first],
                    poly_back.iter().map(|sr| sr.seg()).collect_vec(),
                    vec![seg_opposite_rev],
                ]
                .concat();
                let three = chain_segs.iter().map(|s| s.flip()).rev().collect_vec();
                let plane = self.faces[&face_id].plane().clone();

                let aabb =
                    self.calculate_aabb_from_segments(one.iter().map(|s| self.load_segref(s)));
                let face_one = self.insert_face(Face::create(one, plane.clone(), aabb));

                let aabb =
                    self.calculate_aabb_from_segments(two.iter().map(|s| self.load_segref(s)));
                let face_two = self.insert_face(Face::create(two, plane.clone(), aabb));

                let aabb =
                    self.calculate_aabb_from_segments(three.iter().map(|s| self.load_segref(s)));
                let face_three = self.insert_face(Face::create(three, plane, aabb));

                let faces = [face_one, face_two, face_three]
                    .into_iter()
                    .map(|a| a.0)
                    .collect_vec();

                if let Some(basis_face) = self.face_split_debug.get(&face_id) {
                    let basis = if let Some(face) = basis_face {
                        face.make_ref(self).calculate_2d_basis()
                    } else {
                        face_id.make_ref(self).calculate_2d_basis()
                    };
                    self.debug_svg_face("P-", face_id, &basis, &[]);
                }

                self.replace_faces_in_meshes(face_id, &faces);
                [face_one, face_two, face_three]
                    .into_iter()
                    .map(|(child_face_id, _is_created)| {
                        if let Some(basis_face) = self
                            .face_split_debug
                            .get(&face_id)
                            .or(self.face_split_debug.get(&child_face_id))
                        {
                            let basis = if let Some(face) = basis_face {
                                face.make_ref(self).calculate_2d_basis()
                            } else {
                                child_face_id.make_ref(self).calculate_2d_basis()
                            };
                            self.debug_svg_face(
                                &format!("from-{face_id:?}-"),
                                child_face_id,
                                &basis,
                                &[],
                            );
                        }
                        Self::save_index(&mut self.face_splits, face_id, child_face_id);
                        self.unify_faces_ribs(child_face_id);
                        self.create_common_ribs_for_adjacent_faces(child_face_id);
                        child_face_id
                    })
                    .collect_vec();
                self.remove_face(face_id);
                return faces;
            }
        }

        panic!("Cannot find bridge points for face: {face_id:?}");
    }

    fn create_common_ribs_for_adjacent_faces(&mut self, tool_face_id: FaceId) {
        let tool_aabb = *self.faces[&tool_face_id].aabb();
        let tool_plane = &self.faces[&tool_face_id].plane();
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;

        let faces_in_proximity = self
            .face_index
            .locate_in_envelope_intersecting(&tool_aabb.into())
            .map(|o| o.0)
            .filter(|&p| p != tool_face_id)
            .filter(|p| {
                !matches!(
                    self.faces[p].plane().relate(*tool_plane),
                    PlanarRelation::Intersect(_)
                )
            })
            .filter(|p| {
                let tool_ribs = self
                    .load_face_ref(tool_face_id)
                    .segments(SegmentDir::Fow)
                    .map(|s| s.rib_id)
                    .collect::<HashSet<_>>();
                let src_ribs = self
                    .load_face_ref(*p)
                    .segments(SegmentDir::Fow)
                    .map(|s| s.rib_id)
                    .collect::<HashSet<_>>();

                let mut intersection = src_ribs.intersection(&tool_ribs);
                if let Some(first) = intersection.next() {
                    let sr_first = SegRef {
                        rib_id: *first,
                        dir: SegmentDir::Fow,
                        index: self,
                    };
                    let line_first = Line {
                        origin: sr_first.from(),
                        dir: sr_first.dir().normalize(),
                    };

                    for other in intersection {
                        let sr_other = SegRef {
                            rib_id: *other,
                            dir: SegmentDir::Fow,
                            index: self,
                        };
                        let one_on_line =
                            line_first.distance_to_pt_squared(sr_other.from()) < vertex_pulling_sq;
                        let other_on_line =
                            line_first.distance_to_pt_squared(sr_other.to()) < vertex_pulling_sq;

                        if !(one_on_line && other_on_line) {
                            return true;
                        }
                    }
                }

                false
            })
            .collect_vec();

        // Collect
        for near_face in faces_in_proximity {
            let tool_ribs = self
                .load_face_ref(tool_face_id)
                .segments(SegmentDir::Fow)
                .map(|s| s.rib_id)
                .collect::<HashSet<_>>();
            let src_ribs = self
                .load_face_ref(near_face)
                .segments(SegmentDir::Fow)
                .map(|s| s.rib_id)
                .collect::<HashSet<_>>();

            let difference_src_tool = src_ribs.difference(&tool_ribs).copied().collect_vec();
            let mut chains_src_tool = self.collect_seg_chains(difference_src_tool);
            while let Some(pos) = chains_src_tool
                .iter()
                .position(|chain| self.is_chain_inside_face(chain, tool_face_id))
            {
                let chain = chains_src_tool.swap_remove(pos);
                for c in chain {
                    Self::save_index(&mut self.partially_split_faces, tool_face_id, c.rib_id);
                }
            }
            let difference_tool_src = tool_ribs.difference(&src_ribs).copied().collect_vec();
            let mut chain_tool_src = self.collect_seg_chains(difference_tool_src);

            while let Some(pos) = chain_tool_src
                .iter()
                .position(|chain| self.is_chain_inside_face(chain, near_face))
            {
                let chain = chain_tool_src.swap_remove(pos);
                for c in chain {
                    Self::save_index(&mut self.partially_split_faces, near_face, c.rib_id);
                }
            }
        }
    }

    fn is_bridge(&self, segments: &[SegRef<'_>], (from, to): (PtId, PtId)) -> bool {
        let affected = segments
            .iter()
            .filter(|sr| !(sr.has(from) || sr.has(to)))
            .collect_vec();

        let segment_ref = SegmentRef::new(from, to, self);
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;

        for sr in segments
            .iter()
            .filter(|sr| segment_ref.distance_to_pt_squared(sr.from()).abs() < vertex_pulling_sq)
        {
            if let Some((a, _b)) = segment_ref.get_intersection_params_seg_ref(sr) {
                if a > Dec::zero() && a < Dec::one() {
                    return false;
                }
            }
        }

        let is_some_intersected = affected
            .into_iter()
            .filter_map(|sr| segment_ref.get_intersection_params_seg_ref(sr))
            .any(|(a, b)| a > Dec::zero() && a < Dec::one() && b > Dec::zero() && b < Dec::one());
        !is_some_intersected
    }

    pub(crate) fn load_segref(&self, seg: &Seg) -> SegRef<'_> {
        SegRef {
            rib_id: seg.rib_id,
            dir: seg.dir,
            index: self,
        }
    }

    pub fn split_faces_by_orphan_ribs(&mut self) {
        while let Some((face_id, cutting_chain, leftoffs)) = self
            .partially_split_faces
            .iter()
            .sorted_by_key(|(p, _)| *p)
            .find_map(|(face_id, ribs)| {
                let ribs = ribs
                    .iter()
                    .flat_map(|rib_id| {
                        if self.ribs.contains_key(rib_id) {
                            vec![*rib_id]
                        } else {
                            self.get_ribs_with_root_parent(*rib_id)
                        }
                    })
                    .collect_vec();
                if face_id.0 == 2482 {
                    for r in &ribs {
                        println!("Split ribs for face 2482{}", r.make_ref(self))
                    }
                }

                let mut chains = self.collect_seg_chains(ribs);

                if let Some((ix, mut cutting_set)) =
                    chains.iter().enumerate().find_map(|(ix, chain)| {
                        if let Some((splitting_chain, leftoffs)) =
                            self.collect_chain_splitting_face(*face_id, chain.clone())
                        {
                            Some((
                                ix,
                                (
                                    splitting_chain,
                                    leftoffs.into_iter().map(|s| s.rib_id).collect_vec(),
                                ),
                            ))
                        } else {
                            None
                        }
                    })
                {
                    chains.swap_remove(ix);
                    let leftoffs = chains.into_iter().flatten().map(|s| s.rib_id);
                    cutting_set.1.extend(leftoffs);

                    Some((*face_id, cutting_set.0, cutting_set.1))
                } else {
                    None
                }
            })
        {
            let new_polies = if self.is_chain_circular(&cutting_chain) {
                self.split_face_by_closed_chain(face_id, cutting_chain)
            } else {
                self.split_face_by_chain(cutting_chain, face_id).to_vec()
            };

            self.partially_split_faces.remove(&face_id);

            for rib in leftoffs {
                for p in &new_polies {
                    Self::save_index(&mut self.partially_split_faces, *p, rib);
                }
            }
        }
    }

    pub(crate) fn split_floating_rib_using_indexed_pts(
        &mut self,
        pts: &[PtId],
        rib_id: RibId,
    ) -> Vec<RibId> {
        let mut vs_peekable = pts
            .iter()
            .chain([
                &rib_id.make_ref(self).from_pt(),
                &rib_id.make_ref(self).to_pt(),
            ])
            .sorted_by_key(|pt| {
                (self.vertices.get_point(**pt) - rib_id.make_ref(self).from())
                    .dot(&rib_id.make_ref(self).dir())
            })
            .map(|pt| self.vertices.get_point(*pt))
            .collect_vec()
            .into_iter()
            .peekable();

        let mut replacement = Vec::new();
        while let Some(v) = vs_peekable.next() {
            if let Some(vn) = vs_peekable.peek() {
                replacement.push(self.save_segment_unchecked((v, *vn)));
            }
        }
        let new_ids = replacement.iter().map(|s| s.rib_id).collect();

        new_ids
    }
    pub(crate) fn split_rib_in_face_using_indexed_pts(
        &mut self,
        pts: &[PtId],
        rib_id: RibId,
        face_id: FaceId,
    ) -> Vec<RibId> {
        if face_id.0 == 2495 && rib_id == 3847 {
            println!("DO something with {face_id:?} and {rib_id:?}");
        }
        let fr = self.load_face_ref(face_id);
        if let Some(ix) = fr
            .segments(SegmentDir::Fow)
            .map(|s| s.rib_id)
            .collect_vec()
            .iter()
            .position(|&r| r == rib_id)
        {
            let fr = self.load_face_ref(face_id);
            let seg_ref = fr.segments(SegmentDir::Fow).nth(ix).unwrap();

            let mut vs_peekable = pts
                .iter()
                .chain([&seg_ref.from_pt(), &seg_ref.to_pt()])
                .sorted_by_key(|pt| {
                    (self.vertices.get_point(**pt) - seg_ref.from()).dot(&seg_ref.dir())
                })
                .map(|pt| self.vertices.get_point(*pt))
                .collect_vec()
                .into_iter()
                .peekable();

            let mut replacement = Vec::new();
            while let Some(v) = vs_peekable.next() {
                if let Some(vn) = vs_peekable.peek() {
                    replacement.push(self.save_segment_unchecked((v, *vn)));
                }
            }
            let new_ids = replacement.iter().map(|s| s.rib_id).collect();

            if face_id.0 == 2495 {
                println!(
                    "DO something with {face_id:?} and {rib_id:?} replace with [{replacement:?}]"
                );
            }

            for r in &replacement {
                Self::save_index(&mut self.rib_to_face, r.rib_id, face_id);
                self.rib_parent.insert(r.rib_id, rib_id);
            }

            if let Some(face) = self.faces.get_mut(&face_id) {
                face.replace_segments(ix, replacement);
            }

            new_ids
        } else {
            panic!("NO RIB");
        }
    }

    fn calculate_plane_for_segs(&self, segs: &[Seg]) -> anyhow::Result<Plane> {
        let vertices = segs
            .iter()
            .map(|s| s.from(&self.ribs))
            .map(|pt| self.vertices.get_point(pt))
            .collect_vec();
        self.calculate_plane(&vertices)
    }

    fn calculate_plane(&self, vertices: &[Vector3<Dec>]) -> anyhow::Result<Plane> {
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
            plane.flip();
        }

        if total_area.is_zero() {
            return Err(anyhow!("Zero area"));
        }

        Ok(plane)
    }

    fn save_polygon_new(&mut self, vertices: &[Vector3<Dec>]) -> anyhow::Result<Poly> {
        let mut segs = Vec::new();
        let aabb = Aabb::from_points(vertices);
        for i in 0..vertices.len() {
            let next = (i + 1) % vertices.len();
            let from = vertices[i];
            let to = vertices[next];
            segs.push(self.save_segment((from, to))?);
        }

        let plane = self.calculate_plane_for_segs(&segs)?;
        let face = Face::create(segs, plane, aabb);
        let (face_id, _is_created) = self.insert_face(face.clone());

        if self.is_opposite_face(face_id, face) {
            Ok(Poly::rev(face_id))
        } else {
            Ok(Poly::fow(face_id))
        }
    }

    fn find_older_and_replace_face_in_poly(&mut self, poly: UnrefPoly) {
        let poly_face_id = poly.make_ref(self).face_id();
        if let Some(old_face_id) = self.find_same_face_with_other_id(poly_face_id) {
            self.remove_face(poly_face_id);
            poly.make_mut_ref(self).change_face(old_face_id);
        }
    }

    fn find_same_face_with_other_id(&mut self, face_id: FaceId) -> Option<FaceId> {
        self.faces.iter().find_map(|(old_face, face)| {
            if *face == self.faces[&face_id] && *old_face != face_id {
                Some(*old_face)
            } else {
                None
            }
        })
    }

    pub fn add_polygon_to_mesh<S>(
        &mut self,
        vertices: &[Vector3<S>],
        mesh_id: MeshId,
    ) -> anyhow::Result<()>
    where
        S: Into<Dec> + nalgebra::Scalar + nalgebra::Field + Copy,
    {
        let _ts = SystemTime::now();
        let vertices = vertices
            .iter()
            .map(|s| Vector3::new(s.x.into(), s.y.into(), s.z.into()))
            .collect_vec();

        let poly_mesh = self.save_polygon_new(&vertices)?;

        let poly_id = if let Some(m) = self.meshes.get_mut(&mesh_id) {
            m.add(poly_mesh)
        } else {
            Err(anyhow!("Mesh id {mesh_id:?} not found"))?
        };

        let poly = UnrefPoly { mesh_id, poly_id };

        let _t = SystemTime::now();
        self.unify_faces_ribs(poly_mesh.face_id);

        let _t = SystemTime::now();
        self.find_older_and_replace_face_in_poly(poly);
        //println!("  find-replace: {}ms", _t.elapsed().unwrap().as_millis());

        let _t = SystemTime::now();
        self.create_common_ribs_between_faces(poly, mesh_id);
        //println!( "  common-ribs-between: {}ms", _t.elapsed().unwrap().as_millis());

        let _t = SystemTime::now();
        self.create_common_ribs_for_adjacent_faces(poly.make_ref(self).face_id());
        //println!( "  common-ribs-adjacent: {}ms", _t.elapsed().unwrap().as_millis());

        let _t = SystemTime::now();
        self.split_faces_by_orphan_ribs();
        //println!("  split: {}ms", _t.elapsed().unwrap().as_millis());

        //println!( "Add polygon to mesh time: {}ms", ts.elapsed().unwrap().as_millis());
        Ok(())
    }

    /* TODO: REMOVE
    fn save_polygon(&mut self, polygon: &Polygon, mesh_id: Option<MeshId>) {
        let aabb = Aabb::from_points(&polygon.vertices);
        let mesh_id = mesh_id.unwrap_or(self.default_mesh);
        if polygon
            .get_segments()
            .into_iter()
            .any(|seg| seg.dir().magnitude() < self.input_polygon_min_rib_length)
        {
            for s in polygon.get_segments() {
                let f = s.from;
                let t = s.to;
                let m = s.dir().magnitude();
                println!(
                    "Segment: {} {} {} --> {} {} {} [{}]",
                    f.x.round_dp(10),
                    f.y.round_dp(10),
                    f.z.round_dp(10),
                    t.x.round_dp(10),
                    t.y.round_dp(10),
                    t.z.round_dp(10),
                    m
                );
            }
            panic!("Input rib for new polygon is too short");
        }

        let segments = polygon
            .get_segments()
            .into_iter()
            .map(|s| self.save_segment((s.from, s.to)))
            .collect_vec();

        let plane = polygon.get_plane();

        let poly = Poly::create(segments.clone(), plane.clone(), aabb);
        let face = Face::create(segments, plane, aabb);

        let face_id = self.insert_face(face);
        let poly_id = self.insert_poly(poly).0;
        if poly_id == 31 {
            println!("saved new poly in {mesh_id:?}: {poly_id:?}");
        }

        if let Some(m) = self.meshes.get_mut(&mesh_id) {
            m.0.push(poly_id);
        }

        self.unify_ribs(poly_id);
        self.create_common_ribs(poly_id, mesh_id);
        self.split_polygons_by_orphan_ribs();
    }

    pub fn save_segment_splittin_ribs(&mut self, segment: Segment) -> Vec<Seg> {
        let from = self.insert_point(segment.from);
        let to = self.insert_point(segment.to);

        if let Some(ribs) = self.pt_to_ribs.get(&from).cloned() {
            for r in &ribs {
                let rr = self.get_rib(*r);
                match rr.relate(&to) {
                    PointOnLine::On => {
                        for ps_for_rib in self.rib_to_poly.remove(r).into_iter().flatten() {
                            self.split_rib_in_poly_using_indexed_pt(to, *r, ps_for_rib);
                            Self::remove_item_from_index(&mut self.pt_to_ribs, &to, r);
                            Self::remove_item_from_index(&mut self.pt_to_ribs, &from, r);
                        }
                    }
                    PointOnLine::Outside => {}
                    PointOnLine::Origin => {}
                }

                let rr = self.get_rib(*r);
                for (pt, v) in
                    [rr.from_pt(), rr.to_pt()].map(|pt| (pt, self.vertices.get_point(pt)))
                {
                    match segment.relate(&v) {
                        PointOnLine::On => {
                            if pt != from && pt != to {
                                let mut segs = vec![Seg {
                                    rib_id: *r,
                                    dir: if rr.from_pt() == from {
                                        SegmentDir::Fow
                                    } else {
                                        SegmentDir::Rev
                                    },
                                }];
                                segs.extend(self.save_segment_splittin_ribs(Segment {
                                    from: v,
                                    to: segment.to,
                                }));

                                return segs;
                            }
                        }
                        PointOnLine::Outside => {}
                        PointOnLine::Origin => {}
                    }
                }
            }
        }

        if let Some(ribs) = self.pt_to_ribs.get(&to).cloned() {
            for r in &ribs {
                let rr = self.get_rib(*r);

                match rr.relate(&from) {
                    PointOnLine::On => {
                        for ps_for_rib in self.rib_to_poly.remove(r).into_iter().flatten() {
                            self.split_rib_in_poly_using_indexed_pt(from, *r, ps_for_rib);
                            Self::remove_item_from_index(&mut self.pt_to_ribs, &to, r);
                            Self::remove_item_from_index(&mut self.pt_to_ribs, &from, r);
                        }
                    }
                    PointOnLine::Outside => {}
                    PointOnLine::Origin => {}
                }

                let rr = self.get_rib(*r);
                for (pt, v) in
                    [rr.from_pt(), rr.to_pt()].map(|pt| (pt, self.vertices.get_point(pt)))
                {
                    match segment.relate(&v) {
                        PointOnLine::On => {
                            if pt != from && pt != to {
                                let mut segs = vec![Seg {
                                    rib_id: *r,
                                    dir: if rr.from_pt() == from {
                                        SegmentDir::Fow
                                    } else {
                                        SegmentDir::Rev
                                    },
                                }];
                                segs.extend(self.save_segment_splittin_ribs(Segment {
                                    to: v,
                                    from: segment.from,
                                }));

                                return segs;
                            }
                        }
                        PointOnLine::Outside => {}
                        PointOnLine::Origin => {}
                    }
                }
            }
        }

        vec![self.save_segment((segment.from, segment.to))]
    }
    */

    pub fn save_segment(
        &mut self,
        (from_v, to_v): (Vector3<Dec>, Vector3<Dec>),
    ) -> anyhow::Result<Seg> {
        let from = self.insert_point(from_v);
        let to = self.insert_point(to_v);

        if from == to {
            return Err(anyhow!("Segment too short: {from_v:?} -> {to_v:?}"));
        }
        let (rib, dir) = Rib::build(from, to);

        let (rib_id, _) = self.insert_rib(rib);
        Self::save_index(&mut self.pt_to_ribs, from, rib_id);
        Self::save_index(&mut self.pt_to_ribs, to, rib_id);

        Ok(Seg { rib_id, dir })
    }

    pub fn save_segment_unchecked(&mut self, (from_v, to_v): (Vector3<Dec>, Vector3<Dec>)) -> Seg {
        let from = self.insert_point(from_v);
        let to = self.insert_point(to_v);

        if from == to {
            panic!("Segment too short: {from_v:?} -> {to_v:?}");
        }
        let (rib, dir) = Rib::build(from, to);

        let (rib_id, _) = self.insert_rib(rib);
        Self::save_index(&mut self.pt_to_ribs, from, rib_id);
        Self::save_index(&mut self.pt_to_ribs, to, rib_id);

        Seg { rib_id, dir }
    }

    /*
    pub fn load_polygon_ref(&self, other_id: PolyId) -> PolyRef<'_> {
        PolyRef {
            poly_id: other_id,
            index: self,
        }
    }
    */

    pub(crate) fn load_face_ref(&self, face_id: FaceId) -> FaceRef<'_> {
        FaceRef {
            face_id,
            index: self,
        }
    }

    pub(super) fn insert_rib(&mut self, rib: Rib) -> (RibId, bool) {
        if let Some(rib_id) = self.ribs.iter().find(|(_, &s)| s == rib).map(|(id, _)| id) {
            (*rib_id, false)
        } else {
            let rib_id = self.get_next_rib_id();
            self.ribs.insert(rib_id, rib);
            (rib_id, true)
        }
    }

    fn insert_face(&mut self, face: Face) -> (FaceId, bool) {
        if let Some(face_id) = self
            .faces
            .iter()
            .find(|&(_id, s)| *s == face)
            .map(|(id, _)| id)
        {
            (*face_id, false)
        } else {
            let face_id = self.get_next_face_id();
            let rtree_item = FaceRtreeRecord(face_id, *face.aabb());
            self.face_index.insert(rtree_item);
            face.update_rib_index(face_id, &mut self.rib_to_face);
            self.faces.insert(face_id, face);

            (face_id, true)
        }
    }

    fn insert_point(&mut self, pt: Vector3<Dec>) -> PtId {
        self.vertices.get_or_insert_point(pt, self.points_precision)
    }

    pub(crate) fn save_index<Ix, Item>(index: &mut BTreeMap<Ix, Vec<Item>>, ix: Ix, item: Item)
    where
        Ix: Hash + Ord,
        Item: PartialEq,
    {
        if let Some(items) = index.get_mut(&ix) {
            if !items.contains(&item) {
                items.push(item);
            }
        } else {
            index.insert(ix, vec![item]);
        }
    }

    pub(crate) fn remove_item_from_index<Ix, Item>(
        index: &mut BTreeMap<Ix, Vec<Item>>,
        ix: &Ix,
        item: &Item,
    ) where
        Ix: Hash + Ord,
        Item: PartialEq,
    {
        if let Some(items) = index.get_mut(ix) {
            items.retain(|i| i != item)
        }
        if index.get(ix).is_some_and(|v| v.is_empty()) {
            index.remove(ix);
        }
    }

    /// Remove polygon from all available related structures
    pub fn remove_face(&mut self, face_id: FaceId) {
        if let Some(face) = self.faces.remove(&face_id) {
            face.delete_me_from_rib_index(face_id, &mut self.rib_to_face);

            self.face_index
                .remove(&FaceRtreeRecord(face_id, *face.aabb()));
            self.deleted_faces.insert(face_id, face);
        }
    }

    /// Remove polygon from all available related structures
    pub(crate) fn remove_polygon(&mut self, poly_ix: PolyId, in_mesh: MeshId) {
        if let Some(mesh) = self.meshes.get_mut(&in_mesh) {
            if let Some(poly) = mesh.polies.remove(&poly_ix) {
                let left_meshes = self.get_face_meshes(poly.face_id);

                if left_meshes.is_empty() {
                    if let Some(face) = self.faces.remove(&poly.face_id) {
                        face.delete_me_from_rib_index(poly.face_id, &mut self.rib_to_face);

                        self.face_index
                            .remove(&FaceRtreeRecord(poly.face_id, *face.aabb()));
                        self.deleted_faces.insert(poly.face_id, face);
                    }
                }
            }
        }
    }

    /*
    /// Remove polygon from all available related structures
    ///
    pub fn remove_polygon_old(&mut self, poly_id: PolyId) {
        if let Some(poly) = self.polygons.remove(&poly_id) {
            poly.delete_me_from_rib_index(poly_id, &mut self.rib_to_poly);

            self.polygon_index
                .remove(&PolyRtreeRecord(poly_id, *poly.aabb()));
            self.deleted_polygons.insert(poly_id, poly);
        }
    }
    */

    /*
    pub(crate) fn remove_mesh(&mut self, mesh_id: MeshId) {
        if let Some(mesh) = self.meshes.remove(&mesh_id) {
            for p in mesh.0 {
                self.remove_polygon(p, mesh_id);
            }
        }
    }

    /// Remove polygon from all available related structures
    pub fn remove_polygon(&mut self, poly: Poly, in_mesh: MeshId) {
        if let Some(mesh) = self.meshes.get_mut(&in_mesh) {
            if let Some(pos) = mesh.0.iter().position(|&item| item == poly) {
                let poly = mesh.0.remove(pos);

                let left_meshes = self.get_face_meshes(poly.face_id);

                if left_meshes.is_empty() {
                    if let Some(face) = self.faces.remove(&poly.face_id) {
                        face.delete_me_from_rib_index(poly.face_id, &mut self.rib_to_face);

                        self.face_index
                            .remove(&FaceRtreeRecord(poly.face_id, *face.aabb()));
                        self.deleted_faces.insert(poly.face_id, face);
                    }
                }
            }
        }
    }
    */

    pub fn meshes(&self) -> Vec<MeshRef> {
        self.meshes
            .keys()
            .map(|&mesh_id| MeshRef {
                geo_index: self,
                mesh_id,
            })
            .collect()
    }

    pub(super) fn get_face_points(&self, face_id: FaceId) -> HashSet<PtId> {
        self.load_face_ref(face_id)
            .segments(SegmentDir::Fow)
            .map(|seg| seg.rib_id)
            .filter_map(|rib_id| self.ribs.get(&rib_id))
            .flat_map(|rib| [rib.0, rib.1])
            .collect()
    }

    fn load_mesh_ref(&self, mesh_id: MeshId) -> MeshRef {
        MeshRef {
            geo_index: self,
            mesh_id,
        }
    }

    fn replace_faces_in_meshes(&mut self, prev_face: FaceId, new_faces: &[FaceId]) {
        while let Some(poly) = self
            .meshes
            .keys()
            .map(|k| self.load_mesh_ref(*k))
            .flat_map(|m| m.into_polygons())
            .find(|pr| pr.make_ref(self).face_id() == prev_face)
        {
            let current_plane = poly.make_ref(self).plane();
            let replacement = new_faces
                .iter()
                .map(|face_id| {
                    let face_ref = self.load_face_ref(*face_id);
                    let face_plane = face_ref.plane();
                    if face_plane.normal().dot(&current_plane.normal()) > Dec::zero() {
                        Poly::fow(*face_id)
                    } else {
                        Poly::rev(*face_id)
                    }
                })
                .collect_vec();

            let UnrefPoly {
                poly_id, mesh_id, ..
            } = poly;
            let mut mut_poly = PolyRefMut {
                poly_id,
                mesh_id,
                index: self,
            };
            mut_poly.replace(replacement);
        }
    }

    pub(crate) fn get_face_meshes(&self, face_id: FaceId) -> Vec<MeshId> {
        self.meshes
            .iter()
            .filter(|(_, mesh)| mesh.polies.values().any(|pn| pn.face_id == face_id))
            .map(|item| *item.0)
            .collect()
    }

    pub fn get_mesh(&self, mesh_id: MeshId) -> MeshRef<'_> {
        MeshRef {
            mesh_id,
            geo_index: self,
        }
    }

    fn remove_rib(&mut self, rib_id: RibId) {
        if self.rib_to_face.get(&rib_id).is_some_and(|v| !v.is_empty()) {
            panic!("rib index to poly is not empty");
        }
        if let Some(rib) = self.ribs.remove(&rib_id) {
            Self::remove_item_from_index(&mut self.pt_to_ribs, &rib.0, &rib_id);
            Self::remove_item_from_index(&mut self.pt_to_ribs, &rib.1, &rib_id);
        }

        self.rib_to_face.remove(&rib_id);
    }

    fn collect_intersection_points_between_two_faces(
        &self,
        src_id: FaceId,
        tool_id: FaceId,
    ) -> Vec<(Either<Vector3<Dec>, PtId>, RibId)> {
        let tool = self.load_face_ref(tool_id);
        let plane = tool.plane();
        let mut vertices = Vec::new();
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;

        for seg in self.load_face_ref(src_id).segments(SegmentDir::Fow) {
            if seg.to_pt() == seg.from_pt() {
                panic!("Seg to equals to seg_from");
            }
            if let Some(t) = plane.get_intersection_param2(seg.from(), seg.to()) {
                let maybe_zero = (Dec::zero() - t).abs() < vertex_pulling_sq;
                let maybe_one = (Dec::one() - t).abs() < vertex_pulling_sq;
                if (t >= Dec::zero() && t <= Dec::one()) || maybe_zero || maybe_one {
                    vertices.push((seg.from().lerp(&seg.to(), t), seg.rib_id));
                }
            }
        }

        vertices
            .clone()
            .into_iter()
            .map(|(v, rib)| {
                if let Some(pt) = self.get_face_points(src_id).into_iter().find(|pt| {
                    let poly_vertex = self.vertices.get_point(*pt);
                    let distance = (poly_vertex - v).magnitude_squared();
                    distance < vertex_pulling_sq
                }) {
                    (Either::Right(pt), rib)
                } else {
                    (Either::Left(v), rib)
                }
            })
            .collect()
    }

    fn create_common_ribs_between_faces(&mut self, tool: UnrefPoly, mesh_id: MeshId) {
        let tool_face_id = tool.make_ref(self).face_id();
        let tool_aabb = *self.load_face_ref(tool_face_id).aabb();

        let faces = self
            .face_index
            .locate_in_envelope_intersecting(&tool_aabb.into())
            .map(|o| o.0)
            .filter(|&p| p != tool_face_id)
            .filter(|face_id| {
                self.meshes[&mesh_id]
                    .polies
                    .values()
                    .all(|poly| poly.face_id != *face_id)
            })
            .collect_vec();

        if faces.is_empty() {
            return;
        }

        let tool_plane = self.load_face_ref(tool_face_id).plane().to_owned();
        for src_id in faces.iter() {
            let src_plane = self.load_face_ref(*src_id).plane().to_owned();
            let common_line = match tool_plane.relate(&src_plane) {
                PlanarRelation::Intersect(line) => line,
                _ => {
                    continue;
                }
            };

            let vertices_src =
                self.collect_intersection_points_between_two_faces(*src_id, tool_face_id);
            let vertices_tool =
                self.collect_intersection_points_between_two_faces(tool_face_id, *src_id);

            let mut cut_ribs_index = BTreeMap::new();
            let pts_src = vertices_src
                .into_iter()
                .map(|(v, rib_id)| match v {
                    Either::Left(v) => {
                        let pt = self.vertices.get_or_insert_point(v, self.points_precision);
                        (pt, rib_id)
                    }
                    Either::Right(pt) => (pt, rib_id),
                })
                .map(|(pt, rib_id)| {
                    Self::save_index(&mut cut_ribs_index, pt, rib_id);
                    pt
                })
                .collect_vec();

            let pts_tool = vertices_tool
                .into_iter()
                .map(|(v, rib_id)| match v {
                    Either::Left(v) => {
                        let pt = self.vertices.get_or_insert_point(v, self.points_precision);
                        (pt, rib_id)
                    }
                    Either::Right(pt) => (pt, rib_id),
                })
                //.filter(|(pt, _)| !pts_tool.contains(pt))
                .map(|(pt, rib_id)| {
                    Self::save_index(&mut cut_ribs_index, pt, rib_id);
                    pt
                })
                .collect_vec();

            if pts_src.is_empty() || pts_tool.is_empty() {
                continue;
            }
            let pts_src = pts_src
                .into_iter()
                .sorted_by_key(|pt| {
                    (self.vertices.get_point(*pt) - common_line.origin).dot(&common_line.dir)
                })
                .dedup()
                .collect_vec();

            let pts_tool = pts_tool
                .into_iter()
                .sorted_by_key(|pt| {
                    (self.vertices.get_point(*pt) - common_line.origin).dot(&common_line.dir)
                })
                .dedup()
                .collect_vec();

            let mut segs_src = Vec::new();
            let mut segs_tool = Vec::new();

            let mut pts_src = pts_src.into_iter().peekable();
            let mut pts_tool = pts_tool.into_iter().peekable();

            while let Some(from) = pts_src.next() {
                if let Some(to) = pts_src.peek() {
                    segs_src.push([from, *to]);
                }
            }

            while let Some(from) = pts_tool.next() {
                if let Some(to) = pts_tool.peek() {
                    segs_tool.push([from, *to]);
                }
            }

            let between = |a: usize, b: usize, c: usize| (c > a && c < b) || (c > b && c < a);
            let intersection = |a: [PtId; 2], b: [PtId; 2]| {
                let common = a
                    .into_iter()
                    .chain(b)
                    .sorted_by_key(|pt| {
                        (self.vertices.get_point(*pt) - common_line.origin).dot(&common_line.dir)
                    })
                    .dedup()
                    .collect_vec();
                let a0 = common.iter().position(|&i| i == a[0]).expect("ok");
                let a1 = common.iter().position(|&i| i == a[1]).expect("ok");
                let b0 = common.iter().position(|&i| i == b[0]).expect("ok");
                let b1 = common.iter().position(|&i| i == b[1]).expect("ok");
                let is_overlap = between(a0, a1, b0)
                    || between(a0, a1, b1)
                    || between(b0, b1, a0)
                    || between(b0, b1, a1);

                if common.len() == 2 {
                    Some([common[0], common[1]])
                } else if common.len() == 3 && !is_overlap {
                    None
                } else if common.len() == 3 && is_overlap {
                    let mut zeros = 0;
                    for i in [a0, a1, b0, b1] {
                        if i == 0 {
                            zeros += 1;
                        }
                    }
                    if zeros == 2 {
                        Some([common[0], common[1]])
                    } else {
                        Some([common[1], common[2]])
                    }
                } else if is_overlap {
                    Some([common[1], common[2]])
                } else {
                    None
                }
            };

            let mut new_ribs = Vec::new();

            for s in segs_src {
                for t in &segs_tool {
                    if let Some([a, b]) = intersection(s, *t) {
                        new_ribs.push(Rib::build(a, b).0);
                    }
                }
            }

            let mut splitted_new_ribs = Vec::new();

            for rib in new_ribs.clone() {
                let (new_rib_id, is_created) = self.insert_rib(rib);

                if is_created {
                    // When create new rib - lets_check, if it splitted by some other rib, or it
                    // splits some other rib
                    // We assume, that existing ribs already split.
                    let mut this_rib_splitted = false;
                    for face_id in [tool_face_id, *src_id] {
                        while let Some(splits) =
                            self.find_intersecting_ribs_on_same_line_in_face(new_rib_id, face_id)
                        {
                            for (rib_id, pts) in splits {
                                let rib_faces = self
                                    .rib_to_face
                                    .remove(&rib_id)
                                    .into_iter()
                                    .flatten()
                                    .collect_vec();
                                let new_splitted_ribs = if rib_faces.is_empty() {
                                    self.split_floating_rib_using_indexed_pts(&pts, rib_id)
                                } else {
                                    let new_ribs = rib_faces
                                        .into_iter()
                                        .flat_map(|face_id| {
                                            self.split_rib_in_face_using_indexed_pts(
                                                &pts, rib_id, face_id,
                                            )
                                        })
                                        .collect::<HashSet<_>>();
                                    new_ribs.into_iter().collect_vec()
                                };
                                new_splitted_ribs.iter().for_each(|new_rib_id| {
                                    Self::save_index(&mut self.split_ribs, rib_id, *new_rib_id)
                                });

                                if rib_id == new_rib_id {
                                    this_rib_splitted = true;
                                    splitted_new_ribs.extend(new_splitted_ribs);
                                }

                                self.remove_rib(rib_id);
                            }
                        }
                    }
                    if !this_rib_splitted {
                        splitted_new_ribs.push(new_rib_id);
                    }

                    for pt in [rib.0, rib.1] {
                        Self::save_index(&mut self.pt_to_ribs, pt, new_rib_id);
                        for poly_rib_id in cut_ribs_index.remove(&pt).into_iter().flatten() {
                            if self
                                .ribs
                                .get(&poly_rib_id)
                                .is_some_and(|poly_rib| poly_rib.0 != pt && poly_rib.1 != pt)
                            {
                                for poly_id in
                                    self.rib_to_face.remove(&poly_rib_id).into_iter().flatten()
                                {
                                    self.split_rib_in_face_using_indexed_pts(
                                        &[pt],
                                        poly_rib_id,
                                        poly_id,
                                    );
                                }
                            }
                        }
                    }
                } else {
                    splitted_new_ribs.push(new_rib_id);
                }
            }

            if !new_ribs.is_empty() && splitted_new_ribs.is_empty() {
                panic!("splitted_new_ribs : {splitted_new_ribs:?} / {new_ribs:?}");
            }

            for new_rib_id in splitted_new_ribs.clone() {
                if !self
                    .rib_to_face
                    .get(&new_rib_id)
                    .is_some_and(|faces| faces.contains(&tool_face_id))
                    && self.rib_inside_face(new_rib_id, tool_face_id)
                {
                    if new_rib_id == 3846 {
                        println!("Push 3846 for {tool_face_id:?}");
                    }
                    Self::save_index(&mut self.partially_split_faces, tool_face_id, new_rib_id);
                }

                if !self
                    .rib_to_face
                    .get(&new_rib_id)
                    .is_some_and(|ps| ps.contains(src_id))
                    && self.rib_inside_face(new_rib_id, *src_id)
                {
                    if new_rib_id == 3846 {
                        println!("Push 3846 for {src_id:?}");
                    }
                    Self::save_index(&mut self.partially_split_faces, *src_id, new_rib_id);
                }
            }
        }
    }

    fn find_intersecting_ribs_on_same_line_in_face(
        &self,
        rib_id: RibId,
        face_id: FaceId,
    ) -> Option<BTreeMap<RibId, Vec<PtId>>> {
        if !self.ribs.contains_key(&rib_id) {
            return None;
        }
        let rib1 = RibRef {
            rib_id,
            index: self,
        };
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;

        let line = crate::linear::line::Line {
            origin: rib1.from(),
            dir: rib1.dir().normalize(),
        };
        if face_id.0 == 2496 && rib_id == 3847 {
            println!("LOOK FOR splits of 3847");
        }

        self.load_face_ref(face_id)
            .segments(SegmentDir::Fow)
            .filter(|seg| {
                line.distance_to_pt_squared(seg.from()).abs() < vertex_pulling_sq
                    && line.distance_to_pt_squared(seg.to()).abs() < vertex_pulling_sq
                    && line.dir.dot(&seg.dir().normalize()).abs()
                        > Dec::from(dec!(0.9999984769132877))
                // Less than 1 degree
            })
            .map(|s| RibRef {
                rib_id: s.rib_id,
                index: self,
            })
            .map(|rib2| {
                (
                    (vec![rib1.from_pt(), rib1.to_pt(), rib2.from_pt(), rib2.to_pt()])
                        .into_iter()
                        .sorted_by_key(|pt| {
                            let v = self.vertices.get_point(*pt);
                            (v - line.origin).dot(&line.dir)
                        })
                        .dedup()
                        .collect_vec(),
                    rib2,
                )
            })
            .find_map(|(pts, rib2)| {
                let mut split_ribs = BTreeMap::new();
                if pts.len() > 2 {
                    let rib_start_ix = pts.iter().position(|pt| *pt == rib1.from_pt()).unwrap();
                    let rib_end_ix = pts.iter().position(|pt| *pt == rib1.to_pt()).unwrap();
                    let seg_start_ix = pts.iter().position(|pt| *pt == rib2.from_pt()).unwrap();
                    let seg_end_ix = pts.iter().position(|pt| *pt == rib2.to_pt()).unwrap();

                    let between =
                        |a: usize, b: usize, c: usize| (c > a && c < b) || (c > b && c < a);

                    if between(rib_start_ix, rib_end_ix, seg_start_ix) {
                        Self::save_index(&mut split_ribs, rib1.rib_id, pts[seg_start_ix]);
                    } else if between(rib_start_ix, rib_end_ix, seg_end_ix) {
                        Self::save_index(&mut split_ribs, rib1.rib_id, pts[seg_end_ix]);
                    } else if between(seg_start_ix, seg_end_ix, rib_start_ix) {
                        Self::save_index(&mut split_ribs, rib2.rib_id, pts[rib_start_ix]);
                    } else if between(seg_start_ix, seg_end_ix, rib_end_ix) {
                        Self::save_index(&mut split_ribs, rib2.rib_id, pts[rib_end_ix]);
                    };
                }
                if split_ribs.is_empty() {
                    None
                } else {
                    //let d = rib1.dir().normalize().dot(&rib2.dir().normalize());

                    //println!("LOOK {:?}/{:?}  {d}", rib1.rib_id, rib2.rib_id);

                    Some(split_ribs)
                }
            })
    }

    /// make common lines have common ribs
    /// This function processes only those ribs, which are on one line
    fn unify_faces_ribs(&mut self, tool_face_id: FaceId) {
        let tool_aabb = *self.load_face_ref(tool_face_id).aabb();
        let faces = self
            .face_index
            .locate_in_envelope_intersecting(&tool_aabb.into())
            .map(|o| o.0)
            .filter(|&p| p != tool_face_id)
            .collect_vec();

        for face_id in faces.iter() {
            while let Some(splits) = self
                .load_face_ref(*face_id)
                .segments(SegmentDir::Fow)
                .map(|s| s.rib_id)
                .collect_vec()
                .iter()
                .find_map(|&rib| {
                    self.find_intersecting_ribs_on_same_line_in_face(rib, tool_face_id)
                })
            {
                let mut splitted = Vec::new();
                for (rib_id, pts) in splits {
                    for face_id in self.rib_to_face.remove(&rib_id).into_iter().flatten() {
                        if rib_id == 3847 && face_id.0 == 2495 {
                            println!("split rib 3847 because {tool_face_id:?}")
                        }
                        let new_ribs =
                            self.split_rib_in_face_using_indexed_pts(&pts, rib_id, face_id);

                        new_ribs
                            .iter()
                            .for_each(|r| Self::save_index(&mut self.split_ribs, rib_id, *r));

                        splitted.extend(new_ribs);
                    }

                    self.remove_rib(rib_id);
                }
            }
        }
    }

    pub fn move_all_polygons(&mut self, from_mesh: MeshId, to_mesh: MeshId) {
        for (_, poly) in self
            .meshes
            .get_mut(&from_mesh)
            .into_iter()
            .flat_map(|map| map.polies.drain())
            .collect_vec()
        {
            if let Some(mesh) = self.meshes.get_mut(&to_mesh) {
                mesh.add(poly);
            }
        }
    }

    pub fn is_vec_dir_between_two_other_dirs(
        &self,
        plane_normal: Vector3<Dec>,
        one: Vector3<Dec>,
        two: Vector3<Dec>,
        test: Vector3<Dec>,
    ) -> bool {
        let x = one;
        let y = plane_normal.cross(&one).normalize();

        let v = two;

        let v_x = x.dot(&v);
        let v_y = y.dot(&v);

        let a_v = v_y.atan2(&v_x);
        let a_v = if a_v.is_negative() {
            Dec::two_pi() + a_v
        } else {
            a_v
        };

        let test_x = x.dot(&test);
        let test_y = y.dot(&test);

        let test_v = test_y.atan2(&test_x);

        let test_v = if test_v.is_negative() {
            Dec::two_pi() + test_v
        } else {
            test_v
        };

        test_v < a_v
    }

    pub fn is_poly_between_fronts(
        &self,
        common_rib: RibId,
        one: UnrefPoly,
        two: UnrefPoly,
        to_check: UnrefPoly,
    ) -> bool {
        let x = self.detect_poly_dir(common_rib, one);
        let y = one.make_ref(self).normal();

        let v = self.detect_poly_dir(common_rib, two);
        let test = self.detect_poly_dir(common_rib, to_check);

        let v_x = x.dot(&v);
        let v_y = y.dot(&v);

        let a_v = v_y.atan2(&v_x);
        let a_v = if a_v.is_negative() {
            Dec::two_pi() + a_v
        } else {
            a_v
        };

        let test_x = x.dot(&test);
        let test_y = y.dot(&test);

        let test_v = test_y.atan2(&test_x);

        let test_v = if test_v.is_negative() {
            Dec::two_pi() + test_v
        } else {
            test_v
        };

        test_v < a_v
    }

    pub fn detect_poly_dir(&self, rib_id: RibId, poly: UnrefPoly) -> Vector3<Dec> {
        let rib_dir = rib_id.make_ref(self).dir();
        let plane_dir = poly.make_ref(self).normal();
        let in_poly_dir = rib_dir.cross(&plane_dir).normalize();
        let line_straight = Line {
            origin: rib_id.make_ref(self).middle(),
            dir: in_poly_dir,
        };

        let segs = poly
            .make_ref(self)
            .segments()
            .filter(|s| s.rib_id != rib_id)
            .collect_vec();
        let vertex_pulling = rib_id
            .make_ref(self)
            .magnitude()
            .div(2)
            .min(Dec::one() / Dec::from(1000));

        let intersections = self.collect_line_segs_intersections(
            line_straight,
            segs.iter(),
            vertex_pulling * vertex_pulling,
            false,
        );
        if intersections % 2 == 1 {
            in_poly_dir
        } else {
            -in_poly_dir
        }
    }

    pub fn select_polygons(
        &self,
        of_mesh: MeshId,
        by_mesh: MeshId,
        filter: PolygonFilter,
    ) -> Vec<UnrefPoly> {
        let _ts = SystemTime::now();
        let _t = SystemTime::now();
        let mut face_mesh_index = BTreeMap::new();
        for (mesh_id, mesh) in &self.meshes {
            if [of_mesh, by_mesh].contains(mesh_id) {
                for p in mesh.polies.keys() {
                    let face_id = UnrefPoly {
                        poly_id: *p,
                        mesh_id: *mesh_id,
                    }
                    .make_ref(self)
                    .face_id();
                    Self::save_index(&mut face_mesh_index, face_id, *mesh_id);
                }
            }
        }
        println!(
            "  .. collect facemeshindex {}ms",
            _t.elapsed().unwrap().as_millis()
        );
        let _t = SystemTime::now();

        let ribs_with_faces = self
            .rib_to_face
            .iter()
            .filter(|(_rib_id, faces)| {
                let meshes = faces
                    .iter()
                    .filter_map(|face_id| face_mesh_index.get(face_id))
                    .flatten()
                    .collect::<HashSet<_>>();

                meshes.contains(&of_mesh) && meshes.contains(&by_mesh)
            })
            .collect_vec();
        println!(
            "  .. collect ribs with faces {}ms",
            _t.elapsed().unwrap().as_millis()
        );

        let mut planes: Vec<Plane> = Vec::new();
        let mut poly_plane = BTreeMap::new();

        let mut visited = BTreeMap::new();

        let _t = SystemTime::now();
        for (_, faces) in &ribs_with_faces {
            for &face_id in faces.iter() {
                let meshes = self
                    .get_face_meshes(face_id)
                    .into_iter()
                    .filter(|m| [of_mesh, by_mesh].contains(m))
                    .collect_vec();
                let this_plane = self.faces[&face_id].plane();
                if let Some(plane) = planes.iter().position(|p| *p == *this_plane) {
                    poly_plane.insert(face_id, plane);
                } else {
                    poly_plane.insert(face_id, planes.len());
                    planes.push(this_plane.to_owned());
                }
                if meshes.len() > 1 {
                    let poly = self.meshes[&of_mesh]
                        .polies
                        .iter()
                        .find(|p| p.1.face_id == face_id)
                        .map(|p| p.0)
                        .expect("mesh has it");
                    visited.insert(*poly, PolygonFilter::Shared);
                }
            }
        }
        println!("  .. fill shared {}ms", _t.elapsed().unwrap().as_millis());

        if matches!(filter, PolygonFilter::Shared) {
            // early return for shareds
            let collect_vec = visited
                .into_iter()
                .filter(|(_, r)| *r == filter)
                .map(|(poly_id, _)| UnrefPoly {
                    mesh_id: of_mesh,
                    poly_id,
                })
                .collect_vec();
            return collect_vec;
        }
        let _t = SystemTime::now();

        let mut ribs = HashSet::with_capacity(ribs_with_faces.len());
        for (rib_id, faces) in &ribs_with_faces {
            ribs.insert(**rib_id);
            let polygons_to_sort_of = faces
                .iter()
                .filter(|f| self.get_face_meshes(**f).contains(&of_mesh))
                .filter_map(|f| {
                    self.meshes[&of_mesh]
                        .polies
                        .iter()
                        .find(|p| p.1.face_id == *f)
                        .map(|p| p.0)
                })
                .filter(|f| !visited.contains_key(f))
                .map(|&poly_id| UnrefPoly {
                    mesh_id: of_mesh,
                    poly_id,
                })
                .collect_vec();

            let polygons_to_sort_relative_to = faces
                .iter()
                .filter(|f| self.get_face_meshes(**f).contains(&by_mesh))
                .filter_map(|f| {
                    self.meshes[&by_mesh]
                        .polies
                        .iter()
                        .find(|p| p.1.face_id == *f)
                        .map(|p| p.0)
                })
                .map(|&poly_id| UnrefPoly {
                    mesh_id: by_mesh,
                    poly_id,
                })
                .collect_vec();

            for poly_to_sort in polygons_to_sort_of {
                if let Ok([poly_one, poly_two]) = TryInto::<[UnrefPoly; 2]>::try_into(
                    polygons_to_sort_relative_to
                        .iter()
                        .filter(|p| {
                            poly_to_sort.make_ref(self).face_id() != p.make_ref(self).face_id()
                        })
                        .copied()
                        .collect_vec(),
                ) {
                    if self.is_poly_between_fronts(**rib_id, poly_one, poly_two, poly_to_sort) {
                        visited.insert(poly_to_sort.poly_id, PolygonFilter::Front);
                    } else {
                        visited.insert(poly_to_sort.poly_id, PolygonFilter::Back);
                    }
                }
            }
        }
        println!(
            "  .. detect edge polies {}ms",
            _t.elapsed().unwrap().as_millis()
        );
        let _t = SystemTime::now();
        let visited = self.spread_visited_around_2(&ribs, of_mesh, visited);
        println!(
            "  .. spread outer polies {}ms",
            _t.elapsed().unwrap().as_millis()
        );
        let result = visited
            .into_iter()
            .filter(|(_, r)| *r == filter)
            .map(|(poly_id, _)| UnrefPoly {
                mesh_id: of_mesh,
                poly_id,
            })
            .collect_vec();
        println!("select: {}ms", _ts.elapsed().unwrap().as_millis());
        result
    }

    fn spread_visited_around_2(
        &self,
        common_ribs: &HashSet<RibId>,
        of_mesh: MeshId,
        mut visited: BTreeMap<PolyId, PolygonFilter>,
    ) -> BTreeMap<PolyId, PolygonFilter> {
        let mesh_poly_map = of_mesh.make_ref(self).face_poly_map();
        for filter in [PolygonFilter::Front, PolygonFilter::Back] {
            'inner: loop {
                let adjacent = visited
                    .iter()
                    .filter(|&(_, mark)| *mark == filter)
                    .map(|(poly_id, _)| *poly_id)
                    .flat_map(|poly_id| {
                        UnrefPoly {
                            poly_id,
                            mesh_id: of_mesh,
                        }
                        .make_ref(self)
                        .segments()
                        .map(|s| s.rib_id)
                        .filter(|rib_id| !common_ribs.contains(rib_id))
                        .flat_map(|rib_id| &self.rib_to_face[&rib_id])
                        .filter_map(|face_id| mesh_poly_map.get(face_id))
                        .filter(|adjacent| !visited.contains_key(&adjacent.poly_id))
                    })
                    .collect::<HashSet<_>>();
                if adjacent.is_empty() {
                    break 'inner;
                }

                for p in adjacent {
                    visited.insert(p.poly_id, filter);
                }
            }
        }

        visited
    }

    fn calculate_aabb_from_segments<'a>(&'a self, one: impl Iterator<Item = SegRef<'a>>) -> Aabb {
        Aabb::from_points(&one.map(|seg| seg.from()).collect_vec())
    }

    fn is_chain_circular(&self, chain: &[Seg]) -> bool {
        if chain.len() < 3 {
            false
        } else {
            chain.first().unwrap().from(&self.ribs) == chain.last().unwrap().to(&self.ribs)
        }
    }

    fn collect_chain_splitting_face(
        &self,
        face_id: FaceId,
        chain: Vec<Seg>,
    ) -> Option<(Vec<Seg>, Vec<Seg>)> {
        let face_ref = self.load_face_ref(face_id);
        if face_id.0 == 2494 {
            println!("~~~~~~~~~~~~~~~~~~~~~~~~");
            for ch in &chain {
                let sr = ch.make_ref(self);
                println!(
                    "possible chain: {:?}: {:?} -> {:?}",
                    sr.rib_id,
                    sr.from_pt(),
                    sr.to_pt()
                )
            }
            println!("--");
            for sr in face_id.make_ref(self).segments(SegmentDir::Fow) {
                println!(
                    "poly: {:?}: {:?} -> {:?}",
                    sr.rib_id,
                    sr.from_pt(),
                    sr.to_pt()
                )
            }
        }

        let chain_without_face_ribs = chain
            .into_iter()
            .filter(|seg| {
                face_ref
                    .segments(SegmentDir::Fow)
                    .all(|ps| ps.rib_id != seg.rib_id)
            })
            .collect_vec();

        if let Some(seg_id) = chain_without_face_ribs
            .iter()
            .map(|seg| self.load_segref(seg))
            .position(|chain_seg| {
                face_ref
                    .segments(SegmentDir::Fow)
                    .any(|poly_seg| poly_seg.to_pt() == chain_seg.from_pt())
            })
        {
            let rotated_chain = {
                let mut ch = chain_without_face_ribs;
                ch.rotate_left(seg_id);
                ch
            };

            let first_item_of = rotated_chain
                .first()
                .expect("non-empty chain")
                .from(&self.ribs);

            if let Some(seg_id) = rotated_chain
                .iter()
                .map(|seg| self.load_segref(seg))
                .position(|chain_seg| {
                    face_ref.segments(SegmentDir::Fow).any(|poly_seg| {
                        poly_seg.from_pt() == chain_seg.to_pt()
                            && poly_seg.from_pt() != first_item_of
                    })
                })
            {
                let (chain_part, chain_rest) = rotated_chain.split_at(seg_id + 1);

                return Some((chain_part.to_vec(), chain_rest.to_vec()));
            }
        } else if self.is_chain_circular(&chain_without_face_ribs) {
            return Some((chain_without_face_ribs, Vec::new()));
        }

        None
    }

    pub fn get_ribs_with_root_parent(&self, rib_id: RibId) -> Vec<RibId> {
        let mut collected = Vec::new();

        let mut to_check = vec![rib_id];
        while let Some(p) = to_check.pop() {
            if self.ribs.contains_key(&p) {
                collected.push(p);
            }
            if let Some(ps) = self.split_ribs.get(&p) {
                to_check.extend(ps);
            }
        }

        collected
    }

    pub fn get_face_with_root_parent(&self, face_id: FaceId) -> Vec<FaceId> {
        let mut collected = Vec::new();

        let mut to_check = vec![face_id];
        while let Some(p) = to_check.pop() {
            if self.faces.contains_key(&p) {
                collected.push(p);
            }
            if let Some(ps) = self.face_splits.get(&p) {
                to_check.extend(ps);
            }
        }

        collected
    }

    pub fn find_splitted_face_parent(&self, face_id: FaceId) -> Option<FaceId> {
        self.face_splits
            .iter()
            .find(|(_, ps)| ps.contains(&face_id))
            .map(|p| *p.0)
    }

    fn debug_svg_face(
        &mut self,
        pre: &str,
        face_id: FaceId,
        basis: &PolygonBasis,
        additional_points: &[PtId],
    ) {
        const COLORS: &[&str] = &["magenta", "#fd9", "#f9d", "#df9", "#9fd", "#d9f", "#9df"];
        let color = COLORS[self.current_color % COLORS.len()];

        let filename = self.debug_path.join(format!("{pre}face-{face_id:?}.svg"));
        println!("~~~DEBUG {filename:?}  {face_id:?}");
        std::fs::write(
            filename,
            face_id
                .make_ref(self)
                .svg_debug_fill(basis, color, additional_points),
        )
        .unwrap();

        self.current_color += 1;
    }

    pub fn scad(&self) -> String {
        let pts = self
            .vertices
            .get_vertex_array()
            .into_iter()
            .map(|[x, y, z]| format!("[{x}, {y}, {z}]"))
            .join(", \n");
        let points = format!("[{pts}];");
        let hedras = self
            .meshes()
            .into_iter()
            .flat_map(|m| m.into_polygons())
            .map(|poly_ref| poly_ref.make_ref(self).serialized_polygon_pt())
            .map(|pts| format!("[{pts}]"))
            .join(", \n");

        format!("points={points};\n polyhedron(points, [{hedras}]);")
    }

    fn is_chain_inside_face(&self, chain: &[Seg], face_id: FaceId) -> bool {
        chain
            .iter()
            .all(|s| self.rib_inside_face(s.rib_id, face_id))
    }

    fn collect_line_segs_intersections<'a>(
        &'a self,
        line: Line,
        seg_refs: impl Iterator<Item = &'a SegRef<'a>> + Clone,
        vertex_pulling_sq: Dec,
        _do_debug: bool,
    ) -> usize {
        //let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        //let vertex_pulling_sq = vertex_pulling * vertex_pulling;
        if _do_debug {
            println!("Line: {line:?}");
        }

        let mut hits_points_new = seg_refs
            .clone()
            .filter_map(|seg_ref| {
                let distance_to = line.distance_to_pt_squared(seg_ref.from());
                if distance_to < vertex_pulling_sq {
                    let dot = (seg_ref.from() - line.origin).dot(&line.dir);
                    // Filter for positive line direction
                    if dot.is_positive() {
                        if _do_debug {
                            println!(
                                " push pt: {} because {} is positive distance_to: {}",
                                seg_ref.from_pt(),
                                dot,
                                distance_to
                            );
                        }
                        return Some(seg_ref.from_pt());
                    }
                }
                None
            })
            .map(Either::Left)
            .collect_vec();

        // Collect also points, that hitting segments somewhere in half
        for seg_ref in seg_refs.clone() {
            if _do_debug {
                println!(
                    " {:?} -> {:?}: {:?}",
                    seg_ref.from_pt(),
                    seg_ref.to_pt(),
                    seg_ref.rib_id
                );
            }
            let some_ab = line.get_intersection_params_seg_ref(seg_ref);
            if let Some((a, b)) = some_ab {
                let diff = (b - Dec::one()).abs().min(b.abs());
                let hits_point = diff < vertex_pulling_sq;
                let hits_segment = b > Dec::zero() && b < Dec::one();

                // Take only positive line direction
                if (hits_point || hits_segment) && a > Zero::zero() {
                    let pt = line.origin + line.dir * a;

                    if !hits_points_new
                        .iter()
                        .map(|lr| match lr {
                            Either::Left(pt) => self.vertices.get_point(*pt),
                            Either::Right(v) => *v,
                        })
                        .any(|v| (v - pt).magnitude_squared() < vertex_pulling_sq)
                    {
                        if _do_debug {
                            let ab = line.get_intersection_params_seg_ref(seg_ref);
                            println!(
                                " push vertex: {} {} {}  ({}, {}) [[{ab:?}]]",
                                pt.x, pt.y, pt.z, a, b
                            );
                            println!(" seg dir: {:?}", seg_ref.dir().normalize());
                        }
                        hits_points_new.push(Either::Right(pt));
                    }
                }
            }
        }
        // All points and intersecions collected, lets analyze em.

        // With all collected points - lets check, if our line crosses both point of some segment
        let total_segments_hitted = seg_refs
            .clone()
            .filter(|sr| {
                hits_points_new
                    .iter()
                    .any(|p| p.left().is_some_and(|pt| sr.from_pt() == pt))
                    && hits_points_new
                        .iter()
                        .any(|p| p.left().is_some_and(|pt| sr.to_pt() == pt))
            })
            .collect_vec();

        // And remove points, that we see as a single segment
        hits_points_new.retain(|item| {
            !item.left().is_some_and(|pt| {
                total_segments_hitted
                    .iter()
                    .any(|sr| [sr.from_pt(), sr.to_pt()].contains(&pt))
            })
        });

        let crossed_points = hits_points_new
            .iter()
            .filter_map(|hp| hp.left())
            .filter(|hp| {
                let from = seg_refs.clone().find(|sr| sr.from_pt() == *hp);
                let to = seg_refs.clone().find(|sr| sr.to_pt() == *hp);

                match (from, to) {
                    (Some(from), Some(to)) => {
                        let base = self.vertices.get_point(*hp);
                        let f = from.to() - base;
                        let t = to.from() - base;
                        let norm = f.normalize().cross(&line.dir).normalize();
                        let perpendicular_in_plane = norm.cross(&line.dir).normalize();
                        let fd = perpendicular_in_plane.dot(&f);
                        let td = perpendicular_in_plane.dot(&t);

                        fd.is_positive() != td.is_positive()
                    }
                    _ => false,
                }
            })
            .collect_vec();

        let crossed_on_ribs = total_segments_hitted
            .iter()
            .filter(|seg_hitted| {
                let from = seg_refs
                    .clone()
                    .find(|sr| sr.from_pt() == seg_hitted.to_pt());
                let to = seg_refs
                    .clone()
                    .find(|sr| sr.to_pt() == seg_hitted.from_pt());

                // TODO: Check this out
                match (from, to) {
                    (Some(from), Some(to)) => {
                        let f = from.to() - from.from();
                        let t = to.from() - to.to();
                        let norm = f.normalize().cross(&line.dir).normalize();
                        let perpendicular_in_plane = norm.cross(&line.dir).normalize();
                        let fd = perpendicular_in_plane.dot(&f);
                        let td = perpendicular_in_plane.dot(&t);

                        fd.is_positive() != td.is_positive()
                    }
                    _ => false,
                }
            })
            .collect_vec();

        if _do_debug {
            println!(
                "ribs:{} points:{:?} hits_points_new: {} [{:?}]",
                crossed_on_ribs.len(),
                crossed_points,
                hits_points_new.iter().filter_map(|hp| hp.right()).count(),
                hits_points_new
            )
        }
        crossed_on_ribs.len()
            + crossed_points.len()
            + hits_points_new.iter().filter_map(|hp| hp.right()).count()
    }

    fn collect_line_face_intersections(
        &self,
        line: Line,
        face_id: FaceId,
        vertex_pulling_sq: Dec,
        _do_debug: bool,
    ) -> usize {
        let all_face_segments = self
            .load_face_ref(face_id)
            .segments(SegmentDir::Fow)
            .collect_vec();
        self.collect_line_segs_intersections(
            line.clone(),
            all_face_segments.iter(),
            vertex_pulling_sq,
            _do_debug,
        )
    }

    /// Check if line from center of segment and direction of segment dir crossed with plane normal
    /// intersects any other segment in polygon.
    /// If intersects one of them in positive direction of line - I assume_
    fn rib_inside_face(&self, rib_id: RibId, face_id: FaceId) -> bool {
        let poly_plane = self.faces[&face_id].plane().clone();

        let rib_from = rib_id.make_ref(self).from();
        let rib_to = rib_id.make_ref(self).to();

        let line_normal_in_poly_plane = rib_id
            .make_ref(self)
            .dir()
            .normalize()
            .cross(&poly_plane.normal())
            .normalize();

        let line = Line {
            origin: rib_from.lerp(&rib_to, Dec::from(1) / 2),
            dir: line_normal_in_poly_plane,
        };
        let vertex_pulling = rib_id
            .make_ref(self)
            .magnitude()
            .div(2)
            .min(Dec::one() / Dec::from(1000));
        if face_id.0 == 2482 && rib_id == 3846 {
            println!("select vp: {vertex_pulling}");
        }

        let total_intersects = self.collect_line_face_intersections(
            line,
            face_id,
            vertex_pulling * vertex_pulling,
            face_id.0 == 2482 && rib_id == 3846,
        );
        if face_id.0 == 2482 && rib_id == 3846 {
            println!("Counted intersecions for 3846 and 2482 {total_intersects}");
        }

        /*
        let is_i = if face_pts.contains(&rib_id.make_ref(self).from_pt())
            || face_pts.contains(&rib_id.make_ref(self).to_pt())
        {
            println!(
                "face border contains some point of rib => need to check, if other point is inside {total_intersects}"
            );
            (total_intersects.saturating_sub(1)) % 2 != 0
        } else {
            println!("All rib points somewhere in space");

            total_intersects % 2 != 0
        };
        */
        total_intersects % 2 != 0
    }

    fn is_opposite_face(&self, face_id: FaceId, face: Face) -> bool {
        self.faces
            .get(&face_id)
            .is_some_and(|f| matches!(f.is_opposite_face(&face), FaceToFaceRelation::Opposite))
    }

    pub fn get_mesh_polygons(&self, mesh_id: MeshId) -> Vec<UnrefPoly> {
        self.meshes
            .get(&mesh_id)
            .iter()
            .flat_map(|mesh| {
                mesh.polies
                    .keys()
                    .map(|&poly_id| UnrefPoly { mesh_id, poly_id })
            })
            .collect()
    }

    pub(crate) fn load_polygon_ref(&self, mesh_id: MeshId, ix: PolyId) -> PolyRef {
        PolyRef {
            poly_id: ix,
            mesh_id,
            index: self,
        }
    }

    pub fn new_mesh(&mut self) -> MeshId {
        let mesh_id = self.get_next_mesh_id();
        self.meshes.insert(mesh_id, Mesh::default());
        mesh_id
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PolygonFilter {
    Front,
    Back,
    Shared,
}

#[derive(Clone, Copy, Debug)]
pub enum PolygonRelation {
    SrcPolygonFrontOfTool,
    SrcPolygonBackOfTool,
    ToolPolygonBackOfSrc,
    ToolPolygonFrontOfSrc,
}
