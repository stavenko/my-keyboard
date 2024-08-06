use std::collections::BTreeMap;
use std::fmt::format;
use std::path::PathBuf;
use std::{
    borrow::Cow,
    collections::{HashMap, HashSet, VecDeque},
    fmt::Debug,
    hash::Hash,
};

use itertools::{Either, Itertools};
use nalgebra::{ComplexField, Vector3};
use num_traits::{One, Signed, Zero};
use rstar::{RTree, AABB};
use rust_decimal_macros::dec;
use stl_io::Triangle;

use crate::linear::line::Line;
use crate::planar::plane::Plane;
use crate::polygon_basis::PolygonBasis;
use crate::{
    decimal::Dec,
    indexes::{
        aabb::Aabb,
        geo_index::{poly::PolyRef, seg::SegRef},
        polygon_oriented_bb::PolygonOrientedBb,
        vertex_index::{PtId, VertexIndex},
    },
    linear::segment::Segment,
    planar::polygon::Polygon,
    primitives_relation::{linear_point::PointOnLine, planar::PlanarRelation, relation::Relation},
    reversable::Reversable,
};

use super::rib::RibRef;
use super::seg::SegmentRef;
use super::tri_iter::TriIter;
use super::{
    mesh::{Mesh, MeshId, MeshRef, MeshRefMut},
    poly::{Poly, PolyId, PolyRtreeRecord},
    rib::{Rib, RibId},
    seg::{Seg, SegmentDir},
};

#[derive(Debug)]
pub struct GeoIndex {
    pub(crate) vertices: VertexIndex,
    pub(super) polygon_index: RTree<PolyRtreeRecord>,
    pub(super) ribs: HashMap<RibId, Rib>,
    pub(super) polygons: BTreeMap<PolyId, Poly>,
    pub(super) meshes: HashMap<MeshId, Mesh>,
    pub(super) pt_to_ribs: HashMap<PtId, Vec<RibId>>,
    pub(super) polygon_bounding_boxes: HashMap<PolyId, PolygonOrientedBb>,
    pub(super) rib_to_poly: HashMap<RibId, Vec<PolyId>>,
    pub(super) partially_split_polygons: HashMap<PolyId, Vec<RibId>>,
    pub(super) default_mesh: MeshId,
    pub(super) polygon_splits: HashMap<PolyId, Vec<PolyId>>,
    pub(super) rib_parent: HashMap<RibId, RibId>,
    pub(super) deleted_polygons: HashMap<PolyId, Poly>,
    pub(super) split_ribs: HashMap<RibId, Vec<RibId>>,
    pub(super) poly_to_mesh: HashMap<PolyId, Vec<MeshId>>,
    poly_split_debug: HashMap<PolyId, PolygonBasis>,
    input_polygon_min_rib_length: Dec,
    points_precision: Dec,
    rib_counter: usize,
    poly_counter: usize,
    mesh_counter: usize,
    current_color: usize,
    debug_path: PathBuf,
}

impl GeoIndex {
    pub fn new(aabb: Aabb) -> Self {
        let default_mesh = MeshId(0);
        let meshes = [(default_mesh, Mesh(Vec::new()))]
            .into_iter()
            .collect::<HashMap<_, _>>();
        let vertices = VertexIndex::new(aabb);

        Self {
            meshes,
            default_mesh,
            vertices,
            poly_to_mesh: Default::default(),
            polygon_index: Default::default(),
            ribs: Default::default(),
            polygons: Default::default(),
            pt_to_ribs: Default::default(),
            polygon_bounding_boxes: Default::default(),
            rib_to_poly: Default::default(),
            partially_split_polygons: Default::default(),
            polygon_splits: Default::default(),
            split_ribs: Default::default(),
            rib_parent: Default::default(),
            deleted_polygons: Default::default(),
            input_polygon_min_rib_length: Default::default(),
            points_precision: Default::default(),
            rib_counter: Default::default(),
            poly_counter: Default::default(),
            mesh_counter: Default::default(),
            poly_split_debug: HashMap::new(),

            current_color: 0,
            debug_path: "/tmp/".into(),
        }
    }

    pub fn poly_split_debug(&mut self, poly_id: impl Into<PolyId>, basis: PolygonBasis) {
        self.poly_split_debug.insert(poly_id.into(), basis);
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

    pub(super) fn triangles(&self) -> impl Iterator<Item = Triangle> + '_ {
        self.all_polygons()
            .filter_map(|poly_ref| poly_ref.triangles().ok())
            .flatten()
    }

    fn get_next_rib_id(&mut self) -> RibId {
        self.rib_counter += 1;
        RibId(self.rib_counter)
    }

    fn get_next_poly_id(&mut self) -> PolyId {
        self.poly_counter += 1;
        PolyId(self.poly_counter)
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
        segment_loop: &'b mut [Seg],
        (from, to): (PtId, PtId),
    ) -> (&'b [Seg], &'b [Seg]) {
        let from = segment_loop
            .iter()
            .map(|s| self.load_segref(s))
            .position(|sr| sr.from_pt() == from)
            .unwrap();

        segment_loop.rotate_left(from);

        let to = segment_loop
            .iter()
            .map(|s| self.load_segref(s))
            .position(|sr| sr.from_pt() == to)
            .unwrap();

        let (fronts, backs) = segment_loop.split_at(to);
        (fronts, backs)
    }

    pub fn split_poly_by_chain(&mut self, chain: Vec<Seg>, poly_id: PolyId) -> [PolyId; 2] {
        let pr = self.load_polygon_ref(poly_id);
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

        let mut segments = self.polygons[&poly_id].segments.clone();

        let (fronts, backs) = self.split_segment_loop(&mut segments, (chain_last, chain_first));

        let fronts = [fronts, &chain.into_iter().map(|s| s.seg()).collect_vec()].concat();
        let backs = [
            backs,
            &reversed_chain.into_iter().map(|s| s.seg()).collect_vec(),
        ]
        .concat();

        if fronts.len() < 3 || backs.len() < 3 {
            panic!("Less than 3 segments per polygon is not possible {poly_id:?}");
        }

        /*
        if poly_id == 913 {
            for seg in &fronts {
                let s = self.load_segref(&seg);
                println!(
                    "FRONTS s[{:?}] {:?} --> {:?}",
                    s.rib_id,
                    s.from_pt(),
                    s.to_pt()
                );
            }
            for seg in &backs {
                let s = self.load_segref(&seg);
                println!(
                    "BACKs s[{:?}] {:?} --> {:?}",
                    s.rib_id,
                    s.from_pt(),
                    s.to_pt()
                );
            }
        }
        */

        let front_aabb = self.calculate_aabb_from_segments(&fronts);
        let back_aabb = self.calculate_aabb_from_segments(&backs);

        let poly_one = Poly {
            segments: fronts,
            plane: pr.get_plane(),
            aabb: front_aabb,
        };
        let poly_two = Poly {
            segments: backs,
            plane: pr.get_plane(),
            aabb: back_aabb,
        };

        let one = self.insert_poly(poly_one);
        let two = self.insert_poly(poly_two);
        let new_ids = [one, two].into_iter().map(|a| a.0).collect_vec();
        self.replace_polygons_in_meshes(poly_id, &new_ids);
        self.remove_polygon(poly_id);
        [one, two]
            .into_iter()
            .map(|(child_id, is_created)| {
                if poly_id == 10 {
                    println!("Create poly {child_id:?} from {poly_id:?}");
                }
                for r in &ribs_to_index {
                    Self::save_index(&mut self.rib_to_poly, *r, child_id);
                }
                for mesh_id in self
                    .meshes
                    .iter()
                    .filter(|(_, polies)| polies.0.contains(&child_id))
                    .map(|a| a.0)
                {
                    Self::save_index(&mut self.poly_to_mesh, child_id, *mesh_id);
                }
                self.unify_ribs(child_id);

                self.split_sub_polygons(child_id);
                //println!("after split subs {poly_id:?}{child_id:?}");
                Self::save_index(&mut self.polygon_splits, poly_id, child_id);
                poly_id
            })
            .collect_vec();

        new_ids.try_into().expect("ok")
    }

    pub fn split_polygon_by_closed_chain(
        &mut self,
        poly_id: PolyId,
        mut chain: Vec<Seg>,
    ) -> Vec<PolyId> {
        let pr = self.load_polygon_ref(poly_id);
        let pb = pr.calculate_polygon_basis();
        let area = |s: SegRef| {
            s.from().dot(&pb.x) * s.to().dot(&pb.y) - s.to().dot(&pb.x) * s.from().dot(&pb.y)
        };

        let poly_area: Dec = pr.segments_iter().map(area).sum();
        let loop_area: Dec = chain.iter().map(|s| self.load_segref(s)).map(area).sum();
        if (loop_area * poly_area).is_positive() {
            chain = chain.into_iter().map(|s| s.flip()).rev().collect_vec();
        }

        let segs = [chain.clone(), self.polygons[&poly_id].segments.clone()].concat();
        let chain_pts = chain
            .iter()
            .map(|s| self.load_segref(s))
            .map(|s| s.from_pt())
            .collect_vec();

        let chain_center = chain_pts
            .iter()
            .fold(Vector3::zeros(), |a, p| a + self.vertices.get_point(*p))
            / Dec::from(chain.len());

        if let Some(first) = chain.iter().find_map(|ch_s| {
            let from = ch_s.from(&self.ribs);
            let normal = (self.vertices.get_point(from) - chain_center).normalize();

            let candidates = self.polygons[&poly_id]
                .segments
                .iter()
                .map(|s| s.from(&self.ribs))
                .filter(|pt| self.is_bridge(&segs, (from, *pt)))
                .sorted_by_key(|pt| {
                    (self.vertices.get_point(*pt) - self.vertices.get_point(from))
                        .normalize()
                        .dot(&normal)
                })
                .collect_vec();

            candidates.last().map(|to| (from, *to))
        }) {
            let first_in_chain = self.vertices.get_point(first.0);
            let opposite = chain_pts
                .into_iter()
                .filter(|pt| *pt != first.0)
                .sorted_by_key(|pt| {
                    (self.vertices.get_point(*pt) - first_in_chain).magnitude_squared()
                })
                .rev()
                .find_map(|from| {
                    let normal = (self.vertices.get_point(from) - chain_center).normalize();

                    let candidates = self.polygons[&poly_id]
                        .segments
                        .iter()
                        .map(|s| s.from(&self.ribs))
                        .filter(|pt| self.is_bridge(&segs, (from, *pt)))
                        .sorted_by_key(|pt| {
                            (self.vertices.get_point(*pt) - self.vertices.get_point(from))
                                .normalize()
                                .dot(&normal)
                        })
                        .collect_vec();

                    candidates.last().map(|to| (from, *to))
                });

            if let Some(opposite) = opposite {
                let (rib, dir) = Rib::build(first.0, first.1);
                let (rib_id, _) = self.insert_rib(rib);
                let seg_first = Seg { rib_id, dir };
                let seg_first_rev = seg_first.flip();
                let (rib, dir) = Rib::build(opposite.0, opposite.1);
                let (rib_id, _) = self.insert_rib(rib);
                let seg_opposite = Seg { rib_id, dir };
                let seg_opposite_rev = seg_opposite.flip();

                let (chain_front, chain_back) =
                    self.split_segment_loop(&mut chain, (first.0, opposite.0));

                let mut polygon_segments = self.polygons[&poly_id].segments.clone();

                let (poly_front, poly_back) =
                    self.split_segment_loop(&mut polygon_segments, (opposite.1, first.1));

                let one = [chain_front, &[seg_opposite], poly_front, &[seg_first_rev]].concat();
                let two = [chain_back, &[seg_first], poly_back, &[seg_opposite_rev]].concat();
                let three = chain.into_iter().map(|s| s.flip()).rev().collect_vec();
                let plane = self.polygons[&poly_id].plane.clone();

                let aabb = self.calculate_aabb_from_segments(&one);
                let one = self.insert_poly(Poly {
                    segments: one,
                    plane: plane.clone(),
                    aabb,
                });
                let aabb = self.calculate_aabb_from_segments(&two);
                let two = self.insert_poly(Poly {
                    segments: two,
                    plane: plane.clone(),
                    aabb,
                });
                let aabb = self.calculate_aabb_from_segments(&three);
                let three = self.insert_poly(Poly {
                    segments: three,
                    plane,
                    aabb,
                });

                if let Some(basis) = self.poly_split_debug.get(&poly_id).cloned() {
                    self.debug_svg_poly(poly_id, &basis);
                }

                //self.debug_svg_poly(one_id, &basis);
                //self.debug_svg_poly(two_id, &basis);
                // Self::save_index(&mut self.polygon_splits, poly_id, one_id);
                //Self::save_index(&mut self.polygon_splits, poly_id, two_id);
                //Self::save_index(&mut self.polygon_splits, poly_id, three_id);
                //return vec![one_id, two_id, three_id];
                let polies = [one, two, three].into_iter().map(|a| a.0).collect_vec();

                self.replace_polygons_in_meshes(poly_id, &polies);
                [one, two, three]
                    .into_iter()
                    .map(|(child_polygon_id, is_created)| {
                        if child_polygon_id == 320 {
                            println!("{child_polygon_id:?} is created: {}", is_created);
                        }
                        Self::save_index(&mut self.polygon_splits, poly_id, child_polygon_id);
                        if let Some(basis) = self.poly_split_debug.get(&poly_id).cloned() {
                            self.debug_svg_poly(child_polygon_id, &basis);
                        }
                        if !is_created {
                            for mesh_id in self
                                .meshes
                                .iter()
                                .filter(|(_, polies)| polies.0.contains(&child_polygon_id))
                                .map(|a| a.0)
                            {
                                Self::save_index(&mut self.poly_to_mesh, poly_id, *mesh_id);
                            }
                        }
                        self.unify_ribs(child_polygon_id);
                        self.split_sub_polygons(child_polygon_id);
                        //println!("after split subs (CIR) {poly_id:?}{child_polygon_id:?}");
                        child_polygon_id
                    })
                    .collect_vec();
                self.remove_polygon(poly_id);
                return polies;
            }
        }

        panic!("Cannot find bridge points");
    }

    fn split_sub_polygons(&mut self, tool_id: PolyId) {
        let tool_aabb = self.polygons[&tool_id].aabb;
        let tool_plane = &self.polygons[&tool_id].plane;
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;

        let polygons = self
            .polygon_index
            .locate_in_envelope_intersecting(&tool_aabb.into())
            .map(|o| o.0)
            .filter(|&p| p != tool_id)
            .filter(|p| {
                !matches!(
                    self.polygons[p].plane.relate(tool_plane),
                    PlanarRelation::Intersect(_)
                )
            })
            .filter(|p| {
                let tool_ribs = self.polygons[&tool_id]
                    .segments
                    .iter()
                    .map(|s| s.rib_id)
                    .collect::<HashSet<_>>();
                let src_ribs = self.polygons[p]
                    .segments
                    .iter()
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
        for polygon in polygons {
            let tool_ribs = self.polygons[&tool_id]
                .segments
                .iter()
                .map(|s| s.rib_id)
                .collect::<HashSet<_>>();
            let src_ribs = self.polygons[&polygon]
                .segments
                .iter()
                .map(|s| s.rib_id)
                .collect::<HashSet<_>>();

            let difference_src_tool = src_ribs.difference(&tool_ribs).copied().collect_vec();
            let mut chains_src_tool = self.collect_seg_chains(difference_src_tool);
            while let Some(pos) = chains_src_tool
                .iter()
                .position(|chain| self.is_chain_inside_polygon(chain, tool_id))
            {
                let chain = chains_src_tool.swap_remove(pos);
                for c in chain {
                    if tool_id == 913 && c.rib_id == 1634 {
                        //panic!("!!!!!");
                    }
                    Self::save_index(&mut self.partially_split_polygons, tool_id, c.rib_id);
                }
            }
            let difference_tool_src = tool_ribs.difference(&src_ribs).copied().collect_vec();
            let mut chain_tool_src = self.collect_seg_chains(difference_tool_src);

            while let Some(pos) = chain_tool_src
                .iter()
                .position(|chain| self.is_chain_inside_polygon(chain, polygon))
            {
                let chain = chain_tool_src.swap_remove(pos);
                for c in chain {
                    if tool_id == 913 && c.rib_id == 1634 {
                        //panic!("!!!!!");
                    }
                    Self::save_index(&mut self.partially_split_polygons, polygon, c.rib_id);
                }
            }
        }
    }

    fn is_bridge(&self, segments: &[Seg], (from, to): (PtId, PtId)) -> bool {
        let affected = segments
            .iter()
            .map(|s| self.load_segref(s))
            .filter(|sr| !(sr.has(from) || sr.has(to)))
            .collect_vec();

        let segment_ref = SegmentRef::new(from, to, self);
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;

        for sr in segments.iter().filter(|s| {
            let sr = self.load_segref(s);

            segment_ref.distance_to_pt_squared(sr.from()).abs() < vertex_pulling_sq
        }) {
            if let Some((a, _)) = segment_ref.get_intersection_params_seg_ref(&self.load_segref(sr))
            {
                if a > Dec::zero() && a < Dec::one() {
                    return false;
                }
            }
        }

        let is_some_intersected = affected
            .into_iter()
            .filter_map(|sr| segment_ref.get_intersection_params_seg_ref(&sr))
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

    pub fn split_polygons_by_orphan_ribs(&mut self) {
        while let Some((poly_id, cutting_chain, leftoffs)) = self
            .partially_split_polygons
            .iter()
            .sorted_by_key(|(p, _)| *p)
            .find_map(|(poly_id, ribs)| {
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

                let mut chains = self.collect_seg_chains(ribs);
                if let Some((ix, mut cutting_set)) =
                    chains.iter().enumerate().find_map(|(ix, chain)| {
                        if let Some((splitting_chain, leftoffs)) =
                            self.collect_splitting_chain(*poly_id, chain.clone())
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

                    Some((*poly_id, cutting_set.0, cutting_set.1))
                } else {
                    None
                }
            })
        {
            let new_polies = if self.is_chain_circular(&cutting_chain) {
                self.split_polygon_by_closed_chain(poly_id, cutting_chain)
            } else {
                if let Some(basis) = self.poly_split_debug.get(&poly_id).cloned() {
                    self.debug_svg_poly(poly_id, &basis);
                }

                let ps = self.split_poly_by_chain(cutting_chain, poly_id).to_vec();

                if let Some(basis) = self.poly_split_debug.get(&poly_id).cloned() {
                    for new_poly in &ps {
                        self.debug_svg_poly(*new_poly, &basis);
                        println!("new polygon from {poly_id:?} saved: {new_poly:?}");
                    }
                }
                ps
            };

            self.partially_split_polygons.remove(&poly_id);

            for rib in leftoffs {
                for p in &new_polies {
                    Self::save_index(&mut self.partially_split_polygons, *p, rib);
                }
            }
        }
    }

    pub fn split_polygons_by_orphan_ribs_old(&mut self) {
        let polygons = self.partially_split_polygons.keys().copied().collect_vec();
        let mut splits = HashMap::new();
        let mut unused_chains: HashMap<PolyId, Vec<RibId>> = HashMap::new();
        for poly_id in polygons {
            if let Some(ribs) = self.partially_split_polygons.remove(&poly_id) {
                let mut chains = self.collect_seg_chains(ribs.clone());
                let mut checked_polygons = [poly_id].into_iter().collect::<HashSet<_>>();
                while let Some(chain) = chains.pop() {
                    if chain.first().expect("must be").from(&self.ribs)
                        == chain.last().expect("must be").to(&self.ribs)
                    {
                        if checked_polygons.len() > 1 {
                            panic!("Implement splitting for several polygons");
                        }

                        let splitted = self.split_polygon_by_closed_chain(poly_id, chain);
                        checked_polygons.extend(splitted);
                    } else if let Some(poly_id) = checked_polygons
                        .iter()
                        .find(|checked_poly_id| {
                            chain
                                .iter()
                                .skip(1) // So we are not using first point of chain
                                .map(|seg| seg.from(&self.ribs)) // ...and not using last
                                .collect::<HashSet<_>>()
                                .intersection(&self.get_polygon_points(**checked_poly_id))
                                .count()
                                > 0
                        })
                        .cloned()
                    {
                        let poly_ref = self.load_polygon_ref(poly_id);
                        let chain_without_polygon_ribs = chain
                            .into_iter()
                            .filter(|seg| {
                                poly_ref.segments_iter().all(|ps| ps.rib_id != seg.rib_id)
                            })
                            .collect_vec();
                        if let Some(seg_id) = chain_without_polygon_ribs
                            .iter()
                            .map(|seg| self.load_segref(seg))
                            .position(|chain_seg| {
                                poly_ref
                                    .segments_iter()
                                    .any(|poly_seg| poly_seg.to_pt() == chain_seg.from_pt())
                            })
                        {
                            let rotated_chain = {
                                let mut ch = chain_without_polygon_ribs;
                                ch.rotate_left(seg_id);
                                ch
                            };

                            if let Some(seg_id) = rotated_chain
                                .iter()
                                .map(|seg| self.load_segref(seg))
                                .position(|chain_seg| {
                                    poly_ref
                                        .segments_iter()
                                        .any(|poly_seg| poly_seg.from_pt() == chain_seg.to_pt())
                                })
                            {
                                let (chain_part, chain_rest) = rotated_chain.split_at(seg_id + 1);
                                let [front, back] =
                                    self.split_poly_by_chain(chain_part.to_vec(), poly_id);
                                splits.insert(poly_id, [front, back]);
                                checked_polygons.remove(&poly_id);
                                checked_polygons.insert(front);

                                checked_polygons.insert(back);

                                if !chain_rest.is_empty() {
                                    chains.push(chain_rest.to_vec());
                                }
                            }
                        }
                    } else if let Some(poly_id) =
                        checked_polygons.clone().into_iter().find(|&poly_id| {
                            let polygon_points = self.get_polygon_points(poly_id);
                            let chain_start_on_poly = polygon_points
                                .iter()
                                .any(|&pt| chain.first().unwrap().from(&self.ribs) == pt);
                            let chain_ends_on_poly = polygon_points
                                .iter()
                                .any(|&pt| chain.last().unwrap().to(&self.ribs) == pt);

                            let pr = PolyRef {
                                poly_id,
                                index: self,
                            };

                            let rev_chain =
                                chain.iter().map(|s| s.clone().flip()).rev().collect_vec();
                            let poly_includes_chain =
                                pr.has_chain(&chain) || pr.has_chain(&rev_chain);

                            !poly_includes_chain && chain_start_on_poly && chain_ends_on_poly
                        })
                    {
                        let [front, back] = self.split_poly_by_chain(chain, poly_id);

                        splits.insert(poly_id, [front, back]);

                        checked_polygons.remove(&poly_id);
                        checked_polygons.insert(front);
                        checked_polygons.insert(back);
                    } else {
                        for p in &checked_polygons {
                            for r in chain.iter().map(|seg| seg.rib_id) {
                                Self::save_index(&mut unused_chains, *p, r);
                            }
                        }
                    }
                }
            }
        }

        for (poly_id, [front, back]) in splits {
            if unused_chains.contains_key(&poly_id) {
                if let Some(chain_ribs) = unused_chains.remove(&poly_id) {
                    let pts = chain_ribs
                        .iter()
                        .flat_map(|r| [self.ribs[r].0, self.ribs[r].1])
                        .sorted()
                        .dedup()
                        .collect_vec();
                    let is_relative_to_front = self.polygons[&front]
                        .segments
                        .iter()
                        .map(|seg| SegRef {
                            rib_id: seg.rib_id,
                            dir: seg.dir,
                            index: self,
                        })
                        .any(|sr| pts.iter().any(|pt| sr.has(*pt)));
                    let is_relative_to_back = self.polygons[&back]
                        .segments
                        .iter()
                        .map(|seg| SegRef {
                            rib_id: seg.rib_id,
                            dir: seg.dir,
                            index: self,
                        })
                        .any(|sr| pts.iter().any(|pt| sr.has(*pt)));

                    if is_relative_to_front {
                        unused_chains.insert(front, chain_ribs.clone());
                    }
                    if is_relative_to_back {
                        unused_chains.insert(back, chain_ribs.clone());
                    }

                    if !is_relative_to_back && !is_relative_to_front {
                        unused_chains.insert(back, chain_ribs.clone());
                        unused_chains.insert(front, chain_ribs);
                    }
                }
            }
        }

        for (p, ribs) in unused_chains {
            for r in ribs {
                Self::save_index(&mut self.partially_split_polygons, p, r);
            }
        }
    }

    fn is_polygon_in_front(
        &self,
        relative_to_rib: RibId,
        some_poly: PolyId,
        cutter: PolyId,
    ) -> Option<bool> {
        let some_poly_plane = self.polygons[&some_poly].plane.clone();
        let cutter_plane = self.polygons[&cutter].plane.clone();
        if let Some(projected_plane_normal) = some_poly_plane.project_unit(cutter_plane.normal()) {
            let rib = &self.ribs[&relative_to_rib];

            let rib_from = self.vertices.get_point(rib.0);
            let rib_to = self.vertices.get_point(rib.1);

            let line = Line {
                origin: rib_from.lerp(&rib_to, Dec::from(1) / 2),
                dir: projected_plane_normal,
            };

            //let mut hits_points: Vec<Vector3<Dec>> = Vec::new();
            //let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
            //let vertex_pulling_sq = vertex_pulling * vertex_pulling;
            let all_but_this_rib = self
                .load_polygon_ref(some_poly)
                .segments_iter()
                .filter(|sr| sr.rib_id != relative_to_rib)
                .collect_vec();

            let total_intersects =
                self.collect_line_segs_intersections(line, all_but_this_rib.iter());

            Some(total_intersects % 2 == 1)
        } else {
            None
        }
    }

    pub fn save_mesh<'i, 'o>(
        &'i mut self,
        bounds: impl Iterator<Item = Cow<'o, Polygon>>,
    ) -> MeshId {
        let mesh_id = self.insert_mesh(Mesh(Vec::new()));
        for polygon_id in bounds {
            self.save_polygon(&polygon_id, Some(mesh_id));
        }

        mesh_id
    }

    pub(crate) fn split_rib_in_poly_using_indexed_pt(
        &mut self,
        pt: PtId,
        rib_id: RibId,
        poly_id: PolyId,
    ) -> Vec<RibId> {
        self.split_rib_in_poly_using_indexed_pts(&[pt], rib_id, poly_id)
    }

    /*
    pub(crate) fn split_rib_in_poly(
        &mut self,
        pt: Vector3<Dec>,
        rib_id: RibId,
        poly_id: PolyId,
    ) -> Vec<RibId> {
        if let Some(ix) = self.polygons[&poly_id]
            .segments
            .iter()
            .position(|s| s.rib_id == rib_id)
        {
            let seg = self.polygons[&poly_id].segments[ix];
            let segment = self.load_segment(&seg);
            let replacement = [
                Segment {
                    from: segment.from,
                    to: pt,
                },
                Segment {
                    from: pt,
                    to: segment.to,
                },
            ]
            .map(|s| self.save_segment(s));
            let new_ids = replacement.iter().map(|s| s.rib_id).collect();

            for r in replacement {
                Self::save_index(&mut self.rib_to_poly, r.rib_id, poly_id);
            }
            if let Some(poly) = self.polygons.get_mut(&poly_id) {
                poly.segments.splice(ix..(ix + 1), replacement);
            }
            new_ids
        } else {
            panic!("NO RIB");
        }
    }
    */

    pub(crate) fn split_rib_in_poly_using_indexed_pts(
        &mut self,
        pts: &[PtId],
        rib_id: RibId,
        poly_id: PolyId,
    ) -> Vec<RibId> {
        if let Some(ix) = self.polygons[&poly_id]
            .segments
            .iter()
            .position(|s| s.rib_id == rib_id)
        {
            let seg = self.polygons[&poly_id].segments[ix];

            let seg_ref = SegRef {
                rib_id,
                dir: seg.dir,
                index: self,
            };
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
                    replacement.push(self.save_segment(Segment { from: v, to: *vn }));
                }
            }
            let new_ids = replacement.iter().map(|s| s.rib_id).collect();

            if poly_id == 3389 {
                for r in &replacement {
                    let sr = self.load_segref(r);
                    let f = sr.from();
                    let t = sr.to();
                    println!(
                        "  >replacement: {:?} -> {:?}       {} {} {}  --->  {} {} {}",
                        sr.from_pt(),
                        sr.to_pt(),
                        f.x.round_dp(6),
                        f.y.round_dp(6),
                        f.z.round_dp(6),
                        t.x.round_dp(6),
                        t.y.round_dp(6),
                        t.z.round_dp(6),
                    );
                }
                let seg_ref = SegRef {
                    rib_id,
                    dir: seg.dir,
                    index: self,
                };
                let f = seg_ref.from();
                let t = seg_ref.to();
                println!(
                    "  >replacing segment: {:?} -> {:?}          {} {} {}  --->  {} {} {}",
                    seg_ref.from_pt(),
                    seg_ref.to_pt(),
                    f.x.round_dp(6),
                    f.y.round_dp(6),
                    f.z.round_dp(6),
                    t.x.round_dp(6),
                    t.y.round_dp(6),
                    t.z.round_dp(6),
                );
            }

            for r in &replacement {
                Self::save_index(&mut self.rib_to_poly, r.rib_id, poly_id);
                self.rib_parent.insert(r.rib_id, rib_id);
            }

            if let Some(poly) = self.polygons.get_mut(&poly_id) {
                poly.segments.splice(ix..(ix + 1), replacement);
            }

            new_ids
        } else {
            panic!("NO RIB");
        }
    }

    pub fn save_as_polygon<S>(
        &mut self,
        polygon_vertices: &[Vector3<S>],
        mesh_id: Option<MeshId>,
    ) -> anyhow::Result<()>
    where
        S: Into<Dec> + nalgebra::Scalar + nalgebra::Field + Copy,
    {
        let vx = polygon_vertices
            .iter()
            .map(|s| Vector3::new(s.x.into(), s.y.into(), s.z.into()))
            .collect_vec();
        let polygon = Polygon::new(vx)?;

        self.save_polygon(&polygon, mesh_id);
        Ok(())
    }

    pub fn get_current_default_mesh(&self) -> MeshId {
        self.default_mesh
    }

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
            .map(|s| self.save_segment(s))
            .collect_vec();

        let plane = polygon.get_plane();

        let poly = Poly {
            segments: segments.clone(),
            aabb,
            plane,
        };

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

        vec![self.save_segment(segment)]
    }

    pub fn save_segment(&mut self, segment: Segment) -> Seg {
        let from = self.insert_point(segment.from);
        let to = self.insert_point(segment.to);

        if from == to {
            panic!("Segment too short: {:?}", segment);
        }
        let (rib, dir) = Rib::build(from, to);

        let (rib_id, _) = self.insert_rib(rib);
        Self::save_index(&mut self.pt_to_ribs, from, rib_id);
        Self::save_index(&mut self.pt_to_ribs, to, rib_id);

        Seg { rib_id, dir }
    }

    pub fn load_polygon_ref(&self, other_id: PolyId) -> PolyRef<'_> {
        PolyRef {
            poly_id: other_id,
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

    fn insert_poly(&mut self, poly: Poly) -> (PolyId, bool) {
        if let Some(poly_id) = self
            .polygons
            .iter()
            .find(|&(_, s)| *s == poly)
            .map(|(id, _)| id)
        {
            (*poly_id, false)
        } else {
            let poly_id = self.get_next_poly_id();
            let rtree_item = PolyRtreeRecord(poly_id, poly.aabb);
            self.polygon_index.insert(rtree_item);
            for seg in &poly.segments {
                Self::save_index(&mut self.rib_to_poly, seg.rib_id, poly_id);
            }
            self.polygons.insert(poly_id, poly);

            let poly_ref = PolyRef {
                poly_id,
                index: self,
            };
            let polygon_bounding_box = PolygonOrientedBb::create_from_poly(poly_ref);
            self.polygon_bounding_boxes
                .insert(poly_id, polygon_bounding_box);

            (poly_id, true)
        }
    }

    /*
    fn insert_mesh_bound(&mut self, item: MeshBound) -> MeshBoundId {
        if let Some(id) = self
            .mesh_bounds
            .iter()
            .find(|&(_, s)| *s == item)
            .map(|(id, _)| id)
        {
            *id
        } else {
            let id = MeshBoundId::default();
            self.mesh_bounds.insert(id, item);
            id
        }
    }
    */

    fn insert_mesh(&mut self, item: Mesh) -> MeshId {
        if let Some(id) = self
            .meshes
            .iter()
            .find(|&(_, s)| *s == item)
            .map(|(id, _)| id)
        {
            *id
        } else {
            let id = self.get_next_mesh_id();
            self.meshes.insert(id, item);
            id
        }
    }

    fn insert_point(&mut self, pt: Vector3<Dec>) -> PtId {
        self.vertices.get_or_insert_point(pt, self.points_precision)
    }

    fn save_index<Ix, Item>(index: &mut HashMap<Ix, Vec<Item>>, ix: Ix, item: Item)
    where
        Ix: Hash + Eq,
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

    fn remove_item_from_index<Ix, Item>(index: &mut HashMap<Ix, Vec<Item>>, ix: &Ix, item: &Item)
    where
        Ix: Hash + Eq,
        Item: PartialEq,
    {
        if let Some(items) = index.get_mut(ix) {
            items.retain(|i| i != item)
        }
        if index.get(ix).is_some_and(|v| v.is_empty()) {
            index.remove(ix);
        }
    }

    fn load_segment(&self, seg: &Seg) -> Segment {
        let from = self.vertices.get_point(seg.from(&self.ribs));
        let to = self.vertices.get_point(seg.to(&self.ribs));
        Segment { from, to }
    }

    pub(crate) fn remove_mesh(&mut self, mesh_id: MeshId) {
        if let Some(mesh) = self.meshes.remove(&mesh_id) {
            for p in mesh.0 {
                self.remove_polygon(p);
            }
        }
    }

    /// Remove polygon from all available related structures
    pub fn remove_polygon(&mut self, poly_id: PolyId) {
        if let Some(poly) = self.polygons.remove(&poly_id) {
            let ribs = poly.segments.iter().map(|s| s.rib_id).collect_vec();

            for r in ribs {
                Self::remove_item_from_index(&mut self.rib_to_poly, &r, &poly_id);
            }

            self.polygon_index
                .remove(&PolyRtreeRecord(poly_id, poly.aabb));
            self.deleted_polygons.insert(poly_id, poly);
            self.polygon_bounding_boxes.remove(&poly_id);
        }
    }

    /// Return all polygons, belonging to given mesh
    pub fn get_mesh_polygon_ids(&self, tool_mesh_id: MeshId) -> impl Iterator<Item = PolyId> + '_ {
        self.meshes
            .get(&tool_mesh_id)
            .into_iter()
            .flat_map(|mesh| &mesh.0)
            .copied()
    }

    pub(super) fn get_polygon_points(&self, polygon_id: PolyId) -> HashSet<PtId> {
        self.polygons
            .get(&polygon_id) // find mesh
            .into_iter()
            .flat_map(|mesh| &mesh.segments) // take in bound ids ...
            .map(|seg| seg.rib_id)
            .filter_map(|rib_id| self.ribs.get(&rib_id))
            .flat_map(|rib| [rib.0, rib.1])
            .collect()
    }

    pub(crate) fn drop_polygons(&mut self, polygon_ids: impl Iterator<Item = PolyId>) {
        for p in polygon_ids {
            self.remove_polygon(p);
        }
    }

    pub(crate) fn polygons(&self, mesh_id: &MeshId) -> impl Iterator<Item = PolyRef<'_>> {
        self.meshes
            .get(mesh_id)
            .into_iter()
            .flat_map(|m| &m.0)
            .map(|polygon_id| self.load_polygon_ref(*polygon_id))
    }

    pub(crate) fn all_polygons(&self) -> impl Iterator<Item = PolyRef<'_>> {
        self.polygons
            .keys()
            .map(|polygon_id| self.load_polygon_ref(*polygon_id))
    }

    fn replace_polygons_in_meshes(&mut self, poly_id: PolyId, polygons: &[PolyId]) {
        let mesh_id = self.get_mesh_for_polygon(poly_id);
        let polygon_index = self
            .meshes
            .get(&mesh_id)
            .into_iter()
            .flat_map(|mesh| &mesh.0)
            .position(|p| *p == poly_id)
            .expect("We cannot find polygon position in newfound mesh");

        for (i, new_polygon_id) in polygons.iter().enumerate() {
            if i == 0 {
                if let Some(mesh) = self.meshes.get_mut(&mesh_id) {
                    mesh.0[polygon_index] = *new_polygon_id;
                }
            } else if let Some(m) = self.meshes.get_mut(&mesh_id) {
                m.0.push(*new_polygon_id);
            }
        }
    }

    pub(crate) fn get_polygon_meshes(&self, poly_id: PolyId) -> Vec<MeshId> {
        self.meshes
            .iter()
            .filter(|item| item.1 .0.contains(&poly_id))
            .map(|item| *item.0)
            .collect()
    }

    pub(crate) fn get_mesh_for_polygon(&self, poly_id: PolyId) -> MeshId {
        if let Some(mesh_id) = self
            .meshes
            .iter()
            .find(|(_, m)| m.0.contains(&poly_id))
            .map(|(id, _)| *id)
        {
            mesh_id
        } else {
            panic!("Cannot find mesh for polygon {poly_id:?}");
        }
    }

    pub fn get_mesh(&self, mesh_id: MeshId) -> MeshRef<'_> {
        MeshRef {
            mesh_id,
            geo_index: self,
        }
    }

    pub(crate) fn get_polygon_vertices(&self, poly_id: PolyId) -> Vec<Vector3<Dec>> {
        self.get_polygon_points(poly_id)
            .into_iter()
            .map(|pt| self.vertices.get_point(pt))
            .collect()
    }

    pub(crate) fn get_mesh_vertices(&self, tool: MeshId) -> HashSet<PtId> {
        self.meshes
            .get(&tool)
            .into_iter()
            .flat_map(|m| &m.0)
            .filter_map(|polygon_id| self.polygons.get(polygon_id))
            .flat_map(|polygon| &polygon.segments) // take in bound ids ...
            .map(|seg| seg.rib_id)
            .filter_map(|rib_id| self.ribs.get(&rib_id))
            .flat_map(|rib| [rib.0, rib.1])
            .collect()
    }

    fn remove_rib(&mut self, rib_id: RibId) {
        if self.rib_to_poly.get(&rib_id).is_some_and(|v| !v.is_empty()) {
            panic!("rib index to poly is not empty");
        }
        if let Some(rib) = self.ribs.remove(&rib_id) {
            Self::remove_item_from_index(&mut self.pt_to_ribs, &rib.0, &rib_id);
            Self::remove_item_from_index(&mut self.pt_to_ribs, &rib.1, &rib_id);
        }

        self.rib_to_poly.remove(&rib_id);
    }

    fn get_rib(&self, r: RibId) -> RibRef<'_> {
        RibRef {
            index: self,
            rib_id: r,
        }
    }

    pub fn flip_polygon(&mut self, poly_id: PolyId) {
        for p in self.polygons.get_mut(&poly_id).into_iter() {
            for s in p.segments.iter_mut() {
                *s = s.flip();
            }
            p.segments.reverse();

            p.plane = p.plane.clone().flip();
        }
    }

    pub(crate) fn print_debug_polygon(&self, poly_id: PolyId) -> String {
        let pr = PolyRef {
            poly_id,
            index: self,
        };

        pr.segments_iter()
            .map(|s| {
                let f = s.from();
                let t = s.to();
                format!("{} {} {} -> {} {} {}", f.x, f.y, f.z, t.x, t.y, t.z)
            })
            .join("\n")
    }

    pub fn save_as_single_mesh(&mut self) -> MeshId {
        let mesh = Mesh(self.polygons.keys().cloned().collect_vec());
        let mesh_id = self.get_next_mesh_id();
        self.meshes.insert(mesh_id, mesh);
        mesh_id
    }

    fn collect_intersections(
        &self,
        poly_id: PolyId,
        tool: PolyId,
    ) -> Vec<(Either<Vector3<Dec>, PtId>, RibId)> {
        let plane = self.load_polygon_ref(tool).get_plane();
        let mut vertices = Vec::new();

        for seg in self.load_polygon_ref(poly_id).segments_iter() {
            if seg.to_pt() == seg.from_pt() {
                panic!("Seg to equals to seg_from");
            }
            if let Some(t) = plane.get_intersection_param2(seg.from(), seg.to()) {
                if t >= Dec::zero() && t <= Dec::one() {
                    vertices.push((seg.from().lerp(&seg.to(), t), seg.rib_id));
                }
            }
        }
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;
        vertices
            .clone()
            .into_iter()
            .map(|(v, rib)| {
                if let Some(pt) = self.get_polygon_points(poly_id).into_iter().find(|pt| {
                    let poly_vertex = self.vertices.get_point(*pt);
                    (poly_vertex - v).magnitude_squared() < vertex_pulling_sq
                }) {
                    (Either::Right(pt), rib)
                } else {
                    (Either::Left(v), rib)
                }
            })
            .collect()
    }

    fn create_common_ribs(&mut self, tool_id: PolyId, mesh_id: MeshId) {
        let mut tool_aabb = self.polygons[&tool_id].aabb;

        let polygons = self
            .polygon_index
            .locate_in_envelope_intersecting(&tool_aabb.into())
            .map(|o| o.0)
            .filter(|&p| p != tool_id)
            .filter(|p| !self.meshes[&mesh_id].0.contains(p))
            .collect_vec();

        if polygons.is_empty() {
            return;
        }

        let tool_plane = self.load_polygon_ref(tool_id).get_plane();
        let tool_ribs = self.polygons[&tool_id]
            .segments
            .iter()
            .map(|s| s.rib_id)
            .collect::<HashSet<_>>();

        for src_id in polygons.iter() {
            let src_ribs = self.polygons[src_id]
                .segments
                .iter()
                .map(|s| s.rib_id)
                .collect::<HashSet<_>>();
            if tool_id == 173 {
                println!("cut src with {src_id:?}");
            }

            /*
            if src_ribs.intersection(&tool_ribs).count() > 0 {
                panic!("tool and src already have common ribs");
                continue;
            }
            */

            let src_plane = self.load_polygon_ref(*src_id).get_plane();
            let common_line = match tool_plane.relate(&src_plane) {
                PlanarRelation::Intersect(line) => line,
                _ => {
                    continue;
                }
            };

            let vertices_src = self.collect_intersections(*src_id, tool_id);
            let vertices_tool = self.collect_intersections(tool_id, *src_id);

            if tool_id == 38 && src_id.0 == 24 {
                println!("=======tool: {tool_id:?} src:{src_id:?}===========");
                for sr in self.load_polygon_ref(tool_id).segments_iter() {
                    let v = sr.from();
                    println!(
                        "tool: [{:?}] {:?} -> {:?}  [{} {} {}]",
                        sr.rib_id,
                        sr.from_pt(),
                        sr.to_pt(),
                        v.x,
                        v.y,
                        v.z
                    );
                }

                for v in &vertices_tool {
                    match v.0 {
                        Either::Left(ve) => {
                            println!(
                                "got tool v: {} {} {}  [{:?}]",
                                ve.x.round_dp(6),
                                ve.y.round_dp(6),
                                ve.z.round_dp(6),
                                v.1,
                            )
                        }
                        Either::Right(pt) => {
                            let ve = self.vertices.get_point(pt);
                            println!(
                                "got tool pt {pt:?}: {} {} {} [{:?}]",
                                ve.x.round_dp(6),
                                ve.y.round_dp(6),
                                ve.z.round_dp(6),
                                v.1,
                            );
                        }
                    }
                }

                for sr in self.load_polygon_ref(*src_id).segments_iter() {
                    let v = sr.from();
                    println!(
                        "src: {:?} {:?} -> {:?} [{} {} {}]",
                        sr.rib_id,
                        sr.from_pt(),
                        sr.to_pt(),
                        v.x,
                        v.y,
                        v.z
                    );
                }
                for v in &vertices_src {
                    match v.0 {
                        Either::Left(ve) => {
                            println!(
                                "got src v: {} {} {}  [{:?}]",
                                ve.x.round_dp(6),
                                ve.y.round_dp(6),
                                ve.z.round_dp(6),
                                v.1,
                            )
                        }
                        Either::Right(pt) => {
                            let ve = self.vertices.get_point(pt);
                            println!(
                                "got src pt {pt:?}: {} {} {} [{:?}]",
                                ve.x.round_dp(6),
                                ve.y.round_dp(6),
                                ve.z.round_dp(6),
                                v.1,
                            );
                        }
                    }
                }
            }
            let mut cut_ribs_index = HashMap::new();
            let pts_src = vertices_src
                .into_iter()
                .map(|(v, rib_id)| match v {
                    Either::Left(v) => {
                        let pt = self.vertices.get_or_insert_point(v, self.points_precision);
                        (pt, rib_id)
                    }
                    Either::Right(pt) => (pt, rib_id),
                })
                //.filter(|(pt, _)| !pts_src.contains(pt))
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

            let mut pts_src = pts_src
                .into_iter()
                .sorted_by_key(|pt| {
                    (self.vertices.get_point(*pt) - common_line.origin).dot(&common_line.dir)
                })
                .dedup()
                .collect_vec();

            let mut pts_tool = pts_tool
                .into_iter()
                .sorted_by_key(|pt| {
                    (self.vertices.get_point(*pt) - common_line.origin).dot(&common_line.dir)
                })
                .dedup()
                .collect_vec();

            //if !(src_id.0 == 4 || src_id.0 == 3) {
            pts_src = self.collaps_points_on_same_rib(pts_src, *src_id);
            pts_tool = self.collaps_points_on_same_rib(pts_tool, tool_id);
            //}

            if tool_id == 38 && *src_id == 24 {
                println!("SRC {pts_src:?}");
                println!("TOL {pts_tool:?}")
            }
            let mut segs_src = Vec::new();
            let mut segs_tool = Vec::new();

            for c in pts_src.chunks(2) {
                if let Ok(inner_segment) = <&[PtId] as TryInto<[PtId; 2]>>::try_into(c) {
                    segs_src.push(inner_segment);
                }
            }
            for c in pts_tool.chunks(2) {
                if let Ok(inner_segment) = <&[PtId] as TryInto<[PtId; 2]>>::try_into(c) {
                    segs_tool.push(inner_segment);
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

                if tool_id == 38 && src_id.0 == 24 {
                    println!("a: {a:?}");
                    println!("b: {b:?}");
                    println!("common: {common:?}");
                    for s in self.load_polygon_ref(tool_id).segments_iter() {
                        println!("tool: {} ~~>  {}", s.from_pt(), s.to_pt());
                    }
                }

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

            if tool_id == 38 && src_id.0 == 24 {
                println!("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                for s in &segs_src {
                    println!("src: {s:?} ");
                }
                for s in &segs_tool {
                    println!("tool: {s:?} ");
                }
            }

            for s in segs_src {
                for t in &segs_tool {
                    if let Some([a, b]) = intersection(s, *t) {
                        new_ribs.push(Rib::build(a, b).0);
                    }
                }
            }

            for rib in new_ribs {
                let (r, is_created) = self.insert_rib(rib);
                if tool_id == 899 {
                    println!("created rib {r:?}:  {tool_id:?}, {src_id:?},")
                }

                if is_created {
                    for polygon_id in [tool_id, *src_id] {
                        while let Some(splits) =
                            self.find_intersecting_ribs_on_same_line_in_polygon(r, polygon_id)
                        {
                            for (rib_id, pts) in splits {
                                for poly_id in
                                    self.rib_to_poly.remove(&rib_id).into_iter().flatten()
                                {
                                    let new_ribs = self
                                        .split_rib_in_poly_using_indexed_pts(&pts, rib_id, poly_id);

                                    new_ribs.iter().for_each(|r| {
                                        Self::save_index(&mut self.split_ribs, rib_id, *r)
                                    });
                                }

                                self.remove_rib(rib_id);
                            }
                        }
                    }
                    /*
                    for rib in self.find_ribs_on_same_line(r, tool_id) {
                        //println!("{r:?}, {rib:?}");
                        self.split_ribs_on_same_line(r, rib);
                    }
                    for rib in self.find_ribs_on_same_line(r, *src_id) {
                        self.split_ribs_on_same_line(r, rib);
                    }
                    */

                    if !self
                        .rib_to_poly
                        .get(&r)
                        .is_some_and(|ps| ps.contains(&tool_id))
                    {
                        Self::save_index(&mut self.partially_split_polygons, tool_id, r);
                    }

                    if !self
                        .rib_to_poly
                        .get(&r)
                        .is_some_and(|ps| ps.contains(src_id))
                    {
                        Self::save_index(&mut self.partially_split_polygons, *src_id, r);
                    }

                    Self::save_index(&mut self.pt_to_ribs, rib.0, r);
                    Self::save_index(&mut self.pt_to_ribs, rib.1, r);

                    for rib_point in [rib.0, rib.1] {
                        for poly_rib_id in cut_ribs_index.remove(&rib_point).into_iter().flatten() {
                            // let poly_rib = self.ribs[&poly_rib_id];

                            if self.ribs.get(&poly_rib_id).is_some_and(|poly_rib| {
                                poly_rib.0 != rib_point && poly_rib.1 != rib_point
                            }) {
                                for poly_id in
                                    self.rib_to_poly.remove(&poly_rib_id).into_iter().flatten()
                                {
                                    if let Some(b) = self.poly_split_debug.get(&poly_id).cloned() {
                                        println!("\n<!-- ~~~~~~~~~~~~~~~~Before cut rib {poly_rib_id:?} in {poly_id:?}~~~~~ -->");
                                        //self.debug_svg_poly(poly_id, &b);
                                        for sr in self.load_polygon_ref(poly_id).segments_iter() {
                                            let f = sr.from();
                                            let t = sr.to();
                                            println!(
                                                "before: {:?} -> {:?} {} {} {}  --->  {} {} {}",
                                                sr.from_pt(),
                                                sr.to_pt(),
                                                f.x.round_dp(6),
                                                f.y.round_dp(6),
                                                f.z.round_dp(6),
                                                t.x.round_dp(6),
                                                t.y.round_dp(6),
                                                t.z.round_dp(6),
                                            );
                                        }
                                    }

                                    self.split_rib_in_poly_using_indexed_pt(
                                        rib_point,
                                        poly_rib_id,
                                        poly_id,
                                    );
                                    if let Some(b) = self.poly_split_debug.get(&poly_id).cloned() {
                                        println!("\n<!--~~~~~~~~~~~~~~~~ after cut rib~~~~~ -->");
                                        //self.debug_svg_poly(poly_id, &b);
                                        for sr in self.load_polygon_ref(poly_id).segments_iter() {
                                            let f = sr.from();
                                            let t = sr.to();
                                            println!(
                                                "after: {:?} -> {:?}   {} {} {}  --->  {} {} {}",
                                                sr.from_pt(),
                                                sr.to_pt(),
                                                f.x.round_dp(6),
                                                f.y.round_dp(6),
                                                f.z.round_dp(6),
                                                t.x.round_dp(6),
                                                t.y.round_dp(6),
                                                t.z.round_dp(6),
                                            );
                                        }
                                        println!();
                                    }
                                }
                            }
                        }
                    }
                } else {
                    if !self
                        .rib_to_poly
                        .get(&r)
                        .is_some_and(|ps| ps.contains(&tool_id))
                    {
                        println!("save for tool");
                        Self::save_index(&mut self.partially_split_polygons, tool_id, r);
                    }

                    if !self
                        .rib_to_poly
                        .get(&r)
                        .is_some_and(|ps| ps.contains(src_id))
                    {
                        println!(":{}> save for src {r:?}", line!());
                        Self::save_index(&mut self.partially_split_polygons, *src_id, r);
                    }
                }
            }
        }
    }

    fn find_intersecting_ribs_on_same_line_in_polygon(
        &self,
        rib_id: RibId,
        poly_id: PolyId,
    ) -> Option<HashMap<RibId, Vec<PtId>>> {
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

        self.load_polygon_ref(poly_id)
            .segments_iter()
            .filter(|seg| {
                line.distance_to_pt_squared(seg.from()).abs() < vertex_pulling_sq
                    && line.distance_to_pt_squared(seg.to()).abs() < vertex_pulling_sq
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
                let mut split_ribs = HashMap::new();
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
                    Some(split_ribs)
                }
            })
    }

    fn find_ribs_on_same_line(&self, rib_id: RibId, poly_id: PolyId) -> Vec<RibId> {
        let rr = RibRef {
            rib_id,
            index: self,
        };
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;

        let line = crate::linear::line::Line {
            origin: rr.from(),
            dir: rr.dir().normalize(),
        };

        self.load_polygon_ref(poly_id)
            .segments_iter()
            .filter(|seg| {
                line.distance_to_pt_squared(seg.from()).abs() < vertex_pulling_sq
                    && line.distance_to_pt_squared(seg.to()).abs() < vertex_pulling_sq
            })
            .map(|s| s.rib_id)
            .collect_vec()
    }

    fn split_ribs_on_same_line(&mut self, one_rib: RibId, other_rib: RibId) -> Vec<RibId> {
        let rib1 = RibRef {
            rib_id: one_rib,
            index: self,
        };
        let rib2 = RibRef {
            rib_id: other_rib,
            index: self,
        };

        let line = crate::linear::line::Line {
            origin: rib1.from(),
            dir: rib1.dir().normalize(),
        };

        let pts = (vec![rib1.from_pt(), rib1.to_pt(), rib2.from_pt(), rib2.to_pt()])
            .into_iter()
            .sorted_by_key(|pt| {
                let v = self.vertices.get_point(*pt);
                (v - line.origin).dot(&line.dir)
            })
            .dedup()
            .collect_vec();

        let mut split_ribs = HashMap::new();

        if pts.len() > 2 {
            let rib_start_ix = pts.iter().position(|pt| *pt == rib1.from_pt()).unwrap();
            let rib_end_ix = pts.iter().position(|pt| *pt == rib1.to_pt()).unwrap();
            let seg_start_ix = pts.iter().position(|pt| *pt == rib2.from_pt()).unwrap();
            let seg_end_ix = pts.iter().position(|pt| *pt == rib2.to_pt()).unwrap();

            let between = |a: usize, b: usize, c: usize| (c > a && c < b) || (c > b && c < a);

            if between(rib_start_ix, rib_end_ix, seg_start_ix) {
                Self::save_index(&mut split_ribs, one_rib, pts[seg_start_ix]);
            } else if between(rib_start_ix, rib_end_ix, seg_end_ix) {
                Self::save_index(&mut split_ribs, one_rib, pts[seg_end_ix]);
            } else if between(seg_start_ix, seg_end_ix, rib_start_ix) {
                Self::save_index(&mut split_ribs, other_rib, pts[rib_start_ix]);
            } else if between(seg_start_ix, seg_end_ix, rib_end_ix) {
                Self::save_index(&mut split_ribs, other_rib, pts[rib_end_ix]);
            };
        }

        let mut splitted = Vec::new();
        for (rib_id, pts) in split_ribs {
            for poly_id in self.rib_to_poly.remove(&rib_id).into_iter().flatten() {
                if let Some(basis) = self.poly_split_debug.get(&poly_id).cloned() {
                    println!("<!-- before cutting rib in polygon: {poly_id:?} -->");
                    self.debug_svg_poly(poly_id, &basis);
                    println!();
                    println!();
                    println!();
                }
                let new_ribs = self.split_rib_in_poly_using_indexed_pts(&pts, rib_id, poly_id);
                new_ribs
                    .iter()
                    .for_each(|r| Self::save_index(&mut self.split_ribs, rib_id, *r));
                splitted.extend(new_ribs);

                if let Some(basis) = self.poly_split_debug.get(&poly_id).cloned() {
                    println!("<!-- after cutting rib in polygon: {poly_id:?} -->");
                    self.debug_svg_poly(poly_id, &basis);
                    println!();
                    println!();
                }
            }
            self.remove_rib(rib_id);
        }
        splitted
    }

    /// make common lines have common ribs
    /// This function processes only those ribs, which are on one line
    fn unify_ribs(&mut self, tool_id: PolyId) {
        let tool_aabb = self.polygons[&tool_id].aabb;
        let polygons = self
            .polygon_index
            .locate_in_envelope_intersecting(&tool_aabb.into())
            .map(|o| o.0)
            .filter(|&p| p != tool_id)
            .collect_vec();

        for p in polygons.iter() {
            while let Some(splits) = self
                .polygons
                .get(p)
                .into_iter()
                .flat_map(|p| &p.segments)
                .map(|s| s.rib_id)
                .find_map(|rib| self.find_intersecting_ribs_on_same_line_in_polygon(rib, tool_id))
            {
                let mut splitted = Vec::new();
                for (rib_id, pts) in splits {
                    for poly_id in self.rib_to_poly.remove(&rib_id).into_iter().flatten() {
                        if let Some(basis) = self.poly_split_debug.get(&poly_id).cloned() {
                            println!("<!-- before cutting rib in polygon: {poly_id:?} -->");
                            self.debug_svg_poly(poly_id, &basis);
                            println!();
                            println!();
                            println!();
                        }
                        let new_ribs =
                            self.split_rib_in_poly_using_indexed_pts(&pts, rib_id, poly_id);

                        new_ribs
                            .iter()
                            .for_each(|r| Self::save_index(&mut self.split_ribs, rib_id, *r));

                        splitted.extend(new_ribs);

                        if let Some(basis) = self.poly_split_debug.get(&poly_id).cloned() {
                            println!("<!-- after cutting rib in polygon: {poly_id:?} -->");
                            self.debug_svg_poly(poly_id, &basis);
                            println!();
                            println!();
                        }
                    }

                    self.remove_rib(rib_id);
                }
                //splitted;
                //todo!("Rewrite ribs selection {splitted:?}");
            }
            /*
            for rib in self
                .polygons
                .get(p)
                .into_iter()
                .flat_map(|p| &p.segments)
                .map(|s| s.rib_id)
                .collect_vec()
            {
                for r in self.find_ribs_on_same_line(rib, tool_id) {
                    println!("RRR: {rib:?} {r:?}");
                    self.split_ribs_on_same_line(rib, r);
                    if !self.ribs.contains_key(&rib) {
                        break;
                    }
                }
            }
            */
        }
    }

    pub fn create_new_mesh_and_set_as_default(&mut self) -> MeshId {
        let mesh_id = self.get_next_mesh_id();
        self.meshes.insert(mesh_id, Mesh(Vec::new()));
        self.default_mesh = mesh_id;

        mesh_id
    }

    pub(crate) fn mesh_borders(&self, mesh_id: MeshId) -> Vec<Vec<Seg>> {
        let mesh_polygons = self
            .meshes
            .get(&mesh_id)
            .into_iter()
            .flat_map(|m| &m.0)
            .copied()
            .collect::<HashSet<_>>();

        let border_ribs = mesh_polygons
            .iter()
            .filter_map(|p| self.polygons.get(&p))
            .flat_map(|p| p.segments.iter().map(|s| s.rib_id))
            .filter(|rib_id| {
                self.rib_to_poly[rib_id]
                    .iter()
                    .filter(|poly_id| mesh_polygons.contains(&poly_id))
                    .count()
                    == 1
            });

        self.collect_seg_chains(border_ribs.collect())
    }

    pub(crate) fn collect_split_chains(&self, mesh_id: MeshId, tool: MeshId) -> Vec<Vec<Seg>> {
        let common_chains = self.collect_common_chains(mesh_id, tool);
        let mut mesh_borders = None;
        let is_closed = |chain: &[Seg]| {
            chain.first().is_some_and(|first| {
                chain
                    .last()
                    .is_some_and(|last| last.to(&self.ribs) == first.from(&self.ribs))
            })
        };

        let is_touches_mesh_borders = |chain: &[Seg], borders: &mut Option<Vec<Vec<Seg>>>| {
            if borders.is_none() {
                *borders = Some(self.mesh_borders(mesh_id));
            }
            if let Some(borders) = borders {
                borders
                    .iter()
                    .flatten()
                    .map(|s| s.from(&self.ribs))
                    .any(|pt| chain.first().is_some_and(|seg| seg.from(&self.ribs) == pt))
                    && borders
                        .iter()
                        .flatten()
                        .map(|s| s.from(&self.ribs))
                        .any(|pt| chain.last().is_some_and(|seg| seg.to(&self.ribs) == pt))
            } else {
                false
            }
        };

        common_chains
            .into_iter()
            .filter(|chain| is_closed(chain) || is_touches_mesh_borders(chain, &mut mesh_borders))
            .collect()
    }

    pub(crate) fn collect_common_chains(&self, mesh_id: MeshId, tool: MeshId) -> Vec<Vec<Seg>> {
        let ribs = self
            .rib_to_poly
            .iter()
            .sorted_by_key(|(r, _)| r.0)
            .filter(|(_, polies)| polies.len() > 2)
            .filter(|(_, polies)| {
                polies
                    .iter()
                    .all(|p| [mesh_id, tool].contains(&self.get_mesh_for_polygon(*p)))
            })
            .map(|(rib_id, _)| *rib_id)
            .collect_vec();

        if mesh_id == 0 && tool == 2 {
            dbg!(&ribs);
            dbg!(&self.poly_to_mesh);
        }

        self.collect_seg_chains(ribs)
    }

    pub(crate) fn move_polygon(&mut self, poly_id: PolyId, mesh_id: MeshId) {
        let original = self.get_mesh_for_polygon(poly_id);
        self.meshes
            .get_mut(&original)
            .expect("ok")
            .0
            .retain(|p| *p != poly_id);
        self.meshes.get_mut(&mesh_id).expect("ok").0.push(poly_id);
    }

    pub fn move_all_polygons(&mut self, from_mesh: MeshId, to_mesh: MeshId) {
        for poly in self.meshes[&from_mesh].0.clone() {
            self.move_polygon(poly, to_mesh);
        }
        self.poly_to_mesh
            .retain(|_, ms| !(ms.contains(&from_mesh) && ms.len() > 1));
    }

    pub fn select_polygons(
        &self,
        of_mesh: MeshId,
        by_mesh: MeshId,
        filter: PolygonFilter,
    ) -> Vec<PolyId> {
        if of_mesh == 3 && by_mesh == 1 {
            println!("Polies in mesh 4: {:?}", self.meshes[&of_mesh].0);
            //println!("{:?}", self.get_polygon_meshes(PolyId(300)));
        }
        let ribs_with_polies = self
            .rib_to_poly
            .iter()
            .filter(|(rib_id, polies)| {
                if of_mesh == 3 && by_mesh == 1 && **rib_id == 17 {
                    println!("RIB ID {rib_id:?}: {}", self.rib_to_poly[rib_id].len());
                    for p in &self.rib_to_poly[rib_id] {
                        println!("  Sharing.. {:?} ", p); //, self.get_polygon_meshes(*p));
                                                          //println!("    \n    ---\n    {:?}\n", self.polygons.get(p));
                        for s in self.load_polygon_ref(*p).segments_iter() {
                            println!("     S: {:?} -> {:?}", s.from_pt(), s.to_pt())
                        }
                    }
                }
                let meshes = polies
                    .iter()
                    .flat_map(|p| self.get_polygon_meshes(*p))
                    .collect::<HashSet<_>>();
                meshes.contains(&of_mesh) && meshes.contains(&by_mesh)
            })
            .collect_vec();

        let mut planes: Vec<Plane> = Vec::new();
        let mut poly_plane = HashMap::new();

        let mut visited = BTreeMap::new();

        for (_, polies) in &ribs_with_polies {
            for p in polies.iter() {
                if *p == 300 {
                    println!("~~~~~~~~~~~>{:?}", self.get_polygon_meshes(*p));
                }
                let meshes = self.get_polygon_meshes(*p);
                let this_plane = &self.polygons[p].plane;
                if let Some(plane) = planes.iter().position(|p| *p == *this_plane) {
                    poly_plane.insert(*p, plane);
                } else {
                    poly_plane.insert(*p, planes.len());
                    planes.push(this_plane.to_owned());
                }
                if meshes.len() > 1 {
                    visited.insert(*p, PolygonFilter::Shared);
                }
            }
        }

        let mut ribs = Vec::with_capacity(ribs_with_polies.len());
        for (rib_id, polies) in ribs_with_polies {
            for group_of_polies_in_one_plane in polies
                .iter()
                .group_by(|p| poly_plane[p])
                .into_iter()
                .map(|(_, g)| g.collect_vec())
                .filter(|g| g.len() > 1)
                .filter(|g| {
                    g.iter().any(|poly| {
                        let meshes = self.get_polygon_meshes(**poly);
                        meshes.len() == 1 && meshes[0] == of_mesh
                    })
                })
            {
                // this two polies are in one plane
                // So we need to determine, state of both polies
                //
                ribs.push(*rib_id);
                let group_plane = self.polygons[group_of_polies_in_one_plane[0]].plane.clone();

                if let Some(cutter) = polies
                    .iter()
                    .filter(|cutter| !group_of_polies_in_one_plane.contains(cutter))
                    .min_by_key(|a| {
                        self.polygons[a]
                            .plane
                            .normal()
                            .dot(&group_plane.normal())
                            .abs()
                    })
                {
                    for polygon in &group_of_polies_in_one_plane {
                        if **polygon == 300 {
                            println!("Doing");
                        }
                        if !visited.contains_key(polygon) {
                            if let Some(mark) =
                                self.is_polygon_in_front(*rib_id, **polygon, *cutter)
                            {
                                let mark = if mark {
                                    PolygonFilter::Front
                                } else {
                                    PolygonFilter::Back
                                };
                                visited.insert(**polygon, mark);
                            } else {
                                println!("WTF?? could not cut {polygon:?} by {cutter:?}");
                                for p in &group_of_polies_in_one_plane {
                                    println!(
                                        "Plane of poly in one plane: {:?} {:?}",
                                        p, self.polygons[p].plane
                                    )
                                }
                                for p in polies {
                                    println!(
                                        "plane of polygon group: {:?} {:?}",
                                        p, self.polygons[p].plane
                                    )
                                }
                                panic!("SHIiiiiiit");
                            };
                        }
                    }
                }
            }
        }
        if of_mesh == 4 && by_mesh == 1 {
            dbg!(&visited);
            return visited
                .into_iter()
                .filter(|(_, r)| *r == filter)
                .map(|(p, _)| p)
                .collect_vec();
        }

        let visited = self.spread_visited_around(&ribs, of_mesh, visited);

        visited
            .into_iter()
            .filter(|(_, r)| *r == filter)
            .map(|(p, _)| p)
            .collect_vec()
    }

    fn get_polygon_root(&self, poly_id: PolyId) -> Option<PolyId> {
        self.polygon_splits
            .iter()
            .find(|p| p.1.contains(&poly_id))
            .map(|p| *p.0)
    }

    fn spread_visited_around(
        &self,
        common_ribs: &[RibId],
        of_mesh: MeshId,
        mut visited: BTreeMap<PolyId, PolygonFilter>,
    ) -> BTreeMap<PolyId, PolygonFilter> {
        for filter in [PolygonFilter::Front, PolygonFilter::Back] {
            'inner: loop {
                let adjacent = visited
                    .iter()
                    .filter(|&(_, f)| *f == filter)
                    .map(|(p, _)| p)
                    .flat_map(|p| {
                        self.polygons[p]
                            .segments
                            .iter()
                            .map(|s| s.rib_id)
                            .filter(|rib_id| !common_ribs.contains(rib_id))
                            .flat_map(|rib_id| &self.rib_to_poly[&rib_id])
                            .filter(|poly_id| self.get_mesh_for_polygon(**poly_id) == of_mesh)
                            .filter(|adjacent| !visited.contains_key(adjacent))
                    })
                    .collect::<HashSet<_>>();
                if adjacent.is_empty() {
                    break 'inner;
                }
                for p in adjacent {
                    visited.insert(*p, filter);
                }
            }
        }

        visited
    }

    /*
    pub fn select_polygons_old(
        &self,
        of_mesh: MeshId,
        by_mesh: MeshId,
        filter: PolygonFilter,
    ) -> Vec<PolyId> {
        let chains = self.collect_common_chains(of_mesh, by_mesh);
        let mut visited = BTreeMap::new();
        let mut chain_ribs = HashSet::new();

        if filter == PolygonFilter::Shared {
            return self
                .poly_to_mesh
                .iter()
                .filter(|(_, meshes)| meshes.contains(&of_mesh) && meshes.contains(&by_mesh))
                .map(|p| *p.0)
                .collect();
        }

        for chain in chains {
            chain_ribs.extend(chain.iter().map(|s| s.rib_id));
            visited.extend(self.mark_mesh_polygons_around_common_chain(chain, of_mesh));
        }

        loop {
            let adjacent = visited
                .iter()
                .filter(|&(_, f)| *f == filter)
                .map(|(p, _)| p)
                .flat_map(|p| {
                    self.polygons[p]
                        .segments
                        .iter()
                        .map(|s| s.rib_id)
                        .filter(|rib_id| !chain_ribs.contains(rib_id))
                        .flat_map(|rib_id| &self.rib_to_poly[&rib_id])
                        .filter(|poly_id| self.get_mesh_for_polygon(**poly_id) == of_mesh)
                        .filter(|adjacent| !visited.contains_key(adjacent))
                })
                .collect::<HashSet<_>>();
            if adjacent.is_empty() {
                break;
            }
            for p in adjacent {
                if self
                    .poly_to_mesh
                    .get(p)
                    .is_some_and(|meshes| meshes.contains(&of_mesh) && meshes.contains(&by_mesh))
                {
                    visited.insert(*p, PolygonFilter::Shared);
                } else {
                    visited.insert(*p, filter);
                }
            }
        }

        visited
            .into_iter()
            .filter(|(_, r)| *r == filter)
            .map(|(p, _)| p)
            .collect_vec()
    }
    */

    fn calculate_aabb_from_segments(&self, one: &[Seg]) -> Aabb {
        Aabb::from_points(
            &one.iter()
                .map(|seg| seg.from(&self.ribs))
                .map(|pt| self.vertices.get_point(pt))
                .collect_vec(),
        )
    }

    fn is_chain_circular(&self, chain: &[Seg]) -> bool {
        if chain.len() < 3 {
            false
        } else {
            chain.first().unwrap().from(&self.ribs) == chain.last().unwrap().to(&self.ribs)
        }
    }

    fn collect_splitting_chain(
        &self,
        poly_id: PolyId,
        chain: Vec<Seg>,
    ) -> Option<(Vec<Seg>, Vec<Seg>)> {
        let poly_ref = self.load_polygon_ref(poly_id);
        /*
        if poly_id == 314 {
            for s in &chain {
                println!("CHAIN: {:?} --> {:?}", s.from(&self.ribs), s.to(&self.ribs));
            }
        }
        */

        let chain_without_polygon_ribs = chain
            .into_iter()
            .filter(|seg| poly_ref.segments_iter().all(|ps| ps.rib_id != seg.rib_id))
            .collect_vec();

        /*
        if poly_id == 5 && is_chain_circular {
            for s in &chain_without_polygon_ribs {
                println!(
                    "CHAIN_NO_PSR: {:?} --> {:?}",
                    s.from(&self.ribs),
                    s.to(&self.ribs)
                );
            }
            panic!("~~~~")
        }
        if poly_id == 314 {
            for s in &chain_without_polygon_ribs {
                println!(
                    "CHAIN_NO_PSR: {:?} --> {:?}",
                    s.from(&self.ribs),
                    s.to(&self.ribs)
                );
            }
        }
        */
        if let Some(seg_id) = chain_without_polygon_ribs
            .iter()
            .map(|seg| self.load_segref(seg))
            .position(|chain_seg| {
                poly_ref
                    .segments_iter()
                    .any(|poly_seg| poly_seg.to_pt() == chain_seg.from_pt())
            })
        {
            let rotated_chain = {
                let mut ch = chain_without_polygon_ribs;
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
                    poly_ref.segments_iter().any(|poly_seg| {
                        poly_seg.from_pt() == chain_seg.to_pt()
                            && poly_seg.from_pt() != first_item_of
                    })
                })
            {
                let (chain_part, chain_rest) = rotated_chain.split_at(seg_id + 1);

                return Some((chain_part.to_vec(), chain_rest.to_vec()));
            }
        } else if self.is_chain_circular(&chain_without_polygon_ribs) {
            return Some((chain_without_polygon_ribs, Vec::new()));
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

    pub fn get_polygon_with_root_parent(&self, poly_id: PolyId) -> Vec<PolyId> {
        let mut collected = Vec::new();

        let mut to_check = vec![poly_id];
        while let Some(p) = to_check.pop() {
            if self.polygons.contains_key(&p) {
                collected.push(p);
            }
            if let Some(ps) = self.polygon_splits.get(&p) {
                to_check.extend(ps);
            }
        }

        collected
    }

    pub fn find_splitted_polygon_parent(&self, poly_id: PolyId) -> Option<PolyId> {
        self.polygon_splits
            .iter()
            .find(|(_, ps)| ps.contains(&poly_id))
            .map(|p| *p.0)
    }

    fn debug_svg_poly(&mut self, poly_id: PolyId, basis: &PolygonBasis) {
        const COLORS: &[&str] = &["magenta", "#fd9", "#f9d", "#df9", "#9fd", "#d9f", "#9df"];
        let color = COLORS[self.current_color % COLORS.len()];

        let pr = self.load_polygon_ref(poly_id);

        let filename = self.debug_path.join(format!("poly-{poly_id:?}.svg"));
        std::fs::write(filename, pr.svg_debug_fill(basis, color)).unwrap();

        self.current_color += 1;
    }

    /// Preconditions:
    /// Points sorted on one line before going here
    fn collaps_points_on_same_rib(&self, pts: Vec<PtId>, poly_id: PolyId) -> Vec<PtId> {
        // let mut pts = pts.into_iter().peekable();
        let mut result = Vec::new();
        let mut f = 0;

        while f < pts.len() {
            let pt1 = pts[f];
            if f + 1 == pts.len() {
                result.push(Either::Right(pts[f]));
                break;
            } else {
                let pt2 = pts[f + 1];
                let pt_ribs1 = self
                    .pt_to_ribs
                    .get(&pt1)
                    .iter()
                    .copied()
                    .flatten()
                    .collect::<HashSet<_>>();
                let pt_ribs2 = self
                    .pt_to_ribs
                    .get(&pt2)
                    .iter()
                    .copied()
                    .flatten()
                    .collect::<HashSet<_>>();
                let both_pt_ribs = pt_ribs1.intersection(&pt_ribs2).collect_vec();
                if let Some(rib_id) = both_pt_ribs.first().and_then(|rib_id| {
                    self.load_polygon_ref(poly_id)
                        .segments_iter()
                        .find(|seg| seg.rib_id == ***rib_id)
                }) {
                    result.push(Either::Left([pt1, pt2]));
                    f += 1;
                } else {
                    result.push(Either::Right(pt1));
                    f += 1;
                }
            }
        }

        result
            .into_iter()
            .flat_map(|f| match f {
                Either::Left(rib) => rib.to_vec(),
                Either::Right(pt) => vec![pt],
            })
            .collect()
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
            .all_polygons()
            .map(|poly_ref| poly_ref.serialized_polygon_pt())
            .map(|pts| format!("[{pts}]"))
            .join(", \n");

        format!("points={points};\n polyhedron(points, [{hedras}]);")
    }

    fn is_chain_inside_polygon(&self, chain: &[Seg], polygon: PolyId) -> bool {
        if polygon == 64 || polygon == 67 {
            println!("chain len: {}", chain.len());
        }
        chain.iter().all(|s| self.seg_inside_polygon(s, polygon))
    }

    fn collect_line_segs_intersections<'a>(
        &'a self,
        line: Line,
        seg_refs: impl Iterator<Item = &'a SegRef<'a>> + Clone,
    ) -> usize {
        let vertex_pulling = Dec::from(dec!(0.001)); // one micrometer
        let vertex_pulling_sq = vertex_pulling * vertex_pulling;

        let mut hits_points_new = seg_refs
            .clone()
            .filter_map(|seg_ref| {
                let distance_to = line.distance_to_pt_squared(seg_ref.from());
                if distance_to < vertex_pulling_sq {
                    let dot = (seg_ref.from() - line.origin).dot(&line.dir);
                    // Filter for positive line direction
                    if dot.is_positive() {
                        return Some(seg_ref.from_pt());
                    }
                }
                None
            })
            .map(Either::Left)
            .collect_vec();

        // Collect also points, that hitting segments somewhere in half
        for seg_ref in seg_refs.clone() {
            let some_ab = line.get_intersection_params_seg_ref_2(seg_ref);
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
            .filter(|sr| {
                let from = seg_refs.clone().find(|sr| sr.from_pt() == sr.to_pt());
                let to = seg_refs.clone().find(|sr| sr.to_pt() == sr.from_pt());

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

        crossed_on_ribs.len()
            + crossed_points.len()
            + hits_points_new.iter().filter_map(|hp| hp.right()).count()
    }

    fn collect_line_poly_intersections(&self, line: Line, some_poly: PolyId) -> usize {
        let all_polygon_segments = self
            .load_polygon_ref(some_poly)
            .segments_iter()
            .collect_vec();
        self.collect_line_segs_intersections(line.clone(), all_polygon_segments.iter())
    }

    /// Check if line from center of segment and direction of segment dir crossed with plane normal
    /// intersects any other segment in polygon.
    /// If intersects one of them in positive direction of line - I assume_
    fn seg_inside_polygon(&self, seg: &Seg, some_poly: PolyId) -> bool {
        let sr = self.load_segref(seg);
        let poly_plane = self.polygons[&some_poly].plane.clone();
        let rib_from = sr.from();
        let rib_to = sr.to();

        let line_normal_in_poly_plane =
            sr.dir().normalize().cross(&poly_plane.normal()).normalize();

        let line = Line {
            origin: rib_from.lerp(&rib_to, Dec::from(1) / 2),
            dir: line_normal_in_poly_plane,
        };

        let total_intersects = self.collect_line_poly_intersections(line, some_poly);

        total_intersects % 2 != 0
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PolygonFilter {
    Front,
    Back,
    Shared,
}

impl IntoIterator for GeoIndex {
    type Item = Triangle;

    type IntoIter = TriIter;

    fn into_iter(self) -> Self::IntoIter {
        let triangles = self.triangles().collect_vec();

        let size = dbg!(triangles.len());

        TriIter {
            inner: triangles.into_iter(),
            size,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum PolygonRelation {
    SrcPolygonFrontOfTool,
    SrcPolygonBackOfTool,
    ToolPolygonBackOfSrc,
    ToolPolygonFrontOfSrc,
}
#[cfg(test)]
mod tests {

    #[test]
    fn real_world_problem() {
        // let mut index = GeoIndex::default();

        // #[rustfmt::skip]
        // let p1: &[Vector3<Dec>] = &[
        // Vector3::new(dec!(20.6180339887).into(), dec!(12.4559581815).into(), dec!(3.8817317944).into()),
        // Vector3::new(dec!(20.6180339887).into(), dec!(12.4560890688).into(), dec!(3.8826800212).into()),
        // Vector3::new(dec!(20.6180339887).into(), dec!(12.9097762979).into(), dec!(7.1694655225).into()),
        // Vector3::new(dec!(21.6180339887).into(), dec!(12.1900579255).into(), dec!(7.2688109267).into()),
        // Vector3::new(dec!(21.6180339887).into(), dec!(11.7365480016).into(), dec!(3.9833099320).into()),
        // Vector3::new(dec!(20.9079214406).into(), dec!(12.2474101976).into(), dec!(3.9111780219).into()),
        // Vector3::new(dec!(20.7607867695).into(), dec!(12.3532603778).into(), dec!(3.8962323560).into()),

        // ];

        // #[rustfmt::skip]
        // let p2: &[Vector3<Dec>] = &[
        // Vector3::new(dec!(18.6975304844).into(), dec!(14.1709823847).into(), dec!(3.5093506678).into() ),
        // Vector3::new(dec!(18.8115659456).into(), dec!(13.7573368844).into(), dec!(3.7111817178).into() ),
        // Vector3::new(dec!(20.6180339887).into(), dec!(12.4560890688).into(), dec!(3.8826800212).into() ),
        // Vector3::new(dec!(20.7607867695).into(), dec!(12.3532603778).into(), dec!(3.8962323560).into() ),
        // ];

        // index.create_default_mesh();
        // let p1 = index.save_as_polygon(p1, None).unwrap()[0];
        // let p2 = index.save_as_polygon(p2, None).unwrap()[0];

        // let pl = index.calculate_polygon_plane(p2);
        // let cutted = index.is_polygon_cutted_by(p1, p2);
        // dbg!(cutted);
        // index.split_poly_by_plane(p1, &pl).unwrap();
    }
}
