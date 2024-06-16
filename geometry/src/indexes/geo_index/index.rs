use std::collections::BTreeMap;
use std::{
    borrow::Cow,
    collections::{HashMap, HashSet, VecDeque},
    fmt::Debug,
    hash::Hash,
};

use itertools::Itertools;
use nalgebra::{ComplexField, Vector3};
use num_traits::{One, Signed, Zero};
use rstar::RTree;
use rust_decimal_macros::dec;
use stl_io::Triangle;

use crate::{
    decimal::Dec,
    indexes::{
        aabb::Aabb,
        geo_index::{poly::PolyRef, seg::SegRef},
        polygon_oriented_bb::PolygonOrientedBb,
        vertex_index::{PtId, VertexIndex, MAX_DIGITS},
    },
    linear::segment::Segment,
    planar::{plane::Plane, polygon::Polygon},
    primitives_relation::{
        linear_point::PointOnLine, planar::PlanarRelation, point_planar::PointPlanarRelation,
        relation::Relation,
    },
    reversable::Reversable,
};

use super::rib::RibRef;
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
    pub(super) polygons: HashMap<PolyId, Poly>,
    // pub(super) mesh_bounds: HashMap<MeshBoundId, MeshBound>,
    pub(super) meshes: HashMap<MeshId, Mesh>,
    pub(super) pt_to_ribs: HashMap<PtId, Vec<RibId>>,
    pub(super) polygon_bounding_boxes: HashMap<PolyId, PolygonOrientedBb>,

    pub(super) rib_to_poly: HashMap<RibId, Vec<PolyId>>,
    pub(super) partially_split_polygons: HashMap<PolyId, Vec<RibId>>,
    pub(super) default_mesh: MeshId,
    rib_counter: usize,
    poly_counter: usize,
    mesh_counter: usize,
}

impl GeoIndex {
    pub fn new() -> Self {
        let default_mesh = MeshId(0);
        let meshes = [(default_mesh, Mesh(Vec::new()))]
            .into_iter()
            .collect::<HashMap<_, _>>();
        Self {
            vertices: Default::default(),
            polygon_index: Default::default(),
            ribs: Default::default(),
            polygons: Default::default(),
            meshes,
            pt_to_ribs: Default::default(),
            polygon_bounding_boxes: Default::default(),
            rib_to_poly: Default::default(),
            partially_split_polygons: Default::default(),
            default_mesh,
            rib_counter: 0,
            poly_counter: 0,
            mesh_counter: 1,
        }
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

    pub fn inspect_common_ribs(&self) {
        let common_ribs = self
            .ribs
            .keys()
            .filter(|rib_id| self.rib_to_poly.get(*rib_id).is_some_and(|ps| ps.len() > 2))
            .collect_vec();

        println!("Found {} common ribs", common_ribs.len());
    }

    pub fn get_mutable_mesh(&mut self, mesh_id: MeshId) -> MeshRefMut {
        MeshRefMut {
            geo_index: self,
            mesh_id,
        }
    }

    fn collect_seg_chains(&self, mut ribs: Vec<RibId>) -> Vec<Vec<Seg>> {
        let mut result = Vec::new();

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
                    let rib = &self.ribs[&rib_id];
                    let to = chain.back().unwrap().to_pt();

                    to == rib.0 || to == rib.1
                }) {
                    //println!("found from back");
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
                    let rib = &self.ribs[&rib_id];
                    let from = chain.front().unwrap().from_pt();

                    from == rib.0 || from == rib.1
                }) {
                    //println!("found from front");
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
                    //println!("============ NONE FOund============");
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

    pub fn split_poly_by_chain(&mut self, chain: Vec<Seg>, poly_id: PolyId) -> [PolyId; 2] {
        let pr = self.load_polygon_ref(poly_id);
        let chain = chain.into_iter().map(|s| s.to_ref(self)).collect_vec();
        /*
        println!(" chain: ");
        for sr in &chain {
            println!("   {:?}: {:?} -> {:?}", sr.rib_id, sr.from_pt(), sr.to_pt());
        }
        */
        let chain_last = chain.last().unwrap().to_pt();
        let chain_first = chain.first().unwrap().from_pt();
        let ribs_to_index = chain.iter().map(|s| s.rib_id).collect_vec();
        let reversed_chain = chain
            .clone()
            .into_iter()
            .map(|sr| sr.flip())
            .rev()
            .collect_vec();

        let from = pr
            .segments_iter()
            .position(|sr| sr.from_pt() == chain_last)
            .unwrap();

        let to = pr
            .segments_iter()
            .position(|sr| sr.to_pt() == chain_first)
            .unwrap();
        let mut segments = self.polygons[&poly_id].segments.clone();

        segments.rotate_left(from);
        let to = if from > to {
            to + segments.len() - from
        } else {
            to - from
        } + 1;
        let (fronts, backs) = segments.split_at(to);

        let fronts = [fronts, &chain.into_iter().map(|s| s.seg()).collect_vec()].concat();
        let backs = [
            backs,
            &reversed_chain.into_iter().map(|s| s.seg()).collect_vec(),
        ]
        .concat();

        if fronts.len() < 3 || backs.len() < 3 {
            panic!("Less than 3 segments per polygon is not possible");
        }

        let front_points = fronts
            .iter()
            .map(|s| s.from(&self.ribs))
            .map(|pt| self.vertices.get_point(pt))
            .collect_vec();

        let back_points = backs
            .iter()
            .map(|s| s.from(&self.ribs))
            .map(|pt| self.vertices.get_point(pt))
            .collect_vec();

        let front_aabb = Aabb::from_points(&front_points);
        let back_aabb = Aabb::from_points(&back_points);

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

        let poly_one = self.insert_poly(poly_one);
        let poly_two = self.insert_poly(poly_two);
        for r in ribs_to_index {
            Self::save_index(&mut self.rib_to_poly, r, poly_one);
            Self::save_index(&mut self.rib_to_poly, r, poly_two);
        }

        self.replace_polygons_in_meshes(poly_id, &[poly_one, poly_two]);
        self.remove_polygon(poly_id);

        [poly_one, poly_two]
    }

    pub fn split_polygons_by_orphan_ribs(&mut self) {
        for (poly_id, ribs) in self
            .partially_split_polygons
            .clone()
            .into_iter()
            .sorted_by_key(|v| v.0)
        {
            let mut chains = self.collect_seg_chains(ribs.clone());
            let mut checked_polygons = [poly_id].into_iter().collect::<HashSet<_>>();
            let polygon_points = self.get_polygon_points(poly_id);
            while let Some(chain) = chains.pop() {
                if chain.first().expect("must be").from(&self.ribs)
                    == chain.last().expect("must be").to(&self.ribs)
                {
                    unimplemented!("implement locked cutting chains");
                } else if chain
                    .iter()
                    .skip(1)
                    .map(|seg| seg.from(&self.ribs))
                    .collect::<HashSet<_>>()
                    .intersection(&polygon_points)
                    .count()
                    > 0
                {
                    unimplemented!("implement when chain touches polygon in center");
                } else {
                    for poly_id in checked_polygons.clone() {
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
                        let rev_chain = chain.iter().map(|s| s.clone().flip()).rev().collect_vec();

                        if !(pr.has_chain(&chain) || pr.has_chain(&rev_chain)) {
                            if chain_start_on_poly && chain_ends_on_poly {
                                let [front, back] = self.split_poly_by_chain(chain, poly_id);

                                checked_polygons.remove(&poly_id);
                                checked_polygons.insert(front);
                                checked_polygons.insert(back);
                                break;
                            }
                        }
                    }
                }
            }
        }
        self.partially_split_polygons
            .retain(|poly_id, _| self.polygons.contains_key(poly_id));
    }

    pub(crate) fn mark_polygons_around_common_chain(
        &self,
        chain: Vec<Seg>,
        src_mesh_id: MeshId,
        tool_mesh_id: MeshId,
    ) -> BTreeMap<PolyId, PolygonRelation> {
        let mut visited = BTreeMap::new();
        let ribs_in_chain = chain.clone().into_iter().map(|s| s.rib_id).collect_vec();

        for seg in chain {
            let polygons = self.rib_to_poly[&seg.rib_id]
                .iter()
                .map(|p| (*p, self.get_mesh_for_polygon(*p)))
                .collect::<HashMap<_, _>>();

            for (&polygon_id, mesh_id) in polygons.iter() {
                let cutting_ribs = self.polygons[&polygon_id]
                    .segments
                    .iter()
                    .filter(|s| self.rib_to_poly[&s.rib_id].len() == 4)
                    .filter(|s| ribs_in_chain.contains(&s.rib_id))
                    .map(|s| s.rib_id)
                    .collect_vec();

                let rib_points = cutting_ribs
                    .iter()
                    .flat_map(|s| [self.ribs[&s].0, self.ribs[&s].1])
                    .collect::<HashSet<_>>();

                let checking_planes = cutting_ribs
                    .iter()
                    .filter_map(|rib_id| {
                        self.rib_to_poly[&rib_id]
                            .iter()
                            .find(|poly_id| self.get_mesh_for_polygon(**poly_id) != *mesh_id)
                    })
                    .map(|poly_id| self.polygons[&poly_id].plane.clone())
                    .collect::<Vec<_>>();

                let points = self.get_polygon_points(polygon_id);

                let is_outside = points.difference(&rib_points).all(|pt| {
                    let v = self.vertices.get_point(*pt);

                    checking_planes.iter().any(|plane| {
                        let distance = plane.normal().dot(&v) - plane.d();
                        distance.is_positive()
                    })
                });

                if is_outside && *mesh_id == src_mesh_id {
                    visited.insert(polygon_id, PolygonRelation::SrcPolygonFrontOfTool);
                } else if !is_outside && *mesh_id == src_mesh_id {
                    visited.insert(polygon_id, PolygonRelation::SrcPolygonBackOfTool);
                } else if is_outside && *mesh_id == tool_mesh_id {
                    visited.insert(polygon_id, PolygonRelation::ToolPolygonFrontOfSrc);
                } else if !is_outside && *mesh_id == tool_mesh_id {
                    visited.insert(polygon_id, PolygonRelation::ToolPolygonBackOfSrc);
                } else {
                    panic!("Well, unexpected");
                }
            }
        }
        visited
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
        let v = self.vertices.get_point(pt);
        self.split_rib_in_poly(v, rib_id, poly_id)
    }

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

            for r in &replacement {
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

    pub fn find_first_intersection(&self, mesh_id: &MeshId) -> Option<(PolyId, PolyId)> {
        let tool_polygons = self.get_mesh_polygon_ids(mesh_id).collect_vec();

        for tool_id in &tool_polygons {
            let poly_aabb = self.get_poly_aabb(tool_id);

            if let Some(poly) = self
                .polygon_index
                .locate_in_envelope_intersecting(&poly_aabb.into())
                .map(|o| o.0)
                .filter(|p| !tool_polygons.contains(p))
                .filter(|p| !self.have_common_rib(p, tool_id))
                .find(|other_id| self.is_polygon_cutted_by(*other_id, *tool_id))
            {
                return Some((poly, *tool_id));
            }
        }

        None
    }

    pub fn find_first_intersection_within_list(
        &self,
        polygons: Vec<PolyId>,
    ) -> Option<(PolyId, PolyId)> {
        for tool_id in &polygons {
            let poly_aabb = self.get_poly_aabb(tool_id);

            if let Some(poly) = self
                .polygon_index
                .locate_in_envelope_intersecting(&poly_aabb.into())
                .map(|o| o.0)
                .filter(|p| p != tool_id)
                .filter(|p| !self.have_common_rib(p, tool_id))
                .find(|other_id| self.is_polygon_cutted_by(*other_id, *tool_id))
            {
                return Some((poly, *tool_id));
            }
        }

        None
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

    pub fn save_polygon(&mut self, polygon: &Polygon, mesh_id: Option<MeshId>) {
        let aabb = Aabb::from_points(&polygon.vertices);
        let mesh_id = mesh_id.unwrap_or(self.default_mesh);

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

        let poly_id = self.insert_poly(poly);

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

        let mut found_for_from = false;

        if let Some(ribs) = self.pt_to_ribs.get(&from).cloned() {
            for r in &ribs {
                let rr = self.get_rib(*r);
                match rr.relate(&to) {
                    PointOnLine::On => {
                        found_for_from = true;
                        for ps_for_rib in self.rib_to_poly.remove(r).into_iter().flatten() {
                            // println!("cut {r:?} in {ps_for_rib:?}");

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
                        if found_for_from {
                            dbg!("Found point on for `to`, after finding for `from`");
                        }
                        for ps_for_rib in self.rib_to_poly.remove(r).into_iter().flatten() {
                            println!("cut {r:?} in {ps_for_rib:?}");
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

    fn get_poly_aabb(&self, poly_id: &PolyId) -> Aabb {
        self.polygons[poly_id].aabb
    }

    pub(crate) fn is_polygon_cutted_by(&self, poly_id: PolyId, tool_id: PolyId) -> bool {
        let tool = self.load_polygon_ref(tool_id);
        let tool_plane = tool.get_plane();

        // Firstly, detect, that there are points in `poly_id`, that lay on different sides of
        // polygon plane.
        // Then, detect, that intersection line of plane lays on both polygons

        self.is_polygon_vertices_on_different_side_of_plane(poly_id, &tool_plane)
            && self.is_polygon_intersection_segments_overlap(poly_id, tool_id)
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
            //println!("@NEW RIB  {rib_id:?} ");
            self.ribs.insert(rib_id, rib);
            (rib_id, true)
        }
    }

    fn insert_poly(&mut self, poly: Poly) -> PolyId {
        if let Some(poly_id) = self
            .polygons
            .iter()
            .find(|&(_, s)| *s == poly)
            .map(|(id, _)| id)
        {
            *poly_id
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

            // self.polygon_source.insert(poly_id, PolygonSource::Outside);
            poly_id
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
        self.vertices.get_vertex_index(pt)
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
    pub(crate) fn remove_polygon(&mut self, poly_id: PolyId) {
        if let Some(poly) = self.polygons.remove(&poly_id) {
            // println!("X REMOVE POLY {poly_id:?}");
            let ribs = poly.segments.iter().map(|s| s.rib_id).collect_vec();
            let _points = ribs
                .iter()
                .flat_map(|s| {
                    [
                        self.ribs.get(s).cloned().map(|r| r.0),
                        self.ribs.get(s).cloned().map(|r| r.1),
                    ]
                })
                .collect_vec();

            for r in ribs {
                Self::remove_item_from_index(&mut self.rib_to_poly, &r, &poly_id);
            }

            self.polygon_index
                .remove(&PolyRtreeRecord(poly_id, poly.aabb));
            self.polygon_bounding_boxes.remove(&poly_id);
        }
    }

    /*
        /// Removes this mesh bound, also check, if this mesh_bound contained in mesh.
        /// and if it is, remove mesh
        fn remove_mesh_bound(&mut self, mesh_bound_id: &MeshBoundId) {
            if let Some(mb) = self.mesh_bounds.remove(mesh_bound_id) {
                for r in mb
                    .polies()
                    .flat_map(|p| self.get_polygon_ribs(&p))
                    .collect::<HashSet<_>>()
                {
                    Self::remove_item_from_index(&mut self.rib_to_mesh_bound, &r, mesh_bound_id);
                    /*
                    if self.rib_to_mesh_bound[&r].len() < 2 {
                        println!(
                            " This rib ({r:?})does not connect mesh bounds {}",
                            self.rib_to_mesh_bound[&r].len()
                        );
                    }
                        */
                }
                for m in self
                    .meshes
                    .iter_mut()
                    .filter(|m| m.1 .0.contains(mesh_bound_id))
                {
                    //println!("remove bound {:?} from {:?}", mesh_bound_id, m.0);
                    m.1 .0.retain(|mb| mb != mesh_bound_id);
                }
            }
        }

    */
    /// Return all polygons, belonging to given mesh
    pub(crate) fn get_mesh_polygon_ids(
        &self,
        tool_mesh_id: &MeshId,
    ) -> impl Iterator<Item = PolyId> + '_ {
        self.meshes
            .get(tool_mesh_id)
            .into_iter()
            .flat_map(|mesh| &mesh.0)
            .copied()
    }

    /*
    /// Load mesh in the same way, we as load polygon
    pub(crate) fn load_mesh(&self, mesh_id: MeshId) -> crate::volume::mesh::Mesh {
        let mesh = &self.meshes[&mesh_id];
        let bounds = mesh
            .0
            .iter()
            .map(|ix| self.load_mesh_bound(ix))
            .collect_vec();
        crate::volume::mesh::Mesh::new(bounds)
    }
    */

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

    /*
    fn load_mesh_bound(&self, ix: &MeshBoundId) -> Edge {
        let mesh_bound = &self.mesh_bounds[&ix];
        let bound = self.load_polygon(mesh_bound.bound);
        let holes = mesh_bound
            .holes
            .iter()
            .map(|hole| self.load_polygon(*hole))
            .collect();
        Edge { bound, holes }
    }
    */

    pub(crate) fn drop_polygons(&mut self, polygon_ids: impl Iterator<Item = PolyId>) {
        for p in polygon_ids {
            self.remove_polygon(p);
        }
    }

    /*
    pub fn collect_meshes(&mut self) -> Vec<MeshRef<'_>> {
        // let checkpoint = std::time::Instant::now();
        self.meshes.clear();
        // let checkpoint = print_time("  update index and clean", checkpoint);

        while let Some(m) = self.get_closed_mesh() {
            self.insert_mesh(m);
        }

        // let checkpoint = print_time("  collected all meshes", checkpoint);
        let meshes = self
            .meshes
            .keys()
            .map(|m| MeshRef {
                mesh_id: *m,
                geo_index: self,
            })
            .collect();
        // print_time("  loaded meshes", checkpoint);
        meshes
    }
    */

    /*
    pub(crate) fn mesh_bounds(&self, mesh_id: &MeshId) -> impl Iterator<Item = Cow<'_, Edge>> {
        self.meshes
            .get(mesh_id)
            .into_iter()
            .flat_map(|m| &m.0)
            .map(|mb_id| Cow::Owned(self.load_mesh_bound(mb_id)))
    }
    */

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

    /*
    fn mesh_bound_ribs(&self, mesh_bound_id: &MeshBoundId) -> HashSet<RibId> {
        self.mesh_bounds
            .get(mesh_bound_id)
            .into_iter()
            .flat_map(|mb| mb.polies())
            .filter_map(|p| self.polygons.get(&p))
            .flat_map(|s| &s.segments)
            .map(|s| s.rib_id)
            .collect()
    }
    */

    pub(super) fn get_polygon_ribs(&self, poly_id: &PolyId) -> HashSet<RibId> {
        self.polygons
            .get(poly_id)
            .into_iter()
            .flat_map(|p| &p.segments)
            .map(|s| s.rib_id)
            .collect()
    }

    fn have_common_rib(&self, p: &PolyId, poly_id: &PolyId) -> bool {
        let ribs = self.get_polygon_ribs(p);
        ribs.intersection(&self.get_polygon_ribs(poly_id)).count() > 0
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
                    if [2608, 2600, 2623, 2636, 2649, 2662, 2673, 2686, 2708]
                        .contains(&new_polygon_id.0)
                    {
                        println!("replace in {mesh_id:?} {poly_id:?} > {new_polygon_id:?}");
                    }
                    mesh.0[polygon_index] = *new_polygon_id;
                }
            } else if let Some(m) = self.meshes.get_mut(&mesh_id) {
                if [2608, 2600, 2623, 2636, 2649, 2662, 2673, 2686, 2708]
                    .contains(&new_polygon_id.0)
                {
                    println!("add in {mesh_id:?} > {new_polygon_id:?}");
                }
                m.0.push(*new_polygon_id);
            }
        }
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

    pub fn find_orphan_ribs(&self) -> HashSet<RibId> {
        self.rib_to_poly
            .iter()
            .filter(|(_, r)| r.len() < 2)
            .filter(|(id, r)| {
                self.polygons[&r[0]]
                    .segments
                    .iter()
                    .filter(|s| s.rib_id == **id)
                    .count()
                    == 1
            })
            .inspect(|(id, r)| {
                if r.is_empty() {
                    println!("{:?}", id);
                }
            })
            .map(|(id, _)| *id)
            .collect()
    }

    fn is_polygon_intersection_segments_overlap(&self, poly_id: PolyId, tool_id: PolyId) -> bool {
        let my_poly = self.load_polygon_ref(poly_id);
        let my_plane = my_poly.get_plane();
        let tool_poly = self.load_polygon_ref(tool_id);
        let tool_plane = tool_poly.get_plane();
        match my_plane.relate(&tool_plane) {
            PlanarRelation::Intersect(line) => {
                let my_proj = my_poly
                    .get_segments()
                    .filter_map(|s| line.get_intersection_params_seg_ref(&s))
                    .filter_map(|(l, ss)| {
                        if ss >= Zero::zero() && ss <= One::one() {
                            Some(line.origin + line.dir * l)
                        } else {
                            None
                        }
                    })
                    .map(|v| (v + line.origin).dot(&line.dir).round_dp(MAX_DIGITS as u32))
                    .sorted()
                    .dedup()
                    .collect_vec();

                let tool_proj = tool_poly
                    .get_segments()
                    /*
                    .inspect(|s| {
                        dbg!(s.dir().magnitude());
                    })
                    */
                    .filter_map(|s| line.get_intersection_params_seg_ref(&s))
                    .filter_map(|(l, ss)| {
                        if ss >= Zero::zero() && ss <= One::one() {
                            Some(line.origin + line.dir * l)
                        } else {
                            //dbg!(ss);
                            None
                        }
                    })
                    .map(|v| (v + line.origin).dot(&line.dir).round_dp(MAX_DIGITS as u32))
                    .sorted()
                    .dedup()
                    .collect_vec();

                let btw = |seg_my: &(Dec, Dec), item: Dec| item > seg_my.0 && item < seg_my.1;

                let overlaps = |seg_my: &(Dec, Dec), seg_other: &(Dec, Dec)| {
                    btw(seg_my, seg_other.0) || btw(seg_my, seg_other.1)
                };

                let equals = |seg_my: &(Dec, Dec), seg_other: &(Dec, Dec)| {
                    seg_my.0 == seg_other.0 && seg_my.1 == seg_other.1
                };

                //let segments_me = collapse(segments_me);
                //let segments_other = collapse(segments_other);
                /*
                for s in &segments_me.chunks(2) {
                    println!("{} - {}", s.0, s.1);
                }

                for s in &segments_other.chunks(2).map(|c| {}) {
                    println!("{} - {}", s.0, s.1);
                }
                */

                for seg_my in &my_proj.into_iter().chunks(2) {
                    if let Some(s) = seg_my.collect_tuple() {
                        for seg_other in &tool_proj.clone().into_iter().chunks(2) {
                            if let Some(o) = seg_other.collect_tuple() {
                                if equals(&s, &o) || overlaps(&s, &o) || overlaps(&o, &s) {
                                    return true;
                                }
                            }
                        }
                    }
                }

                /*
                let line_intersects_me = line.relate(&my_poly);
                let line_intersects_other = line.relate(&tool_poly);

                let mut total_intersections_me: Vec<Vector3<Dec>> = Vec::new();
                let mut total_intersections_other: Vec<Vector3<Dec>> = Vec::new();

                if let LinearPolygonRefRelation::IntersectInPlane {
                    vertices,
                    ribs,
                    common_ribs,
                } = line_intersects_me
                {
                    dbg!(&vertices);
                    dbg!(&ribs);
                    dbg!(&common_ribs);
                    total_intersections_me
                        .extend(vertices.iter().map(|pt| self.vertices.get_point(*pt)));
                    total_intersections_me.extend(ribs.into_iter().map(|e| e.1));
                    total_intersections_me.extend(
                        common_ribs
                            .into_iter()
                            .map(|r| self.get_rib(r))
                            .flat_map(|e| [e.from(), e.to()]),
                    );
                }
                if let LinearPolygonRefRelation::IntersectInPlane {
                    vertices,
                    ribs,
                    common_ribs,
                } = line_intersects_other
                {
                    dbg!(&vertices);
                    dbg!(&ribs);
                    dbg!(&common_ribs);
                    for v in &vertices {
                        dbg!(&self.pt_to_ribs[&v]);
                    }
                    total_intersections_other
                        .extend(vertices.iter().map(|pt| self.vertices.get_point(*pt)));
                    total_intersections_other.extend(ribs.into_iter().map(|e| e.1));
                    total_intersections_other.extend(
                        common_ribs
                            .into_iter()
                            .map(|r| self.get_rib(r))
                            .flat_map(|e| [e.from(), e.to()]),
                    );
                }
                let mut segments_me = Vec::new();
                dbg!(&total_intersections_me);
                dbg!(&total_intersections_other);
                for mut chunk in &total_intersections_me
                    .into_iter()
                    .map(|p| {
                        (p - line.origin)
                            .dot(&line.dir)
                            .round_dp(NORMAL_DOT_ROUNDING)
                    })
                    .sorted()
                    .chunks(2)
                {
                    if let Some(item) = chunk
                        .next()
                        .and_then(|one| chunk.next().map(|two| (one, two)))
                    {
                        segments_me.push(item);
                    }
                }

                let mut segments_other = Vec::new();
                for mut chunk in &total_intersections_other
                    .into_iter()
                    .map(|p| {
                        (p - line.origin)
                            .dot(&line.dir)
                            .round_dp(NORMAL_DOT_ROUNDING)
                    })
                    .sorted()
                    .chunks(2)
                {
                    if let Some(item) = chunk
                        .next()
                        .and_then(|one| chunk.next().map(|two| (one, two)))
                    {
                        segments_other.push(item);
                    }
                }

                let btw = |seg_my: &(Dec, Dec), item: Dec| item > seg_my.0 && item < seg_my.1;

                let overlaps = |seg_my: &(Dec, Dec), seg_other: &(Dec, Dec)| {
                    btw(seg_my, seg_other.0) || btw(seg_my, seg_other.1)
                };

                let equals = |seg_my: &(Dec, Dec), seg_other: &(Dec, Dec)| {
                    seg_my.0 == seg_other.0 && seg_my.1 == seg_other.1
                };

                let segments_me = collapse(segments_me);
                let segments_other = collapse(segments_other);
                for s in &segments_me {
                    println!("{} - {}", s.0, s.1);
                }

                for s in &segments_other {
                    println!("{} - {}", s.0, s.1);
                }

                for seg_my in &segments_me {
                    for seg_other in &segments_other {
                        if equals(seg_my, seg_other)
                            || overlaps(seg_my, seg_other)
                            || overlaps(seg_other, seg_my)
                        {
                            return true;
                        }
                    }
                }
                */

                false
            }
            PlanarRelation::Coplanar => false,
            PlanarRelation::Opposite => false,
            PlanarRelation::Parallel => false,
        }
    }

    fn is_polygon_vertices_on_different_side_of_plane(
        &self,
        poly_id: PolyId,
        other_plane: &Plane,
    ) -> bool {
        let mut has_fronts = false;
        let mut has_backs = false;
        for pt in self.get_polygon_points(poly_id) {
            let vec = self.vertices.get_point(pt);
            match other_plane.relate(&vec) {
                PointPlanarRelation::In => {}
                PointPlanarRelation::WithNormal => has_fronts = true,
                PointPlanarRelation::OpposeToNormal => has_backs = true,
            }
        }

        has_fronts && has_backs
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

    /*
        fn join_polygons_on_rib(&mut self, rib_id: RibId) -> bool {
            if self.rib_to_poly[&rib_id].len() > 2 {
                return false;
            }

            let poly1_id = self.rib_to_poly[&rib_id][0];
            let poly2_id = self.rib_to_poly[&rib_id][1];

            let poly1 = &self.polygons[&poly1_id];
            let poly2 = &self.polygons[&poly2_id];

            if poly1.plane != poly2.plane {
                return false;
            }

            let poly1_mesh = self.get_mesh_for_polygon(poly1_id);
            let poly2_mesh = self.get_mesh_for_polygon(poly2_id);

            if (poly1_mesh != poly2_mesh) || poly1_mesh.is_none() {
                return false;
            }
            let poly1_ribs = poly1
                .segments
                .iter()
                .map(|s| s.rib_id)
                .collect::<HashSet<_>>();
            let poly2_ribs = poly1
                .segments
                .iter()
                .map(|s| s.rib_id)
                .collect::<HashSet<_>>();
            let union_len = poly1_ribs.union(&poly2_ribs).collect_vec();
            if union_len.len() > 1 {
                return false;
            }

            let poly_mesh_id = poly1_mesh.unwrap();

            if let Some(ix1) = poly1.segments.iter().position(|s| s.rib_id == rib_id) {
                if let Some(ix2) = poly2.segments.iter().position(|s| s.rib_id == rib_id) {
                    let mut segments_from2 = poly2.segments.clone();
                    segments_from2.rotate_left(ix2);
                    let mut segments_from1 = poly1.segments.clone();

                    segments_from1.splice(ix1..ix1 + 1, segments_from2.into_iter().skip(1));
                    assert!(!segments_from1.iter().any(|s| s.rib_id == rib_id));
                    let new_poly = Poly {
                        segments: segments_from1,
                        aabb: poly1.aabb.merge(poly2.aabb),
                        plane: poly1.plane.to_owned(),
                    };

                    let segs = new_poly.segments.len();
                    for ix in 0..segs - 2 {
                        let iy = ix + 1;
                        assert_eq!(
                            new_poly.segments[ix].to(&self.ribs),
                            new_poly.segments[iy].from(&self.ribs)
                        );
                    }

                    let new_poly_id = self.insert_poly(new_poly);
                    if let Some(mesh) = self.meshes.get_mut(&poly_mesh_id) {
                        //dbg!(poly1_id, poly2_id, new_poly_id);
                        mesh.0.push(new_poly_id);
                        mesh.0.retain(|&p| !(p == poly1_id || p == poly2_id));
                        //dbg!(&mesh.0);

                        //dbg!(mesh.0.iter().any(|&p| p == new_poly_id));
                        assert!(!mesh.0.iter().any(|&p| p == poly1_id || p == poly2_id));
                        //dbg!(poly1_id, poly2_id);
                        self.remove_polygon(poly1_id);
                        self.remove_polygon(poly2_id);
                        self.remove_rib(rib_id);
                        return true;
                    }
                }
            }
            false
        }
    */

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

    pub fn find_polygons_with_orphans(&self) -> Vec<PolyRef<'_>> {
        self.find_orphan_ribs()
            .into_iter()
            .map(|r| {
                let p = self.rib_to_poly[&r][0];
                PolyRef {
                    index: self,
                    poly_id: p,
                }
            })
            .collect()
    }

    pub fn flip_polygon(&mut self, poly_id: PolyId) {
        for p in self.polygons.get_mut(&poly_id).into_iter() {
            for s in p.segments.iter_mut() {
                *s = s.flip();
            }
            p.plane = p.plane.clone().flip();
        }
    }

    pub fn flip_polygons_with_orphans(&mut self) {
        for poly_id in self
            .find_orphan_ribs()
            .into_iter()
            .flat_map(|r| &self.rib_to_poly[&r])
            .map(|p| *p)
            .collect::<HashSet<_>>()
        {
            //println!(" Flip poly: {poly_id:?}");
            self.flip_polygon(poly_id);
        }
    }

    pub fn flip_intersected_polygons(&mut self) {
        for poly_id in self
            .rib_to_poly
            .iter()
            .filter(|(_, ps)| ps.len() == 4)
            .flat_map(|(_, ps)| ps)
            .map(|p| *p)
            .collect::<HashSet<_>>()
        {
            //println!(" Flip poly: {poly_id:?}");
            self.flip_polygon(poly_id);
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

    fn create_common_ribs(&mut self, tool_id: PolyId, mesh_id: MeshId) {
        let tool_aabb = self.polygons[&tool_id].aabb;
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
        for src_id in polygons.iter() {
            let src_plane = self.load_polygon_ref(*src_id).get_plane();
            let common_line = match tool_plane.relate(&src_plane) {
                PlanarRelation::Intersect(line) => line,
                _ => panic!("polygon intersection line not exists"),
            };

            let mut pts_src = self
                .get_polygon_points(*src_id)
                .into_iter()
                .filter(|p| {
                    common_line.distance_to_pt_squared(self.vertices.get_point(*p))
                        < Dec::from(0.0001)
                })
                .collect_vec();

            let mut pts_tool = self
                .get_polygon_points(tool_id)
                .into_iter()
                .filter(|p| {
                    common_line.distance_to_pt_squared(self.vertices.get_point(*p))
                        < Dec::from(0.0001)
                })
                .collect_vec();

            //dbg!(&pts_src);
            //dbg!(&pts_tool);
            let mut vertices_src = Vec::new();
            let mut vertices_tool = Vec::new();
            //dbg!(&pts_src);
            //dbg!(&pts_tool);

            for seg in self.load_polygon_ref(*src_id).segments_iter() {
                if let Some((a, b)) = common_line.get_intersection_params_seg_ref(&seg) {
                    if b > Zero::zero() && b < One::one() {
                        //println!(" S---> {:?} ({})", seg.rib_id, b,);
                        let value = (common_line.origin + common_line.dir * a, seg.rib_id);
                        if !pts_src.iter().any(|pt| {
                            let v = self.vertices.get_point(*pt);
                            let mag = (value.0 - v).magnitude_squared();
                            if *src_id == 1008 {
                                println!(
                                    " distance to existing point SRC {} {} {} : {}",
                                    value.0.x.round_dp(6),
                                    value.0.y.round_dp(6),
                                    value.0.z.round_dp(6),
                                    mag.round_dp(6)
                                );
                            }
                            mag < Dec::from(dec!(0.001))
                        }) {
                            //println!(" seems, like we look at same spot");
                            vertices_src.push(value);
                        }
                    }
                }
            }

            for seg in self.load_polygon_ref(tool_id).segments_iter() {
                if let Some((a, b)) = common_line.get_intersection_params_seg_ref(&seg) {
                    if b > Zero::zero() && b < One::one() {
                        let value = (common_line.origin + common_line.dir * a, seg.rib_id);
                        if !pts_tool.iter().any(|pt| {
                            let v = self.vertices.get_point(*pt);
                            let mag = (value.0 - v).magnitude_squared();
                            if *src_id == 1008 {
                                println!(
                                    " distance to existing point TOOL {} {} {} : {}",
                                    value.0.x.round_dp(6),
                                    value.0.y.round_dp(6),
                                    value.0.z.round_dp(6),
                                    mag.round_dp(6)
                                );
                            }
                            mag < Dec::from(dec!(0.001))
                        }) {
                            vertices_tool.push(value);
                        }
                    }
                }
            }

            if *src_id == 1008 {
                for (v, _) in &vertices_src {
                    println!(
                        "  Vs: {} {} {}",
                        v.x.round_dp(6),
                        v.y.round_dp(6),
                        v.z.round_dp(6)
                    );
                }
                for (v, _) in &vertices_tool {
                    println!(
                        "  Vt: {} {} {}",
                        v.x.round_dp(6),
                        v.y.round_dp(6),
                        v.z.round_dp(6)
                    );
                }
            }

            let mut cut_rib_is_case = HashMap::new();
            let vertices_src = vertices_src
                .into_iter()
                .map(|(v, rib_id)| {
                    let pt = self.vertices.get_vertex_index(v);
                    (pt, rib_id)
                })
                .filter(|(pt, _)| !pts_src.contains(pt))
                .map(|(pt, rib_id)| {
                    cut_rib_is_case.insert(pt, rib_id);
                    pt
                })
                .collect_vec();

            let vertices_tool = vertices_tool
                .into_iter()
                .map(|(v, rib_id)| {
                    let pt = self.vertices.get_vertex_index(v);
                    (pt, rib_id)
                })
                .filter(|(pt, _)| !pts_tool.contains(pt))
                .map(|(pt, rib_id)| {
                    cut_rib_is_case.insert(pt, rib_id);
                    pt
                })
                .collect_vec();

            pts_src.extend(vertices_src);
            pts_tool.extend(vertices_tool);

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

            for c in pts_src.chunks(2) {
                if let Some(inner_segment) = <&[PtId] as TryInto<[PtId; 2]>>::try_into(c).ok() {
                    segs_src.push(inner_segment);
                }
            }
            for c in pts_tool.chunks(2) {
                if let Some(inner_segment) = <&[PtId] as TryInto<[PtId; 2]>>::try_into(c).ok() {
                    segs_tool.push(inner_segment);
                }
            }

            let between = |a: usize, b: usize, c: usize| (c > a && c < b) || (c > b && c < a);
            let intersection = |a: [PtId; 2], b: [PtId; 2]| {
                let common = a
                    .clone()
                    .into_iter()
                    .chain(b.clone())
                    .sorted_by_key(|pt| {
                        (self.vertices.get_point(*pt) - common_line.origin).dot(&common_line.dir)
                    })
                    .dedup()
                    .collect_vec();
                let a0 = common.iter().position(|&i| i == a[0]).expect("ok");
                let a1 = common.iter().position(|&i| i == a[1]).expect("ok");
                let b0 = common.iter().position(|&i| i == b[0]).expect("ok");
                let b1 = common.iter().position(|&i| i == b[1]).expect("ok");

                if common.len() < 4 {
                    panic!("AAAAA {:?}", common);
                }
                if between(a0, a1, b0)
                    || between(a0, a1, b1)
                    || between(b0, b1, a0)
                    || between(b0, b1, a1)
                {
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

            for rib in new_ribs {
                let (r, _) = self.insert_rib(rib);
                Self::save_index(&mut self.partially_split_polygons, tool_id, r);
                Self::save_index(&mut self.partially_split_polygons, *src_id, r);
                Self::save_index(&mut self.pt_to_ribs, rib.0, r);
                Self::save_index(&mut self.pt_to_ribs, rib.1, r);

                for r in [rib.0, rib.1] {
                    if let Some(poly_rib) = cut_rib_is_case.remove(&r) {
                        for poly_id in self.rib_to_poly.remove(&poly_rib).into_iter().flatten() {
                            self.split_rib_in_poly_using_indexed_pt(r, poly_rib, poly_id);
                        }
                    }
                }
            }
        }
    }

    fn unify_ribs(&mut self, tool_id: PolyId) {
        let tool_aabb = self.polygons[&tool_id].aabb;
        let polygons = self
            .polygon_index
            .locate_in_envelope_intersecting(&tool_aabb.into())
            .map(|o| o.0)
            .filter(|&p| p != tool_id)
            .collect_vec();

        let mut split_ribs = HashMap::new();
        for p in polygons.iter() {
            for rib in self
                .polygons
                .get(&p)
                .into_iter()
                .flat_map(|p| &p.segments)
                .map(|s| s.rib_id)
            {
                let rr = RibRef {
                    rib_id: rib,
                    index: self,
                };

                let line = crate::linear::line::Line {
                    origin: rr.from(),
                    dir: rr.dir().normalize(),
                };

                if tool_id == 1135 {
                    println!("  == {rib:?} {:?} -- {:?}", rr.from_pt(), rr.to_pt());
                }

                for seg in self.load_polygon_ref(tool_id).segments_iter() {
                    let is_segment_on_line = line.distance_to_pt_squared(seg.from()).abs()
                        < Dec::from(0.0001)
                        && line.distance_to_pt_squared(seg.to()).abs() < Dec::from(0.0001);

                    if tool_id == 1135 {
                        println!(
                            "    ** {:?} {} {} {}",
                            seg.rib_id,
                            line.distance_to_pt_squared(seg.from()).abs().round_dp(4),
                            line.distance_to_pt_squared(seg.to()).abs().round_dp(4),
                            is_segment_on_line
                        );
                    }

                    if is_segment_on_line {
                        let pts = (vec![rr.from_pt(), rr.to_pt(), seg.from_pt(), seg.to_pt()])
                            .into_iter()
                            .sorted_by_key(|pt| {
                                let v = self.vertices.get_point(*pt);
                                (v - line.origin).dot(&line.dir)
                            })
                            .dedup()
                            .collect_vec();

                        if pts.len() > 2 {
                            let rib_start_ix =
                                pts.iter().position(|pt| *pt == rr.from_pt()).unwrap();
                            let rib_end_ix = pts.iter().position(|pt| *pt == rr.to_pt()).unwrap();
                            let seg_start_ix =
                                pts.iter().position(|pt| *pt == seg.from_pt()).unwrap();
                            let seg_end_ix = pts.iter().position(|pt| *pt == seg.to_pt()).unwrap();

                            let between =
                                |a: usize, b: usize, c: usize| (c > a && c < b) || (c > b && c < a);

                            if between(rib_start_ix, rib_end_ix, seg_start_ix) {
                                Self::save_index(&mut split_ribs, rib, pts[seg_start_ix]);
                            } else if between(rib_start_ix, rib_end_ix, seg_end_ix) {
                                Self::save_index(&mut split_ribs, rib, pts[seg_end_ix]);
                            } else if between(seg_start_ix, seg_end_ix, rib_start_ix) {
                                Self::save_index(&mut split_ribs, seg.rib_id, pts[rib_start_ix]);
                            } else if between(seg_start_ix, seg_end_ix, rib_end_ix) {
                                Self::save_index(&mut split_ribs, seg.rib_id, pts[rib_end_ix]);
                            };
                        }
                    }
                }
            }
        }
        for (rib_id, pts) in split_ribs {
            for poly_id in self.rib_to_poly.remove(&rib_id).into_iter().flatten() {
                self.split_rib_in_poly_using_indexed_pts(&pts, rib_id, poly_id);
            }
            self.remove_rib(rib_id);
        }
    }

    pub fn create_default_mesh(&mut self) {
        let mesh_id = self.get_next_mesh_id();
        self.meshes.insert(mesh_id, Mesh(Vec::new()));
        self.default_mesh = mesh_id;
    }

    pub(crate) fn collect_common_chains(&self, mesh_id: MeshId, tool: MeshId) -> Vec<Vec<Seg>> {
        let ribs = self
            .rib_to_poly
            .iter()
            .sorted_by_key(|(r, _)| r.0)
            .filter(|(_, polies)| polies.len() == 4)
            .filter(|(_, polies)| {
                polies
                    .iter()
                    .all(|p| [mesh_id, tool].contains(&self.get_mesh_for_polygon(*p)))
            })
            .map(|(rib_id, _)| *rib_id)
            .collect_vec();

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
