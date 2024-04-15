use std::{
    borrow::Cow,
    collections::{HashMap, HashSet, VecDeque},
    fmt::Debug,
    hash::Hash,
};

use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::{One, Zero};
use rstar::{Point, RTree, RTreeObject, AABB};
use rust_decimal_macros::dec;
use uuid::Uuid;

use crate::{
    decimal::{Dec, NORMAL_DOT_ROUNDING, STABILITY_ROUNDING},
    edge::Edge,
    linear::segment::Segment,
    measure_time::print_time,
    planar::{plane::Plane, polygon::Polygon},
    primitives_relation::{
        linear_planar::LinearPolygonRelation,
        planar::PlanarRelation,
        point_planar::{PointPlanarRelation, PointPolygonRelation},
        point_volume::PointVolumeRelation,
        relation::Relation,
    },
};

use super::{
    aabb::Aabb,
    vertex_index::{PtId, VertexIndex},
};

#[derive(Debug, Default)]
pub struct GeoIndex {
    vertices: VertexIndex,
    polygon_index: RTree<PolyRtreeRecord>,
    ribs: HashMap<RibId, Rib>,
    polygons: HashMap<PolyId, Poly>,
    mesh_bounds: HashMap<MeshBoundId, MeshBound>,
    meshes: HashMap<MeshId, Mesh>,
    pt_to_ribs: HashMap<PtId, Vec<RibId>>,
    pt_to_poly: HashMap<PtId, Vec<PolyId>>,
    rib_to_poly: HashMap<RibId, Vec<PolyId>>,
    rib_to_mesh_bound: HashMap<RibId, Vec<MeshBoundId>>,
    adjacent_ribs: HashMap<RibId, Vec<RibId>>,
}

impl GeoIndex {
    pub(crate) fn find_insides(&self, of_mesh: MeshId, by_mesh: MeshId) -> Vec<PolyId> {
        let marked =
            self.mark_foreign_polygons(of_mesh, by_mesh, |r, p, m| self.is_polygon_inside(r, p, m));

        marked
            .into_iter()
            .flatten()
            .filter(|(_, is_inside)| *is_inside)
            .map(|kv| kv.0)
            .collect()
    }
    pub(crate) fn find_outsides(&self, of_mesh: MeshId, by_mesh: MeshId) -> Vec<PolyId> {
        let marked = self
            .mark_foreign_polygons(of_mesh, by_mesh, |r, p, m| self.is_polygon_outside(r, p, m));

        marked
            .into_iter()
            .flatten()
            .filter(|(_, is_outside)| *is_outside)
            .map(|kv| kv.0)
            .collect()
    }

    fn mark_foreign_polygons<T: Clone + Debug>(
        &self,
        of_mesh: MeshId,
        by_mesh: MeshId,
        marker: impl Fn(RibId, PolyId, MeshId) -> T,
    ) -> Option<HashMap<PolyId, T>> {
        let checkpoint = std::time::Instant::now();
        let mut result = HashMap::new();
        let common_ribs = self
            .get_mesh_ribs(by_mesh)
            .into_iter()
            .filter(|rib_id| self.rib_to_poly.get(rib_id).is_some_and(|ps| ps.len() > 2))
            .collect_vec();
        println!("Found {} common ribs", common_ribs.len());
        let checkpoint = print_time("   mark_foreign_polygons: find common ribs", checkpoint);

        for r in common_ribs {
            for p in &self.rib_to_poly[&r] {
                if !result.contains_key(p) {
                    result.insert(*p, marker(r, *p, by_mesh));
                }
            }
        }
        let checkpoint = print_time(
            "   mark_foreign_polygons: mark polies around ribs",
            checkpoint,
        );

        if result.is_empty() {
            return None;
        }

        let mut mesh_polygons: VecDeque<PolyId> = self.get_mesh_polygon_ids(&of_mesh).collect();
        while let Some(p) = mesh_polygons.pop_front() {
            if result.contains_key(&p) {
                continue;
            }
            if let Some(adjacent) = self
                .get_polygon_ribs(&p)
                .into_iter()
                .flat_map(|r| &self.rib_to_poly[&r])
                .filter(|&cand| *cand != p)
                .find(|poly| result.contains_key(poly))
            {
                result.insert(p, result[&adjacent].clone());
            } else {
                mesh_polygons.push_back(p)
            }
        }

        print_time("   mark_foreign_polygons: fill mesh", checkpoint);
        Some(result)
    }

    fn is_polygon_inside(&self, rib_id: RibId, poly_id: PolyId, mesh_id: MeshId) -> bool {
        //let checkpoint = std::time::Instant::now();
        let test_pt = self.get_pt_in_poly(rib_id, poly_id);
        //let checkpoint = print_time("   IPI: get pt in poly", checkpoint);

        let mesh = self.load_mesh(mesh_id);
        //let checkpoint = print_time("   IPI: load mesh", checkpoint);

        matches!(mesh.relate(&test_pt), PointVolumeRelation::In)
    }

    fn is_polygon_outside(&self, rib_id: RibId, poly_id: PolyId, mesh_id: MeshId) -> bool {
        let test_pt = self.get_pt_in_poly(rib_id, poly_id);

        matches!(
            self.load_mesh(mesh_id).relate(&test_pt),
            PointVolumeRelation::Out
        )
    }

    fn get_pt_in_poly(&self, rib_id: RibId, poly_id: PolyId) -> Vector3<Dec> {
        let rib = self.ribs[&rib_id];
        let u = self.vertices.get_point(rib.0);
        let v = self.vertices.get_point(rib.1);

        let dir = (u - v).normalize();
        let center = u.lerp(&v, dec!(0.5).into());

        let plane = self.get_polygon_plane(poly_id);
        let poly = self.load_polygon(poly_id);
        let in_poly = dir.cross(&plane.normal()).normalize();
        let d: Dec = dec!(0.0001).into();
        let in_poly_pt = center + in_poly * d;
        if matches!(poly.relate(&in_poly_pt), PointPolygonRelation::In) {
            in_poly_pt
        } else {
            center - in_poly * d
        }
    }

    pub(crate) fn inverse_poly(&mut self, poly_id: PolyId) {
        if let Some(poly) = self.polygons.get_mut(&poly_id) {
            for s in poly.segments.iter_mut() {
                *s = s.flip();
            }
            poly.segments.reverse();
        }
    }

    pub(crate) fn inverse_mesh_bound(&mut self, mesh_bound_id: MeshBoundId) {
        if let Some(mb) = self.mesh_bounds.get(&mesh_bound_id).cloned() {
            for hole in mb.holes.iter() {
                self.inverse_poly(*hole)
            }
            self.inverse_poly(mb.bound)
        }
    }

    pub(crate) fn inverse_mesh(&mut self, mesh_id: MeshId) {
        if let Some(mesh) = self.meshes.get(&mesh_id).cloned() {
            for mb in mesh.0 {
                self.inverse_mesh_bound(mb)
            }
        }
    }

    pub fn inspect_common_ribs(&self) {
        let common_ribs = self
            .ribs
            .keys()
            .filter(|rib_id| self.rib_to_poly.get(*rib_id).is_some_and(|ps| ps.len() > 2))
            .collect_vec();

        println!("Found {} common ribs", common_ribs.len());
    }

    pub fn has_common_ribs(&self) -> bool {
        self.ribs
            .keys()
            .any(|rib_id| self.rib_to_poly.get(rib_id).is_some_and(|ps| ps.len() > 2))
    }

    pub fn save_mesh<'i, 'o>(&'i mut self, bounds: impl Iterator<Item = Cow<'o, Edge>>) -> MeshId {
        let bound_ids = bounds
            .map(|edge| {
                let bound = self.save_polygon(&edge.bound);
                let holes = edge
                    .holes
                    .iter()
                    .map(|p| self.save_polygon(p))
                    .collect_vec();
                let mut polygon_ribs: HashSet<RibId> = self.get_polygon_ribs(&bound);
                polygon_ribs.extend(holes.iter().flat_map(|p| self.get_polygon_ribs(p)));
                let mesh_bound_id = self.insert_mesh_bound(MeshBound { bound, holes });
                for cur_rib_id in polygon_ribs {
                    Self::save_index(&mut self.rib_to_mesh_bound, cur_rib_id, mesh_bound_id);
                }

                mesh_bound_id
            })
            .collect();
        self.insert_mesh(Mesh(bound_ids))
    }

    pub(crate) fn split_poly_by_plane(&mut self, poly_id: PolyId, plane: &Plane) {
        self.inspect_ribs();
        let poly = &self.polygons[&poly_id];
        let mut fronts = Vec::new();
        let mut backs = Vec::new();
        let mut plane_intersects = Vec::new();
        let mut starting = None;
        let mut split_ribs = Vec::new();
        let mut ff = HashMap::new();
        let mut bb = HashMap::new();
        for seg in poly.segments.iter() {
            let rib = self.ribs[&seg.rib_id];
            let (from, to) = match seg.dir {
                SegmentDir::Fow => (rib.0, rib.1),
                SegmentDir::Rev => (rib.1, rib.0),
            };

            let pt_from = self.vertices.get_point(from);
            let pt_to = self.vertices.get_point(to);
            let relation_from = plane.relate(&pt_from);
            let relation_to = plane.relate(&pt_to);

            match (&relation_from, &relation_to) {
                (PointPlanarRelation::WithNormal, PointPlanarRelation::OpposeToNormal) => {
                    let d = (pt_to - plane.point_on_plane())
                        .dot(&plane.normal())
                        .round_dp(STABILITY_ROUNDING);
                    let t =
                        (d / (pt_to - pt_from).dot(&plane.normal())).round_dp(STABILITY_ROUNDING);
                    assert!(t >= Dec::zero());
                    assert!(t <= Dec::one());
                    let u = pt_to.lerp(&pt_from, t);
                    let uid = self.vertices.get_vertex_index(u);
                    plane_intersects.push(uid);

                    fronts.push(Segment {
                        from: pt_from,
                        to: u,
                    });
                    backs.push(Segment { from: u, to: pt_to });
                    ff.insert(fronts.len() - 1, "WO");
                    bb.insert(backs.len() - 1, "WO");

                    if starting.is_none() {
                        starting = Some(Starting::AfterFront);
                    }
                    //println!(" WO");
                    //println!("u {} {} {}", u.x, u.y, u.z);
                    //println!("f {} {} {}", pt_from.x, pt_from.y, pt_from.z);
                    //println!("t {} {} {}", pt_to.x, pt_to.y, pt_to.z);
                    //dbg!(&fronts);
                    //dbg!(&backs);
                    split_ribs.push((u, seg.rib_id));
                }
                (PointPlanarRelation::OpposeToNormal, PointPlanarRelation::WithNormal) => {
                    let d = (pt_from - plane.point_on_plane())
                        .dot(&plane.normal())
                        .round_dp(STABILITY_ROUNDING);
                    let t =
                        (d / (pt_from - pt_to).dot(&plane.normal())).round_dp(STABILITY_ROUNDING);
                    assert!(t >= Dec::zero());
                    assert!(t <= Dec::one());
                    let u = pt_from.lerp(&pt_to, t);
                    let uid = self.vertices.get_vertex_index(u);
                    plane_intersects.push(uid);
                    backs.push(Segment {
                        from: pt_from,
                        to: u,
                    });

                    fronts.push(Segment { from: u, to: pt_to });
                    ff.insert(fronts.len() - 1, "OW");
                    bb.insert(backs.len() - 1, "OW");
                    if starting.is_none() {
                        starting = Some(Starting::AfterBack);
                    }

                    //println!(" OW");
                    //println!("u {} {} {}", u.x, u.y, u.z);
                    //println!("f {} {} {}", pt_from.x, pt_from.y, pt_from.z);
                    //println!("t {} {} {}", pt_to.x, pt_to.y, pt_to.z);
                    //dbg!(&fronts);
                    //dbg!(&backs);
                    split_ribs.push((u, seg.rib_id));
                }
                (PointPlanarRelation::WithNormal, PointPlanarRelation::WithNormal) => {
                    //println!(" WW");
                    fronts.push(Segment {
                        from: pt_from,
                        to: pt_to,
                    });
                    ff.insert(fronts.len() - 1, "WW");
                    //dbg!(&fronts);
                    //dbg!(&backs);
                }
                (PointPlanarRelation::OpposeToNormal, PointPlanarRelation::OpposeToNormal) => {
                    //println!(" OO");
                    backs.push(Segment {
                        from: pt_from,
                        to: pt_to,
                    });
                    bb.insert(backs.len() - 1, "OO");
                    //dbg!(&fronts);
                    //dbg!(&backs);
                }
                (PointPlanarRelation::OpposeToNormal, PointPlanarRelation::In) => {
                    //println!(" OI");
                    if starting.is_none() {
                        starting = Some(Starting::AfterBack);
                    }
                    if !plane_intersects.contains(&to) {
                        plane_intersects.push(to);
                    }
                    backs.push(Segment {
                        from: pt_from,
                        to: pt_to,
                    });
                    bb.insert(backs.len() - 1, "OI");
                    //dbg!(&fronts);
                    //dbg!(&backs);
                }
                (PointPlanarRelation::WithNormal, PointPlanarRelation::In) => {
                    //println!(" WI");
                    if starting.is_none() {
                        starting = Some(Starting::AfterFront);
                    }
                    if !plane_intersects.contains(&to) {
                        plane_intersects.push(to);
                    }
                    fronts.push(Segment {
                        from: pt_from,
                        to: pt_to,
                    });
                    ff.insert(fronts.len() - 1, "WI");
                    //dbg!(&fronts);
                    //dbg!(&backs);
                }
                (PointPlanarRelation::In, PointPlanarRelation::OpposeToNormal) => {
                    //println!(" IO");
                    if starting.is_none() {
                        starting = Some(Starting::BeforeBack);
                    }

                    if !plane_intersects.contains(&from) {
                        plane_intersects.push(from);
                    }
                    backs.push(Segment {
                        from: pt_from,
                        to: pt_to,
                    });
                    bb.insert(backs.len() - 1, "IO");
                    //dbg!(&fronts);
                    //dbg!(&backs);
                    // segments are circular, we already putted this point, or will put this point
                    // later
                }
                (PointPlanarRelation::In, PointPlanarRelation::WithNormal) => {
                    //println!(" IW");
                    if starting.is_none() {
                        starting = Some(Starting::BeforeFront);
                    }
                    if !plane_intersects.contains(&from) {
                        plane_intersects.push(from);
                    }
                    fronts.push(Segment {
                        from: pt_from,
                        to: pt_to,
                    });
                    ff.insert(fronts.len() - 1, "IW");
                    //dbg!(&fronts);
                    //dbg!(&backs);
                    // segments are circular, we already putted this point, or will put this point
                    // later
                }
                (PointPlanarRelation::In, _) => {
                    //println!(" I > {:?}", x);
                    fronts.push(Segment {
                        from: pt_from,
                        to: pt_to,
                    });
                    backs.push(Segment {
                        to: pt_from,
                        from: pt_to,
                    });
                    ff.insert(fronts.len() - 1, "II");
                    bb.insert(backs.len() - 1, "II");
                }
            }
        }
        let original_polygon = self.load_polygon(poly_id);
        for (point, rib_id) in &split_ribs {
            for poly_id in self.rib_to_poly.remove(rib_id).into_iter().flatten() {
                self.split_rib_in_poly(*point, *rib_id, poly_id);
            }
            self.rib_to_mesh_bound.remove(rib_id);
            self.ribs.remove(rib_id);
        }

        //dbg!(&plane_intersects);
        let mut plane_intersects = plane_intersects
            .into_iter()
            .map(|v| self.vertices.get_point(v))
            .collect_vec();
        // lets check, of first segment inside polygon
        if plane_intersects.len() > 1 {
            let pt = plane_intersects.first().and_then(|f| {
                plane_intersects
                    .get(1)
                    .map(|t| f.lerp(t, Dec::one() / Dec::from(2)))
            });

            if pt
                .map(|p| original_polygon.relate(&p))
                .is_some_and(|t| matches!(t, PointPolygonRelation::In))
            {
                plane_intersects.rotate_left(1);
            }
            // Now we create segments and ribs in plane
            for chunk in &plane_intersects.clone().into_iter().chunks(2) {
                let maybe_chunk: Result<[Vector3<Dec>; 2], _> =
                    chunk.collect::<Vec<Vector3<Dec>>>().try_into();

                if let Ok([from, to]) = maybe_chunk {
                    let (back, front) = if starting.as_ref().is_some_and(|t| {
                        matches!(t, Starting::AfterFront) || matches!(t, Starting::BeforeBack)
                    }) {
                        (Segment { from, to }, Segment { from: to, to: from })
                    } else {
                        (Segment { from: to, to: from }, Segment { from, to })
                    };
                    //dbg!("===OK===");
                    //dbg!(&fronts);
                    //dbg!(&backs);
                    fronts.push(front);
                    backs.push(back);
                    ff.insert(fronts.len() - 1, "INSIDE");
                    bb.insert(backs.len() - 1, "INSIDE");
                }
            }
        } else {
            panic!("WTF");
        }

        let front_segments = fronts
            .into_iter()
            .map(|front| self.save_segment(front.clone()))
            .collect_vec();
        let front_segments = self.arrange_segments(front_segments);

        let front_points = front_segments
            .iter()
            .map(|s| s.rib_id)
            .filter_map(|r_id| self.ribs.get(&r_id))
            .flat_map(|r| [r.0.to_owned(), r.1.to_owned()])
            .collect::<HashSet<_>>()
            .into_iter()
            .map(|p| self.vertices.get_point(p))
            .collect_vec();
        let back_segments = backs
            .into_iter()
            .map(|front| self.save_segment(front.clone()))
            .collect_vec();
        let back_segments = self.arrange_segments(back_segments);
        let back_points = back_segments
            .iter()
            .map(|s| s.rib_id)
            .filter_map(|r_id| self.ribs.get(&r_id))
            .flat_map(|r| [r.0, r.1])
            .collect::<HashSet<_>>()
            .into_iter()
            .map(|p| self.vertices.get_point(p))
            .collect_vec();
        let front_aabb = Aabb::from_points(&front_points);
        let back_aabb = Aabb::from_points(&back_points);
        let front_polygon = Poly {
            segments: front_segments.clone(),
            aabb: front_aabb,
        };
        let back_polygon = Poly {
            segments: back_segments.clone(),
            aabb: back_aabb,
        };
        // now we have full collection of segments in front of plane and in back

        let front_polygon_id = self.insert_poly(front_polygon);
        let back_polygon_id = self.insert_poly(back_polygon);
        for s in &front_segments {
            Self::save_index(&mut self.rib_to_poly, s.rib_id, front_polygon_id);
        }
        for s in &back_segments {
            Self::save_index(&mut self.rib_to_poly, s.rib_id, back_polygon_id);
        }
        /*
        println!("++++");
        for new_poly in &[front_polygon_id, back_polygon_id] {
            println!("#> poly {:?}", new_poly);
            for segment in &self.polygons[&new_poly].segments {
                let connected_with = self.rib_to_poly[&segment.rib_id]
                    .iter()
                    .filter(|&p| p != new_poly)
                    .collect_vec();
                println!(" ## rib {:?}", segment.rib_id);
                for cw in connected_with {
                    println!("   @@ with poly {:?}", cw);
                }
            }
        }
        */

        self.replace_polygons(poly_id, &[front_polygon_id, back_polygon_id]);

        /*
        println!("++++-");
        println!("++++-~");
        */
        self.remove_polygon(poly_id);
    }

    fn arrange_segments(&self, mut segments: Vec<Seg>) -> Vec<Seg> {
        let mut result = Vec::new();
        let mut last = segments.swap_remove(0);
        result.push(last);

        while let Some(s) = segments
            .iter()
            .position(|s| s.from(&self.ribs) == last.to(&self.ribs))
        {
            last = segments.swap_remove(s);
            result.push(last);
        }
        if !segments.is_empty() {
            panic!("Not all segments taken");
        }

        result
    }

    fn split_rib_in_poly(&mut self, pt: Vector3<Dec>, rib_id: RibId, poly_id: PolyId) {
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

            for r in replacement {
                //println!("save index: {:?}", r.rib_id);
                Self::save_index(&mut self.rib_to_poly, r.rib_id, poly_id);
                for mb_id in self.get_mesh_bounds_for_polygon(poly_id) {
                    Self::save_index(&mut self.rib_to_mesh_bound, r.rib_id, mb_id);
                }
            }
            if let Some(poly) = self.polygons.get_mut(&poly_id) {
                poly.segments.splice(ix..(ix + 1), replacement);
            }
        } else {
            panic!("NO RIB");
        }
    }

    fn get_mesh_bounds_for_polygon(&mut self, poly_id: PolyId) -> Vec<MeshBoundId> {
        self.mesh_bounds
            .iter()
            .filter(|(_, mb)| mb.polies().any(|p| p == poly_id))
            .map(|x| *x.0)
            .collect()
    }

    pub fn find_first_intersection(&self, mesh_id: &MeshId) -> Option<(PolyId, PolyId)> {
        let tool_polygons = self.get_mesh_polygon_ids(mesh_id).collect_vec();
        for tool_id in &tool_polygons {
            //println!("----------{tool_id:?}-------------");
            //dbg!(self.load_polygon(*tool_id));
            let poly_aabb = self.get_poly_aabb(tool_id);
            /*
            let ps = self
                .polygon_index
                .locate_in_envelope_intersecting(&poly_aabb.into())
                .map(|o| o.0)
                .filter(|p| !mesh_polygons.contains(p))
                .map(|p| self.load_polygon(p).get_plane())
                .collect_vec();
            dbg!(ps);
            panic!("ddd");
            */

            if let Some(poly) = self
                .polygon_index
                .locate_in_envelope_intersecting(&poly_aabb.into())
                .map(|o| o.0)
                .filter(|p| !tool_polygons.contains(p))
                .filter(|p| !self.have_common_rib(p, tool_id))
                /*
                .inspect(|p| {
                    let other_plane = self.get_polygon_plane(*p);
                    let point_on_plane = other_plane.point_on_plane();
                    if other_plane.normal() == -Vector3::x()
                        && point_on_plane == Vector3::x() / Dec::from(2)
                    {
                        dbg!(point_on_plane);
                        dbg!(self.load_polygon(*p));
                        dbg!(mesh_polygons.contains(p));
                        println!("~~~~~~~~~~~~~~~~~~~~~~~~");
                    }
                })
                    */
                .find(|other_id| self.is_polygon_cutted_by(*other_id, *tool_id))
            {
                return Some((poly, *tool_id));
            }
        }

        None
    }
    /*
    pub fn find_intersecting_groups(&self, mesh_id: &MeshId) -> HashMap<PolyId, Vec<PolyId>> {
        let mut result = HashMap::new();
        let mesh_polygons = self.get_mesh_polygon_ids(mesh_id).collect_vec();
        for poly_id in &mesh_polygons {
            //println!("----------{poly_id:?}-------------");
            //dbg!(self.load_polygon(*poly_id));
            let poly_aabb = self.get_poly_aabb(poly_id);
            let poly_ids = self
                .polygon_index
                .locate_in_envelope_intersecting(&poly_aabb.into())
                .map(|o| o.0)
                .filter(|p| !mesh_polygons.contains(p))
                .inspect(|p| {
                    println!("~~~");
                })
                .filter(|p| !self.have_common_rib(p, poly_id))
                .filter(|other_id| self.is_polygon_cutted_by(*poly_id, *other_id))
                .collect_vec();

            if !poly_ids.is_empty() {
                //println!("OK {}", poly_ids.len());
                result.insert(*poly_id, poly_ids);
                //dbg!(result.keys());
            } else {
                println!("none ");
            }
        }
        result
    }
    */

    pub fn save_polygon(&mut self, polygon: &Polygon) -> PolyId {
        let aabb = Aabb::from_points(&polygon.vertices);

        let segments = polygon
            .get_segments()
            .into_iter()
            .map(|s| self.save_segment(s))
            .collect_vec();

        let poly = Poly {
            segments: segments.clone(),
            aabb,
        };
        let poly_id = self.insert_poly(poly);
        let segs_len = segments.len();
        for current in 0..segs_len {
            let next = (current + 1) % segs_len;
            let Seg {
                rib_id: cur_rib_id, ..
            } = segments[current];
            let Seg {
                rib_id: next_rib_id,
                ..
            } = segments[next];

            Self::save_index(&mut self.rib_to_poly, cur_rib_id, poly_id);
            //println!("      <> SAVE {cur_rib_id:?} > {poly_id:?}");

            Self::save_index(&mut self.adjacent_ribs, next_rib_id, cur_rib_id);
            Self::save_index(&mut self.adjacent_ribs, cur_rib_id, next_rib_id);
            let Rib(from, to) = self.ribs[&cur_rib_id];
            Self::save_index(&mut self.pt_to_poly, from, poly_id);
            Self::save_index(&mut self.pt_to_poly, to, poly_id);

            let Rib(from, to) = self.ribs[&cur_rib_id];
            Self::save_index(&mut self.pt_to_poly, from, poly_id);
            Self::save_index(&mut self.pt_to_poly, to, poly_id);
        }
        poly_id
    }

    pub fn save_segment(&mut self, segment: Segment) -> Seg {
        let from = self.insert_point(segment.from);
        let to = self.insert_point(segment.to);

        if from == to {
            panic!("Segment too short: {:?}", segment);
        }
        let mut dir = SegmentDir::Fow;
        let rib = if from > to {
            dir = SegmentDir::Rev;
            Rib(to, from)
        } else {
            Rib(from, to)
        };
        let rib_id = self.insert_rib(rib);
        Self::save_index(&mut self.pt_to_ribs, from, rib_id);
        Self::save_index(&mut self.pt_to_ribs, to, rib_id);

        Seg { rib_id, dir }
    }

    fn get_poly_aabb(&self, poly_id: &PolyId) -> Aabb {
        self.polygons[poly_id].aabb
    }

    fn is_polygon_cutted_by(&self, poly_id: PolyId, tool_id: PolyId) -> bool {
        let tool: Polygon = self.load_polygon(tool_id);
        // let my: Polygon = self.load_polygon(poly_id);
        let tool_plane = tool.get_plane();

        // Firstly, detect, that there are points in `poly_id`, that lay on different sides of
        // polygon plane.
        // Then, detect, that intersection line of plane lays on both polygons

        self.is_polygon_vertices_on_different_side_of_plane(poly_id, tool_plane)
            && self.is_polygon_intersection_segments_overlap(poly_id, tool_id)

        /*
            match my_plane.relate(&other_plane) {
                // TODO: Make this check based on segments.
                // line-polygon intersection is a segment.
                // all vertices could be projected on a line.
                // And intersection happens only when those segments are overlapped
                PlanarRelation::Intersect(line) => {
                    let line_intersects_me = line.relate(&poly);
                    let line_intersects_other = line.relate(&other);

                    let mut total_intersections_me: Vec<Vector3<Dec>> = Vec::new();
                    let mut total_intersections_other: Vec<Vector3<Dec>> = Vec::new();
                    let my_only_edges = matches!(
                        &line_intersects_me,
                        LinearPolygonRelation::IntersectInPlane { vertices, edges,.. } if (vertices.len() +edges.len()) == 0);
                    let other_only_edges = matches!(
                        &line_intersects_other,
                        LinearPolygonRelation::IntersectInPlane { vertices, edges,.. } if (vertices.len() +edges.len()) == 0);
                    if my_only_edges && other_only_edges  {
                        return false;
                    }

                    if let LinearPolygonRelation::IntersectInPlane {
                        vertices,
                        edges,
                        common_edges,
                    } = line_intersects_me
                    {
                        dbg!(&vertices, &edges, &common_edges);
                        total_intersections_me.extend(vertices);
                        total_intersections_me.extend(edges.into_iter().map(|e| e.1));
                        total_intersections_me.extend(
                            common_edges
                                .into_iter()
                                .flat_map(|e| [e.from, e.to])
                                .dedup(),
                        );
                    }
                    if let LinearPolygonRelation::IntersectInPlane {
                        vertices,
                        edges,
                        common_edges,
                    } = line_intersects_other
                    {
                        dbg!(&vertices, &edges, &common_edges);
                        total_intersections_other.extend(vertices);
                        total_intersections_other.extend(edges.into_iter().map(|e| e.1));
                        total_intersections_other.extend(
                            common_edges
                                .into_iter()
                                .flat_map(|e| [e.from, e.to])
                                .dedup(),
                        );
                    }
                    let mut segments_me = Vec::new();
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

                    dbg!("======================");
                    dbg!(&segments_me);
                    dbg!(&segments_other);
                    let btw = |seg_my: &(Dec, Dec), item: Dec| item > seg_my.0 && item < seg_my.1;
                    let overlaps = |seg_my: &(Dec, Dec), seg_other: &(Dec, Dec)| {
                        btw(seg_my, seg_other.0) || btw(seg_my, seg_other.1)
                    };
                    for seg_my in &segments_me {
                        for seg_other in &segments_other {
                            println!("check {} {}", seg_my.0, seg_my.1);
                            println!("with {} {}", seg_other.0, seg_other.1);
                            if overlaps(seg_my, seg_other) || overlaps(seg_other, seg_my) {
                                return true;
                            }
                        }
                    }

                    false
                }
                PlanarRelation::Coplanar => false,
                PlanarRelation::Opposite => {
                    //println!("oppo");
                    false
                }
                PlanarRelation::Parallel => false,
            }
        */
    }

    pub(crate) fn load_polygon(&self, other_id: PolyId) -> Polygon {
        let poly = &self.polygons[&other_id];
        let vertices = poly
            .segments
            .iter()
            .map(|seg| {
                if let Some(r) = self.ribs.get(&seg.rib_id) {
                    match seg.dir {
                        SegmentDir::Fow => self.vertices.get_point(r.0),
                        SegmentDir::Rev => self.vertices.get_point(r.1),
                    }
                } else {
                    panic!("failed to load {other_id:?} {:?}", seg.rib_id);
                }
            })
            .collect();
        Polygon::new(vertices).expect("must be ok")
    }

    fn insert_rib(&mut self, rib: Rib) -> RibId {
        if let Some(rib_id) = self.ribs.iter().find(|(_, &s)| s == rib).map(|(id, _)| id) {
            *rib_id
        } else {
            let rib_id = RibId::default();
            //println!("@NEW RIB  {rib_id:?} ");
            self.ribs.insert(rib_id, rib);
            rib_id
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
            let poly_id = PolyId::default();
            let rtree_item = PolyRtreeRecord(poly_id, poly.aabb);
            self.polygon_index.insert(rtree_item);
            //println!("@NEW POLY  {poly_id:?}");
            self.polygons.insert(poly_id, poly);
            poly_id
        }
    }

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

    fn insert_mesh(&mut self, item: Mesh) -> MeshId {
        if let Some(id) = self
            .meshes
            .iter()
            .find(|&(_, s)| *s == item)
            .map(|(id, _)| id)
        {
            *id
        } else {
            let id = MeshId::default();
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
    {
        if let Some(items) = index.get_mut(&ix) {
            items.push(item);
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

    /*
    fn replace_rib(&mut self, rib_id: &RibId) {
        if let Some(rib) = self.ribs.remove(rib_id) {
            self.rib_to_poly.remove(rib_id);
            self.rib_to_mesh_bound.remove(rib_id);
            Self::remove_item_from_index(&mut self.pt_to_ribs, &rib.0, rib_id);
            Self::remove_item_from_index(&mut self.pt_to_ribs, &rib.1, rib_id);
            for ar in self
                .adjacent_ribs
                .get(rib_id)
                .cloned()
                .into_iter()
                .flatten()
            {
                Self::remove_item_from_index(&mut self.adjacent_ribs, &ar, rib_id);
            }
            self.adjacent_ribs.remove(rib_id);
        }
    }
    */

    /// Remove polygon from all available related structures
    fn remove_polygon(&mut self, poly_id: PolyId) {
        if let Some(poly) = self.polygons.remove(&poly_id) {
            //println!("X REMOVE POLY {poly_id:?}");
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
                //println!("before delete {}", self.rib_to_poly[&r].len());

                Self::remove_item_from_index(&mut self.rib_to_poly, &r, &poly_id);
                /*
                if self.rib_to_poly[&r].len() < 2 {
                    println!(
                        " This rib ({r:?})does not connect polygon {}",
                        self.rib_to_poly[&r].len()
                    );
                }
                    */
            }

            self.polygon_index
                .remove(&PolyRtreeRecord(poly_id, poly.aabb));
        }
    }

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

    /// Return all polygons, belonging to given mesh
    pub(crate) fn get_mesh_polygon_ids(
        &self,
        tool_mesh_id: &MeshId,
    ) -> impl Iterator<Item = PolyId> + '_ {
        self.meshes
            .get(tool_mesh_id)
            .into_iter()
            .flat_map(|mesh| &mesh.0)
            .filter_map(|bound_id| self.mesh_bounds.get(bound_id))
            .flat_map(|mesh_bound| [&[mesh_bound.bound], mesh_bound.holes.as_slice()].concat())
    }

    /// Load mesh in the same way, we as load polygon
    pub(crate) fn load_mesh(&self, mesh_id: MeshId) -> crate::volume::mesh::Mesh {
        let mesh = &self.meshes[&mesh_id];
        /*
        for pt in self.get_mesh_points(&mesh_id) {
            println!(" # mesh {} {} {}", pt.x, pt.y, pt.z);
        }
        */
        let bounds = mesh
            .0
            .iter()
            .map(|ix| self.load_mesh_bound(ix))
            .collect_vec();
        crate::volume::mesh::Mesh::new(bounds)
    }

    fn get_polygon_points(&self, polygon_id: PolyId) -> HashSet<PtId> {
        self.polygons
            .get(&polygon_id) // find mesh
            .into_iter()
            .flat_map(|mesh| &mesh.segments) // take in bound ids ...
            .map(|seg| seg.rib_id)
            .filter_map(|rib_id| self.ribs.get(&rib_id))
            .flat_map(|rib| [rib.0, rib.1])
            .collect()
    }

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

    pub(crate) fn drop_polygons(&mut self, polygon_ids: impl Iterator<Item = PolyId>) {
        for p in polygon_ids {
            if let Some(mb_id) = self
                .mesh_bounds
                .iter()
                .find(|(_, mb)| mb.bound == p)
                .map(|(ix, _)| *ix)
            {
                for m in self.meshes.values_mut().filter(|m| m.0.contains(&mb_id)) {
                    m.0.retain(|&mb| mb != mb_id);
                }

                self.remove_mesh_bound(&mb_id);
            }

            if let Some(mb) = self
                .mesh_bounds
                .values_mut()
                .find(|mb| mb.holes.contains(&p))
            {
                mb.holes.retain(|&poly_id| p != poly_id);
            }

            self.remove_polygon(p);
        }
    }

    pub(crate) fn collect_meshes(&mut self) -> Vec<crate::volume::mesh::Mesh> {
        let checkpoint = std::time::Instant::now();
        self.update_mesh_bounds_index();
        self.meshes.clear();
        let checkpoint = print_time("  update index and clean", checkpoint);

        while let Some(m) = self.get_closed_mesh() {
            self.insert_mesh(m);
        }

        let checkpoint = print_time("  collected all meshes", checkpoint);
        let meshes = self.meshes.keys().map(|m| self.load_mesh(*m)).collect();
        print_time("  loaded meshes", checkpoint);
        meshes
    }

    pub(crate) fn mesh_bounds(&self, mesh_id: &MeshId) -> impl Iterator<Item = Cow<'_, Edge>> {
        self.meshes
            .get(mesh_id)
            .into_iter()
            .flat_map(|m| &m.0)
            .map(|mb_id| Cow::Owned(self.load_mesh_bound(mb_id)))
    }

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

    fn get_polygon_ribs(&self, bound: &PolyId) -> HashSet<RibId> {
        self.polygons
            .get(bound)
            .into_iter()
            .flat_map(|p| &p.segments)
            .map(|s| s.rib_id)
            .collect()
    }

    fn get_mesh_bound_ribs(&self, mesh_bound: &MeshBoundId) -> HashSet<RibId> {
        self.mesh_bounds
            .get(mesh_bound)
            .iter()
            .flat_map(|mb| mb.polies())
            .filter_map(|p| self.polygons.get(&p))
            .flat_map(|p| &p.segments)
            .map(|seg| seg.rib_id)
            .collect()
    }

    pub(crate) fn remove_opposites_and_collapse_sames(&mut self) {
        self.update_mesh_bounds_index();
        let totally_same_ribs = self.find_sides_sharing_same_ribs();
        let mut to_remove = Vec::new();
        println!("Found {} pairs of sames", totally_same_ribs.len());
        let mut coplanars = 0;
        let mut opposites = 0;
        for [one, two] in totally_same_ribs {
            let plane_one = self.get_polygon_plane(one);
            let plane_two = self.get_polygon_plane(two);
            match plane_two.relate(&plane_one) {
                PlanarRelation::Coplanar => {
                    coplanars += 1;
                    to_remove.push(two);
                }
                PlanarRelation::Opposite => {
                    opposites += 1;
                    to_remove.push(two);
                    to_remove.push(one);
                }
                _ => unreachable!(),
            }
        }

        println!("Coplanars: {coplanars}. Opposite {opposites}.");

        let collect_mb = self
            .mesh_bounds
            .iter()
            .filter(|(_, mb)| to_remove.contains(&mb.bound))
            .map(|k| *k.0)
            .collect_vec();

        for mb in collect_mb {
            self.remove_mesh_bound(&mb);
        }

        for mb in to_remove {
            self.remove_polygon(mb);
        }
    }

    fn find_sides_sharing_same_ribs(&self) -> Vec<[PolyId; 2]> {
        let special_bounds = self
            .rib_to_poly
            .iter()
            .filter(|kv| kv.1.len() > 2)
            .flat_map(|kv| kv.1)
            .collect::<HashSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        let mut totally_same_ribs = Vec::new();
        if special_bounds.is_empty() {
            return totally_same_ribs;
        }
        for ix in 0..special_bounds.len() - 1 {
            let prev_bound = special_bounds[ix];
            let prev_ribs = self.get_polygon_ribs(prev_bound);
            #[allow(clippy::needless_range_loop)]
            for iy in (ix + 1)..special_bounds.len() {
                let next_bound = special_bounds[iy];
                let next_ribs = self.get_polygon_ribs(next_bound);
                if prev_ribs.difference(&next_ribs).map(|_| 1).sum::<i32>() == 0 {
                    totally_same_ribs.push([*prev_bound, *next_bound]);
                }
            }
        }
        totally_same_ribs
    }

    fn have_common_rib(&self, p: &PolyId, poly_id: &PolyId) -> bool {
        let ribs = self.get_polygon_ribs(p);
        ribs.intersection(&self.get_polygon_ribs(poly_id)).count() > 0
    }

    fn replace_polygons(&mut self, poly_id: PolyId, polygons: &[PolyId]) {
        for mb_id in self
            .mesh_bounds
            .iter()
            .filter(|(_id, mb)| mb.bound == poly_id)
            .map(|(&id, _)| id)
            .collect_vec()
        {
            //dbg!(mb_id);
            let mesh_id = self.get_mesh_for_bound(&mb_id);

            for (i, new_polygon_id) in polygons.iter().enumerate() {
                if i == 0 {
                    if let Some(mb) = self.mesh_bounds.get_mut(&mb_id) {
                        //println!("replace {poly_id:?} with {new_polygon_id:?}");
                        mb.bound = *new_polygon_id;
                    }
                } else {
                    //println!("add new poly  {new_polygon_id:?}");
                    let mesh_bound = self.insert_mesh_bound(MeshBound {
                        bound: *new_polygon_id,
                        holes: Vec::new(),
                    });

                    if let Some(m) = mesh_id.and_then(|mesh_id| self.meshes.get_mut(&mesh_id)) {
                        m.0.push(mesh_bound);
                    }
                }
            }
        }
        //TODO Also do for holes
        for _mb_id in self
            .mesh_bounds
            .iter()
            .filter(|(_id, mb)| mb.holes.contains(&poly_id))
            .map(|(&id, _)| id)
            .collect_vec()
        {
            todo!("replace holes polygons");
        }
    }

    pub(crate) fn get_polygon_plane(&self, p: PolyId) -> Plane {
        self.load_polygon(p).get_plane()
    }

    fn get_mesh_for_bound(&self, mb_id: &MeshBoundId) -> Option<MeshId> {
        self.meshes
            .iter()
            .find(|(_, m)| m.0.contains(mb_id))
            .map(|(id, _)| *id)
    }

    fn update_mesh_bounds_index(&mut self) {
        self.rib_to_mesh_bound.clear();
        for mb_id in self.mesh_bounds.keys() {
            for rib_id in self.get_mesh_bound_ribs(mb_id) {
                Self::save_index(&mut self.rib_to_mesh_bound, rib_id, *mb_id);
            }
        }
        /*
        for (rib_id, items) in self.rib_to_mesh_bound.iter() {
            println!("mb ribs {rib_id:?}: {}", items.len());
        }
        for (rib_id, items) in self.rib_to_poly.iter() {
            println!("poly ribs {rib_id:?}: {}", items.len());
        }
        */
    }

    fn update_rib_polygon_index(&mut self) {
        // println!("~~~~~~~~~~~~~~~~~~~~~~~update rib polygon index~~~~~~~~~~~");

        self.rib_to_poly.clear();
        for polygon_id in self.polygons.keys() {
            for rib_id in self.get_polygon_ribs(polygon_id) {
                Self::save_index(&mut self.rib_to_poly, rib_id, *polygon_id);
            }
        }
    }

    pub fn find_orphan_ribs(&self) {
        for (rib_id, ps) in self.rib_to_poly.iter() {
            if ps.len() < 2 {
                println!(
                    " ## ORPHAN {rib_id:?} {} [{:?}]",
                    ps.len(),
                    self.polygons[&ps[0]]
                        .segments
                        .iter()
                        .map(|s| s.rib_id)
                        .collect_vec()
                );
            }
            if ps.len() > 2 {
                println!(
                    " ## EXCESS {rib_id:?} {} [{:?}]",
                    ps.len(),
                    self.polygons[&ps[0]]
                        .segments
                        .iter()
                        .map(|s| s.rib_id)
                        .collect_vec()
                );
            }
        }
    }

    fn is_polygon_intersection_segments_overlap(&self, poly_id: PolyId, tool_id: PolyId) -> bool {
        let my_poly = self.load_polygon(poly_id);
        let my_plane = my_poly.get_plane();
        let tool_poly = self.load_polygon(tool_id);
        let tool_plane = tool_poly.get_plane();
        match my_plane.relate(&tool_plane) {
            PlanarRelation::Intersect(line) => {
                let line_intersects_me = line.relate(&my_poly);
                let line_intersects_other = line.relate(&tool_poly);

                let mut total_intersections_me: Vec<Vector3<Dec>> = Vec::new();
                let mut total_intersections_other: Vec<Vector3<Dec>> = Vec::new();

                /*
                if my_poly.vertices.len() == 6 {
                    dbg!(&line_intersects_me);
                }
                */

                if let LinearPolygonRelation::IntersectInPlane {
                    vertices,
                    edges,
                    common_edges,
                } = line_intersects_me
                {
                    total_intersections_me.extend(vertices);
                    total_intersections_me.extend(edges.into_iter().map(|e| e.1));
                    total_intersections_me
                        .extend(common_edges.into_iter().flat_map(|e| [e.from, e.to]));
                }
                if let LinearPolygonRelation::IntersectInPlane {
                    vertices,
                    edges,
                    common_edges,
                } = line_intersects_other
                {
                    total_intersections_other.extend(vertices);
                    total_intersections_other.extend(edges.into_iter().map(|e| e.1));
                    total_intersections_other
                        .extend(common_edges.into_iter().flat_map(|e| [e.from, e.to]));
                }
                let mut segments_me = Vec::new();
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
                for seg_my in &segments_me {
                    for seg_other in &segments_other {
                        /*
                        if my_poly.vertices.len() == 6 {

                            println!("check {} {}", seg_my.0, seg_my.1);
                            println!("with {} {}", seg_other.0, seg_other.1);
                        }
                        */
                        if equals(seg_my, seg_other)
                            || overlaps(seg_my, seg_other)
                            || overlaps(seg_other, seg_my)
                        {
                            return true;
                        }
                    }
                }

                false
            }
            PlanarRelation::Coplanar => false,
            PlanarRelation::Opposite => {
                //println!("oppo");
                false
            }
            PlanarRelation::Parallel => false,
        }
    }

    fn is_polygon_vertices_on_different_side_of_plane(
        &self,
        poly_id: PolyId,
        other_plane: Plane,
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

    pub(crate) fn get_mesh_ribs(&self, mesh_id: MeshId) -> HashSet<RibId> {
        self.get_mesh_polygon_ids(&mesh_id)
            .filter_map(|p| self.polygons.get(&p))
            .flat_map(|p| &p.segments)
            .map(|s| s.rib_id)
            .collect()
    }

    fn get_closed_mesh(&self) -> Option<Mesh> {
        let checkpoint = std::time::Instant::now();
        let collected_mesh_bounds = self
            .meshes
            .values()
            .flat_map(|m| &m.0)
            .copied()
            .collect::<HashSet<_>>();
        let checkpoint = print_time("    collected for mesh", checkpoint);
        if let Some(first_mb) = self
            .mesh_bounds
            .keys()
            .find(|k| !collected_mesh_bounds.contains(k))
        {
            let mut mesh_index: HashMap<_, Vec<_>> = HashMap::new();
            let mut mesh_bound_queue = VecDeque::new();
            mesh_bound_queue.push_back(*first_mb);
            while let Some(mesh_bound_id) = mesh_bound_queue.pop_front() {
                let ribs = self.get_mesh_bound_ribs(&mesh_bound_id);
                if ribs.is_empty() {
                    println!("no ribs, no bound, no mesh");
                    return None;
                }
                for rib_id in &ribs {
                    self.rib_to_mesh_bound
                        .get(rib_id)
                        .cloned()
                        .into_iter()
                        .flatten()
                        .filter(|&mb| mb != mesh_bound_id)
                        .for_each(|mb| {
                            if !mesh_index
                                .get(&mesh_bound_id)
                                .is_some_and(|v| v.contains(&mb))
                            {
                                //self.move_mesh_bound_to_mesh(&mb, mesh_id);
                                mesh_bound_queue.push_back(mb);
                                Self::save_index(&mut mesh_index, mesh_bound_id, mb);
                            }
                        });
                }

                if mesh_index
                    .get(&mesh_bound_id)
                    .map(|v| v.iter().map(ToOwned::to_owned).collect::<HashSet<_>>())
                    .is_some_and(|v| v.len() != ribs.len())
                {
                    println!(
                        "Not all mesh bound ribs have correcponding mesh bounds!? \n{:#?}\n{:#?}",
                        mesh_index[&mesh_bound_id].len(),
                        ribs.len()
                    );
                    return None;
                }
            }
            print_time("    done with loops", checkpoint);
            Some(Mesh(mesh_index.into_keys().collect()))
        } else {
            None
        }
    }

    pub(crate) fn inspect_ribs(&self) {
        let mut orphan = Vec::new();
        let mut excess = Vec::new();

        for (r, v) in &self.rib_to_poly {
            if v.len() < 2 {
                orphan.push(r);
            }
            if v.len() > 2 {
                excess.push(r);
            }
        }
        if !orphan.is_empty() {
            panic!("Found {} orphan ribs", orphan.len());
        }
        if !excess.is_empty() {
            panic!("Found {} excess ribs", excess.len());
        }
    }
}

impl Seg {
    fn flip(&self) -> Seg {
        Self {
            rib_id: self.rib_id,
            dir: self.dir.flip(),
        }
    }

    fn to(&self, ribs: &HashMap<RibId, Rib>) -> PtId {
        let rib = ribs[&self.rib_id];
        match self.dir {
            SegmentDir::Fow => rib.1,
            SegmentDir::Rev => rib.0,
        }
    }
    fn from(&self, ribs: &HashMap<RibId, Rib>) -> PtId {
        let rib = ribs[&self.rib_id];
        match self.dir {
            SegmentDir::Fow => rib.0,
            SegmentDir::Rev => rib.1,
        }
    }
}

impl SegmentDir {
    fn flip(&self) -> SegmentDir {
        match self {
            SegmentDir::Fow => Self::Rev,
            SegmentDir::Rev => Self::Fow,
        }
    }
}

impl Default for RibId {
    fn default() -> Self {
        Self(uuid::Uuid::new_v4())
    }
}
impl Default for SegId {
    fn default() -> Self {
        Self(Uuid::new_v4())
    }
}
impl Default for PolyId {
    fn default() -> Self {
        Self(Uuid::new_v4())
    }
}
impl Default for MeshId {
    fn default() -> Self {
        Self(Uuid::new_v4())
    }
}
impl Default for MeshBoundId {
    fn default() -> Self {
        Self(Uuid::new_v4())
    }
}
#[derive(Debug, PartialEq, Clone, Copy)]
enum SegmentDir {
    Fow,
    Rev,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Seg {
    rib_id: RibId,
    dir: SegmentDir,
}

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct PolyId(Uuid);
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct MeshBoundId(Uuid);

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct SegId(Uuid);
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct MeshId(Uuid);
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct RibId(uuid::Uuid);

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Rib(PtId, PtId);
#[derive(Debug, PartialEq, Default, Clone)]
pub struct Poly {
    segments: Vec<Seg>,
    aabb: Aabb,
}

#[derive(Debug, PartialEq, Default, Clone)]
pub struct MeshBound {
    bound: PolyId,
    holes: Vec<PolyId>,
}

#[derive(Debug, PartialEq, Default, Clone)]
pub struct Mesh(Vec<MeshBoundId>);

#[derive(Debug, PartialEq)]
pub struct PolyRtreeRecord(PolyId, Aabb);

#[derive(Clone, PartialEq, Debug)]
pub struct RtreePt([Dec; 3]);

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

impl MeshBound {
    pub(crate) fn polies(&self) -> impl Iterator<Item = PolyId> + '_ {
        [&[self.bound], self.holes.clone().as_slice()]
            .concat()
            .into_iter()
            .map(|o| o.to_owned())
    }
}

fn collapse<T: PartialEq>(mut items: Vec<(T, T)>) -> Vec<(T, T)> {
    let mut res = Vec::new();
    while let Some(item) = items.pop() {
        if let Some(prev) = items.last_mut() {
            if item.0 == prev.1 {
                prev.1 = item.1;
            } else {
                res.push(item);
            }
        } else {
            res.push(item);
        }
    }
    res.reverse();
    res
}
enum Starting {
    AfterFront,
    AfterBack,
    BeforeBack,
    BeforeFront,
}
