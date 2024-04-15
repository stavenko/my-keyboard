use crate::{
    edge::Edge,
    hull::Hull,
    indexes::geo_index::{GeoIndex, MeshId},
    measure_time::print_time,
    planar::polygon::Polygon,
    primitives_relation::{
        point_planar::PointPolygonRelation, point_volume::PointVolumeRelation, relation::Relation,
    },
    surface::topology::Topology,
};
use itertools::Itertools;
use nalgebra::Vector3;
use rust_decimal_macros::dec;
use std::{borrow::Cow, collections::HashMap};
use stl_io::Triangle;
use tap::TapFallible;

use crate::decimal::Dec;

pub enum PolygonJoinResult {
    One(Polygon),
    NoJoin,
    Recombination(Vec<Polygon>),
}

#[derive(Debug)]
pub struct Mesh {
    geo_index: GeoIndex,
    mesh_id: MeshId,
    // sides: Vec<Edge>,
}

impl Mesh {
    pub fn new(sides: Vec<Edge>) -> Self {
        let mut geo_index = GeoIndex::default();
        let mesh_id = geo_index.save_mesh(sides.into_iter().map(Cow::Owned));

        geo_index.inspect_ribs();

        Self { geo_index, mesh_id }
    }

    pub fn is_point_inside(&self, point: Vector3<Dec>) -> bool {
        match dbg!(self.relate(&point)) {
            PointVolumeRelation::In => true,
            PointVolumeRelation::Out => false,
            PointVolumeRelation::Edge(_) => false,
            PointVolumeRelation::Vertex => false,
            PointVolumeRelation::Side(_) => false,
        }
    }

    fn is_polygon_inside(&self, polygon: &Polygon) -> bool {
        let mut vertices_in = 0;
        let len = polygon.vertices.len();
        for vix in 0..len {
            let v = &polygon.vertices[vix];
            let prev_ix = (vix + len - 1) % len;
            let next_ix = (vix + 1) % len;
            match self.relate(v) {
                PointVolumeRelation::In => {
                    vertices_in += 1;
                }
                PointVolumeRelation::Out => {}
                PointVolumeRelation::Edge(s) => {
                    let u = polygon.vertices[prev_ix] - v;
                    let w = polygon.vertices[next_ix] - v;
                    let dd = Dec::from(dec!(0.0001));
                    let pp = u * dd + w * dd + v;
                    let npp = -u * dd + -w * dd + v;

                    let pp_mesh_inner = matches!(self.relate(&pp), PointVolumeRelation::In);
                    let pp_self_inner = matches!(polygon.relate(&pp), PointPolygonRelation::In);

                    let npp_mesh_inner = matches!(self.relate(&npp), PointVolumeRelation::In);
                    let npp_self_inner = matches!(polygon.relate(&npp), PointPolygonRelation::In);

                    if pp_self_inner == npp_self_inner {
                        dbg!(polygon);
                        dbg!(pp);
                        dbg!(npp);
                        dbg!(s);
                        panic!("Seems, like both points outside or inside")
                    }

                    if pp_mesh_inner && pp_self_inner {
                        vertices_in += 1;
                    }
                    if npp_mesh_inner && npp_self_inner {
                        vertices_in += 1;
                    }
                }
                PointVolumeRelation::Vertex => {
                    let u = polygon.vertices[prev_ix] - v;
                    let w = polygon.vertices[next_ix] - v;
                    let dd = Dec::from(dec!(0.0001));
                    let pp = u * dd + w * dd + v;
                    let npp = -u * dd + -w * dd + v;

                    let pp_mesh_inner = matches!(self.relate(&pp), PointVolumeRelation::In);
                    let pp_self_inner = matches!(polygon.relate(&pp), PointPolygonRelation::In);

                    let npp_mesh_inner = matches!(self.relate(&npp), PointVolumeRelation::In);
                    let npp_self_inner = matches!(polygon.relate(&npp), PointPolygonRelation::In);
                    if pp_self_inner == npp_self_inner {
                        panic!("Seems, like both points outside or inside")
                    }

                    if pp_mesh_inner && pp_self_inner {
                        vertices_in += 1;
                    }
                    if npp_mesh_inner && npp_self_inner {
                        vertices_in += 1;
                    }
                }
                PointVolumeRelation::Side(_) => {
                    let u = polygon.vertices[prev_ix] - v;
                    let w = polygon.vertices[next_ix] - v;
                    let dd = Dec::from(dec!(0.0001));
                    let pp = u * dd + w * dd + v;
                    let npp = -u * dd + -w * dd + v;

                    let pp_mesh_inner = matches!(self.relate(&pp), PointVolumeRelation::In);
                    let pp_self_inner = matches!(polygon.relate(&pp), PointPolygonRelation::In);

                    let npp_mesh_inner = matches!(self.relate(&npp), PointVolumeRelation::In);
                    let npp_self_inner = matches!(polygon.relate(&npp), PointPolygonRelation::In);

                    if pp_self_inner == npp_self_inner {
                        panic!("Seems, like both points outside or inside")
                    }
                    if pp_mesh_inner && pp_self_inner {
                        vertices_in += 1;
                    }
                    if npp_mesh_inner && npp_self_inner {
                        vertices_in += 1;
                    }
                }
            }
        }

        vertices_in == len
    }

    /*
    fn split_my_polygons_by(&self, other: &Self) -> Vec<Polygon> {
        let mut my_polygons: VecDeque<Polygon> = self.polygons().into();
        let mut results = Vec::new();

        'my: while let Some(my_polygon) = my_polygons.pop_front() {
            for tool in other.polygons() {
                if let Some((f, b)) = tool.split(&my_polygon) {
                    my_polygons.push_front(b);
                    my_polygons.push_front(f);
                    continue 'my;
                }
            }
            // no cut - push
            results.push(my_polygon);
        }
        results
    }

    fn collect_all_opposites(ones: &mut Vec<Polygon>) -> Vec<Vec<Polygon>> {
        let mut opposites = Vec::new();
        let mut current_ones = 0;
        while let Some(one) = ones.get(current_ones) {
            let mut coplanars_to_current = Vec::new();
            let plane = one.get_plane();
            while let Some((poly_in_one, _)) =
                ones.iter()
                    .enumerate()
                    .skip(current_ones + 1)
                    .find(|(_, p)| match p.get_plane().relate(&plane) {
                        PlanarRelation::Coplanar => false,
                        PlanarRelation::Intersect(_) => false,
                        PlanarRelation::Opposite => true,
                        PlanarRelation::Parallel => false,
                    })
            {
                //dbg!(ones[poly_in_one].get_plane());
                let poly = ones.swap_remove(poly_in_one);
                coplanars_to_current.push(poly);
            }

            if !coplanars_to_current.is_empty() {
                let poly = ones.swap_remove(current_ones);
                coplanars_to_current.push(poly);
                //println!("some {}", coplanars_to_current.len());
                opposites.push(coplanars_to_current);
            } else {
                //println!("none");
                current_ones += 1;
            }
        }

        opposites
    }

    fn collect_all_coplanars(ones: &mut Vec<Polygon>) -> Vec<Vec<Polygon>> {
        let mut coplanars = Vec::new();
        let mut current_ones = 0;
        while let Some(one) = ones.get(current_ones) {
            let mut coplanars_to_current = Vec::new();
            let plane = one.get_plane();
            while let Some((poly_in_one, _)) =
                ones.iter()
                    .enumerate()
                    .skip(current_ones + 1)
                    .find(|(_, p)| match p.get_plane().relate(&plane) {
                        PlanarRelation::Coplanar => true,
                        PlanarRelation::Intersect(_) => false,
                        PlanarRelation::Opposite => false,
                        PlanarRelation::Parallel => false,
                    })
            {
                let poly = ones.swap_remove(poly_in_one);
                coplanars_to_current.push(poly);
            }

            if !coplanars_to_current.is_empty() {
                let poly = ones.swap_remove(current_ones);
                coplanars_to_current.push(poly);
                coplanars.push(coplanars_to_current);
            } else {
                current_ones += 1;
            }
        }

        coplanars
    }

    fn join_polygons(one: &Polygon, two: &Polygon) -> PolygonJoinResult {
        let one_plane = one.get_plane();
        let two_plane = two.get_plane();
        let mut results = match one_plane.relate(&two_plane) {
            PlanarRelation::Coplanar => one.boolean_union(two),

            PlanarRelation::Opposite => {
                return PolygonJoinResult::NoJoin;
            }

            PlanarRelation::Parallel => unreachable!(),
            PlanarRelation::Intersect(_) => unreachable!(),
        };

        if results.len() == 1 {
            PolygonJoinResult::One(results.remove(0))
        } else if results.len() == 2 {
            if results.contains(one) && results.contains(two) {
                PolygonJoinResult::NoJoin
            } else {
                PolygonJoinResult::Recombination(results)
            }
        } else {
            PolygonJoinResult::Recombination(results)
        }
    }

    fn group_by_connected(mut polygons: Vec<Polygon>) -> Vec<Vec<Polygon>> {
        let mut groups = Vec::new();
        while !polygons.is_empty() {
            let (group, rest) = Self::collect_connected_polygons(polygons);
            polygons = rest;
            groups.push(group);
        }

        groups
    }

    fn join_coplanars(polygon_groups: Vec<Vec<Polygon>>) -> Vec<Polygon> {
        polygon_groups
            .into_iter()
            .flat_map(|mut polygons| {
                let mut current = 0;
                let mut next = 1;

                while next < polygons.len() {
                    if let Some(first) = polygons.get(current) {
                        if let Some(second) = polygons.get(next) {
                            dbg!("waaaht");
                            match Self::join_polygons(first, second) {
                                PolygonJoinResult::One(p) => {
                                    dbg!("one");
                                    polygons.push(p);
                                    polygons.swap_remove(current);
                                    polygons.remove(next);
                                }
                                PolygonJoinResult::NoJoin => {
                                    dbg!("nojoin");
                                    if next == polygons.len() - 1 {
                                        current += 1;
                                        next = current + 1;
                                    } else {
                                        next += 1;
                                    }
                                }
                                PolygonJoinResult::Recombination(ps) => {
                                    dbg!("one");
                                    polygons.extend(ps);
                                    polygons.swap_remove(next);
                                    polygons.swap_remove(current);
                                }
                            }
                        }
                    }
                }

                polygons
            })
            .collect()
    }
    */

    /// Takes from tool all polygons, which are inside self.
    pub fn drop_full_insides(&self, tool_polygons: Vec<Polygon>) -> Vec<Polygon> {
        tool_polygons
            .into_iter()
            .filter(|poly| !self.is_polygon_inside(poly))
            .collect()
    }

    pub fn take_full_insides(&self, tool_polygons: Vec<Polygon>) -> Vec<Polygon> {
        tool_polygons
            .into_iter()
            .filter(|poly| self.is_polygon_inside(poly))
            .collect()
    }

    /*
    pub fn collect_connected_edges(mut polygons: Vec<Edge>) -> (Vec<Edge>, Vec<Edge>) {
        let mut mesh: Vec<usize> = Vec::new();
        let mut adjacent: HashMap<usize, Vec<usize>> = HashMap::new();
        if polygons.is_empty() {
            return (Vec::new(), Vec::new());
        }

        mesh.push(0);
        let mut current_mesh_item = 0;
        while let Some(&adjacent_for) = mesh.get(current_mesh_item) {
            if let Some(polygon) = polygons.get(adjacent_for) {
                let segments = polygon.get_segments();

                if adjacent.entry(adjacent_for).or_default().len() < segments.len() {
                    'searcher: for (i, _poly) in polygons.iter().enumerate() {
                        if adjacent
                            .get(&adjacent_for)
                            .is_some_and(|ff| ff.len() == segments.len())
                        {
                            break 'searcher;
                        }

                        if !adjacent
                            .get(&adjacent_for)
                            .is_some_and(|ff| ff.contains(&i))
                        {
                            let candidate_segments = polygons[i].get_segments();
                            'inner: for s in candidate_segments {
                                let flipped = s.flip();
                                if segments.contains(&flipped) {
                                    adjacent
                                        .entry(i)
                                        .and_modify(|v| v.push(adjacent_for))
                                        .or_insert(vec![adjacent_for]);
                                    adjacent
                                        .entry(adjacent_for)
                                        .and_modify(|v| v.push(i))
                                        .or_insert(vec![i]);
                                    if !mesh.contains(&i) {
                                        mesh.push(i);
                                    }
                                    break 'inner;
                                }
                            }
                        }
                    }
                }
            } else {
                break;
            }

            current_mesh_item += 1;
        }
        dbg!(&mesh);

        let mut connected_polygons = Vec::new();
        while let Some(ix) = mesh.pop() {
            let poly = polygons.remove(ix);
            connected_polygons.push(poly);
            mesh.iter_mut().filter(|i| **i > ix).for_each(|ix| *ix -= 1);
        }
        connected_polygons.reverse();

        (connected_polygons, polygons)
    }
    */

    pub fn collect_connected_polygons(mut polygons: Vec<Polygon>) -> (Vec<Polygon>, Vec<Polygon>) {
        let mut mesh: Vec<usize> = Vec::new();
        let mut adjacent: HashMap<usize, Vec<usize>> = HashMap::new();
        if polygons.is_empty() {
            return (Vec::new(), Vec::new());
        }

        dbg!("collect_connected_polygons");
        dbg!(polygons.len());

        mesh.push(0);
        let mut current_mesh_item = 0;
        while let Some(&adjacent_for) = mesh.get(current_mesh_item) {
            if let Some(polygon) = polygons.get(adjacent_for) {
                let segments = polygon.get_segments();

                if adjacent.entry(adjacent_for).or_default().len() < segments.len() {
                    'searcher: for (i, _poly) in polygons.iter().enumerate() {
                        if adjacent
                            .get(&adjacent_for)
                            .is_some_and(|ff| ff.len() == segments.len())
                        {
                            break 'searcher;
                        }

                        if !adjacent
                            .get(&adjacent_for)
                            .is_some_and(|ff| ff.contains(&i))
                        {
                            let candidate_segments = polygons[i].get_segments();
                            'inner: for s in candidate_segments {
                                let flipped = s.flip();
                                if segments.contains(&flipped) {
                                    adjacent
                                        .entry(i)
                                        .and_modify(|v| v.push(adjacent_for))
                                        .or_insert(vec![adjacent_for]);
                                    adjacent
                                        .entry(adjacent_for)
                                        .and_modify(|v| v.push(i))
                                        .or_insert(vec![i]);
                                    if !mesh.contains(&i) {
                                        dbg!("===============");
                                        mesh.push(i);
                                    }
                                    break 'inner;
                                }
                            }
                        }
                    }
                }
            } else {
                break;
            }
            dbg!(current_mesh_item);

            current_mesh_item += 1;
        }
        dbg!("done");

        let mut connected_polygons = Vec::new();
        while let Some(ix) = mesh.pop() {
            let poly = polygons.remove(ix);
            connected_polygons.push(poly);
            mesh.iter_mut().filter(|i| **i > ix).for_each(|ix| *ix -= 1);
        }
        connected_polygons.reverse();

        (connected_polygons, polygons)
    }

    /*
    pub fn collect_closed_mesh(polygons: Vec<Edge>) -> (Option<Self>, Vec<Edge>) {
        let (connected_polygons, leftoffs) = Self::collect_connected_edges(polygons);

        if !connected_polygons.is_empty() {
            let m = Self::new(connected_polygons);

            (Some(m), leftoffs)
        } else {
            (None, leftoffs)
        }
    }

    fn from_edges(mut edges: Vec<Edge>) -> Vec<Self> {
        let mut meshes = Vec::new();
        while let (Some(mesh), rest) = Self::collect_closed_mesh(edges) {
            edges = rest;
            meshes.push(mesh);
        }

        meshes
    }
    */

    pub fn boolean_union(mut self, tool: Self) -> Vec<Self> {
        println!("==== start boolean union ====");
        let mut checkpoint = std::time::Instant::now();
        let tot = checkpoint;
        let tool_mesh_id = self.geo_index.save_mesh(tool.sides());
        let mut total_splitted_polygons = 0;

        while let Some((poly, tool)) = self.geo_index.find_first_intersection(&tool_mesh_id) {
            println!("Found something to split");
            let plane = self.geo_index.get_polygon_plane(tool);
            self.geo_index.split_poly_by_plane(poly, &plane);
            total_splitted_polygons += 1;
        }
        checkpoint = print_time("find intersection in my", checkpoint);

        while let Some((poly, tool)) = self.geo_index.find_first_intersection(&self.mesh_id) {
            let plane = self.geo_index.get_polygon_plane(tool);
            self.geo_index.split_poly_by_plane(poly, &plane);
            total_splitted_polygons += 1;
        }
        checkpoint = print_time("find intersection in tool", checkpoint);
        println!("Total splitted: {total_splitted_polygons}");

        if total_splitted_polygons > 0 {
            let to_drop_inside_tool = self.geo_index.find_insides(self.mesh_id, tool_mesh_id);
            checkpoint = print_time("find dropped polygons #1", checkpoint);
            let to_drop_inside_self = self.geo_index.find_insides(tool_mesh_id, self.mesh_id);
            checkpoint = print_time("find dropped polygons #2", checkpoint);
            let total_polygons_to_remove = to_drop_inside_self
                .into_iter()
                .chain(to_drop_inside_tool)
                .collect_vec();
            println!("Find polygons to remove {}", total_polygons_to_remove.len());

            self.geo_index
                .drop_polygons(total_polygons_to_remove.into_iter());
            checkpoint = print_time("drop polygons", checkpoint);
        } else {
            println!("None is splitted - we may just do something else");
        }

        if self.geo_index.has_common_ribs() {
            self.geo_index.remove_opposites_and_collapse_sames();
            checkpoint = print_time("drop opposites", checkpoint);
        }

        if self.geo_index.has_common_ribs() {
            panic!("Something is wrong");
        }

        let meshes = self.geo_index.collect_meshes();
        print_time("collect", checkpoint);
        print_time("total", tot);

        meshes
    }

    pub(crate) fn sides(&self) -> impl Iterator<Item = Cow<'_, Edge>> {
        self.geo_index.mesh_bounds(&self.mesh_id)
    }

    pub(crate) fn polygons(&self) -> impl Iterator<Item = Polygon> + '_ {
        self.sides().flat_map(|s| s.into_owned().polygons_owned())
    }

    /*
    pub fn boolean_union_heavy(self, tool: Self) -> Vec<Self> {
        dbg!("union");
        let my_polygons = self.split_my_polygons_by(&tool);
        dbg!(my_polygons.len());
        let tool_polygons = tool.split_my_polygons_by(&self);
        dbg!(tool_polygons.len());

        let my_polygons = tool.drop_full_insides(my_polygons);
        dbg!(my_polygons.len());
        let tool_polygons = self.drop_full_insides(tool_polygons);
        dbg!(tool_polygons.len());
        let mut joined = [my_polygons, tool_polygons].concat();

        let coplanars = Self::collect_all_coplanars(&mut joined);
        dbg!(coplanars.len());

        let _opposites = Self::collect_all_opposites(&mut joined);

        let coplanars: Vec<Edge> = coplanars
            .into_iter()
            .map(|was| {
                let plane = was[0].get_plane();
                let polygon_groups = Self::group_by_connected(was);
                dbg!("found groups");
                let joined_connected = Self::join_coplanars(polygon_groups);
                dbg!("joined_connected");

                if joined_connected.len() > 1 && Self::all_in_same_plane(&joined_connected) {
                    let polygons = Self::join_coplanars(vec![joined_connected]);
                    dbg!("joined coplad");
                    Edge { plane, polygons }
                } else {
                    Edge {
                        plane,
                        polygons: joined_connected,
                    }
                }
            })
            .collect_vec();

        dbg!(coplanars.len());

        Self::from_edges(
            [
                joined.into_iter().map(Edge::from_polygon).collect_vec(),
                coplanars,
            ]
            .concat()
            .to_vec(),
        )
    }
    */
    pub fn boolean_diff(mut self, tool: Self) -> Vec<Self> {
        // insert tool polygons in index
        let tool_mesh_id = self.geo_index.save_mesh(tool.sides());

        while let Some((poly, tool)) = self.geo_index.find_first_intersection(&tool_mesh_id) {
            let plane = self.geo_index.get_polygon_plane(tool);
            self.geo_index.split_poly_by_plane(poly, &plane);
        }

        while let Some((poly, tool)) = self.geo_index.find_first_intersection(&self.mesh_id) {
            let plane = self.geo_index.get_polygon_plane(tool);
            self.geo_index.split_poly_by_plane(poly, &plane);
        }

        let to_drop_inside_tool = self.geo_index.find_insides(self.mesh_id, tool_mesh_id);
        let to_drop_outside_self = self.geo_index.find_outsides(tool_mesh_id, self.mesh_id);

        self.geo_index
            .drop_polygons(to_drop_outside_self.into_iter().chain(to_drop_inside_tool));

        /*
        return [
            self.geo_index.load_mesh(self.mesh_id),
            self.geo_index.load_mesh(tool_mesh_id),
        ]
        .into_iter()
        .collect_vec();
        self.geo_index.print_all();
         */
        // panic!("Q");

        self.geo_index.remove_opposites_and_collapse_sames();

        self.geo_index.inverse_mesh(tool_mesh_id);

        //self.geo_index.join_mesh_bounds_in_same_plane();
        self.geo_index.collect_meshes()
    }
    /*


    pub fn group_by_sides(
        polygons: impl IntoIterator<Item = Polygon>,
    ) -> impl Iterator<Item = Edge> {
        let planes = polygons
            .into_iter()
            .map(|poly| (poly.get_plane(), poly))
            .fold(
                Vec::<(Plane, Vec<Polygon>)>::new(),
                |mut acc, (plane, poly)| {
                    if let Some((_, ps)) = acc
                        .iter_mut()
                        .find(|(p, _)| p == &plane || p == &plane.clone().flip())
                    {
                        ps.push(poly);
                    } else {
                        acc.push((plane, vec![poly]));
                    }
                    acc
                },
            );

        planes.into_iter().map(|(pl, ps)| {
            if ps.is_empty() {
                panic!("DEBUG");
            }

            Edge {
                plane: pl,
                polygons: ps,
            }
        })
    }

    pub fn from_polygons(polygons: impl IntoIterator<Item = Polygon>) -> (Self, Vec<Polygon>) {
        let sides = Self::group_by_sides(polygons)
            .filter_map(|s| s.join_polygons_on_side())
            .collect_vec();

        (Self::new(sides), Vec::new())
    }

    fn all_in_same_plane(joined_connected: &[Polygon]) -> bool {
        let pl = joined_connected[0].get_plane();
        for p in joined_connected.iter().map(|p| p.get_plane()) {
            if !matches!(p.relate(&pl), PlanarRelation::Coplanar) {
                return false;
            }
        }
        true
    }
    */
}

pub struct TriIter {
    inner: <Vec<Triangle> as IntoIterator>::IntoIter,
    size: usize,
}

impl ExactSizeIterator for TriIter {}

impl Iterator for TriIter {
    type Item = Triangle;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next()
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.size, Some(self.size))
    }
}

impl IntoIterator for Mesh {
    type Item = Triangle;

    type IntoIter = TriIter;

    fn into_iter(self) -> Self::IntoIter {
        let triangles = self
            .sides()
            .filter_map(|edge| {
                edge.triangles()
                    .tap_err(|e| {
                        dbg!(e);
                    })
                    .ok()
            })
            .flatten()
            .collect_vec();

        let size = dbg!(triangles.len());

        TriIter {
            inner: triangles.into_iter(),
            size,
        }
    }
}

impl<T> TryFrom<Hull<T>> for Mesh
where
    T: Topology,
{
    type Error = anyhow::Error;

    fn try_from(value: Hull<T>) -> Result<Self, Self::Error> {
        /*
        if value.sides.len() == 3 {
            dbg!(&value.sides.iter().map(|s| s.len()).collect_vec());
            dbg!(&value.inner.len());
            dbg!(&value.outer.len());
        }
        */
        let sides = value
            .sides
            .into_iter()
            .flatten()
            .chain(value.outer)
            .chain(value.inner)
            .map(Edge::from_polygon);

        let mesh = Mesh::new(sides.collect());

        Ok(mesh)
    }
}
