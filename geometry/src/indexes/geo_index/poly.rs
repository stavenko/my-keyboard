use std::fmt;

use itertools::Itertools;
use nalgebra::{Vector2, Vector3};
use num_traits::{Bounded, Zero};
use rand::Rng;

use crate::{decimal::Dec, indexes::aabb::Aabb, planar::plane::Plane, polygon_basis::PolygonBasis};

use super::{
    face::{Face, FaceId},
    geo_object::{GeoObject, UnRef},
    index::GeoIndex,
    mesh::MeshId,
    seg::{SegRef, SegmentDir},
};

#[derive(PartialEq, Eq, Clone, Debug, Hash, Copy)]
pub struct UnrefPoly {
    pub mesh_id: MeshId,
    pub poly_id: PolyId,
}

impl<'a> PolyRef<'a> {
    pub fn segments(&self) -> impl Iterator<Item = SegRef<'a>> + 'a {
        let poly = self.polygon();
        let face = self.index.load_face_ref(poly.face_id());
        face.segments(poly.dir())
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

        for seg in self.segments() {
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
        for (ix, pt) in self.segments().map(|s| s.from_pt()).enumerate() {
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
            .segments()
            .map(|seg| format!("{}", seg.from_pt()))
            .collect_vec();
        collect_vec.reverse();
        collect_vec.join(", ")
    }

    /*
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
    */

    pub(crate) fn plane(&self) -> Plane {
        match self.dir() {
            SegmentDir::Fow => self.polygon().face().plane().to_owned(),
            SegmentDir::Rev => self.polygon().face().plane().to_owned().flipped(),
        }
    }

    fn polygon(&self) -> PolyRef {
        self.index.load_polygon_ref(self.mesh_id, self.poly_id)
    }

    fn face(&self) -> &Face {
        let face_id = self.polygon().face_id();
        &self.index.faces[&face_id]
    }

    fn print_debug(&self) -> String {
        self.segments()
            .map(|s| {
                let f = s.from();
                let t = s.to();
                format!("{} {} {} -> {} {} {}", f.x, f.y, f.z, t.x, t.y, t.z)
            })
            .join("\n")
    }

    pub fn face_id(&self) -> FaceId {
        self.index.meshes[&self.mesh_id].polies[&self.poly_id].face_id
    }

    pub fn dir(&self) -> SegmentDir {
        self.index.meshes[&self.mesh_id].polies[&self.poly_id].dir
    }

    pub fn poly_id(&self) -> PolyId {
        self.poly_id
    }

    pub fn mesh_id(&self) -> MeshId {
        self.mesh_id
    }

    pub(crate) fn normal(&self) -> Vector3<Dec> {
        self.plane().normal()
    }
}

#[derive(Clone)]
pub struct PolyRef<'a> {
    pub(super) poly_id: PolyId,
    pub(super) mesh_id: MeshId,
    pub(super) index: &'a GeoIndex,
}

impl<'a> UnRef<'a> for PolyRef<'a> {
    type Obj = UnrefPoly;

    fn un_ref(self) -> Self::Obj {
        UnrefPoly {
            mesh_id: self.mesh_id,
            poly_id: self.poly_id,
        }
    }
}

impl<'a> UnRef<'a> for PolyRefMut<'a> {
    type Obj = UnrefPoly;

    fn un_ref(self) -> Self::Obj {
        UnrefPoly {
            mesh_id: self.mesh_id,
            poly_id: self.poly_id,
        }
    }
}

impl<'a> GeoObject<'a> for UnrefPoly {
    type Ref = PolyRef<'a>;

    type MutRef = PolyRefMut<'a>;

    fn make_ref(&self, index: &'a GeoIndex) -> Self::Ref {
        PolyRef {
            poly_id: self.poly_id,
            mesh_id: self.mesh_id,
            index,
        }
    }

    fn make_mut_ref(&self, index: &'a mut GeoIndex) -> Self::MutRef {
        PolyRefMut {
            poly_id: self.poly_id,
            mesh_id: self.mesh_id,
            index,
        }
    }
}

pub struct PolyRefMut<'a> {
    pub(super) poly_id: PolyId,
    pub(super) mesh_id: MeshId,
    pub(super) index: &'a mut GeoIndex,
}
impl<'a> PolyRefMut<'a> {
    pub(crate) fn change_face(&mut self, face_id: FaceId) {
        if let Some(p) = self
            .index
            .meshes
            .get_mut(&self.mesh_id)
            .and_then(|m| m.polies.get_mut(&self.poly_id))
        {
            p.face_id = face_id;
        }
    }
    pub(crate) fn replace(&mut self, replacement: Vec<Poly>) {
        let poly_ix = self.poly_id;
        if self.mesh_id == 6 && (poly_ix == 35) {
            println!("replace poly 35 with {replacement:?}");
        }
        if let Some(mesh) = self.index.meshes.get_mut(&self.mesh_id) {
            mesh.polies.remove(&poly_ix);
            for p in replacement {
                mesh.add(p);
            }
        }
    }

    pub fn remove(&mut self) {
        self.index.remove_polygon(self.poly_id, self.mesh_id)
    }

    pub fn flip(&mut self) {
        if let Some(poly) = self
            .index
            .meshes
            .get_mut(&self.mesh_id)
            .and_then(|m| m.polies.get_mut(&self.poly_id))
        {
            poly.flip()
        }
    }
}

impl<'a> fmt::Debug for PolyRef<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.print_debug())
    }
}

/*
#[derive(Debug, Clone)]
pub struct Poly {
    segments: Vec<Seg>,
    plane: Plane,
    aabb: Aabb,
}

*/
#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Poly {
    pub(crate) face_id: FaceId,
    pub(crate) dir: SegmentDir,
}

impl Poly {
    pub(crate) fn fow(face_id: FaceId) -> Self {
        Self {
            face_id,
            dir: SegmentDir::Fow,
        }
    }

    pub(crate) fn rev(face_id: FaceId) -> Self {
        Self {
            face_id,
            dir: SegmentDir::Rev,
        }
    }

    fn flip(&mut self) {
        self.dir = self.dir.flip();
    }
}

/*
impl Poly {
    pub(crate) fn create(segments: Vec<Seg>, plane: Plane, aabb: Aabb) -> Self {
        Self {
            segments,
            plane,
            aabb,
        }
    }

    pub(crate) fn plane(&self) -> &Plane {
        &self.plane
    }

    pub(crate) fn aabb(&self) -> &Aabb {
        &self.aabb
    }

    pub(crate) fn replace_segments(
        &mut self,
        replacing_segment_ix: usize,
        replacement: impl IntoIterator<Item = Seg>,
    ) {
        self.segments.splice(
            replacing_segment_ix..(replacing_segment_ix + 1),
            replacement,
        );
    }

    pub(crate) fn update_rib_index(
        &self,
        my_id: PolyId,
        rib_index: &mut HashMap<RibId, Vec<PolyId>>,
    ) {
        for s in &self.segments {
            GeoIndex::save_index(rib_index, s.rib_id, my_id);
        }
    }

    pub(crate) fn delete_me_from_rib_index(
        &self,
        my_id: PolyId,
        rib_index: &mut HashMap<RibId, Vec<PolyId>>,
    ) {
        for s in &self.segments {
            GeoIndex::remove_item_from_index(rib_index, &s.rib_id, &my_id);
        }
    }

    pub(crate) fn flip(&mut self) {
        for s in self.segments.iter_mut() {
            *s = s.flip();
        }
        self.segments.reverse();

        self.plane.flip()
    }

    pub(crate) fn is_same_face(&self, other: &Poly) -> bool {
        let len_is_same = self.segments.len() == other.segments.len();
        if self.segments.is_empty() && len_is_same {
            return true;
        }
        let mut straight = other.segments.clone();
        let mut opposite = straight.iter().map(|s| s.flip()).rev().collect_vec();

        if let Some(other_ix) = straight.iter().position(|s| *s == self.segments[0]) {
            straight.rotate_left(other_ix);

            straight == self.segments
        } else if let Some(other_ix) = opposite.iter().position(|s| *s == self.segments[0]) {
            opposite.rotate_left(other_ix);

            opposite == self.segments
        } else {
            false
        }
    }
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
*/

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
