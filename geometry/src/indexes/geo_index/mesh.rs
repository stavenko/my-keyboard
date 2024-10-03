use std::{collections::HashMap, ops::Deref};

use itertools::Itertools;
use nalgebra::Vector3;

use crate::decimal::Dec;

use super::{
    face::FaceId,
    geo_object::{GeoObject, UnRef},
    index::GeoIndex,
    poly::{Poly, PolyId, UnrefPoly},
};

#[derive(Debug, PartialEq, Clone, Default)]
pub struct Mesh {
    poly_counter: usize,
    pub(super) polies: HashMap<PolyId, Poly>,
}

impl Mesh {
    pub(crate) fn add(&mut self, poly: Poly) -> PolyId {
        let poly_id = PolyId(self.poly_counter);
        if poly.face_id.0 == 2493 || poly.face_id.0 == 2494 {
            println!("  ...{poly_id:?} ");
        }
        self.poly_counter += 1;
        self.polies.insert(poly_id, poly);
        poly_id
    }
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
pub struct MeshId(pub usize);

impl PartialEq<usize> for MeshId {
    fn eq(&self, other: &usize) -> bool {
        self.0 == *other
    }
}

#[derive(Debug, Clone)]
pub struct MeshRef<'a> {
    pub(super) geo_index: &'a GeoIndex,
    pub(super) mesh_id: MeshId,
}

impl<'a> Deref for MeshRef<'a> {
    type Target = MeshId;

    fn deref(&self) -> &Self::Target {
        &self.mesh_id
    }
}

impl<'a> MeshRef<'a> {
    pub fn into_polygons(self) -> Vec<UnrefPoly> {
        self.mesh()
            .polies
            .keys()
            .map(|ix| UnrefPoly {
                poly_id: *ix,
                mesh_id: self.mesh_id,
            })
            .collect()
    }

    pub fn all_polygons<'b>(&'a self) -> Vec<UnrefPoly>
    where
        'a: 'b,
    {
        self.mesh()
            .polies
            .keys()
            .map(|&poly_id| UnrefPoly {
                poly_id,
                mesh_id: self.mesh_id,
            })
            .collect()
    }

    pub fn back_of(self, mesh_ref: MeshRef<'a>) -> Vec<UnrefPoly> {
        self.geo_index.select_polygons(
            self.mesh_id,
            mesh_ref.mesh_id,
            super::index::PolygonFilter::Back,
        )
    }

    pub fn front_of(self, mesh_ref: MeshRef<'a>) -> Vec<UnrefPoly> {
        self.geo_index.select_polygons(
            self.mesh_id,
            mesh_ref.mesh_id,
            super::index::PolygonFilter::Front,
        )
    }

    pub fn shared_with(&self, mesh_ref: MeshRef<'_>) -> Vec<UnrefPoly> {
        self.geo_index.select_polygons(
            self.mesh_id,
            mesh_ref.mesh_id,
            super::index::PolygonFilter::Shared,
        )
    }

    fn mesh(&self) -> &Mesh {
        &self.geo_index.meshes[&self.mesh_id]
    }

    pub(crate) fn face_poly_map(&self) -> HashMap<FaceId, UnrefPoly> {
        self.all_polygons()
            .into_iter()
            .map(|p| (p.make_ref(self.geo_index).face_id(), p))
            .collect()
    }
}

#[derive(Debug)]
pub struct MeshRefMut<'a> {
    pub(super) geo_index: &'a mut GeoIndex,
    pub(super) mesh_id: MeshId,
}

impl<'a> MeshRefMut<'a> {
    pub fn remove(&'a mut self) {
        for p in self.all_polygons() {
            p.make_mut_ref(self.geo_index).remove();
        }
        self.geo_index.meshes.remove(&self.mesh_id);
    }

    pub fn add_polygon<F>(&mut self, p: &[Vector3<F>]) -> anyhow::Result<()>
    where
        F: Into<Dec> + nalgebra::Scalar + nalgebra::Field + Copy,
    {
        self.geo_index.add_polygon_to_mesh(p, self.mesh_id)
    }

    pub fn back_of(&self, mesh_ref: MeshRef<'_>) -> Vec<UnrefPoly> {
        self.geo_index.select_polygons(
            self.mesh_id,
            mesh_ref.mesh_id,
            super::index::PolygonFilter::Back,
        )
    }

    pub fn front_of(&self, mesh_ref: MeshRefMut<'_>) -> Vec<UnrefPoly> {
        self.geo_index.select_polygons(
            self.mesh_id,
            mesh_ref.mesh_id,
            super::index::PolygonFilter::Front,
        )
    }

    fn mesh_obj(&self) -> &Mesh {
        &self.geo_index.meshes[&self.mesh_id]
    }

    fn all_polygons(&mut self) -> Vec<UnrefPoly> {
        let items = self.mesh_obj().polies.keys().copied().collect_vec();
        let mesh_id = self.mesh_id;
        items
            .into_iter()
            .map(move |ix| UnrefPoly {
                poly_id: ix,
                mesh_id,
            })
            .collect()
    }
}

impl<'a> GeoObject<'a> for MeshId {
    type Ref = MeshRef<'a>;

    type MutRef = MeshRefMut<'a>;

    fn make_ref(&self, index: &'a GeoIndex) -> Self::Ref {
        MeshRef {
            geo_index: index,
            mesh_id: *self,
        }
    }

    fn make_mut_ref(&self, index: &'a mut GeoIndex) -> Self::MutRef {
        MeshRefMut {
            geo_index: index,
            mesh_id: *self,
        }
    }
}

impl<'a> UnRef<'a> for MeshRef<'a> {
    type Obj = MeshId;

    fn un_ref(self) -> Self::Obj {
        self.mesh_id
    }
}

impl<'a> UnRef<'a> for MeshRefMut<'a> {
    type Obj = MeshId;

    fn un_ref(self) -> Self::Obj {
        self.mesh_id
    }
}
