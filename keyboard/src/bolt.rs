use std::borrow::Cow;

use geometry::{
    basis::Basis,
    decimal::Dec,
    indexes::geo_index::{index::GeoIndex, mesh::MeshId},
    origin::Origin,
    shapes,
};

pub struct Bolt {
    origin: Origin,
    head_diameter: Dec,
    diameter: Dec,
    head_cylinder_height: Dec,
    /// Height of whole bolt without head
    height: Dec,
    nut: Option<Nut>,
}

pub enum Nut {
    Hex { outer_diameter: Dec, height: Dec },
}

impl Bolt {
    pub fn render_on_thread_part(
        &self,
        index: &mut GeoIndex,
        mesh_id: MeshId,
    ) -> anyhow::Result<()> {
        let b = Basis::new(
            self.origin.x(),
            self.origin.y(),
            self.origin.z(),
            self.origin.center,
        )
        .unwrap();
        let cyl = shapes::cylinder(b, self.height, self.diameter / 2, 10);

        index.save_mesh(cyl.into_iter().map(Cow::Owned));
        Ok(())
    }

    pub fn render_on_head_part(&self, index: &mut GeoIndex, mesh_id: MeshId) -> anyhow::Result<()> {
        Ok(())
    }
}
