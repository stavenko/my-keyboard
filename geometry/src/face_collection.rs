use crate::primitives::Face;

#[derive(Default)]
pub struct FaceCollection {
    faces: Vec<Face>,
}

impl FaceCollection {
    /*
    pub fn join(&mut self, geometry: impl Geometry) -> anyhow::Result<()> {
        self.faces.extend(geometry.polygonize()?);
        Ok(())
    }
    pub fn quantize(face: Face) -> Face {
        let factor = 1e3;
        face.map(|v| (v * factor).map(|c| c.round()) / factor)
    }
    */

    /*
    pub fn make_scad(self) -> anyhow::Result<ScadObject> {
        let mut spatial_index = Index::allocate_default();
        let faces = self
            .faces
            .into_iter()
            // .map(Self::quantize)
            .collect::<Vec<_>>();
        for f in &faces {
            for p in f.iter() {
                spatial_index.insert(*p, 0);
            }
        }
        let spatial_index = spatial_index.rebalance();
        let faces = faces
            .into_par_iter()
            .map(|face| {
                face.into_iter()
                    .map(|point| {
                        spatial_index
                            .get_point_index(&point)
                            .map(|ix| ix as i32)
                            .ok_or(anyhow!(
                                "no such point in index - we lost it somehow {point}"
                            ))
                    })
                    .try_collect::<_, Vec<i32>, _>()
            })
            .collect::<Vec<_>>();
        let faces = faces.into_iter().try_collect()?;

        Ok(ScadObject::new(ScadElement::Polyhedron(
            spatial_index.linearize(),
            faces,
        )))
    }
    */
}
