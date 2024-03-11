use std::fmt::Debug;

#[derive(Clone, Debug)]
pub struct HullEdgeItem<T> {
    pub inner: T,
    pub outer: T,
}

/*
pub(crate) trait StitchTopology<const D: usize, Other> {
    fn stitch_topology(self, other: Other) -> anyhow::Result<FaceCollection>;
}

*/

pub struct StichItem<T> {
    pub left: HullEdgeItem<T>,
    pub right: HullEdgeItem<T>,
}

/*
impl<T: Path + Clone + Debug> StichItem<T> {
    pub fn create_body(self) -> anyhow::Result<FaceCollection> {
        let inner = SurfaceBetweenTwoEdgePaths::new(self.left.inner, self.right.inner);
        let outer = SurfaceBetweenTwoEdgePaths::new(self.left.outer, self.right.outer);

        let hull = HullBetweenSurfaces::new(outer, inner);
        let mut fc = FaceCollection::default();
        fc.join(hull)?;
        Ok(fc)
    }
}
*/
