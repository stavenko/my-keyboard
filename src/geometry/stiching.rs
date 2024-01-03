use std::fmt::Debug;

use anyhow::Ok;

use crate::geometry::{primitives::PointInPlane, surface::SurfaceBetweenTwoPaths};

use super::{
    hull::hull_between_bounded_surfaces::HullBetweenSurfaces,
    path::{bezier::BezierEdge, Path},
    surface::SurfaceBetweenTwoEdgePaths,
    FaceCollection, Geometry,
};

#[derive(Clone, Debug)]
pub struct HullEdgeItem<T> {
    pub inner: T,
    pub outer: T,
}

pub type HullEdgePoints = Vec<HullEdgeItem<PointInPlane<f32>>>;
pub type HullEdgePath<T> = Vec<HullEdgeItem<T>>;

/*
fn hull(
    left_prev: HullEdgeItem<PointInPlane<f32>>,
    right_prev: HullEdgeItem<PointInPlane<f32>>,
    left_next: HullEdgeItem<PointInPlane<f32>>,
    right_next: HullEdgeItem<PointInPlane<f32>>,
) -> HullBetweenBoundedSurfaces<
    4,
    SurfaceBetweenTwoPaths<BezierEdge , BezierEdge >,
    SurfaceBetweenTwoPaths<BezierEdge , BezierEdge >,
> {
    let weights_prev = (left_prev.inner.point - right_prev.inner.point).magnitude() * 0.333;
    let weights_next = (left_next.inner.point - right_next.inner.point).magnitude() * 0.333;

    let prev_inner_bezier = BezierCurve::from_points_with_weights(
        left_prev.inner,
        weights_prev,
        right_prev.inner,
        weights_prev,
    );
    let next_inner_bezier = BezierCurve::from_points_with_weights(
        left_next.inner.to_owned(),
        weights_next,
        right_next.inner.to_owned(),
        weights_next,
    );

    let inner = SurfaceBetweenTwoPaths {
        left: prev_inner_bezier,
        right: next_inner_bezier,
    };
    let weights_prev = (left_prev.outer.point - right_prev.outer.point).magnitude() * 0.333;
    let weights_next = (left_next.outer.point - right_next.outer.point).magnitude() * 0.333;

    let prev_outer_bezier = BezierCurve::from_points_with_weights(
        left_prev.outer,
        weights_prev,
        right_prev.outer,
        weights_prev,
    );

    let next_outer_bezier = BezierCurve::from_points_with_weights(
        left_next.outer.to_owned(),
        weights_next,
        right_next.outer.to_owned(),
        weights_next,
    );

    let outer = SurfaceBetweenTwoPaths {
        right: prev_outer_bezier,
        left: next_outer_bezier,
    };

    HullBetweenBoundedSurfaces::new(outer, inner)
}
*/

/*
fn stitch_topology<const D: usize>(
    left: Vec<HullEdgeItem<PointInPlane<f32>>>,
    right: Vec<HullEdgeItem<PointInPlane<f32>>>,
) -> anyhow::Result<FaceCollection> {
    match (left.len(), right.len()) {
        (1, 1) => Err(anyhow::Error::msg(
            "One to one point gives line, not surface",
        )),

        (l, r) if l == r => {
            let mut points = left.into_iter().zip(right).peekable();
            let mut face_collection = FaceCollection::default();
            while let Some((left_prev, right_prev)) = points.next() {
                if let Some((left_next, right_next)) = points.peek() {
                    let hull = hull(
                        left_prev,
                        right_prev,
                        left_next.to_owned(),
                        right_next.to_owned(),
                    );
                    face_collection.join::<D>(hull)?;
                }
            }

            Ok(face_collection)
        }
        (l, r) => Err(anyhow::Error::msg(format!(
            "Cannot stich together topologies {l}x{r}"
        ))),
    }
}
*/

pub(crate) trait StitchTopology<const D: usize, Other> {
    fn stitch_topology(self, other: Other) -> anyhow::Result<FaceCollection>;
}

/*
impl<const D: usize> StitchTopology<D, Vec<HullEdgeItem<PointInPlane<f32>>>>
    for Vec<HullEdgeItem<PointInPlane<f32>>>
{
    fn stitch_topology(
        self,
        other: Vec<HullEdgeItem<PointInPlane<f32>>>,
    ) -> anyhow::Result<FaceCollection> {
        stitch_topology::<D>(self, other)
    }
}
*/

pub struct StichItem<T> {
    pub left: HullEdgeItem<T>,
    pub right: HullEdgeItem<T>,
}

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

/*

impl<const D: usize, T: Path> StitchTopology<D, Vec<HullEdgeItem<T>>>
    for Vec<HullEdgeItem<PointInPlane<f32>>>
{
    fn stitch_topology(self, other: Vec<HullEdgeItem<T>>) -> anyhow::Result<FaceCollection> {
        match other.len() {
            0 => Err(anyhow::anyhow!("Cannot stitch to empty array")),
            1 => {
                let items = self.len();
                let mut iter = self.into_iter().enumerate().peekable();
                let mut face_collection = FaceCollection::default();
                if let Some(cur_path) = other.first() {
                    while let Some((prev_ix, right_prev)) = iter.next() {
                        if let Some((next_ix, right_next)) = iter.peek() {
                            let prev_t = prev_ix as f32 / (items as f32 - 1.0);
                            let next_t = *next_ix as f32 / (items as f32 - 1.0);

                            let left_prev = HullEdgeItem {
                                inner: PointInPlane {
                                    point: cur_path.inner.get_t(prev_t),
                                    normal: cur_path.inner.get_tangent(prev_t).into_inner(),
                                    dir: None,
                                },
                                outer: PointInPlane {
                                    point: cur_path.outer.get_t(prev_t),
                                    normal: cur_path.outer.get_tangent(prev_t).into_inner(),
                                    dir: None,
                                },
                            };
                            let left_next = HullEdgeItem {
                                inner: PointInPlane {
                                    point: cur_path.inner.get_t(next_t),
                                    normal: cur_path.inner.get_tangent(next_t).into_inner(),
                                    dir: None,
                                },
                                outer: PointInPlane {
                                    point: cur_path.outer.get_t(next_t),
                                    normal: cur_path.outer.get_tangent(next_t).into_inner(),
                                    dir: None,
                                },
                            };
                            let hull = hull(
                                left_prev,
                                right_prev,
                                left_next.to_owned(),
                                right_next.to_owned(),
                            );
                            face_collection.join::<D>(hull)?;
                        }
                    }
                }
                Ok(face_collection)
            }
            _ => Err(anyhow::anyhow!("Only one item supported at the moment")),
        }
    }
}
*/
