use anyhow::anyhow;
use rust_decimal_macros::dec;
use std::cmp::Ordering;

use itertools::Itertools;
use nalgebra::Vector3;
use num_traits::{One, Signed, Zero};
use rust_decimal::Decimal;

use crate::{
    button::Button, button_collections::ButtonsCollection, buttons_column::ButtonsColumn,
    next_and_peek::NextAndPeekBlank,
};
use geometry::{
    decimal::Dec,
    geometry::Geometry,
    hull::Hull,
    indexes::geo_index::{index::GeoIndex, mesh::MeshId},
    origin::Origin,
    path::{bezier::BezierEdge, segment::EdgeSegment, Path, SomePath},
    primitives::PointInPlane,
    stiching::HullEdgeItem,
    surface::{
        topology::{Four, Three},
        tri_bezier::TriBezier,
        SurfaceBetweenTwoEdgePaths,
    },
};

#[derive(Debug)]
pub struct EdgeCoefficients {
    main_inner: Dec,
    main_outer: Dec,
    thumb_inner: Dec,
    thumb_outer: Dec,
}
#[derive(Debug)]
pub struct KeyboardConfig {
    main_button_surface: ButtonsCollection,
    base: Origin,
    thumb_cluster: ButtonsCollection,
    wall_thickness: Dec,
    wall_extension: Dec,
    transition_curvature: EdgeCoefficients,
    bolts: Vec<Bolt>,
}

impl KeyboardConfig {
    pub fn distance_thumb_main_x(&self) -> Option<Dec> {
        let left = self.main_button_surface.left_column()?;
        let right = self.thumb_cluster.right_column()?;
        let leftmost = left
            .buttons()
            .flat_map(|b| {
                vec![
                    b.inner_left_top(self.wall_thickness),
                    b.inner_left_bottom(self.wall_thickness),
                    b.outer_left_top(self.wall_thickness),
                    b.outer_left_bottom(self.wall_thickness),
                ]
                .into_iter()
            })
            .min_by(|p1, p2| p1.x.total_cmp(&p2.x));

        let rightmost = right
            .buttons()
            .flat_map(|b| {
                vec![
                    b.inner_right_top(self.wall_thickness),
                    b.inner_right_bottom(self.wall_thickness),
                    b.outer_right_top(self.wall_thickness),
                    b.outer_right_bottom(self.wall_thickness),
                ]
                .into_iter()
            })
            .max_by(|p1, p2| p1.x.total_cmp(&p2.x));

        leftmost.and_then(|l| rightmost.map(|r| (r - l).x.abs()))
    }

    pub fn thumb_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.thumb_cluster.buttons()
    }

    pub fn main_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.main_button_surface.buttons()
    }

    pub fn buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        Box::new(self.main_buttons().chain(self.thumb_buttons()))
    }

    fn main_right_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.main_button_surface.right_buttons()
    }

    fn main_left_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.main_button_surface.left_buttons()
    }
    fn to_path<P: Into<SomePath>>(edge: HullEdgeItem<P>) -> HullEdgeItem<SomePath> {
        HullEdgeItem {
            inner: edge.inner.into(),
            outer: edge.outer.into(),
        }
    }

    fn edge_around(&self) -> impl Iterator<Item = HullEdgeItem<SomePath>> + '_ {
        self.main_thumb_bottom_transition()
            .map(Self::to_path)
            .chain(self.thumb_bottom_edge().map(Self::to_path))
            .chain(self.thumb_left_edge().map(Self::to_path))
            .chain(self.thumb_top_edge().map(Self::to_path))
            .chain(self.thumb_main_top_transition().map(Self::to_path))
            .chain(self.main_top_edge().map(Self::to_path))
            .chain(self.main_right_edge().map(Self::to_path))
            .chain(self.main_bottom_edge().map(Self::to_path))
    }

    pub fn get_base_plane_projection<T: Path>(
        &self,
        surface_edge: &HullEdgeItem<T>,
    ) -> HullEdgeItem<SomePath> {
        let height_from = Dec::one() / 3
            * (surface_edge.outer.get_t(Dec::zero())
                - self.base.project(surface_edge.outer.get_t(Dec::zero())))
            .dot(&self.base.z());

        let height_to = Dec::one() / 3
            * (surface_edge.outer.get_t(Dec::one())
                - self.base.project(surface_edge.outer.get_t(Dec::one())))
            .dot(&self.base.z());
        let outer_proj = EdgeSegment {
            from: self.base.project(
                surface_edge.outer.get_t(Dec::zero())
                    + surface_edge.outer.get_edge_dir(Dec::zero())
                        * (self.wall_thickness + self.wall_extension),
            ),
            to: self.base.project(
                surface_edge.outer.get_t(Dec::one())
                    + surface_edge.outer.get_edge_dir(Dec::one())
                        * (self.wall_thickness + self.wall_extension),
            ),
            edge_from: self.base.z() * height_from,
            edge_to: self.base.z() * height_to,
        };
        let inner_proj = EdgeSegment {
            from: self.base.project(
                surface_edge.inner.get_t(Dec::zero())
                    + surface_edge.inner.get_edge_dir(Dec::zero()) * self.wall_extension,
            ),
            to: self.base.project(
                surface_edge.inner.get_t(Dec::one())
                    + surface_edge.inner.get_edge_dir(Dec::one()) * self.wall_extension,
            ),

            edge_from: self.base.z() * height_from,
            edge_to: self.base.z() * height_to,
        };

        HullEdgeItem {
            inner: inner_proj.into(),
            outer: outer_proj.into(),
        }
    }

    pub fn build_total_wall(&self, index: &mut GeoIndex) -> anyhow::Result<MeshId> {
        let mut edge_items = self.edge_around().peekable();
        let mut mesh: Option<MeshId> = None;
        while let Some(surface_edge) = edge_items.next() {
            dbg!("---");
            let base_plane = self.get_base_plane_projection(&surface_edge);
            let inner = SurfaceBetweenTwoEdgePaths::new(
                surface_edge.inner.clone(),
                base_plane.inner.clone(),
            );
            let outer = SurfaceBetweenTwoEdgePaths::new(
                surface_edge.outer.clone(),
                base_plane.outer.clone(),
            );
            let hull: Hull<Four> = (inner, outer).try_into()?;
            if let Some(body) = mesh.take() {
                let new_mesh =
                    index.save_mesh(hull.polygonize()?.into_iter().map(std::borrow::Cow::Owned));

                let mut result = index.get_mutable_mesh(body).boolean_union(new_mesh);
                if result.len() > 1 {
                    panic!("no union");
                }
                mesh = Some(*result.remove(0));
            } else {
                let new_mesh =
                    index.save_mesh(hull.polygonize()?.into_iter().map(std::borrow::Cow::Owned));
                mesh = Some(new_mesh)
            }

            if let Some(next_surface_edge) = edge_items.peek() {
                let cur_edge_dir = surface_edge.outer.get_edge_dir(Dec::one());
                let next_edge_dir = next_surface_edge.outer.get_edge_dir(Dec::zero());
                let d = cur_edge_dir.dot(&next_edge_dir);
                let height_current = Dec::one() / 3
                    * (surface_edge.outer.get_t(Dec::one())
                        - self.base.project(surface_edge.outer.get_t(Dec::one())))
                    .dot(&self.base.z());

                let height_next = Dec::one() / 3
                    * (next_surface_edge.outer.get_t(Dec::zero())
                        - self
                            .base
                            .project(next_surface_edge.outer.get_t(Dec::zero())))
                    .dot(&self.base.z());
                if d < Dec::from(0.5) {
                    let next_base_plane = self.get_base_plane_projection(next_surface_edge);
                    let next_edge_dir_proj = self.base.project_unit(next_edge_dir);
                    let cur_edge_dir_proj = self.base.project_unit(cur_edge_dir);
                    let a_inner = next_base_plane.inner.get_t(Dec::zero());
                    let a_outer = next_base_plane.outer.get_t(Dec::zero());
                    let b_inner = base_plane.inner.get_t(Dec::one());
                    let b_outer = base_plane.outer.get_t(Dec::one());
                    let c_inner = surface_edge.inner.get_t(Dec::one());
                    let c_outer = surface_edge.outer.get_t(Dec::one());

                    let base_bezier_inner = BezierEdge::new_simple([
                        a_inner,
                        a_inner + cur_edge_dir_proj,
                        b_inner + next_edge_dir_proj,
                        b_inner,
                    ]);
                    let next_bezier_inner = BezierEdge::new_simple([
                        a_inner,
                        a_inner + self.base.z() * height_current,
                        c_inner + next_edge_dir,
                        c_inner,
                    ]);
                    let cur_bezier_inner = BezierEdge::new_simple([
                        b_inner,
                        b_inner + self.base.z() * height_next,
                        c_inner + cur_edge_dir,
                        c_inner,
                    ]);

                    let base_bezier_outer = BezierEdge::new_simple([
                        a_outer,
                        a_outer + cur_edge_dir_proj,
                        b_outer + next_edge_dir_proj,
                        b_outer,
                    ]);
                    let next_bezier_outer = BezierEdge::new_simple([
                        a_outer,
                        a_outer + self.base.z() * height_current,
                        c_outer + next_edge_dir,
                        c_outer,
                    ]);
                    let cur_bezier_outer = BezierEdge::new_simple([
                        b_outer,
                        b_outer + self.base.z() * height_next,
                        c_outer + cur_edge_dir,
                        c_outer,
                    ]);

                    let inner_bezier = TriBezier::new_from_bezier(
                        [base_bezier_inner, next_bezier_inner, cur_bezier_inner],
                        Dec::one(),
                    );
                    let outer_bezier = TriBezier::new_from_bezier(
                        [base_bezier_outer, next_bezier_outer, cur_bezier_outer],
                        Dec::one(),
                    );
                    let hull: Hull<Three> = (inner_bezier, outer_bezier).try_into()?;
                    if let Some(body) = mesh.take() {
                        let new_poly = index
                            .save_mesh(hull.polygonize()?.into_iter().map(std::borrow::Cow::Owned));

                        let mut result = index.get_mutable_mesh(body).boolean_union(new_poly);
                        if result.len() > 1 {
                            panic!("no union");
                        }
                        mesh = Some(*result.remove(0));
                    } else {
                        let new_mesh = index
                            .save_mesh(hull.polygonize()?.into_iter().map(std::borrow::Cow::Owned));
                        mesh = Some(new_mesh)
                    }
                }
            }
        }
        mesh.ok_or(anyhow!("Mesh is not created"))
    }

    fn thumb_main_top_transition(&self) -> impl Iterator<Item = HullEdgeItem<BezierEdge>> + '_ {
        let thumb = self.thumb_right_edge().last();
        let main = self.main_left_edge().last();
        let (main_top_force_inner, main_top_force_outer) = self
            .main_top_edge()
            .next()
            .map(|s| (s.inner.edge_from, s.outer.edge_from))
            .unwrap_or_else(|| (self.base.y(), self.base.y()));
        let (thumb_top_force_inner, thumb_top_force_outer) = self
            .thumb_top_edge()
            .last()
            .map(|s| (s.inner.edge_from, s.outer.edge_from))
            .unwrap_or_else(|| (self.base.y(), self.base.y()));

        [(thumb, main)]
            .into_iter()
            .filter_map(move |items| match items {
                (Some(thumb), Some(main)) => {
                    let inner_line = [
                        thumb.inner.to,
                        thumb.inner.to
                            + thumb.inner.edge_to * self.transition_curvature.thumb_inner,
                        main.inner.to + main.inner.edge_to * self.transition_curvature.main_inner,
                        main.inner.to,
                    ];
                    let inner_force = [
                        thumb_top_force_inner,
                        thumb_top_force_inner,
                        main_top_force_inner,
                        main_top_force_inner,
                    ];
                    let inner: BezierEdge = BezierEdge::new(inner_line, inner_force);
                    let outer_line = [
                        thumb.outer.to,
                        thumb.outer.to
                            + thumb.outer.edge_to * self.transition_curvature.thumb_outer,
                        main.outer.to + main.outer.edge_to * self.transition_curvature.main_outer,
                        main.outer.to,
                    ];
                    let outer_force = [
                        thumb_top_force_outer,
                        thumb_top_force_outer,
                        main_top_force_outer,
                        main_top_force_outer,
                    ];
                    let outer: BezierEdge = BezierEdge::new(outer_line, outer_force);
                    let hh = HullEdgeItem { inner, outer };
                    Some(hh)
                }
                _ => None,
            })
    }

    fn main_thumb_bottom_transition(&self) -> impl Iterator<Item = HullEdgeItem<BezierEdge>> + '_ {
        let thumbs = self.thumb_right_edge();
        let mains = self.main_left_edge();

        thumbs.zip(mains).take(1).map(|(thumb, main)| {
            let inner_line = [
                main.inner.from,
                main.inner.from + main.inner.edge_from * self.transition_curvature.main_inner,
                thumb.inner.from + thumb.inner.edge_from * self.transition_curvature.thumb_inner,
                thumb.inner.from,
            ];
            let inner_force = [
                -self.base.y(),
                -self.base.y(),
                -self.base.y(),
                -self.base.y(),
            ];
            let inner: BezierEdge = BezierEdge::new(inner_line, inner_force);
            let outer_line = [
                main.outer.from,
                main.outer.from + main.outer.edge_from * self.transition_curvature.main_outer,
                thumb.outer.from + thumb.outer.edge_from * self.transition_curvature.thumb_outer,
                thumb.outer.from,
            ];
            let outer_force = [
                -self.base.y(),
                -self.base.y(),
                -self.base.y(),
                -self.base.y(),
            ];
            let outer: BezierEdge = BezierEdge::new(outer_line, outer_force);

            HullEdgeItem { inner, outer }
        })
    }

    fn thumb_bottom_edge(&self) -> impl Iterator<Item = HullEdgeItem<EdgeSegment>> + '_ {
        self.thumb_bottom_buttons()
            .map(|mut b| {
                b.origin.add(&self.base);
                b
            })
            .sorted_by(sorted_along_vec(self.base.x()))
            .flat_map(|btn| {
                vec![
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_right_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.y()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_right_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.y()),
                        },
                    },
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_left_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.y()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_left_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.y()),
                        },
                    },
                ]
            })
            .next_and_peek(|hull_point, hull_point_next| {
                let inner_segment = EdgeSegment {
                    from: hull_point.inner.point,
                    to: hull_point_next.inner.point,
                    edge_from: hull_point.inner.dir.expect("we placed it"),
                    edge_to: hull_point_next.inner.dir.expect("we placed it"),
                };
                let outer_segment = EdgeSegment {
                    from: hull_point.outer.point,
                    to: hull_point_next.outer.point,
                    edge_from: hull_point.outer.dir.expect("we placed it"),
                    edge_to: hull_point_next.outer.dir.expect("we placed it"),
                };
                HullEdgeItem {
                    inner: inner_segment,
                    outer: outer_segment,
                }
            })
    }
    fn main_bottom_edge(&self) -> impl Iterator<Item = HullEdgeItem<EdgeSegment>> + '_ {
        self.main_bottom_buttons()
            .map(|mut b| {
                b.origin.add(&self.base);
                b
            })
            .sorted_by(sorted_along_vec(self.base.x()))
            .flat_map(|btn| {
                vec![
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_right_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.y()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_right_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.y()),
                        },
                    },
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_left_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.y()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_left_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.y()),
                        },
                    },
                ]
            })
            .next_and_peek(|hull_point, hull_point_next| {
                let inner_segment = EdgeSegment {
                    from: hull_point.inner.point,
                    to: hull_point_next.inner.point,
                    edge_from: hull_point.inner.dir.expect("we placed it"),
                    edge_to: hull_point_next.inner.dir.expect("we placed it"),
                };
                let outer_segment = EdgeSegment {
                    from: hull_point.outer.point,
                    to: hull_point_next.outer.point,
                    edge_from: hull_point.outer.dir.expect("we placed it"),
                    edge_to: hull_point_next.outer.dir.expect("we placed it"),
                };
                HullEdgeItem {
                    inner: inner_segment,
                    outer: outer_segment,
                }
            })
    }

    fn main_right_edge(&self) -> impl Iterator<Item = HullEdgeItem<EdgeSegment>> + '_ {
        self.main_right_buttons()
            .map(|mut b| {
                b.origin.add(&self.base);
                b
            })
            .sorted_by(sorted_along_vec(self.base.y()))
            .flat_map(|btn| {
                vec![
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_right_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.x()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_right_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.x()),
                        },
                    },
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_right_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.x()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_right_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.x()),
                        },
                    },
                ]
            })
            .next_and_peek(|hull_point, hull_point_next| {
                let inner_segment = EdgeSegment {
                    from: hull_point.inner.point,
                    to: hull_point_next.inner.point,
                    edge_from: hull_point.inner.dir.expect("we placed it"),
                    edge_to: hull_point_next.inner.dir.expect("we placed it"),
                };
                let outer_segment = EdgeSegment {
                    from: hull_point.outer.point,
                    to: hull_point_next.outer.point,
                    edge_from: hull_point.outer.dir.expect("we placed it"),
                    edge_to: hull_point_next.outer.dir.expect("we placed it"),
                };
                HullEdgeItem {
                    inner: inner_segment,
                    outer: outer_segment,
                }
            })
    }
    fn thumb_left_edge(&self) -> impl Iterator<Item = HullEdgeItem<EdgeSegment>> + '_ {
        self.thumb_left_buttons()
            .map(|mut b| {
                b.origin.add(&self.base);
                b
            })
            .sorted_by(sorted_along_vec(-self.base.y()))
            .flat_map(|btn| {
                vec![
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_left_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.x()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_left_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.x()),
                        },
                    },
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_left_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.x()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_left_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.x()),
                        },
                    },
                ]
            })
            .next_and_peek(|hull_point, hull_point_next| {
                let inner_segment = EdgeSegment {
                    from: hull_point.inner.point,
                    to: hull_point_next.inner.point,
                    edge_from: hull_point.inner.dir.expect("we placed it"),
                    edge_to: hull_point_next.inner.dir.expect("we placed it"),
                };
                let outer_segment = EdgeSegment {
                    from: hull_point.outer.point,
                    to: hull_point_next.outer.point,
                    edge_from: hull_point.outer.dir.expect("we placed it"),
                    edge_to: hull_point_next.outer.dir.expect("we placed it"),
                };
                HullEdgeItem {
                    inner: inner_segment,
                    outer: outer_segment,
                }
            })
    }
    fn main_left_edge(&self) -> impl Iterator<Item = HullEdgeItem<EdgeSegment>> + '_ {
        self.main_left_buttons()
            .map(|mut b| {
                b.origin.add(&self.base);
                b
            })
            .sorted_by(sorted_along_vec(-self.base.y()))
            .flat_map(|btn| {
                vec![
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_left_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.x()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_left_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.x()),
                        },
                    },
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_left_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.x()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_left_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(-btn.origin.x()),
                        },
                    },
                ]
            })
            .next_and_peek(|hull_point, hull_point_next| {
                let inner_segment = EdgeSegment {
                    from: hull_point.inner.point,
                    to: hull_point_next.inner.point,
                    edge_from: hull_point.inner.dir.expect("we placed it"),
                    edge_to: hull_point_next.inner.dir.expect("we placed it"),
                };
                let outer_segment = EdgeSegment {
                    from: hull_point.outer.point,
                    to: hull_point_next.outer.point,
                    edge_from: hull_point.outer.dir.expect("we placed it"),
                    edge_to: hull_point_next.outer.dir.expect("we placed it"),
                };
                HullEdgeItem {
                    inner: inner_segment,
                    outer: outer_segment,
                }
            })
    }

    fn main_top_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.main_button_surface.top_buttons()
    }

    fn main_top_edge(&self) -> impl Iterator<Item = HullEdgeItem<EdgeSegment>> + '_ {
        self.main_top_buttons()
            .map(|mut b| {
                b.origin.add(&self.base);
                b
            })
            .sorted_by(sorted_along_vec(-self.base.x()))
            .flat_map(|btn| {
                vec![
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_left_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.y()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_left_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.y()),
                        },
                    },
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_right_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.y()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_right_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.y()),
                        },
                    },
                ]
            })
            .next_and_peek(|hull_point, hull_point_next| {
                let inner_segment = EdgeSegment {
                    from: hull_point.inner.point,
                    to: hull_point_next.inner.point,
                    edge_from: hull_point.inner.dir.expect("we placed it"),
                    edge_to: hull_point_next.inner.dir.expect("we placed it"),
                };
                let outer_segment = EdgeSegment {
                    from: hull_point.outer.point,
                    to: hull_point_next.outer.point,
                    edge_from: hull_point.outer.dir.expect("we placed it"),
                    edge_to: hull_point_next.outer.dir.expect("we placed it"),
                };
                HullEdgeItem {
                    inner: inner_segment,
                    outer: outer_segment,
                }
            })
    }

    fn main_bottom_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.main_button_surface.bottom_buttons()
    }

    pub fn thumb_right_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.thumb_cluster.right_buttons()
    }

    fn thumb_right_edge(&self) -> impl Iterator<Item = HullEdgeItem<EdgeSegment>> + '_ {
        self.thumb_cluster
            .right_buttons()
            .map(|mut b| {
                b.origin.add(&self.base);
                b
            })
            .sorted_by(sorted_along_vec(-self.base.y()))
            .flat_map(|btn| {
                vec![
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_right_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.x()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_right_bottom(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.x()),
                        },
                    },
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_right_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.x()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_right_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.x()),
                        },
                    },
                ]
            })
            .next_and_peek(|hull_point, hull_point_next| {
                let inner_segment = EdgeSegment {
                    from: hull_point.inner.point,
                    to: hull_point_next.inner.point,
                    edge_from: hull_point.inner.dir.expect("we placed it"),
                    edge_to: hull_point_next.inner.dir.expect("we placed it"),
                };
                let outer_segment = EdgeSegment {
                    from: hull_point.outer.point,
                    to: hull_point_next.outer.point,
                    edge_from: hull_point.outer.dir.expect("we placed it"),
                    edge_to: hull_point_next.outer.dir.expect("we placed it"),
                };
                HullEdgeItem {
                    inner: inner_segment,
                    outer: outer_segment,
                }
            })
    }
    fn thumb_left_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.thumb_cluster.left_buttons()
    }

    fn thumb_top_edge(&self) -> impl Iterator<Item = HullEdgeItem<EdgeSegment>> + '_ {
        self.thumb_cluster
            .top_buttons()
            .map(|mut b| {
                b.origin.add(&self.base);
                b
            })
            .sorted_by(sorted_along_vec(-self.base.x()))
            .flat_map(|btn| {
                vec![
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_left_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.y()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_left_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.y()),
                        },
                    },
                    HullEdgeItem {
                        inner: PointInPlane {
                            point: btn.inner_right_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.y()),
                        },
                        outer: PointInPlane {
                            point: btn.outer_right_top(self.wall_thickness),
                            normal: btn.origin.z(),
                            dir: Some(btn.origin.y()),
                        },
                    },
                ]
            })
            .next_and_peek(|hull_point, hull_point_next| {
                let inner_segment = EdgeSegment {
                    from: hull_point.inner.point,
                    to: hull_point_next.inner.point,
                    edge_from: hull_point.inner.dir.expect("we placed it"),
                    edge_to: hull_point_next.inner.dir.expect("we placed it"),
                };
                let outer_segment = EdgeSegment {
                    from: hull_point.outer.point,
                    to: hull_point_next.outer.point,
                    edge_from: hull_point.outer.dir.expect("we placed it"),
                    edge_to: hull_point_next.outer.dir.expect("we placed it"),
                };
                HullEdgeItem {
                    inner: inner_segment,
                    outer: outer_segment,
                }
            })
    }

    fn thumb_bottom_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.thumb_cluster.bottom_buttons()
    }

    pub fn simple() -> Self {
        let root = Origin::new().offset_z(Dec::from(dec!(11)));

        let buttons_first = vec![
            Button::chok(
                root.clone()
                    .offset_y(25.into())
                    .rotate_axisangle(Vector3::x() * Angle::Deg(5.into()).rad()),
            ),
            Button::chok(root.clone()),
        ];

        let buttons_second = vec![
            Button::chok(
                root.clone()
                    .offset_y(25.into())
                    .rotate_axisangle(Vector3::x() * Angle::Deg(5.into()).rad()),
            ),
            Button::chok(root.clone()),
        ];

        Self {
            transition_curvature: EdgeCoefficients {
                main_inner: dec!(1).into(),
                main_outer: dec!(1).into(),
                thumb_inner: dec!(1).into(),
                thumb_outer: dec!(1).into(),
            },
            base: root.clone(),
            main_button_surface: ButtonsCollection::new(Origin::new()).with_columns(vec![
                ButtonsColumn::new(root.clone()).chocs(buttons_first.clone()),
                ButtonsColumn::new(root.clone().offset_x(20.into())).chocs(buttons_second.clone()),
            ]),
            thumb_cluster: ButtonsCollection::new(
                root.clone()
                    .offset_x(Dec::from(-20))
                    .offset_z(60.into())
                    .rotate_axisangle(Vector3::y() * Angle::Deg(Dec::from(-60)).rad()),
            )
            .with_columns(vec![
                ButtonsColumn::new(root.clone()).chocs(vec![
                    Button::chok(
                        root.clone()
                            .offset_y(25.into())
                            .rotate_axisangle(Vector3::x() * Angle::Deg(5.into()).rad()),
                    ),
                    Button::chok(root.clone()),
                ]),
                ButtonsColumn::new(root.clone().offset_x(20.into())).chocs(vec![
                    Button::chok(
                        root.clone()
                            .offset_y(25.into())
                            .rotate_axisangle(Vector3::x() * Angle::Deg(5.into()).rad()),
                    ),
                    Button::chok(root.clone()),
                ]),
            ]),
            wall_thickness: Dec::from(2),
            wall_extension: Dec::from(5),
        }
    }
    /*

    pub(crate) fn thumb_right_to_main_left_2(&self) -> anyhow::Result<ScadObject> {
        let thumbs = self.thumb_right_edge().map(|mut h| {
            h.inner.edge_from *= self.transition_curvature.thumb_inner;
            h.inner.edge_to *= self.transition_curvature.thumb_inner;
            h.outer.edge_from *= self.transition_curvature.thumb_outer;
            h.outer.edge_to *= self.transition_curvature.thumb_outer;
            h
        });
        let mains = self.main_left_edge().map(|mut h| {
            h.inner.edge_from *= self.transition_curvature.main_inner;
            h.inner.edge_to *= self.transition_curvature.main_inner;
            h.outer.edge_from *= self.transition_curvature.main_outer;
            h.outer.edge_to *= self.transition_curvature.main_outer;
            h
        });

        let bodies: Vec<FaceCollection> = thumbs
            .zip(mains)
            .map(|(left, right)| StichItem { left, right })
            .map(|si| si.create_body())
            .try_collect()?;
        let mut scad = ScadObject::new(ScadElement::Union);
        for b in bodies {
            scad.add_child(b.make_scad()?)
        }

        Ok(scad)
    }
        pub fn bolts(&self) -> Result<Vec<BoltBuilder>, anyhow::Error> {}

    */
    /*
    pub fn bottom_base(&self) -> Result<ScadObject, anyhow::Error> {
        let polypath = PolyPath::from(
            self.edge_around()
                .with_next(move |item, maybe_next| {
                    let base_plane_projection = self.get_base_plane_projection(&item);
                    let cur_proj = base_plane_projection.outer;
                    if let Some(next) = maybe_next {
                        let cur_edge_dir = item.outer.get_edge_dir(Dec::one());
                        let next_edge_dir = next.outer.get_edge_dir(Dec::zero());
                        let d = cur_edge_dir.dot(&next_edge_dir);
                        if d < 0.5 {
                            let next_plane_projection = self.get_base_plane_projection(next);
                            let a = cur_proj.get_t(Dec::one());
                            let b = next_plane_projection.outer.get_t(Dec::zero());
                            let corner_line =
                                BezierEdge::new_simple([a, a + next_edge_dir, b + cur_edge_dir, b]);
                            return vec![cur_proj, corner_line.into()];
                        }
                    }
                    vec![cur_proj]
                })
                .flatten()
                .collect_vec(),
        );
        let mut line_collection = LineCollection::default();
        line_collection.join(polypath)?;
        let polygon = line_collection.make_scad()?;
        let translated = ScadElement::Translate(Vector3::new(Dec::zero(), Dec::zero(), -Dec::one()));
        let mut scad = ScadObject::new(translated);
        scad.add_child({
            let extruded = ScadElement::LinearExtrude(LinExtrudeParams {
                height: Dec::one(),
                ..Default::default()
            });
            let mut scad = ScadObject::new(extruded);
            scad.add_child(polygon);
            scad
        });
        Ok(scad)
    }

    pub fn between_columns_thumb(&self) -> impl Iterator<Item = ScadObject> + '_ {
        let thickness = self.wall_thickness;
        self.thumb_cluster
            .columns()
            .next_and_peek(move |next, peek| {
                let x = next.origin.x();
                let inner_from = PolyPath::from(
                    next.buttons()
                        .sorted_by(sorted_along_vec(next.origin.y()))
                        .flat_map(move |b| {
                            vec![
                                b.inner_right_top(thickness),
                                b.inner_right_bottom(thickness),
                            ]
                        })
                        .next_and_peek(move |next, peek| EdgeSegment {
                            from: *next,
                            to: *peek,
                            edge_from: x,
                            edge_to: x,
                        })
                        .collect::<Vec<_>>(),
                );
                let outer_from = PolyPath::from(
                    next.buttons()
                        .sorted_by(sorted_along_vec(next.origin.y()))
                        .flat_map(move |b| {
                            vec![
                                b.outer_right_top(thickness),
                                b.outer_right_bottom(thickness),
                            ]
                        })
                        .next_and_peek(move |next, peek| EdgeSegment {
                            from: *next,
                            to: *peek,
                            edge_from: x,
                            edge_to: x,
                        })
                        .collect::<Vec<_>>(),
                );
                let inner_to = PolyPath::from(
                    peek.buttons()
                        .sorted_by(sorted_along_vec(next.origin.y()))
                        .flat_map(move |b| {
                            vec![b.inner_left_top(thickness), b.inner_left_bottom(thickness)]
                        })
                        .next_and_peek(move |next, peek| EdgeSegment {
                            from: *next,
                            to: *peek,
                            edge_from: x,
                            edge_to: x,
                        })
                        .collect::<Vec<_>>(),
                );
                let outer_to = PolyPath::from(
                    peek.buttons()
                        .sorted_by(sorted_along_vec(next.origin.y()))
                        .flat_map(move |b| {
                            vec![b.outer_left_top(thickness), b.outer_left_bottom(thickness)]
                        })
                        .next_and_peek(move |next, peek| EdgeSegment {
                            from: *next,
                            to: *peek,
                            edge_from: x,
                            edge_to: x,
                        })
                        .collect::<Vec<_>>(),
                );

                let left = HullEdgeItem {
                    inner: inner_from,
                    outer: outer_from,
                };
                let right = HullEdgeItem {
                    inner: inner_to,
                    outer: outer_to,
                };
                let stitcher = StichItem { left, right };
                let maybe_body = stitcher.create_body().and_then(|b| b.make_scad());

                if let Err(err) = &maybe_body {
                    dbg!(err);
                }
                maybe_body
            })
            .filter_map(|m| m.ok())
    }
    pub fn between_columns_main(&self) -> impl Iterator<Item = ScadObject> + '_ {
        let thickness = self.wall_thickness;
        self.main_button_surface
            .columns()
            .next_and_peek(move |next, peek| {
                let x = next.origin.x();
                let inner_from = PolyPath::from(
                    next.buttons()
                        .sorted_by(sorted_along_vec(next.origin.y()))
                        .flat_map(move |b| {
                            vec![
                                b.inner_right_top(thickness),
                                b.inner_right_bottom(thickness),
                            ]
                        })
                        .next_and_peek(move |next, peek| EdgeSegment {
                            from: *next,
                            to: *peek,
                            edge_from: x,
                            edge_to: x,
                        })
                        .collect::<Vec<_>>(),
                );
                let outer_from = PolyPath::from(
                    next.buttons()
                        .sorted_by(sorted_along_vec(next.origin.y()))
                        .flat_map(move |b| {
                            vec![
                                b.outer_right_top(thickness),
                                b.outer_right_bottom(thickness),
                            ]
                        })
                        .next_and_peek(move |next, peek| EdgeSegment {
                            from: *next,
                            to: *peek,
                            edge_from: x,
                            edge_to: x,
                        })
                        .collect::<Vec<_>>(),
                );
                let inner_to = PolyPath::from(
                    peek.buttons()
                        .sorted_by(sorted_along_vec(next.origin.y()))
                        .flat_map(move |b| {
                            vec![b.inner_left_top(thickness), b.inner_left_bottom(thickness)]
                        })
                        .next_and_peek(move |next, peek| EdgeSegment {
                            from: *next,
                            to: *peek,
                            edge_from: x,
                            edge_to: x,
                        })
                        .collect::<Vec<_>>(),
                );
                let outer_to = PolyPath::from(
                    peek.buttons()
                        .sorted_by(sorted_along_vec(next.origin.y()))
                        .flat_map(move |b| {
                            vec![b.outer_left_top(thickness), b.outer_left_bottom(thickness)]
                        })
                        .next_and_peek(move |next, peek| EdgeSegment {
                            from: *next,
                            to: *peek,
                            edge_from: x,
                            edge_to: x,
                        })
                        .collect::<Vec<_>>(),
                );

                let left = HullEdgeItem {
                    inner: inner_from,
                    outer: outer_from,
                };
                let right = HullEdgeItem {
                    inner: inner_to,
                    outer: outer_to,
                };
                let stitcher = StichItem { left, right };
                let maybe_body = stitcher.create_body().and_then(|b| b.make_scad());

                if let Err(err) = &maybe_body {
                    dbg!(err);
                }
                maybe_body
            })
            .filter_map(|m| m.ok())
    }
    pub fn between_buttons_in_columns(&self) -> impl Iterator<Item = ScadObject> + '_ {
        self.main_button_surface
            .columns()
            .chain(self.thumb_cluster.columns())
            .flat_map(|c| {
                let column_axis = c.origin.y().to_owned();
                let thickness = self.wall_thickness;
                c.buttons()
                    .sorted_by(sorted_along_vec(-column_axis))
                    .next_and_peek(move |next, peek| {
                        let hull_edge_lower = HullEdgeItem {
                            inner: EdgeSegment {
                                from: next.inner_left_top(thickness),
                                to: next.inner_right_top(thickness),
                                edge_from: column_axis,
                                edge_to: column_axis,
                            },
                            outer: EdgeSegment {
                                from: next.outer_left_top(thickness),
                                to: next.outer_right_top(thickness),
                                edge_from: column_axis,
                                edge_to: column_axis,
                            },
                        };
                        let hull_edge_upper = HullEdgeItem {
                            inner: EdgeSegment {
                                from: peek.inner_left_bottom(thickness),
                                to: peek.inner_right_bottom(thickness),
                                edge_from: -column_axis,
                                edge_to: -column_axis,
                            },
                            outer: EdgeSegment {
                                from: peek.outer_left_bottom(thickness),
                                to: peek.outer_right_bottom(thickness),
                                edge_from: -column_axis,
                                edge_to: -column_axis,
                            },
                        };
                        let stitcher = StichItem {
                            left: hull_edge_lower,
                            right: hull_edge_upper,
                        };

                        let maybe_body = stitcher.create_body().and_then(|b| b.make_scad());
                        if let Err(e) = &maybe_body {
                            dbg!(e);
                        }
                        maybe_body
                    })
                    .filter_map(|mb| mb.ok())
            })
    }
    */
}
fn sorted_along_vec(axis: Vector3<Dec>) -> impl Fn(&Button, &Button) -> Ordering {
    move |a: &Button, b: &Button| {
        let a = a.origin.center.dot(&axis);
        let b = b.origin.center.dot(&axis);
        b.total_cmp(&a)
    }
}
