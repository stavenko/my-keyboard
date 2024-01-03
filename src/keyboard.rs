use std::{cmp::Ordering, f32::consts::PI};

use anyhow::anyhow;
use itertools::Itertools;
use nalgebra::{UnitQuaternion, Vector3};
use scad::{ScadElement, ScadObject};

use crate::{
    geometry::{
        hull::hull_between_bounded_surfaces::HullBetweenSurfaces,
        path::{bezier::BezierEdge, polypath::PolyPath, segment::EdgeSegment, Path, SomePath},
        primitives::{origin::Origin, PointInPlane},
        stiching::{HullEdgeItem, StichItem},
        surface::{topology::Four, tri_bezier::TriBezier, SurfaceBetweenTwoEdgePaths},
        FaceCollection,
    },
    keyboard::button::Button,
};

use self::{
    button_collections::{ButtonsCollection, ButtonsHull},
    buttons_column::ButtonsColumn,
    next_and_peek::NextAndPeekBlank,
};

pub mod button;
pub mod button_collections;
pub mod buttons_column;
pub mod next_and_peek;

pub struct EdgeCoefficients {
    main_inner: f32,
    main_outer: f32,
    thumb_inner: f32,
    thumb_outer: f32,
}
pub(crate) struct KeyboardConfig {
    main_button_surface: ButtonsCollection,
    base: Origin,
    thumb_cluster: ButtonsCollection,
    wall_thickness: f32,
    wall_extension: f32,
    transition_curvature: EdgeCoefficients,
}

pub enum Angle {
    Rad(f32),
    Deg(f32),
}

impl Angle {
    fn rad(&self) -> f32 {
        match self {
            Self::Rad(r) => *r,
            Self::Deg(d) => d * PI / 180.,
        }
    }
}

impl KeyboardConfig {
    fn distance_thumb_main_x(&self) -> Option<f32> {
        let left = self.main_button_surface.left_column()?;
        let right = self.thumb_cluster.right_column()?;
        let leftest = left
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

        let righests = right
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

        leftest.and_then(|l| righests.map(|r| (r - l).x.abs()))
    }
    pub(crate) fn thumb_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.thumb_cluster.buttons()
    }

    pub(crate) fn main_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.main_button_surface.buttons()
    }

    pub(crate) fn buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
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
        let height_from = 0.333
            * (surface_edge.outer.get_t(0.) - self.base.project(surface_edge.outer.get_t(0.)))
                .dot(&self.base.z());

        let height_to = 0.333
            * (surface_edge.outer.get_t(1.) - self.base.project(surface_edge.outer.get_t(1.)))
                .dot(&self.base.z());
        let outer_proj = EdgeSegment {
            from: self.base.project(
                surface_edge.outer.get_t(0.0)
                    + surface_edge.outer.get_edge_dir(0.0)
                        * (self.wall_thickness + self.wall_extension),
            ),
            to: self.base.project(
                surface_edge.outer.get_t(1.0)
                    + surface_edge.outer.get_edge_dir(1.0)
                        * (self.wall_thickness + self.wall_extension),
            ),
            edge_from: self.base.z() * height_from,
            edge_to: self.base.z() * height_to,
        };
        let inner_proj = EdgeSegment {
            from: self.base.project(
                surface_edge.inner.get_t(0.0)
                    + surface_edge.inner.get_edge_dir(0.0) * self.wall_extension,
            ),
            to: self.base.project(
                surface_edge.inner.get_t(1.0)
                    + surface_edge.inner.get_edge_dir(1.0) * self.wall_extension,
            ),

            edge_from: self.base.z() * height_from,
            edge_to: self.base.z() * height_to,
        };

        HullEdgeItem {
            inner: inner_proj.into(),
            outer: outer_proj.into(),
        }
    }

    pub fn build_total_wall(&self) -> Result<ScadObject, anyhow::Error> {
        let mut edge_items = self.edge_around().peekable();
        let mut s = ScadObject::new(ScadElement::Union);
        while let Some(surface_edge) = edge_items.next() {
            let base_plane = self.get_base_plane_projection(&surface_edge);
            let si = StichItem {
                left: surface_edge.clone(),
                right: base_plane.clone(),
            };
            let faces = si.create_body()?;
            s.add_child(faces.make_scad()?);
            if let Some(next_surface_edge) = edge_items.peek() {
                let cur_edge_dir = surface_edge.outer.get_edge_dir(1.0);
                let next_edge_dir = next_surface_edge.outer.get_edge_dir(0.0);
                let d = cur_edge_dir.dot(&next_edge_dir);
                let height_current = 0.333
                    * (surface_edge.outer.get_t(1.)
                        - self.base.project(surface_edge.outer.get_t(1.)))
                    .dot(&self.base.z());

                let height_next = 0.333
                    * (next_surface_edge.outer.get_t(0.)
                        - self.base.project(next_surface_edge.outer.get_t(0.)))
                    .dot(&self.base.z());
                if d < 0.5 {
                    let next_base_plane = self.get_base_plane_projection(next_surface_edge);
                    let a_inner = next_base_plane.inner.get_t(0.0);
                    let a_outer = next_base_plane.outer.get_t(0.0);
                    let b_inner = base_plane.inner.get_t(1.0);
                    let b_outer = base_plane.outer.get_t(1.0);
                    let c_inner = surface_edge.inner.get_t(1.0);
                    let c_outer = surface_edge.outer.get_t(1.0);

                    let base_bezier_inner = BezierEdge::new_simple([
                        a_inner,
                        a_inner + cur_edge_dir,
                        b_inner + next_edge_dir,
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
                        a_outer + cur_edge_dir,
                        b_outer + next_edge_dir,
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
                        1.0,
                    );
                    let outer_bezier = TriBezier::new_from_bezier(
                        [base_bezier_outer, next_bezier_outer, cur_bezier_outer],
                        1.0,
                    );
                    let hull = HullBetweenSurfaces::new(inner_bezier, outer_bezier);
                    let mut faces = FaceCollection::default();
                    faces.join(hull);
                    s.add_child(faces.make_scad()?);
                }
            }
        }
        Ok(s)
    }

    fn thumb_main_top_transition(&self) -> impl Iterator<Item = HullEdgeItem<BezierEdge>> + '_ {
        let thumb = self.thumb_right_edge().last();
        let main = self.main_left_edge().last();
        let main_top_first = self.main_top_edge().next();
        let (main_top_force_inner, main_top_force_outer) = main_top_first
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
                            + self.transition_curvature.thumb_inner * thumb.inner.edge_to,
                        main.inner.to + self.transition_curvature.main_inner * main.inner.edge_to,
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
                            + self.transition_curvature.thumb_outer * thumb.outer.edge_to,
                        main.outer.to + self.transition_curvature.main_outer * main.outer.edge_to,
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
                main.inner.from + self.transition_curvature.main_inner * main.inner.edge_from,
                thumb.inner.from + self.transition_curvature.thumb_inner * thumb.inner.edge_from,
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
                main.outer.from + self.transition_curvature.main_outer * main.outer.edge_from,
                thumb.outer.from + self.transition_curvature.thumb_outer * thumb.outer.edge_from,
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

    fn thumb_right_buttons(&self) -> Box<dyn Iterator<Item = Button> + '_> {
        self.thumb_cluster.right_buttons()
    }

    fn thumb_right_edge(&self) -> impl Iterator<Item = HullEdgeItem<EdgeSegment>> + '_ {
        self.thumb_cluster
            .right_buttons()
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

    pub(crate) fn simple() -> Self {
        let root = Origin::new();
        let buttons_first = vec![
            Button::chok(
                root.clone()
                    .offset_y(25.0)
                    .rotate_axisangle(Vector3::x() * 5.0 / 180.0 * PI),
            ),
            Button::chok(root.clone()),
        ];

        let buttons_second = vec![
            Button::chok(
                root.clone()
                    .offset_y(25.0)
                    .rotate_axisangle(Vector3::x() * 5.0 / 180.0 * PI),
            ),
            Button::chok(root.clone()),
        ];

        Self {
            transition_curvature: EdgeCoefficients {
                main_inner: 12.0,
                main_outer: 9.0,
                thumb_inner: 6f32,
                thumb_outer: 16f32,
            },
            base: Origin::new(),
            main_button_surface: ButtonsCollection::new(root.clone().offset_z(10.0), 2.0).columns(
                vec![
                    ButtonsColumn::new(root.clone()).chocs(buttons_first.clone()),
                    ButtonsColumn::new(root.clone().offset_x(20.0)).chocs(buttons_second.clone()),
                ],
            ),
            thumb_cluster: ButtonsCollection::new(
                root.clone()
                    .offset_x(-30.)
                    .offset_z(10.)
                    .rotate_axisangle(Vector3::y() * Angle::Deg(-60.).rad()),
                2.0,
            )
            .columns(vec![
                ButtonsColumn::new(root.clone()).chocs(vec![
                    Button::chok(
                        root.clone()
                            .offset_y(25.0)
                            .rotate_axisangle(Vector3::x() * 5.0 / 180.0 * PI),
                    ),
                    Button::chok(root.clone()),
                ]),
                ButtonsColumn::new(root.clone().offset_x(20.0)).chocs(vec![
                    Button::chok(
                        root.clone()
                            .offset_y(25.0)
                            .rotate_axisangle(Vector3::x() * 5.0 / 180.0 * PI),
                    ),
                    Button::chok(root.clone()),
                ]),
            ]),
            wall_thickness: 2.0,
            wall_extension: 5.0,
        }
    }

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
}
fn sorted_along_vec(axis: Vector3<f32>) -> impl Fn(&Button, &Button) -> Ordering {
    move |a: &Button, b: &Button| {
        let a = a.origin.center.dot(&axis);
        let b = b.origin.center.dot(&axis);
        b.total_cmp(&a)
    }
}
