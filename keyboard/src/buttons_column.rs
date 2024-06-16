
use geometry::{
    decimal::Dec,
    geometry::Geometry,
    hyper_path::{
        hyper_line::HyperLine,
        hyper_point::SuperPoint,
        hyper_surface::{
            simple_dynamic_surface::SimpleSurface,
        },
    },
    origin::Origin,
};

use crate::{buttons_column_builder::ButtonsColumnBuilder, next_and_peek::NextAndPeekBlank};

use super::button::Button;

#[derive(Clone, Debug)]
pub struct ButtonsColumn {
    pub(super) buttons: Vec<Button>,
}

impl ButtonsColumn {
    pub fn build() -> ButtonsColumnBuilder {
        ButtonsColumnBuilder::default()
    }

    pub(crate) fn buttons(&self) -> impl DoubleEndedIterator<Item = &Button> {
        self.buttons.iter()
    }

    pub(crate) fn top(&self) -> Option<Button> {
        self.buttons.last().cloned()
    }

    pub(crate) fn bottom(&self) -> Option<Button> {
        self.buttons.first().cloned()
    }

    pub(crate) fn apply_origin(&mut self, origin: &Origin) {
        for b in self.buttons.iter_mut() {
            b.origin.apply(origin);
        }
    }

    pub fn left_line_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.buttons.iter().flat_map(move |b| {
            [
                SuperPoint {
                    point: b.inner_left_bottom(thickness),
                    side_dir: -b.origin.x() * b.inner_left_bottom_edge.x,
                },
                SuperPoint {
                    point: b.inner_left_top(thickness),
                    side_dir: -b.origin.x() * b.inner_left_top_edge.x,
                },
            ]
        })
    }

    pub fn top_line_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.top().into_iter().flat_map(move |b| {
            [
                SuperPoint {
                    point: b.inner_left_top(thickness),
                    side_dir: b.origin.y() * b.inner_left_top_edge.y,
                },
                SuperPoint {
                    point: b.inner_right_top(thickness),
                    side_dir: b.origin.y() * b.inner_right_top_edge.y,
                },
            ]
        })
    }

    pub fn bottom_line_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.bottom().into_iter().flat_map(move |b| {
            [
                SuperPoint {
                    point: b.inner_left_bottom(thickness),
                    side_dir: -b.origin.y() * b.inner_left_bottom_edge.y,
                },
                SuperPoint {
                    point: b.inner_right_bottom(thickness),
                    side_dir: -b.origin.y() * b.inner_right_bottom_edge.y,
                },
            ]
        })
    }

    pub fn right_line_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.buttons.iter().rev().flat_map(move |b| {
            [
                SuperPoint {
                    point: b.inner_right_top(thickness),
                    side_dir: b.origin.x() * b.inner_right_top_edge.x,
                },
                SuperPoint {
                    point: b.inner_right_bottom(thickness),
                    side_dir: b.origin.x() * b.inner_right_bottom_edge.x,
                },
            ]
        })
    }

    pub fn left_line_outer(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.buttons.iter().flat_map(move |b| {
            [
                SuperPoint {
                    point: b.outer_left_bottom(thickness),
                    side_dir: -b.origin.x() * b.outer_left_bottom_edge.x,
                },
                SuperPoint {
                    point: b.outer_left_top(thickness),
                    side_dir: -b.origin.x() * b.outer_left_top_edge.x,
                },
            ]
        })
    }

    pub fn top_line_outer(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.top().into_iter().flat_map(move |b| {
            [
                SuperPoint {
                    point: b.outer_left_top(thickness),
                    side_dir: b.origin.y() * b.outer_left_top_edge.y,
                },
                SuperPoint {
                    point: b.outer_right_top(thickness),
                    side_dir: b.origin.y() * b.outer_right_top_edge.y,
                },
            ]
        })
    }

    pub fn bottom_line_outer(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.bottom().into_iter().flat_map(move |b| {
            [
                SuperPoint {
                    point: b.outer_left_bottom(thickness),
                    side_dir: -b.origin.y() * b.outer_left_bottom_edge.y,
                },
                SuperPoint {
                    point: b.outer_right_bottom(thickness),
                    side_dir: -b.origin.y() * b.outer_right_bottom_edge.y,
                },
            ]
        })
    }

    pub fn right_line_outer(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.buttons.iter().rev().flat_map(move |b| {
            [
                SuperPoint {
                    point: b.outer_right_top(thickness),
                    side_dir: b.origin.x() * b.outer_right_top_edge.x,
                },
                SuperPoint {
                    point: b.outer_right_bottom(thickness),
                    side_dir: b.origin.x() * b.outer_right_bottom_edge.x,
                },
            ]
        })
    }

    pub fn right_bottom_corner_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.bottom().into_iter().flat_map(move |b| {
            [
                SuperPoint {
                    point: b.inner_right_bottom(thickness),
                    side_dir: b.origin.x() * b.inner_right_bottom_edge.x,
                },
                SuperPoint {
                    point: b.inner_right_bottom(thickness),
                    side_dir: -b.origin.y() * b.inner_right_bottom_edge.y,
                },
            ]
        })
    }

    pub(crate) fn filler_inner(
        &self,
        index: &mut geometry::indexes::geo_index::index::GeoIndex,
        thickness: Dec,
    ) -> anyhow::Result<()> {
        for s in self.buttons().next_and_peek(move |p, n| {
            let top_btn_hl = HyperLine::new_2(
                SuperPoint {
                    side_dir: p.origin.y() * p.inner_right_top_edge.y,
                    point: p.inner_right_top(thickness),
                },
                SuperPoint {
                    side_dir: p.origin.y() * p.inner_left_top_edge.y,
                    point: p.inner_left_top(thickness),
                },
            );
            let bottom_btn_hl = HyperLine::new_2(
                SuperPoint {
                    side_dir: -n.origin.y() * n.inner_right_bottom_edge.y,
                    point: n.inner_right_bottom(thickness),
                },
                SuperPoint {
                    side_dir: -n.origin.y() * n.inner_left_bottom_edge.y,
                    point: n.inner_left_bottom(thickness),
                },
            );
            SimpleSurface::new(bottom_btn_hl, top_btn_hl)
        }) {
            s.polygonize(index, 1)?;
        }
        Ok(())
    }

    pub(crate) fn filler_outer(
        &self,
        index: &mut geometry::indexes::geo_index::index::GeoIndex,
        thickness: Dec,
    ) -> anyhow::Result<()> {
        for s in self.buttons().next_and_peek(move |p, n| {
            let top_btn_hl = HyperLine::new_2(
                SuperPoint {
                    side_dir: p.origin.y() * p.outer_right_top_edge.y,
                    point: p.outer_right_top(thickness),
                },
                SuperPoint {
                    side_dir: p.origin.y() * p.outer_left_top_edge.y,
                    point: p.outer_left_top(thickness),
                },
            );
            let bottom_btn_hl = HyperLine::new_2(
                SuperPoint {
                    side_dir: -n.origin.y() * n.outer_right_bottom_edge.y,
                    point: n.outer_right_bottom(thickness),
                },
                SuperPoint {
                    side_dir: -n.origin.y() * n.outer_left_bottom_edge.y,
                    point: n.outer_left_bottom(thickness),
                },
            );
            SimpleSurface::new(top_btn_hl, bottom_btn_hl)
        }) {
            s.polygonize(index, 1)?;
        }
        Ok(())
    }
}
