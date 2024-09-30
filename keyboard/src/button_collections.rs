use geometry::{
    decimal::Dec,
    geometry::Geometry,
    hyper_path::{
        hyper_line::HyperLine,
        hyper_path::{HyperPath, Root},
        hyper_point::SuperPoint,
        hyper_surface::dynamic_surface::DynamicSurface,
    },
    indexes::geo_index::mesh::MeshRefMut,
};

use crate::{
    button::Button, button_collection_builder::ButtonsCollectionBuilder,
    buttons_column::ButtonsColumn, next_and_peek::NextAndPeekBlank,
};

#[derive(Debug)]
#[allow(unused)]
pub struct ButtonsCollection {
    pub(crate) columns: Vec<ButtonsColumn>,
}

impl ButtonsCollection {
    pub fn build() -> ButtonsCollectionBuilder {
        ButtonsCollectionBuilder::default()
    }

    pub(crate) fn empty() -> ButtonsCollection {
        Self {
            columns: Vec::new(),
        }
    }

    pub(crate) fn buttons(&self) -> impl DoubleEndedIterator<Item = &Button> {
        self.columns.iter().flat_map(|col| col.buttons())
    }

    pub fn left_column(&self) -> Option<&ButtonsColumn> {
        self.columns.first()
    }

    pub fn right_column(&self) -> Option<&ButtonsColumn> {
        self.columns.last()
    }

    pub fn left_line_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.columns
            .first()
            .into_iter()
            .flat_map(move |c| c.left_line_inner(thickness))
    }

    pub fn right_line_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.columns
            .last()
            .into_iter()
            .flat_map(move |c| c.right_line_inner(thickness))
    }

    pub fn top_line_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.columns
            .iter()
            .flat_map(move |c| c.top_line_inner(thickness))
    }

    pub fn bottom_line_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.columns
            .iter()
            .flat_map(move |c| c.bottom_line_inner(thickness))
    }

    pub fn left_line_outer(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.columns
            .first()
            .into_iter()
            .flat_map(move |c| c.left_line_outer(thickness))
    }

    pub fn right_line_outer(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.columns
            .last()
            .into_iter()
            .flat_map(move |c| c.right_line_outer(thickness))
    }

    pub fn top_line_outer(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.columns
            .iter()
            .flat_map(move |c| c.top_line_outer(thickness))
    }

    pub fn bottom_line_outer(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.columns
            .iter()
            .flat_map(move |c| c.bottom_line_outer(thickness))
    }
    pub fn right_bottom_corner_inner(
        &self,
        thickness: Dec,
    ) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.columns
            .last()
            .into_iter()
            .flat_map(move |c| c.right_bottom_corner_inner(thickness))
    }

    pub(crate) fn fill_columns(&self, mesh: &mut MeshRefMut, thickness: Dec) -> anyhow::Result<()> {
        for c in &self.columns {
            c.filler_inner(mesh, thickness)?;
            c.filler_outer(mesh, thickness)?;
        }
        Ok(())
    }

    pub(crate) fn fill_between_columns_inner(
        &self,
        mesh: &mut MeshRefMut,
        thickness: Dec,
    ) -> anyhow::Result<()> {
        for c in self.columns.iter().next_and_peek(move |p, n| {
            let right_line = p
                .right_line_inner(thickness)
                .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
                .fold(Root::new(), |hp, l| hp.push_back(l));
            let left_line = n
                .left_line_inner(thickness)
                .rev()
                .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
                .fold(Root::new(), |hp, l| hp.push_back(l));
            DynamicSurface::new(right_line, left_line)
        }) {
            c.polygonize(mesh, 1)?;
        }
        Ok(())
    }

    pub(crate) fn fill_between_columns_outer(
        &self,
        mesh: &mut MeshRefMut,
        thickness: Dec,
    ) -> anyhow::Result<()> {
        for c in self.columns.iter().next_and_peek(move |p, n| {
            let right_line = p
                .right_line_outer(thickness)
                .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
                .fold(Root::new(), |hp, l| hp.push_back(l));
            let left_line = n
                .left_line_outer(thickness)
                .rev()
                .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
                .fold(Root::new(), |hp, l| hp.push_back(l));
            DynamicSurface::new(left_line, right_line)
        }) {
            c.polygonize(mesh, 1)?;
        }
        Ok(())
    }
}
