
use geometry::{
    decimal::Dec,
    geometry::Geometry,
    hyper_path::{
        hyper_line::{HyperLine, ShiftInPlane},
        hyper_path::{HyperPath, Root},
        hyper_point::{SideDir, SuperPoint},
        hyper_surface::{
            dynamic_surface::DynamicSurface, primitive_dynamic_surface::PrimitiveSurface,
        },
    },
    indexes::geo_index::index::GeoIndex,
};
use nalgebra::Vector3;

use crate::{
    button_collections::ButtonsCollection, keyboard_builder::KeyboardBuilder,
    next_and_peek::NextAndPeekBlank,
};

pub struct RightKeyboardConfig {
    pub(crate) main_buttons: ButtonsCollection,
    pub(crate) thumb_buttons: ButtonsCollection,
    pub(crate) table_outline: Root<SuperPoint<Dec>>,
    pub(crate) main_plane_thickness: Dec,
}

impl RightKeyboardConfig {
    pub fn build() -> KeyboardBuilder {
        KeyboardBuilder::default()
    }

    pub fn right_line_inner(&self) -> impl Iterator<Item = SuperPoint<Dec>> + '_ {
        self.main_buttons
            .right_line_inner(self.main_plane_thickness)
    }

    pub fn right_line_outer(&self) -> impl Iterator<Item = SuperPoint<Dec>> + '_ {
        self.main_buttons
            .right_line_outer(self.main_plane_thickness)
    }

    pub fn left_line_inner(&self) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.thumb_buttons
            .left_line_inner(self.main_plane_thickness)
    }

    pub fn top_line_inner(&self) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.thumb_buttons
            .top_line_inner(self.main_plane_thickness)
            .chain(self.main_buttons.top_line_inner(self.main_plane_thickness))
    }

    pub fn bottom_line_inner(&self) -> impl DoubleEndedIterator<Item = SuperPoint<Dec>> + '_ {
        self.thumb_buttons
            .bottom_line_inner(self.main_plane_thickness)
            .chain(
                self.main_buttons
                    .bottom_line_inner(self.main_plane_thickness),
            )
    }

    pub fn top_thumb_main_connection_inner(&self) -> HyperLine<SuperPoint<Dec>> {
        let main_point_mutual = self
            .main_buttons
            .left_line_inner(self.main_plane_thickness)
            .last()
            .expect("all good");
        let main_point_side_dir = self
            .main_buttons
            .top_line_inner(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_mutual = self
            .thumb_buttons
            .right_line_inner(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_dir = self
            .thumb_buttons
            .top_line_inner(self.main_plane_thickness)
            .next_back()
            .expect("all good");

        let a = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point,
        };

        let b = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point + thumb_point_mutual.side_dir(),
        };

        let c = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point + main_point_mutual.side_dir(),
        };

        let d = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point,
        };

        HyperLine::new_4(a, b, c, d)
    }

    pub fn top_thumb_main_connection_outer(&self) -> HyperLine<SuperPoint<Dec>> {
        let main_point_mutual = self
            .main_buttons
            .left_line_outer(self.main_plane_thickness)
            .last()
            .expect("all good");
        let main_point_side_dir = self
            .main_buttons
            .top_line_outer(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_mutual = self
            .thumb_buttons
            .right_line_outer(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_dir = self
            .thumb_buttons
            .top_line_outer(self.main_plane_thickness)
            .next_back()
            .expect("all good");

        let a = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point,
        };

        let b = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point + thumb_point_mutual.side_dir(),
        };

        let c = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point + main_point_mutual.side_dir(),
        };

        let d = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point,
        };

        HyperLine::new_4(a, b, c, d)
    }

    pub fn bottom_main_thumb_connection_inner(&self) -> HyperLine<SuperPoint<Dec>> {
        let main_point_mutual = self
            .main_buttons
            .left_line_inner(self.main_plane_thickness)
            .next()
            .expect("all good");
        let main_point_side_dir = self
            .main_buttons
            .bottom_line_inner(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_mutual = self
            .thumb_buttons
            .right_line_inner(self.main_plane_thickness)
            .next_back()
            .expect("all good");
        let thumb_point_dir = self
            .thumb_buttons
            .bottom_line_inner(self.main_plane_thickness)
            .next_back()
            .expect("all good");

        let a = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point,
        };
        let b = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point + main_point_mutual.side_dir(),
        };

        let c = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point + thumb_point_mutual.side_dir(),
        };

        let d = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point,
        };

        HyperLine::new_4(a, b, c, d)
    }

    pub fn bottom_main_thumb_connection_outer(&self) -> HyperLine<SuperPoint<Dec>> {
        let main_point_mutual = self
            .main_buttons
            .left_line_outer(self.main_plane_thickness)
            .next()
            .expect("all good");
        let main_point_side_dir = self
            .main_buttons
            .bottom_line_outer(self.main_plane_thickness)
            .next()
            .expect("all good");
        let thumb_point_mutual = self
            .thumb_buttons
            .right_line_outer(self.main_plane_thickness)
            .next_back()
            .expect("all good");
        let thumb_point_dir = self
            .thumb_buttons
            .bottom_line_outer(self.main_plane_thickness)
            .next_back()
            .expect("all good");

        let a = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point,
        };
        let b = SuperPoint {
            side_dir: main_point_side_dir.side_dir,
            point: main_point_mutual.point + main_point_mutual.side_dir(),
        };

        let c = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point + thumb_point_mutual.side_dir(),
        };

        let d = SuperPoint {
            side_dir: thumb_point_dir.side_dir,
            point: thumb_point_mutual.point,
        };

        HyperLine::new_4(a, b, c, d)
    }

    pub fn line_1_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.right_line_inner()
            .chain(
                self.main_buttons
                    .bottom_line_inner(self.main_plane_thickness)
                    .rev(),
            )
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    pub fn line_1_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.right_line_outer()
            .chain(
                self.main_buttons
                    .bottom_line_outer(self.main_plane_thickness)
                    .rev(),
            )
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    pub fn line_2_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        vec![(self.bottom_main_thumb_connection_inner())].into_iter()
    }

    pub fn line_2_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        vec![(self.bottom_main_thumb_connection_outer())].into_iter()
    }

    pub fn line_3_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.thumb_buttons
            .bottom_line_inner(self.main_plane_thickness)
            .rev()
            .chain(
                self.thumb_buttons
                    .left_line_inner(self.main_plane_thickness),
            )
            .chain(self.thumb_buttons.top_line_inner(self.main_plane_thickness))
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    pub fn line_3_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.thumb_buttons
            .bottom_line_outer(self.main_plane_thickness)
            .rev()
            .chain(
                self.thumb_buttons
                    .left_line_outer(self.main_plane_thickness),
            )
            .chain(self.thumb_buttons.top_line_outer(self.main_plane_thickness))
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    pub fn line_4_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        vec![(self.top_thumb_main_connection_inner())].into_iter()
    }

    pub fn line_4_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        vec![(self.top_thumb_main_connection_outer())].into_iter()
    }

    pub fn line_5_inner(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.main_buttons
            .top_line_inner(self.main_plane_thickness)
            .chain(
                self.main_buttons
                    .right_line_inner(self.main_plane_thickness)
                    .take(1),
            )
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    pub fn line_5_outer(&self) -> impl Iterator<Item = HyperLine<SuperPoint<Dec>>> + '_ {
        self.main_buttons
            .top_line_outer(self.main_plane_thickness)
            .chain(
                self.main_buttons
                    .right_line_outer(self.main_plane_thickness)
                    .take(1),
            )
            .next_and_peek(|n, p| HyperLine::new_2(*n, *p))
    }

    pub fn line_around_buttons_inner(&self) -> Root<SuperPoint<Dec>> {
        self.line_1_inner()
            .chain(self.line_2_inner())
            .chain(self.line_3_inner())
            .chain(self.line_4_inner())
            .chain(self.line_5_inner())
            .fold(Root::new(), |hp, l| hp.push_back(l))
    }

    pub fn line_around_buttons_outer(&self) -> Root<SuperPoint<Dec>> {
        self.line_1_outer()
            .chain(self.line_2_outer())
            .chain(self.line_3_outer())
            .chain(self.line_4_outer())
            .chain(self.line_5_outer())
            .fold(Root::new(), |hp, l| hp.push_back(l))
    }

    pub fn inner_wall_surface(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        let outline = self.table_outline.clone();
        let around_buttons = self.line_around_buttons_inner();
        if outline.len() != around_buttons.len() {
            println!(
                "WARNING, OUTLINE AND BUTTONS HAVE DIFFERRENT SIZE {} <> {}",
                outline.len(),
                around_buttons.len()
            );
        }

        DynamicSurface::new(around_buttons, outline).polygonize(index, 8)?;
        Ok(())
    }

    pub fn outer_wall_surface(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        let mut outline = self
            .table_outline
            .clone()
            .map(|l| l.shift_in_plane(Vector3::z(), -self.main_plane_thickness));
        outline.connect_ends_circular();
        let around_buttons = self.line_around_buttons_outer();
        if outline.len() != around_buttons.len() {
            println!(
                "WARNING, OUTLINE AND BUTTONS HAVE DIFFERRENT SIZE {} <> {}",
                outline.len(),
                around_buttons.len()
            );
        }

        DynamicSurface::new(outline, around_buttons).polygonize(index, 8)?;
        Ok(())
    }

    pub fn fill_between_collections(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        let right_line_inner = self
            .thumb_buttons
            .right_line_inner(self.main_plane_thickness)
            .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
            .fold(Root::new(), |hp, l| hp.push_back(l));

        let left_line_inner = self
            .main_buttons
            .left_line_inner(self.main_plane_thickness)
            .rev()
            .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
            .fold(Root::new(), |hp, l| hp.push_back(l));

        let right_line_outer = self
            .thumb_buttons
            .right_line_outer(self.main_plane_thickness)
            .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
            .fold(Root::new(), |hp, l| hp.push_back(l));

        let left_line_outer = self
            .main_buttons
            .left_line_outer(self.main_plane_thickness)
            .rev()
            .next_and_peek(|a, b| HyperLine::new_2(*a, *b))
            .fold(Root::new(), |hp, l| hp.push_back(l));

        DynamicSurface::new(right_line_inner, left_line_inner).polygonize(index, 8)?;
        DynamicSurface::new(left_line_outer, right_line_outer).polygonize(index, 8)?;
        Ok(())
    }

    pub fn fill_between_buttons(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        self.main_buttons
            .fill_columns(index, self.main_plane_thickness)?;
        self.thumb_buttons
            .fill_columns(index, self.main_plane_thickness)?;
        self.main_buttons
            .fill_between_columns_inner(index, self.main_plane_thickness)?;
        self.main_buttons
            .fill_between_columns_outer(index, self.main_plane_thickness)?;
        self.thumb_buttons
            .fill_between_columns_inner(index, self.main_plane_thickness)?;

        self.thumb_buttons
            .fill_between_columns_outer(index, self.main_plane_thickness)?;

        self.fill_between_collections(index)?;

        Ok(())
    }

    pub fn buttons(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        for b in self
            .main_buttons
            .buttons()
            .chain(self.thumb_buttons.buttons())
        {
            b.mesh(index, self.main_plane_thickness)?;
        }

        Ok(())
    }

    pub fn inner_outer_surface_table_connection(&self, index: &mut GeoIndex) -> anyhow::Result<()> {
        let mut outline = self.table_outline.clone();
        let mut shifted_outline = outline
            .clone()
            .map(|l| l.shift_in_plane(Vector3::z(), -self.main_plane_thickness));
        shifted_outline.connect_ends_circular();

        loop {
            let (f, fs) = outline.head_tail();
            let (s, ss) = shifted_outline.head_tail();
            outline = fs;
            shifted_outline = ss;

            PrimitiveSurface(s.to_points(), f.to_points()).polygonize(index, 8)?;
            if outline.len() == 0 {
                break;
            }
        }
        Ok(())
    }
}
