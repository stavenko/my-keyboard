use geometry::{decimal::Dec, origin::Origin};
use itertools::Itertools;
use num_traits::Zero;

use crate::{
    angle::Angle,
    button::{Button, ButtonMountKind},
    buttons_column::ButtonsColumn,
};

pub struct ButtonRec {
    /// Padding of this button from previous button
    additional_padding: Dec,
    depth: Dec,
    incline: Angle,
    kind: ButtonMountKind,
}

pub struct ButtonsColumnBuilder {
    /// Curvature of button row - how round buttons are in plane of column
    curvature: Angle,

    /// Incline is an angle of the row how it is inclined from horizontal
    incline: Angle,

    /// Shift in mm on the curvature circle (when curvature is zero - then just mm)
    radial_shift: Dec,

    /// Padding between buttons
    padding: Dec,

    /// Total colum depth
    depth: Dec,

    /// Additional padding for this column, applied for all buttons
    addition_column_padding: Dec,

    /// Buttons, collected around center
    main_buttons: Vec<Button>,

    /// Buttons, collected around center
    top_buttons: Vec<Button>,

    /// Buttons, collected around center
    bottom_buttons: Vec<Button>,
}

impl Default for ButtonsColumnBuilder {
    fn default() -> Self {
        Self {
            curvature: Angle::zero(),
            incline: Angle::zero(),
            radial_shift: Dec::zero(),
            padding: Dec::zero(),
            addition_column_padding: Dec::zero(),
            depth: Dec::zero(),
            main_buttons: Vec::new(),
            top_buttons: Vec::new(),
            bottom_buttons: Vec::new(),
        }
    }
}

impl ButtonsColumnBuilder {
    pub fn incline(mut self, angle: Angle) -> Self {
        self.incline = angle;
        self
    }

    pub fn depth(mut self, depth: Dec) -> Self {
        self.depth = depth;
        self
    }

    pub fn curvature(mut self, curvature: Angle) -> Self {
        self.curvature = curvature;
        self
    }

    pub fn padding(mut self, padding: Dec) -> Self {
        self.padding = padding;
        self
    }

    pub fn addition_column_padding(mut self, addition_column_padding: Dec) -> Self {
        self.addition_column_padding = addition_column_padding;
        self
    }

    pub fn add_on_top(
        mut self,
        button: Button,
        //kind: ButtonMountKind,
        //additional_padding: Dec,
        //incline: Angle,
        //depth: Dec,
    ) -> Self {
        self.top_buttons.push(button);
        self
    }

    pub fn add_on_bottom(
        mut self,
        button: Button,
        //kind: ButtonMountKind,
        //additional_padding: Dec,
        //incline: Angle,
        //depth: Dec,
    ) -> Self {
        self.bottom_buttons.push(button);
        self
    }

    pub fn main_button(mut self, button: Button) -> Self {
        self.main_buttons.push(button);
        self
    }

    /*
    pub fn main_buttons(mut self, buttons: usize, kind: ButtonMountKind) -> Self {
        for _ in 0..buttons {
            self.main_buttons.push(ButtonRec {
                additional_padding: Dec::zero(),
                depth: Dec::zero(),
                incline: Angle::zero(),
                kind,
            })
        }
        self
    }
    fn calculate_circle(&self) -> Dec {
        todo!();
    }
    fn calculate_bottom_origin(&self) -> Origin {
        todo!();
    }

    pub fn button_roots(&self) -> Vec<Origin> {
        todo!()
    }
    */

    pub fn build(self) -> ButtonsColumn {
        ButtonsColumn {
            buttons: self.lower_buttons().chain(self.upper_buttons()).collect(),
        }
    }

    fn first_btn(&self) -> Option<(Origin, ButtonMountKind)> {
        let first_upper_btn = match self.main_buttons.len() {
            0 => {
                return None;
            }
            1 => 0,
            2 => 1,
            3 => 2,
            4 => 2,
            5 => 3,
            6 => 3,
            x => {
                if x % 2 == 0 {
                    x / 2
                } else {
                    (x + 1) / 2
                }
            }
        };
        let start_with = Origin::new().offset_z(self.depth);
        let z = start_with.z();
        let start_with = start_with
            .offset_x(self.addition_column_padding)
            .rotate_axisangle(z * -self.incline.rad());

        if self.main_buttons.len() % 2 == 0 {
            let two = Dec::from(2);
            let btn = &self.main_buttons[first_upper_btn];
            let kind = btn.kind;

            let x = start_with.x();
            let tot_move = self.padding + kind.button_height(); // + btn.additional_padding;
            Some((
                Origin::new()
                    .offset_y(tot_move / two)
                    .rotate_axisangle(x * self.curvature.rad() / two),
                kind,
            ))
        } else {
            Some((start_with, self.main_buttons[first_upper_btn].kind))
        }
    }

    /// Gives buttons from central button to up
    fn upper_buttons(&self) -> impl Iterator<Item = Button> {
        let mut button_recs = match self.main_buttons.len() {
            0 => Vec::new(),
            1 => self.main_buttons.iter().collect_vec(),
            2 => self.main_buttons.iter().skip(1).collect_vec(),
            3 => self.main_buttons.iter().skip(1).collect_vec(),
            4 => self.main_buttons.iter().skip(2).collect_vec(),
            5 => self.main_buttons.iter().skip(2).collect_vec(),
            6 => self.main_buttons.iter().skip(3).collect_vec(),
            x => {
                panic!("Lower main_buttons... too much for buttons in raw {x}");
            }
        };
        button_recs.extend(self.top_buttons.iter());

        let mut buttons = Vec::new();
        if let Some((mut o, mut prev_kind)) = self.first_btn() {
            let x = o.x();
            let two = Dec::from(2);
            for b in button_recs.iter_mut() {
                let tot_pad =
                    prev_kind.button_height() / two + b.kind.button_height() / two + self.padding;

                let mut new_b = b.clone();
                new_b.origin.apply(&o);
                buttons.push(new_b);
                /*
                let btn_o = o.clone().offset_y(b.additional_padding).offset_z(-b.depth);
                let btn_x = btn_o.x();
                buttons.push(Button::new(
                    btn_o.rotate_axisangle(btn_x * b.incline.rad()),
                    b.kind,
                ));
                */
                o = o
                    .offset_y(tot_pad / two)
                    .rotate_axisangle(x * (self.curvature.rad()))
                    .offset_y(tot_pad / two);
                prev_kind = b.kind;
            }
        }
        buttons.into_iter()
    }
    /// Gives buttons from central button to
    fn lower_buttons(&self) -> impl Iterator<Item = Button> {
        let mut button_recs = match self.main_buttons.len() {
            0 => Vec::new(),
            1 => Vec::new(),
            2 => self.main_buttons.iter().rev().skip(1).collect_vec(),
            3 => self.main_buttons.iter().rev().skip(2).collect_vec(),
            4 => self.main_buttons.iter().rev().skip(2).collect_vec(),
            5 => self.main_buttons.iter().rev().skip(3).collect_vec(),
            6 => self.main_buttons.iter().rev().skip(3).collect_vec(),
            x => {
                panic!("Lower main_buttons... too much for buttons in raw {x}");
            }
        };

        button_recs.extend(self.bottom_buttons.iter());

        let mut buttons = Vec::new();
        if let Some((mut o, mut prev_kind)) = self.first_btn() {
            let x = o.x();
            let two = Dec::from(2);
            for b in button_recs {
                let tot_pad =
                    prev_kind.button_height() / two + b.kind.button_height() / two + self.padding; //+ b.additional_padding;

                let new_o = o
                    .clone()
                    .offset_y(-tot_pad / two)
                    .rotate_axisangle(x * (-self.curvature.rad()))
                    .offset_y(-tot_pad / two);
                /*
                let btn_o = new_o
                    .clone()
                    .offset_y(b.additional_padding)
                    .offset_z(-b.depth);
                let btn_x = btn_o.x();
                buttons.push(Button::new(
                    btn_o.rotate_axisangle(btn_x * b.incline.rad()),
                    b.kind,
                ));
                */
                let mut new_b = b.clone();
                new_b.origin.apply(&new_o);

                buttons.push(new_b);
                o = new_o;
                prev_kind = b.kind;
            }
        }
        buttons.into_iter().rev()
    }
}
