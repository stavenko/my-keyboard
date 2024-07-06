use geometry::{decimal::Dec, origin::Origin};

use crate::{button_collections::ButtonsCollection, buttons_column::ButtonsColumn, Angle};

#[derive(Default)]
pub struct ButtonsCollectionBuilder {
    columns: Vec<ButtonsColumn>,
    padding: Dec,
    curvature: Angle,
    first_column_angle: Angle,
    plane_pitch: Angle,
    plane_yaw: Angle,
    height: Dec,
    position_shift_x: Dec,
    position_shift_y: Dec,
}

impl ButtonsCollectionBuilder {
    pub fn build(mut self) -> ButtonsCollection {
        let mut org = Origin::new();
        let x = org.x();
        let y = org.y();
        let z = org.z();
        org = org
            .offset_z(self.height)
            .offset_x(self.position_shift_x)
            .offset_y(self.position_shift_y)
            .rotate_axisangle(y * self.first_column_angle.rad())
            .rotate_axisangle(x * self.plane_pitch.rad())
            .rotate_axisangle(z * self.plane_yaw.rad());

        for c in self.columns.iter_mut() {
            c.apply_origin(&org);
            let two = Dec::from(2);
            org = org
                .offset_x(self.padding / two)
                .rotate_axisangle(y * -self.curvature.rad())
                .offset_x(self.padding / two);
        }
        ButtonsCollection {
            columns: self.columns,
        }
    }

    pub fn column(mut self, column: ButtonsColumn) -> Self {
        self.columns.push(column);
        self
    }

    pub fn padding(mut self, padding: impl Into<Dec>) -> Self {
        self.padding = padding.into();
        self
    }

    pub fn height(mut self, height: impl Into<Dec>) -> Self {
        self.height = height.into();
        self
    }

    pub fn curvature(mut self, angle: Angle) -> Self {
        self.curvature = angle;
        self
    }

    pub fn first_column_angle(mut self, angle: Angle) -> Self {
        self.first_column_angle = angle;
        self
    }

    pub fn position_shift_x(mut self, x: Dec) -> Self {
        self.position_shift_x = x;
        self
    }

    pub fn position_shift_y(mut self, y: Dec) -> Self {
        self.position_shift_y = y;
        self
    }

    pub fn plane_pitch(mut self, angle: Angle) -> Self {
        self.plane_pitch = angle;
        self
    }

    pub fn plane_yaw(mut self, angle: Angle) -> Self {
        self.plane_yaw = angle;
        self
    }
}
