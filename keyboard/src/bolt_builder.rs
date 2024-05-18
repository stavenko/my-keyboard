use nalgebra::Vector3;

#[derive(Clone, Debug)]
pub enum Nut {
    Hex { outer_diameter: f32 },
}

impl Nut {
    pub fn outer_diameter(&self) -> f32 {
        match self {
            Nut::Hex { outer_diameter } => *outer_diameter,
        }
    }
    pub fn hex_with_inner_diameter(inner_diameter: f32) -> Self {
        let outer_diameter = 4. / 3. * inner_diameter;
        dbg!(outer_diameter);
        Self::Hex { outer_diameter }
    }
}

#[derive(Clone, Debug)]
pub struct Bolt {
    pub nut: Option<Nut>,
    pub diameter: f32,
    pub height: f32,
    pub head_diameter: f32,
    pub head_height: f32,
}

pub struct BoltPlace {
    pub position: Vector3<f32>,
    pub nut_depth: f32,
    pub bolt: Bolt,
}

/*
impl BoltPlace {
    fn clear(&self) -> ScadObject {
        let diameter = self
            .bolt
            .nut
            .as_ref()
            .map(|d| match d {
                Nut::Hex { outer_diameter, .. } if *outer_diameter > self.bolt.head_diameter => {
                    *outer_diameter
                }
                _ => self.bolt.head_diameter,
            })
            .unwrap();

        let cylinder = ScadElement::Cylinder(500.0, scad::CircleType::Diameter(diameter));

        let mut translate: ScadObject = ScadObject::new(ScadElement::Translate(
            self.position - Vector3::new(0f32, 0f32, 10f32),
        ));
        translate.add_child(ScadObject::new(cylinder));
        translate
    }

    fn head_well_positive(&self) -> ScadObject {
        let d = self.bolt.head_diameter;

        let mut start = self.position;
        let h = self.bolt.height;
        let head_part = h * (1.0 - self.nut_depth);
        start.z -= head_part;
        let tol = 0.5;
        let mut nest = ScadObject::new(ScadElement::Translate(start));
        nest.add_child(ScadObject::new(ScadElement::Cylinder(
            head_part + 5.0,
            scad::CircleType::Diameter(d),
        )));
        nest
    }
    fn nut_well_positive(&self) -> ScadObject {
        let d = self.bolt.nut.as_ref().map(|n| n.outer_diameter());

        let mut start = self.position;
        let h = self.bolt.height;
        start.z -= h;
        let mut nest = ScadObject::new(ScadElement::Translate(start));
        if let Some(d) = d {
            let nut_part = h * self.nut_depth;
            nest.add_child(ScadObject::new(ScadElement::Cylinder(
                nut_part,
                scad::CircleType::Diameter(d),
            )));
        }
        nest
    }

    fn nut_well_negative(&self) -> ScadObject {
        let d = self.bolt.diameter;
        let h = self.bolt.height;
        let nut_part = h * self.nut_depth;
        let mut start = self.position;
        start.z -= h;

        let mut nest = ScadObject::new(ScadElement::Translate(start));
        nest.add_child(ScadObject::new(ScadElement::Cylinder(
            nut_part,
            scad::CircleType::Diameter(d),
        )));

        nest
    }
    fn head_well_negative(&self) -> ScadObject {
        let d = self.bolt.diameter;
        let h = self.bolt.height;
        let mut start = self.position;
        let head_part = h * (1.0 - self.nut_depth);
        let tol = 0.5;
        start.z -= head_part + tol;

        let mut nest = ScadObject::new(ScadElement::Translate(start));
        nest.add_child(ScadObject::new(ScadElement::Cylinder(
            head_part + 10.0,
            scad::CircleType::Diameter(d),
        )));

        nest
    }

    fn cut_head(&self) -> ScadObject {
        let start = self.position;
        let mut nest = ScadObject::new(ScadElement::Translate(start));
        nest.add_child(ScadObject::new(ScadElement::Cylinder(
            self.bolt.head_height,
            scad::CircleType::Diameter(self.bolt.head_diameter),
        )));

        nest
    }
    fn cut_nut(&self) -> ScadObject {
        let h = self.bolt.height;
        let mut start = self.position;
        start.z -= h + self.bolt.head_height;
        let mut nest = ScadObject::new(ScadElement::Translate(start));
        let nut_part = h * self.nut_depth;
        if let Some(nut) = self.nut(nut_part) {
            nest.add_child(nut);
        }

        nest
    }

    fn nut(&self, height: f32) -> Option<ScadObject> {
        let nut = self.bolt.nut.as_ref()?;
        match nut {
            Nut::Hex { outer_diameter } => {
                let mut points = Vec::new();
                for a in 0..6 {
                    let angle = a as f32 / 6.0 * (2.0 * PI);
                    dbg!(angle);

                    let point = outer_diameter / 2.0 * Vector2::new(angle.cos(), angle.sin());
                    points.push(point);
                }
                let mut nut = ScadObject::new(ScadElement::LinearExtrude(LinExtrudeParams {
                    height,
                    ..Default::default()
                }));
                nut.add_child(ScadObject::new(ScadElement::Polygon(
                    PolygonParameters::new(points),
                )));
                Some(nut)
            }
        }
    }
}

pub struct BoltBuilder {
    bolts: Vec<BoltPlace>,
}

impl BoltBuilder {
    pub fn new() -> Self {
        Self { bolts: Vec::new() }
    }
    pub fn add_bolt(mut self, bolt: BoltPlace) -> Self {
        self.bolts.push(bolt);
        self
    }

    pub fn with_head(&self, scad_object: ScadObject) -> ScadObject {
        let mut union = ScadObject::new(ScadElement::Union);
        union.add_child(scad_object);
        for b in &self.bolts {
            union.add_child(b.head_well_positive())
        }
        let mut diff = ScadObject::new(ScadElement::Difference);
        diff.add_child(union);
        for b in &self.bolts {
            diff.add_child(b.head_well_negative());
            //diff.add_child(b.cut_head());
        }
        diff
    }

    pub fn with_nut(&self, scad_object: ScadObject) -> ScadObject {
        let mut union = ScadObject::new(ScadElement::Union);
        union.add_child(scad_object);
        for b in &self.bolts {
            union.add_child(b.nut_well_positive())
        }
        let mut diff = ScadObject::new(ScadElement::Difference);
        diff.add_child(union);
        for b in &self.bolts {
            diff.add_child(b.nut_well_negative());
            diff.add_child(b.cut_nut());
        }
        diff
    }
    // pub fn with_head(&self, scad_object: ScadObject) -> ScadObject {}
}
*/
